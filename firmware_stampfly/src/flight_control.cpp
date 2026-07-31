// ===========================================================================
// flight_control.cpp — 状態機械 + カスケードPID + ミキサ 実装
//
// 制御則(角度→角速度→ミキサ、高度→上下速度→スラスト)と離着陸シーケンスは
// 飛行実績のある OptiTrack版を踏襲。コマンド入出力は PROTOCOL.md v1 に従い
// comm/telemetry モジュール経由で行う。定常状態でUSBシリアルへは出力しない。
// ===========================================================================
#include "flight_control.hpp"

#include <Arduino.h>

#include <cstring>

#include "comm.hpp"
#include "flow/flow_hub.hpp"
#include "indicators.hpp"
#include "pid.hpp"
#include "sensor.hpp"
#include "telemetry.hpp"
#include "yaw_estimation/angle_utils.hpp"
#include "yaw_estimation/persistence.hpp"
#include "yaw_estimation/sensor_hub_ff.hpp"

FlightControlState flight_control_state;

namespace {

using stampfly::Reason;
using stampfly::TlmAck;

// --- PID制御器・フィルタ(OptiTrack版と同構成) ---
PID p_pid;      // ロール角速度
PID q_pid;      // ピッチ角速度
PID r_pid;      // ヨー角速度
PID phi_pid;    // ロール角
PID theta_pid;  // ピッチ角
PID psi_pid;    // ヨー角(v2: 角度誤差 wrapPi → ヨーレート目標)
PID alt_pid;    // 高度 → 上下速度目標
PID z_dot_pid;  // 上下速度 → スラスト補正
Filter thrust_filtered;
Filter duty_filter_fr;
Filter duty_filter_fl;
Filter duty_filter_rr;
Filter duty_filter_rl;

// --- シーケンス内部状態 ---
uint16_t offset_counter = 0;       // ジャイロオフセット平均の進捗
uint16_t auto_takeoff_counter = 0; // 離陸スラストランプの進捗
uint8_t landing_state = 0;         // 着陸シーケンス初期化済みフラグ
uint8_t prev_range0_flag = 0;      // 高度センサ喪失の前回値
bool wait_ready = false;           // WAITでの静定+AHRSリセット完了

// --- v2: ヨー角制御の途絶ラッチ(契約 §1.1: 途絶>200msで現在推定ヨーを保持) ---
bool yaw_latch_active = false;     // ラッチ保持中か
float yaw_latched_ref = 0.0f;      // ラッチしたヨー角目標 [rad]

// ---------------------------------------------------------------------------
// v2.1: 機上XY位置制御(CMD_POS_ERR)
//
// PC 側 core/pid.py(飛行実績あり)と同一アルゴリズムの1軸PID:
// D項LPF / I項条件付き更新 / 異常時減衰 / 動的アンチワインドアップ。
// 更新は CMD_POS_ERR の新サンプル到着時のみ(≈50Hz、dt は受信間隔)。
// PC 側との違いは、誤差を機体ヨー推定で機体座標系へ回転(ヨー回転補償)
// してから PID に入れる点のみ(update_pos_control 参照)。
// ---------------------------------------------------------------------------

// 注: PC 側 pid.py との既知の(意図的な)相違点 — PC は途絶中 calculate を
// 呼ばず I 項が凍結、異常解除に confidence>0.5 のヒステリシスがあるのに対し、
// 本実装は bit2(データ有効)のみで判定し、無効中は I 項を微減衰させる。
// いずれも安全側(I 項が育たない方向)の簡略化。
struct PosAxisPid {
    float integral = 0.0f;
    float prev_error = 0.0f;
    bool has_prev = false;
    bool prev_valid = true;
    float filtered_derivative = 0.0f;
    float last_output = 0.0f;
    bool i_update_suspended = false;
    bool anomaly_detected = false;
    uint8_t anomaly_recovery_count = 0;

    void reset(void) {
        integral = 0.0f;
        prev_error = 0.0f;
        has_prev = false;
        prev_valid = true;
        filtered_derivative = 0.0f;
        last_output = 0.0f;
        i_update_suspended = false;
        anomaly_detected = false;
        anomaly_recovery_count = 0;
    }

    // D 項の履歴を再シードする(受信ギャップ明けの復帰1サンプル目に、
    // ギャップをまたいだ誤差差分で D 項が暴れるのを防ぐ。I 項は保持)。
    void reseed_derivative(void) {
        has_prev = false;
        filtered_derivative = 0.0f;
    }

    // 異常中は I 項更新を停止、回復後は pos_anomaly_recovery_ticks 回の減衰期間
    void set_anomaly(bool is_anomaly) {
        if (is_anomaly && !anomaly_detected) {
            anomaly_detected = true;
            i_update_suspended = true;
            anomaly_recovery_count = 0;
        } else if (!is_anomaly && anomaly_detected) {
            anomaly_detected = false;
            anomaly_recovery_count = FLIGHT_CONFIG.pos_anomaly_recovery_ticks;
        }
    }

    float update(float error, float dt, bool data_valid) {
        const FlightConfig& cfg = FLIGHT_CONFIG;
        if (!has_prev) {
            has_prev = true;
            prev_error = error;
            dt = cfg.pos_initial_dt_s;
        }
        if (dt < cfg.pos_min_dt_s) return last_output;
        // 無効区間明けの復帰1サンプル目も D を再シード(防御。PC 側は
        // 実誤差を送り続ける規約だが、誤差ストリームの不連続に頑健にする)
        if (data_valid && !prev_valid) {
            prev_error = error;
            filtered_derivative = 0.0f;
        }
        prev_valid = data_valid;

        const float p_term = cfg.pos_kp * error;

        if (anomaly_recovery_count > 0) {
            integral *= cfg.pos_i_decay_rate;
            anomaly_recovery_count--;
            if (anomaly_recovery_count == 0) i_update_suspended = false;
        }
        const bool should_update_i = !i_update_suspended && data_valid &&
                                     fabsf(error) < cfg.pos_i_update_threshold_m;
        if (should_update_i) {
            integral += error * dt;
            if (cfg.pos_ki != 0.0f) {
                // 動的アンチワインドアップ(誤差が大きいほど上限を絞る)
                const float error_factor =
                    expf(-fabsf(error) * cfg.pos_integral_error_factor_gain);
                const float max_integral =
                    fabsf(cfg.pos_output_limit_rad / cfg.pos_ki) * error_factor;
                if (integral > max_integral) integral = max_integral;
                if (integral < -max_integral) integral = -max_integral;
            }
        } else if (!data_valid) {
            integral *= cfg.pos_invalid_i_decay;
        }
        const float i_term = cfg.pos_ki * integral;

        // D 項の dt はバースト受信(ESP-NOW rx 時刻の密集)によるスパイクを
        // 防ぐため下限を設ける(誤差サンプルは PC 側 50Hz クロックで生成
        // されており、受信間隔の縮みは情報の増加ではない)
        const float dt_d = (dt < cfg.pos_dt_floor_s) ? cfg.pos_dt_floor_s : dt;
        const float raw_derivative = (error - prev_error) / dt_d;
        filtered_derivative = cfg.pos_d_filter_alpha * raw_derivative +
                              (1.0f - cfg.pos_d_filter_alpha) * filtered_derivative;
        float derivative = filtered_derivative;
        if (!data_valid) derivative *= cfg.pos_invalid_d_scale;
        const float d_term = cfg.pos_kd * derivative;

        float output = p_term + i_term + d_term;
        if (output > cfg.pos_output_limit_rad) output = cfg.pos_output_limit_rad;
        if (output < -cfg.pos_output_limit_rad) output = -cfg.pos_output_limit_rad;

        prev_error = error;
        last_output = output;
        return output;
    }
};

PosAxisPid pos_pid_x;   // 機体座標系 前後軸誤差 → roll 指令(既存の x誤差→roll 対応を踏襲)
PosAxisPid pos_pid_y;   // 機体座標系 左右軸誤差 → pitch 指令(同 y誤差→pitch)
float pos_target_roll_rad = 0.0f;   // XY PID 出力(スルー制限前の目標)
float pos_target_pitch_rad = 0.0f;
float pos_cmd_roll_rad = 0.0f;      // スルー制限後の適用値(400Hz で前進)
float pos_cmd_pitch_rad = 0.0f;
// XY回転補償ヨーの EKF→Madgwick 縮退用フレーム差ラッチ(EKF 不健全遷移の
// 瞬間に両推定のフレーム差を保持し、回転補償ヨーに段差を作らない)
bool pos_yaw_fallback_prev_ok = true;
float pos_yaw_fallback_offset = 0.0f;

float slew_toward(float current, float target, float max_delta) {
    const float delta = target - current;
    if (delta > max_delta) return current + max_delta;
    if (delta < -max_delta) return current - max_delta;
    return target;
}

void reset_pos_control(void) {
    pos_pid_x.reset();
    pos_pid_y.reset();
    pos_target_roll_rad = 0.0f;
    pos_target_pitch_rad = 0.0f;
    pos_cmd_roll_rad = 0.0f;
    pos_cmd_pitch_rad = 0.0f;
    pos_yaw_fallback_prev_ok = true;
    pos_yaw_fallback_offset = 0.0f;
    flight_control_state.command.pos_err_fresh_sample = false;
    flight_control_state.diag.xy_onboard_active = 0;
}

// --- v2: モーターテストサービス内部状態(yaw側 motor.cpp のソフトスタート/
//     フェイルセーフを移植。PWM の書き手は MOTOR_TEST 状態の本サービスのみ) ---
float mt_target_duty = 0.0f;    // 指令 duty(ソフトスタートランプの目標)
float mt_applied_duty = 0.0f;   // ランプ後の実効 duty(PWM に出ている値)
uint8_t mt_mask = 0;            // 駆動対象(bit0=FL,1=FR,2=RL,3=RR)
bool mt_running = false;        // 指令上の駆動中(フェイルセーフ監視対象)
uint32_t mt_last_cmd_ms = 0;    // 直近 CMD_MOTOR_RUN/STOP 受理時刻
uint32_t mt_last_ramp_ms = 0;   // ランプ dt の基準時刻

// 400Hzタイマ割り込み: フラグを立てるだけ(ISR内で他の処理はしない)
hw_timer_t* loop_timer = nullptr;
void IRAM_ATTR on_loop_timer() {
    flight_control_state.timing.Loop_flag = 1;
}

// ---------------------------------------------------------------------------
// モータPWM
// ---------------------------------------------------------------------------

void set_duty_fr(float duty) { ledcWrite(LEDC_CH_MOTOR_FRONT_RIGHT, (uint32_t)(MOTOR_PWM_MAX_COUNT * duty)); }
void set_duty_fl(float duty) { ledcWrite(LEDC_CH_MOTOR_FRONT_LEFT, (uint32_t)(MOTOR_PWM_MAX_COUNT * duty)); }
void set_duty_rr(float duty) { ledcWrite(LEDC_CH_MOTOR_REAR_RIGHT, (uint32_t)(MOTOR_PWM_MAX_COUNT * duty)); }
void set_duty_rl(float duty) { ledcWrite(LEDC_CH_MOTOR_REAR_LEFT, (uint32_t)(MOTOR_PWM_MAX_COUNT * duty)); }

void init_pwm(void) {
    ledcSetup(LEDC_CH_MOTOR_FRONT_LEFT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_MOTOR_FRONT_RIGHT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_MOTOR_REAR_LEFT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcSetup(LEDC_CH_MOTOR_REAR_RIGHT, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_RESOLUTION_BITS);
    ledcAttachPin(PIN_MOTOR_FRONT_LEFT, LEDC_CH_MOTOR_FRONT_LEFT);
    ledcAttachPin(PIN_MOTOR_FRONT_RIGHT, LEDC_CH_MOTOR_FRONT_RIGHT);
    ledcAttachPin(PIN_MOTOR_REAR_LEFT, LEDC_CH_MOTOR_REAR_LEFT);
    ledcAttachPin(PIN_MOTOR_REAR_RIGHT, LEDC_CH_MOTOR_REAR_RIGHT);
}

void motor_stop(void) {
    set_duty_fr(0.0f);
    set_duty_fl(0.0f);
    set_duty_rr(0.0f);
    set_duty_rl(0.0f);
}

// ---------------------------------------------------------------------------
// モーターテストサービス(v2 契約 §2.4)
//
// yaw側 motor.cpp のソフトスタート(2.0duty/s)・キープアライブ途絶停止(1.5s)を
// 移植。PWM 出力は既存の set_duty_*(LEDC)経路を使い、書き手は MOTOR_TEST
// 状態の本サービスのみ(飛行ミキサとは状態機械で排他)。ソフトスタートは
// 1S バッテリの突入電流ブラウンアウト対策なので削除しないこと。
// ---------------------------------------------------------------------------

// 実効 duty×マスクを PWM へ出力し、TLM_STATE の duty 表示にもミラーする
void motor_test_write_outputs(void) {
    auto& out = flight_control_state.output;
    const float d = mt_applied_duty;
    const float fl = (mt_mask & stampfly::CmdMotorRun::MASK_FL) ? d : 0.0f;
    const float fr = (mt_mask & stampfly::CmdMotorRun::MASK_FR) ? d : 0.0f;
    const float rl = (mt_mask & stampfly::CmdMotorRun::MASK_RL) ? d : 0.0f;
    const float rr = (mt_mask & stampfly::CmdMotorRun::MASK_RR) ? d : 0.0f;
    set_duty_fl(fl);
    set_duty_fr(fr);
    set_duty_rl(rl);
    set_duty_rr(rr);
    out.FrontLeft_motor_duty = fl;
    out.FrontRight_motor_duty = fr;
    out.RearLeft_motor_duty = rl;
    out.RearRight_motor_duty = rr;
}

// ソフトスタートランプを1step進める(millis 差分ベースでループ周波数非依存)
void motor_test_ramp(void) {
    const uint32_t now_ms = millis();
    const float dt = (now_ms - mt_last_ramp_ms) * 1.0e-3f;
    mt_last_ramp_ms = now_ms;
    if (mt_applied_duty != mt_target_duty) {
        const float step = FLIGHT_CONFIG.motor_test_slew_duty_per_s * dt;
        if (mt_applied_duty < mt_target_duty) {
            mt_applied_duty += step;
            if (mt_applied_duty > mt_target_duty) mt_applied_duty = mt_target_duty;
        } else {
            mt_applied_duty -= step;
            if (mt_applied_duty < mt_target_duty) mt_applied_duty = mt_target_duty;
        }
    }
    motor_test_write_outputs();
}

// 即時無条件停止(yaw側 motorTestStop 踏襲: 停止は負荷が抜ける方向なので
// ランプダウン不要)。MOTOR_TEST 進入時の初期化にも使う。
void motor_test_stop_now(void) {
    mt_target_duty = 0.0f;
    mt_applied_duty = 0.0f;
    mt_mask = 0;
    mt_running = false;
    mt_last_cmd_ms = millis();
    mt_last_ramp_ms = mt_last_cmd_ms;
    motor_test_write_outputs();  // 全ch 0 を強制
}

// CMD_MOTOR_RUN の適用(duty はクランプ、キープアライブタイマ更新)
void motor_test_set(float duty, uint8_t mask) {
    if (!isfinite(duty) || duty < 0.0f) duty = 0.0f;
    if (duty > FLIGHT_CONFIG.motor_test_max_duty) duty = FLIGHT_CONFIG.motor_test_max_duty;
    mask &= (stampfly::CmdMotorRun::MASK_FL | stampfly::CmdMotorRun::MASK_FR |
             stampfly::CmdMotorRun::MASK_RL | stampfly::CmdMotorRun::MASK_RR);
    mt_target_duty = duty;
    mt_mask = mask;
    mt_running = (duty > 0.0f) && (mask != 0);
    mt_last_cmd_ms = millis();
    motor_test_ramp();
}

// 毎tickサービス: キープアライブ途絶の自動停止+ランプ前進
void motor_test_service(void) {
    if (mt_running &&
        (millis() - mt_last_cmd_ms) > FLIGHT_CONFIG.motor_test_failsafe_ms) {
        motor_test_stop_now();
        return;
    }
    motor_test_ramp();
}

// ---------------------------------------------------------------------------
// ユーティリティ
// ---------------------------------------------------------------------------

// 電圧→ホバリング付近のトリムデューティ(OptiTrack版の実測一次近似)
float get_trim_duty(float voltage) {
    return FLIGHT_CONFIG.trim_duty_slope * voltage + FLIGHT_CONFIG.trim_duty_intercept;
}

// Thrust0(トリムFF)用の電圧1次LPF(config.hpp trim_voltage_filter_tc_s)。
// 負荷電流による電圧サグが Thrust0 経由で正帰還(dT0/dduty≈+0.18)を作り
// 0.45Hz 上下ボビングの減衰を削るため、トリム計算に使う電圧だけ秒スケールで
// 平滑する。低電圧フェイルセーフ判定(Under_voltage_flag)は生値のまま。
static float trim_voltage_state = 0.0f;
static bool trim_voltage_seeded = false;

// 毎tick(400Hz)呼ぶ。初回の有効電圧でシードし、以後1次LPFで追従する。
void update_trim_voltage(void) {
    const float v = sensor_state.Voltage;
    if (!(v > 0.5f)) return;                 // 起動直後の未計測値では更新しない
    if (!trim_voltage_seeded) {
        trim_voltage_state = v;
        trim_voltage_seeded = true;
        return;
    }
    float dt = flight_control_state.timing.Interval_time;
    if (dt <= 0.0f || dt > 0.01f) dt = FLIGHT_CONFIG.control_period_s;
    const float alpha = dt / (FLIGHT_CONFIG.trim_voltage_filter_tc_s + dt);
    trim_voltage_state += alpha * (v - trim_voltage_state);
}

// トリムFF計算用の平滑済み電圧(未シード時は生電圧にフォールバック)
float trim_voltage(void) {
    return trim_voltage_seeded ? trim_voltage_state : sensor_state.Voltage;
}

bool low_voltage_detected(void) {
    return sensor_state.Under_voltage_flag >= UNDER_VOLTAGE_COUNT;
}

// 状態遷移(同一状態への遷移は無視)。TLM_EVENT即時送信+LED/ブザー通知。
void transition_to(AutoFlightState next, Reason reason) {
    auto& mode = flight_control_state.mode;
    const AutoFlightState prev = mode.auto_state;
    if (prev == next) return;
    mode.auto_state = next;
    mode.last_reason = static_cast<uint8_t>(reason);
    mode.phase_start_ms = millis();
    telemetry_notify_transition(static_cast<uint8_t>(next), static_cast<uint8_t>(prev),
                                static_cast<uint8_t>(reason));
    indicators_notify_transition(static_cast<uint8_t>(next), static_cast<uint8_t>(prev),
                                 static_cast<uint8_t>(reason));
}

// WAITへ入る(目標高度を既定値へ戻し、高度推定器を飛行前状態に初期化)
void enter_wait(Reason reason) {
    flight_control_state.output.Alt_ref = FLIGHT_CONFIG.default_alt_ref_m;
    sensor_state.EstimatedAltitude.reset();
    transition_to(AUTO_WAIT, reason);
}

// START拒否(状態遷移なし)。理由つきTLM_EVENTで即時通知する。
void reject_start(Reason reason) {
    auto& mode = flight_control_state.mode;
    mode.last_reason = static_cast<uint8_t>(reason);
    telemetry_notify_transition(static_cast<uint8_t>(mode.auto_state),
                                static_cast<uint8_t>(mode.auto_state),
                                static_cast<uint8_t>(reason));
}

// 飛行中のリンク途絶経過 [ms]。離陸直後は flight_start_ms を起点に猶予を与える。
uint32_t setpoint_link_age_ms(uint32_t now_ms) {
    const auto& cs = flight_control_state.command;
    const auto& mode = flight_control_state.mode;
    uint32_t reference = mode.flight_start_ms;
    if (cs.setpoint_received) {
        // どちらか新しい方を起点にする(millisラップ安全な比較)
        if ((int32_t)(cs.last_setpoint_ms - reference) > 0) reference = cs.last_setpoint_ms;
    }
    return now_ms - reference;
}

// ---------------------------------------------------------------------------
// 制御初期化
// ---------------------------------------------------------------------------

void control_init(void) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    const float h = flight_control_state.timing.Control_period;

    // 角速度制御
    p_pid.set_parameter(cfg.roll_rate.kp, cfg.roll_rate.ti, cfg.roll_rate.td, cfg.roll_rate.eta, h);
    q_pid.set_parameter(cfg.pitch_rate.kp, cfg.pitch_rate.ti, cfg.pitch_rate.td, cfg.pitch_rate.eta, h);
    r_pid.set_parameter(cfg.yaw_rate.kp, cfg.yaw_rate.ti, cfg.yaw_rate.td, cfg.yaw_rate.eta, h);

    // 角度制御
    phi_pid.set_parameter(cfg.roll_angle.kp, cfg.roll_angle.ti, cfg.roll_angle.td, cfg.roll_angle.eta, h);
    theta_pid.set_parameter(cfg.pitch_angle.kp, cfg.pitch_angle.ti, cfg.pitch_angle.td, cfg.pitch_angle.eta, h);
    psi_pid.set_parameter(cfg.yaw_angle.kp, cfg.yaw_angle.ti, cfg.yaw_angle.td, cfg.yaw_angle.eta, h);

    // 高度制御
    alt_pid.set_parameter(cfg.altitude.kp, cfg.altitude.ti, cfg.altitude.td, cfg.altitude.eta,
                          cfg.altitude_pid_period_s);
    z_dot_pid.set_parameter(cfg.z_velocity.kp, cfg.z_velocity.ti, cfg.z_velocity.td, cfg.z_velocity.eta,
                            cfg.altitude_pid_period_s);

    // 出力フィルタ
    duty_filter_fr.set_parameter(cfg.duty_filter_tc_s, h);
    duty_filter_fl.set_parameter(cfg.duty_filter_tc_s, h);
    duty_filter_rr.set_parameter(cfg.duty_filter_tc_s, h);
    duty_filter_rl.set_parameter(cfg.duty_filter_tc_s, h);
    thrust_filtered.set_parameter(cfg.thrust_filter_tc_s, h);
}

void reset_duty_filters(void) {
    duty_filter_fr.reset();
    duty_filter_fl.reset();
    duty_filter_rr.reset();
    duty_filter_rl.reset();
}

// ヨー角制御のリセット(psi_pid・途絶ラッチ・適用中目標)。
// 既存の角度制御リセット群と同じ箇所(reset_rate/angle_control)から呼ぶ。
void reset_yaw_angle_control(void) {
    auto& out = flight_control_state.output;
    psi_pid.reset();
    yaw_latch_active = false;
    out.Yaw_angle_command = 0.0f;
    out.Yaw_ctrl_active = 0;
}

// ---------------------------------------------------------------------------
// コマンド適用
// ---------------------------------------------------------------------------

// 受信した CMD_SETPOINT を適用する(ハートビート時刻・seq_echo の更新を含む)。
// alt_ref は flags bit0 が立っている場合のみ、WAIT/TAKEOFF/HOVER でクランプ更新。
void apply_setpoint(const CommandSnapshot& cmd) {
    if (!cmd.setpoint_pending) return;
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& cs = flight_control_state.command;
    auto& out = flight_control_state.output;
    const AutoFlightState st = flight_control_state.mode.auto_state;

    cs.setpoint_received = true;
    cs.last_setpoint_ms = cmd.setpoint_rx_ms;
    cs.applied_setpoint_seq = cmd.setpoint_seq;
    cs.xy_source = XY_SOURCE_ATTITUDE;
    // 非有限値は入口で無害化する(NaN はクランプ比較をすり抜けて
    // 姿勢/高度 PID を汚染するため)
    cs.target_roll_rad = isfinite(cmd.setpoint.roll_ref) ? cmd.setpoint.roll_ref : 0.0f;
    cs.target_pitch_rad = isfinite(cmd.setpoint.pitch_ref) ? cmd.setpoint.pitch_ref : 0.0f;
    // v2: ヨー角目標(flags bit1 有効時のみ制御に使う。無効時は V1 同一動作)
    cs.target_yaw_rad = cmd.setpoint.yaw_ref;
    cs.target_yaw_valid = (cmd.setpoint.flags & stampfly::CmdSetpoint::FLAG_YAW_REF_VALID) != 0 &&
                          isfinite(cmd.setpoint.yaw_ref);

    if ((cmd.setpoint.flags & stampfly::CmdSetpoint::FLAG_ALT_REF_VALID) != 0 &&
        isfinite(cmd.setpoint.alt_ref) &&
        (st == AUTO_WAIT || st == AUTO_TAKEOFF || st == AUTO_HOVER)) {
        float alt = cmd.setpoint.alt_ref;
        if (alt < cfg.alt_ref_min_m) alt = cfg.alt_ref_min_m;
        if (alt > cfg.alt_ref_max_m) alt = cfg.alt_ref_max_m;
        out.Alt_ref = alt;
    }
}

// 受信した CMD_POS_ERR を適用する(v2.1 機上XY制御)。ハートビート
// (last_setpoint_ms / seq_echo)は CMD_SETPOINT と共有し、既存フェイル
// セーフ(>200ms 水平保持 / >500ms 自動着陸)がそのまま効く。
void apply_pos_err(const CommandSnapshot& cmd) {
    if (!cmd.pos_err_pending) return;
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& cs = flight_control_state.command;
    auto& out = flight_control_state.output;
    const AutoFlightState st = flight_control_state.mode.auto_state;
    const stampfly::CmdPosErr& pe = cmd.pos_err;

    cs.setpoint_received = true;
    cs.last_setpoint_ms = cmd.pos_err_rx_ms;
    cs.applied_setpoint_seq = cmd.pos_err_seq;
    cs.xy_source = XY_SOURCE_POS_ERR;

    cs.prev_pos_err_ms = (cs.last_pos_err_ms != 0) ? cs.last_pos_err_ms
                                                   : cmd.pos_err_rx_ms;
    cs.last_pos_err_ms = cmd.pos_err_rx_ms;
    cs.pos_err_x_m = pe.err_x;
    cs.pos_err_y_m = pe.err_y;
    cs.pos_err_xy_valid =
        (pe.flags & stampfly::CmdPosErr::FLAG_XY_ERR_VALID) != 0;
    cs.pos_mocap_yaw_rad = pe.mocap_yaw;
    // 非有限値は入口で無害化する(NaN はクランプ比較をすり抜けるため)
    cs.pos_mocap_yaw_valid =
        (pe.flags & stampfly::CmdPosErr::FLAG_MOCAP_YAW_VALID) != 0 &&
        isfinite(pe.mocap_yaw);
    // bit4 = 基準ヨー低信頼(移動ベースヨー時に PC が立てる。契約 §1.2)
    cs.pos_mocap_yaw_low_trust =
        (pe.flags & stampfly::CmdPosErr::FLAG_YAW_REF_LOW_TRUST) != 0;
    cs.pos_err_fresh_sample = true;
    // 外部ヨー基準を EKF2 のヨー擬似観測へ配線(consume-once ステージ。
    // 次 tick の sensorHubFfUpdate が age<100ms 検査の上で融合する。契約 §2.3)
    if (cs.pos_mocap_yaw_valid) {
        sensorHubFfStageYawObs(pe.mocap_yaw, cs.pos_mocap_yaw_low_trust,
                               cmd.pos_err_rx_ms);
    }

    // ヨー・高度目標は CMD_SETPOINT と同じ規約
    cs.target_yaw_rad = pe.yaw_ref;
    cs.target_yaw_valid = (pe.flags & stampfly::CmdPosErr::FLAG_YAW_REF_VALID) != 0 &&
                          isfinite(pe.yaw_ref);

    if ((pe.flags & stampfly::CmdPosErr::FLAG_ALT_REF_VALID) != 0 &&
        isfinite(pe.alt_ref) &&
        (st == AUTO_WAIT || st == AUTO_TAKEOFF || st == AUTO_HOVER)) {
        float alt = pe.alt_ref;
        if (alt < cfg.alt_ref_min_m) alt = cfg.alt_ref_min_m;
        if (alt > cfg.alt_ref_max_m) alt = cfg.alt_ref_max_m;
        out.Alt_ref = alt;
    }
}

// 機上XY位置制御の1周期: 新サンプル到着時のみ PID を1ステップ回し(≈50Hz)、
// 出力へは PC 側 SetpointShaper と同値のスルーレート制限を 400Hz で適用する。
// yaw_rad はヨー角制御と同じソース(EKF 健全時は EKF ψ、それ以外 Madgwick)。
void update_pos_control(float yaw_rad) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& cs = flight_control_state.command;

    if (cs.pos_err_fresh_sample) {
        cs.pos_err_fresh_sample = false;
        float dt = (cs.last_pos_err_ms - cs.prev_pos_err_ms) * 1.0e-3f;
        if (dt <= 0.0f || dt > cfg.pos_max_dt_s) {
            // 途絶明け(または初回): dt を仮値に置き換えるだけでは
            // ギャップをまたいだ誤差差分で D 項が暴れる(gap/dt 倍)ため、
            // D の履歴も再シードする(pos_max_dt_s の本来の意図)
            dt = cfg.pos_initial_dt_s;
            pos_pid_x.reseed_derivative();
            pos_pid_y.reseed_derivative();
        }
        const bool valid = cs.pos_err_xy_valid;

        float ex = cs.pos_err_x_m;
        float ey = cs.pos_err_y_m;
        if (!isfinite(ex) || !isfinite(ey)) { ex = 0.0f; ey = 0.0f; }
        if (ex > cfg.pos_err_clamp_m) ex = cfg.pos_err_clamp_m;
        if (ex < -cfg.pos_err_clamp_m) ex = -cfg.pos_err_clamp_m;
        if (ey > cfg.pos_err_clamp_m) ey = cfg.pos_err_clamp_m;
        if (ey < -cfg.pos_err_clamp_m) ey = -cfg.pos_err_clamp_m;

        // ヨー回転補償: 制御座標系(ワールド)誤差 → 機体座標系。
        // ψ=0 のとき既存の「x誤差→roll / y誤差→pitch」対応に一致する。
        // 符号は pos_ctrl_yaw_sign(config.hpp)で調整可能(要地上検証)。
        const float psi = cfg.pos_ctrl_yaw_sign * yaw_rad;
        const float c = cosf(psi);
        const float s = sinf(psi);
        const float e_roll = c * ex + s * ey;
        const float e_pitch = -s * ex + c * ey;

        pos_pid_x.set_anomaly(!valid);
        pos_pid_y.set_anomaly(!valid);
        float roll_t = pos_pid_x.update(e_roll, dt, valid);
        float pitch_t = pos_pid_y.update(e_pitch, dt, valid);
        if (!valid) {
            // PC 側と同じ規約: 無効データ中の指令は水平(PID 内部状態は減衰継続)
            roll_t = 0.0f;
            pitch_t = 0.0f;
        }
        pos_target_roll_rad = roll_t;
        pos_target_pitch_rad = pitch_t;
    }

    // スルーレート制限(PC SetpointShaper 相当)を毎tick前進させる
    float dt_tick = flight_control_state.timing.Interval_time;
    if (dt_tick < 0.0f) dt_tick = 0.0f;
    if (dt_tick > cfg.pos_max_dt_s) dt_tick = cfg.pos_max_dt_s;
    const float max_delta = cfg.pos_slew_rad_per_s * dt_tick;
    pos_cmd_roll_rad = slew_toward(pos_cmd_roll_rad, pos_target_roll_rad, max_delta);
    pos_cmd_pitch_rad = slew_toward(pos_cmd_pitch_rad, pos_target_pitch_rad, max_delta);
}

// 飛行中(TAKEOFF/HOVER)のスラストベース+姿勢指令を更新する(OptiTrack版 get_command 相当)
void update_thrust_and_attitude_command(void) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& out = flight_control_state.output;
    auto& mode = flight_control_state.mode;
    const auto& cs = flight_control_state.command;

    // 高度自動制御ON
    mode.Alt_flag = 1;

    // 離陸スラストランプ(実績シーケンス)
    if (auto_takeoff_counter < cfg.takeoff_ramp_phase1_ticks) {
        out.Thrust0 = (float)auto_takeoff_counter / cfg.takeoff_ramp_divisor;
        const float cap = get_trim_duty(cfg.takeoff_ramp_phase1_v);
        if (out.Thrust0 > cap) out.Thrust0 = cap;
        auto_takeoff_counter++;
    } else if (auto_takeoff_counter < cfg.takeoff_ramp_total_ticks) {
        out.Thrust0 = (float)auto_takeoff_counter / cfg.takeoff_ramp_divisor;
        const float cap = get_trim_duty(trim_voltage());
        if (out.Thrust0 > cap) out.Thrust0 = cap;
        auto_takeoff_counter++;
    } else {
        out.Thrust0 = get_trim_duty(trim_voltage());
    }

    // 高度センサ喪失時の安全降下(流用ロジック)
    if ((sensor_state.Range0flag > prev_range0_flag) || (sensor_state.Range0flag == RANGE0_FLAG_MAX)) {
        out.Thrust0 = out.Thrust0 - cfg.range_loss_thrust_step;
        prev_range0_flag = sensor_state.Range0flag;
    }
    out.Thrust_command = out.Thrust0 * cfg.battery_nominal_v;

    // ヨー推定ソースの選択(ヨー角制御と機上XY回転補償が共用)。
    // yaw_used: est_mode==1 かつ EKF 健全なら EKF ψ、est_mode==2 かつ EKF2
    // 健全(healthy2)なら EKF2 ψ、est_mode==0 なら Madgwick。
    // est_mode==1/2 で推定器が不健全(磁気更新凍結/アンカー無効/FF無効)な間は、
    // 飛行中のヨーソース切替による指令段差を作らないため角度制御を止めて
    // レートダンピングに縮退する(PC へは ffg / ff_status bit5 で通知される)。
    // EKF2 の healthy2 に yaw_obs 鮮度は含まれない(MoCap 途絶はコースト。契約 §2.2)。
    bool yaw_source_ok = true;
    float yaw_used = sensor_state.Yaw_angle;
    if (sensor_state.Est_mode == 1) {
        if (sensorHubFfEkfHealthy()) {
            yaw_used = sensor_state.Yaw_ekf_rad;
        } else {
            yaw_source_ok = false;
        }
    } else if (sensor_state.Est_mode == 2) {
        if (sensorHubFfEkf2Healthy()) {
            yaw_used = sensor_state.Ekf2_yaw_rad;
        } else {
            yaw_source_ok = false;
        }
    }

    // 姿勢指令: setpoint/pos_err が新鮮なら適用、200ms超は水平保持
    // (PROTOCOL.md フェイルセーフ)。機体差バイアスはPC側プロファイルで
    // 加算済み(ファームには置かない)。
    const uint32_t now_ms = millis();
    const bool link_fresh =
        cs.setpoint_received && (now_ms - cs.last_setpoint_ms) < cfg.link_level_hold_ms;
    if (link_fresh && cs.xy_source == XY_SOURCE_POS_ERR) {
        // v2.1 機上XY位置制御: 位置誤差をヨー回転補償して XY PID。
        // 回転に使うヨーは健全時 yaw_used(EKF/Madgwick)。EKF 不健全へ
        // 遷移した瞬間は EKF-Madgwick のフレーム差をラッチし、不健全中は
        // Madgwick+オフセットで連続なヨーを供給する(生の Madgwick へ
        // 切り替えると誤差フレームに段差ができるため。ヨー角制御の
        // レートダンピング縮退とは独立)。
        float yaw_for_pos = yaw_used;
        if (!yaw_source_ok) {
            if (pos_yaw_fallback_prev_ok) {
                // フレーム差は直前まで使っていた推定器(est_mode に対応)で取る
                const float est_yaw = sensor_state.Est_mode == 2
                    ? sensor_state.Ekf2_yaw_rad
                    : sensor_state.Yaw_ekf_rad;
                pos_yaw_fallback_offset =
                    wrapPi(est_yaw - sensor_state.Yaw_angle);
            }
            yaw_for_pos = wrapPi(sensor_state.Yaw_angle
                                 + pos_yaw_fallback_offset);
        }
        pos_yaw_fallback_prev_ok = yaw_source_ok;
        update_pos_control(yaw_for_pos);
        out.Roll_angle_command = pos_cmd_roll_rad;
        out.Pitch_angle_command = pos_cmd_pitch_rad;
        flight_control_state.diag.xy_onboard_active = 1;  // TLM_CTRL flags bit0
    } else if (link_fresh) {
        out.Roll_angle_command = cs.target_roll_rad;
        out.Pitch_angle_command = cs.target_pitch_rad;
        flight_control_state.diag.xy_onboard_active = 0;
    } else {
        out.Roll_angle_command = 0.0f;
        out.Pitch_angle_command = 0.0f;
        flight_control_state.diag.xy_onboard_active = 0;
        // 途絶中は XY 制御の出力状態も水平へ戻す(復帰時の段差防止)
        pos_target_roll_rad = 0.0f;
        pos_target_pitch_rad = 0.0f;
        pos_cmd_roll_rad = 0.0f;
        pos_cmd_pitch_rad = 0.0f;
        pos_pid_x.set_anomaly(true);
        pos_pid_y.set_anomaly(true);
    }

    // --- ヨー角制御(v2 契約 §2.3。flags bit1 無効時は V1 と同一のレートダンピング) ---

    bool yaw_cmd_valid = false;
    float yaw_cmd = 0.0f;
    if (link_fresh && cs.target_yaw_valid) {
        yaw_cmd = wrapPi(cs.target_yaw_rad);
        yaw_latch_active = false;
        yaw_cmd_valid = true;
    } else if (!link_fresh && cs.setpoint_received && cs.target_yaw_valid) {
        // 途絶(>200ms): 途絶検出時点の推定ヨーをラッチして保持(契約 §1.1)。
        // 0 指令へ落とすと「離陸方位への回頭」を意味するため角度保持が安全。
        // >500ms は既存フェイルセーフで LANDING(auto_landing_step はヨー0)。
        if (!yaw_latch_active) {
            yaw_latch_active = true;
            // ±π を保証してからラッチする(値はそのまま TLM の yaw_ref_rad に
            // 載るため。契約: yaw_ref は ±π)
            yaw_latched_ref = wrapPi(yaw_used);
        }
        yaw_cmd = yaw_latched_ref;
        yaw_cmd_valid = true;
    } else {
        yaw_latch_active = false;
    }

    if (yaw_cmd_valid && yaw_source_ok) {
        // 角度誤差は必ず ±π ラップしてから PID へ(最短経路で回頭する)
        const float dt = flight_control_state.timing.Interval_time;
        float yaw_rate_ref = psi_pid.update(wrapPi(yaw_cmd - yaw_used), dt);
        if (yaw_rate_ref > cfg.yaw_rate_limit_rad_s) yaw_rate_ref = cfg.yaw_rate_limit_rad_s;
        if (yaw_rate_ref < -cfg.yaw_rate_limit_rad_s) yaw_rate_ref = -cfg.yaw_rate_limit_rad_s;
        out.Yaw_rate_reference = yaw_rate_ref;
        out.Yaw_angle_command = yaw_cmd;
        out.Yaw_ctrl_active = 1;
    } else {
        // V1 同一動作: レートダンピングのみ(角度制御 off)
        out.Yaw_rate_reference = 0.0f;
        out.Yaw_angle_command = 0.0f;
        out.Yaw_ctrl_active = 0;
        psi_pid.reset();  // 不使用中に積分を持ち越さない(再開時の段差防止)
    }
}

// ---------------------------------------------------------------------------
// 制御則(OptiTrack版を踏襲)
// ---------------------------------------------------------------------------

void reset_rate_control(void) {
    auto& out = flight_control_state.output;
    motor_stop();
    out.FrontRight_motor_duty = 0.0f;
    out.FrontLeft_motor_duty = 0.0f;
    out.RearRight_motor_duty = 0.0f;
    out.RearLeft_motor_duty = 0.0f;
    reset_duty_filters();
    p_pid.reset();
    q_pid.reset();
    r_pid.reset();
    alt_pid.reset();
    z_dot_pid.reset();
    out.Roll_rate_reference = 0.0f;
    out.Pitch_rate_reference = 0.0f;
    out.Yaw_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    reset_yaw_angle_control();
    phi_pid.set_error(out.Roll_angle_reference);
    theta_pid.set_error(out.Pitch_angle_reference);
    out.Roll_angle_offset = 0.0f;
    out.Pitch_angle_offset = 0.0f;
}

void reset_angle_control(void) {
    auto& out = flight_control_state.output;
    out.Roll_rate_reference = 0.0f;
    out.Pitch_rate_reference = 0.0f;
    phi_pid.reset();
    theta_pid.reset();
    reset_yaw_angle_control();
    phi_pid.set_error(out.Roll_angle_reference);
    theta_pid.set_error(out.Pitch_angle_reference);
    out.Roll_angle_offset = 0.0f;
    out.Pitch_angle_offset = 0.0f;
}

// 角度制御(外側ループ): 高度PID + 姿勢角PID → 角速度目標
void angle_control(void) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& out = flight_control_state.output;
    const float dt = flight_control_state.timing.Interval_time;

    if (out.Thrust_command / cfg.battery_nominal_v < cfg.motor_on_duty_threshold) {
        reset_angle_control();
        return;
    }

    // 高度PID → 上下速度目標
    if (flight_control_state.mode.Alt_flag >= 1) {
        const float alt_err = out.Alt_ref - sensor_state.Altitude2;
        out.Z_dot_ref = alt_pid.update(alt_err, dt);
    }

    // 姿勢角目標(ファーム層クランプ ±30°)
    out.Roll_angle_reference = out.Roll_angle_command;
    out.Pitch_angle_reference = out.Pitch_angle_command;
    if (out.Roll_angle_reference > cfg.max_angle_ref_rad) out.Roll_angle_reference = cfg.max_angle_ref_rad;
    if (out.Roll_angle_reference < -cfg.max_angle_ref_rad) out.Roll_angle_reference = -cfg.max_angle_ref_rad;
    if (out.Pitch_angle_reference > cfg.max_angle_ref_rad) out.Pitch_angle_reference = cfg.max_angle_ref_rad;
    if (out.Pitch_angle_reference < -cfg.max_angle_ref_rad) out.Pitch_angle_reference = -cfg.max_angle_ref_rad;

    // 姿勢角PID → 角速度目標
    const float phi_err = out.Roll_angle_reference - (sensor_state.Roll_angle - out.Roll_angle_offset);
    const float theta_err = out.Pitch_angle_reference - (sensor_state.Pitch_angle - out.Pitch_angle_offset);
    out.Roll_rate_reference = phi_pid.update(phi_err, dt);
    out.Pitch_rate_reference = theta_pid.update(theta_err, dt);
}

// 角速度制御(内側ループ)+高度速度制御+ミキサ
void rate_control(void) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& out = flight_control_state.output;
    const float dt = flight_control_state.timing.Interval_time;

    if (out.Thrust_command / cfg.battery_nominal_v < cfg.motor_on_duty_threshold) {
        reset_rate_control();
        return;
    }

    // 角速度PID
    const float p_err = out.Roll_rate_reference - sensor_state.Roll_rate;
    const float q_err = out.Pitch_rate_reference - sensor_state.Pitch_rate;
    const float r_err = out.Yaw_rate_reference - sensor_state.Yaw_rate;
    out.Roll_rate_command = p_pid.update(p_err, dt);
    out.Pitch_rate_command = q_pid.update(q_err, dt);
    out.Yaw_rate_command = r_pid.update(r_err, dt);

    // 上下速度制御 → スラスト指令
    if (flight_control_state.mode.Alt_flag == 1) {
        const float z_dot_err = out.Z_dot_ref - sensor_state.Alt_velocity;
        out.Thrust_command = thrust_filtered.update(
            (out.Thrust0 + z_dot_pid.update(z_dot_err, dt)) * cfg.battery_nominal_v, dt);
        if (out.Thrust_command / cfg.battery_nominal_v > out.Thrust0 * cfg.thrust_cmd_max_ratio) {
            out.Thrust_command = cfg.battery_nominal_v * out.Thrust0 * cfg.thrust_cmd_max_ratio;
        }
        if (out.Thrust_command / cfg.battery_nominal_v < out.Thrust0 * cfg.thrust_cmd_min_ratio) {
            out.Thrust_command = cfg.battery_nominal_v * out.Thrust0 * cfg.thrust_cmd_min_ratio;
        }
    } else if (flight_control_state.mode.auto_state == AUTO_LANDING) {
        // 着陸時は固定降下速度指令(クランプなし: 実績挙動)
        const float z_dot_err = cfg.landing_z_dot_ref - sensor_state.Alt_velocity;
        out.Thrust_command = thrust_filtered.update(
            (out.Thrust0 + z_dot_pid.update(z_dot_err, dt)) * cfg.battery_nominal_v, dt);
    }

    // ミキサ(X配置: 各軸トルクをデューティへ配分)
    out.FrontRight_motor_duty = duty_filter_fr.update(
        (out.Thrust_command + (-out.Roll_rate_command + out.Pitch_rate_command + out.Yaw_rate_command) *
                                  cfg.mixer_torque_gain) / cfg.battery_nominal_v, dt);
    out.FrontLeft_motor_duty = duty_filter_fl.update(
        (out.Thrust_command + (out.Roll_rate_command + out.Pitch_rate_command - out.Yaw_rate_command) *
                                  cfg.mixer_torque_gain) / cfg.battery_nominal_v, dt);
    out.RearRight_motor_duty = duty_filter_rr.update(
        (out.Thrust_command + (-out.Roll_rate_command - out.Pitch_rate_command - out.Yaw_rate_command) *
                                  cfg.mixer_torque_gain) / cfg.battery_nominal_v, dt);
    out.RearLeft_motor_duty = duty_filter_rl.update(
        (out.Thrust_command + (out.Roll_rate_command - out.Pitch_rate_command + out.Yaw_rate_command) *
                                  cfg.mixer_torque_gain) / cfg.battery_nominal_v, dt);

    // デューティクランプ
    if (out.FrontRight_motor_duty < cfg.duty_min) out.FrontRight_motor_duty = cfg.duty_min;
    if (out.FrontRight_motor_duty > cfg.duty_max) out.FrontRight_motor_duty = cfg.duty_max;
    if (out.FrontLeft_motor_duty < cfg.duty_min) out.FrontLeft_motor_duty = cfg.duty_min;
    if (out.FrontLeft_motor_duty > cfg.duty_max) out.FrontLeft_motor_duty = cfg.duty_max;
    if (out.RearRight_motor_duty < cfg.duty_min) out.RearRight_motor_duty = cfg.duty_min;
    if (out.RearRight_motor_duty > cfg.duty_max) out.RearRight_motor_duty = cfg.duty_max;
    if (out.RearLeft_motor_duty < cfg.duty_min) out.RearLeft_motor_duty = cfg.duty_min;
    if (out.RearLeft_motor_duty > cfg.duty_max) out.RearLeft_motor_duty = cfg.duty_max;

    // 出力(OverG時は即時モータ停止 → COMPLETE)
    if (sensor_state.OverG_flag == 0) {
        set_duty_fr(out.FrontRight_motor_duty);
        set_duty_fl(out.FrontLeft_motor_duty);
        set_duty_rr(out.RearRight_motor_duty);
        set_duty_rl(out.RearLeft_motor_duty);
    } else {
        out.FrontRight_motor_duty = 0.0f;
        out.FrontLeft_motor_duty = 0.0f;
        out.RearRight_motor_duty = 0.0f;
        out.RearLeft_motor_duty = 0.0f;
        motor_stop();
        transition_to(AUTO_COMPLETE, Reason::OVER_G);
    }
}

// 着陸シーケンス1tick分。着陸完了で1を返す。
// (OptiTrack版 auto_landing と等価: 高度履歴配列は実質「1tick前の高度」比較
//  だったため、等価な単一変数 prev_alt に整理)
uint8_t auto_landing_step(void) {
    static float prev_alt = 0.0f;
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& out = flight_control_state.output;

    flight_control_state.mode.Alt_flag = 0;
    if (landing_state == 0) {
        landing_state = 1;
        prev_alt = sensor_state.Altitude2;
        out.Thrust0 = get_trim_duty(trim_voltage());
    }

    // 降下中はスラストを徐々に絞る
    if (prev_alt >= sensor_state.Altitude2) {
        out.Thrust0 = out.Thrust0 * cfg.landing_decay_slow;
    }
    if (sensor_state.Altitude2 < cfg.landing_near_ground_m) {
        out.Thrust0 = out.Thrust0 * cfg.landing_decay_fast;
    }

    uint8_t landed = 0;
    if (sensor_state.Altitude2 < cfg.landed_threshold_m) {
        landed = 1;
        landing_state = 0;
    }
    prev_alt = sensor_state.Altitude2;

    // 姿勢は水平保持。ヨーは着陸中つねにレートダンピング(=0)を維持し、
    // 角度制御の適用中フラグ/ラッチも落とす(契約 §2.3: auto_landing_step は 0 のまま)
    out.Roll_angle_command = 0.0f;
    out.Pitch_angle_command = 0.0f;
    out.Yaw_rate_reference = 0.0f;
    reset_yaw_angle_control();
    flight_control_state.diag.xy_onboard_active = 0;  // 着陸中は機上XY指令を生成しない
    return landed;
}

// ---------------------------------------------------------------------------
// 状態ハンドラ
// ---------------------------------------------------------------------------

void handle_init(void) {
    auto& out = flight_control_state.output;
    motor_stop();
    out.Roll_angle_offset = 0.0f;
    out.Pitch_angle_offset = 0.0f;
    sensor_reset_offset();
    offset_counter = 0;
    transition_to(AUTO_CALIBRATION, Reason::NONE);
}

void handle_calibration(void) {
    motor_stop();
    if (offset_counter < FLIGHT_CONFIG.gyro_calib_samples) {
        sensor_calc_offset_avarage();
        offset_counter++;
        return;
    }
    // オフセット確定 → 経過時間計測の起点を設定してWAITへ
    flight_control_state.timing.S_time = micros();
    wait_ready = false;
    enter_wait(Reason::NONE);
}

void handle_wait(const CommandSnapshot& cmd) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& mode = flight_control_state.mode;
    auto& out = flight_control_state.output;

    motor_stop();

    // 静定待ち完了 → AHRSリセット(離陸準備完了)
    if (!wait_ready && (millis() - mode.phase_start_ms) > cfg.wait_settle_ms) {
        for (uint8_t i = 0; i < cfg.ahrs_reset_repeat; i++) {
            ahrs_reset();
        }
        wait_ready = true;
        comm_send_log("calibration complete: ready for START");
    }

    // コマンド処理(優先度 STOP > START。STOPはWAITでは何もしない)
    if (!cmd.stop_pending && cmd.start_pending) {
        if (!wait_ready) {
            reject_start(Reason::START_REJECTED_NOT_READY);
        } else if (low_voltage_detected()) {
            reject_start(Reason::START_REJECTED_LOW_VOLTAGE);
        } else {
            // 離陸開始
            ahrs_reset();
            mode.flight_start_ms = millis();
            auto_takeoff_counter = 0;
            landing_state = 0;
            prev_range0_flag = 0;
            reset_pos_control();   // 機上XY PID を毎フライト初期化(v2.1)
            transition_to(AUTO_TAKEOFF, Reason::START_CMD);
        }
    }

    // アイドル状態の各種リセット(離陸へ遷移していない場合のみ。実績挙動を踏襲)
    if (mode.auto_state == AUTO_WAIT) {
        sensor_state.OverG_flag = 0;
        sensor_state.Range0flag = 0;
        out.Thrust0 = 0.0f;
        mode.Alt_flag = 0;
        out.Roll_rate_reference = 0.0f;
        out.Pitch_rate_reference = 0.0f;
        out.Yaw_rate_reference = 0.0f;
        reset_yaw_angle_control();
        reset_pos_control();
        landing_state = 0;
        auto_takeoff_counter = 0;
        prev_range0_flag = 0;
        thrust_filtered.reset();
        reset_duty_filters();
    }
}

// MOTOR_TEST 状態(v2 契約 §2.4): 飛行制御 PID・ミキサは動かさず、PWM の
// 書き手はモーターテストサービスのみ。CMD_STOP を最優先で脱出する。
void handle_motor_test(const CommandSnapshot& cmd) {
    // STOP(最優先): 即時モーター停止 → WAIT へ
    if (cmd.stop_pending) {
        motor_test_stop_now();
        enter_wait(Reason::STOP_CMD);
        return;
    }
    // MOTOR_TEST 中の CMD_START は拒否(reason=10)
    if (!cmd.stop_pending && cmd.start_pending) {
        reject_start(Reason::START_REJECTED_NOT_READY);
    }
    // キープアライブ途絶(1.5s)の自動停止+ソフトスタートランプ
    motor_test_service();
}

void handle_flight(const CommandSnapshot& cmd) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& mode = flight_control_state.mode;
    const uint32_t now_ms = millis();

    // STOP(最優先): 即時着陸
    bool stop_requested = false;
    if (cmd.stop_pending) {
        transition_to(AUTO_LANDING, Reason::STOP_CMD);
        stop_requested = true;
    }

    // 離陸完了判定: 目標高度に到達したらHOVERへ
    if (!stop_requested && mode.auto_state == AUTO_TAKEOFF &&
        sensor_state.Altitude2 >= flight_control_state.output.Alt_ref - cfg.takeoff_complete_margin_m) {
        transition_to(AUTO_HOVER, Reason::NONE);
    }

    // フェイルセーフ(PROTOCOL.md の表)
    if (mode.auto_state != AUTO_LANDING && low_voltage_detected()) {
        transition_to(AUTO_LANDING, Reason::LOW_VOLTAGE);
    }
    if (mode.auto_state != AUTO_LANDING && (now_ms - mode.flight_start_ms) > cfg.max_flight_time_ms) {
        transition_to(AUTO_LANDING, Reason::MAX_FLIGHT_TIME);
    }
    if (mode.auto_state != AUTO_LANDING && setpoint_link_age_ms(now_ms) > cfg.link_loss_landing_ms) {
        transition_to(AUTO_LANDING, Reason::LINK_LOSS);
    }
    if (sensor_state.OverG_flag == 1) {
        transition_to(AUTO_COMPLETE, Reason::OVER_G);
    }

    // 指令更新 → 制御(遷移直後のtickも実績どおり制御を1回実行する。
    // OverG時は rate_control 内で即時モータ停止)
    update_thrust_and_attitude_command();
    angle_control();
    rate_control();
}

void handle_landing(const CommandSnapshot& cmd) {
    (void)cmd;  // STOPは受理するが、すでに着陸中のため挙動は変わらない

    if (auto_landing_step() == 1) {
        enter_wait(Reason::LANDED);
    }
    angle_control();
    rate_control();
}

void handle_complete(const CommandSnapshot& cmd) {
    const FlightConfig& cfg = FLIGHT_CONFIG;
    auto& mode = flight_control_state.mode;
    auto& out = flight_control_state.output;

    motor_stop();
    sensor_state.OverG_flag = 0;
    sensor_state.Range0flag = 0;
    out.Thrust0 = 0.0f;
    mode.Alt_flag = 0;
    out.Roll_rate_reference = 0.0f;
    out.Pitch_rate_reference = 0.0f;
    out.Yaw_rate_reference = 0.0f;
    reset_yaw_angle_control();
    reset_pos_control();
    landing_state = 0;
    auto_takeoff_counter = 0;
    thrust_filtered.reset();
    reset_duty_filters();
    // 高度推定器はリセットしない: CMD_RESET の altitude_est<0.15m ガードを有効に保つ

    // CMD_RESET: COMPLETE かつ altitude_est < 0.15m でのみ受理(STOP/STARTは無視)
    if (cmd.reset_pending && sensor_state.Altitude2 < cfg.reset_accept_alt_m) {
        enter_wait(Reason::RESET_CMD);
    }
}

// ---------------------------------------------------------------------------
// v2 コマンド処理(0x14-0x23, 0x25-0x28。契約 §1.2 / §2.4 / §2.5、
// 磁気オートチューン/フロー拡張は MAG_AUTOTUNE_DESIGN.md §1.4)
//
// 全コマンドに TLM_ACK で応答する。キャリブ/FF系(0x17-0x23)と 0x26-0x28 は
// WAIT / COMPLETE / MOTOR_TEST でのみ受理(飛行中の NVS 書込み禁止)。
// 処理内容は yaw側 command.cpp の各ハンドラをバイナリプロトコルへ写像したもの。
// ---------------------------------------------------------------------------

// キャリブ/FF系コマンドを受理できる状態か(非飛行=NVS書込み可)
bool in_cal_command_state(void) {
    const AutoFlightState st = flight_control_state.mode.auto_state;
    return st == AUTO_WAIT || st == AUTO_COMPLETE || st == AUTO_MOTOR_TEST;
}

// yaw側 resetYawEstimatorKeepingZero の移植: キャリブ変更後にリファレンスCFを
// 現在ヨーで再シードする。reset() は捕捉済み磁気オフセットを消して次の有効
// 磁気サンプルで再捕捉するため、ヨーゼロ有効時は遅延NVS保存を予約する
// (捕捉後に service_pending_yaw_zero_save が永続化する)。
void reset_yaw_estimator_keeping_zero(void) {
    g_yaw_est.yaw_estimator.reset(g_yaw_est.yaw_estimator.yaw());
    if (g_yaw_est.yaw_estimator.yawZeroValid()) {
        g_yaw_est.yaw_zero_save_pending = true;
    }
}

// yaw側 clearAttitudeMountZero の移植
void clear_attitude_mount_zero(void) {
    g_yaw_est.roll_mount_offset_rad = 0.0f;
    g_yaw_est.pitch_mount_offset_rad = 0.0f;
    g_yaw_est.attitude_mount_valid = false;
    saveAttitudeMountZero(false, 0.0f, 0.0f);
}

// yaw_zero の遅延NVS保存(yaw側 commandService の移植): 磁気オフセットが
// 実際に捕捉されてから、非飛行状態でのみ1回だけ書き込む。
void service_pending_yaw_zero_save(void) {
    if (!g_yaw_est.yaw_zero_save_pending) return;
    if (!in_cal_command_state()) return;
    if (!g_yaw_est.yaw_estimator.yawZeroOffsetValid()) return;
    saveYawZero(true, g_yaw_est.yaw_estimator.magYawOffsetRad());
    g_yaw_est.yaw_zero_save_pending = false;
}

// CMD_MODE: WAIT --mode=1--> MOTOR_TEST / MOTOR_TEST --mode=0--> WAIT。
// 他状態では bad_state(契約 §1.2)。
uint8_t handle_cmd_mode(const stampfly::CmdMode& m) {
    if (m.mode > stampfly::CmdMode::MODE_MOTOR_TEST) return TlmAck::STATUS_INVALID_ARG;
    const AutoFlightState st = flight_control_state.mode.auto_state;
    if (m.mode == stampfly::CmdMode::MODE_MOTOR_TEST) {
        if (st == AUTO_MOTOR_TEST) return TlmAck::STATUS_OK;  // 冪等(ACKロスト再送)
        if (st != AUTO_WAIT) return TlmAck::STATUS_BAD_STATE;
        motor_test_stop_now();  // duty/mask/キープアライブタイマを初期化
        transition_to(AUTO_MOTOR_TEST, Reason::MODE_CHANGE);
        return TlmAck::STATUS_OK;
    }
    // mode == MODE_FLIGHT
    if (st == AUTO_WAIT) return TlmAck::STATUS_OK;  // 冪等(ACKロスト再送)
    if (st != AUTO_MOTOR_TEST) return TlmAck::STATUS_BAD_STATE;
    motor_test_stop_now();  // モーター停止後に WAIT へ(契約 §2.4)
    enter_wait(Reason::MODE_CHANGE);
    return TlmAck::STATUS_OK;
}

// CMD_MAG3D_SET: 適用/クリア+NVS 永続化+FF自動無効+アンカー破棄+再シード
uint8_t handle_cmd_mag3d_set(const stampfly::CmdMag3dSet& m) {
    if (m.valid != 0) {
        const MagVector offset{m.offset[0], m.offset[1], m.offset[2]};
        if (!g_yaw_est.mag3d_calibration.set(offset, m.matrix)) {
            return TlmAck::STATUS_INVALID_ARG;
        }
    } else {
        g_yaw_est.mag3d_calibration.reset();
    }
    g_yaw_est.mag_filter.reset();
    saveMag3DCalibration(g_yaw_est.mag3d_calibration);
    reset_yaw_estimator_keeping_zero();
    // b_cal 空間が変わる: アンカー破棄+補正系再シード+ff_mode=0 強制(NVS込み)
    sensorHubFfOnMag3dChange();
    return TlmAck::STATUS_OK;
}

// CMD_ACCEL6_SET: 係数の NVS 復元系(yaw側 accel6_set/accel6_clear 相当)。
// V2 では飛行AHRS(Madgwick)へは未適用(飛行挙動不変の分離設計)だが、
// yaw側仕様どおり姿勢参照(マウントゼロ・AHRS・リファレンスCF)をリセットする。
uint8_t handle_cmd_accel6_set(const stampfly::CmdAccel6Set& m) {
    if (m.valid != 0) {
        const AccelVector offset{m.offset[0], m.offset[1], m.offset[2]};
        const AccelVector scale{m.scale[0], m.scale[1], m.scale[2]};
        if (!g_yaw_est.accel_calibration.setCalibration(offset, scale)) {
            return TlmAck::STATUS_INVALID_ARG;
        }
    } else {
        g_yaw_est.accel_calibration.reset();
    }
    saveAccelCalibration(g_yaw_est.accel_calibration);
    // yaw側 resetAttitudeReferencesAfterAccelChange の移植
    clear_attitude_mount_zero();
    ahrs_reset();  // 非飛行状態でのみ受理されるため安全(Yaw_gyro_integral も0へ)
    reset_yaw_estimator_keeping_zero();
    return TlmAck::STATUS_OK;
}

// CMD_ATTMOUNT_SET: マウントオフセット設定/クリア(磁気レベル化入力にのみ適用)
uint8_t handle_cmd_attmount_set(const stampfly::CmdAttmountSet& m) {
    if (m.valid != 0) {
        // rad かつ ±π 内のみ受理(yaw側 applyAttitudeMountSetCommand と同じ検査)
        if (!isfinite(m.roll_rad) || !isfinite(m.pitch_rad) ||
            fabsf(m.roll_rad) > PI || fabsf(m.pitch_rad) > PI) {
            return TlmAck::STATUS_INVALID_ARG;
        }
        g_yaw_est.roll_mount_offset_rad = m.roll_rad;
        g_yaw_est.pitch_mount_offset_rad = m.pitch_rad;
        g_yaw_est.attitude_mount_valid = true;
        saveAttitudeMountZero(true, m.roll_rad, m.pitch_rad);
    } else {
        clear_attitude_mount_zero();
    }
    reset_yaw_estimator_keeping_zero();
    return TlmAck::STATUS_OK;
}

// CMD_YAWZERO_SET: 保存済みヨーゼロ(磁気オフセット)の復元/クリア
uint8_t handle_cmd_yawzero_set(const stampfly::CmdYawzeroSet& m) {
    if (m.valid != 0) {
        if (!isfinite(m.offset_rad) || fabsf(m.offset_rad) > PI) {
            return TlmAck::STATUS_INVALID_ARG;
        }
        g_yaw_est.yaw_estimator.restoreYawZero(m.offset_rad);
        // 推定器が実際に保持したラップ済み値を永続化(NVSとRAMの乖離防止)
        saveYawZero(true, g_yaw_est.yaw_estimator.magYawOffsetRad());
        g_yaw_est.yaw_zero_save_pending = false;
        // ヨーゼロ設定と同時に Madgwick ヨー(ヨー成分のみ回転。ロール/ピッチ
        // 無傷)とジャイロ積算も 0 基準へ揃える(3系統のヨー基準統一)。
        // 本コマンドは dispatch 側の非飛行ガード(WAIT/COMPLETE/MOTOR_TEST)を
        // 通過済みだが、MOTOR_TEST はモーター回転中があり得るため、実PWM出力
        // なし(直近tickの motors_running=false)のときのみ実行する。
        // なお UI「Yaw 0」以外にプロファイル適用(calprofile)でも valid=1 が
        // 届くが、Madgwickヨーの0基準はもともと任意(WAIT静定/START で全リセット)
        // のため現在方位への再基準化は無害。
        if (!g_yaw_est.motors_running) {
            ahrs_zero_yaw();
        }
    } else {
        g_yaw_est.yaw_estimator.clearYawZero();
        g_yaw_est.yaw_zero_save_pending = false;
        saveYawZero(false, 0.0f);
    }
    return TlmAck::STATUS_OK;
}

// CMD_GEOMAG_SET: 地磁気リファレンス設定(deg→rad 換算、許容値は既定値)
uint8_t handle_cmd_geomag_set(const stampfly::CmdGeomagSet& m) {
    GeomagneticReference reference;  // 許容値・z符号は yaw_config.hpp の既定値
    reference.valid = true;
    reference.declination_east_rad = m.declination_east_deg * DEG_TO_RAD;
    reference.inclination_rad = m.inclination_deg * DEG_TO_RAD;
    reference.horizontal_uT = m.horizontal_ut;
    reference.vertical_uT = m.vertical_ut;
    reference.total_uT = m.total_ut;
    if (!g_yaw_est.yaw_estimator.setGeomagneticReference(reference)) {
        return TlmAck::STATUS_INVALID_ARG;
    }
    saveGeomagneticReference(g_yaw_est.yaw_estimator.geomagneticReference());
    return TlmAck::STATUS_OK;
}

// CMD_FF_COMMIT: CRC 照合→NVS 永続化→補正系再シード。冪等(yaw側 C5)。
// commit の失敗理由(静的文字列)を TLM_ACK status へ分類する。
uint8_t handle_cmd_ff_commit(const stampfly::CmdFfCommit& m) {
    const char* message = "";
    if (!g_yaw_est.ff_calibration.commit(m.crc32, message)) {
        if (std::strcmp(message, "crc mismatch") == 0) return TlmAck::STATUS_CRC_MISMATCH;
        if (std::strstr(message, "missing") != nullptr ||
            std::strstr(message, "no staging") != nullptr) {
            return TlmAck::STATUS_INCOMPLETE;
        }
        return TlmAck::STATUS_INVALID_ARG;  // LUT非昇順・非有限値など
    }
    saveFfCalibration(g_yaw_est.ff_calibration, g_yaw_est.ff.ff_mode, g_yaw_est.ff.est_mode);
    sensorHubFfReseed();  // 新係数で補正パスが変わるため再シード
    return TlmAck::STATUS_OK;
}

// CMD_FF_MODE: ff_mode / est_mode の実行時切替+NVS 永続化+再シード。
// est_mode の受理範囲は 0..2(2=EKF2。契約 §1.3)。
uint8_t handle_cmd_ff_mode(const stampfly::CmdFfMode& m) {
    if (m.ff_mode > stampfly::CmdFfMode::FF_MODE_B ||
        m.est_mode > stampfly::CmdFfMode::EST_MODE_EKF2) {
        return TlmAck::STATUS_INVALID_ARG;
    }
    const uint8_t prev_est_mode = g_yaw_est.ff.est_mode;
    // 切替前のアクティブ推定器ヨー(EKF2 アクティブ化時の再シード種)
    const float prev_active_yaw = g_yaw_est.ff.yaw_active_rad;
    g_yaw_est.ff.ff_mode = m.ff_mode;
    g_yaw_est.ff.est_mode = m.est_mode;
    saveFfModes(m.ff_mode, m.est_mode);
    sensorHubFfReseed();
    // 契約 §1.3: EKF2 のアクティブ化時は「直近のアクティブ推定器ヨー」で
    // reseedYaw する(sensorHubFfReseed のリファレンスCF種を上書き)。
    if (m.est_mode == stampfly::CmdFfMode::EST_MODE_EKF2 &&
        prev_est_mode != stampfly::CmdFfMode::EST_MODE_EKF2) {
        g_yaw_est.yaw_kf2.reseedYaw(prev_active_yaw);
    }
    return TlmAck::STATUS_OK;
}

// CMD_MAGBIAS_SET: 学習ハードアイアン残差 Δb [µT, b_cal 空間] の適用/クリア
// (契約 §1.4)。NVS `magbias` 保存+即適用+アンカー無効化+窓リセット+
// 補正系再シード(mag3d 変更時と同パターン)。**ff_mode は降格しない**
// (FF 係数は b_cal 空間のまま有効)。受理状態はキャリブ系と同じ非飛行ガード。
uint8_t handle_cmd_magbias_set(const stampfly::CmdMagbiasSet& m) {
    if (m.mode > stampfly::CmdMagbiasSet::MODE_SET) {
        return TlmAck::STATUS_INVALID_ARG;
    }
    if (m.mode == stampfly::CmdMagbiasSet::MODE_SET) {
        if (!isfinite(m.dx) || !isfinite(m.dy) || !isfinite(m.dz)) {
            return TlmAck::STATUS_INVALID_ARG;
        }
        const MagVector delta{m.dx, m.dy, m.dz};
        sensorHubFfApplyMagbias(true, delta);
        saveMagbias(true, delta);
    } else {
        sensorHubFfApplyMagbias(false, MagVector{});
        saveMagbias(false, MagVector{});
    }
    return TlmAck::STATUS_OK;
}

// 1件の v2 コマンドを処理して TLM_ACK の status を返す。
// send_cal_data_after: ACK 送信後に TLM_CAL_DATA を送るべきか(CMD_CAL_GET)。
uint8_t process_one_v2_command(const V2Command& c, bool& send_cal_data_after) {
    using stampfly::MsgType;
    const MsgType type = static_cast<MsgType>(c.type);

    // キャリブ/FF系(0x17-0x23)と CMD_LED_MODE(0x25)、磁気オートチューン/
    // フロー系(0x26-0x28)は WAIT/COMPLETE/MOTOR_TEST のみ受理
    // (契約 §1.2 / MAG_AUTOTUNE_DESIGN.md §1.4: 飛行中の NVS 書込み禁止。
    //  LED_MODE も同じ非飛行ガードに相乗り)
    if (((c.type >= static_cast<uint8_t>(MsgType::CMD_CAL_GET) &&
          c.type <= static_cast<uint8_t>(MsgType::CMD_FF_ANCHOR)) ||
         c.type == static_cast<uint8_t>(MsgType::CMD_LED_MODE) ||
         (c.type >= static_cast<uint8_t>(MsgType::CMD_MAGBIAS_SET) &&
          c.type <= static_cast<uint8_t>(MsgType::CMD_FLOW_PROBE))) &&
        !in_cal_command_state()) {
        return TlmAck::STATUS_BAD_STATE;
    }

    switch (type) {
        case MsgType::CMD_MODE: {
            stampfly::CmdMode m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_mode(m);
        }
        case MsgType::CMD_MOTOR_RUN: {
            // MOTOR_TEST 状態のみ。飛行状態では PWM に触れず bad_state で拒否
            if (flight_control_state.mode.auto_state != AUTO_MOTOR_TEST) {
                return TlmAck::STATUS_BAD_STATE;
            }
            stampfly::CmdMotorRun m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (!isfinite(m.duty) || m.duty < 0.0f || m.duty > 1.0f) {
                return TlmAck::STATUS_INVALID_ARG;
            }
            motor_test_set(m.duty, m.mask);
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_MOTOR_STOP: {
            if (flight_control_state.mode.auto_state != AUTO_MOTOR_TEST) {
                return TlmAck::STATUS_BAD_STATE;
            }
            motor_test_stop_now();
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_CAL_GET:
            send_cal_data_after = true;  // ACK(ok) の後に TLM_CAL_DATA を返す
            return TlmAck::STATUS_OK;
        case MsgType::CMD_MAG3D_SET: {
            stampfly::CmdMag3dSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_mag3d_set(m);
        }
        case MsgType::CMD_ACCEL6_SET: {
            stampfly::CmdAccel6Set m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_accel6_set(m);
        }
        case MsgType::CMD_ATTMOUNT_SET: {
            stampfly::CmdAttmountSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_attmount_set(m);
        }
        case MsgType::CMD_YAWZERO_SET: {
            stampfly::CmdYawzeroSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_yawzero_set(m);
        }
        case MsgType::CMD_GEOMAG_SET: {
            stampfly::CmdGeomagSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_geomag_set(m);
        }
        case MsgType::CMD_FF_BEGIN: {
            stampfly::CmdFfBegin m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (m.nlut < stampfly::CmdFfBegin::NLUT_MIN ||
                m.nlut > stampfly::CmdFfBegin::NLUT_MAX ||
                !g_yaw_est.ff_calibration.stageBegin(m.nlut)) {
                return TlmAck::STATUS_INVALID_ARG;
            }
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FF_LUT: {
            stampfly::CmdFfLut m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (!g_yaw_est.ff_calibration.stageLutPoint(m.idx, m.i_a, m.db_x, m.db_y, m.db_z)) {
                return TlmAck::STATUS_INVALID_ARG;  // begin 未実行 / idx 範囲外 / 非有限値
            }
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FF_MOT: {
            stampfly::CmdFfMot m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (!g_yaw_est.ff_calibration.stageMotor(
                    m.idx, m.a_tilde[0], m.a_tilde[1], m.a_tilde[2], m.c2, m.c1, m.c0)) {
                return TlmAck::STATUS_INVALID_ARG;
            }
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FF_AUX: {
            stampfly::CmdFfAux m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (!g_yaw_est.ff_calibration.stageAux(m.iid_a)) return TlmAck::STATUS_INVALID_ARG;
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FF_COMMIT: {
            stampfly::CmdFfCommit m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_ff_commit(m);
        }
        case MsgType::CMD_FF_MODE: {
            stampfly::CmdFfMode m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_ff_mode(m);
        }
        case MsgType::CMD_FF_ANCHOR:
            // モーター回転中(実出力あり)/停止窓が未充填なら busy で拒否可(契約 §1.2)
            return sensorHubFfAnchorNow() ? TlmAck::STATUS_OK : TlmAck::STATUS_BUSY;
        case MsgType::CMD_LED_MODE: {
            // 計測中 LED インジケータ。表示効果は MOTOR_TEST 中のみ
            // (フェイルセーフ・自動復帰は indicators 側が管理する)。
            stampfly::CmdLedMode m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (m.mode > stampfly::CmdLedMode::MODE_RECORDING) return TlmAck::STATUS_INVALID_ARG;
            indicators_set_led_mode(m.mode);
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_MAGBIAS_SET: {
            stampfly::CmdMagbiasSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            return handle_cmd_magbias_set(m);
        }
        case MsgType::CMD_FLOWCAL_SET: {
            // フロー較正行列 K の適用/クリア+NVS `flowcal`(MAG_AUTOTUNE_DESIGN.md
            // §1.4/§2.5。2026-07-31 改訂: 2×2 行列化 17B)。mode=0 でクリア=
            // 既定 diag(450,450) に戻る。妥当性(対角スケール範囲・非対角上限・
            // det≈0 ガード)は flowcalMatrixValid に集約。
            stampfly::CmdFlowcalSet m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (m.mode > stampfly::CmdFlowcalSet::MODE_SET) return TlmAck::STATUS_INVALID_ARG;
            if (m.mode == stampfly::CmdFlowcalSet::MODE_SET) {
                if (!flowcalMatrixValid(m.m00, m.m01, m.m10, m.m11)) {
                    return TlmAck::STATUS_INVALID_ARG;
                }
                // 即適用+NVS 保存(K⁻¹ 計算不成立は防御的に invalid_arg)
                if (!flowHubApplyCalibration(m.m00, m.m01, m.m10, m.m11)) {
                    return TlmAck::STATUS_INVALID_ARG;
                }
            } else {
                flowHubClearCalibration();  // 既定 diag(450,450) へ+NVS クリア
            }
            return TlmAck::STATUS_OK;
        }
        case MsgType::CMD_FLOW_PROBE: {
            // PMW3901 burst プローブ(MAG_AUTOTUNE_DESIGN.md §1.4/§2.4)。
            // 実行中は 400Hz ループが数秒ブロックする(テレメトリ・failsafe も
            // 止まる)ためモーター停止時のみ: 非飛行ガードは通過済みだが、
            // MOTOR_TEST 中の実出力あり(ランプダウン含む)は busy で拒否する。
            stampfly::CmdFlowProbe m;
            if (!stampfly::deserialize(c.payload, c.len, &m)) return TlmAck::STATUS_INVALID_ARG;
            if (motor_test_output_active()) return TlmAck::STATUS_BUSY;
            // n_cycles=0 は既定 200。結果要約は LOG_TEXT 複数行、合否で
            // burst_ok ラッチ更新(flow_hub 側)。実行不能(センサー init
            // 不成立等)は incomplete。
            if (!flowHubRunBurstProbe(m.n_cycles)) return TlmAck::STATUS_INCOMPLETE;
            return TlmAck::STATUS_OK;
        }
        default:
            // comm 層のレンジ検査により到達しない(防御的に invalid_arg)
            return TlmAck::STATUS_INVALID_ARG;
    }
}

// 取り出した v2 コマンド群を受信順に処理し、全件へ TLM_ACK を返す
void process_v2_commands(const CommandSnapshot& cmd) {
    for (size_t i = 0; i < cmd.v2_count; i++) {
        const V2Command& c = cmd.v2[i];
        bool send_cal_data_after = false;
        const uint8_t status = process_one_v2_command(c, send_cal_data_after);
        telemetry_send_ack(c.type, c.seq, status);
        if (send_cal_data_after && status == TlmAck::STATUS_OK) {
            telemetry_send_cal_data();
        }
    }
}

// ループ実測タイミングの更新(loop_dt_us テレメトリの根拠)
void update_loop_timing(void) {
    auto& t = flight_control_state.timing;
    t.E_time = micros();
    t.Old_Elapsed_time = t.Elapsed_time;
    t.Elapsed_time = 1.0e-6f * (t.E_time - t.S_time);
    t.Interval_time = t.Elapsed_time - t.Old_Elapsed_time;
}

// v2: 角度/角速度ループの PID 成分を TLM_CTRL 用に毎tick転記する(telemetry が
// 25Hz で読む)。PID::reset() が成分も 0 にするため、リセット中(飛行中の
// 低スラスト・ヨー制御OFF時の psi_pid 毎tickリセット等)はそのまま 0 が載る。
// 非飛行状態では PID オブジェクトが直前飛行の値を保持したままになる経路が
// ある(緩減衰着陸では低スラストリセットが発火しない)ため、契約
// 「非飛行時は成分 0」を転記側で保証する(判定は TLM_STATE FLAG_FLYING と同一)。
// yaw の角度ループ成分はクランプ前の値(psi_pid 出力の3分解)、
// out.Yaw_rate_reference はクランプ後 → 差でクランプ発動が分かる(契約 §TLM_CTRL)。
void update_control_diag(void) {
    auto& diag = flight_control_state.diag;
    const AutoFlightState st = flight_control_state.mode.auto_state;
    if (st != AUTO_TAKEOFF && st != AUTO_HOVER && st != AUTO_LANDING) {
        for (int i = 0; i < 9; ++i) {
            diag.pid_ang[i] = 0.0f;
            diag.pid_rate[i] = 0.0f;
        }
        return;
    }
    diag.pid_ang[0] = phi_pid.p_term();
    diag.pid_ang[1] = phi_pid.i_term();
    diag.pid_ang[2] = phi_pid.d_term();
    diag.pid_ang[3] = theta_pid.p_term();
    diag.pid_ang[4] = theta_pid.i_term();
    diag.pid_ang[5] = theta_pid.d_term();
    diag.pid_ang[6] = psi_pid.p_term();
    diag.pid_ang[7] = psi_pid.i_term();
    diag.pid_ang[8] = psi_pid.d_term();
    diag.pid_rate[0] = p_pid.p_term();
    diag.pid_rate[1] = p_pid.i_term();
    diag.pid_rate[2] = p_pid.d_term();
    diag.pid_rate[3] = q_pid.p_term();
    diag.pid_rate[4] = q_pid.i_term();
    diag.pid_rate[5] = q_pid.d_term();
    diag.pid_rate[6] = r_pid.p_term();
    diag.pid_rate[7] = r_pid.i_term();
    diag.pid_rate[8] = r_pid.d_term();
}

}  // namespace

// ---------------------------------------------------------------------------
// 公開API
// ---------------------------------------------------------------------------

void init_copter(void) {
    USBSerial.begin(115200);
    delay(FLIGHT_CONFIG.boot_serial_wait_ms);
    USBSerial.printf("StampFly Integrated Control: boot\r\n");

    flight_control_state.timing.Control_period = FLIGHT_CONFIG.control_period_s;
    flight_control_state.output.Alt_ref = FLIGHT_CONFIG.default_alt_ref_m;

    init_pwm();
    sensor_init();
    control_init();
    indicators_init();
    comm_init();

    // 400Hz割り込み(1µsティック、2500µs周期)
    loop_timer = timerBegin(0, FLIGHT_CONFIG.timer_prescaler, true);
    timerAttachInterrupt(loop_timer, &on_loop_timer, true);
    timerAlarmWrite(loop_timer, FLIGHT_CONFIG.loop_period_us, true);
    timerAlarmEnable(loop_timer);

    USBSerial.printf("StampFly Integrated Control: init done\r\n");
    // 以後、定常状態ではUSBシリアルへ出力しない(人間向けメッセージはLOG_TEXT)
}

void loop_400Hz(void) {
    // タイマ割り込みによる400Hzペーシング(製品版と同一方式)
    while (flight_control_state.timing.Loop_flag == 0) {
    }
    flight_control_state.timing.Loop_flag = 0;

    update_loop_timing();
    sensor_read();
    update_trim_voltage();   // トリムFF用電圧LPF(毎tick。§config trim_voltage_filter_tc_s)

    // 受信コマンドの取り出しと setpoint / pos_err 適用
    CommandSnapshot cmd;
    comm_consume_commands(&cmd);
    apply_setpoint(cmd);
    apply_pos_err(cmd);

    // v2 コマンド(0x14-0x23, 0x25-0x28)の処理+TLM_ACK 応答。CMD_MODE の遷移は
    // この直後の状態スイッチで同tick内に反映される(STOP は各ハンドラが最優先)。
    process_v2_commands(cmd);

    // yaw_zero の遅延NVS保存(磁気オフセット捕捉後、非飛行状態のみ)
    service_pending_yaw_zero_save();

    switch (flight_control_state.mode.auto_state) {
        case AUTO_INIT:
            handle_init();
            break;
        case AUTO_CALIBRATION:
            handle_calibration();
            break;
        case AUTO_WAIT:
            handle_wait(cmd);
            break;
        case AUTO_TAKEOFF:
        case AUTO_HOVER:
            handle_flight(cmd);
            break;
        case AUTO_LANDING:
            handle_landing(cmd);
            break;
        case AUTO_COMPLETE:
            handle_complete(cmd);
            break;
        case AUTO_MOTOR_TEST:
            handle_motor_test(cmd);
            break;
    }

    update_control_diag();  // PID 成分の転記(TLM_CTRL。telemetry_update より前)
    telemetry_update();
    indicators_update();
}

// --- v2: モーターテスト実出力のアクセサ(sensor.cpp の FF duty 配線・telemetry 用) ---

float motor_test_applied_duty(void) {
    return mt_applied_duty;
}

uint8_t motor_test_active_mask(void) {
    return mt_mask;
}

bool motor_test_output_active(void) {
    // ランプダウン中の実出力>0 も「回転中」として扱う(アンカー窓の汚染防止)
    return mt_applied_duty > 0.0f || mt_running;
}
