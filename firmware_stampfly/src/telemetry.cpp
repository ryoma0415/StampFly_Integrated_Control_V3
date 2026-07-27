// ===========================================================================
// telemetry.cpp — TLM_STATE / TLM_EVENT / TLM_CTRL 生成 実装
// ===========================================================================
#include "telemetry.hpp"

#include <Arduino.h>

#include "comm.hpp"
#include "config.hpp"
#include "flight_control.hpp"
#include "flow/flow_hub.hpp"
#include "imu.hpp"
#include "sensor.hpp"
#include "stampfly_protocol.hpp"
#include "yaw_estimation/sensor_hub_ff.hpp"

namespace {

uint16_t state_divider_counter = 0;  // TLM_STATE 分周カウンタ
uint16_t exp_divider_counter = 0;    // TLM_EXP 分周カウンタ(TLM_STATE と 8tick 位相ずらし)
uint16_t ctrl_divider_counter = 0;   // TLM_CTRL 分周カウンタ(TLM_STATE と 4tick 位相ずらし)
uint32_t last_event_tx_ms = 0;       // 直近のTLM_EVENT送信時刻
bool event_valid = false;            // 一度でも遷移イベントが発生したか
uint8_t event_state = 0;             // 直近イベントの内容(2Hz再送に使う)
uint8_t event_prev_state = 0;
uint8_t event_reason = 0;

// TLM_STATE/TLM_EVENT 共通の flags(PROTOCOL.md: bit0 low_voltage,
// bit1 setpoint_fresh(<200ms), bit2 flying)
uint8_t build_status_flags(void) {
    uint8_t flags = 0;
    if (sensor_state.Under_voltage_flag >= UNDER_VOLTAGE_COUNT) {
        flags |= stampfly::TlmState::FLAG_LOW_VOLTAGE;
    }
    const auto& cs = flight_control_state.command;
    if (cs.setpoint_received &&
        (millis() - cs.last_setpoint_ms) < FLIGHT_CONFIG.link_level_hold_ms) {
        flags |= stampfly::TlmState::FLAG_SETPOINT_FRESH;
    }
    const AutoFlightState st = flight_control_state.mode.auto_state;
    if (st == AUTO_TAKEOFF || st == AUTO_HOVER || st == AUTO_LANDING) {
        flags |= stampfly::TlmState::FLAG_FLYING;
    }
    return flags;
}

void send_event_frame(void) {
    stampfly::TlmEvent ev;
    ev.state = event_state;
    ev.prev_state = event_prev_state;
    ev.reason = event_reason;
    ev.flags = build_status_flags();
    ev.voltage = sensor_state.Voltage;

    uint8_t payload[stampfly::TlmEvent::PAYLOAD_SIZE];
    if (!stampfly::serialize(ev, payload, sizeof(payload))) return;
    comm_send_payload(stampfly::MsgType::TLM_EVENT, payload, sizeof(payload));
    last_event_tx_ms = millis();
}

void send_state_frame(void) {
    const auto& fc = flight_control_state;
    stampfly::TlmState m;

    m.seq_echo = fc.command.applied_setpoint_seq;  // 未受信なら0
    m.elapsed_ms = millis();                       // 起動からの経過
    m.state = static_cast<uint8_t>(fc.mode.auto_state);
    m.flags = build_status_flags();
    m.reason = fc.mode.last_reason;

    // 実測姿勢・角速度(AHRS / ジャイロ)
    m.roll = sensor_state.Roll_angle;
    m.pitch = sensor_state.Pitch_angle;
    m.yaw = sensor_state.Yaw_angle;
    m.p = sensor_state.Roll_rate;
    m.q = sensor_state.Pitch_rate;
    m.r = sensor_state.Yaw_rate;

    // 適用中の指令
    m.roll_ref = fc.output.Roll_angle_command;
    m.pitch_ref = fc.output.Pitch_angle_command;
    m.alt_ref = fc.output.Alt_ref;

    // 高度系(ToF由来生値 / カルマン推定)
    m.altitude_tof = sensor_state.Altitude;
    m.altitude_est = sensor_state.Altitude2;
    m.alt_velocity = sensor_state.Alt_velocity;
    m.z_dot_ref = fc.output.Z_dot_ref;

    m.voltage = sensor_state.Voltage;

    m.duty_fr = fc.output.FrontRight_motor_duty;
    m.duty_fl = fc.output.FrontLeft_motor_duty;
    m.duty_rr = fc.output.RearRight_motor_duty;
    m.duty_rl = fc.output.RearLeft_motor_duty;

    // フィルタ後加速度 [g]
    m.ax = sensor_state.Accel_x;
    m.ay = sensor_state.Accel_y;
    m.az = sensor_state.Accel_z;

    // 直近の実測制御周期 [µs](u16へ飽和)
    float dt_us = fc.timing.Interval_time * 1.0e6f;
    if (dt_us < 0.0f) dt_us = 0.0f;
    if (dt_us > 65535.0f) dt_us = 65535.0f;
    m.loop_dt_us = static_cast<uint16_t>(dt_us);

    // --- v2 追加フィールド(オフセット97-134。契約 §1.4) ---
    m.yaw_est_rad = sensor_state.Yaw_est_rad;            // アクティブ推定器ヨー
    m.yaw_gyro_int_rad = sensor_state.Yaw_gyro_integral; // Z軸ジャイロ単純積算
    // 適用中ヨー目標(途絶ラッチ後含む)。ヨー角制御 off 時は 0。
    m.yaw_ref_rad = fc.output.Yaw_ctrl_active ? fc.output.Yaw_angle_command : 0.0f;
    m.current_a = sensor_state.Current_a;                // 総電流(20Hz更新)
    m.db_hat_x_ut = sensor_state.Db_hat_x_ut;            // FF補正ベクトル ΔB̂
    m.db_hat_y_ut = sensor_state.Db_hat_y_ut;
    m.bm_x_ut = sensor_state.Bm_x_ut;                    // EKF 磁気バイアス状態
    m.bm_y_ut = sensor_state.Bm_y_ut;
    m.nis = sensor_state.Ekf_nis;                        // 直近 EKF 更新の NIS
    m.ffg = sensor_state.Ekf_ffg;                        // EKF ゲート/健全性ビット
    uint8_t ff_status =
        static_cast<uint8_t>(sensor_state.Ff_mode & stampfly::TlmState::FF_STATUS_FF_MODE_MASK);
    if (sensor_state.Est_mode == 1) ff_status |= stampfly::TlmState::FF_STATUS_EST_EKF;
    // bit7 = est_mode==2(EKF2)。bit2/bit7 とも 0 なら相補CF
    // (MAG_AUTOTUNE_DESIGN.md §1.1)
    if (sensor_state.Est_mode == 2) ff_status |= stampfly::TlmState::FF_STATUS_EST_EKF2;
    if (sensor_state.Ff_anchor_valid) ff_status |= stampfly::TlmState::FF_STATUS_ANCHOR_VALID;
    if (sensor_state.Ff_cal_loaded) ff_status |= stampfly::TlmState::FF_STATUS_FFCAL_LOADED;
    if (fc.output.Yaw_ctrl_active) ff_status |= stampfly::TlmState::FF_STATUS_YAW_CTRL_ACTIVE;
    if (sensor_state.Mag_fresh) ff_status |= stampfly::TlmState::FF_STATUS_MAG_FRESH;
    m.ff_status = ff_status;

    // --- 磁気オートチューン/フロー拡張(オフセット135-183。MAG_AUTOTUNE_DESIGN.md §1.1) ---
    m.mag_cal_x_ut = sensor_state.Mag_cal_x_ut;          // b_cal(FF補正前)
    m.mag_cal_y_ut = sensor_state.Mag_cal_y_ut;
    m.mag_cal_z_ut = sensor_state.Mag_cal_z_ut;
    m.mag_lev_x_ut = sensor_state.Mag_lev_x_ut;          // レベル化観測 z(=EKF観測)
    m.mag_lev_y_ut = sensor_state.Mag_lev_y_ut;
    m.ekf2_yaw_rad = sensor_state.Ekf2_yaw_rad;          // EKF2 ψ(シャドー中も常時)
    m.ekf2_bm_x_ut = sensor_state.Ekf2_bm_x_ut;          // EKF2 磁気バイアス状態
    m.ekf2_bm_y_ut = sensor_state.Ekf2_bm_y_ut;
    m.ekf2_yaw_innov_rad = sensor_state.Ekf2_yaw_innov_rad;  // 直近ヨー観測イノベーション
    m.ekf2_status = sensor_state.Ekf2_status;            // EKF2_STATUS_* ビット
    m.ekf2_gate = sensor_state.Ekf2_gate;                // EKF2 ゲート(ffg と同一定義)
    // flow_*(オフセット173-183): sensor.cpp が flow/ の出力から毎tick転記した
    // 公開値(契約 §1.1/§2.4。burst 不成立時は vx/vy=0・status に sensor_ok のみ)
    m.flow_vx_mps = sensor_state.Flow_vx_mps;
    m.flow_vy_mps = sensor_state.Flow_vy_mps;
    m.flow_squal = sensor_state.Flow_squal;
    m.flow_status = sensor_state.Flow_status;
    m.flow_dt_ms = sensor_state.Flow_dt_ms;

    uint8_t payload[stampfly::TlmState::PAYLOAD_SIZE];
    if (!stampfly::serialize(m, payload, sizeof(payload))) return;
    comm_send_payload(stampfly::MsgType::TLM_STATE, payload, sizeof(payload));
}

// TLM_CTRL(制御ループ診断 89B)。25Hz・全飛行状態で常時送出(MOTOR_TEST 中も
// 含む。契約 §TLM_CTRL)。PID 成分は 400Hz ループが flight_control_state.diag へ
// 毎tick転記したスナップショット。yaw_rate_ref はクランプ後、pid_ang の yaw
// 成分はクランプ前 → 差でクランプ発動が分かる。
void send_ctrl_frame(void) {
    const auto& fc = flight_control_state;
    stampfly::TlmCtrl m;

    m.elapsed_ms = millis();  // TLM_STATE と同一クロック(行突合用)
    m.roll_rate_ref = fc.output.Roll_rate_reference;
    m.pitch_rate_ref = fc.output.Pitch_rate_reference;
    m.yaw_rate_ref = fc.output.Yaw_rate_reference;
    for (size_t i = 0; i < 9; i++) m.pid_ang[i] = fc.diag.pid_ang[i];
    for (size_t i = 0; i < 9; i++) m.pid_rate[i] = fc.diag.pid_rate[i];

    uint8_t flags = 0;
    if (fc.diag.xy_onboard_active) flags |= stampfly::TlmCtrl::FLAG_XY_ONBOARD_ACTIVE;
    if (fc.output.Yaw_ctrl_active) flags |= stampfly::TlmCtrl::FLAG_YAW_CTRL_ACTIVE;
    // flying の定義は TLM_STATE flags bit2 と同一(build_status_flags を単一の根拠にする)
    if (build_status_flags() & stampfly::TlmState::FLAG_FLYING) {
        flags |= stampfly::TlmCtrl::FLAG_FLYING;
    }
    m.flags = flags;

    uint8_t payload[stampfly::TlmCtrl::PAYLOAD_SIZE];
    if (!stampfly::serialize(m, payload, sizeof(payload))) return;
    comm_send_payload(stampfly::MsgType::TLM_CTRL, payload, sizeof(payload));
}

// TLM_EXP(実験テレメトリ 86B)。MOTOR_TEST 状態でのみ 25Hz 送出。
// 磁気生値/較正後値・電流・温度など FF 較正実験(スイープ)のサンプル源。
void send_exp_frame(void) {
    stampfly::TlmExp m;
    const auto& frame = g_yaw_est.frame;
    const auto& cur = g_yaw_est.current_sample;

    m.elapsed_ms = millis();
    m.current_a = cur.current_a;      // INA3221 CH2 総電流
    m.vbat_v = cur.bus_voltage_v;
    m.shunt_uv = cur.shunt_uv;
    m.bx_raw = frame.mag_raw_body.x;  // RHALL補償+軸変換後・mag3D前 [µT]
    m.by_raw = frame.mag_raw_body.y;
    m.bz_raw = frame.mag_raw_body.z;
    m.bx_cal = frame.mag_cal_body.x;  // mag3D 後 [µT]
    m.by_cal = frame.mag_cal_body.y;
    m.bz_cal = frame.mag_cal_body.z;
    m.imu_temp_c = imu_get_temperature();

    m.roll = sensor_state.Roll_angle;   // Madgwick [rad]
    m.pitch = sensor_state.Pitch_angle;
    m.yaw = sensor_state.Yaw_angle;
    m.p = sensor_state.Roll_rate;       // [rad/s]
    m.q = sensor_state.Pitch_rate;
    m.r = sensor_state.Yaw_rate;
    m.ax = sensor_state.Accel_x;        // フィルタ後 [g]
    m.ay = sensor_state.Accel_y;
    m.az = sensor_state.Accel_z;

    m.duty_cmd = motor_test_applied_duty();
    m.motors_mask = motor_test_active_mask();
    uint8_t flags = 0;
    if (cur.valid) flags |= stampfly::TlmExp::FLAG_CURRENT_VALID;
    if (sensor_state.Mag_fresh) flags |= stampfly::TlmExp::FLAG_MAG_FRESH;
    if (motor_test_output_active()) flags |= stampfly::TlmExp::FLAG_MOTORS_RUNNING;
    m.flags = flags;

    uint8_t payload[stampfly::TlmExp::PAYLOAD_SIZE];
    if (!stampfly::serialize(m, payload, sizeof(payload))) return;
    comm_send_payload(stampfly::MsgType::TLM_EXP, payload, sizeof(payload));
}

}  // namespace

void telemetry_notify_transition(uint8_t state, uint8_t prev_state, uint8_t reason) {
    event_state = state;
    event_prev_state = prev_state;
    event_reason = reason;
    event_valid = true;
    send_event_frame();  // 即時送信(リレー未学習時は黙って失敗)
}

void telemetry_update(void) {
    // TLM_EVENT 2Hz定期再送
    if (event_valid && (millis() - last_event_tx_ms) >= FLIGHT_CONFIG.event_resend_ms) {
        send_event_frame();
    }

    // TLM_STATE 25Hz(16分周)
    state_divider_counter++;
    if (state_divider_counter >= FLIGHT_CONFIG.telemetry_state_divider) {
        state_divider_counter = 0;
        send_state_frame();
    }

    // TLM_CTRL 25Hz(全状態で常時)。TLM_STATE と 4tick 位相をずらして
    // 同一tickでの2フレーム送出(送信バースト)を避ける(TLM_EXP の 8tick とも重ならない)。
    ctrl_divider_counter++;
    if (ctrl_divider_counter >= FLIGHT_CONFIG.telemetry_state_divider) {
        ctrl_divider_counter = 0;
    }
    if (ctrl_divider_counter == FLIGHT_CONFIG.telemetry_ctrl_phase_ticks) {
        send_ctrl_frame();
    }

    // TLM_EXP 25Hz(MOTOR_TEST 状態のみ)。TLM_STATE と 8tick 位相をずらして
    // 同一tickでの2フレーム送出(送信バースト)を避ける(契約 §1.3)。
    exp_divider_counter++;
    if (exp_divider_counter >= FLIGHT_CONFIG.telemetry_state_divider) {
        exp_divider_counter = 0;
    }
    if (exp_divider_counter == FLIGHT_CONFIG.telemetry_exp_phase_ticks &&
        flight_control_state.mode.auto_state == AUTO_MOTOR_TEST) {
        send_exp_frame();
    }
}

void telemetry_send_ack(uint8_t acked_type, uint32_t acked_seq, uint8_t status) {
    stampfly::TlmAck m;
    m.acked_type = acked_type;
    m.acked_seq = acked_seq;
    m.status = status;
    uint8_t payload[stampfly::TlmAck::PAYLOAD_SIZE];
    if (!stampfly::serialize(m, payload, sizeof(payload))) return;
    comm_send_payload(stampfly::MsgType::TLM_ACK, payload, sizeof(payload));
}

void telemetry_send_cal_data(void) {
    stampfly::TlmCalData m;
    const MagSoftIronCalibration& m3 = g_yaw_est.mag3d_calibration;
    const AccelSixFaceCalibration& a6 = g_yaw_est.accel_calibration;
    const YawEstimator& ye = g_yaw_est.yaw_estimator;
    const GeomagneticReference gm = ye.geomagneticReference();
    const FfCalibration& ffc = g_yaw_est.ff_calibration;

    uint8_t valid = 0;
    if (m3.valid()) valid |= stampfly::TlmCalData::VALID_MAG3D;
    if (a6.valid()) valid |= stampfly::TlmCalData::VALID_ACCEL6;
    if (g_yaw_est.attitude_mount_valid) valid |= stampfly::TlmCalData::VALID_ATTMOUNT;
    // yawzero は磁気オフセットが実際に捕捉済みのときのみ有効(yaw側 cal_data の yzv と同義)
    if (ye.yawZeroOffsetValid()) valid |= stampfly::TlmCalData::VALID_YAWZERO;
    if (gm.valid) valid |= stampfly::TlmCalData::VALID_GEOMAG;
    if (ffc.valid()) valid |= stampfly::TlmCalData::VALID_FFCAL;
    if (g_yaw_est.magbias_valid) valid |= stampfly::TlmCalData::VALID_MAGBIAS;
    // VALID_FLOWCAL(bit7)= NVS `flowcal` 較正済み(契約 §1.5)
    const FlowCalibration& fcal = flowHubCalibration();
    if (fcal.valid) valid |= stampfly::TlmCalData::VALID_FLOWCAL;
    m.valid_flags = valid;

    const MagVector m3_off = m3.offset();
    m.mag3d_offset[0] = m3_off.x;
    m.mag3d_offset[1] = m3_off.y;
    m.mag3d_offset[2] = m3_off.z;
    const float* mm = m3.matrix();
    for (size_t i = 0; i < 9; i++) m.mag3d_matrix[i] = mm[i];

    const AccelVector a6_off = a6.offset();
    const AccelVector a6_sc = a6.scale();
    m.accel6_offset[0] = a6_off.x;
    m.accel6_offset[1] = a6_off.y;
    m.accel6_offset[2] = a6_off.z;
    m.accel6_scale[0] = a6_sc.x;
    m.accel6_scale[1] = a6_sc.y;
    m.accel6_scale[2] = a6_sc.z;

    m.attmount_roll_rad = g_yaw_est.roll_mount_offset_rad;
    m.attmount_pitch_rad = g_yaw_est.pitch_mount_offset_rad;
    m.yawzero_offset_rad = ye.magYawOffsetRad();

    // 地磁気は内部 rad 保持 → プロトコルは deg(契約 §1.3 の cal_data 表)
    m.geomag[0] = gm.declination_east_rad * RAD_TO_DEG;
    m.geomag[1] = gm.inclination_rad * RAD_TO_DEG;
    m.geomag[2] = gm.horizontal_uT;
    m.geomag[3] = gm.vertical_uT;
    m.geomag[4] = gm.total_uT;

    m.ff_nlut = ffc.nlut();
    m.ff_crc32 = ffc.crc();
    m.ff_mode = g_yaw_est.ff.ff_mode;
    m.est_mode = g_yaw_est.ff.est_mode;

    // 磁気オートチューン拡張(オフセット112-131。MAG_AUTOTUNE_DESIGN.md §1.5):
    // magbias Δb [µT, b_cal 空間](無効時は0ベクトルを保証済み)。
    m.magbias[0] = g_yaw_est.magbias.x;
    m.magbias[1] = g_yaw_est.magbias.y;
    m.magbias[2] = g_yaw_est.magbias.z;
    // flowcal kx/ky [counts/rad]: 適用中の実スケールを送る(未較正時は既定
    // 450.0 が載り、bit7=0 で「既定値のまま」と読み分けられる。契約 §2.5)
    m.flowcal[0] = fcal.x_scale;
    m.flowcal[1] = fcal.y_scale;

    uint8_t payload[stampfly::TlmCalData::PAYLOAD_SIZE];
    if (!stampfly::serialize(m, payload, sizeof(payload))) return;
    comm_send_payload(stampfly::MsgType::TLM_CAL_DATA, payload, sizeof(payload));
}
