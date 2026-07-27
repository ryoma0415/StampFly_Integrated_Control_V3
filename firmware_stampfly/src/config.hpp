// ===========================================================================
// config.hpp — 機体ファームの全チューナブル集約
//
// ARCHITECTURE.md コーディング規約: マジックナンバーはこのファイル以外に
// 置かない。PIDゲインは飛行実績のある OptiTrack版
// (StampFly_OptiTrack_PID_Control_System/M5StampFly/src/flight_control.cpp:77-92。
//  ローカルの Previous_Version/ は削除済み、原典は
//  github.com/ryoma0415/StampFly_OptiTrack_PID_Control_System)
// の値を踏襲する。機体固有のバイアスはPC側プロファイルで扱い、ここには置かない。
// ===========================================================================
#pragma once

#include <math.h>
#include <stdint.h>

// PIDゲイン1軸分(pid.hpp の PID::set_parameter 引数順に対応)
struct PidGainConfig {
    float kp;   // 比例ゲイン
    float ti;   // 積分時間(大きいほど積分が弱い)
    float td;   // 微分時間
    float eta;  // 不完全微分のフィルタ係数
};

struct FlightConfig {
    // --- カスケードPIDゲイン(OptiTrack版より踏襲) ---
    PidGainConfig roll_rate{0.65f, 0.7f, 0.01f, 0.125f};
    PidGainConfig pitch_rate{0.95f, 0.7f, 0.025f, 0.125f};
    PidGainConfig yaw_rate{3.0f, 0.8f, 0.01f, 0.125f};
    PidGainConfig roll_angle{5.0f, 4.0f, 0.04f, 0.125f};
    PidGainConfig pitch_angle{5.0f, 4.0f, 0.04f, 0.125f};
    PidGainConfig altitude{0.38f, 10.0f, 0.5f, 0.125f};
    PidGainConfig z_velocity{0.08f, 0.95f, 0.08f, 0.125f};

    // --- ヨー角制御(v2: psi_pid。角度誤差 wrapPi → ヨーレート目標) ---
    // 初期ゲインは契約 §2.3 の kp=3.0, ki=0.3, kd=0.0。PID クラスは
    // (kp, ti, td) 形式のため ki は ti=kp/ki=10.0 で表現する。
    // ※ 未飛行の初期値であり、要飛行チューニング。
    PidGainConfig yaw_angle{3.0f, 10.0f, 0.0f, 0.125f};
    // psi_pid 出力クランプ [rad/s]。2026-07-17 ログ解析: ヨー角速度ループ出力が
    // ミキサ差動の最大消費者で duty=0.95 天井の主要因(相関+0.77)だったため
    // 1.0 → 0.5 に縮小(実運用のヨーレート指令 RMS は 0.15 rad/s で影響僅少。
    // PC 側ヨースルー 45°/s=0.785rad/s に対しランプ中は遅れが増える点に注意)
    float yaw_rate_limit_rad_s = 0.5f;

    // --- 機上XY位置制御(v2.1: CMD_POS_ERR。位置誤差 → roll/pitch 角度指令) ---
    // アルゴリズム・数値は旧 PC 側 XY PID(飛行実績あり。v4 で PC 側実装は
    // 削除済みで、以下の値が唯一の実体)を踏襲する。旧 PC 側計算との違いは
    // 誤差を機体ヨー推定で機体座標系へ回転(ヨー回転補償)してから PID に
    // 入れる点のみ。
    float pos_kp = 0.139f;                   // XY PID 比例ゲイン(旧 PC 実績値)
    float pos_ki = 0.020f;                   // XY PID 積分ゲイン(同上)
    float pos_kd = 0.204f;                   // XY PID 微分ゲイン(同上)
    float pos_output_limit_rad = 0.087f;     // 出力クランプ ≈±5°
    float pos_d_filter_alpha = 0.6f;         // D項LPF係数
    float pos_i_decay_rate = 0.98f;          // 異常回復期間の I 項減衰率
    float pos_i_update_threshold_m = 0.3f;   // I 項を更新する誤差上限 [m]
    float pos_invalid_i_decay = 0.995f;      // データ無効時の I 項微減衰率(旧 PC 実績値)
    float pos_invalid_d_scale = 0.3f;        // データ無効時の D 項抑制係数(同上)
    uint8_t pos_anomaly_recovery_ticks = 10; // 異常回復後に I 項を減衰させ続ける回数(同上)
    float pos_integral_error_factor_gain = 2.0f;  // 動的アンチワインドアップの誤差感度(同上)
    float pos_initial_dt_s = 0.02f;          // 初回/異常 dt の仮値(50Hz 想定)
    float pos_min_dt_s = 0.001f;             // これ未満の dt はスキップ(ゼロ除算防止)
    float pos_max_dt_s = 0.1f;               // これ超過は dt 仮値+D項再シード(途絶復帰時のD項暴れ防止)
    float pos_dt_floor_s = 0.01f;            // D項計算の dt 下限(ESP-NOW バースト受信によるD項スパイク防止)
    float pos_slew_rad_per_s = 30.0f * (float)M_PI / 180.0f;  // 出力スルーレート(server.json clamps と同値)
    float pos_err_clamp_m = 2.0f;            // 受信誤差の安全クランプ [m]
    // ヨー回転補償の符号。機体ヨー推定(Madgwick/EKF)の正方向が制御座標系
    // ヨー(上から見て CCW 正)と一致していれば +1。地上検証(機体を手で
    // +90° 回して TLM の roll_ref/pitch_ref の符号応答、または PC ログの
    // mocap_heading_deg と tlm_yaw_est の追従方向を比較)で確認すること。
    float pos_ctrl_yaw_sign = 1.0f;

    // --- 制御ループ周期 ---
    float control_period_s = 0.0025f;       // 400Hz 制御周期 [s]
    uint32_t loop_period_us = 2500;         // タイマアラーム周期 [µs]
    uint32_t timer_prescaler = 80;          // 80MHz APBクロック → 1µs ティック
    float altitude_pid_period_s = 0.0333f;  // 高度PIDの設計周期(set_parameter初期値)

    // --- フィルタ時定数 ---
    float duty_filter_tc_s = 0.003f;    // モータデューティ1次LPF
    float thrust_filter_tc_s = 0.01f;   // スラスト指令1次LPF

    // --- モータ・ミキサ ---
    float battery_nominal_v = 3.7f;          // 推力正規化に使う公称電圧 [V]
    float motor_on_duty_threshold = 0.1f;    // これ未満は制御リセット(実質モータ停止)
    float duty_min = 0.0f;                   // モータデューティ下限
    float duty_max = 0.95f;                  // モータデューティ上限
    float mixer_torque_gain = 0.25f;         // 各軸トルク→デューティ配分係数

    // --- 姿勢・高度クランプ(多層クランプのファーム層, ARCHITECTURE.md) ---
    float max_angle_ref_rad = 30.0f * (float)M_PI / 180.0f;  // roll/pitch指令 ±30°
    float alt_ref_min_m = 0.05f;             // CMD_SETPOINT alt_ref クランプ下限 [m]
    float alt_ref_max_m = 1.5f;              // CMD_SETPOINT alt_ref クランプ上限 [m]
    float default_alt_ref_m = 0.3f;          // 離陸時の既定目標高度 [m]
    float alt_sensor_limit_m = 2.0f;         // これ超過で高度センサ喪失扱い(Range0flag加算)

    // --- 離陸シーケンス(OptiTrack版の実績ランプ) ---
    uint16_t takeoff_ramp_phase1_ticks = 500;   // ランプ前半(電圧3.8V仮定の上限)
    uint16_t takeoff_ramp_total_ticks = 1000;   // ランプ全体のtick数
    float takeoff_ramp_divisor = 1000.0f;       // Thrust0 = tick / divisor
    float takeoff_ramp_phase1_v = 3.8f;         // ランプ前半のトリム計算用電圧 [V]
    float takeoff_complete_margin_m = 0.05f;    // alt_ref - margin 到達でHOVERへ

    // --- トリムデューティ(電圧→ホバリングデューティの一次近似) ---
    // 2026-07-17 ログ解析: z_dot PID の積分が常時 +7.3%(+0.054duty)分を負担して
    // おり、切片が現機体の実ホバー推力を過小評価していたため +0.054 再校正。
    // これで ±15% スラストクランプ帯の中心が実ホバー点に一致し上下対称になる。
    float trim_duty_slope = -0.2448f;
    float trim_duty_intercept = 1.643f;     // 製品版値: 1.5892
    // Thrust0(トリムFF)計算に使う電圧の1次LPF時定数 [s]。
    // 負荷電流による電圧サグ(dV/dduty≈-0.7V/duty)が Thrust0 経由で
    // dT0/dduty≈+0.18 の正帰還を作り 0.45Hz 上下ボビングの減衰を削るため、
    // 秒スケールで遮断する(低電圧フェイルセーフ判定は生値のまま)
    float trim_voltage_filter_tc_s = 1.5f;

    // --- 高度制御 ---
    float thrust_cmd_max_ratio = 1.15f;     // スラスト指令の Thrust0 比上限
    float thrust_cmd_min_ratio = 0.85f;     // スラスト指令の Thrust0 比下限
    float range_loss_thrust_step = 0.02f;   // 高度センサ喪失時の降下ステップ
    uint8_t range0_flag_max = 20;           // Range0flag の飽和値

    // --- ToF チルト補正(スラント距離→鉛直高度。eco版 ESKF updateToF と同方針) ---
    bool tof_tilt_comp_enabled = true;      // 補正の有効/無効(※飛行検証待ち)
    float tof_tilt_gate_rad = 0.70f;        // チルト角 √(φ²+θ²) がこれ超で新規測距を棄却(直前値保持)

    // --- 着陸シーケンス ---
    float landing_z_dot_ref = -0.15f;       // 着陸時の降下速度指令 [m/s]
    float landing_decay_slow = 0.9999f;     // 降下中のスラスト減衰率/tick
    float landing_decay_fast = 0.999f;      // 接地間際のスラスト減衰率/tick
    float landing_near_ground_m = 0.15f;    // 接地間際とみなす高度 [m]
    float landed_threshold_m = 0.1f;        // 着陸完了とみなす高度 [m]

    // --- フェイルセーフ(PROTOCOL.md 規範) ---
    uint32_t link_level_hold_ms = 200;      // setpoint途絶: 水平保持(=setpoint_fresh閾値)
    uint32_t link_loss_landing_ms = 500;    // setpoint途絶: 自動着陸(reason=8)
    float low_voltage_threshold_v = 3.34f;  // 低電圧閾値 [V]
    uint8_t low_voltage_count = 5;          // 低電圧判定の連続サンプル数。電圧読みは
                                            // INA3221 の 20Hz 電流読み(bus_voltage)に
                                            // 一本化したため、旧 400Hz×100tick(0.25s)を
                                            // 20Hz×5サンプル(0.25s)に置換(意味を保存)
    uint32_t max_flight_time_ms = 120000;   // 最大飛行時間 120s(reason=3)
    float reset_accept_alt_m = 0.15f;       // CMD_RESET 受理高度(COMPLETE時) [m]
    float over_g_threshold_g = 2.0f;        // OverG判定 [g](reason=7)

    // --- キャリブレーション・待機 ---
    uint16_t gyro_calib_samples = 800;      // ジャイロオフセット平均サンプル数
    uint32_t wait_settle_ms = 3000;         // WAITでの静定時間 → AHRSリセット
    uint8_t ahrs_reset_repeat = 20;         // 静定後のAHRSリセット回数

    // --- モーターテスト(MOTOR_TEST 状態。ベンチ実験専用: 機体固定前提) ---
    // yaw側 motor.cpp / config.hpp の実績値を踏襲。ソフトスタートは1Sバッテリの
    // 突入電流ブラウンアウト対策なので削除しないこと。
    float motor_test_max_duty = 1.0f;        // テストdutyクランプ上限(yaw側 MOTOR_TEST_MAX_DUTY)
    uint32_t motor_test_failsafe_ms = 1500;  // CMD_MOTOR_RUN 途絶で自動停止 [ms]
    float motor_test_slew_duty_per_s = 2.0f; // ソフトスタートのスルーレート [duty/s]

    // --- 通信 ---
    uint8_t wifi_channel = 1;               // ESP-NOWチャネル(PC側機体プロファイルと一致させる)

    // --- テレメトリ・表示レート(400Hzループの分周比) ---
    uint16_t telemetry_state_divider = 16;   // TLM_STATE 25Hz
    uint16_t telemetry_exp_phase_ticks = 8;  // TLM_EXP の位相オフセット(TLM_STATEと8tickずらす)
    uint16_t telemetry_ctrl_phase_ticks = 4; // TLM_CTRL の位相オフセット(TLM_STATEと4tickずらす)
    uint32_t event_resend_ms = 500;         // TLM_EVENT 2Hz 定期再送
    uint16_t led_show_divider = 16;         // LED更新 25Hz
    uint16_t led_blink_period_ticks = 200;  // 点滅の半周期(0.5s @400Hz)
    uint16_t led_cycle_step_ticks = 21;     // WAITイルミネーションの色送り間隔
    uint32_t led_recording_failsafe_ms = 3000;  // CMD_LED_MODE(1) 途絶で AUTO 復帰 [ms]

    // --- 起動 ---
    uint32_t boot_serial_wait_ms = 1500;    // USBシリアル安定待ち [ms]
};

// 実効コンフィグ(コンパイル時定数)
inline constexpr FlightConfig FLIGHT_CONFIG{};

// ---------------------------------------------------------------------------
// ハードウェア固定値(StampFly ESP32-S3)
// ---------------------------------------------------------------------------

// モータPWM。GPIO5 はモータ(FrontLeft)であり、ブザー等から絶対に触らないこと。
constexpr int PIN_MOTOR_FRONT_LEFT = 5;
constexpr int PIN_MOTOR_FRONT_RIGHT = 42;
constexpr int PIN_MOTOR_REAR_LEFT = 10;
constexpr int PIN_MOTOR_REAR_RIGHT = 41;
constexpr int MOTOR_PWM_FREQ_HZ = 150000;
constexpr int MOTOR_PWM_RESOLUTION_BITS = 8;
constexpr int MOTOR_PWM_MAX_COUNT = (1 << MOTOR_PWM_RESOLUTION_BITS) - 1;
constexpr int LEDC_CH_MOTOR_FRONT_LEFT = 0;
constexpr int LEDC_CH_MOTOR_FRONT_RIGHT = 1;
constexpr int LEDC_CH_MOTOR_REAR_LEFT = 2;
constexpr int LEDC_CH_MOTOR_REAR_RIGHT = 3;

// LED(WS2812: 機体上面2連 + StampS3本体1)
constexpr int PIN_LED_ONBOARD = 39;
constexpr int PIN_LED_ESP = 21;
constexpr int NUM_ONBOARD_LEDS = 2;
constexpr uint8_t LED_BRIGHTNESS = 15;

// ブザー(製品版バグ修正: ピンはGPIO40。GPIO5はモータなので絶対に使わない)
constexpr int PIN_BUZZER = 40;
constexpr int LEDC_CH_BUZZER = 5;  // モータ(ch0-3)とLEDCタイマを共有しないチャネル
constexpr int BUZZER_PWM_RESOLUTION_BITS = 8;
constexpr int BUZZER_BASE_FREQ_HZ = 4000;

// ---------------------------------------------------------------------------
// 流用層(sensor.cpp)互換の定数名。値の正はあくまで FLIGHT_CONFIG。
// ---------------------------------------------------------------------------
inline constexpr float POWER_LIMIT = FLIGHT_CONFIG.low_voltage_threshold_v;
inline constexpr uint8_t UNDER_VOLTAGE_COUNT = FLIGHT_CONFIG.low_voltage_count;
inline constexpr float ALT_LIMIT = FLIGHT_CONFIG.alt_sensor_limit_m;
inline constexpr uint8_t RANGE0_FLAG_MAX = FLIGHT_CONFIG.range0_flag_max;
inline constexpr float OVER_G_THRESHOLD = FLIGHT_CONFIG.over_g_threshold_g;
