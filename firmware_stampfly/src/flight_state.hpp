// ===========================================================================
// flight_state.hpp — 飛行制御の共有状態構造体
//
// OptiTrack版 flight_state.hpp の構造化パターンを踏襲しつつ、ジョイスティック
// 時代の死フィールドを除去したもの。SensorState は流用層(sensor.cpp)が
// そのまま書き込むため、フィールド名・volatile指定を原典どおり維持する。
// ===========================================================================
#pragma once

#include <stdint.h>

#include "alt_kalman.hpp"
#include "stampfly_protocol.hpp"

// 飛行状態。数値は PROTOCOL.md の FlightState enum と1対1対応で、
// 流用層(sensor.cpp)が比較演算で使うため plain enum とする。
enum AutoFlightState : uint8_t {
    AUTO_INIT = 0,
    AUTO_CALIBRATION = 1,
    AUTO_WAIT = 2,
    AUTO_TAKEOFF = 3,
    AUTO_HOVER = 4,
    AUTO_LANDING = 5,
    AUTO_COMPLETE = 6,
    AUTO_MOTOR_TEST = 7,  // v2: モーターテスト(WAIT⇔MOTOR_TEST、CMD_MODEで遷移)
};

// PROTOCOL.md FlightState との対応をコンパイル時に固定する
static_assert(AUTO_INIT == static_cast<uint8_t>(stampfly::FlightState::INIT), "enum mismatch");
static_assert(AUTO_CALIBRATION == static_cast<uint8_t>(stampfly::FlightState::CALIBRATION), "enum mismatch");
static_assert(AUTO_WAIT == static_cast<uint8_t>(stampfly::FlightState::WAIT), "enum mismatch");
static_assert(AUTO_TAKEOFF == static_cast<uint8_t>(stampfly::FlightState::TAKEOFF), "enum mismatch");
static_assert(AUTO_HOVER == static_cast<uint8_t>(stampfly::FlightState::HOVER), "enum mismatch");
static_assert(AUTO_LANDING == static_cast<uint8_t>(stampfly::FlightState::LANDING), "enum mismatch");
static_assert(AUTO_COMPLETE == static_cast<uint8_t>(stampfly::FlightState::COMPLETE), "enum mismatch");
static_assert(AUTO_MOTOR_TEST == static_cast<uint8_t>(stampfly::FlightState::MOTOR_TEST), "enum mismatch");

// ループタイミング。Loop_flag のみISR(400Hzタイマ)が書くため volatile。
struct FlightTimingState {
    volatile uint8_t Loop_flag = 0;   // タイマISRが1にし、ループ先頭で0に戻す
    float Control_period = 0.0f;      // 設計制御周期 [s](init_copterでconfigから設定)
    float Elapsed_time = 0.0f;        // 計測開始(WAIT遷移)からの経過 [s]
    float Old_Elapsed_time = 0.0f;
    float Interval_time = 0.0f;       // 直近の実測制御周期 [s]
    uint32_t S_time = 0;              // 計測開始時刻 [µs]
    uint32_t E_time = 0;              // 今tickの時刻 [µs]
};

// 状態機械の状態
struct FlightModeState {
    AutoFlightState auto_state = AUTO_INIT;
    uint8_t last_reason = 0;        // 直近の遷移理由(stampfly::Reason)
    uint32_t flight_start_ms = 0;   // TAKEOFF開始時刻(最大飛行時間の起点)[ms]
    uint32_t phase_start_ms = 0;    // 現在フェーズの開始時刻 [ms]
    uint8_t Alt_flag = 0;           // 高度制御有効(流用層が高度上限検査で参照)
};

// 制御出力(400Hzループ内のみで読み書きする)
struct ControlOutputState {
    float FrontRight_motor_duty = 0.0f;
    float FrontLeft_motor_duty = 0.0f;
    float RearRight_motor_duty = 0.0f;
    float RearLeft_motor_duty = 0.0f;
    float Roll_rate_reference = 0.0f;    // 角度PID出力 = 角速度目標 [rad/s]
    float Pitch_rate_reference = 0.0f;
    float Yaw_rate_reference = 0.0f;     // ヨー角PID出力 = ヨーレート目標(v2。制御off時0)
    float Roll_angle_reference = 0.0f;   // クランプ後の角度目標 [rad]
    float Pitch_angle_reference = 0.0f;
    float Roll_rate_command = 0.0f;      // 角速度PID出力(トルク相当)
    float Pitch_rate_command = 0.0f;
    float Yaw_rate_command = 0.0f;
    float Roll_angle_command = 0.0f;     // 適用中の姿勢指令 [rad](水平保持含む)
    float Pitch_angle_command = 0.0f;
    float Roll_angle_offset = 0.0f;      // 角度誤差計算用オフセット(流用)
    float Pitch_angle_offset = 0.0f;
    float Thrust_command = 0.0f;         // スラスト指令(電圧スケール)
    float Thrust0 = 0.0f;                // 正規化ベーススラスト(0-1)
    float Alt_ref = 0.0f;                // 目標高度 [m](CMD_SETPOINTでクランプ更新)
    float Z_dot_ref = 0.0f;              // 高度PID出力 = 上下速度目標 [m/s]
    // --- v2: ヨー角制御 ---
    float Yaw_angle_command = 0.0f;      // 適用中ヨー角目標 [rad](途絶ラッチ後含む)
    uint8_t Yaw_ctrl_active = 0;         // ヨー角度制御が実際に効いているか(ff_status bit5)
};

// XY 指令ソース(最後に受信したストリームが姿勢指令の出どころを決める)
enum XyCommandSource : uint8_t {
    XY_SOURCE_ATTITUDE = 0,  // CMD_SETPOINT: PC 計算の roll/pitch 角度指令
    XY_SOURCE_POS_ERR = 1,   // CMD_POS_ERR: 位置誤差 → 機上XY PID(ヨー回転補償)
};

// CMD_SETPOINT / CMD_POS_ERR の適用追跡(seq_echo・リンク鮮度の根拠)。
// last_setpoint_ms / setpoint_received / applied_setpoint_seq は両ストリーム
// 共有のハートビート(既存フェイルセーフ: >200ms 水平保持 / >500ms 自動着陸)。
struct CommandTrackingState {
    bool setpoint_received = false;     // 一度でも有効なsetpoint/pos_errを受けたか
    uint32_t applied_setpoint_seq = 0;  // 最後に適用したフレームの seq(未受信なら0)
    uint32_t last_setpoint_ms = 0;      // 最後の受信時刻 [ms]
    float target_roll_rad = 0.0f;       // 受信した姿勢目標 [rad](XY_SOURCE_ATTITUDE)
    float target_pitch_rad = 0.0f;
    // --- v2: CMD_SETPOINT 17B(yaw_ref / flags bit1) ---
    float target_yaw_rad = 0.0f;        // 受信したヨー角目標 [rad](target_yaw_valid時のみ有効)
    bool target_yaw_valid = false;      // flags bit1 = ヨー角制御ON
    // --- v2.1: CMD_POS_ERR(機上XY位置制御) ---
    XyCommandSource xy_source = XY_SOURCE_ATTITUDE;  // 最後に受信したストリーム
    float pos_err_x_m = 0.0f;           // 受信した位置誤差(制御座標系)[m]
    float pos_err_y_m = 0.0f;
    bool pos_err_xy_valid = false;      // flags bit2(MoCap 新鮮+閉ループ有効)
    float pos_mocap_yaw_rad = 0.0f;     // 受信した外部ヨー基準 [rad](MoCap実測 or
                                        // 移動ベース。EKF2 のヨー擬似観測へ配線)
    bool pos_mocap_yaw_valid = false;   // flags bit3
    bool pos_mocap_yaw_low_trust = false;  // flags bit4(基準ヨー低信頼 → EKF2 の
                                           // R_ψ 低信頼プリセット。契約 §1.2)
    uint32_t last_pos_err_ms = 0;       // 最後の pos_err 受信時刻 [ms]
    uint32_t prev_pos_err_ms = 0;       // 1つ前の受信時刻(XY PID の dt 根拠)[ms]
    bool pos_err_fresh_sample = false;  // 未処理の新サンプルがあるか(XY PID が消費)
};

// v2: 制御ループ診断(TLM_CTRL 用)。400Hzループが毎tick PID アクセサから
// 転記し、telemetry が 25Hz で読む(指令角速度と yaw_ctrl_active は output、
// flying は mode から読む)。PID リセット中は各成分が 0(契約 §TLM_CTRL)。
struct ControlDiagState {
    // 角度ループPID成分: roll_p,i,d, pitch_p,i,d, yaw_p,i,d(yaw はクランプ前)
    float pid_ang[9] = {};
    // 角速度ループPID成分: 同順(roll=p_pid, pitch=q_pid, yaw=r_pid)
    float pid_rate[9] = {};
    // CMD_POS_ERR 経路で機上XY指令生成中(TLM_CTRL flags bit0)
    uint8_t xy_onboard_active = 0;
};

struct FlightControlState {
    FlightTimingState timing;
    FlightModeState mode;
    ControlOutputState output;
    CommandTrackingState command;
    ControlDiagState diag;
};

// センサ状態(流用層 sensor.cpp が書き込む。原典 flight_state.hpp と同一)
struct SensorState {
    volatile float Roll_angle = 0.0f;
    volatile float Pitch_angle = 0.0f;
    volatile float Yaw_angle = 0.0f;
    volatile float Roll_rate = 0.0f;
    volatile float Pitch_rate = 0.0f;
    volatile float Yaw_rate = 0.0f;
    volatile float Accel_x_raw = 0.0f;
    volatile float Accel_y_raw = 0.0f;
    volatile float Accel_z_raw = 0.0f;
    volatile float Accel_x = 0.0f;
    volatile float Accel_y = 0.0f;
    volatile float Accel_z = 0.0f;
    volatile float Accel_z_d = 0.0f;
    volatile int16_t RawRange = 0;
    volatile int16_t Range = 0;
    volatile int16_t RawRangeFront = 0;
    volatile int16_t RangeFront = 0;
    volatile float Altitude = 0.0f;
    volatile float Altitude2 = 0.0f;
    volatile float Alt_velocity = 0.0f;
    volatile float Voltage = 0.0f;
    float Acc_norm = 0.0f;
    float Over_g = 0.0f;
    float Over_rate = 0.0f;
    uint8_t OverG_flag = 0;
    uint8_t Range0flag = 0;
    volatile uint8_t Under_voltage_flag = 0;
    volatile float Az = 0.0f;
    volatile float Az_bias = 0.0f;
    Alt_kalman EstimatedAltitude;

    // ---- v2: ヨー推定・電流計測の公開値(sensor.cpp が yaw_estimation/ の
    //      出力から毎tick転記し、telemetry(TLM_STATE 97-134)と飛行制御の
    //      ヨー角制御(次ステージ)が読む) ----
    volatile float Yaw_gyro_integral = 0.0f;  // Z軸角速度の単純積算 [rad](400Hz、ahrs_resetでゼロクリア)
    volatile float Yaw_est_rad = 0.0f;        // アクティブ推定器ヨー [rad](est_mode=2:EKF2 ψ / 1:EKF ψ / 0:補正CF、FF無効時はリファレンスCF)
    volatile float Yaw_ekf_rad = 0.0f;        // 4状態EKF の ψ [rad]
    volatile float Current_a = 0.0f;          // INA3221 CH2 総電流 [A](20Hz更新)
    volatile float Db_hat_x_ut = 0.0f;        // FF補正ベクトル ΔB̂ x [µT]
    volatile float Db_hat_y_ut = 0.0f;        // 同 y [µT]
    volatile float Bm_x_ut = 0.0f;            // EKF 磁気バイアス状態 b_m x [µT]
    volatile float Bm_y_ut = 0.0f;            // 同 y [µT]
    volatile float Ekf_nis = 0.0f;            // 直近 EKF 更新の NIS
    volatile uint8_t Ekf_ffg = 0;             // EKF ゲート/健全性ビット(yaw側 ffg 定義)
    volatile uint8_t Ff_mode = 0;             // FF補正モード(0=off,1=方式A,2=方式B)
    volatile uint8_t Est_mode = 0;            // 推定器選択(0=相補フィルタ,1=EKF,2=EKF2)
    volatile uint8_t Ff_anchor_valid = 0;     // アイドルアンカー有効(0/1)
    volatile uint8_t Ff_cal_loaded = 0;       // FF係数確定済み(0/1)
    volatile uint8_t Mag_fresh = 0;           // fresh磁気が鮮度タイムアウト内(0/1)

    // ---- 磁気オートチューン拡張(TLM_STATE 135-172。MAG_AUTOTUNE_DESIGN.md
    //      §1.1。sensor.cpp が yaw_estimation/ の出力から毎tick転記する) ----
    volatile float Mag_cal_x_ut = 0.0f;       // b_cal(mag3d後・FF補正前)機体系 x [µT]
    volatile float Mag_cal_y_ut = 0.0f;       // 同 y [µT]
    volatile float Mag_cal_z_ut = 0.0f;       // 同 z [µT]
    volatile float Mag_lev_x_ut = 0.0f;       // FF補正+magbias+EMA後レベル化水平 x(=EKF観測 zx)[µT]
    volatile float Mag_lev_y_ut = 0.0f;       // 同 y(=zy)[µT]
    volatile float Ekf2_yaw_rad = 0.0f;       // EKF2 の ψ [rad](シャドー中も常時)
    volatile float Ekf2_bm_x_ut = 0.0f;       // EKF2 の b_m x [µT]
    volatile float Ekf2_bm_y_ut = 0.0f;       // 同 y [µT]
    volatile float Ekf2_yaw_innov_rad = 0.0f; // 直近ヨー観測イノベーション [rad](未受信時0)
    volatile uint8_t Ekf2_status = 0;         // EKF2_STATUS_* ビット(契約 §1.1)
    volatile uint8_t Ekf2_gate = 0;           // EKF2 のゲートビット(ffg と同一定義)

    // ---- オプティカルフロー拡張(TLM_STATE 173-183。MAG_AUTOTUNE_DESIGN.md
    //      §1.1/§2.4。sensor.cpp が flow/ の出力から毎tick転記する) ----
    volatile float Flow_vx_mps = 0.0f;        // フロー機体系 x 速度 [m/s](無効時0)
    volatile float Flow_vy_mps = 0.0f;        // 同 y [m/s]
    volatile uint8_t Flow_squal = 0;          // SQUAL 生値
    volatile uint8_t Flow_status = 0;         // FLOW_STATUS_* ビット(契約 §1.1)
    volatile uint8_t Flow_dt_ms = 0;          // フロー読み実測 dt [ms](0-255クランプ)
};
