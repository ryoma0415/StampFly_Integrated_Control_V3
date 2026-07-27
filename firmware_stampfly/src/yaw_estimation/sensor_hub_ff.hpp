// ===========================================================================
// sensor_hub_ff.hpp — ヨー推定・電流FF補正の統合フック(yaw側 sensor_hub.cpp
// の FF補正挿入点+アイドルアンカー+3系統推定器駆動ロジックを抽出移植)
//
// ベースの sensor.cpp が 400Hz の sensor_read() から sensorHubFfUpdate() を
// 呼ぶ。IMU/AHRS はベース側(Madgwick)をそのまま使い、本モジュールは
//   - 20Hz スロットで updateCurrent() → updateMagnetometer() の順の読み取り
//     (同tick内順序契約: FF補正が同tickの電流を参照する)
//   - ToF 読みが発生した tick では磁気/電流読みをスキップ(位相スタガ、
//     次 tick へ繰延べ)
//   - 磁気パス: RHALL補償 → 軸変換 → mag3D → (FF補正) → EMA
//   - 推定器3系統: リファレンスCF / 補正CF / 4状態EKF
//     (EKF: predict は毎tick dt実測、update は fresh 磁気サンプルのみ)
//   - アイドルアンカー: モーター完全停止中 20Hz×2s 窓 → 始動遷移で凍結
// を担う。電圧はINA3221 CH2 の 20Hz 読み(bus_voltage)に一本化され、
// ベース sensor.cpp が current_sample から取得する(毎tick getVoltage 廃止)。
//
// 数式・符号・定数値は yaw側と同一(V2契約 §2.1)。コマンド処理(CMD_MAG3D_SET
// 等)・テレメトリ配線は次ステージが g_yaw_est と公開APIを使って行う。
// ===========================================================================
#ifndef STAMPFLY_YAW_ESTIMATION_SENSOR_HUB_FF_HPP
#define STAMPFLY_YAW_ESTIMATION_SENSOR_HUB_FF_HPP

#include <Arduino.h>
#include <Wire.h>

#include "accel_calibration.hpp"
#include "bmm150_driver.hpp"
#include "current_sensor.hpp"
#include "ff_calibration.hpp"
#include "mag_calibration.hpp"
#include "yaw_estimator.hpp"
#include "yaw_estimator_kf.hpp"
#include "yaw_estimator_kf2.hpp"

// 1tick分の磁気フレーム(本モジュールが唯一のライター)
struct YawFfMagFrame {
    MagVector mag_raw_body;            // RHALL補償+軸変換後・mag3D前 [µT]
    MagVector mag_cal_body;            // mag3D適用直後の b_cal(アンカー窓・FF補正の入力)
    MagVector mag_filtered_body;       // 非補正系EMA出力(リファレンスCF入力)
    MagVector mag_corr_filtered_body;  // b_corr = b_cal − ΔB̂ − Δb_magbias の補正系EMA出力
    float yaw_mag_level_rad = 0.0f;    // 非補正EMA磁場の水平ヨー [rad]
    // 補正系EMAのレベル化水平成分(=EKF観測 zx/zy。fresh 磁気 tick でその時の
    // 姿勢で更新。TLM_STATE mag_lev_x/y_ut。MAG_AUTOTUNE_DESIGN.md §1.1/§2.2)
    float mag_lev_x_ut = 0.0f;
    float mag_lev_y_ut = 0.0f;
    float mag_dt_s = 0.0f;             // fresh サンプル間隔 [s](今tickが fresh のときのみ有効)
    bool mag_sample_fresh = false;     // 今tickで fresh な磁気サンプルを得たか
    uint32_t last_mag_fresh_ms = 0;    // 直近 fresh サンプル時刻 [ms](鮮度判定用)
};

// 電流FF補正・補正Yaw推定のランタイム状態(yaw側 FfState と同一構成)。
// 本モジュールが毎tick更新し、telemetry/command(次ステージ)が読む。
struct FfState {
    uint8_t ff_mode = 0;   // 0=補正off, 1=方式A, 2=方式B (NVS永続)
    uint8_t est_mode = 0;  // 0=相補フィルタ, 1=EKF, 2=EKF2 (NVS永続)

    // アイドルアンカー: モーター始動遷移で凍結される基準
    bool anchor_valid = false;
    MagVector anchor_b0;         // 停止中2s平均の b_cal [µT]
    float anchor_b0h_x = 0.0f;   // levelMag(B0)_xy (レベル座標)
    float anchor_b0h_y = 0.0f;
    float anchor_psi0 = 0.0f;    // アンカー時のリファレンス推定器 yaw [rad]
    float anchor_i_idle = 0.0f;  // 停止中2s平均の I_total [A]

    // 直近tickの適用値 (テレメトリ用)
    MagVector delta_b;           // 適用中の ΔB̂ (off時は0)
    float sigma_ff_uT = 0.0f;
    float sigma_slew_uT = 0.0f;
    float sigma_diff_uT = 0.0f;
    float yaw_active_rad = 0.0f;  // アクティブ推定器の出力 (off時=リファレンスCF)
};

// sensorHubFfUpdate() への 1tick 分の入力(ベース sensor.cpp が組み立てる)
struct SensorHubFfInputs {
    float yaw_rate_rad_s = 0.0f;   // ω_z = −gyro_z − 起動offset(既存規約のまま渡す)
    float roll_rate_rad_s = 0.0f;  // p = ロールレート(gyro_y − 起動offset、未フィルタ)。
                                   // EKF チルト運動学予測(V2改修A)専用の追加入力
    float roll_rad = 0.0f;         // Madgwick ロール [rad](マウントオフセット適用前)
    float pitch_rad = 0.0f;        // Madgwick ピッチ [rad]
    float dt_s = 0.0f;             // 実測tick周期 [s]
    bool tof_read_this_tick = false;  // 今tickでToF読みが発生(→磁気/電流を繰延べ)
    bool in_flight = false;           // 飛行状態(TAKEOFF/HOVER/LANDING)。true の間は
                                      // ブロッキングし得る BMM150 再初期化リトライを
                                      // 保留する(400Hzループの 2.5ms 予算保護)
    // EKF2 飛行状態再アンカー(MAG_AUTOTUNE_DESIGN.md §2.1-3)の条件判定用
    float alt_est_m = 0.0f;           // 高度推定 Altitude2 [m]
    float alt_ref_m = 0.0f;           // 適用中の目標高度 Alt_ref [m]
    bool motors_running = false;      // PWM実出力あり(ランプダウン中も true にすること)
    float duty[4] = {0.0f, 0.0f, 0.0f, 0.0f};  // 実効duty。順序は FL,FR,RL,RR(FF係数の順)
    uint8_t motor_mask = 0x0F;        // bit0=FL,1=FR,2=RL,3=RR
};

// モジュール共有状態(yaw側 AppState のヨー推定部分に相当)。
// 本モジュールがライター、telemetry/command(次ステージ)がリーダー/ミューテーター。
struct YawEstimationState {
    Bmm150Driver bmm150;
    MagSoftIronCalibration mag3d_calibration;
    AccelSixFaceCalibration accel_calibration;  // NVS復元のみ(飛行AHRSへは未適用)
    ExpMagFilter mag_filter;            // 非補正系EMA
    ExpMagFilter mag_filter_corr;       // 補正系EMA(非補正系と完全分離)
    YawEstimator yaw_estimator;         // リファレンスCF(非補正mag、既存挙動不変)
    YawEstimator yaw_estimator_corr;    // 補正CF(アンカー/モード切替で再シード)
    YawEstimatorKf yaw_kf;              // 4状態EKF(補正mag)
    YawEstimatorKf2 yaw_kf2;            // EKF2(ヨー擬似観測付き。常時シャドー実行。
                                        // MAG_AUTOTUNE_DESIGN.md §2.2)
    FfCalibration ff_calibration;
    FfState ff;
    // 学習ハードアイアン残差 Δb(NVS "magbias"。b_cal 空間 [µT]。補正系のみに
    // 適用: b_corr = b_cal − ΔB̂ − Δb_magbias。無効時は0ベクトルを保証する。
    // MAG_AUTOTUNE_DESIGN.md §2.2/§2.5)
    bool magbias_valid = false;
    MagVector magbias;
    YawFfMagFrame frame;
    CurrentSample current_sample;       // 直近の 20Hz 電流/電圧サンプル
    bool current_slot_fired = false;    // 今tickで 20Hz 電流スロットが発火したか
                                        // (ベース sensor.cpp の電圧取り込み・低電圧判定用)

    bool bmm_ready = false;
    bool current_ready = false;

    // 姿勢マウントオフセット(NVS復元。磁気レベル化の入力にのみ適用)
    bool attitude_mount_valid = false;
    float roll_mount_offset_rad = 0.0f;
    float pitch_mount_offset_rad = 0.0f;

    // 直近 update の入力スナップショット(手動アンカー・FF計算が参照)
    bool motors_running = false;
    float duty[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    uint8_t motor_mask = 0x0F;

    // yaw_zero コマンド用(yaw側踏襲): 磁気オフセット捕捉後にNVS保存を行う
    // ための保留フラグ(次ステージのコマンド処理が消費する)。
    bool yaw_zero_save_pending = false;
};

extern YawEstimationState g_yaw_est;

// 初期化: BMM150 / INA3221 を(begin済みの)I2Cバス上で初期化し、NVSから
// mag3d → accel6 → attmount → geomag → yawzero → ffcal → magbias の順で
// 復元する(MAG_AUTOTUNE_DESIGN.md §2.5 のブートロード順。flowcal はフロー
// 移植 §2.4 側が追加する)。ベースの sensor_init() から呼ぶ。
void sensorHubFfInit(TwoWire& wire);

// 400Hz tick: ベース sensor.cpp の Yaw_rate 確定後に毎tick呼ぶ。
// 内部で 20Hz スロット(電流→磁気)・推定器3系統・アンカーサービスを回す。
void sensorHubFfUpdate(const SensorHubFfInputs& in);

// 手動アンカー再取得(CMD_FF_ANCHOR 用)。モーター回転中(直近updateの
// motors_running=true)、または停止中2s窓がまだ満ちていない場合は false。
bool sensorHubFfAnchorNow();

// ffmode 切替 / FF_COMMIT 後に呼ぶ: 補正CFへリファレンスCFの状態をコピーし、
// 補正系EMA・ノルム基準を再初期化、EKF の ψ をリファレンス yaw で再シードする。
void sensorHubFfReseed();

// FF補正が実際に有効か: ff_mode!=0 かつ係数確定済み、かつ電流サンプルが有効。
bool sensorHubFfCorrectionActive();

// mag3d 変更(CMD_MAG3D_SET / クリア)成功時に呼ぶ: 旧 b_cal 空間のアンカーを
// 破棄し、窓を貯め直し、補正系推定器を再シード、ff_mode を安全側の 0 に落として
// NVS 保存する(係数blobは残る)。
void sensorHubFfOnMag3dChange();

// EKF が角度制御のヨーソースとして健全か(契約 §2.3 の縮退判定用):
// FF補正が有効 かつ アンカー有効 かつ 磁気更新が凍結されていない。
bool sensorHubFfEkfHealthy();

// fresh 磁気サンプルの鮮度(ff_status bit6 用): 直近 fresh から
// YAW_MAG_FRESH_TIMEOUT_MS 以内なら true。
bool sensorHubFfMagFresh(uint32_t now_ms);

// ---- EKF2 / magbias(MAG_AUTOTUNE_DESIGN.md §2.1-§2.3, §2.5) ----

// 外部ヨー基準(CMD_POS_ERR bit3 有効時の mocap_yaw)を1件ステージする。
// flight_control の apply_pos_err が受信 tick に呼び、次の sensorHubFfUpdate が
// consume-once で取り出し、age < FF_EKF2_YAW_OBS_MAX_AGE_MS のときのみ
// EKF2 のヨー擬似観測(updateYawObs)へ回す(契約 §2.1-1 / §2.3)。
// low_trust は flags bit4(FLAG_YAW_REF_LOW_TRUST)。rx_ms は受信時刻 millis()。
void sensorHubFfStageYawObs(float psi_meas_rad, bool low_trust, uint32_t rx_ms);

// magbias の適用/クリア(CMD_MAGBIAS_SET。契約 §1.4): Δb を即適用し、
// mag3d 変更時と同パターンでアンカー無効化+窓リセット+補正系再シードを行う。
// **ff_mode は降格しない**(FF 係数は b_cal 空間のまま有効)。NVS 保存
// (saveMagbias)は呼び出し側の責務。
void sensorHubFfApplyMagbias(bool valid, const MagVector& delta_b_ut);

// EKF2 が角度制御のヨーソースとして健全か(healthy2。契約 §2.2):
// 判定は sensorHubFfEkfHealthy() と同等(FF有効 ∧ アンカー有効 ∧ 非凍結)。
// yaw_obs 鮮度は含めない(MoCap 途絶でヨー角制御を落とさない — コースト)。
bool sensorHubFfEkf2Healthy();

// TLM_STATE ekf2_status バイトの合成(契約 §1.1 のビット定義)。
uint8_t sensorHubFfEkf2Status(uint32_t now_ms);

#endif
