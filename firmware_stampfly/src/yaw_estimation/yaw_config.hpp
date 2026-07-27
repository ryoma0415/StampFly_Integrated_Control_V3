// ===========================================================================
// yaw_config.hpp — ヨー推定モジュール(yaw_estimation/)の定数集約
//
// 旧 Yaw_Estimation_Project(yaw側)の firmware/src/config.hpp から FF_* / FF_EKF_* /
// MAG_BODY_* / YAW_* 等の定数を分離移植したもの(移植元フォルダは 2026-07-27 に
// 削除済み。本ファイルが定数の正典)。数値は yaw側と完全同一
// (V2契約 §2.1: 数式・符号・定数値は一切変更しない)。UDP/Wi-Fi 系の定数は
// 持ち込まない。値の変更はベンチ再検証が前提。
// ===========================================================================
#pragma once

#include <Arduino.h>

#include "../config.hpp"

// ---------------------------------------------------------------------------
// 周期・スケジューリング
// ---------------------------------------------------------------------------

// 400Hz センサーtick周期 [µs]。ベースの制御ループ周期と同一値
// (yaw側 SENSOR_PERIOD_US=2500 と一致することを値で保証する)。
static constexpr uint32_t SENSOR_PERIOD_US = FLIGHT_CONFIG.loop_period_us;
static_assert(SENSOR_PERIOD_US == 2500, "yaw側の400Hz前提(EKF定数のチューニング条件)");

// 磁気/電流の低速スロット周期 [ms](20Hz)。yaw側では TELEMETRY_PERIOD_MS の
// 名前で磁気・電流読みのゲートに使われていた値(50ms)を役割名で再定義。
static constexpr uint32_t YAW_SLOW_SLOT_PERIOD_MS = 50;

// fresh 磁気サンプルの鮮度タイムアウト [ms]。BMM150 の実効 fresh レートは
// ~10Hz(100ms)なので、3サンプル連続欠落で mag_fresh(ff_status bit6)を
// 落とす(V2追加: テレメトリの健全性表示用で推定の数式には関与しない)。
static constexpr uint32_t YAW_MAG_FRESH_TIMEOUT_MS = 300;

// ---------------------------------------------------------------------------
// BMM150 磁気センサ
// ---------------------------------------------------------------------------

static const uint8_t BMM150_I2C_ADDRESS = 0x10;
static const uint8_t BMM150_EXPECTED_CHIP_ID = 0x32;
static const float MAG_FILTER_ALPHA = 0.18f;

// BMM150 sensor-axis to aircraft body-axis mapping.
// Body axes are x=front, y=right, z=down. Adjust these after a static axis test.
static const int8_t MAG_BODY_X_SOURCE = 1;
static const int8_t MAG_BODY_Y_SOURCE = 0;
static const int8_t MAG_BODY_Z_SOURCE = 2;
static const float MAG_BODY_X_SIGN = 1.0f;
static const float MAG_BODY_Y_SIGN = 1.0f;
static const float MAG_BODY_Z_SIGN = -1.0f;

// ---------------------------------------------------------------------------
// INA3221 電流/電圧モニタ(CH2 = バッテリライン、10mΩ シャント)
// ---------------------------------------------------------------------------

static const uint8_t INA3221_I2C_ADDRESS = 0x40;
static const uint32_t INA3221_SHUNT_MILLIOHM = 10;

// ---------------------------------------------------------------------------
// 地磁気リファレンス既定値
// ---------------------------------------------------------------------------

// Heading uses the leveled magnetic X/Y components. Inclination comparison can
// use an independent Z sign so geomagnetic validation matches down-positive
// reference values without changing heading direction.
static const float GEOMAG_INCLINATION_Z_SIGN = -1.0f;

// Default geomagnetic consistency tolerances: used when a geomag_set command omits
// them and as the sanitizer fallback for out-of-range values. Kept here so the
// GeomagneticReference struct defaults and the command fallbacks stay in sync.
static const float GEOMAG_DEFAULT_TOTAL_TOLERANCE_RATIO = 0.35f;
static const float GEOMAG_DEFAULT_HORIZONTAL_TOLERANCE_RATIO = 0.45f;
static const float GEOMAG_DEFAULT_INCLINATION_TOLERANCE_DEG = 20.0f;

// ---------------------------------------------------------------------------
// 相補フィルタ(YawEstimator)のゲイン・ゲート
// ---------------------------------------------------------------------------

static const float YAW_CORRECTION_GAIN_RAD_S = 1.2f;
static const float YAW_MAG_NORM_TOLERANCE = 0.35f;
static const float YAW_MAG_INNOVATION_GATE_RAD = 0.9f;
static const float YAW_RECAPTURE_MIN_HOLD_TIME_S = 1.0f;
static const float YAW_RECAPTURE_STABLE_TIME_S = 0.8f;
static const float YAW_RECAPTURE_YAW_STABILITY_RAD = 5.0f * PI / 180.0f;
static const float YAW_RECAPTURE_NORM_STABILITY_TOLERANCE = 0.12f;
static const float YAW_RECAPTURE_MAX_STEP_RAD = 2.0f * PI / 180.0f;
static const float YAW_RECAPTURE_MAX_YAW_RATE_RAD_S = 20.0f * PI / 180.0f;

// ===== 電流FF較正・補正Yaw推定 (yaw側 ff_pipeline_design.md §4-5) =====

// ffcal_begin / CMD_FF_BEGIN で宣言できる LUT 点数の範囲 (4 <= N <= 24)。
static const uint8_t FF_LUT_MIN_POINTS = 4;
static const uint8_t FF_LUT_MAX_POINTS = 24;

// 方式B差動項: ΣÎ_m がこの値 [A] 未満なら差動項を 0 にする (0 割り回避)。
static const float FF_DIFF_MIN_SUM_CURRENT_A = 0.05f;

// FF 不確かさの自己申告 (yaw_estimation_ff_two_methods.md §2.†):
//   σ_ff = κ_ff·|ΔB̂_xy|, σ_slew = |a_xy|·|dI/dt|·τ_resid,
//   σ_diff = 0.3·|δ_xy|·|δI| (|δ_xy| ≈ 30 µT/A → 係数 0.3×30 = 9.0 µT/A)
static const float FF_KAPPA_FF = 0.03f;
static const float FF_TAU_RESID_S = 0.05f;
static const float FF_SIGMA_DIFF_UT_PER_A = 0.3f * 30.0f;

// アイドルアンカー (§5.3): モーター停止中の b_cal / I_total を 20Hz で
// リングバッファに積み、2s 窓 (40 サンプル) が満ちたら取得可能になる。
static const uint32_t FF_ANCHOR_PERIOD_MS = 50;      // 20 Hz
static const uint8_t FF_ANCHOR_WINDOW_SAMPLES = 40;  // 2 s @ 20 Hz

// ---- 4状態EKF 定数 (§5.5)。内部単位は rad 系に統一。仕様の deg 表記値を
//      DEG_TO_RAD で換算して定義する (換算根拠を右コメントに示す)。----
// q_ψ = 5e-4 deg²/s → rad²/s (×(π/180)² ≈ 3.046e-4) ≈ 1.523e-7
static const float FF_EKF_Q_PSI_RAD2_S = 5.0e-4f * DEG_TO_RAD * DEG_TO_RAD;
// q_bg = 1e-8 (°/s)²/s → (rad/s)²/s ≈ 3.046e-12
static const float FF_EKF_Q_BG_RAD2_S3 = 1.0e-8f * DEG_TO_RAD * DEG_TO_RAD;
// q_bm = 0.02 µT²/s (µT はそのまま内部単位)
static const float FF_EKF_Q_BM_UT2_S = 0.02f;
// τ_bm = 120 s (b_m の Gauss-Markov 回帰時定数)
static const float FF_EKF_TAU_BM_S = 120.0f;
// P0 = diag((10°)², (0.5°/s)², (4µT)², (4µT)²)
static const float FF_EKF_P0_PSI_RAD2 = (10.0f * DEG_TO_RAD) * (10.0f * DEG_TO_RAD);      // ≈ 0.0305 rad²
static const float FF_EKF_P0_BG_RAD2_S2 = (0.5f * DEG_TO_RAD) * (0.5f * DEG_TO_RAD);      // ≈ 7.62e-5 (rad/s)²
static const float FF_EKF_P0_BM_UT2 = 16.0f;                                              // (4 µT)²
// R_base = 4.0 µT² = (2.0 µT)², σ_rz = 3.5 µT (チルト依存 R 膨張)
static const float FF_EKF_R_BASE_UT2 = 4.0f;
static const float FF_EKF_SIGMA_RZ_UT = 3.5f;
// NIS ゲート: χ²₂(95%) = 5.99 で R 膨張、χ²₂(99.9%) = 13.8 で棄却
static const float FF_EKF_NIS_INFLATE = 5.99f;
static const float FF_EKF_NIS_REJECT = 13.8f;
// ノルムゲート: |‖b_corr,filt‖−‖B0‖| が 8-20 µT で R 膨張、>20 µT で棄却
static const float FF_EKF_NORM_GATE_SOFT_UT = 8.0f;
static const float FF_EKF_NORM_GATE_HARD_UT = 20.0f;
// z 残差ゲート: |b_corr,filt.z − B0.z| > 12 µT で棄却
static const float FF_EKF_Z_GATE_UT = 12.0f;
// tilt > 25° で磁気更新スキップ
static const float FF_EKF_TILT_SKIP_RAD = 25.0f * DEG_TO_RAD;
// ‖b_m‖ > 20 µT で磁気更新凍結 (FF モデル破綻、要再アンカー)
static const float FF_EKF_BM_FREEZE_UT = 20.0f;
// |db_m/dt| > 0.3 µT/s が 10 s 継続で警告フラグ
static const float FF_EKF_BM_DRIFT_WARN_UT_S = 0.3f;
static const float FF_EKF_BM_DRIFT_WARN_HOLD_S = 10.0f;
// 連続棄却 > 3 s で P の ψ・b_m 対角を 1.02/s で緩膨張 (P0 の 10 倍上限)
static const float FF_EKF_REJECT_INFLATE_AFTER_S = 3.0f;
static const float FF_EKF_REJECT_INFLATE_RATE_PER_S = 1.02f;
static const float FF_EKF_P_INFLATE_MAX_RATIO = 10.0f;

// ---- V2 改修 (2026-07): チルト運動学予測 / NISロック脱出 / 再アンカー健全化 ----
// (7/10 実験 explog_20260710_203534 の NISロック(t=82.3s〜)・再アンカー時
//  −15° スナップ・ぐるぐる区間の予測誤差急拡大への対策)

// [A] チルト運動学予測: 本ファームの姿勢規約では ψ̇ = ((r−b_g)·cosθ − p·sinθ)/cosφ
// (導出と検証は yaw_estimator_kf.cpp の predict を参照)。cosφ(ロール) が
// この値未満 (|roll| > 60°) では特異点 (cosφ→0) を避けて従来式 ψ̇ = r−b_g に
// フォールバックする。
static const float FF_EKF_TILT_KIN_COS_MIN = 0.5f;  // = cos(60°)
// [A] ψ̇ の数値安全弁: ±720°/s にクランプ (異常姿勢・異常レート入力での暴走防止)
static const float FF_EKF_PSI_DOT_CLAMP_RAD_S = 720.0f * DEG_TO_RAD;

// [B-1] ソフト再捕捉: 最終「通常受理」からこの時間を超えて NIS 棄却が続き、かつ
// 観測が norm/z/tilt ゲートを通過している (=NIS だけで弾かれている) とき、棄却の
// 代わりに制限付き更新 (ffg bit7) で引き込みを再開する。飛行中も有効。
static const float FF_EKF_RECAPTURE_AFTER_S = 5.0f;
// [B-1] 制限付き更新の 1 更新あたり Δψ クランプ (3°/更新。磁気更新は実効 10Hz
// なので最大 30°/s の引き込みレートに相当)
static const float FF_EKF_RECAPTURE_MAX_STEP_RAD = 3.0f * DEG_TO_RAD;

// [B-2] 地上自動再アンカー: モーター停止中・B0 窓 full で、最終受理からこの時間を
// 超えて NIS 棄却が続いていたら ffFreezeAnchorFromWindow() を自動発動する
// (ソフト再捕捉でも戻れない大乖離・磁場環境変化の最終救済)。
static const float FF_EKF_AUTO_REANCHOR_AFTER_S = 10.0f;
// [B-2] 自動再アンカーの連発防止クールダウン
static const float FF_EKF_AUTO_REANCHOR_COOLDOWN_S = 30.0f;

// [C] 再アンカー ψ0 の健全 EKF 判定: 最終受理からこの時間以内 (かつ nis<5.99・
// 非凍結・アンカー有効) なら anchor_psi0 に EKF の現在 ψ を使う。停止直後の
// リファレンスCF は過渡誤差 (リキャプチャ 2°/更新の引き込み遅れ、7/10 実測で
// −15° 前後) を持ち得るため、健全な EKF を優先してスナップを防ぐ。
static const float FF_EKF_ANCHOR_PSI0_FRESH_S = 1.0f;

// ---- EKF2 (est_mode=2、ヨー擬似観測付き。MAG_AUTOTUNE_DESIGN.md §2.1) ----
// 既存 FF_EKF_* は現行EKF(est_mode=1)と共用の無改変値。ここから下は EKF2
// 専用の「追加」定数のみ(V2契約: 既存定数の変更禁止・追加は可)。

// [§2.1-1] ヨー擬似観測の観測ノイズ R_ψ: 通常 (2°)² / 基準ヨー低信頼
// (CMD_POS_ERR flags bit4 = FLAG_YAW_REF_LOW_TRUST) 時 (6°)²
static const float FF_EKF2_R_PSI_RAD2 =
    (2.0f * DEG_TO_RAD) * (2.0f * DEG_TO_RAD);
static const float FF_EKF2_R_PSI_LOW_TRUST_RAD2 =
    (6.0f * DEG_TO_RAD) * (6.0f * DEG_TO_RAD);
// [§2.1-1] ヨー観測イノベーションゲート: |y| > 30° は棄却+連続棄却カウンタ
static const float FF_EKF2_YAW_GATE_RAD = 30.0f * DEG_TO_RAD;
// [§2.1-1] 連続棄却がこの回数 (N≥25) に達したら融合停止ラッチ
// (イノベーション値はテレメトリに残す。解除は reanchor / reseedYaw)
static const uint8_t FF_EKF2_YAW_GATE_STOP_COUNT = 25;
// [§2.1-1] ヨー観測の消費許容 age: CMD_POS_ERR 受信からこの時間以内の
// ステージ分のみ EKF2 へ融合する(consume-once の防御的鮮度検査)
static const uint32_t FF_EKF2_YAW_OBS_MAX_AGE_MS = 100;
// [§2.1-2] τ_bm 適応: yaw_obs 健全 (最終受理 < 1.0s) なら q_bm 現行値の
// ランダムウォーク、喪失 (≥1.0s) なら q_bm×0.1 の準凍結ホールド
static const float FF_EKF2_YAW_OBS_HEALTHY_S = 1.0f;
static const float FF_EKF2_Q_BM_LOST_FACTOR = 0.1f;
// ekf2_status bit0 (yaw_obs_fresh): 直近のヨー観測「受信」からこの時間未満
static const float FF_EKF2_YAW_FRESH_WINDOW_S = 1.0f;
// ekf2_status bit1 (yaw_obs_fused): 直近のヨー観測「受理」からこの時間未満
// (飛行状態再アンカーの yaw_obs_fused 条件もこの窓を使う)
static const float FF_EKF2_YAW_FUSED_WINDOW_S = 0.5f;
// [§2.1-3] 飛行状態再アンカー (1フライト1回) の発動条件:
// flying 遷移後 >5s ∧ |alt_est−alt_ref|<0.1m が 2s 継続 ∧ 電流 >1.0A ∧
// yaw_obs_fused。実行時 P_bm ← (2µT)²(P_ψ は維持 = P0 スナップ窓を開かない)
static const float FF_EKF2_FLIGHT_ANCHOR_MIN_FLY_S = 5.0f;
static const float FF_EKF2_FLIGHT_ANCHOR_ALT_TOL_M = 0.1f;
static const float FF_EKF2_FLIGHT_ANCHOR_ALT_HOLD_S = 2.0f;
static const float FF_EKF2_FLIGHT_ANCHOR_MIN_CURRENT_A = 1.0f;
static const float FF_EKF2_FLIGHT_ANCHOR_P_BM_UT2 = 4.0f;  // (2 µT)²
