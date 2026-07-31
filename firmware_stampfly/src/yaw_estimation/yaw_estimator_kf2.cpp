// ===========================================================================
// yaw_estimator_kf2.cpp — EKF2 (est_mode=2、ヨー擬似観測付き4状態EKF) 実装
//
// yaw_estimator_kf.cpp のコピーを基に MAG_AUTOTUNE_DESIGN.md §2.1 の変更のみを
// 加えたもの (コピー元は無改変で並走)。磁気観測の数式・ゲート・定数は現行EKFと
// 完全同一。差分は updateYawObs / τ_bm 適応 (a=1 + q_bm 切替) / flightReanchor
// の3点で、該当箇所に契約書参照コメントを付す。
// ===========================================================================
#include "yaw_estimator_kf2.hpp"

#include <math.h>

#include "angle_utils.hpp"
#include "yaw_config.hpp"

namespace {
float clamp01Abs(float value) {
    if (value > 1.0f) return 1.0f;
    if (value < -1.0f) return -1.0f;
    return value;
}
}  // namespace

void YawEstimatorKf2::resetCovariance() {
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {
            P_[r][c] = 0.0f;
        }
    }
    P_[0][0] = FF_EKF_P0_PSI_RAD2;
    P_[1][1] = FF_EKF_P0_BG_RAD2_S2;
    P_[2][2] = FF_EKF_P0_BM_UT2;
    P_[3][3] = FF_EKF_P0_BM_UT2;
}

// ヨー観測まわりの再初期化 (契約 §2.1-1): ψ が飛ぶ操作 (reanchor / reseedYaw)
// では過去の連続棄却・融合停止ラッチは意味を失うため解除する。
// time_since_yaw_accept は「未受理」へ戻す (τ_bm はホールド側 = 安全側)。
void YawEstimatorKf2::resetYawObsState() {
    yaw_innov_rad_ = 0.0f;
    time_since_yaw_accept_s_ = 1.0e6f;
    yaw_reject_count_ = 0;
    yaw_fusion_stopped_ = false;
    // ソフト再捕捉状態機械 (FF_EKF2_YAW_RECAPTURE_*) も初期状態へ
    time_since_yaw_stop_s_ = 0.0f;
    yaw_obs_gap_s_ = 0.0f;
    yaw_recapture_streak_ = 0;
    yaw_recapture_innov0_rad_ = 0.0f;
    yaw_recapture_window_s_ = 0.0f;
    yaw_recapture_active_ = false;
    yaw_recapture_hold_s_ = 0.0f;
}

void YawEstimatorKf2::reanchor(float psi0_rad, float b0h_x, float b0h_y, const MagVector& b0_full) {
    psi0_ = wrapPi(psi0_rad);
    b0h_x_ = b0h_x;
    b0h_y_ = b0h_y;
    b0_ = b0_full;
    b0_norm_ = magNorm(b0_full);
    x_[0] = psi0_;
    x_[2] = 0.0f;  // b_m ← 0 (アンカーで基準場を取り直したため)
    x_[3] = 0.0f;
    resetCovariance();
    anchor_valid_ = true;
    mag_frozen_ = false;
    nis_ = 0.0f;
    gate_bits_ = 0;
    time_since_accept_s_ = 0.0f;
    drift_warn_time_s_ = 0.0f;
    resetYawObsState();
}

void YawEstimatorKf2::reseedYaw(float psi_rad) {
    x_[0] = wrapPi(psi_rad);
    resetCovariance();
    nis_ = 0.0f;
    gate_bits_ &= FF_EKF_GATE_BM_FROZEN;  // 凍結状態はアンカーでのみ解除
    time_since_accept_s_ = 0.0f;
    drift_warn_time_s_ = 0.0f;
    resetYawObsState();
}

void YawEstimatorKf2::flightReanchor(float b0h_x, float b0h_y, const MagVector& b0f_full) {
    // 契約 §2.1-3: B0f = 飛行中 b_corr_filt の 2s リング平均へ基準を差し替える。
    // ψ0 ← EKF2 現在 ψ (β=0 から再開)、b_m ← 0、**P_ψ・P_bg は維持**
    // (P0 スナップ窓を開かない — 地上アンカーの K_ψ=0.87°/µT スナップ問題
    //  [FLIGHT_ANALYSIS_20260727.md §3.3] を飛行中に再現しないため)。
    // P_bm ← (2µT)² とし、b_m 行/列の相互共分散はクリアして整合を保つ。
    psi0_ = x_[0];
    b0h_x_ = b0h_x;
    b0h_y_ = b0h_y;
    b0_ = b0f_full;
    b0_norm_ = magNorm(b0f_full);
    x_[2] = 0.0f;
    x_[3] = 0.0f;
    for (uint8_t k = 0; k < 4; k++) {
        P_[2][k] = 0.0f;
        P_[k][2] = 0.0f;
        P_[3][k] = 0.0f;
        P_[k][3] = 0.0f;
    }
    P_[2][2] = FF_EKF2_FLIGHT_ANCHOR_P_BM_UT2;
    P_[3][3] = FF_EKF2_FLIGHT_ANCHOR_P_BM_UT2;
    anchor_valid_ = true;
    mag_frozen_ = false;  // b_m←0 のため凍結 (‖b_m‖>20µT) は解除
    gate_bits_ &= static_cast<uint8_t>(~FF_EKF_GATE_BM_FROZEN);
    // 磁気受理タイマ・ドリフト警告・ヨー観測状態は連続性を保って維持する
    // (ψ は飛ばないため。yaw_obs_fused が発動条件なので受理は直近にある)。
}

void YawEstimatorKf2::predict(
    float omega_z_rad_s,
    float roll_rate_rad_s,
    float roll_rad,
    float pitch_rad,
    float dt_s
) {
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        dt_s = SENSOR_PERIOD_US * 1.0e-6f;
    }

    // ---- τ_bm 適応 (契約 §2.1-2) ----
    // EKF2 は Gauss-Markov ゼロ回帰を廃止 (a=1 固定。現行EKFの
    // a = 1 − dt/τ_bm によるゼロ回帰ポンプ [FLIGHT_ANALYSIS_20260727.md §2-4]
    // を除去)。代わりに q_bm を yaw_obs 健全性で切り替える:
    //   健全 (最終受理 < FF_EKF2_YAW_OBS_HEALTHY_S): q_bm 現行値 (ランダム
    //   ウォーク = ヨー観測で可観測化された b_m を自由に追従)
    //   喪失 (≥ 同): q_bm×FF_EKF2_Q_BM_LOST_FACTOR (準凍結ホールド =
    //   学習済み b_m でコースト)
    time_since_yaw_accept_s_ += dt_s;
    // ソフト再捕捉の計時: 観測間隔 (updateYawObs で消費) と段階1の経過
    yaw_obs_gap_s_ += dt_s;
    if (yaw_fusion_stopped_) {
        time_since_yaw_stop_s_ += dt_s;
    }
    tau_rw_mode_ = time_since_yaw_accept_s_ < FF_EKF2_YAW_OBS_HEALTHY_S;
    const float q_bm = tau_rw_mode_
        ? FF_EKF_Q_BM_UT2_S
        : FF_EKF_Q_BM_UT2_S * FF_EKF2_Q_BM_LOST_FACTOR;

    // ---- チルト運動学予測 (V2改修A。現行EKFと同一数式) ----
    // 導出・実データ検証は yaw_estimator_kf.cpp の predict 冒頭コメント参照。
    const float cos_phi = cosf(roll_rad);
    float psi_dot;
    float dpsidot_dbg;  // ∂ψ̇/∂b_g (F[0][1] = dpsidot_dbg·dt のヤコビアン用)
    if (cos_phi >= FF_EKF_TILT_KIN_COS_MIN) {
        const float cos_theta = cosf(pitch_rad);
        const float sin_theta = sinf(pitch_rad);
        const float inv_cos_phi = 1.0f / cos_phi;
        psi_dot =
            ((omega_z_rad_s - x_[1]) * cos_theta - roll_rate_rad_s * sin_theta) * inv_cos_phi;
        dpsidot_dbg = -cos_theta * inv_cos_phi;
    } else {
        // ロール特異点近傍 (|φ|>60°, FF_EKF_TILT_KIN_COS_MIN): 従来式へフォールバック
        psi_dot = omega_z_rad_s - x_[1];
        dpsidot_dbg = -1.0f;
    }
    // 数値安全弁: ±720°/s にクランプ
    if (psi_dot > FF_EKF_PSI_DOT_CLAMP_RAD_S) psi_dot = FF_EKF_PSI_DOT_CLAMP_RAD_S;
    if (psi_dot < -FF_EKF_PSI_DOT_CLAMP_RAD_S) psi_dot = -FF_EKF_PSI_DOT_CLAMP_RAD_S;

    x_[0] = wrapPi(x_[0] + psi_dot * dt_s);
    // b_m は a=1 (状態遷移で減衰しない。契約 §2.1-2)

    // P⁻ = F·P·Fᵀ + Q·dt,  F = [[1,g,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]],
    // g = ∂(Δψ)/∂b_g = dpsidot_dbg·dt (b_m ブロックは恒等 = a=1)
    const float g01 = dpsidot_dbg * dt_s;
    float FP[4][4];
    for (uint8_t c = 0; c < 4; c++) {
        FP[0][c] = P_[0][c] + g01 * P_[1][c];
        FP[1][c] = P_[1][c];
        FP[2][c] = P_[2][c];
        FP[3][c] = P_[3][c];
    }
    for (uint8_t r = 0; r < 4; r++) {
        const float col0 = FP[r][0] + g01 * FP[r][1];
        P_[r][0] = col0;
        // col1-3 = FP[r][1..3] そのまま
        P_[r][1] = FP[r][1];
        P_[r][2] = FP[r][2];
        P_[r][3] = FP[r][3];
    }
    P_[0][0] += FF_EKF_Q_PSI_RAD2_S * dt_s;
    P_[1][1] += FF_EKF_Q_BG_RAD2_S3 * dt_s;
    P_[2][2] += q_bm * dt_s;
    P_[3][3] += q_bm * dt_s;

    // 連続棄却 > 3s: ψ・b_m の対角を 1.02/s で緩膨張(P0 の 10 倍上限)。
    if (anchor_valid_) {
        time_since_accept_s_ += dt_s;
        if (time_since_accept_s_ > FF_EKF_REJECT_INFLATE_AFTER_S) {
            const float factor = 1.0f + (FF_EKF_REJECT_INFLATE_RATE_PER_S - 1.0f) * dt_s;
            const float psi_cap = FF_EKF_P_INFLATE_MAX_RATIO * FF_EKF_P0_PSI_RAD2;
            const float bm_cap = FF_EKF_P_INFLATE_MAX_RATIO * FF_EKF_P0_BM_UT2;
            P_[0][0] = fminf(P_[0][0] * factor, fmaxf(P_[0][0], psi_cap));
            P_[2][2] = fminf(P_[2][2] * factor, fmaxf(P_[2][2], bm_cap));
            P_[3][3] = fminf(P_[3][3] * factor, fmaxf(P_[3][3], bm_cap));
        }
    }
}

void YawEstimatorKf2::update(
    const MagVector& b_corr_filt,
    float roll_rad,
    float pitch_rad,
    float sigma_ff_uT,
    float sigma_slew_uT,
    float sigma_diff_uT,
    float mag_dt_s
) {
    // ---- 磁気更新: 現行EKF (yaw_estimator_kf.cpp) と完全同一 (契約 §2.1-4) ----
    if (!anchor_valid_) {
        return;
    }
    if (mag_dt_s <= 0.0f || mag_dt_s > 0.5f) {
        mag_dt_s = 0.1f;
    }
    // bit5(凍結)/bit6(ドリフト警告)はラッチ、bit0-4 は直近更新の状態。
    uint8_t bits = gate_bits_ & (FF_EKF_GATE_BM_FROZEN | FF_EKF_GATE_DRIFT_WARN);

    // bit4: tilt > 25° → 磁気更新スキップ (レベル化の信頼性が落ちる)
    const float cos_tilt = clamp01Abs(cosf(roll_rad) * cosf(pitch_rad));
    const float tilt_rad = acosf(cos_tilt);
    if (tilt_rad > FF_EKF_TILT_SKIP_RAD) {
        gate_bits_ = bits | FF_EKF_GATE_TILT_SKIP;
        return;
    }

    // bit5: ‖b_m‖>20µT → 磁気更新凍結 (FFモデル破綻。再アンカーで解除)
    if (mag_frozen_) {
        gate_bits_ = bits;
        return;
    }

    // ノルム/z ゲート (基準はアンカー実測 B0。飛行状態再アンカー後は B0f)
    const float norm = magNorm(b_corr_filt);
    const float norm_dev = fabsf(norm - b0_norm_);
    if (norm_dev > FF_EKF_NORM_GATE_HARD_UT) {
        gate_bits_ = bits | FF_EKF_GATE_NORM_REJECT;
        return;
    }
    if (fabsf(b_corr_filt.z - b0_.z) > FF_EKF_Z_GATE_UT) {
        gate_bits_ = bits | FF_EKF_GATE_Z_REJECT;
        return;
    }

    // 観測 z = レベル化した水平2成分
    const MagVector level = levelMagVectorBody(roll_rad, pitch_rad, b_corr_filt);
    const float zx = level.x;
    const float zy = level.y;

    // h(x) = R_z(ψ−ψ0)·B0_horiz + b_m,  R_z は標準CCW
    const float beta = wrapPi(x_[0] - psi0_);
    const float cb = cosf(beta);
    const float sb = sinf(beta);
    const float hx = cb * b0h_x_ - sb * b0h_y_ + x_[2];
    const float hy = sb * b0h_x_ + cb * b0h_y_ + x_[3];
    // ∂h/∂ψ = R_z'(β)·B0_horiz
    const float dhx = -sb * b0h_x_ - cb * b0h_y_;
    const float dhy = cb * b0h_x_ - sb * b0h_y_;

    const float y0 = zx - hx;
    const float y1 = zy - hy;

    // 適応 R: R_eff = R_base + σ_ff² + σ_slew² + σ_diff² + (sinθ_tilt·σ_rz)²
    const float sin_tilt = sinf(tilt_rad);
    float r_eff = FF_EKF_R_BASE_UT2 +
                  sigma_ff_uT * sigma_ff_uT +
                  sigma_slew_uT * sigma_slew_uT +
                  sigma_diff_uT * sigma_diff_uT +
                  (sin_tilt * FF_EKF_SIGMA_RZ_UT) * (sin_tilt * FF_EKF_SIGMA_RZ_UT);
    // ノルム偏差 8-20µT はソフト側: R 膨張 (bit0 扱い)
    if (norm_dev > FF_EKF_NORM_GATE_SOFT_UT) {
        const float ratio = norm_dev / FF_EKF_NORM_GATE_SOFT_UT;
        r_eff *= ratio * ratio;
        bits |= FF_EKF_GATE_R_INFLATED;
    }

    // S = H·P⁻·Hᵀ + R_eff·I₂  (H = [[dhx,0,1,0],[dhy,0,0,1]])
    float HP0[4];
    float HP1[4];
    for (uint8_t c = 0; c < 4; c++) {
        HP0[c] = dhx * P_[0][c] + P_[2][c];
        HP1[c] = dhy * P_[0][c] + P_[3][c];
    }
    float s00 = HP0[0] * dhx + HP0[2] + r_eff;
    float s01 = HP0[0] * dhy + HP0[3];
    float s10 = HP1[0] * dhx + HP1[2];
    float s11 = HP1[0] * dhy + HP1[3] + r_eff;

    float det = s00 * s11 - s01 * s10;
    if (det <= 1.0e-9f || !isfinite(det)) {
        gate_bits_ = bits | FF_EKF_GATE_NIS_REJECT;
        return;
    }
    float inv00 = s11 / det;
    float inv01 = -s01 / det;
    float inv10 = -s10 / det;
    float inv11 = s00 / det;

    // NIS = yᵀ·S⁻¹·y (基本 R での値を報告)
    const float nis = y0 * (inv00 * y0 + inv01 * y1) + y1 * (inv10 * y0 + inv11 * y1);
    nis_ = nis;

    // V2改修B-1: ソフト再捕捉 (現行EKFと同一)。
    const bool recapture =
        nis > FF_EKF_NIS_REJECT && time_since_accept_s_ > FF_EKF_RECAPTURE_AFTER_S;
    if (nis > FF_EKF_NIS_REJECT && !recapture) {
        // bit1: NIS > χ²₂(99.9%) = 13.8 → 棄却
        gate_bits_ = bits | FF_EKF_GATE_NIS_REJECT;
        return;
    }
    if (recapture) {
        r_eff *= nis / FF_EKF_NIS_REJECT;
        bits |= FF_EKF_GATE_RECAPTURE;
    } else if (nis > FF_EKF_NIS_INFLATE) {
        // bit0: NIS > χ²₂(95%) = 5.99 → R×(NIS/5.99) に膨張して採用
        r_eff *= nis / FF_EKF_NIS_INFLATE;
        bits |= FF_EKF_GATE_R_INFLATED;
    }
    if (recapture || nis > FF_EKF_NIS_INFLATE) {
        // 膨張後の R で S・S⁻¹ を再計算
        s00 = HP0[0] * dhx + HP0[2] + r_eff;
        s01 = HP0[0] * dhy + HP0[3];
        s10 = HP1[0] * dhx + HP1[2];
        s11 = HP1[0] * dhy + HP1[3] + r_eff;
        det = s00 * s11 - s01 * s10;
        if (det <= 1.0e-9f || !isfinite(det)) {
            gate_bits_ = bits | FF_EKF_GATE_NIS_REJECT;
            return;
        }
        inv00 = s11 / det;
        inv01 = -s01 / det;
        inv10 = -s10 / det;
        inv11 = s00 / det;
    }

    // K = P⁻·Hᵀ·S⁻¹ (4×2)。P·Hᵀ の列は HP の転置。
    float K[4][2];
    for (uint8_t r = 0; r < 4; r++) {
        const float ph0 = HP0[r];  // (P·Hᵀ)[r][0] = (H·P)[0][r] (P 対称)
        const float ph1 = HP1[r];
        K[r][0] = ph0 * inv00 + ph1 * inv10;
        K[r][1] = ph0 * inv01 + ph1 * inv11;
    }
    if (recapture) {
        // 制限付き更新: バイアス (b_g/b_mx/b_my) への補正はゼロ (現行EKFと同一)
        K[1][0] = 0.0f;
        K[1][1] = 0.0f;
        K[2][0] = 0.0f;
        K[2][1] = 0.0f;
        K[3][0] = 0.0f;
        K[3][1] = 0.0f;
    }

    const float bm_prev_x = x_[2];
    const float bm_prev_y = x_[3];
    float dx[4];
    for (uint8_t r = 0; r < 4; r++) {
        dx[r] = K[r][0] * y0 + K[r][1] * y1;
    }
    if (recapture) {
        // Δψ を ±FF_EKF_RECAPTURE_MAX_STEP_RAD(3°)/更新にクランプ
        if (dx[0] > FF_EKF_RECAPTURE_MAX_STEP_RAD) dx[0] = FF_EKF_RECAPTURE_MAX_STEP_RAD;
        if (dx[0] < -FF_EKF_RECAPTURE_MAX_STEP_RAD) dx[0] = -FF_EKF_RECAPTURE_MAX_STEP_RAD;
    }
    for (uint8_t r = 0; r < 4; r++) {
        x_[r] += dx[r];
    }
    x_[0] = wrapPi(x_[0]);

    // P = (I − K·H)·P⁻
    float KH[4][4];
    for (uint8_t r = 0; r < 4; r++) {
        KH[r][0] = K[r][0] * dhx + K[r][1] * dhy;
        KH[r][1] = 0.0f;
        KH[r][2] = K[r][0];
        KH[r][3] = K[r][1];
    }
    float newP[4][4];
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {
            float acc = P_[r][c];
            for (uint8_t k = 0; k < 4; k++) {
                acc -= KH[r][k] * P_[k][c];
            }
            newP[r][c] = acc;
        }
    }
    // 対称化 (数値誤差の蓄積対策)
    for (uint8_t r = 0; r < 4; r++) {
        for (uint8_t c = 0; c < 4; c++) {
            P_[r][c] = 0.5f * (newP[r][c] + newP[c][r]);
        }
    }

    if (recapture) {
        gate_bits_ = bits;
        return;
    }

    time_since_accept_s_ = 0.0f;

    // bit5: ‖b_m‖ > 20µT → FF モデル破綻 → 磁気更新凍結 (要再アンカー)
    const float bm_norm = sqrtf(x_[2] * x_[2] + x_[3] * x_[3]);
    if (bm_norm > FF_EKF_BM_FREEZE_UT) {
        mag_frozen_ = true;
        bits |= FF_EKF_GATE_BM_FROZEN;
    }

    // bit6: |db_m/dt| > 0.3µT/s が 10s 継続で警告
    const float dbm = sqrtf(
        (x_[2] - bm_prev_x) * (x_[2] - bm_prev_x) + (x_[3] - bm_prev_y) * (x_[3] - bm_prev_y));
    const float bm_rate = dbm / mag_dt_s;
    if (bm_rate > FF_EKF_BM_DRIFT_WARN_UT_S) {
        drift_warn_time_s_ += mag_dt_s;
    } else {
        drift_warn_time_s_ = 0.0f;
        bits &= ~FF_EKF_GATE_DRIFT_WARN;
    }
    if (drift_warn_time_s_ >= FF_EKF_BM_DRIFT_WARN_HOLD_S) {
        bits |= FF_EKF_GATE_DRIFT_WARN;
    }

    gate_bits_ = bits;
}

void YawEstimatorKf2::updateYawObs(float psi_meas_rad, bool low_trust, float dt_s) {
    // ---- ヨー擬似観測 (契約 §2.1-1) ----
    // H=[1,0,0,0]、y=wrapPi(ψ_meas−ψ) のスカラー逐次更新。4状態すべてに
    // 効かせる — b_g/b_m は P の相関経由で可観測化される (磁気2式+ヨー1式で
    // 一定ヘディングのホバ中でも b_m の2自由度が完全可観測になる。これが本体)。
    (void)dt_s;  // 契約シグネチャ互換 (観測間隔は yaw_obs_gap_s_ で実測)
    if (!anchor_valid_ || !isfinite(psi_meas_rad)) {
        return;
    }
    // 観測間隔 (predict 側で積算した実時間)。dt_s は 400Hz tick の周期であって
    // 観測間隔 (実効50Hz) ではないため、再捕捉の窓/ホールド計時には使わない。
    const float obs_dt = yaw_obs_gap_s_;
    yaw_obs_gap_s_ = 0.0f;

    const float y = wrapPi(psi_meas_rad - x_[0]);
    yaw_innov_rad_ = y;  // 棄却・融合停止中もテレメトリへ残す (契約 §2.1-1)

    // ゲート: |y| > 30° は棄却+連続棄却カウンタ。連続 N≥25 で融合停止ラッチ
    // (基準ヨーソースの座標系不整合などの持続異常から ψ を守る)。ラッチの
    // 解除は reanchor / reseedYaw、またはソフト再捕捉状態機械
    // (yaw_config.hpp FF_EKF2_YAW_RECAPTURE_*。FLIGHT_ANALYSIS_20260731:
    //  飛行中の恒久ラッチ発動 = fused 0% への回復経路)。
    if (fabsf(y) > FF_EKF2_YAW_GATE_RAD) {
        if (yaw_reject_count_ < 255) {
            yaw_reject_count_++;
        }
        if (yaw_reject_count_ >= FF_EKF2_YAW_GATE_STOP_COUNT && !yaw_fusion_stopped_) {
            yaw_fusion_stopped_ = true;
            time_since_yaw_stop_s_ = 0.0f;  // 再捕捉 段階1の起点
        }
        // ゲート外 → 段階2の窓は破棄。制限融合中 (段階3) なら段階1へ戻る
        // (5s 待機からやり直し)
        yaw_recapture_streak_ = 0;
        yaw_recapture_window_s_ = 0.0f;
        if (yaw_recapture_active_) {
            yaw_recapture_active_ = false;
            yaw_recapture_hold_s_ = 0.0f;
            time_since_yaw_stop_s_ = 0.0f;
        }
        return;
    }
    yaw_reject_count_ = 0;

    if (yaw_fusion_stopped_ && !yaw_recapture_active_) {
        // ---- ソフト再捕捉 段階1→2 (融合はまだ行わない) ----
        // 段階1: stopped 遷移 (または段階3からの転落) から 5s は再入不能
        // (誤基準のまま瞬間的にゲート内へ入るケースの様子見)。
        if (time_since_yaw_stop_s_ < FF_EKF2_YAW_RECAPTURE_AFTER_S) {
            yaw_recapture_streak_ = 0;
            yaw_recapture_window_s_ = 0.0f;
            return;
        }
        // 段階2: ゲート内が M 観測連続、かつ窓のイノベーションドリフトレート
        // が閾値未満なら段階3 (制限融合) へ。観測ストリーム断 (>1s) は
        // 「連続」が切れたとみなし窓を貯め直す (防御的継続性検査)。
        if (obs_dt > FF_EKF2_YAW_FRESH_WINDOW_S) {
            yaw_recapture_streak_ = 0;
        }
        if (yaw_recapture_streak_ == 0) {
            yaw_recapture_innov0_rad_ = y;
            yaw_recapture_window_s_ = 0.0f;
        } else {
            yaw_recapture_window_s_ += obs_dt;
        }
        yaw_recapture_streak_++;
        if (yaw_recapture_streak_ >= FF_EKF2_YAW_RECAPTURE_M) {
            // ドリフトレートは窓全体の平均 |Δinnov|/T (観測毎差分だと基準ヨー
            // のフレームジッタ ~0.5°/20ms=25°/s で誤遮断するため)。誤基準
            // (鏡像) のまま回頭中は d(innov)/dt≈2ψ̇ がここで遮断される。
            const float drift_rad_s = yaw_recapture_window_s_ > 1.0e-3f
                ? fabsf(wrapPi(y - yaw_recapture_innov0_rad_)) / yaw_recapture_window_s_
                : 1.0e6f;
            if (drift_rad_s < FF_EKF2_YAW_RECAPTURE_DRIFT_RAD_S) {
                yaw_recapture_active_ = true;  // 段階3: 制限融合モードへ
                yaw_recapture_hold_s_ = 0.0f;
            }
            // 判定は窓単位: 不合格なら窓を貯め直す
            yaw_recapture_streak_ = 0;
            yaw_recapture_window_s_ = 0.0f;
        }
        return;
    }
    if (yaw_recapture_active_ && obs_dt > FF_EKF2_YAW_FRESH_WINDOW_S) {
        // 段階3中の観測ストリーム断: ゲート内「継続」を確認できないため
        // 段階1へ戻る (防御的継続性検査)
        yaw_recapture_active_ = false;
        yaw_recapture_hold_s_ = 0.0f;
        yaw_recapture_streak_ = 0;
        yaw_recapture_window_s_ = 0.0f;
        time_since_yaw_stop_s_ = 0.0f;
        return;
    }

    // R_ψ = (2°)² (通常) / (6°)² (low_trust。移動ベースヨー等の低信頼基準)
    const float r_psi = low_trust ? FF_EKF2_R_PSI_LOW_TRUST_RAD2 : FF_EKF2_R_PSI_RAD2;

    // スカラー更新: S = P00 + R_ψ, K = P·Hᵀ/S = P[:,0]/S
    const float s = P_[0][0] + r_psi;
    if (s <= 1.0e-12f || !isfinite(s)) {
        return;
    }
    const float inv_s = 1.0f / s;
    float K[4];
    for (uint8_t r = 0; r < 4; r++) {
        K[r] = P_[r][0] * inv_s;
    }

    // Δψ クランプ ±3°/更新 (既存 FF_EKF_RECAPTURE_MAX_STEP_RAD 流用。
    // 実効50Hz → 最大150°/s の引き込みレート)
    float dx0 = K[0] * y;
    bool clamped = false;
    if (dx0 > FF_EKF_RECAPTURE_MAX_STEP_RAD) {
        dx0 = FF_EKF_RECAPTURE_MAX_STEP_RAD;
        clamped = true;
    } else if (dx0 < -FF_EKF_RECAPTURE_MAX_STEP_RAD) {
        dx0 = -FF_EKF_RECAPTURE_MAX_STEP_RAD;
        clamped = true;
    }
    if (clamped || yaw_recapture_active_) {
        // クランプ発動時は制限付き更新へ切替 (磁気更新の recapture と同流儀 —
        // yaw_estimator_kf.cpp の「制限付き更新」コメント参照):
        //   - バイアス行 (b_g/b_m) の K を 0 化。reject-inflation で P00 が膨張し
        //     磁気更新由来の P02/P03 相関が stale なまま大イノベーション (最大
        //     30° = ゲート内) が来ると、フル K[r]·y がバイアスを一撃で蹴り
        //     誤配分が数十秒残るため (EKF1 recapture と同じ既知故障モード)。
        //   - ψ 行はクランプ相当の実効ゲイン K0_eff = Δψ_clamp/y に置換
        //     (ψ が 3° しか動いていないのに P00 が「全補正済み」に収縮する
        //     不整合を防ぐ)。
        // ソフト再捕捉の制限融合モード (段階3) 中は未クランプでも常時この
        // 経路 (基準の信頼が回復するまで誤補正をバイアスへ配分しない)。
        if (clamped) {
            K[0] = dx0 / y;  // クランプ発動 ⇒ |y| > 3° (K0≤1) のためゼロ除算なし
        }
        K[1] = 0.0f;
        K[2] = 0.0f;
        K[3] = 0.0f;

        x_[0] = wrapPi(x_[0] + dx0);

        // P 更新は Joseph 形 P=(I−KH)P(I−KH)ᵀ+KRKᵀ の K=[k0,0,0,0] 特殊化:
        //   P00←(1−k0)²P00+k0²R_ψ, P0c←(1−k0)P0c (c≥1), 他は不変。
        // 劣最適ゲイン (バイアス行 K=0) に簡略形 (I−KH)P を使うと ψ 行だけが
        // 収縮して stale な P02/P03 相関が相対的に残り、反復適用で P が
        // 非正定 (P00<0 → S≤0 → 融合恒久停止) に転落する (制限融合モードの
        // リプレイ実証で再現)。Joseph 形は任意ゲインで PSD を保存する。
        const float k0 = K[0];
        const float omk = 1.0f - k0;
        for (uint8_t c = 1; c < 4; c++) {
            P_[0][c] *= omk;
            P_[c][0] = P_[0][c];
        }
        P_[0][0] = omk * omk * P_[0][0] + k0 * k0 * r_psi;
    } else {
        x_[0] = wrapPi(x_[0] + dx0);
        x_[1] += K[1] * y;
        x_[2] += K[2] * y;
        x_[3] += K[3] * y;

        // P = (I − K·H)·P⁻,  K·H は第0列のみ非零 → (K·H·P)[r][c] = K[r]·P[0][c]
        // (最適ゲインの標準形。制限付き更新は上の Joseph 形を使う)
        float newP[4][4];
        for (uint8_t r = 0; r < 4; r++) {
            for (uint8_t c = 0; c < 4; c++) {
                newP[r][c] = P_[r][c] - K[r] * P_[0][c];
            }
        }
        // 対称化 (数値誤差の蓄積対策。磁気更新と同じ流儀)
        for (uint8_t r = 0; r < 4; r++) {
            for (uint8_t c = 0; c < 4; c++) {
                P_[r][c] = 0.5f * (newP[r][c] + newP[c][r]);
            }
        }
    }

    if (yaw_recapture_active_) {
        // ---- 段階3→4: 制限融合中は「受理」に数えない ----
        // time_since_yaw_accept_ は進めたまま (= q_bm ホールド維持・
        // yaw_obs_fused / flightReanchor 条件は成立させない。磁気更新
        // recapture が time_since_accept_ を維持するのと同流儀)。
        // ゲート内のまま FF_EKF2_YAW_RECAPTURE_HOLD_S 継続でラッチ完全解除。
        yaw_recapture_hold_s_ += obs_dt;
        if (yaw_recapture_hold_s_ >= FF_EKF2_YAW_RECAPTURE_HOLD_S) {
            yaw_fusion_stopped_ = false;
            yaw_recapture_active_ = false;
            yaw_recapture_hold_s_ = 0.0f;
        }
        return;
    }

    time_since_yaw_accept_s_ = 0.0f;  // τ_bm 適応 (§2.1-2)・status bit1 の根拠
}
