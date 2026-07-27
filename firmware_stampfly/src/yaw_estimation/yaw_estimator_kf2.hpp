#ifndef STAMPFLY_YAW_ESTIMATION_YAW_ESTIMATOR_KF2_HPP
#define STAMPFLY_YAW_ESTIMATION_YAW_ESTIMATOR_KF2_HPP

#include <Arduino.h>

#include "angle_utils.hpp"
#include "mag_calibration.hpp"
#include "yaw_estimator_kf.hpp"  // FfEkfGateBits(ekf2_gate は ffg と同一ビット定義)

// EKF2 (est_mode=2): 現行4状態EKF (yaw_estimator_kf.{hpp,cpp}) のコピーを基に
// MAG_AUTOTUNE_DESIGN.md §2.1 の変更を加えたヨー擬似観測付き推定器。
// コピー元 (YawEstimatorKf) は無改変で並走する (V2契約維持)。
// 状態 x = [ψ, b_g, b_mx, b_my] (rad / rad/s / µT / µT)。
//
// 現行EKFとの差分 (契約 §2.1):
//  1. ヨー擬似観測 updateYawObs(): H=[1,0,0,0] のスカラー逐次更新。b_g/b_m は
//     P の相関経由で可観測化される (これが本体)。R_ψ は通常/low_trust の2段。
//     |y|>30° ゲート+連続棄却 N≥25 で融合停止ラッチ。Δψ は
//     FF_EKF_RECAPTURE_MAX_STEP_RAD(3°)/更新にクランプ (クランプ発動時は
//     recapture と同流儀の制限付き更新: バイアス行 K=0+ψ行は実効ゲイン)。
//  2. τ_bm 適応: Gauss-Markov ゼロ回帰を廃止 (a=1固定)。q_bm は yaw_obs 健全
//     (最終受理<1.0s) で現行値 (ランダムウォーク)、喪失で ×0.1 (準凍結ホールド)。
//  3. 飛行状態再アンカー flightReanchor(): B0 を飛行中 b_corr_filt 平均 (B0f) へ
//     差し替え。ψ0←現在ψ、b_m←0、P_ψ 維持 (P0 スナップ窓を開かない)、
//     P_bm←(2µT)²。発動条件・1フライト1回の管理は sensor_hub_ff 側。
//  4. その他 (磁気観測数式・norm/z/tilt/NIS/ソフト再捕捉ゲート・定数) は
//     現行EKFと同一。ゲートビットも FfEkfGateBits (ffg) と同一定義。
class YawEstimatorKf2 {
public:
    // アンカー確定 (モーター始動遷移 / ffanchor)。ψ←ψ0, b_m←0, P←P0。
    // b_g は継続(ジャイロバイアス推定は磁場に依存しないため)。
    // b0_full はノルム/z ゲートの基準となるアンカー3軸磁場。
    // EKF2 追加: ヨー観測の連続棄却カウンタ・融合停止ラッチも解除する。
    void reanchor(float psi0_rad, float b0h_x, float b0h_y, const MagVector& b0_full);
    // ffmode 切替時の再シード: アンカーは保持したまま ψ を与えられた yaw に
    // 合わせ、P を P0 へ戻す。ヨー観測の棄却カウンタ・融合停止ラッチも解除。
    void reseedYaw(float psi_rad);
    // 飛行状態再アンカー (契約 §2.1-3)。b0f_full = 飛行中 b_corr_filt の
    // 2s リング平均 (B0f)、b0h_* はそのレベル化水平成分。
    // ψ0←現在ψ / b_m←0 / P_ψ・P_bg 維持 / P_bm←FF_EKF2_FLIGHT_ANCHOR_P_BM_UT2。
    // 以後 norm/z ゲート基準は B0f。凍結 (bit5) も解除する。
    void flightReanchor(float b0h_x, float b0h_y, const MagVector& b0f_full);
    // 予測ステップ (400Hz, dt 実測)。数式は現行EKFのチルト運動学予測と同一。
    // EKF2 差分: b_m は a=1 (ゼロ回帰なし)、q_bm は yaw_obs 健全性で切替 (§2.1-2)。
    void predict(
        float omega_z_rad_s,
        float roll_rate_rad_s,
        float roll_rad,
        float pitch_rad,
        float dt_s
    );
    // 磁気更新ステップ (現行EKFと同一。fresh 磁気サンプルのみ呼ぶこと)。
    void update(
        const MagVector& b_corr_filt,
        float roll_rad,
        float pitch_rad,
        float sigma_ff_uT,
        float sigma_slew_uT,
        float sigma_diff_uT,
        float mag_dt_s
    );
    // ヨー擬似観測 (契約 §2.1-1)。psi_meas_rad は外部ヨー基準 (CMD_POS_ERR の
    // mocap_yaw)、low_trust は flags bit4。呼び出しは新しい CMD_POS_ERR を
    // 消費した 400Hz tick (実効50Hz)、bit3 有効+age<100ms のときのみ。
    // dt_s は呼び出し tick の実測周期 (現状の数式では未使用。契約 §2.1 の
    // シグネチャ互換のため保持)。
    void updateYawObs(float psi_meas_rad, bool low_trust, float dt_s);

    float yaw() const { return x_[0]; }
    float gyroBias() const { return x_[1]; }
    float bmx() const { return x_[2]; }
    float bmy() const { return x_[3]; }
    float nis() const { return nis_; }
    uint8_t gateBits() const { return gate_bits_; }
    bool anchorValid() const { return anchor_valid_; }
    // 最終「通常受理」(磁気更新 NIS<13.8 での採用) からの経過秒 (現行EKFと同一)。
    float timeSinceAccept() const { return time_since_accept_s_; }
    // ‖b_m‖>20µT による磁気更新凍結中か (ekf2_status bit4。再アンカーで解除)
    bool magFrozen() const { return mag_frozen_; }
    // 直近ヨー観測イノベーション y=wrapPi(ψ_meas−ψ) [rad](棄却時も更新。
    // 未受信時 0。TLM_STATE ekf2_yaw_innov_rad)
    float yawInnov() const { return yaw_innov_rad_; }
    // 最終ヨー観測「受理」からの経過秒 (τ_bm 適応 §2.1-2 と
    // ekf2_status bit1 / 飛行状態再アンカーの yaw_obs_fused 判定が参照)
    float timeSinceYawAccept() const { return time_since_yaw_accept_s_; }
    // q_bm ランダムウォークモード中か (ekf2_status bit3 = tau_rw_mode)
    bool tauRwMode() const { return tau_rw_mode_; }
    // ヨー観測の融合停止ラッチ中か (連続棄却 N≥25。解除は reanchor/reseedYaw)
    bool yawFusionStopped() const { return yaw_fusion_stopped_; }

private:
    void resetCovariance();
    void resetYawObsState();

    float x_[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    float P_[4][4] = {};
    float psi0_ = 0.0f;
    float b0h_x_ = 0.0f;
    float b0h_y_ = 0.0f;
    MagVector b0_;
    float b0_norm_ = 0.0f;
    bool anchor_valid_ = false;
    bool mag_frozen_ = false;         // bit4: ‖b_m‖>20µT で凍結、再アンカーで解除
    float nis_ = 0.0f;
    uint8_t gate_bits_ = 0;
    float time_since_accept_s_ = 0.0f;  // 連続棄却の P 緩膨張用 (磁気更新)
    float drift_warn_time_s_ = 0.0f;    // ffg bit6 の継続時間

    // ---- EKF2 追加状態 (契約 §2.1) ----
    float yaw_innov_rad_ = 0.0f;              // 直近ヨー観測イノベーション
    float time_since_yaw_accept_s_ = 1.0e6f;  // 最終ヨー観測受理からの経過
                                              // (初期値は「未受理」= 喪失扱い)
    uint8_t yaw_reject_count_ = 0;            // ヨー観測の連続棄却カウンタ
    bool yaw_fusion_stopped_ = false;         // 連続棄却 N≥25 の融合停止ラッチ
    bool tau_rw_mode_ = false;                // q_bm ランダムウォークモード中
};

#endif
