#!/usr/bin/env python3
"""EKF2(est_mode=2)の Python リプレイ — 飛行ログからのヨー軌跡 A/B 検証.

仕様: docs/MAG_AUTOTUNE_DESIGN.md §2.1(EKF2数式)・§4(リプレイ)。
YawEstimatorKf2Replay は firmware_stampfly/src/yaw_estimation/yaw_estimator_kf.cpp
(現行EKF=コピー元, 無改変)の Code Identity 移植に、契約 §2.1 の EKF2 変更
(ヨー擬似観測 / τ_bm適応 / 飛行状態再アンカー)を加えたもの。定数は
yaw_config.hpp から転記し、対応行をコメントで明示する。

パイプライン(契約 §4):
  v6ログ: tlm_mag_cal_x/y/z(b_cal) + FFプロファイルJSON + magbias JSON から
    ΔB̂ を再構成 → b_corr = b_cal − ΔB̂ − Δb → EMA(α=0.18) → レベル化 →
    EKF2 predict/update(+ヨー観測 mocap真値 or 任意列)。
  v5ログ(制限モード): 磁気生値が無いため、ログ済み EKF1 状態
    (tlm_yaw_est_rad, tlm_bm_x/y_ut)から観測を合成
    z_synth = R_z(ψ_est−ψ0)·B0h + b_m_logged し、EKF1 の更新検出
    (nis/ffg 変化)+ ffg 受理マスクのサンプルのみ update する
    (FLIGHT_ANALYSIS_20260727.md §2(4) の「実ログの受理マスク使用の
    KF再生シム」と同型)。
    【制限モードの限界】合成観測は EKF1 の事後推定値であり、EKF1 の
    ψ/b_m 配分を決めた実観測イノベーション履歴は v5 ログに存在しない。
    このため再現されるのは「観測整合なヨー軌跡のクラス」(ψ誤差の規模・
    ゲート/凍結挙動・準定常バイアスの吸収)であって、ψ と b_m の配分そのもの
    ではない(配分は不可観測方向の事前分布依存 — 同 §3.3 の「押し付け合い」)。
    EKF1 相当挙動の確認(受入基準 #6)は --ekf1-mode + mocap比較RMSが
    EKF1 実測と同オーダーであることで行う。--yaw-obs mocap は
    「EKF2 だったらどうなったか」(b_m 可観測化)の A/B に使う。

近似の明記(契約 §4「姿勢はテレメトリ25Hz補間の近似である旨を明記」):
  - 姿勢(roll/pitch)・レートはテレメトリ 25Hz サンプル(ファームは 400Hz)。
    predict の dt はテレメトリフレーム間隔(≈40ms)で、Q は dt スケールのため
    統計的には等価だが、ラップ・非線形項の離散化誤差が残る。
  - roll/pitch はマウントオフセット適用前(tlm_roll/pitch_rad)。
  - レートはファームのフィルタ後(tlm_p/r_rad_s)。EKF実機は未フィルタ値。
  - 磁気 fresh 判定は mag_cal 値の変化検出(実機は data_ready フラグ)。
  - ヨー観測はテレメトリフレーム毎(≈25Hz。実機は CMD_POS_ERR 実効50Hz)。

使い方:
  .venv/bin/python replay/ekf2_replay.py <flight_log.csv>
      [--ff-profile <ff.json>] [--magbias <magbias.json>]
      [--yaw-obs none|mocap|<列名>] [--low-trust] [-o out_dir]
      [--b0h-ut 27.9] [--no-flight-anchor]

出力: <out_dir>/ekf2_replay_<stem>.csv(ψ軌跡・b_m・NIS・ゲート)+比較図 PNG。
依存: numpy, matplotlib。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent          # data_analysis/replay
DATA_ANALYSIS = HERE.parent
GRAPH_DIR = DATA_ANALYSIS / "graphs"

DEG = math.pi / 180.0

# ===========================================================================
# 定数転記 (firmware_stampfly/src/yaw_estimation/yaw_config.hpp)。
# 「L__」は同ファイルの行番号。V2契約: 値の変更禁止(転記のみ)。
# ===========================================================================
MAG_FILTER_ALPHA = 0.18                          # L40
FF_DIFF_MIN_SUM_CURRENT_A = 0.05                 # L95
FF_KAPPA_FF = 0.03                               # L100
FF_TAU_RESID_S = 0.05                            # L101
FF_SIGMA_DIFF_UT_PER_A = 0.3 * 30.0              # L102
FF_EKF_Q_PSI_RAD2_S = 5.0e-4 * DEG * DEG         # L112
FF_EKF_Q_BG_RAD2_S3 = 1.0e-8 * DEG * DEG         # L114
FF_EKF_Q_BM_UT2_S = 0.02                         # L116
FF_EKF_TAU_BM_S = 120.0                          # L118 (EKF2では未使用: a=1固定)
FF_EKF_P0_PSI_RAD2 = (10.0 * DEG) ** 2           # L120
FF_EKF_P0_BG_RAD2_S2 = (0.5 * DEG) ** 2          # L121
FF_EKF_P0_BM_UT2 = 16.0                          # L122
FF_EKF_R_BASE_UT2 = 4.0                          # L124
FF_EKF_SIGMA_RZ_UT = 3.5                         # L125
FF_EKF_NIS_INFLATE = 5.99                        # L127
FF_EKF_NIS_REJECT = 13.8                         # L128
FF_EKF_NORM_GATE_SOFT_UT = 8.0                   # L130
FF_EKF_NORM_GATE_HARD_UT = 20.0                  # L131
FF_EKF_Z_GATE_UT = 12.0                          # L133
FF_EKF_TILT_SKIP_RAD = 25.0 * DEG                # L135
FF_EKF_BM_FREEZE_UT = 20.0                       # L137
FF_EKF_BM_DRIFT_WARN_UT_S = 0.3                  # L139
FF_EKF_BM_DRIFT_WARN_HOLD_S = 10.0               # L140
FF_EKF_REJECT_INFLATE_AFTER_S = 3.0              # L142
FF_EKF_REJECT_INFLATE_RATE_PER_S = 1.02          # L143
FF_EKF_P_INFLATE_MAX_RATIO = 10.0                # L144
FF_EKF_TILT_KIN_COS_MIN = 0.5                    # L154
FF_EKF_PSI_DOT_CLAMP_RAD_S = 720.0 * DEG         # L156
FF_EKF_RECAPTURE_AFTER_S = 5.0                   # L161
FF_EKF_RECAPTURE_MAX_STEP_RAD = 3.0 * DEG        # L164

# ---- EKF2 追加定数 (契約 §2.1 = yaw_config.hpp L179-213 と同名・同値) ----
FF_EKF2_R_PSI_RAD2 = (2.0 * DEG) ** 2            # L185 R_ψ 通常 = (2°)²
FF_EKF2_R_PSI_LOW_TRUST_RAD2 = (6.0 * DEG) ** 2  # L187 low_trust = (6°)²
FF_EKF2_YAW_GATE_RAD = 30.0 * DEG                # L190 |y|>30° 棄却
FF_EKF2_YAW_GATE_STOP_COUNT = 25                 # L193 連続棄却N≥25で融合停止
FF_EKF2_YAW_OBS_HEALTHY_S = 1.0                  # L199 yaw_obs健全(最終受理<1.0s)
FF_EKF2_Q_BM_LOST_FACTOR = 0.1                   # L200 喪失時 q_bm×0.1(準凍結)
FF_EKF2_YAW_FRESH_WINDOW_S = 1.0                 # L202 status bit0(受信<1s)
FF_EKF2_YAW_FUSED_WINDOW_S = 0.5                 # L205 status bit1(受理<0.5s)
FF_EKF2_FLIGHT_ANCHOR_MIN_FLY_S = 5.0            # L209 flying遷移後>5s
FF_EKF2_FLIGHT_ANCHOR_ALT_TOL_M = 0.1            # L210 |alt_est−alt_ref|<0.1m
FF_EKF2_FLIGHT_ANCHOR_ALT_HOLD_S = 2.0           # L211 が2s継続
FF_EKF2_FLIGHT_ANCHOR_MIN_CURRENT_A = 1.0        # L212 電流>1.0A
FF_EKF2_FLIGHT_ANCHOR_P_BM_UT2 = 4.0             # L213 P_bm←(2µT)²
FF_EKF2_FLIGHT_ANCHOR_RING_S = 2.0               # §2.1(3) b_corr_filt の2sリング平均

# EKF ゲートビット (yaw_estimator_kf.hpp L21-31 FfEkfGateBits と同一)
GATE_R_INFLATED = 1 << 0
GATE_NIS_REJECT = 1 << 1
GATE_NORM_REJECT = 1 << 2
GATE_Z_REJECT = 1 << 3
GATE_TILT_SKIP = 1 << 4
GATE_BM_FROZEN = 1 << 5
GATE_DRIFT_WARN = 1 << 6
GATE_RECAPTURE = 1 << 7
GATE_REJECT_MASK = (GATE_NIS_REJECT | GATE_NORM_REJECT | GATE_Z_REJECT |
                    GATE_TILT_SKIP | GATE_BM_FROZEN)

SENSOR_PERIOD_S = 2500e-6                        # yaw_config.hpp L22 (400Hz)
TLM_FLAG_FLYING = 1 << 2                         # protocol TlmState.FLAG_FLYING


def wrap_pi(v: float) -> float:
    """angle_utils.hpp L20-26 wrapPi と同義 (スカラー)。"""
    if not math.isfinite(v):
        return 0.0
    return (v + math.pi) % (2.0 * math.pi) - math.pi


def level_mag_vector_body(roll_rad: float, pitch_rad: float, m) -> np.ndarray:
    """angle_utils.hpp L53-63 levelMagVectorBody の忠実移植。

    非教科書符号(実機検証済み)を含む — 変更禁止。
    """
    cr = math.cos(roll_rad)
    sr = math.sin(roll_rad)
    cp = math.cos(pitch_rad)
    sp = math.sin(pitch_rad)
    return np.array([
        m[0] * cp + m[2] * sp,
        m[0] * sr * sp + m[1] * cr + m[2] * sr * cp,
        -m[0] * cr * sp + m[1] * sr + m[2] * cr * cp,
    ])


# ===========================================================================
# EKF2 (yaw_estimator_kf.cpp の Code Identity 移植 + 契約 §2.1 変更)
# ===========================================================================
class YawEstimatorKf2Replay:
    """状態 x=[ψ, b_g, b_mx, b_my]。数式は yaw_estimator_kf.cpp と同一,
    変更点は §2.1 の (1)ヨー擬似観測 (2)τ_bm適応 (3)飛行状態再アンカー のみ。

    ekf1_compat=True で契約 §2.1(2) の変更を無効化し、現行EKF1の
    Gauss-Markov 減衰 (a=1−dt/τ_bm, kf.cpp L66)・固定 q_bm に戻す
    (受入基準 #6 の「v5ログで EKF1 相当挙動を再現」検証用)。
    """

    def __init__(self, ekf1_compat: bool = False, initial_align: bool = False):
        self.ekf1_compat = ekf1_compat
        # ---- 改造(scratch専用): 初回整列 — 一度も受理していない状態で
        # 最初の有効ヨー観測が届いたら、30°ゲートの前に ψ を観測へ reseed して
        # フレーム整列する(ブートストラップ・デッドロック対策案の効果実証用)。
        self.initial_align = initial_align
        self.yaw_align_done = False
        self.align_event_innov_rad = None
        self.x = np.zeros(4)
        self.P = np.zeros((4, 4))
        self.psi0 = 0.0
        self.b0h_x = 0.0
        self.b0h_y = 0.0
        self.b0 = np.zeros(3)
        self.b0_norm = 0.0
        self.anchor_valid = False
        self.mag_frozen = False
        self.nis = 0.0
        self.gate_bits = 0
        self.time_since_accept_s = 0.0
        self.drift_warn_time_s = 0.0
        # ---- EKF2 追加状態 (契約 §2.1) ----
        self.yaw_obs_last_accept_s = math.inf   # 最終ヨー観測受理からの経過
        self.yaw_obs_last_recv_s = math.inf     # 最終ヨー観測受信からの経過
        self.yaw_obs_reject_count = 0           # 連続棄却カウンタ
        self.yaw_obs_stopped = False            # 連続棄却N≥25の融合停止フラグ
        self.yaw_obs_innov = 0.0                # 直近イノベーション (テレメトリ用)
        self.flight_anchor_done = False         # 1フライト1回

    # ---- yaw_estimator_kf.cpp L16-26 resetCovariance ----
    def _reset_covariance(self):
        self.P = np.zeros((4, 4))
        self.P[0][0] = FF_EKF_P0_PSI_RAD2
        self.P[1][1] = FF_EKF_P0_BG_RAD2_S2
        self.P[2][2] = FF_EKF_P0_BM_UT2
        self.P[3][3] = FF_EKF_P0_BM_UT2

    # ---- yaw_estimator_kf2.cpp L39-44 resetYawObsState ----
    # ψ が飛ぶ操作 (reanchor / reseedYaw) では連続棄却・融合停止ラッチを解除。
    # time_since_yaw_accept は「未受理」(=τ_bm ホールド側 = 安全側) へ戻す。
    def _reset_yaw_obs_state(self):
        self.yaw_obs_innov = 0.0
        self.yaw_obs_last_accept_s = math.inf
        self.yaw_obs_reject_count = 0
        self.yaw_obs_stopped = False

    # ---- yaw_estimator_kf.cpp L28-44 reanchor (地上アンカー) ----
    def reanchor(self, psi0_rad: float, b0h_x: float, b0h_y: float, b0_full):
        self.psi0 = wrap_pi(psi0_rad)
        self.b0h_x = b0h_x
        self.b0h_y = b0h_y
        self.b0 = np.asarray(b0_full, float).copy()
        self.b0_norm = float(np.linalg.norm(self.b0))
        self.x[0] = self.psi0
        self.x[2] = 0.0
        self.x[3] = 0.0
        self._reset_covariance()
        self.anchor_valid = True
        self.mag_frozen = False
        self.nis = 0.0
        self.gate_bits = 0
        self.time_since_accept_s = 0.0
        self.drift_warn_time_s = 0.0
        self._reset_yaw_obs_state()

    # ---- yaw_estimator_kf.cpp L46-53 reseedYaw ----
    def reseed_yaw(self, psi_rad: float):
        self.x[0] = wrap_pi(psi_rad)
        self._reset_covariance()
        self.nis = 0.0
        self.gate_bits &= GATE_BM_FROZEN
        self.time_since_accept_s = 0.0
        self.drift_warn_time_s = 0.0
        self._reset_yaw_obs_state()

    # ---- yaw_estimator_kf2.cpp L75-101 flightReanchor (契約 §2.1-3) ----
    def flight_reanchor(self, b0h_x: float, b0h_y: float, b0f_full):
        """B0f=飛行中 b_corr_filt の2sリング平均(機体系)へ基準を差し替える。
        ψ0←現在ψ(β=0から再開)、b_m←0、P_ψ・P_bg は維持
        (P0スナップ窓を開かない)、P_bm←(2µT)²(相互共分散クリア)。
        磁気受理タイマ・ドリフト警告・ヨー観測状態は連続性を保って維持。"""
        b0f = np.asarray(b0f_full, float)
        self.psi0 = float(self.x[0])
        self.b0h_x = float(b0h_x)
        self.b0h_y = float(b0h_y)
        self.b0 = b0f.copy()
        self.b0_norm = float(np.linalg.norm(b0f))
        self.x[2] = 0.0
        self.x[3] = 0.0
        for i in (2, 3):
            for j in range(4):
                self.P[i][j] = 0.0
                self.P[j][i] = 0.0
        self.P[2][2] = FF_EKF2_FLIGHT_ANCHOR_P_BM_UT2
        self.P[3][3] = FF_EKF2_FLIGHT_ANCHOR_P_BM_UT2
        self.anchor_valid = True
        self.mag_frozen = False  # b_m←0 のため凍結は解除 (kf2.cpp L97-98)
        self.gate_bits &= ~GATE_BM_FROZEN & 0xFF
        self.flight_anchor_done = True

    # ---- yaw_estimator_kf.cpp L55-143 predict + 契約 §2.1(2) τ_bm適応 ----
    def predict(self, omega_z_rad_s, roll_rate_rad_s, roll_rad, pitch_rad, dt_s):
        if dt_s <= 0.0 or dt_s > 0.2:
            dt_s = SENSOR_PERIOD_S

        if self.ekf1_compat:
            # EKF1 互換: Gauss-Markov 減衰 (kf.cpp L66) + 固定 q_bm
            a = 1.0 - dt_s / FF_EKF_TAU_BM_S
            q_bm = FF_EKF_Q_BM_UT2_S
        else:
            # §2.1(2): Gauss-Markov ゼロ回帰を廃止 (a=1固定)。
            a = 1.0
            # q_bm 適応: yaw_obs健全(最終受理<1.0s)=現行値 / 喪失=×0.1(準凍結)
            yaw_obs_healthy = self.yaw_obs_last_accept_s < FF_EKF2_YAW_OBS_HEALTHY_S
            q_bm = FF_EKF_Q_BM_UT2_S if yaw_obs_healthy \
                else FF_EKF_Q_BM_UT2_S * FF_EKF2_Q_BM_LOST_FACTOR

        # チルト運動学予測 (kf.cpp L83-100 と同一)
        cos_phi = math.cos(roll_rad)
        if cos_phi >= FF_EKF_TILT_KIN_COS_MIN:
            cos_theta = math.cos(pitch_rad)
            sin_theta = math.sin(pitch_rad)
            inv_cos_phi = 1.0 / cos_phi
            psi_dot = ((omega_z_rad_s - self.x[1]) * cos_theta
                       - roll_rate_rad_s * sin_theta) * inv_cos_phi
            dpsidot_dbg = -cos_theta * inv_cos_phi
        else:
            psi_dot = omega_z_rad_s - self.x[1]
            dpsidot_dbg = -1.0
        psi_dot = max(-FF_EKF_PSI_DOT_CLAMP_RAD_S,
                      min(FF_EKF_PSI_DOT_CLAMP_RAD_S, psi_dot))

        self.x[0] = wrap_pi(self.x[0] + psi_dot * dt_s)
        self.x[2] *= a
        self.x[3] *= a

        # P⁻ = F·P·Fᵀ + Q·dt (kf.cpp L106-129 と同一。a=1)
        g01 = dpsidot_dbg * dt_s
        F = np.eye(4)
        F[0][1] = g01
        F[2][2] = a
        F[3][3] = a
        self.P = F @ self.P @ F.T
        self.P[0][0] += FF_EKF_Q_PSI_RAD2_S * dt_s
        self.P[1][1] += FF_EKF_Q_BG_RAD2_S3 * dt_s
        self.P[2][2] += q_bm * dt_s
        self.P[3][3] += q_bm * dt_s

        # 連続棄却 > 3s の P 緩膨張 (kf.cpp L131-142 と同一)
        if self.anchor_valid:
            self.time_since_accept_s += dt_s
            if self.time_since_accept_s > FF_EKF_REJECT_INFLATE_AFTER_S:
                factor = 1.0 + (FF_EKF_REJECT_INFLATE_RATE_PER_S - 1.0) * dt_s
                psi_cap = FF_EKF_P_INFLATE_MAX_RATIO * FF_EKF_P0_PSI_RAD2
                bm_cap = FF_EKF_P_INFLATE_MAX_RATIO * FF_EKF_P0_BM_UT2
                self.P[0][0] = min(self.P[0][0] * factor,
                                   max(self.P[0][0], psi_cap))
                self.P[2][2] = min(self.P[2][2] * factor,
                                   max(self.P[2][2], bm_cap))
                self.P[3][3] = min(self.P[3][3] * factor,
                                   max(self.P[3][3], bm_cap))

        # EKF2 追加: ヨー観測タイマの前進
        self.yaw_obs_last_accept_s += dt_s
        self.yaw_obs_last_recv_s += dt_s

    # ---- yaw_estimator_kf.cpp L145-382 update (磁気観測) — 完全同一 ----
    def update(self, b_corr_filt, roll_rad, pitch_rad,
               sigma_ff_ut, sigma_slew_ut, sigma_diff_ut, mag_dt_s):
        if not self.anchor_valid:
            return
        if mag_dt_s <= 0.0 or mag_dt_s > 0.5:
            mag_dt_s = 0.1
        bits = self.gate_bits & (GATE_BM_FROZEN | GATE_DRIFT_WARN)

        cos_tilt = max(-1.0, min(1.0, math.cos(roll_rad) * math.cos(pitch_rad)))
        tilt_rad = math.acos(cos_tilt)
        if tilt_rad > FF_EKF_TILT_SKIP_RAD:
            self.gate_bits = bits | GATE_TILT_SKIP
            return

        if self.mag_frozen:
            self.gate_bits = bits
            return

        b = np.asarray(b_corr_filt, float)
        norm = float(np.linalg.norm(b))
        norm_dev = abs(norm - self.b0_norm)
        if norm_dev > FF_EKF_NORM_GATE_HARD_UT:
            self.gate_bits = bits | GATE_NORM_REJECT
            return
        if abs(b[2] - self.b0[2]) > FF_EKF_Z_GATE_UT:
            self.gate_bits = bits | GATE_Z_REJECT
            return

        level = level_mag_vector_body(roll_rad, pitch_rad, b)
        zx = level[0]
        zy = level[1]

        beta = wrap_pi(self.x[0] - self.psi0)
        cb = math.cos(beta)
        sb = math.sin(beta)
        hx = cb * self.b0h_x - sb * self.b0h_y + self.x[2]
        hy = sb * self.b0h_x + cb * self.b0h_y + self.x[3]
        dhx = -sb * self.b0h_x - cb * self.b0h_y
        dhy = cb * self.b0h_x - sb * self.b0h_y

        y0 = zx - hx
        y1 = zy - hy

        sin_tilt = math.sin(tilt_rad)
        r_eff = (FF_EKF_R_BASE_UT2 + sigma_ff_ut ** 2 + sigma_slew_ut ** 2 +
                 sigma_diff_ut ** 2 + (sin_tilt * FF_EKF_SIGMA_RZ_UT) ** 2)
        if norm_dev > FF_EKF_NORM_GATE_SOFT_UT:
            ratio = norm_dev / FF_EKF_NORM_GATE_SOFT_UT
            r_eff *= ratio * ratio
            bits |= GATE_R_INFLATED

        HP0 = dhx * self.P[0] + self.P[2]
        HP1 = dhy * self.P[0] + self.P[3]
        s00 = HP0[0] * dhx + HP0[2] + r_eff
        s01 = HP0[0] * dhy + HP0[3]
        s10 = HP1[0] * dhx + HP1[2]
        s11 = HP1[0] * dhy + HP1[3] + r_eff
        det = s00 * s11 - s01 * s10
        if det <= 1.0e-9 or not math.isfinite(det):
            self.gate_bits = bits | GATE_NIS_REJECT
            return
        inv00, inv01 = s11 / det, -s01 / det
        inv10, inv11 = -s10 / det, s00 / det

        nis = y0 * (inv00 * y0 + inv01 * y1) + y1 * (inv10 * y0 + inv11 * y1)
        self.nis = nis

        recapture = (nis > FF_EKF_NIS_REJECT and
                     self.time_since_accept_s > FF_EKF_RECAPTURE_AFTER_S)
        if nis > FF_EKF_NIS_REJECT and not recapture:
            self.gate_bits = bits | GATE_NIS_REJECT
            return
        if recapture:
            r_eff *= nis / FF_EKF_NIS_REJECT
            bits |= GATE_RECAPTURE
        elif nis > FF_EKF_NIS_INFLATE:
            r_eff *= nis / FF_EKF_NIS_INFLATE
            bits |= GATE_R_INFLATED
        if recapture or nis > FF_EKF_NIS_INFLATE:
            s00 = HP0[0] * dhx + HP0[2] + r_eff
            s01 = HP0[0] * dhy + HP0[3]
            s10 = HP1[0] * dhx + HP1[2]
            s11 = HP1[0] * dhy + HP1[3] + r_eff
            det = s00 * s11 - s01 * s10
            if det <= 1.0e-9 or not math.isfinite(det):
                self.gate_bits = bits | GATE_NIS_REJECT
                return
            inv00, inv01 = s11 / det, -s01 / det
            inv10, inv11 = -s10 / det, s00 / det

        K = np.zeros((4, 2))
        for r in range(4):
            ph0 = HP0[r]
            ph1 = HP1[r]
            K[r][0] = ph0 * inv00 + ph1 * inv10
            K[r][1] = ph0 * inv01 + ph1 * inv11
        if recapture:
            K[1][:] = 0.0
            K[2][:] = 0.0
            K[3][:] = 0.0

        bm_prev = (self.x[2], self.x[3])
        dx = K[:, 0] * y0 + K[:, 1] * y1
        if recapture:
            dx[0] = max(-FF_EKF_RECAPTURE_MAX_STEP_RAD,
                        min(FF_EKF_RECAPTURE_MAX_STEP_RAD, dx[0]))
        self.x += dx
        self.x[0] = wrap_pi(self.x[0])

        H = np.zeros((2, 4))
        H[0][0] = dhx
        H[0][2] = 1.0
        H[1][0] = dhy
        H[1][3] = 1.0
        newP = (np.eye(4) - K @ H) @ self.P
        self.P = 0.5 * (newP + newP.T)

        if recapture:
            self.gate_bits = bits
            return

        self.time_since_accept_s = 0.0

        bm_norm = math.hypot(self.x[2], self.x[3])
        if bm_norm > FF_EKF_BM_FREEZE_UT:
            self.mag_frozen = True
            bits |= GATE_BM_FROZEN

        dbm = math.hypot(self.x[2] - bm_prev[0], self.x[3] - bm_prev[1])
        bm_rate = dbm / mag_dt_s
        if bm_rate > FF_EKF_BM_DRIFT_WARN_UT_S:
            self.drift_warn_time_s += mag_dt_s
        else:
            self.drift_warn_time_s = 0.0
            bits &= ~GATE_DRIFT_WARN
        if self.drift_warn_time_s >= FF_EKF_BM_DRIFT_WARN_HOLD_S:
            bits |= GATE_DRIFT_WARN
        self.gate_bits = bits

    # ---- yaw_estimator_kf2.cpp L415-507 updateYawObs (契約 §2.1-1) ----
    def update_yaw_obs(self, psi_meas_rad: float, low_trust: bool) -> bool:
        """H=[1,0,0,0] のスカラー逐次更新(4状態全てに効かせる — b_g/b_m は
        P の相関経由で可観測化。これが本体)。戻り値: 受理したか。"""
        if not self.anchor_valid or not math.isfinite(psi_meas_rad):
            return False
        self.yaw_obs_last_recv_s = 0.0
        y = wrap_pi(psi_meas_rad - self.x[0])
        self.yaw_obs_innov = y  # 棄却・融合停止中もテレメトリへ残す

        # ---- 改造: 初回整列 (未融合なら初回有効観測で ψ を reseed) ----
        if self.initial_align and not self.yaw_align_done:
            self.yaw_align_done = True
            if abs(y) > FF_EKF2_YAW_GATE_RAD:
                self.align_event_innov_rad = float(y)
                self.reseed_yaw(psi_meas_rad)   # P0リセット+ラッチ解除込み
                self.yaw_obs_last_accept_s = 0.0
                return True

        # ゲート: |y|>30° は棄却+カウンタ。連続 N≥25 で融合停止ラッチ
        # (解除は reanchor / reseedYaw — kf2.cpp L428-443)。
        if abs(y) > FF_EKF2_YAW_GATE_RAD:
            self.yaw_obs_reject_count = min(self.yaw_obs_reject_count + 1, 255)
            if self.yaw_obs_reject_count >= FF_EKF2_YAW_GATE_STOP_COUNT:
                self.yaw_obs_stopped = True
            return False
        self.yaw_obs_reject_count = 0
        if self.yaw_obs_stopped:
            return False

        r_psi = FF_EKF2_R_PSI_LOW_TRUST_RAD2 if low_trust else FF_EKF2_R_PSI_RAD2
        s = self.P[0][0] + r_psi
        if s <= 1.0e-12 or not math.isfinite(s):
            return False
        K = self.P[:, 0] / s          # K = P·Hᵀ/S (4x1)
        # Δψ クランプ ±3°/更新 (FF_EKF_RECAPTURE_MAX_STEP_RAD 流用。§2.1(1))
        dx0 = K[0] * y
        clamped = abs(dx0) > FF_EKF_RECAPTURE_MAX_STEP_RAD
        if clamped:
            # クランプ発動時は制限付き更新 (kf2.cpp updateYawObs と同一。
            # recapture と同流儀): バイアス行 K=0 (stale 相関経由の一撃防止)、
            # ψ 行は実効ゲイン K0_eff=Δψ_clamp/y (状態と P 更新の整合。保守側)。
            dx0 = math.copysign(FF_EKF_RECAPTURE_MAX_STEP_RAD, dx0)
            K = np.array([dx0 / y, 0.0, 0.0, 0.0])  # |y|>3° のためゼロ除算なし
        self.x[0] = wrap_pi(self.x[0] + dx0)
        self.x[1:] += K[1:] * y
        # P = (I − K·H)·P, H=[1,0,0,0] → 対称化
        newP = self.P - np.outer(K, self.P[0])
        self.P = 0.5 * (newP + newP.T)
        self.yaw_obs_last_accept_s = 0.0
        return True

    # ---- ステータス (protocol EKF2_STATUS_* 相当) ----
    def status_bits(self) -> int:
        bits = 0
        if self.yaw_obs_last_recv_s < FF_EKF2_YAW_FRESH_WINDOW_S:
            bits |= 0x01                       # yaw_obs_fresh
        if self.yaw_obs_last_accept_s < FF_EKF2_YAW_FUSED_WINDOW_S:
            bits |= 0x02                       # yaw_obs_fused
        if self.flight_anchor_done:
            bits |= 0x04
        if self.yaw_obs_last_accept_s < FF_EKF2_YAW_OBS_HEALTHY_S:
            bits |= 0x08                       # tau_rw_mode (RW側)
        if self.mag_frozen:
            bits |= 0x10
        return bits


# ===========================================================================
# FF補正の再構成 (ff_calibration.cpp compute の移植。プロファイルJSON入力)
# ===========================================================================
class FfProfileCorrector:
    """stampfly_ff_profile v1 JSON から ΔB̂ とσ自己申告を再構成する。

    ff_calibration.cpp L244-312 compute と同一ロジック
    (方式A: 区分線形LUT、方式B: +個別モーター差動項)。
    """

    def __init__(self, profile: dict, i_idle_a: float | None = None):
        lut = profile["method_a"]["lut"]
        pts = sorted(lut["points"], key=lambda p: p["i_a"])
        self.lut_ia = np.array([p["i_a"] for p in pts])
        self.lut_db = np.array([p["db"] for p in pts])
        self.iid = float(i_idle_a if i_idle_a is not None else lut["i_idle_a"])
        mb = profile["method_b"]
        self.a_tilde = {m: np.asarray(mb["a_tilde"][m], float)
                        for m in ("FL", "FR", "RL", "RR")}
        self.d2c = {m: mb["duty_to_current"][m] for m in ("FL", "FR", "RL", "RR")}
        self._slew_last = None  # (t_s, i_a)

    def _lut_interp(self, i_a: float) -> np.ndarray:
        # 範囲外は端区間の傾きで外挿 (ff_calibration.cpp L208-242)
        k = int(np.searchsorted(self.lut_ia, i_a, side="left")) - 1
        k = max(0, min(k, len(self.lut_ia) - 2))
        di = self.lut_ia[k + 1] - self.lut_ia[k]
        if di <= 0.0:
            return self.lut_db[k].copy()
        slope = (self.lut_db[k + 1] - self.lut_db[k]) / di
        return self.lut_db[k] + slope * (i_a - self.lut_ia[k])

    def _lut_slope_xy(self, i_a: float) -> float:
        k = int(np.searchsorted(self.lut_ia, i_a, side="left")) - 1
        k = max(0, min(k, len(self.lut_ia) - 2))
        di = self.lut_ia[k + 1] - self.lut_ia[k]
        if di <= 0.0:
            return 0.0
        sl = (self.lut_db[k + 1] - self.lut_db[k]) / di
        return math.hypot(sl[0], sl[1])

    def compute(self, t_s: float, i_total_a: float, duty4: dict,
                motors_running: bool, ff_mode: int) -> tuple[np.ndarray, float, float, float]:
        """→ (ΔB̂[3], σ_ff, σ_slew, σ_diff)。duty4 は {FL,FR,RL,RR: duty}。"""
        if ff_mode == 0 or not math.isfinite(i_total_a):
            self._slew_last = None
            return np.zeros(3), 0.0, 0.0, 0.0
        delta_b = self._lut_interp(i_total_a)

        sigma_slew = 0.0
        if self._slew_last is not None and t_s > self._slew_last[0]:
            dt = t_s - self._slew_last[0]
            if 0.0 < dt < 0.5:
                didt = abs(i_total_a - self._slew_last[1]) / dt
                sigma_slew = self._lut_slope_xy(i_total_a) * didt * FF_TAU_RESID_S
        self._slew_last = (t_s, i_total_a)

        sigma_diff = 0.0
        if ff_mode == 2 and motors_running:
            i_active = i_total_a - self.iid
            i_hat = {}
            sum_i_hat = 0.0
            for m in ("FL", "FR", "RL", "RR"):
                d = duty4.get(m, 0.0)
                if not math.isfinite(d):
                    d = 0.0
                c = self.d2c[m]
                i_hat[m] = c["c2"] * d * d + c["c1"] * d + c["c0"]
                sum_i_hat += i_hat[m]
            if sum_i_hat >= FF_DIFF_MIN_SUM_CURRENT_A:
                s = i_active / sum_i_hat
                max_abs_di = 0.0
                for m in ("FL", "FR", "RL", "RR"):
                    di = s * i_hat[m] - i_active * 0.25
                    delta_b = delta_b + self.a_tilde[m] * di
                    max_abs_di = max(max_abs_di, abs(di))
                sigma_diff = FF_SIGMA_DIFF_UT_PER_A * max_abs_di

        sigma_ff = FF_KAPPA_FF * math.hypot(delta_b[0], delta_b[1])
        return delta_b, sigma_ff, sigma_slew, sigma_diff


# ===========================================================================
# ログ読み込み・前処理
# ===========================================================================
def _f(v) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load_flight_log(path: Path) -> dict:
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f"空のCSVです: {path}")
    out: dict = {}
    for key in rows[0].keys():
        if key is None:
            continue
        vals = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
        if not np.isfinite(vals).any():
            out[key] = np.array([r.get(key) or "" for r in rows], dtype=object)
        else:
            out[key] = vals
    out["_n"] = len(rows)
    return out


def col(data: dict, name: str) -> np.ndarray:
    v = data.get(name)
    if v is None or getattr(v, "dtype", None) == object:
        return np.full(data["_n"], np.nan)
    return v


def dedupe_frames(data: dict) -> np.ndarray:
    """PC側50Hz記録から機体テレメトリフレーム(≈25Hz)の行indexを抽出。

    tlm_elapsed_ms の変化行のみ採用(重複ホールド行を落とす)。
    """
    em = col(data, "tlm_elapsed_ms")
    if not np.isfinite(em).any():
        return np.arange(data["_n"])
    idx = [i for i in range(data["_n"]) if np.isfinite(em[i])]
    keep = []
    last = None
    for i in idx:
        if last is None or em[i] != last:
            keep.append(i)
            last = em[i]
    return np.array(keep, dtype=int)


# ===========================================================================
# リプレイ本体
# ===========================================================================
def run_replay(data: dict, *,
               ff_profile: dict | None = None,
               magbias: dict | None = None,
               yaw_obs_mode: str = "none",
               low_trust: bool = False,
               b0h_ut: float = 27.9,
               enable_flight_anchor: bool = True,
               align_yaw_obs: bool = True,
               ekf1_compat: bool = False,
               initial_align: bool = False) -> dict:
    """飛行ログ dict → リプレイ結果 dict(時系列と統計)。

    yaw_obs_mode: "none" / "mocap"(mocap_yaw_true_deg) / 任意のrad列名。
    v6(mag_cal あり)はフル再構成、v5 は制限モード(合成観測+受理マスク)。
    """
    frames = dedupe_frames(data)
    em = col(data, "tlm_elapsed_ms")[frames] * 1e-3
    t_log = col(data, "elapsed_time")[frames]
    t = em if np.isfinite(em).all() else t_log
    n = len(frames)

    roll = col(data, "tlm_roll_rad")[frames]
    pitch = col(data, "tlm_pitch_rad")[frames]
    r_rate = col(data, "tlm_r_rad_s")[frames]     # ω_z (フィルタ後。近似)
    p_rate = col(data, "tlm_p_rad_s")[frames]     # ロールレート (同上)
    current = col(data, "tlm_current_a")[frames]
    yaw_est_log = col(data, "tlm_yaw_est_rad")[frames]
    bm_x_log = col(data, "tlm_bm_x_ut")[frames]
    bm_y_log = col(data, "tlm_bm_y_ut")[frames]
    ffg_log = col(data, "tlm_ffg")[frames]
    nis_log = col(data, "tlm_nis")[frames]
    ffs = col(data, "tlm_ff_status")[frames]
    ff_mode_log = (np.nan_to_num(ffs, nan=0.0).astype(np.int64) & 0x03)
    flags = np.nan_to_num(col(data, "tlm_flags")[frames], nan=0.0).astype(np.int64)
    flying = (flags & TLM_FLAG_FLYING) != 0
    alt_est = col(data, "tlm_altitude_est_m")[frames]
    alt_ref = col(data, "tlm_alt_ref_m")[frames]
    duty = {m: col(data, f"tlm_duty_{m.lower()}")[frames]
            for m in ("FL", "FR", "RL", "RR")}
    mocap_true_rad = np.radians(col(data, "mocap_yaw_true_deg")[frames])
    db_hat_x_log = col(data, "tlm_db_hat_x_ut")[frames]
    db_hat_y_log = col(data, "tlm_db_hat_y_ut")[frames]

    mag_cal = np.column_stack([col(data, f"tlm_mag_cal_{a}_ut")[frames]
                               for a in ("x", "y", "z")])
    is_v6 = np.isfinite(mag_cal).any()
    warnings: list[str] = []

    dbias = np.zeros(3)
    if magbias is not None and magbias.get("delta_b_ut"):
        dbias = np.asarray(magbias["delta_b_ut"], float)

    kf = YawEstimatorKf2Replay(ekf1_compat=ekf1_compat,
                               initial_align=initial_align)
    if ekf1_compat:
        enable_flight_anchor = False
        warnings.append("EKF1互換モード: τ_bm GM減衰・固定q_bm・飛行再アンカー無効")
    corrector = None

    # ---- アンカー初期化 ----
    #   v6: 地上窓 (非飛行・duty=0) の b_cal 平均 (ffAnchorService 相当の近似)。
    #       地上窓が無ければ最初のフレームの b_cal で代用 (warning)。
    #   v5: 合成観測用の仮想アンカー (B0h=(b0h_ut,0), B0z は norm を一定に保つ値)。
    duty_off = np.ones(n, bool)
    for m in duty:
        duty_off &= ~(np.isfinite(duty[m]) & (duty[m] > 0.0))
    ground = (~flying) & duty_off

    if is_v6:
        i_idle = float(np.nanmedian(current[ground])) if ground.sum() >= 10 else None
        if ff_profile is not None:
            corrector = FfProfileCorrector(ff_profile, i_idle_a=i_idle)
            if i_idle is None:
                warnings.append("地上窓が無いため I_idle はプロファイル iid で代用")
        else:
            warnings.append("FFプロファイル未指定: ΔB̂=0 (非補正) でリプレイ")
        gidx = np.where(ground & np.isfinite(mag_cal).all(axis=1))[0]
        if len(gidx) >= 10:
            # 始動直前 2s 窓 (FF_ANCHOR_WINDOW 相当)
            gidx = gidx[-40:]
            anchor_i = int(gidx[-1])
        else:
            gidx = np.array([0])
            anchor_i = 0
            warnings.append("地上アンカー窓なし: 先頭フレームの b_cal でアンカー近似")
        # アンカーの b_corr (地上は電流≈idle で ΔB̂≈LUT(idle)≈0 だが忠実に計算)
        b0_samples = []
        for i in gidx:
            b_cal_i = mag_cal[i]
            if corrector is not None:
                db, *_ = corrector.compute(t[i], current[i],
                                           {m: duty[m][i] for m in duty},
                                           False, int(ff_mode_log[i]))
            else:
                db = np.zeros(3)
            b0_samples.append(b_cal_i - db - dbias)
        b0 = np.mean(b0_samples, axis=0)
        b0h = level_mag_vector_body(roll[anchor_i], pitch[anchor_i], b0)
        psi0 = yaw_est_log[anchor_i] if math.isfinite(yaw_est_log[anchor_i]) else 0.0
        kf.reanchor(psi0, b0h[0], b0h[1], b0)
        mode = "v6-full"
    else:
        # ---- v5 制限モード ----
        warnings.append(
            "v5制限モード: mag_cal 列なし → ログ済みEKF1状態から観測を合成 "
            "(z=R_z(ψ_est−ψ0)·B0h+b_m_log、ffg受理マスク使用)。"
            "FFプロファイル/magbias は使用しない。実観測イノベーション履歴が"
            "無いため ψ/b_m の配分は再現対象外(軌跡クラスとゲート挙動のみ)")
        psi0 = yaw_est_log[0] if math.isfinite(yaw_est_log[0]) else 0.0
        b0z_virtual = 40.0  # norm ゲートを構成上沈黙させる仮想z成分の基準
        b0 = np.array([b0h_ut, 0.0, b0z_virtual])
        kf.reanchor(psi0, b0h_ut, 0.0, b0)
        mode = "v5-restricted"

    # ---- v5 用: EKF1 磁気更新の fresh 検出列 (NaN 安全に事前計算) ----
    #   nis_ は NIS棄却でも更新される (yaw_estimator_kf.cpp L246)、ffg は
    #   ゲート遷移で変わる。両列とも無いログでは b_m 変化にフォールバック。
    if np.isfinite(nis_log).any() or np.isfinite(ffg_log).any():
        a1 = np.nan_to_num(nis_log, nan=0.0)
        a2 = np.nan_to_num(ffg_log, nan=0.0)
        v5_fresh = np.concatenate(
            [[True], (a1[1:] != a1[:-1]) | (a2[1:] != a2[:-1])])
    else:
        a1 = np.nan_to_num(bm_x_log, nan=0.0)
        a2 = np.nan_to_num(bm_y_log, nan=0.0)
        v5_fresh = np.concatenate(
            [[True], (a1[1:] != a1[:-1]) | (a2[1:] != a2[:-1])])

    # ---- ヨー観測ソース ----
    yaw_obs = np.full(n, np.nan)
    if yaw_obs_mode == "mocap":
        yaw_obs = mocap_true_rad.copy()
    elif yaw_obs_mode not in ("none", ""):
        yaw_obs = col(data, yaw_obs_mode)[frames]
    if align_yaw_obs and np.isfinite(yaw_obs).any():
        # リプレイ座標系(ψ0シード)とヨー基準座標系の定数オフセットを
        # アンカー時点で一致させる(実運用ではファームψが基準系へ収束する。
        # リプレイでは初期過渡を避けるため事前整列する)
        k0 = np.where(np.isfinite(yaw_obs))[0][0]
        off = wrap_pi(float(kf.x[0]) - float(yaw_obs[k0]))
        yaw_obs = yaw_obs + off

    # ---- 時系列ループ ----
    ema = None                     # 補正系EMA (ExpMagFilter, α=0.18)
    last_mag = None
    last_fresh_t = None
    out = {k: np.full(n, np.nan) for k in
           ("psi", "bm_x", "bm_y", "nis", "gate", "status", "yaw_innov",
            "zx", "zy", "db_hat_x", "db_hat_y", "obs_accept", "obs_stopped")}
    fly_since = None
    alt_ok_since = None
    ring: list[tuple[float, np.ndarray]] = []   # 飛行中 b_corr_filt 2s リング

    for k in range(n):
        dt = float(t[k] - t[k - 1]) if k > 0 else 0.04
        kf.predict(r_rate[k] if math.isfinite(r_rate[k]) else 0.0,
                   p_rate[k] if math.isfinite(p_rate[k]) else 0.0,
                   roll[k] if math.isfinite(roll[k]) else 0.0,
                   pitch[k] if math.isfinite(pitch[k]) else 0.0,
                   dt)

        # ---- 磁気観測 ----
        if is_v6:
            b_cal_k = mag_cal[k]
            if np.isfinite(b_cal_k).all():
                fresh = last_mag is None or not np.array_equal(b_cal_k, last_mag)
                if fresh:
                    last_mag = b_cal_k.copy()
                    if corrector is not None:
                        db, s_ff, s_slew, s_diff = corrector.compute(
                            t[k], current[k], {m: duty[m][k] for m in duty},
                            bool(flying[k]) or not duty_off[k],
                            int(ff_mode_log[k]))
                    else:
                        db, s_ff, s_slew, s_diff = np.zeros(3), 0.0, 0.0, 0.0
                    out["db_hat_x"][k] = db[0]
                    out["db_hat_y"][k] = db[1]
                    b_corr = b_cal_k - db - dbias
                    ema = b_corr.copy() if ema is None else \
                        MAG_FILTER_ALPHA * b_corr + (1.0 - MAG_FILTER_ALPHA) * ema
                    mag_dt = (t[k] - last_fresh_t) if last_fresh_t is not None else 0.1
                    last_fresh_t = t[k]
                    kf.update(ema, roll[k], pitch[k], s_ff, s_slew, s_diff, mag_dt)
                    lev = level_mag_vector_body(roll[k], pitch[k], ema)
                    out["zx"][k] = lev[0]
                    out["zy"][k] = lev[1]
                    if flying[k]:
                        ring.append((t[k], ema.copy()))
                        while ring and t[k] - ring[0][0] > FF_EKF2_FLIGHT_ANCHOR_RING_S:
                            ring.pop(0)
        else:
            # v5 合成観測: EKF1 の磁気更新が走った(=nis か ffg が動いた)
            # フレームのみ fresh(b_m は GM 減衰で毎フレーム動くため更新検出に
            # 使えない)。受理マスク: ログ ffg に棄却ビットなし。
            fresh = bool(v5_fresh[k])
            ffg_k = int(ffg_log[k]) if math.isfinite(ffg_log[k]) else 0xFF
            accepted = (ffg_k & GATE_REJECT_MASK) == 0 and (ffg_k & GATE_RECAPTURE) == 0
            if fresh and accepted and math.isfinite(yaw_est_log[k]) and \
                    math.isfinite(bm_x_log[k]):
                beta = wrap_pi(float(yaw_est_log[k]) - psi0)
                zx = math.cos(beta) * b0h_ut + bm_x_log[k]
                zy = math.sin(beta) * b0h_ut + bm_y_log[k]
                # norm ゲートを構成上沈黙させる合成z成分 (‖b‖=‖B0‖ を保つ)
                zz2 = kf.b0_norm ** 2 - zx * zx - zy * zy
                zz = math.sqrt(max(zz2, 0.0))
                mag_dt = (t[k] - last_fresh_t) if last_fresh_t is not None else 0.1
                last_fresh_t = t[k]
                # σ_ff はログ済み ΔB̂ から自己申告を再構成
                dbx = db_hat_x_log[k]
                dby = db_hat_y_log[k]
                s_ff = FF_KAPPA_FF * math.hypot(dbx, dby) \
                    if math.isfinite(dbx) and math.isfinite(dby) else 0.0
                # 合成観測は既にレベル化済み → roll=pitch=0 で update
                kf.update(np.array([zx, zy, zz]), 0.0, 0.0, s_ff, 0.0, 0.0, mag_dt)
                out["zx"][k] = zx
                out["zy"][k] = zy

        # ---- ヨー擬似観測 (bit3有効+age<100ms 相当 = フレーム毎) ----
        if math.isfinite(yaw_obs[k]):
            acc = kf.update_yaw_obs(float(yaw_obs[k]), low_trust)
            out["yaw_innov"][k] = kf.yaw_obs_innov
            out["obs_accept"][k] = 1.0 if acc else 0.0
            out["obs_stopped"][k] = 1.0 if kf.yaw_obs_stopped else 0.0

        # ---- 飛行状態再アンカー (契約 §2.1(3)) ----
        if enable_flight_anchor and is_v6 and not kf.flight_anchor_done:
            if flying[k]:
                if fly_since is None:
                    fly_since = t[k]
                alt_ok = (math.isfinite(alt_est[k]) and math.isfinite(alt_ref[k])
                          and abs(alt_est[k] - alt_ref[k])
                          < FF_EKF2_FLIGHT_ANCHOR_ALT_TOL_M)
                if alt_ok:
                    if alt_ok_since is None:
                        alt_ok_since = t[k]
                else:
                    alt_ok_since = None
                yaw_fused = kf.yaw_obs_last_accept_s < FF_EKF2_YAW_FUSED_WINDOW_S
                if (t[k] - fly_since > FF_EKF2_FLIGHT_ANCHOR_MIN_FLY_S and
                        alt_ok_since is not None and
                        t[k] - alt_ok_since >= FF_EKF2_FLIGHT_ANCHOR_ALT_HOLD_S and
                        math.isfinite(current[k]) and
                        current[k] > FF_EKF2_FLIGHT_ANCHOR_MIN_CURRENT_A and
                        yaw_fused and
                        ring and t[k] - ring[0][0] >= FF_EKF2_FLIGHT_ANCHOR_RING_S * 0.9):
                    b0f_body = np.mean([b for _, b in ring], axis=0)
                    b0f_lev = level_mag_vector_body(roll[k], pitch[k], b0f_body)
                    kf.flight_reanchor(b0f_lev[0], b0f_lev[1], b0f_body)
            else:
                fly_since = None
                alt_ok_since = None

        out["psi"][k] = kf.x[0]
        out["bm_x"][k] = kf.x[2]
        out["bm_y"][k] = kf.x[3]
        out["nis"][k] = kf.nis
        out["gate"][k] = kf.gate_bits
        out["status"][k] = kf.status_bits()

    if ekf1_compat:
        mode += "+ekf1-compat"
    result = {
        "t_s": t, "frames": frames, "mode": mode, "warnings": warnings,
        "psi_ekf2_rad": out["psi"], "bm_x_ut": out["bm_x"], "bm_y_ut": out["bm_y"],
        "nis": out["nis"], "gate": out["gate"], "status": out["status"],
        "yaw_innov_rad": out["yaw_innov"],
        "obs_accept": out["obs_accept"], "obs_stopped": out["obs_stopped"],
        "align_innov_rad": kf.align_event_innov_rad,
        "zx_ut": out["zx"], "zy_ut": out["zy"],
        "db_hat_x_ut": out["db_hat_x"], "db_hat_y_ut": out["db_hat_y"],
        "psi_ekf1_log_rad": yaw_est_log, "mocap_true_rad": mocap_true_rad,
        "psi_ekf2_log_rad": col(data, "tlm_ekf2_yaw_rad")[frames],
        "flying": flying,
    }
    result["stats"] = _stats(result)
    return result


def _wrap_arr(a):
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def _stats(res: dict) -> dict:
    stats = {"mode": res["mode"]}
    fly = res["flying"]

    def rms_deg(a, b, mask):
        m = mask & np.isfinite(a) & np.isfinite(b)
        if not m.any():
            return None
        return float(np.degrees(np.sqrt(np.mean(_wrap_arr(a[m] - b[m]) ** 2))))

    # mocap 系とリプレイ系の定数オフセットは除去して形状RMSを見る
    psi = res["psi_ekf2_rad"]
    mc = res["mocap_true_rad"]
    m = fly & np.isfinite(psi) & np.isfinite(mc)
    if m.any():
        off = math.atan2(np.mean(np.sin(psi[m] - mc[m])),
                         np.mean(np.cos(psi[m] - mc[m])))
        stats["rms_vs_mocap_deg"] = rms_deg(psi, mc + off, fly)
        stats["mocap_offset_deg"] = float(np.degrees(off))
    stats["rms_vs_ekf1_log_deg"] = rms_deg(psi, res["psi_ekf1_log_rad"], fly)
    if np.isfinite(res["psi_ekf2_log_rad"]).any():
        stats["rms_vs_ekf2_log_deg"] = rms_deg(psi, res["psi_ekf2_log_rad"], fly)
    bmf = np.isfinite(res["bm_x_ut"])
    if bmf.any():
        stats["bm_final_ut"] = [float(res["bm_x_ut"][bmf][-1]),
                                float(res["bm_y_ut"][bmf][-1])]
    g = np.nan_to_num(res["gate"], nan=0.0).astype(np.int64)
    stats["gate_reject_ratio"] = float(np.mean((g & GATE_REJECT_MASK) != 0))
    return stats


# ===========================================================================
# 出力
# ===========================================================================
def write_outputs(res: dict, stem: str, out_dir: Path) -> Path:
    out_dir.mkdir(parents=True, exist_ok=True)
    csv_path = out_dir / f"ekf2_replay_{stem}.csv"
    cols = ["t_s", "psi_ekf2_rad", "bm_x_ut", "bm_y_ut", "nis", "gate",
            "status", "yaw_innov_rad", "zx_ut", "zy_ut",
            "db_hat_x_ut", "db_hat_y_ut",
            "psi_ekf1_log_rad", "psi_ekf2_log_rad", "mocap_true_rad"]
    with csv_path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(cols)
        n = len(res["t_s"])
        for k in range(n):
            row = []
            for c in cols:
                v = res[c][k]
                row.append("" if not math.isfinite(v) else f"{v:.6f}")
            w.writerow(row)
    return csv_path


def _setup_matplotlib():
    """日本語フォント設定 (plot_explog.py と同じ選定ロジック)。"""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    installed = {f.name for f in fm.fontManager.ttflist}
    for jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic",
               "Yu Gothic", "Noto Sans CJK JP", "IPAexGothic"):
        if jp in installed:
            plt.rcParams["font.family"] = jp
            break
    plt.rcParams["axes.unicode_minus"] = False
    return plt


def save_plot(res: dict, stem: str, out_dir: Path) -> Path:
    plt = _setup_matplotlib()

    out_dir.mkdir(parents=True, exist_ok=True)
    t = res["t_s"]
    t0 = t[0]
    fig, axes = plt.subplots(3, 1, figsize=(12, 9), sharex=True)

    ax = axes[0]
    mc = res["mocap_true_rad"]
    if np.isfinite(mc).any():
        off = math.radians(res["stats"].get("mocap_offset_deg") or 0.0)
        ax.plot(t - t0, np.degrees(_wrap_arr(mc + off)), color="k", lw=1.2,
                alpha=0.6, label="mocap真値 (整列後)")
    ax.plot(t - t0, np.degrees(_wrap_arr(res["psi_ekf1_log_rad"])), lw=0.9,
            label="EKF1 (ログ)")
    if np.isfinite(res["psi_ekf2_log_rad"]).any():
        ax.plot(t - t0, np.degrees(_wrap_arr(res["psi_ekf2_log_rad"])), lw=0.9,
                label="EKF2 (ログ)")
    ax.plot(t - t0, np.degrees(_wrap_arr(res["psi_ekf2_rad"])), lw=0.9,
            label="EKF2 (リプレイ)")
    st = res["stats"]
    title = f"EKF2 リプレイ: {stem} [{res['mode']}]"
    if st.get("rms_vs_mocap_deg") is not None:
        title += f"  RMS vs mocap {st['rms_vs_mocap_deg']:.2f}°"
    if st.get("rms_vs_ekf1_log_deg") is not None:
        title += f" / vs EKF1ログ {st['rms_vs_ekf1_log_deg']:.2f}°"
    ax.set_title(title, fontsize=10)
    ax.set_ylabel("yaw [deg]")
    ax.legend(fontsize=8, ncol=4)

    ax = axes[1]
    ax.plot(t - t0, res["bm_x_ut"], lw=0.9, label="b_mx (リプレイ)")
    ax.plot(t - t0, res["bm_y_ut"], lw=0.9, label="b_my (リプレイ)")
    ax.set_ylabel("b_m [uT]")
    ax.legend(fontsize=8)

    ax = axes[2]
    ax.plot(t - t0, res["nis"], lw=0.7, label="NIS")
    ax.axhline(FF_EKF_NIS_INFLATE, color="orange", ls=":", lw=1)
    ax.axhline(FF_EKF_NIS_REJECT, color="r", ls=":", lw=1)
    g = np.nan_to_num(res["gate"], nan=0.0).astype(np.int64)
    rej = (g & GATE_REJECT_MASK) != 0
    if rej.any():
        ax.plot((t - t0)[rej], np.full(rej.sum(), -1.0), "|", color="r",
                ms=6, label="reject")
    ax.set_ylabel("NIS")
    ax.set_xlabel("t [s]")
    ax.set_yscale("symlog")
    ax.legend(fontsize=8)

    fig.tight_layout()
    png = out_dir / f"ekf2_replay_{stem}.png"
    fig.savefig(png, dpi=110)
    plt.close(fig)
    return png


# ===========================================================================
# CLI
# ===========================================================================
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="EKF2 Python リプレイ (契約 §2.1/§4)")
    ap.add_argument("log", help="飛行ログ CSV (v6=フル再構成 / v5=制限モード)")
    ap.add_argument("--ff-profile", help="FFプロファイルJSON (v6再構成用)")
    ap.add_argument("--magbias", help="magbias プロファイルJSON (Δb適用)")
    ap.add_argument("--yaw-obs", default="none",
                    help="ヨー観測: none | mocap | <rad列名> (既定 none)")
    ap.add_argument("--low-trust", action="store_true",
                    help="ヨー観測を低信頼 R_ψ=(6°)² で融合")
    ap.add_argument("--b0h-ut", type=float, default=27.9,
                    help="v5制限モードの仮想 |B0h| [µT] (既定 27.9 = 7/27解析の暗黙値)")
    ap.add_argument("--no-flight-anchor", action="store_true",
                    help="飛行状態再アンカー (§2.1(3)) を無効化")
    ap.add_argument("--ekf1-mode", action="store_true",
                    help="EKF1互換 (τ_bm GM減衰・固定q_bm)。受入基準#6 の再現検証用")
    ap.add_argument("-o", "--out", help="出力dir (既定 graphs/ekf2_replay_<stem>/)")
    args = ap.parse_args(argv)

    log_path = Path(args.log).expanduser()
    if not log_path.is_file():
        print(f"ログが見つかりません: {log_path}", file=sys.stderr)
        return 2
    stem = log_path.stem

    ff_profile = None
    if args.ff_profile:
        ff_profile = json.loads(Path(args.ff_profile).read_text(encoding="utf-8"))
        if ff_profile.get("schema") != "stampfly_ff_profile":
            print("--ff-profile が stampfly_ff_profile ではありません",
                  file=sys.stderr)
            return 2
    magbias = None
    if args.magbias:
        magbias = json.loads(Path(args.magbias).read_text(encoding="utf-8"))
        if magbias.get("schema") != "stampfly_magbias_profile":
            print("--magbias が stampfly_magbias_profile ではありません",
                  file=sys.stderr)
            return 2

    try:
        data = load_flight_log(log_path)
        res = run_replay(data, ff_profile=ff_profile, magbias=magbias,
                         yaw_obs_mode=args.yaw_obs, low_trust=args.low_trust,
                         b0h_ut=args.b0h_ut,
                         enable_flight_anchor=not args.no_flight_anchor,
                         ekf1_compat=args.ekf1_mode)
    except ValueError as exc:
        print(f"リプレイ失敗: {exc}", file=sys.stderr)
        return 1

    out_dir = Path(args.out) if args.out else GRAPH_DIR / f"ekf2_replay_{stem}"
    csv_path = write_outputs(res, stem, out_dir)
    png_path = save_plot(res, stem, out_dir)

    print(f"EKF2 リプレイ完了 [{res['mode']}] → {csv_path}")
    print(f"比較図: {png_path}")
    for key, val in res["stats"].items():
        print(f"  {key}: {val}")
    for w in res["warnings"]:
        print(f"  警告: {w}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
