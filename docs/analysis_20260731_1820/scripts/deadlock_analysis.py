#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
ヨー観測「融合ゼロ」機構の確定解析:
 1. 18:20 イノベーション時系列の完全分解 + ラッチ発動時刻推定
 2. 11:19 vs 18:20 の初期オフセット比較(アーム時 EKF/真値/送信基準)
 3. 回頭中の磁気EKF挙動と RMS80° アーチファクトの分解
図は plots/ へ、数値は deadlock_stats.json へ。
"""
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
matplotlib.rcParams["font.family"] = ["Hiragino Sans", "Hiragino Kaku Gothic Pro",
                                      "AppleGothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

BASE = "/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731b"
PLOTS = BASE + "/plots"
REPO = "/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/StampFly_MoCap_System_v2/StampFly_Integrated_Control_V3"
LOG1820C = BASE + "/log_corrected.csv"
LOG1119 = REPO + "/logs/flight_logs/20260731_111906_position.csv"

import os
os.makedirs(PLOTS, exist_ok=True)

def wrap(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0

R2D = 180.0 / np.pi
S = {}

# ============================================================
# 1. 18:20 innovation full decomposition
# ============================================================
df = pd.read_csv(LOG1820C)
t = df["elapsed_time"].values
innov = df["tlm_ekf2_yaw_innov_rad"].values * R2D
status = df["tlm_ekf2_status"].values.astype(int)
glitch = df["mocap_glitch"].values.astype(int)
truth = df["yaw_true_corrected_deg"].values
ekf2 = df["tlm_ekf2_yaw_rad"].values * R2D
ekf1 = df["tlm_yaw_est_rad"].values * R2D
sent = df["yaw_ref_sent_rad"].values * R2D
ref_corr = df["yaw_ref_corrected_rad"].values * R2D
phase = df["phase"].astype(str).values
tstate = df["tlm_state"].values.astype(int)

t_arm = t[np.argmax(phase != phase[0])] if phase[0] == "init" else 0.063
# arm detection via phase column
armed_idx = np.where((phase == "armed") | (phase == "flying"))[0]
t_arm = t[armed_idx[0]] if len(armed_idx) else 0.063

# --- observation-tick extraction (innov changes on each accepted staging @~23.5Hz) ---
chg = np.ones(len(df), dtype=bool)
chg[1:] = np.abs(np.diff(innov)) > 1e-9
obs_idx = np.where(chg)[0]
obs_t = t[obs_idx]
obs_innov = innov[obs_idx]
obs_glitch = glitch[obs_idx]
in_gate = np.abs(obs_innov) <= 30.0

# --- reject-counter / latch simulation (FW: 25 consecutive rejects -> permanent latch) ---
cnt = 0
latch_i = None
cnt_series = np.zeros(len(obs_idx), dtype=int)
for k in range(len(obs_idx)):
    if not in_gate[k]:
        cnt += 1
    else:
        if latch_i is None:
            cnt = 0
    cnt_series[k] = cnt
    if cnt >= 25 and latch_i is None:
        latch_i = k
t_latch = obs_t[latch_i] if latch_i is not None else np.nan
S["obs"] = dict(
    n_obs=int(len(obs_idx)),
    rate_hz=float(len(obs_idx) / (obs_t[-1] - obs_t[0])),
    first_obs_t=float(obs_t[0]),
    first_obs_innov=float(obs_innov[0]),
    innov_first10=[float(x) for x in obs_innov[:10]],
    n_in_gate=int(in_gate.sum()),
    latch_obs_index=int(latch_i),
    t_latch=float(t_latch),
    t_arm=float(t_arm),
    latch_before_liftoff=bool(t_latch < 3.422),
    rejects_before_latch_all_out=bool((~in_gate[: latch_i + 1]).all()),
    first_ingate_t=float(obs_t[in_gate][0]) if in_gate.any() else None,
    n_ingate_after_latch=int(in_gate[latch_i + 1 :].sum()),
    ingate_glitch_frac=float(obs_glitch[in_gate].mean()) if in_gate.any() else None,
)
# innov piecewise stats (clean rows only)
clean = obs_glitch == 0
for name, m in [("clean", clean), ("glitch", ~clean)]:
    x = obs_innov[m]
    S["obs"][f"innov_{name}"] = dict(
        n=int(m.sum()), mean=float(x.mean()), std=float(x.std()),
        min=float(x.min()), max=float(x.max()))

# would-be innov with corrected reference
wb = wrap(ref_corr - ekf2)
S["obs"]["wouldbe"] = dict(mean=float(np.mean(wb)), rms=float(np.sqrt(np.mean(wb**2))),
                           maxabs=float(np.max(np.abs(wb))),
                           frac_in_gate=float(np.mean(np.abs(wb) <= 30.0)))

# status bits confirm: bit1 (fused) ever set?
S["obs"]["status_values"] = {str(v): int(c) for v, c in
                             zip(*np.unique(status, return_counts=True))}
S["obs"]["fused_bit_ever"] = bool((status & 2).any())

# longest out-of-gate (=reject) runs after latch — glitch-runs would re-latch scenario
runs = []
c = 0
for k in range(latch_i + 1, len(obs_idx)):
    if in_gate[k]:
        c += 1
    else:
        if c:
            runs.append(c)
        c = 0
if c:
    runs.append(c)
S["obs"]["ingate_runs_after_latch_top"] = sorted(runs)[-5:] if runs else []

# consecutive OUT-of-gate runs among post-latch obs (for future re-latch risk with corrected ref)
# with corrected ref, glitch obs are ~+95 deg off -> out of gate
wb_obs = wb[obs_idx]
out_wb = np.abs(wb_obs) > 30.0
runs_wb = []
c = 0
for k in range(len(obs_idx)):
    if out_wb[k]:
        c += 1
    else:
        if c:
            runs_wb.append((c, float(obs_t[k - c])))
        c = 0
if c:
    runs_wb.append((c, float(obs_t[len(obs_idx) - c])))
S["obs"]["corrected_ref_out_of_gate_runs_over_10"] = [r for r in runs_wb if r[0] >= 10]
# NOTE: wb uses corrected(clean) ref; for glitch-latch risk use raw glitch offset vs truth
glitch_off = wrap(sent - ref_corr)  # == -(90.434) const; for glitch rows sent jumps +~95
wb_glitch_view = wrap((sent - ref_corr) + 90.434)  # deviation of sent from its own baseline
gl_out = np.abs(wb_glitch_view[obs_idx]) > 30.0
runs_gl = []
c = 0
for k in range(len(obs_idx)):
    if gl_out[k]:
        c += 1
    else:
        if c:
            runs_gl.append((c, float(obs_t[k - c])))
        c = 0
if c:
    runs_gl.append((c, float(obs_t[len(obs_idx) - c])))
S["obs"]["glitch_reject_runs_ge_25_if_ref_fixed"] = [r for r in runs_gl if r[0] >= 25]

# ============================================================
# 2. 11:19 comparison — initial offsets
# ============================================================
d1 = pd.read_csv(LOG1119)
t1 = d1["elapsed_time"].values
p1 = d1["phase"].astype(str).values
a1 = np.where((p1 == "armed") | (p1 == "flying"))[0]
t1_arm = t1[a1[0]] if len(a1) else 0.0
heading1 = d1["mocap_heading_deg"].values
sent1 = d1["yaw_ref_sent_rad"].values * R2D
innov1 = d1["tlm_ekf2_yaw_innov_rad"].values * R2D
ekf1_1 = d1["tlm_yaw_est_rad"].values * R2D
ekf2_1 = d1["tlm_ekf2_yaw_rad"].values * R2D
status1 = d1["tlm_ekf2_status"].values.astype(int)
C11 = 88.62  # truth offset from FLIGHT_ANALYSIS_20260731.md (ground-window anchored)
truth1 = wrap(-heading1 + C11)

# wire sign check for 11:19: sent == -heading? or +heading?
rms_minus = np.sqrt(np.mean(wrap(sent1 - (-heading1)) ** 2))
rms_plus = np.sqrt(np.mean(wrap(sent1 - (+heading1)) ** 2))
S["f1119"] = dict(
    t_arm=float(t1_arm),
    sent_eq_minus_heading_rms=float(rms_minus),
    sent_eq_plus_heading_rms=float(rms_plus),
)
# innov obs extraction for 11:19
chg1 = np.ones(len(d1), dtype=bool)
chg1[1:] = np.abs(np.diff(innov1)) > 1e-9
o1 = np.where(chg1)[0]
in_gate1 = np.abs(innov1[o1]) <= 30.0
cnt = 0
latch1_i = None
for k in range(len(o1)):
    cnt = cnt + 1 if not in_gate1[k] else 0
    if cnt >= 25 and latch1_i is None:
        latch1_i = k
S["f1119"].update(
    n_obs=int(len(o1)),
    innov_first5=[float(x) for x in innov1[o1][:5]],
    innov_mean=float(innov1[o1].mean()),
    innov_min=float(innov1[o1].min()),
    innov_max=float(innov1[o1].max()),
    frac_in_gate=float(in_gate1.mean()),
    t_latch=float(t1[o1][latch1_i]) if latch1_i is not None else None,
    fused_bit_ever=bool((status1 & 2).any()),
)
# arm-time initial offsets (mean over ground window t_arm..t_arm+1.5s)
def win_mean(tv, x, t0, t1_):
    m = (tv >= t0) & (tv <= t1_)
    return float(wrap(x[m]).mean()) if m.any() else np.nan

g1 = (t1 >= t1_arm + 0.1) & (t1 <= t1_arm + 1.9)
g2 = (t >= t_arm + 0.1) & (t <= t_arm + 1.9) & (glitch == 0)
S["init_compare"] = dict(
    f1119=dict(
        heading_at_arm=float(np.mean(heading1[g1])),
        truth_at_arm=float(np.mean(truth1[g1])),
        ekf1_minus_truth=float(np.mean(wrap(ekf1_1[g1] - truth1[g1]))),
        ekf2_minus_truth=float(np.mean(wrap(ekf2_1[g1] - truth1[g1]))),
        sent_minus_truth=float(np.mean(wrap(sent1[g1] - truth1[g1]))),
        innov_ground_mean=float(np.mean(innov1[g1])),
        offset_c_deg=C11,
    ),
    f1820=dict(
        heading_at_arm=float(np.mean(df["mocap_heading_deg"].values[g2])),
        truth_at_arm=float(np.mean(truth[g2])),
        ekf1_minus_truth=float(np.mean(wrap(ekf1[g2] - truth[g2]))),
        ekf2_minus_truth=float(np.mean(wrap(ekf2[g2] - truth[g2]))),
        sent_minus_truth=float(np.mean(wrap(sent[g2] - truth[g2]))),
        innov_ground_mean=float(np.mean(innov[g2])),
        offset_c_deg=90.434,
    ),
)

# EKF yaw value itself at arm (frame origin check: ~0 => boot heading == arm heading)
S["init_compare"]["f1119"]["ekf2_yaw_at_arm"] = float(np.mean(wrap(ekf2_1[g1])))
S["init_compare"]["f1820"]["ekf2_yaw_at_arm"] = float(np.mean(wrap(ekf2[g2])))

# ============================================================
# 3. turn-phase EKF behavior + RMS80 artifact decomposition
# ============================================================
myt = df["mocap_yaw_true_deg"].values  # contaminated logged column
fly = (tstate == 4) | (tstate == 5)
err_contam = wrap(ekf1[fly] - myt[fly])
err_clean_rows = fly & (glitch == 0)
err_clean = wrap(ekf1[err_clean_rows] - truth[err_clean_rows])
S["artifact"] = dict(
    ekf1_vs_logged_col_rms=float(np.sqrt(np.mean(err_contam**2))),
    ekf1_vs_corrected_truth_rms=float(np.sqrt(np.mean(err_clean**2))),
    contaminated_rows_frac=float(np.mean(glitch[fly])),
)
# decompose: on glitch rows, logged col deviates by ~-95 or ~+85 (180-flip error)
gl_fly = fly & (glitch == 1)
dev = wrap(myt[gl_fly] - truth[gl_fly])
S["artifact"]["glitch_row_dev_median"] = float(np.median(np.abs(dev)))
S["artifact"]["glitch_row_dev_over_60_frac"] = float(np.mean(np.abs(dev) > 60))

# turn lag: effective delay of EKF1/EKF2 vs truth during turn windows
def eff_delay(tv, a, b, m, max_lag_s=0.6):
    # cross-correlate derivatives on clean rows within mask
    tv2, a2, b2 = tv[m], a[m], b[m]
    if len(tv2) < 50:
        return np.nan
    dt = np.median(np.diff(tv2))
    da = np.gradient(np.unwrap(np.radians(a2)))
    db = np.gradient(np.unwrap(np.radians(b2)))
    lags = np.arange(0, int(max_lag_s / dt))
    best, bl = -2, 0
    for L in lags:
        if L == 0:
            c = np.corrcoef(da, db)[0, 1]
        else:
            c = np.corrcoef(da[L:], db[:-L])[0, 1]
        if c > best:
            best, bl = c, L
    return float(bl * dt)

turn_m = ((t >= 23.0) & (t <= 33.0)) | ((t >= 36.8) & (t <= 47.0))
cl = glitch == 0
S["turn"] = dict(
    ekf1_delay_s=eff_delay(t, ekf1, truth, turn_m & cl),
    ekf2_delay_s=eff_delay(t, ekf2, truth, turn_m & cl),
)
# b_m during turns
for tag, bx, by in [("ekf1", "tlm_bm_x_ut", "tlm_bm_y_ut"),
                    ("ekf2", "tlm_ekf2_bm_x_ut", "tlm_ekf2_bm_y_ut")]:
    bm = np.hypot(df[bx].values, df[by].values)
    S["turn"][f"{tag}_bm_norm_pre_turn"] = float(np.mean(bm[(t > 20) & (t < 23)]))
    S["turn"][f"{tag}_bm_norm_post_turn"] = float(np.mean(bm[(t > 47) & (t < 50)]))
    S["turn"][f"{tag}_bm_norm_max_turn"] = float(np.max(bm[turn_m]))

# z-gate / gate bits during turns
gate2 = df["tlm_ekf2_gate"].values.astype(int)
S["turn"]["z_reject_frac_turn"] = float(np.mean((gate2[turn_m] & 8) > 0))
S["turn"]["z_reject_frac_hover"] = float(np.mean((gate2[fly & ~turn_m] & 8) > 0))
S["turn"]["nis_reject_frac_turn"] = float(np.mean((gate2[turn_m] & 2) > 0))
S["turn"]["recapture_rows_t"] = [float(x) for x in t[(gate2 & 128) > 0][:3]] + \
                                [float(t[(gate2 & 128) > 0][-1])]

# ============================================================
# FIGURE 1: innovation timeline + gate + latch
# ============================================================
fig, axes = plt.subplots(3, 1, figsize=(13, 11),
                         gridspec_kw=dict(height_ratios=[2.2, 1.4, 1.2]))
ax = axes[0]
ax.axhspan(-30, 30, color="tab:green", alpha=0.12, label="±30° ゲート")
m0 = obs_glitch == 0
ax.plot(obs_t[m0], obs_innov[m0], ".", ms=3, color="tab:red",
        label="innov(クリーンMoCap): 常時 −90°帯 → 全棄却")
ax.plot(obs_t[~m0], obs_innov[~m0], ".", ms=3, color="tab:orange",
        label="innov(+95°グリッチ観測): 偶然ゲート内 25.1%")
ax.plot(t, wb, "-", lw=1.0, color="tab:blue", alpha=0.8,
        label="would-be innov(正基準なら): 100%ゲート内")
ax.axvline(t_latch, color="k", ls="--", lw=1.5)
ax.annotate(f"融合停止ラッチ t={t_latch:.2f}s\n(25連続棄却)", (t_latch, 150),
            xytext=(t_latch + 1.5, 150), fontsize=10,
            arrowprops=dict(arrowstyle="->"))
ax.axvline(t_arm, color="gray", ls=":", lw=1)
ax.text(t_arm, 178, " アーム", va="top", fontsize=9, color="gray")
ax.axvline(3.422, color="gray", ls=":", lw=1)
ax.text(3.422, 178, " 離陸", va="top", fontsize=9, color="gray")
ax.set_ylim(-185, 185)
ax.set_ylabel("ヨー観測イノベーション [deg]")
ax.set_title("18:20飛行: EKF2ヨー観測イノベーション時系列 — t=0から−90°(ブートストラップ・デッドロック)")
ax.legend(loc="lower right", fontsize=9)
ax.grid(alpha=0.3)

ax = axes[1]
mzoom = obs_t <= 3.5
ax.axhspan(-30, 30, color="tab:green", alpha=0.12)
ax.plot(obs_t[mzoom], obs_innov[mzoom], "o-", ms=4, lw=0.7, color="tab:red", label="innov(観測tick)")
ax2 = ax.twinx()
ax2.step(obs_t[mzoom], cnt_series[mzoom], where="post", color="tab:purple", lw=1.5,
         label="連続棄却カウンタ")
ax2.axhline(25, color="tab:purple", ls="--", lw=1)
ax2.text(0.05, 25.7, "ラッチ閾値 N=25", color="tab:purple", fontsize=9)
ax2.set_ylabel("連続棄却カウンタ", color="tab:purple")
ax2.set_ylim(0, 40)
ax.axvline(t_latch, color="k", ls="--", lw=1.5)
ax.axvline(t_arm, color="gray", ls=":", lw=1)
ax.set_ylim(-120, 40)
ax.set_ylabel("innov [deg]")
ax.set_title(f"ズーム t=0–3.5s: 初回観測 t={obs_t[0]:.2f}s から innov≈−90° → "
             f"25個目の観測 t={t_latch:.2f}s で恒久ラッチ(離陸 t=3.42s の前)")
ax.legend(loc="lower left", fontsize=9)
ax.grid(alpha=0.3)

ax = axes[2]
ax.axhspan(-30, 30, color="tab:green", alpha=0.12)
ax.plot(t1[o1], innov1[o1], ".", ms=2.5, color="tab:red", label="11:19 innov")
ax.plot(obs_t, obs_innov, ".", ms=2.5, color="tab:brown", alpha=0.45, label="18:20 innov")
ax.axhline(-88.6, color="tab:red", ls=":", lw=1)
ax.axhline(-90.4, color="tab:brown", ls=":", lw=1)
ax.set_ylim(-185, 185)
ax.set_xlabel("t [s]")
ax.set_ylabel("innov [deg]")
ax.set_title("両フライト比較: 11:19も18:20も初回観測から innov≈−89/−90°(=送信基準のオフセット欠落。開始オフセットは同一)")
ax.legend(loc="upper right", fontsize=9)
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(PLOTS + "/innov_gate_latch_timeline.png", dpi=130)
plt.close(fig)

# ============================================================
# FIGURE 2: initial offset budget comparison
# ============================================================
fig, axes = plt.subplots(1, 2, figsize=(13, 5))
ax = axes[0]
labels = ["11:19", "18:20"]
ic = S["init_compare"]
sent_off = [ic["f1119"]["sent_minus_truth"], ic["f1820"]["sent_minus_truth"]]
ekf_off = [ic["f1119"]["ekf2_minus_truth"], ic["f1820"]["ekf2_minus_truth"]]
x = np.arange(2)
ax.bar(x - 0.18, sent_off, 0.32, color="tab:red", label="送信基準 − 真値(=未適用オフセット −c)")
ax.bar(x + 0.18, ekf_off, 0.32, color="tab:blue", label="EKF2 − 真値(アーム時。EKFは健全)")
for i, v in enumerate(sent_off):
    ax.text(i - 0.18, v - 4, f"{v:.1f}°", ha="center", va="top", fontsize=10)
for i, v in enumerate(ekf_off):
    ax.text(i + 0.18, v + 1.5, f"{v:.2f}°", ha="center", fontsize=10)
ax.axhspan(-30, 30, color="tab:green", alpha=0.12)
ax.text(1.35, 25, "±30°ゲート", fontsize=9, color="green")
ax.set_xticks(x, labels)
ax.set_ylabel("アーム時オフセット [deg]")
ax.set_title("アーム時の初期オフセット収支: 両フライトとも送信基準が −89/−90°\n(機体の置き方はほぼ同一・EKF初期誤差は<1°)")
ax.legend(fontsize=9, loc="lower right")
ax.grid(alpha=0.3, axis="y")

ax = axes[1]
mz1 = (t1 >= 0) & (t1 <= 5)
mz2 = (t >= 0) & (t <= 5)
ax.axhspan(-30, 30, color="tab:green", alpha=0.12)
ax.plot(t1[mz1], innov1[mz1], "-", color="tab:red", lw=1.2, label="11:19 innov(latch t≈1.1s)")
ax.plot(t[mz2], innov[mz2], "-", color="tab:brown", lw=1.2, label="18:20 innov(latch t=%.2fs)" % t_latch)
ax.axvline(S["f1119"]["t_latch"], color="tab:red", ls="--", lw=1)
ax.axvline(t_latch, color="tab:brown", ls="--", lw=1)
ax.set_xlabel("t [s]")
ax.set_ylabel("innov [deg]")
ax.set_ylim(-120, 40)
ax.set_title("最初の5秒: 両フライトとも初回観測から一定の約−90°\n→ 途中発散ではなくフレーム定数オフセット")
ax.legend(fontsize=9)
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(PLOTS + "/initial_offset_comparison.png", dpi=130)
plt.close(fig)

# ============================================================
# FIGURE 3: turn-phase EKF error + b_m + recapture
# ============================================================
fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
ax = axes[0]
mcl = (glitch == 0) & fly
e1 = wrap(ekf1 - truth)
e2 = wrap(ekf2 - truth)
ax.plot(t[mcl], e1[mcl], ".", ms=2, color="tab:blue", label="EKF1 − 真値")
ax.plot(t[mcl], e2[mcl], ".", ms=2, color="tab:red", label="EKF2 − 真値")
for t0_, t1__ in [(23.0, 26.3), (30.9, 33.0), (36.8, 40.0), (43.9, 46.8)]:
    ax.axvspan(t0_, t1__, color="gray", alpha=0.12)
rec = (gate2 & 128) > 0
ax.plot(t[rec], np.full(rec.sum(), 22), "v", ms=4, color="tab:green",
        label="磁気ソフト再捕捉 bit7(EKF2)")
ax.set_ylabel("ヨー誤差 [deg]")
ax.set_ylim(-30, 30)
ax.set_title("回頭中のEKF挙動(クリーン行のみ。灰色帯=回頭。EKF1飛行RMS 13.1° — 「80°」は汚染列アーチファクト)")
ax.legend(fontsize=9, loc="lower right")
ax.grid(alpha=0.3)

ax = axes[1]
ax.plot(t, np.hypot(df["tlm_bm_x_ut"], df["tlm_bm_y_ut"]), lw=1, color="tab:blue", label="|b_m| EKF1")
ax.plot(t, np.hypot(df["tlm_ekf2_bm_x_ut"], df["tlm_ekf2_bm_y_ut"]), lw=1, color="tab:red", label="|b_m| EKF2")
for t0_, t1__ in [(23.0, 26.3), (30.9, 33.0), (36.8, 40.0), (43.9, 46.8)]:
    ax.axvspan(t0_, t1__, color="gray", alpha=0.12)
ax.set_ylabel("|b_m| [µT]")
ax.legend(fontsize=9)
ax.grid(alpha=0.3)

ax = axes[2]
ax.plot(t, truth, lw=1, color="k", label="真値ヨー")
ax.plot(t, ekf1, lw=0.8, color="tab:blue", alpha=0.7, label="EKF1")
ax.plot(t, wrap(df["cmd_yaw_ref_rad"].values * R2D), lw=0.8, color="tab:green", alpha=0.8, label="cmd_yaw_ref")
ax.set_ylabel("yaw [deg]")
ax.set_xlabel("t [s]")
ax.legend(fontsize=9)
ax.grid(alpha=0.3)
fig.tight_layout()
fig.savefig(PLOTS + "/turn_ekf_behavior.png", dpi=130)
plt.close(fig)

with open(BASE + "/deadlock_stats.json", "w") as f:
    json.dump(S, f, indent=1, ensure_ascii=False)
print(json.dumps(S, indent=1, ensure_ascii=False))
