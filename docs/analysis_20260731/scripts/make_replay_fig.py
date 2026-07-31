#!/usr/bin/env python3
"""7/31飛行のEKF2リプレイ救済解析 まとめ図 (4パネル)."""
import sys, math, json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

BASE = "/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731"
sys.path.insert(0, "/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/StampFly_MoCap_System_v2/StampFly_Integrated_Control_V3/data_analysis/replay")

installed = {f.name for f in fm.fontManager.ttflist}
for jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic", "Noto Sans CJK JP"):
    if jp in installed:
        plt.rcParams["font.family"] = jp
        break
plt.rcParams["axes.unicode_minus"] = False

# palette (dataviz reference, light mode)
SURF = "#fcfcfb"; INK = "#0b0b0b"; INK2 = "#52514e"; GRID = "#e8e8e6"
C_A = "#2a78d6"     # slot1 blue  : replay(a) 正解基準
C_E1 = "#eb6834"    # slot2 orange: logged EKF1
C_E2 = "#1baf7a"    # slot3 aqua  : logged EKF2
C_B = "#eda100"     # slot4 yellow: replay(b) コースト (要直ラベル)

wrap = lambda a: (a + np.pi) % (2 * np.pi) - np.pi
d = pd.read_csv(f"{BASE}/log_fixed.csv")
em = d["tlm_elapsed_ms"].values
keep, last = [], None
for i, v in enumerate(em):
    if np.isfinite(v) and (last is None or v != last):
        keep.append(i); last = v
dd = d.iloc[keep].reset_index(drop=True)
t = dd["elapsed_time"].values
truth = np.radians(dd["mocap_yaw_true_deg"].values)
phys = t > 2.4

ra = pd.read_csv(f"{BASE}/replay_a_yawobs/ekf2_replay_log_fixed.csv")
rb = pd.read_csv(f"{BASE}/replay_b_noyaw/ekf2_replay_log_fixed.csv")

def offrm(psi, ref, mask):
    m = mask & np.isfinite(psi) & np.isfinite(ref)
    e = wrap(psi - ref)
    off = math.atan2(np.nanmean(np.sin(e[m])), np.nanmean(np.cos(e[m])))
    return np.degrees(wrap(e - off)), np.degrees(off)

fig, axes = plt.subplots(2, 2, figsize=(13.5, 8.6), facecolor=SURF)
fig.subplots_adjust(hspace=0.42, wspace=0.24, top=0.90, bottom=0.08, left=0.07, right=0.97)
for ax in axes.flat:
    ax.set_facecolor(SURF)
    ax.grid(True, color=GRID, lw=0.8)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    for s in ("left", "bottom"):
        ax.spines[s].set_color(INK2)
    ax.tick_params(colors=INK2, labelsize=9)

# ---- P1: ヨー軌跡 ----
ax = axes[0, 0]
ax.plot(t, np.degrees(truth), color=INK, lw=1.6, ls="--", label="MoCap真値(復元)")
ax.plot(t, np.degrees(dd["tlm_yaw_est_rad"]), color=C_E1, lw=1.8, label="EKF1 (実機ログ)")
ax.plot(t, np.degrees(dd["tlm_ekf2_yaw_rad"]), color=C_E2, lw=1.8, label="EKF2 (実機ログ・ヨー観測全棄却)")
pa = wrap(ra["psi_ekf2_rad"].values - math.radians(-3.92))  # align補正(定数)を真値系へ
ax.plot(t, np.degrees(pa), color=C_A, lw=1.8, label="リプレイ(a) 正解基準融合")
ax.set_title("ヨー軌跡: 正しいMoCap設定ならEKF2は真値に密着していた", fontsize=11, color=INK, pad=8)
ax.set_xlabel("t [s]", fontsize=9, color=INK2); ax.set_ylabel("ψ [deg]", fontsize=9, color=INK2)
ax.legend(fontsize=8, loc="upper left", framealpha=0.9)

# ---- P2: 真値に対する誤差 ----
ax = axes[0, 1]
e1, o1 = offrm(dd["tlm_yaw_est_rad"].values, truth, phys)
e2, o2 = offrm(dd["tlm_ekf2_yaw_rad"].values, truth, phys)
eaa = np.degrees(wrap(ra["psi_ekf2_rad"].values - truth - math.radians(-3.92)))
raw1 = np.degrees(wrap(dd["tlm_yaw_est_rad"].values - truth))
raw2 = np.degrees(wrap(dd["tlm_ekf2_yaw_rad"].values - truth))
ax.axhspan(-2, 2, color="#e7efe9", zorder=0)
ax.plot(t[phys], raw1[phys], color=C_E1, lw=1.8)
ax.plot(t[phys], raw2[phys], color=C_E2, lw=1.8)
ax.plot(t[phys], eaa[phys], color=C_A, lw=1.8)
ax.text(50.5, raw1[phys][-1] - 4.0, "EKF1", color=C_E1, fontsize=9, ha="right")
ax.text(36.5, -21.5, "EKF2(コースト)", color=C_E2, fontsize=9, ha="right")
ax.text(53.5, 3.2, "リプレイ(a) RMS 0.41°", color=C_A, fontsize=9, ha="right")
ax.text(2.8, -0.5, "±2°", color=INK2, fontsize=8)
ax.set_title("真値に対するヨー誤差 (飛行中)", fontsize=11, color=INK, pad=8)
ax.set_xlabel("t [s]", fontsize=9, color=INK2); ax.set_ylabel("ψ−ψ_true [deg]", fontsize=9, color=INK2)
ax.set_ylim(-30, 12)

# ---- P3: ヨー観測イノベーション ----
ax = axes[1, 0]
innov_log = np.degrees(dd["tlm_ekf2_yaw_innov_rad"].values)
innov_rep = np.degrees(ra["yaw_innov_rad"].values)
ax.axhspan(-30, 30, color="#e7efe9", zorder=0)
ax.plot(t, innov_log, color=C_E2, lw=1.8)
ax.plot(t, innov_rep, color=C_A, lw=1.8)
ax.axhline(30, color=INK2, lw=0.9, ls=":"); ax.axhline(-30, color=INK2, lw=0.9, ls=":")
ax.text(1.2, -68, "実飛行: 鏡像基準で|innov|≈77° → 30°ゲート全棄却", color=C_E2, fontsize=9)
ax.text(30, 9, "正解基準なら innov RMS 0.42° (100%融合)", color=C_A, fontsize=9)
ax.text(45.5, 24, "±30°ゲート", color=INK2, fontsize=8)
ax.set_title("EKF2ヨー観測イノベーション: 実飛行 vs 正解基準リプレイ", fontsize=11, color=INK, pad=8)
ax.set_xlabel("t [s]", fontsize=9, color=INK2); ax.set_ylabel("innov [deg]", fontsize=9, color=INK2)

# ---- P4: FF残差の直接測定 (真値ヨーで回転除去) ----
ax = axes[1, 1]
res = json.load(open(f"{BASE}/ff_residual_direct.json"))
from ekf2_replay import level_mag_vector_body
mag = dd[[f"tlm_mag_cal_{c}_ut" for c in "xyz"]].values
roll = dd["tlm_roll_rad"].values; pitch = dd["tlm_pitch_rad"].values
B0h = level_mag_vector_body(roll[0], pitch[0], mag[0])[:2]
beta = wrap(truth - truth[0])
px = np.cos(beta) * B0h[0] - np.sin(beta) * B0h[1]
py = np.sin(beta) * B0h[0] + np.cos(beta) * B0h[1]
zx = ra["zx_ut"].values; zy = ra["zy_ut"].values
fr = np.isfinite(zx) & (t > 3.0)
ax.plot(t[fr], (zx - px)[fr], color=C_A, lw=1.8)
ax.plot(t[fr], (zy - py)[fr], color=C_E1, lw=1.8)
cb = res["stale(frame0)"]["c_body"]
ax.axhline(cb[0], color=C_A, lw=1.0, ls="--")
ax.axhline(cb[1], color=C_E1, lw=1.0, ls="--")
ax.axhspan(-16, -13, color="#f3e2dc", zorder=0)
ax.text(3.2, -15.6, "7/27の残差ノルム帯 13–16µT", color="#a04a2a", fontsize=8)
ax.text(29.0, cb[0] + 1.0, f"x成分: 平均{cb[0]:+.1f}µT", color=C_A, fontsize=9, ha="right")
ax.text(29.0, cb[1] + 1.0, f"y成分: 平均{cb[1]:+.1f}µT", color=C_E1, fontsize=9, ha="right")
ax.text(3.2, 8.2, "機体固定成分 |c_body|=7.4µT (magbias_learn: 7.65µT)", color=INK, fontsize=9)
ax.set_title("FF補正残差の直接測定 (真値ヨーで可観測化)", fontsize=11, color=INK, pad=8)
ax.set_xlabel("t [s]", fontsize=9, color=INK2); ax.set_ylabel("残差 [µT] (レベル化機体系)", fontsize=9, color=INK2)
ax.set_ylim(-20, 10)

fig.suptitle("2026-07-31 飛行のリプレイ救済: MoCap設定が正しければEKF2はヨーRMS 0.41°を達成していた",
             fontsize=13, color=INK, y=0.97)
out = f"{BASE}/replay_rescue_summary.png"
fig.savefig(out, dpi=150, facecolor=SURF)
print(out)
