#!/usr/bin/env python
"""Plots for the 18:20 flow calibration assessment."""
import json
import math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

SP = ("/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-"
      "Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/"
      "ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad")
PL = SP + "/a0731b/plots"
z = np.load(SP + "/a0731b/flow_eval.npz")
t, fvx, fvy = z["t"], z["fvx"], z["fvy"]
rvx, rvy, m, spd = z["rvx"], z["rvy"], z["m"], z["spd"]
turns = [(23.0, 26.5), (30.9, 33.2), (36.8, 40.2), (43.9, 47.0)]

# ---- 1. time series ----
fig, axes = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
for ax, f_, r_, nm in ((axes[0], fvx, rvx, "vx (body-x, sensor -dy ch)"),
                       (axes[1], fvy, rvy, "vy (body-y, sensor +dx ch)")):
    for a_, b_ in turns:
        ax.axvspan(a_, b_, color="orange", alpha=0.15, lw=0)
    ax.plot(t[m], r_[m], "-", lw=1.0, color="k", label="MoCap ref (rot+lag comp)")
    ax.plot(t[m], f_[m], ".", ms=2.5, color="tab:blue", alpha=0.6, label="tlm_flow")
    ax.set_ylabel(nm + " [m/s]")
    ax.set_ylim(-0.6, 0.6)
    ax.grid(alpha=0.3)
axes[0].legend(loc="upper right", fontsize=9)
axes[0].set_title("18:20 flight: optical flow vs MoCap body-frame reference "
                  "(orange = yaw-turn windows)")
axes[1].set_xlabel("t [s]")
fig.tight_layout()
fig.savefig(PL + "/flow_vs_ref_timeseries.png", dpi=130)
plt.close(fig)

# ---- 2. scatter ----
fig, axes = plt.subplots(1, 2, figsize=(11, 5))
for ax, f_, r_, nm, sc in ((axes[0], fvx, rvx, "vx", 0.815),
                           (axes[1], fvy, rvy, "vy", 1.028)):
    ax.plot(r_[m], f_[m], ".", ms=3, alpha=0.5)
    xx = np.array([-0.35, 0.35])
    ax.plot(xx, xx, "k--", lw=1, label="1:1")
    ax.plot(xx, sc * xx, "r-", lw=1.2, label=f"fit scale={sc:.2f}")
    ax.set_xlabel(f"MoCap ref {nm} [m/s]")
    ax.set_ylabel(f"flow {nm} [m/s]")
    ax.set_xlim(-0.35, 0.35); ax.set_ylim(-0.5, 0.5)
    ax.grid(alpha=0.3); ax.legend(fontsize=9)
axes[0].set_title("18:20 scatter (rot+lag-comp reference)")
fig.tight_layout()
fig.savefig(PL + "/flow_scatter.png", dpi=130)
plt.close(fig)

# ---- 3. rotation & scale evidence panel ----
fig, axes = plt.subplots(1, 3, figsize=(15, 4.6))
# 3a: phase vs frequency
freqs, coh, ph, band, phi0 = (z["freqs"], z["coh"], z["ph"], z["band"], float(z["phi0"]))
o = np.argsort(freqs)
ax = axes[0]
sel = band[o]
ax.plot(freqs[o][sel], np.degrees(ph[o][sel]) + math.degrees(phi0), "o", ms=5,
        color="tab:blue", label="cross-spectrum phase (coh>0.3)")
ff = np.linspace(-2.5, 2.5, 50)
ax.plot(ff, 94.61 - 360 * ff * 0.030, "r-", lw=1.2,
        label="fit: phi0=+94.6 deg, lag 30 ms")
ax.axhline(90.43, color="g", ls=":", lw=1.2, label="perfect alignment (= c 90.4 deg)")
ax.set_xlabel("frequency [Hz]"); ax.set_ylabel("phase [deg]")
ax.set_title("rotation vs lag decomposition")
ax.grid(alpha=0.3); ax.legend(fontsize=8)
# 3b: speed-binned iso gain both flights
bins = ["0-0.03", "0.03-0.06", "0.06-0.10", "0.10-0.15", "0.15-0.25"]
g1820 = [0.442, 1.028, 0.924, 0.996, np.nan]
g1119 = [1.826, 1.282, 1.249, 1.376, 1.237]
xb = np.arange(len(bins))
ax = axes[1]
ax.bar(xb - 0.18, g1119, 0.34, label="11:19 (pre-fix, K uncal)", color="tab:red", alpha=0.75)
ax.bar(xb + 0.18, g1820, 0.34, label="18:20 (post-fix)", color="tab:blue", alpha=0.75)
ax.axhline(1.0, color="k", ls="--", lw=1)
ax.set_xticks(xb); ax.set_xticklabels(bins, fontsize=8)
ax.set_xlabel("|v_ref| bin [m/s]"); ax.set_ylabel("isotropic gain flow/ref")
ax.set_ylim(0, 2.0); ax.set_title("speed-matched gain: 1.25-1.38 -> 0.92-1.03")
ax.grid(alpha=0.3, axis="y"); ax.legend(fontsize=8)
# 3c: channel gains summary
ax = axes[2]
labels = ["dx ch\n11:19", "dy ch\n11:19", "dx ch\n18:20\n(all/turn)", "dy ch\n18:20\n(all/turn)"]
vals = [1.154, 1.293, 1.039, 0.812]
err = [[0.03, 0.03, 1.039 - 0.975, 0.812 - 0.692], [0.03, 0.03, 1.102 - 1.039, 0.944 - 0.812]]
cols = ["tab:red", "tab:red", "tab:blue", "tab:blue"]
ax.bar(range(4), vals, 0.55, yerr=err, color=cols, alpha=0.75, capsize=4)
ax.plot([2, 3], [1.132, 1.044], "k^", ms=8, label="turn-only estimate")
ax.axhline(1.0, color="k", ls="--", lw=1)
ax.set_xticks(range(4)); ax.set_xticklabels(labels, fontsize=8)
ax.set_ylabel("channel gain flow/ref")
ax.set_title("per-sensor-channel gain (90% CI)")
ax.grid(alpha=0.3, axis="y"); ax.legend(fontsize=8)
fig.tight_layout()
fig.savefig(PL + "/rotation_scale_evidence.png", dpi=130)
plt.close(fig)

# ---- 4. dead reckoning ----
dr_t, dr_px, dr_py, dr_em = z["dr_t"], z["dr_px"], z["dr_py"], z["dr_em"]
mx, my = z["mocap_x"], z["mocap_y"]
fig, axes = plt.subplots(1, 2, figsize=(12, 5.2))
ax = axes[0]
ax.plot(mx, my, "-", lw=1.2, color="k", label="MoCap")
ax.plot(dr_px, dr_py, "-", lw=1.2, color="tab:blue", label="flow DR (fitted frame)")
ax.plot(mx[0], my[0], "go", ms=8, label="start")
ax.plot(dr_px[-1], dr_py[-1], "bs", ms=7, label="DR end")
ax.plot(mx[-1], my[-1], "ks", ms=7, label="MoCap end")
ax.set_xlabel("x [m]"); ax.set_ylabel("y [m]")
ax.axis("equal"); ax.grid(alpha=0.3); ax.legend(fontsize=8)
ax.set_title("dead reckoning vs MoCap (t=4.0-53.5s)")
ax = axes[1]
ax.plot(dr_t, dr_em, "-", color="tab:blue")
for a_, b_ in turns:
    ax.axvspan(a_, b_, color="orange", alpha=0.15, lw=0)
ax.set_xlabel("t [s]"); ax.set_ylabel("|DR - MoCap| [m]")
ax.grid(alpha=0.3)
ax.set_title("DR error growth (terminal 0.49 m / 49.5 s = 1.0 cm/s)")
fig.tight_layout()
fig.savefig(PL + "/dead_reckoning.png", dpi=130)
plt.close(fig)

# ---- extra numbers for the report ----
resx = fvx - rvx
resy = fvy - rvy
out = {
    "bias_x_mps": round(float(resx[m].mean()), 4),
    "bias_y_mps": round(float(resy[m].mean()), 4),
    "outlier_times": [round(float(v), 1) for v in
                      t[m & (np.hypot(resx, resy) > 0.35)][:40]],
}
print(json.dumps(out, indent=1))
print("plots saved")
