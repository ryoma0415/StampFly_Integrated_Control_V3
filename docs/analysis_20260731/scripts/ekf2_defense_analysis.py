#!/usr/bin/env python
# EKF2 + safeguard in-flight behavior analysis (2026-07-31 flight, contaminated yaw ref)
import json
import numpy as np
import pandas as pd

BASE = "/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731"
df = pd.read_csv(f"{BASE}/log_corrected.csv")
t = df["elapsed_time"].to_numpy()
D = 180.0 / np.pi


def wrap_deg(a):
    return (a + 180.0) % 360.0 - 180.0


out = {}
n = len(df)
out["n_rows"] = n
out["t_span"] = [float(t[0]), float(t[-1])]

# ---------------- 1. innovation / gate / latch ----------------
innov = df["tlm_ekf2_yaw_innov_rad"].to_numpy() * D
status = df["tlm_ekf2_status"].to_numpy().astype(int)
gate2 = df["tlm_ekf2_gate"].to_numpy().astype(int)
ffg = df["tlm_ffg"].to_numpy().astype(int)

out["innov_stats"] = dict(
    mean=float(np.mean(innov)), mean_abs=float(np.mean(np.abs(innov))),
    min=float(np.min(innov)), max=float(np.max(innov)),
    n_nonzero=int(np.sum(innov != 0)),
    n_inside_30=int(np.sum(np.abs(innov) <= 30.0)),
    n_outside_30=int(np.sum(np.abs(innov) > 30.0)),
)
# rows where |innov|<=30 — when? (before first obs, innov=0 default?)
inside = np.abs(innov) <= 30.0
if inside.any():
    out["inside30_rows_t"] = [float(t[i]) for i in np.where(inside)[0][:20]]
    out["inside30_innov"] = [float(innov[i]) for i in np.where(inside)[0][:20]]

# status bits
bits = {f"bit{b}": int(np.sum((status >> b) & 1)) for b in range(8)}
out["status_bit_rowcounts"] = bits
out["status_values"] = {str(k): int(v) for k, v in
                        pd.Series(status).value_counts().items()}
out["fused_bit1_any"] = bool(np.any((status >> 1) & 1))
out["fresh_bit0_all"] = bool(np.all((status >> 0) & 1))
out["healthy_bit5_all"] = bool(np.all((status >> 5) & 1))
out["anchor_bit2_any"] = bool(np.any((status >> 2) & 1))
out["taurw_bit3_any"] = bool(np.any((status >> 3) & 1))

# innovation update cadence: count changes of innov value between rows
dinnov = np.abs(np.diff(innov))
changes = dinnov > 1e-9
out["innov_change_rows"] = int(np.sum(changes))
out["innov_change_rate_hz"] = float(np.sum(changes) / (t[-1] - t[0]))
# first nonzero innovation row -> first yaw obs processed
nz = np.where(innov != 0)[0]
first_obs_i = int(nz[0]) if len(nz) else -1
out["first_obs_t"] = float(t[first_obs_i]) if first_obs_i >= 0 else None
out["first_obs_innov"] = float(innov[first_obs_i]) if first_obs_i >= 0 else None

# estimate latch time: 25th consecutive rejection.
# every obs from the start was outside gate (all |innov|>30 after first obs?)
after = np.abs(innov[first_obs_i:]) > 30.0
out["all_rejected_after_first_obs"] = bool(np.all(after))
# obs arrive ~change cadence; estimate obs timestamps = rows where innov changes
obs_rows = [first_obs_i] + [i + 1 for i in np.where(changes)[0] if i + 1 > first_obs_i]
obs_rows = sorted(set(obs_rows))
out["n_obs_estimated_lb"] = len(obs_rows)  # lower bound (equal consecutive values undercount)
if len(obs_rows) >= 25:
    out["latch_t_est_from_changes"] = float(t[obs_rows[24]])
# telemetry row cadence
out["row_dt_median_ms"] = float(np.median(np.diff(t)) * 1000)
# obs inter-arrival from change rows
if len(obs_rows) > 10:
    dts = np.diff(t[obs_rows])
    out["obs_dt_median_ms"] = float(np.median(dts) * 1000)
    out["obs_dt_p90_ms"] = float(np.percentile(dts, 90) * 1000)
# latch estimate assuming 50 Hz obs: first_obs_t + 24/50
out["latch_t_est_50hz"] = float(t[first_obs_i] + 24 / 50.0) if first_obs_i >= 0 else None

# verify innov == wrap(ref_sent - ekf2_yaw) (already done upstream; recompute here)
ref = df["yaw_ref_sent_rad"].to_numpy() * D
e2y = df["tlm_ekf2_yaw_rad"].to_numpy() * D
pred = wrap_deg(ref - e2y)
resid = wrap_deg(innov - pred)
out["innov_vs_refsent_median_absdiff_deg"] = float(np.median(np.abs(resid)))

# would-be innovation with corrected ref
refc = df["yaw_ref_corrected_rad"].to_numpy() * D
innov_would = wrap_deg(refc - e2y)
out["would_be_innov"] = dict(
    mean=float(np.mean(innov_would)), rms=float(np.sqrt(np.mean(innov_would**2))),
    max_abs=float(np.max(np.abs(innov_would))),
    pct_inside_gate=float(np.mean(np.abs(innov_would) <= 30.0) * 100),
)

# ---------------- 2. EKF2 vs EKF1 ----------------
e1y = df["tlm_yaw_est_rad"].to_numpy() * D
dpsi = wrap_deg(e2y - e1y)
b1x = df["tlm_bm_x_ut"].to_numpy(); b1y = df["tlm_bm_y_ut"].to_numpy()
b2x = df["tlm_ekf2_bm_x_ut"].to_numpy(); b2y = df["tlm_ekf2_bm_y_ut"].to_numpy()
n1 = np.hypot(b1x, b1y); n2 = np.hypot(b2x, b2y)
out["dpsi_deg"] = dict(mean=float(np.mean(dpsi)), rms=float(np.sqrt(np.mean(dpsi**2))),
                       max_abs=float(np.max(np.abs(dpsi))), final=float(dpsi[-1]))
out["bm_final"] = dict(ekf1=[float(b1x[-1]), float(b1y[-1]), float(n1[-1])],
                       ekf2=[float(b2x[-1]), float(b2y[-1]), float(n2[-1])])
out["bm_norm_max"] = dict(ekf1=float(np.max(n1)), ekf2=float(np.max(n2)))
dbm = np.hypot(b2x - b1x, b2y - b1y)
out["bm_diff_norm"] = dict(mean=float(np.mean(dbm)), final=float(dbm[-1]),
                           max=float(np.max(dbm)))

# theory: EKF1 has GM decay a=1-dt/120 (leak 0.12 uT/s at |bm|~14 -> here |bm|/120 per s)
# EKF2: a=1, q_bm x0.1 after 1 s without accepted yaw obs (never accepted -> whole flight lost mode)
# During mag-accept periods both track; difference should be EKF1 leak + gain diff from q_bm.
# integrate expected EKF1 leak: d(bm1)/dt has extra -bm1/120 term vs EKF2
# Compare growth rates in flight window (post-takeoff to end)
fly = (t > 3.0)
for name, nn in [("ekf1", n1), ("ekf2", n2)]:
    tf = t[fly]; nf = nn[fly]
    # linear fit growth rate
    A = np.polyfit(tf, nf, 1)
    out[f"bm_growth_{name}_uT_per_s"] = float(A[0])
out["bm_growth_ratio_2over1"] = float(out["bm_growth_ekf2_uT_per_s"] / out["bm_growth_ekf1_uT_per_s"])

# GM-leak visibility test: predict bm1 from bm2 via first-order leak model
# bm1_dot = bm2_dot - bm1/tau  (if only difference were the leak and identical K updates)
dt = np.median(np.diff(t))
bm1_pred_x = np.zeros(n); bm1_pred_y = np.zeros(n)
bm1_pred_x[0] = b2x[0]; bm1_pred_y[0] = b2y[0]
for i in range(1, n):
    ddt = t[i] - t[i - 1]
    if not np.isfinite(ddt) or ddt <= 0 or ddt > 0.2:
        ddt = dt
    bm1_pred_x[i] = bm1_pred_x[i - 1] * (1 - ddt / 120.0) + (b2x[i] - b2x[i - 1])
    bm1_pred_y[i] = bm1_pred_y[i - 1] * (1 - ddt / 120.0) + (b2y[i] - b2y[i - 1])
leak_resid = np.hypot(b1x - bm1_pred_x, b1y - bm1_pred_y)
out["gm_leak_model_resid_uT"] = dict(mean=float(np.mean(leak_resid)),
                                     final=float(leak_resid[-1]),
                                     max=float(np.max(leak_resid)))
# expected leak-only divergence: integrate bm2/120
leak_int = np.cumsum(np.hypot(b2x, b2y) / 120.0 * np.gradient(t))
out["expected_leak_integral_final_uT"] = float(leak_int[-1])

# ---------------- 3. gate bits ----------------
def bitrate(arr, b):
    return float(np.mean((arr >> b) & 1) * 100)

gate_tbl = {}
for b, nm in [(0, "R_inflated"), (1, "NIS_reject"), (2, "norm_reject"),
              (3, "z_reject"), (4, "tilt_skip"), (5, "bm_frozen"),
              (6, "drift_warn"), (7, "recapture")]:
    gate_tbl[nm] = dict(ekf2_pct=bitrate(gate2, b), ekf1_pct=bitrate(ffg, b))
out["gate_bit_rates_pct"] = gate_tbl
out["gate2_values"] = {str(k): int(v) for k, v in pd.Series(gate2).value_counts().items()}
out["ffg_values"] = {str(k): int(v) for k, v in pd.Series(ffg).value_counts().items()}
# in-flight only z-reject rate (EKF updates only meaningful during mag updates; use t>3)
out["z_reject_pct_flight"] = dict(
    ekf2=float(np.mean((gate2[fly] >> 3) & 1) * 100),
    ekf1=float(np.mean((ffg[fly] >> 3) & 1) * 100))
out["nis_reject_pct_flight"] = dict(
    ekf2=float(np.mean((gate2[fly] >> 1) & 1) * 100),
    ekf1=float(np.mean((ffg[fly] >> 1) & 1) * 100))
out["r_infl_pct_flight"] = dict(
    ekf2=float(np.mean((gate2[fly] >> 0) & 1) * 100),
    ekf1=float(np.mean((ffg[fly] >> 0) & 1) * 100))

# altitude correlation of z-reject (EKF1)
z = df["pos_z"].to_numpy() if "pos_z" in df else None
if z is not None:
    zr = ((ffg >> 3) & 1).astype(float)
    out["z_reject_vs_alt_corr_ekf1"] = float(np.corrcoef(z[fly], zr[fly])[0, 1])
    out["alt_mean_when_zreject"] = float(np.mean(z[fly][zr[fly] > 0.5])) if zr[fly].any() else None
    out["alt_mean_when_accept"] = float(np.mean(z[fly][zr[fly] < 0.5]))

# ---------------- 4. flight anchor conditions ----------------
cur = df["tlm_current_a"].to_numpy()
alt_ok = None
out["anchor_cond"] = dict(
    current_gt1A_pct_flight=float(np.mean(cur[fly] > 1.0) * 100),
    fused_ever=bool(np.any((status >> 1) & 1)),
)
# alt_est - alt_ref within 0.1 m for 2 s: use pos_z vs target 0.28? target z from cmd? check col
for c in df.columns:
    if "alt" in c or ("z" in c and "ref" in c):
        pass
# hover altitude stability from pos_z std
out["anchor_cond"]["alt_std_hover"] = float(np.std(z[(t > 10) & (t < 35)])) if z is not None else None

# ---------------- 6. b_m growth vs 7/27 ----------------
# 7/27: |bm| 15-16 uT at ~100 s (log1 (-15.1,-5.9)=16.2, log2 (-3.7,-14.9)=15.4)
out["growth_compare"] = dict(
    today_final_uT=dict(ekf1=float(n1[-1]), ekf2=float(n2[-1])),
    today_dur_s=float(t[-1]),
    today_rate=dict(ekf1=float(n1[-1] / t[-1]), ekf2=float(n2[-1] / t[-1])),
    ref_0727_log1=dict(norm=16.2, dur=100.0, rate=0.162),
    ref_0727_log2=dict(norm=15.4, dur=100.0, rate=0.154),
)

print(json.dumps(out, indent=1, ensure_ascii=False))
