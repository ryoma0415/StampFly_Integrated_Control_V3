#!/usr/bin/env python
"""Cross-verify key numbers claimed across the 5 sub-analyses of 20260731_111906."""
import numpy as np, pandas as pd, json

SP = "/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731"
df = pd.read_csv(f"{SP}/log_corrected.csv")
print("rows:", len(df), "cols:", len(df.columns))

def wrapd(a):
    return (a + 180.0) % 360.0 - 180.0

t = df["elapsed_time"].astype(float).values
print("time col: elapsed_time range", t[0], t[-1])

true_d = df["yaw_true_corrected_deg"].values
ekf1_d = np.degrees(df["tlm_yaw_est_rad"].values)
ekf2_d = np.degrees(df["tlm_ekf2_yaw_rad"].values)
gyro_d = np.degrees(df["tlm_yaw_gyro_int_rad"].values)

# --- error stats, two windows ---
def stats(err, mask, label):
    e = err[mask]
    print(f"{label}: n={mask.sum()} mean={e.mean():+.2f} rms={np.sqrt((e**2).mean()):.2f} max|.|={np.abs(e).max():.2f}")

e1 = wrapd(ekf1_d - true_d)
e2 = wrapd(ekf2_d - true_d)
allm = np.ones(len(df), bool)
allm[0] = False  # stale first row
state = df["tlm_state"].values if "tlm_state" in df.columns else None
if state is not None:
    fly = (state == 4)
    print("flight window t:", t[fly].min(), t[fly].max(), "n=", fly.sum())
else:
    fly = (t > 4.55) & (t < 52.95)
stats(e1, allm, "EKF1 err all-rows")
stats(e1, fly, "EKF1 err flight")
stats(e2, allm, "EKF2 err all-rows")
stats(e2, fly, "EKF2 err flight")

# offset-corrected variant (ground window anchor per report [4])
g = (t >= 0.15) & (t <= 1.5)
off1 = wrapd(e1[g]).mean(); off2 = wrapd(e2[g]).mean()
print(f"ground offsets: EKF1 {off1:+.2f} EKF2 {off2:+.2f}")
stats(wrapd(e1 - off1), fly, "EKF1 err flight corrected")
stats(wrapd(e2 - off2), fly, "EKF2 err flight corrected")

# --- delta psi EKF2-EKF1 ---
dpsi = wrapd(ekf2_d - ekf1_d)
stats(dpsi, allm, "dpsi all")
stats(dpsi, fly, "dpsi flight")
print("dpsi final:", dpsi[-1])

# --- b_m finals ---
for c in ["tlm_bm_x_ut", "tlm_bm_y_ut", "tlm_ekf2_bm_x_ut", "tlm_ekf2_bm_y_ut"]:
    print(c, "final", df[c].values[-1])
bm1 = np.hypot(df["tlm_bm_x_ut"], df["tlm_bm_y_ut"]).values
bm2 = np.hypot(df["tlm_ekf2_bm_x_ut"], df["tlm_ekf2_bm_y_ut"]).values
print(f"|bm1| final {bm1[-1]:.2f} max {bm1.max():.2f}; |bm2| final {bm2[-1]:.2f}")
print(f"bm1 rate final/54.5 = {bm1[-1]/t[-1]:.4f} uT/s ; bm2 {bm2[-1]/t[-1]:.4f}")

# --- ekf2 status / gate ---
st = df["tlm_ekf2_status"].astype(int).values
print("ekf2_status uniq:", np.unique(st, return_counts=True))
gate = df["tlm_ekf2_gate"].astype(int).values
print("ekf2_gate dist:", dict(zip(*np.unique(gate, return_counts=True))))
ffg = df["tlm_ffg"].astype(int).values
print("ekf2_gate == ffg all rows:", bool((gate == ffg).all()))
print("flight z_reject pct:", 100.0 * (gate[fly] == 8).mean())

# --- innovation ---
innov = np.degrees(df["tlm_ekf2_yaw_innov_rad"].values)
nz = innov != 0
print(f"innov nonzero from row {np.argmax(nz)} t={t[np.argmax(nz)]:.3f} first val {innov[nz][0]:.2f}")
m = nz.copy()
print(f"innov(nonzero): n={m.sum()} |mean|={np.abs(innov[m].mean()):.2f} range [{innov[m].min():.2f},{innov[m].max():.2f}]")
print(f"pct |innov|>30 among nonzero: {100.0*(np.abs(innov[m])>30).mean():.2f}")
# latch: 25th change of innov value
chg = np.where(np.abs(np.diff(innov)) > 1e-9)[0] + 1
if len(chg) >= 25:
    print(f"25th innov change at t={t[chg[24]]:.3f}s (obs count proxy); n changes={len(chg)} rate={len(chg)/(t[chg[-1]]-t[chg[0]]):.1f}Hz")

# --- would-be innovation with corrected reference ---
ref_corr = np.degrees(df["yaw_ref_corrected_rad"].values)
wb = wrapd(ref_corr - ekf2_d)
stats(wb, allm, "wouldbe innov all")
stats(wb, fly, "wouldbe innov flight")
print("pct within 30deg (all):", 100.0 * (np.abs(wb[allm]) <= 30).mean(), " (flight):", 100.0 * (np.abs(wb[fly]) <= 30).mean())

# --- gyro drift ---
eg = wrapd(gyro_d - true_d)
# linear fit over flight
A = np.polyfit(t[fly], eg[fly], 1)
print(f"gyro-int drift fit: {A[0]*60:+.1f} deg/min over flight")
madg = np.degrees(df["tlm_yaw_rad"].values)
em = wrapd(madg - true_d)
Am = np.polyfit(t[fly], em[fly], 1)
print(f"madgwick drift fit: {Am[0]*60:+.1f} deg/min")

# --- flow ---
fs = df["tlm_flow_status"].astype(int).values
u, c = np.unique(fs, return_counts=True)
print("flow_status dist:", {int(a): (int(b), round(100*b/len(df),1)) for a, b in zip(u, c)})
print("burst_ok(bit1) all rows:", bool(((fs & 2) > 0).all()))
sq = df["tlm_flow_squal"].values
print(f"squal mean all {sq.mean():.1f}; flight mean {sq[fly].mean():.1f} min {sq[fly].min():.0f}")
vx = df["tlm_flow_vx_mps"].values; vy = df["tlm_flow_vy_mps"].values
print("flow vel nonzero pct:", 100.0 * ((vx != 0) | (vy != 0)).mean())

# --- ff_status ---
if "tlm_ff_status" in df.columns:
    u, c = np.unique(df["tlm_ff_status"].astype(int).values, return_counts=True)
    print("ff_status dist:", dict(zip(u.tolist(), c.tolist())))
cols = [c for c in df.columns if "ff" in c.lower()]
print("ff cols:", cols)

# --- position hover stats ---
if "pos_x" in df.columns:
    hov = fly & ~((t >= 39) & (t <= 48))
    ex = df["pos_x"].values; ey = df["pos_y"].values
    # target (0,0)
    r = np.hypot(ex, ey)
    print(f"hover XY rms x={np.sqrt((ex[hov]**2).mean())*1000:.0f}mm y={np.sqrt((ey[hov]**2).mean())*1000:.0f}mm radiusRMS={np.sqrt((r[hov]**2).mean())*1000:.0f}mm P95={np.percentile(r[hov],95)*1000:.0f}mm")
    z = df["pos_z"].values
    print(f"z mean hover {z[hov].mean():.3f} std {z[hov].std():.3f}")

# --- yaw_ref_sent mirror check vs corrected ---
sent = np.degrees(df["yaw_ref_sent_rad"].values)
mirror = wrapd(-(true_d) + 2 * 0.0)  # sent = -yaw_raw ; yaw_raw = -(true - 88.617) => sent = true - 88.617... verify:
# true = -raw + 88.617  => raw = 88.617 - true => sent = -raw = true - 88.617
pred = wrapd(true_d - 88.617)
d = wrapd(sent - pred)
print(f"sent vs (true-88.617): rms {np.sqrt((d[allm]**2).mean()):.3f} deg")
