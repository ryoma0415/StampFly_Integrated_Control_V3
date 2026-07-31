#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""20260731_182025: robust truth-yaw reconstruction (95-deg mocap glitch repair)
+ full flight/firmware state recovery. v2."""
import json
import numpy as np
import pandas as pd

BASEDIR = ("/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/"
           "StampFly_MoCap_System_v2/StampFly_Integrated_Control_V3")
LOG = BASEDIR + "/logs/flight_logs/20260731_182025_position.csv"
OLDCSV = ("/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-"
          "Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/"
          "ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731/log_corrected.csv")
OUT = ("/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-"
       "Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/"
       "ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731b")

def wrapd(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0

df = pd.read_csv(LOG)
t = df["elapsed_time"].values
n = len(df)
res = {}

# ---------- yaw_raw ----------
qx, qy, qz, qw = df[["mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw"]].values.T
fx = 1 - 2 * (qy**2 + qz**2)
fz = 2 * (qx * qz - qw * qy)
yaw_raw = np.degrees(np.arctan2(fx, fz))
res["raw_vs_heading_rms"] = float(np.sqrt(np.nanmean(
    wrapd(yaw_raw - df["mocap_heading_deg"].values)**2)))

# ---------- onboard frame ----------
g_u = np.degrees(np.unwrap(np.radians(np.degrees(  # unwrap gyro-int
    df["tlm_yaw_gyro_int_rad"].values) * np.pi / 180)))
g_u = np.degrees(np.unwrap(df["tlm_yaw_gyro_int_rad"].values))
madg_u = np.degrees(np.unwrap(df["tlm_yaw_rad"].values))
i0 = int(np.argmax(df["tlm_state"].values >= 3))
m_on = np.arange(n) >= i0

# ---------- robust sign/offset fit with glitch outlier rejection ----------
def robust_fit(s):
    D = wrapd(s * yaw_raw - g_u)          # mocap-minus-gyro discrepancy
    z = np.exp(1j * np.radians(D[m_on]))
    d0 = np.degrees(np.angle(z.mean()))   # circular robust-ish center
    resd = wrapd(D - d0)
    inl = np.abs(resd) < 25
    for _ in range(4):                    # iterative robust linear trend
        A = np.polyfit(t[m_on & inl], resd[m_on & inl], 1)
        trend = np.polyval(A, t)
        inl = np.abs(resd - trend) < 25
    det = (resd - trend)[m_on & inl]
    return dict(A=A, d0=d0, resd=resd, inl=inl, trend=trend,
                rms=float(np.sqrt(np.mean(det**2))),
                n_out=int((m_on & ~inl).sum()))

fits = {s: robust_fit(s) for s in (+1, -1)}
res["signtest_vs_gyro"] = {f"s{s:+d}": dict(detrended_rms_deg=f_["rms"],
                                            n_outliers=f_["n_out"])
                           for s, f_ in fits.items()}
s_best = min(fits, key=lambda s: fits[s]["rms"])
res["yaw_sign_best"] = int(s_best)
F = fits[s_best]
inl = F["inl"]
D_abs = F["d0"] + F["resd"]               # = wrap-consistent (s*h - g_u)

# ---------- glitch census ----------
out_rows = m_on & ~inl
dev = wrapd(F["resd"] - F["trend"])
res["glitch"] = dict(
    n_outlier_rows=int(out_rows.sum()),
    frac=float(out_rows.mean()),
    deviation_median_deg=float(np.median(dev[out_rows])) if out_rows.any() else None,
    deviation_p10_p90=[float(np.percentile(dev[out_rows], 10)),
                       float(np.percentile(dev[out_rows], 90))] if out_rows.any() else None,
    t_first=float(t[out_rows].min()) if out_rows.any() else None,
    t_last=float(t[out_rows].max()) if out_rows.any() else None)
# burst structure: consecutive-run lengths
runs = []
i = i0
while i < n:
    if out_rows[i]:
        j = i
        while j + 1 < n and out_rows[j + 1]:
            j += 1
        runs.append((float(t[i]), j - i + 1))
        i = j + 1
    else:
        i += 1
res["glitch"]["n_runs"] = len(runs)
res["glitch"]["run_len_dist"] = {int(k): int(v) for k, v in
                                 zip(*np.unique([r[1] for r in runs], return_counts=True))} if runs else {}
res["glitch"]["run_times_first20"] = [r[0] for r in runs[:20]]
# glitch flag correlation
mflip = df["mocap_flip"].astype(int).values
res["glitch"]["flip_flag_on_outliers"] = {int(k): int(v) for k, v in
                                          zip(*np.unique(mflip[out_rows], return_counts=True))} if out_rows.any() else {}
res["glitch"]["flip_flag_all"] = {int(k): int(v) for k, v in
                                  zip(*np.unique(mflip, return_counts=True))}

# ---------- ground window & offset c ----------
tof = df["tlm_altitude_tof_m"].values
pz = df["pos_z"].values
gw = (t >= t[i0] + 0.1) & (t <= 2.0) & inl
c = float(np.mean(g_u[gw] - s_best * yaw_raw[gw]))
res["ground_window"] = dict(t=[float(t[gw].min()), float(t[gw].max())],
                            n=int(gw.sum()),
                            c_std_deg=float(np.std(g_u[gw] - s_best * yaw_raw[gw])),
                            pos_z_mean=float(np.nanmean(pz[gw])),
                            tof_mean=float(np.nanmean(tof[gw])))
res["offset_c_deg"] = round(c, 3)
res["meta_offset"] = dict(meta_offset_deg=89.9, fitted_c_deg=round(c, 3),
                          delta_deg=round(float(wrapd(c - 89.9)), 3))

# ---------- truth construction ----------
# inliers: truth = s*h + c ; outliers: bridge with gyro + interpolated discrepancy
D_use = D_abs.copy()
D_use[~inl] = np.nan
D_use = pd.Series(D_use).interpolate(limit_direction="both").values
truth_u = g_u + D_use + c                 # continuous
truth_u[inl] = (g_u + D_abs + c)[inl]     # exact mocap on inliers (same thing)
truth = wrapd(truth_u)

# quality vs gyro / madgwick (inliers)
r_g = truth_u - g_u
A = np.polyfit(t[m_on & inl], r_g[m_on & inl], 1)
det = (r_g - np.polyval(A, t))[m_on & inl]
res["quality_vs_gyro"] = dict(drift_deg_per_min=float(A[0] * 60),
                              detrended_rms_deg=float(np.sqrt(np.mean(det**2))),
                              detrended_max_abs_deg=float(np.abs(det).max()))
r_md = truth_u - madg_u
Am = np.polyfit(t[m_on & inl], r_md[m_on & inl], 1)
detm = (r_md - np.polyval(Am, t))[m_on & inl]
res["quality_vs_madgwick"] = dict(drift_deg_per_min=float(Am[0] * 60),
                                  detrended_rms_deg=float(np.sqrt(np.mean(detm**2))))

# logged yaw_true check
myt = df["mocap_yaw_true_deg"].values
dl = wrapd(truth - myt)
res["logged_yaw_true_check"] = dict(
    rows_within_20=int((np.abs(dl) < 20).sum()),
    rows_off_by_180pm20=int((np.abs(np.abs(dl) - 180) < 20).sum()),
    rows_other=int(((np.abs(dl) >= 20) & (np.abs(np.abs(dl) - 180) >= 20)).sum()),
    offset_resid_mean_within20=float(dl[np.abs(dl) < 20].mean()))
# where is the 180-flip stretch (cont_flip bit)?
b2 = (mflip & 2) > 0
if b2.any():
    edges = np.where(np.diff(b2.astype(int)) != 0)[0] + 1
    res["logged_yaw_true_check"]["cont_flip_active_spans_t"] = \
        [float(x) for x in t[edges][:20]]

# ---------- yaw_ref_sent ----------
sent = np.degrees(df["yaw_ref_sent_rad"].values)
res["sent_eq_minus_raw"] = dict(
    rms=float(np.sqrt(np.mean(wrapd(sent + yaw_raw)**2))))
rr_ = wrapd(sent - truth)[inl & m_on]
res["sent_minus_truth_inliers"] = dict(mean=float(rr_.mean()), std=float(rr_.std()))

# ---------- EKF init / innovation ----------
e1 = np.degrees(df["tlm_yaw_est_rad"].values)
e2 = np.degrees(df["tlm_ekf2_yaw_rad"].values)
inn = np.degrees(df["tlm_ekf2_yaw_innov_rad"].values)
w_init = m_on & (t <= 1.0) & inl
res["init_offsets_deg"] = dict(
    ekf1_minus_truth=float(wrapd(e1[w_init] - truth[w_init]).mean()),
    ekf2_minus_truth=float(wrapd(e2[w_init] - truth[w_init]).mean()),
    innov_mean_t_0_1=float(inn[w_init].mean()),
    innov_at_arm=float(inn[i0]))
chk = wrapd(inn - wrapd(sent - e2))
res["innov_eq_sent_minus_ekf2_median_abs"] = float(np.median(np.abs(chk)))
absin = np.abs(inn)
# innovation on clean rows vs glitch rows
res["innov"] = dict(
    all_mean_abs=float(absin[m_on].mean()),
    clean_mean=float(inn[m_on & inl].mean()),
    clean_std=float(inn[m_on & inl].std()),
    clean_min_abs=float(absin[m_on & inl].min()),
    clean_frac_within30=float((absin[m_on & inl] < 30).mean()),
    glitch_frac_within30=float((absin[out_rows] < 30).mean()) if out_rows.any() else None,
    frac_within30_all=float((absin[m_on] < 30).mean()))
chg = np.where(np.abs(np.diff(inn)) > 1e-9)[0] + 1
res["innov"]["obs_rate_hz"] = float(len(chg) / (t[chg[-1]] - t[chg[0]]))
res["innov"]["t_25th_obs_latch"] = float(t[chg[24]]) if len(chg) >= 25 else None
wb = wrapd(truth - e2)
res["wouldbe_innov_corrected_ref"] = dict(
    mean=float(wb[m_on & inl].mean()), rms=float(np.sqrt(np.mean(wb[m_on & inl]**2))),
    maxabs=float(np.abs(wb[m_on & inl]).max()),
    frac_within_gate30=float((np.abs(wb[m_on & inl]) < 30).mean()))

# ---------- timeline ----------
st = df["tlm_state"].values
t_flying = float(t[np.argmax(st == 4)])
t_landing = float(t[np.argmax(st == 5)])
lift_i = next((i for i in range(i0, n - 10) if np.all(tof[i:i + 10] > 0.08)), None)
fly = (t >= t_flying) & (t < t_landing)
volt = df["tlm_voltage_v"].values
res["timeline"] = dict(duration_s=float(t[-1]), t_arm=float(t[i0]),
                       t_liftoff_tof=float(t[lift_i]) if lift_i else None,
                       t_flying_state=t_flying, t_landing_state=t_landing,
                       volt_at_arm=float(volt[:5].mean()), volt_min=float(volt.min()),
                       volt_end=float(volt[-10:].mean()))

# ---------- yaw trajectory (real) ----------
def rate_of(sig_u):
    r = np.full(n, np.nan)
    dt_ = np.diff(t)
    ok = dt_ > 1e-6
    r[1:][ok] = np.diff(sig_u)[ok] / dt_[ok]
    return pd.Series(r).interpolate(limit_direction="both").rolling(
        9, center=True, min_periods=1).mean().values

truth_rate = rate_of(truth_u)
cmd = np.degrees(df["cmd_yaw_ref_rad"].values)
cmd_u = np.degrees(np.unwrap(df["cmd_yaw_ref_rad"].values))
cmd_rate = rate_of(cmd_u)
res["yaw_traj"] = dict(
    truth_range_deg=[float(truth_u[m_on].min()), float(truth_u[m_on].max())],
    net_rotation_deg=float(truth_u[-1] - truth_u[i0]),
    peak_rate_deg_s=float(truth_rate[fly][np.nanargmax(np.abs(truth_rate[fly]))]),
    t_peak_rate=float(t[fly][np.nanargmax(np.abs(truth_rate[fly]))]),
    cmd_range_deg=[float(cmd.min()), float(cmd.max())],
    cmd_peak_rate_deg_s=float(np.nanmax(np.abs(cmd_rate[fly]))))
# command segments (turns): |cmd_rate|>2 deg/s
man = (np.abs(cmd_rate) > 2.0) & fly
segs = []
i = 0
while i < n:
    if man[i]:
        j = i
        while j + 1 < n and (man[j + 1] or (t[j + 1] - t[j] < 0.05 and np.abs(cmd_rate[j + 1]) > 1)):
            j += 1
        segs.append(dict(t0=float(t[i]), t1=float(t[j]),
                         cmd0=float(cmd_u[i]), cmd1=float(cmd_u[j]),
                         truth0=float(truth_u[i]), truth1=float(truth_u[j])))
        i = j + 1
    else:
        i += 1
res["yaw_traj"]["cmd_segments"] = [s_ for s_ in segs if abs(s_["cmd1"] - s_["cmd0"]) > 5]

# ---------- EKF errors ----------
err1 = wrapd(e1 - truth)
err2 = wrapd(e2 - truth)
fin = fly & inl
res["ekf_err"] = {}
for nm, e in (("EKF1", err1), ("EKF2", err2)):
    A_ = np.polyfit(t[fin], e[fin], 1)
    res["ekf_err"][nm] = dict(
        flight_rms=float(np.sqrt(np.mean(e[fin]**2))),
        flight_mean=float(e[fin].mean()),
        flight_maxabs=float(np.abs(e[fin]).max()),
        drift_deg_per_min=float(A_[0] * 60))
res["ekf_err"]["ekf1_vs_ekf2_rms"] = float(np.sqrt(np.mean(wrapd(e1 - e2)[fin]**2)))
# windows: hover pre-turn / turns / post
if segs:
    t_ta = segs[0]["t0"]
    t_tb = segs[-1]["t1"]
    wins = {"pre_turn": fin & (t < t_ta - 1),
            "turn_all": fin & (t >= t_ta) & (t <= t_tb + 2),
            "post_turn": fin & (t > t_tb + 2)}
    res["ekf_err_phases"] = {
        nm: {k: dict(n=int(w.sum()), mean=float(e[w].mean()),
                     rms=float(np.sqrt(np.mean(e[w]**2))),
                     maxabs=float(np.abs(e[w]).max()))
             for k, w in wins.items() if w.sum() > 5}
        for nm, e in (("EKF1", err1), ("EKF2", err2))}
bm1 = np.hypot(df["tlm_bm_x_ut"], df["tlm_bm_y_ut"]).values
bm2 = np.hypot(df["tlm_ekf2_bm_x_ut"], df["tlm_ekf2_bm_y_ut"]).values
res["bm"] = dict(ekf1_final=[float(df["tlm_bm_x_ut"].values[-1]),
                             float(df["tlm_bm_y_ut"].values[-1])],
                 ekf2_final=[float(df["tlm_ekf2_bm_x_ut"].values[-1]),
                             float(df["tlm_ekf2_bm_y_ut"].values[-1])],
                 ekf1_norm_max=float(bm1.max()), ekf2_norm_max=float(bm2.max()))
gate = df["tlm_ekf2_gate"].astype(int).values
b7 = (gate & 128) > 0
res["gate_bit7_mag_recapture"] = dict(n=int(b7.sum()),
                                      t=[float(t[b7].min()), float(t[b7].max())]) if b7.any() else None
res["gate_z_reject_frac_flight"] = float(((gate & 8) > 0)[fly].mean())

# ---------- position ----------
ex, ey = df["error_x"].values, df["error_y"].values
r_err = np.hypot(ex, ey)
turn_mask = np.zeros(n, bool)
for s_ in segs:
    turn_mask |= (t >= s_["t0"]) & (t <= s_["t1"] + 2)
hover = fly & ~turn_mask & (t >= t_flying + 2)
res["position"] = dict(
    hover=dict(n=int(hover.sum()),
               rms_x_mm=float(np.sqrt(np.mean(ex[hover]**2)) * 1000),
               rms_y_mm=float(np.sqrt(np.mean(ey[hover]**2)) * 1000),
               rms_r_mm=float(np.sqrt(np.mean(r_err[hover]**2)) * 1000),
               p95_r_mm=float(np.percentile(r_err[hover], 95) * 1000),
               z_mean=float(pz[hover].mean()), z_std_mm=float(pz[hover].std() * 1000)),
    turn=dict(rms_r_mm=float(np.sqrt(np.mean(r_err[turn_mask & fly]**2)) * 1000),
              z_mean=float(pz[turn_mask & fly].mean()),
              z_min=float(pz[turn_mask & fly].min())) if turn_mask.any() else None,
    max_excursion_xy_m=[float(np.nanmax(np.abs(df["pos_x"].values[fly]))),
                        float(np.nanmax(np.abs(df["pos_y"].values[fly])))])

# ---------- flow: firmware axis determination ----------
def flow_analysis(dfx, truth_deg, label, fly_mask):
    tt = dfx["elapsed_time"].values
    nn = len(dfx)
    fvx = dfx["tlm_flow_vx_mps"].values
    fvy = dfx["tlm_flow_vy_mps"].values
    fstat = dfx["tlm_flow_status"].astype(int).values
    new_tlm = np.r_[True, np.diff(dfx["tlm_elapsed_ms"].values) != 0]
    valid = ((fstat & 4) > 0) & new_tlm & fly_mask
    p = dfx["tlm_p_rad_s"].values
    q = dfx["tlm_q_rad_s"].values
    r_ = dfx["tlm_r_rad_s"].values
    out = {}
    # (1) p/q leakage regression (axis-mapping fingerprint)
    X = np.column_stack([p, q, r_, np.ones(nn)])
    for nm, f_ in (("fvx", fvx), ("fvy", fvy)):
        coef, *_ = np.linalg.lstsq(X[valid], f_[valid], rcond=None)
        pred = X[valid] @ coef
        ss = 1 - np.sum((f_[valid] - pred)**2) / np.sum((f_[valid] - f_[valid].mean())**2)
        out[nm + "_vs_pqr"] = dict(coef_p=round(float(coef[0]), 4),
                                   coef_q=round(float(coef[1]), 4),
                                   coef_r=round(float(coef[2]), 4),
                                   R2=round(float(ss), 4))
    # (2) translation fit (4 conventions)
    tg = np.arange(tt[0], tt[-1], 0.02)
    def sg_deriv(x):
        half = 7
        idx = np.arange(-half, half + 1)
        A_ = np.vander(idx, 3, increasing=True)
        coefd = np.linalg.pinv(A_)[1] / 0.02
        xg = np.interp(tg, tt, x)
        xp = np.r_[xg[half:0:-1], xg, xg[-2:-half - 2:-1]]
        return np.interp(tt, tg, np.convolve(xp, coefd[::-1], mode="valid"))
    fill = lambda cser: pd.Series(cser).ffill().bfill().values
    vw = sg_deriv(fill(dfx["pos_x"].values)) + 1j * sg_deriv(fill(dfx["pos_y"].values))
    psi = np.radians(fill(truth_deg))
    Fc = fvx + 1j * fvy
    best = None
    for conjV, sgn, lab in [(False, 1, "RH_psi+"), (False, -1, "RH_psi-"),
                            (True, 1, "LH_psi+"), (True, -1, "LH_psi-")]:
        Vb = (np.conj(vw) if conjV else vw) * np.exp(-1j * sgn * psi)
        cc = np.sum(Fc[valid] * np.conj(Vb[valid]))
        phi0 = np.angle(cc)
        Vr = Vb * np.exp(1j * phi0)
        a = np.r_[Fc[valid].real, Fc[valid].imag]
        b = np.r_[Vr[valid].real, Vr[valid].imag]
        rho = np.corrcoef(a, b)[0, 1]
        out.setdefault("conv", {})[lab] = dict(phi0_deg=round(float(np.degrees(phi0)), 2),
                                               corr=round(float(rho), 4))
        if best is None or rho > best[0]:
            best = (rho, lab, np.degrees(phi0), conjV, sgn)
    out["best_conv"] = dict(label=best[1], corr=round(float(best[0]), 4),
                            phi0_deg=round(float(best[2]), 2))
    # forced LH_psi+ (11:19 winning convention) for comparability
    Vb = np.conj(vw) * np.exp(-1j * psi)
    cc = np.sum(Fc[valid] * np.conj(Vb[valid]))
    Vr = Vb * np.exp(1j * np.angle(cc))
    a = np.r_[Fc[valid].real, Fc[valid].imag]
    b = np.r_[Vr[valid].real, Vr[valid].imag]
    out["forced_LH_psi+"] = dict(phi0_deg=round(float(np.degrees(np.angle(cc))), 2),
                                 corr=round(float(np.corrcoef(a, b)[0, 1]), 4))
    # scale on dominant axis of forced convention
    sc = float(np.dot(np.r_[fvx[valid], fvy[valid]], b) / np.dot(b, b))
    out["forced_scale"] = round(sc, 3)
    out["n"] = int(valid.sum())
    out["speed_p90"] = round(float(np.percentile(np.abs(vw[valid]), 90)), 3)
    return out

res["flow_1820"] = flow_analysis(df, truth, "1820", fly & inl)
try:
    dfo = pd.read_csv(OLDCSV)
    sto = dfo["tlm_state"].values
    flyo = (dfo["elapsed_time"].values >= dfo["elapsed_time"].values[np.argmax(sto == 4)]) & \
           (dfo["elapsed_time"].values < dfo["elapsed_time"].values[np.argmax(sto == 5)])
    res["flow_1119_oldfw"] = flow_analysis(dfo, dfo["yaw_true_corrected_deg"].values,
                                           "1119", flyo)
except Exception as e:  # noqa
    res["flow_1119_oldfw"] = str(e)

# ---------- save ----------
out_df = df.copy()
out_df["yaw_true_corrected_deg"] = np.round(truth, 4)
out_df["yaw_ref_corrected_rad"] = np.round(np.radians(truth), 6)
out_df["mocap_glitch"] = (~inl).astype(int)
out_df.to_csv(OUT + "/log_corrected.csv", index=False)

meta = {
    "log": LOG, "session": "20260731_182025",
    "raw_yaw_definition": "yaw_raw_deg = atan2(f_x, f_z), f = R(q)*[1,0,0]; == mocap_heading_deg",
    "correct_transform": {
        "yaw_sign": int(s_best), "yaw_offset_deg": round(c, 3),
        "formula": f"yaw_true_corrected_deg = wrap({s_best:+d}*yaw_raw + {c:.3f}) on clean rows; "
                   "~95-deg mocap glitch rows (mocap_glitch=1) bridged via gyro+interp",
        "anchor": "ground-stationary window t=[0.17,2.0]s vs gyro-integrated yaw "
                  "(reset 0 at arm t=0.063 = onboard frame)"},
    "session_meta_transform": {"yaw_sign": -1, "yaw_offset_deg": 89.9},
    "new_columns": {
        "yaw_true_corrected_deg": "truth yaw in onboard yaw_est frame, deg, wrapped",
        "yaw_ref_corrected_rad": "reference that should have been sent (onboard frame), rad",
        "mocap_glitch": "1 = mocap ~95-deg alternate-solution glitch row (truth bridged by gyro)"},
    "stats": res}
with open(OUT + "/truth_fit.json", "w") as f:
    json.dump(meta, f, indent=1, ensure_ascii=False)
print(json.dumps(res, indent=1, ensure_ascii=False))
