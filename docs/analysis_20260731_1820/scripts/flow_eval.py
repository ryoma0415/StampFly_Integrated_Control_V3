#!/usr/bin/env python
"""18:20 flight: optical-flow calibration state assessment.

Unified pipeline applied to both 18:20 (new firmware axes) and 11:19 (old axes):
 - reference body velocity from MoCap (SG derivative, LH convention conj(Vw)e^{-i psi})
 - rotation-invariant lag via complex cross-correlation magnitude
 - full 2x2 matrix fit F ~ A Vb0 -> SVD rotation + per-axis stretch
 - complex phi0/scale fit; phase-vs-frequency (rotation vs lag disambiguation)
 - p/q gyro-leak regression (axis mapping fingerprint)
 - hover sigma / turn-window quality / |r| bins
 - dead reckoning vs mocap trajectory (18:20)
"""
import json
import math
import numpy as np
import pandas as pd

SP = ("/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-"
      "Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/"
      "ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad")
OUT = {}


def savgol(x, window, poly, deriv=0, delta=1.0):
    half = window // 2
    idx = np.arange(-half, half + 1)
    A = np.vander(idx, poly + 1, increasing=True)
    coef = np.linalg.pinv(A)[deriv] * math.factorial(deriv) / (delta ** deriv)
    xp = np.r_[x[half:0:-1], x, x[-2:-half - 2:-1]]
    return np.convolve(xp, coef[::-1], mode="valid")


def pipeline(df, t_lo, t_hi, turn_windows, label):
    res = {"label": label}
    t = df.elapsed_time.values
    tg = np.arange(t[0], t[-1], 0.02)
    fill = lambda c: pd.Series(c).ffill().bfill().values

    def sg_deriv(x, win=9):
        xg = np.interp(tg, t, fill(x))
        vg = savgol(xg, win, 2, deriv=1, delta=0.02)
        return np.interp(t, tg, vg)

    vwx = sg_deriv(df.pos_x.values)
    vwy = sg_deriv(df.pos_y.values)
    psi = np.deg2rad(fill(df.yaw_true_corrected_deg.values))
    fvx = df.tlm_flow_vx_mps.values
    fvy = df.tlm_flow_vy_mps.values
    new_tlm = np.r_[True, np.diff(df.tlm_elapsed_ms.values) != 0]
    fstat = df.tlm_flow_status.values.astype(int)
    valid = ((fstat & 4) > 0) & new_tlm
    m = valid & (t > t_lo) & (t < t_hi)
    res["n_fit"] = int(m.sum())

    Vw = vwx + 1j * vwy
    Vb0 = np.conj(Vw) * np.exp(-1j * psi)          # LH family (11:19 winner)
    F = fvx + 1j * fvy

    # ---- rotation-invariant lag: max |sum F(t) conj(Vb0(t-L))| ----
    def cplx_lag(mask, max_ms=400, step_ms=4):
        tt = t[mask]
        Fm = F[mask] - F[mask].mean()
        lags = np.arange(-max_ms, max_ms + 1, step_ms) / 1000.0
        best = []
        for L in lags:
            b = np.interp(tt - L, t, Vb0.real) + 1j * np.interp(tt - L, t, Vb0.imag)
            b = b - b.mean()
            c = np.abs(np.vdot(b, Fm)) / (np.linalg.norm(Fm) * np.linalg.norm(b) + 1e-12)
            best.append(c)
        best = np.array(best)
        i = int(np.argmax(best))
        if 0 < i < len(best) - 1:
            d = (best[i - 1] - best[i + 1]) / (2 * (best[i - 1] - 2 * best[i] + best[i + 1]) + 1e-12)
            lag = lags[i] + d * step_ms / 1000.0
        else:
            lag = lags[i]
        return float(lag), float(best[i])

    lag, lag_corr = cplx_lag(m)
    res["lag_ms"] = round(lag * 1000, 1)
    res["lag_peak_corr"] = round(lag_corr, 4)

    Vb0_l = (np.interp(t - lag, t, Vb0.real) + 1j * np.interp(t - lag, t, Vb0.imag))

    # ---- full 2x2 matrix fit + SVD ----
    R0 = np.c_[Vb0_l.real[m], Vb0_l.imag[m]]
    F0 = np.c_[fvx[m], fvy[m]]
    A0, *_ = np.linalg.lstsq(R0, F0, rcond=None)
    A0 = A0.T
    U, S, Vt = np.linalg.svd(A0)
    rot = U @ Vt
    if np.linalg.det(rot) < 0:   # guard: reflection would mean handedness mismatch
        res["reflection"] = True
    ang = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
    stretch = rot.T @ A0
    res["A"] = [[round(float(v), 3) for v in row] for row in A0]
    res["rot_deg"] = round(ang, 2)
    res["sv"] = [round(float(v), 3) for v in S]
    res["stretch_diag"] = [round(float(stretch[0, 0]), 3), round(float(stretch[1, 1]), 3)]

    # ---- complex phi0 + isotropic scale ----
    cc = np.vdot(Vb0_l[m], F[m])          # sum F * conj(Vb)
    phi0 = float(np.angle(cc))
    Vb = Vb0_l * np.exp(1j * phi0)
    scale_iso = float(np.abs(cc) / np.dot(np.abs(Vb0_l[m]), np.abs(Vb0_l[m])) if False else
                      (np.vdot(Vb[m], F[m]).real / np.vdot(Vb[m], Vb[m]).real))
    res["phi0_deg"] = round(math.degrees(phi0), 2)
    res["scale_iso"] = round(scale_iso, 3)
    a = np.r_[F[m].real, F[m].imag]
    b = np.r_[Vb[m].real, Vb[m].imag]
    res["corr"] = round(float(np.corrcoef(a, b)[0, 1]), 4)

    # per-axis (rotation-compensated reference)
    rvx, rvy = Vb.real, Vb.imag
    for nm, f_, r_ in (("x", fvx, rvx), ("y", fvy, rvy)):
        res["scale_" + nm] = round(float(np.dot(f_[m], r_[m]) / np.dot(r_[m], r_[m])), 3)
        res["corr_" + nm] = round(float(np.corrcoef(f_[m], r_[m])[0, 1]), 4)

    # restricted to real motion
    spd = np.abs(Vb0_l)
    for thr in (0.10, 0.15):
        mm = m & (spd > thr)
        if mm.sum() > 40:
            cc2 = np.vdot(Vb0_l[mm], F[mm])
            res[f"phi0_deg_v{thr}"] = round(math.degrees(float(np.angle(cc2))), 2)
            Vb2 = Vb0_l * np.exp(1j * np.angle(cc2))
            res[f"scale_iso_v{thr}"] = round(
                float(np.vdot(Vb2[mm], F[mm]).real / np.vdot(Vb2[mm], Vb2[mm]).real), 3)
            res[f"n_v{thr}"] = int(mm.sum())

    res["spd_p50_p90_p99_max"] = [round(float(np.percentile(spd[m], p)), 3)
                                  for p in (50, 90, 99)] + [round(float(spd[m].max()), 3)]

    # ---- phase vs frequency (rotation vs lag) on uniform grid, fly window ----
    gm = (tg > t_lo) & (tg < t_hi)
    Fg = np.interp(tg, t, F.real) + 1j * np.interp(tg, t, F.imag)
    Vg = np.interp(tg, t, Vb0.real) + 1j * np.interp(tg, t, Vb0.imag)  # NO lag comp here
    Fg, Vg = Fg[gm], Vg[gm]
    nseg, step = 512, 256
    win = np.hanning(nseg)
    Sfv = np.zeros(nseg, complex); Sff = np.zeros(nseg); Svv = np.zeros(nseg); k = 0
    for i0 in range(0, len(Fg) - nseg + 1, step):
        fs_ = np.fft.fft((Fg[i0:i0 + nseg] - Fg[i0:i0 + nseg].mean()) * win)
        vs_ = np.fft.fft((Vg[i0:i0 + nseg] - Vg[i0:i0 + nseg].mean()) * win)
        Sfv += fs_ * np.conj(vs_); Sff += np.abs(fs_) ** 2; Svv += np.abs(vs_) ** 2
        k += 1
    freqs = np.fft.fftfreq(nseg, 0.02)
    coh = np.abs(Sfv) ** 2 / (Sff * Svv + 1e-20)
    band = (np.abs(freqs) > 0.08) & (np.abs(freqs) < 2.5) & (coh > 0.3)
    res["phase_fit_nbins"] = int(band.sum())
    if band.sum() >= 6:
        ph = np.angle(Sfv[band] * np.exp(-1j * phi0))      # de-rotate by coarse phi0
        w = coh[band] * Svv[band]
        X = np.c_[np.ones(band.sum()), -2 * np.pi * freqs[band]]
        beta, *_ = np.linalg.lstsq(X * w[:, None], ph * w, rcond=None)
        res["phase_intercept_phi0_deg"] = round(math.degrees(beta[0] + phi0), 2)
        res["phase_slope_lag_ms"] = round(beta[1] * 1000, 1)
    res["_spec"] = (freqs, coh, np.angle(Sfv * np.exp(-1j * phi0)), band, phi0)

    # ---- residual stats: hover/turn ----
    resid_x = fvx - rvx
    resid_y = fvy - rvy
    hover = m & (spd < 0.10)
    rr = np.abs(df.tlm_r_rad_s.values)
    turn = np.zeros(len(t), bool)
    for a_, b_ in turn_windows:
        turn |= (t >= a_) & (t <= b_)
    emag = np.hypot(resid_x, resid_y)
    rb = lambda e: float(np.median(np.abs(e - np.median(e))) / 0.6745)
    res["hover_sigma"] = {"x_std": round(float(resid_x[hover].std()), 4),
                          "y_std": round(float(resid_y[hover].std()), 4),
                          "x_robust": round(rb(resid_x[hover]), 4),
                          "y_robust": round(rb(resid_y[hover]), 4),
                          "n": int(hover.sum())}
    res["turn_rms"] = round(float(np.sqrt(np.mean(emag[m & turn] ** 2))), 4) if (m & turn).sum() else None
    res["nonturn_rms"] = round(float(np.sqrt(np.mean(emag[m & ~turn] ** 2))), 4)
    res["r_bins"] = {}
    for lo, hi in [(0, 0.1), (0.1, 0.5), (0.5, 1.0), (1.0, 2.5)]:
        mm = m & (rr >= lo) & (rr < hi)
        if mm.sum() > 10:
            res["r_bins"][f"{lo}-{hi}"] = {"n": int(mm.sum()),
                                           "rms": round(float(np.sqrt(np.mean(emag[mm] ** 2))), 4)}
    # outliers
    res["outliers_gt0.3"] = int((emag[m] > 0.3).sum())
    res["outliers_frac"] = round(float((emag[m] > 0.3).mean()), 4)

    # ---- p/q gyro-leak regression on residuals (axis-mapping fingerprint) ----
    p_ = df.tlm_p_rad_s.values
    q_ = df.tlm_q_rad_s.values
    X = np.c_[np.ones(m.sum()), p_[m], q_[m]]
    leak = {}
    for nm, y_ in (("x", resid_x[m]), ("y", resid_y[m])):
        beta, res_, *_ = np.linalg.lstsq(X, y_, rcond=None)
        yhat = X @ beta
        ss_res = float(np.sum((y_ - yhat) ** 2)); ss_tot = float(np.sum((y_ - y_.mean()) ** 2))
        sig2 = ss_res / (len(y_) - 3)
        covb = sig2 * np.linalg.inv(X.T @ X)
        leak[nm] = {"coef_p": round(float(beta[1]), 3), "coef_q": round(float(beta[2]), 3),
                    "se_p": round(float(np.sqrt(covb[1, 1])), 3),
                    "se_q": round(float(np.sqrt(covb[2, 2])), 3),
                    "R2": round(1 - ss_res / ss_tot, 4)}
    leak["std_p"] = round(float(p_[m].std()), 3)
    leak["std_q"] = round(float(q_[m].std()), 3)
    res["pq_leak"] = leak

    res["_arrays"] = dict(t=t, fvx=fvx, fvy=fvy, rvx=rvx, rvy=rvy, m=m, valid=valid,
                          psi=psi, spd=spd, hover=hover, F=F, Vb0_l=Vb0_l, phi0=phi0,
                          scale_iso=scale_iso, new_tlm=new_tlm, emag=emag, rr=rr)
    return res


# ================= 18:20 =================
dfb = pd.read_csv(SP + "/a0731b/log_corrected.csv")
turns_b = [(23.0, 26.5), (30.9, 33.2), (36.8, 40.2), (43.9, 47.0)]
rb = pipeline(dfb, 5.5, 53.0, turns_b, "1820")

# ================= 11:19 =================
dfa = pd.read_csv(SP + "/a0731/log_corrected.csv")
ra = pipeline(dfa, 4.5, 53.0, [(39.0, 48.0)], "1119")

for r in (rb, ra):
    out = {k: v for k, v in r.items() if not k.startswith("_")}
    OUT[r["label"]] = out
    print("=" * 30, r["label"], "=" * 30)
    print(json.dumps(out, indent=1))

d = ((rb["phi0_deg"] - ra["phi0_deg"]) + 180) % 360 - 180
OUT["delta_phi0_deg_1820_minus_1119"] = round(d, 2)
print("\nDelta phi0 (1820-1119): %.2f deg (expected +90 if new axes only change)" % d)

# ---- Dead reckoning (18:20): integrate flow on fresh frames ----
arr = rb["_arrays"]
t = arr["t"]; F = arr["F"]; psi = arr["psi"]; new_tlm = arr["new_tlm"]
fstat = dfb.tlm_flow_status.values.astype(int)
phi0 = arr["phi0"]; s_iso = arr["scale_iso"]
t0, t1 = 4.0, 53.5
idx = np.where(new_tlm & (t >= t0) & (t <= t1))[0]
tt = t[idx]
vel_ok = ((fstat[idx] & 4) > 0)
Fb = F[idx].copy()
# hold last valid on invalid frames
last = 0 + 0j
held = 0
for i in range(len(Fb)):
    if vel_ok[i]:
        last = Fb[i]
    else:
        Fb[i] = last
        held += 1
res_dr = {"n_frames": len(idx), "n_held_invalid": held}


def dead_reckon(rot_extra_deg=0.0, use_scale=True):
    Vb_est = Fb * np.exp(-1j * (phi0 + np.deg2rad(rot_extra_deg)))
    if use_scale:
        Vb_est = Vb_est / s_iso
    Vw_est = np.conj(Vb_est * np.exp(1j * psi[idx]))
    dt = np.r_[np.diff(tt), 0.043]
    px = dfb.pos_x.values[idx][0] + np.cumsum(Vw_est.real * dt)
    py = dfb.pos_y.values[idx][0] + np.cumsum(Vw_est.imag * dt)
    ex = px - dfb.pos_x.values[idx]
    ey = py - dfb.pos_y.values[idx]
    em = np.hypot(ex, ey)
    return px, py, em


px, py, em = dead_reckon()
res_dr["fitted_frame"] = {"terminal_err_m": round(float(em[-1]), 3),
                          "max_err_m": round(float(em.max()), 3),
                          "rms_err_m": round(float(np.sqrt(np.mean(em ** 2))), 3),
                          "err_at_30s_m": round(float(em[np.argmin(np.abs(tt - 30))], ), 3),
                          "drift_mps": round(float(em[-1] / (tt[-1] - tt[0])), 4)}
_, _, em90 = dead_reckon(rot_extra_deg=90.0)
res_dr["plus90_frame_terminal_err_m"] = round(float(em90[-1]), 3)
_, _, em_ns = dead_reckon(use_scale=False)
res_dr["fitted_rot_noscale_terminal_err_m"] = round(float(em_ns[-1]), 3)
OUT["dead_reckoning_1820"] = res_dr
print("\nDR:", json.dumps(res_dr, indent=1))

with open(SP + "/a0731b/flow_eval.json", "w") as f:
    json.dump(OUT, f, indent=1)

np.savez(SP + "/a0731b/flow_eval.npz",
         t=t, fvx=arr["fvx"], fvy=arr["fvy"], rvx=arr["rvx"], rvy=arr["rvy"],
         m=arr["m"], spd=arr["spd"], emag=arr["emag"], rr=arr["rr"],
         dr_t=tt, dr_px=px, dr_py=py, dr_em=em,
         mocap_x=dfb.pos_x.values[idx], mocap_y=dfb.pos_y.values[idx],
         freqs=rb["_spec"][0], coh=rb["_spec"][1], ph=rb["_spec"][2],
         band=rb["_spec"][3], phi0=rb["_spec"][4],
         a_t=ra["_arrays"]["t"], a_fvx=ra["_arrays"]["fvx"], a_fvy=ra["_arrays"]["fvy"],
         a_rvx=ra["_arrays"]["rvx"], a_rvy=ra["_arrays"]["rvy"], a_m=ra["_arrays"]["m"])
print("saved flow_eval.json / flow_eval.npz")
