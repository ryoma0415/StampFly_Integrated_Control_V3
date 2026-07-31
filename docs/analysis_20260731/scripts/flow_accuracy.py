#!/usr/bin/env python
"""Flow accuracy vs MoCap-derived body-frame reference velocity."""
import numpy as np
import pandas as pd
def savgol_filter(x, window, poly, deriv=0, delta=1.0):
    """numpy-only Savitzky-Golay (mirror-padded)."""
    half = window // 2
    idx = np.arange(-half, half + 1)
    A = np.vander(idx, poly + 1, increasing=True)  # columns: idx^0..idx^poly
    pinv = np.linalg.pinv(A)
    import math
    coef = pinv[deriv] * math.factorial(deriv) / (delta ** deriv)
    xp = np.r_[x[half:0:-1], x, x[-2:-half-2:-1]]
    return np.convolve(xp, coef[::-1], mode="valid")

BASE = "/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731"
df = pd.read_csv(BASE + "/log_corrected.csv")
t = df.elapsed_time.values
fs_hz = (len(t) - 1) / (t[-1] - t[0])
print("PC log rate: %.1f Hz (median dt %.1f ms, p95 %.1f ms)" %
      (fs_hz, 1000*np.median(np.diff(t)), 1000*np.percentile(np.diff(t), 95)))

# ---- reference world velocity from MoCap position (SG derivative, ~0.3s window) ----
# resample to uniform 50Hz grid to be robust to timestamp jitter
tg = np.arange(t[0], t[-1], 0.02)
def sg_deriv(x, win=15):
    xg = np.interp(tg, t, x)
    vg = savgol_filter(xg, win, 2, deriv=1, delta=0.02)
    return np.interp(t, tg, vg)
win = 15
fill = lambda c: pd.Series(c).ffill().bfill().values
vwx = sg_deriv(fill(df.pos_x.values))
vwy = sg_deriv(fill(df.pos_y.values))
vwx_raw = sg_deriv(fill(df.raw_pos_x.values))
print("pos vs raw_pos vel diff RMS: %.4f m/s" % np.sqrt(np.mean((vwx-vwx_raw)**2)))

psi = np.deg2rad(pd.Series(df.yaw_true_corrected_deg.values).ffill().bfill().values)
print("NaN in psi:", np.isnan(psi).sum(), " NaN in pos:", df.pos_x.isna().sum())

# ---- flow samples: dedupe telemetry frames ----
new_tlm = np.r_[True, np.diff(df.tlm_elapsed_ms.values) != 0]
fvx = df.tlm_flow_vx_mps.values
fvy = df.tlm_flow_vy_mps.values
fs_stat = df.tlm_flow_status.values.astype(int)
valid = ((fs_stat & 4) > 0) & new_tlm  # vel_valid & fresh tlm frame

# ---- convention fit: complex, on valid in-air samples ----
F = fvx + 1j * fvy
Vw = vwx + 1j * vwy
m = valid & (t > 4.5) & (t < 53.0)
print("fit samples:", m.sum())
results = []
for conjV, sgn_psi, label in [
    (False, +1, "V_b=e^{-i psi}V_w        (RH, psi=+corr)"),
    (False, -1, "V_b=e^{+i psi}V_w        (RH, psi=-corr)"),
    (True,  +1, "V_b=e^{-i psi}conj(V_w)  (LH, psi=+corr)"),
    (True,  -1, "V_b=e^{+i psi}conj(V_w)  (LH, psi=-corr)"),
]:
    Vw_c = np.conj(Vw) if conjV else Vw
    Vb = Vw_c * np.exp(-1j * sgn_psi * psi)
    # fit constant residual rotation phi0: F ~ e^{-i phi0} Vb
    cc = np.sum(F[m] * np.conj(Vb[m]))
    phi0 = np.angle(cc)
    Vb_rot = Vb * np.exp(1j * phi0)
    # correlation of real/imag stacked
    a = np.r_[F[m].real, F[m].imag]
    b = np.r_[Vb_rot[m].real, Vb_rot[m].imag]
    r = np.corrcoef(a, b)[0, 1]
    results.append((r, phi0, conjV, sgn_psi, label))
    print("%s  phi0=%7.2f deg  corr=%.4f" % (label, np.rad2deg(phi0), r))

r_best, phi0, conjV, sgn_psi, label = max(results, key=lambda x: x[0])
print("\nBEST:", label, "phi0=%.2f deg corr=%.4f" % (np.rad2deg(phi0), r_best))

# ---- build best-fit reference body velocity, full timeline ----
Vw_c = np.conj(Vw) if conjV else Vw
Vb = Vw_c * np.exp(-1j * sgn_psi * psi) * np.exp(1j * phi0)
rvx, rvy = Vb.real, Vb.imag

# ---- lag estimation via cross-correlation on interpolated uniform grid ----
def est_lag(sig_f, sig_r, mask, max_ms=400):
    tt = t[mask]
    a = sig_f[mask] - sig_f[mask].mean()
    b_full = sig_r - sig_r.mean()
    lags = np.arange(-max_ms, max_ms + 1, 2) / 1000.0
    cs = []
    for L in lags:
        bi = np.interp(tt - L, t, b_full)  # reference shifted
        bi = bi - bi.mean()
        cs.append(np.dot(a, bi) / (np.linalg.norm(a) * np.linalg.norm(bi) + 1e-12))
    cs = np.array(cs)
    i = np.argmax(cs)
    # parabolic refine
    if 0 < i < len(cs) - 1:
        d = (cs[i-1] - cs[i+1]) / (2 * (cs[i-1] - 2*cs[i] + cs[i+1]) + 1e-12)
        lag = lags[i] + d * 0.002
    else:
        lag = lags[i]
    return lag, cs[i]

assert np.isfinite(rvx).all() and np.isfinite(rvy).all(), "NaN in reference velocity"
lag_x, cx = est_lag(fvx, rvx, m, max_ms=300)
lag_y, cy_ = est_lag(fvy, rvy, m, max_ms=300)
print("\nlag (flow lags ref by): x=%+.0f ms (peak corr %.4f), y=%+.0f ms (%.4f)" %
      (lag_x*1000, cx, lag_y*1000, cy_))

# lag-compensated reference
lag = 0.5 * (lag_x + lag_y)
rvx_l = np.interp(t - lag, t, rvx)
rvy_l = np.interp(t - lag, t, rvy)
# re-fit phi0 on lag-compensated reference
Vb_l = (rvx_l + 1j * rvy_l)
cc2 = np.sum((fvx[m] + 1j*fvy[m]) * np.conj(Vb_l[m]))
dphi = np.angle(cc2)
phi0 = phi0 + dphi
Vb_l = Vb_l * np.exp(1j * dphi)
rvx_l, rvy_l = Vb_l.real, Vb_l.imag
print("re-fit phi0 after lag comp: total phi0=%.2f deg (delta %.2f)" %
      (np.rad2deg(phi0), np.rad2deg(dphi)))

# ---- per-axis stats (lag-compensated) ----
def axis_stats(f, r_, mask, name):
    ex = f[mask] - r_[mask]
    corr = np.corrcoef(f[mask], r_[mask])[0, 1]
    scale = np.dot(f[mask], r_[mask]) / np.dot(r_[mask], r_[mask])
    rms = np.sqrt(np.mean(ex**2))
    print("%s: corr=%.4f scale=%.3f rms=%.4f m/s bias=%+.4f std=%.4f n=%d" %
          (name, corr, scale, rms, ex.mean(), ex.std(), mask.sum()))
    return ex

print("\n== full flight (t=4.5-53, lag-comp) ==")
ex = axis_stats(fvx, rvx_l, m, "vx")
ey = axis_stats(fvy, rvy_l, m, "vy")

# speed for windows
spd = np.abs(Vb)
hover = m & (spd < 0.3)
moving = m & (spd >= 0.3)
print("\nspeed |v_ref|: p50=%.3f p90=%.3f p99=%.3f max=%.3f; frac>=0.3: %.1f%%" %
      (np.median(spd[m]), np.percentile(spd[m], 90), np.percentile(spd[m], 99),
       spd[m].max(), 100*moving.sum()/m.sum()))
print("\n== hover (|v|<0.3) ==")
exh = axis_stats(fvx, rvx_l, hover, "vx")
eyh = axis_stats(fvy, rvy_l, hover, "vy")
if moving.sum() > 20:
    print("== moving (|v|>=0.3) ==")
    axis_stats(fvx, rvx_l, moving, "vx")
    axis_stats(fvy, rvy_l, moving, "vy")

# ---- intrinsic flow noise: first-difference / sqrt2 on consecutive fresh samples in hover ----
idx = np.where(valid & new_tlm)[0]
# consecutive tlm frames ~43ms apart; hover mask
h_idx = [i for i in idx if hover[i]]
h_idx = np.array(h_idx)
dfx = np.diff(fvx[h_idx])
dfy = np.diff(fvy[h_idx])
# robust sigma from first difference (white part)
sig_fx = np.median(np.abs(dfx)) / 0.6745 / np.sqrt(2)
sig_fy = np.median(np.abs(dfy)) / 0.6745 / np.sqrt(2)
print("\nflow intrinsic white sigma (hover, first-diff robust): x=%.4f y=%.4f m/s" % (sig_fx, sig_fy))
drx = np.diff(rvx_l[h_idx]); dry = np.diff(rvy_l[h_idx])
print("ref  vel first-diff robust sigma: x=%.4f y=%.4f m/s (smoothed -> colored)" %
      (np.median(np.abs(drx))/0.6745/np.sqrt(2), np.median(np.abs(dry))/0.6745/np.sqrt(2)))

# residual robust sigma
print("residual robust sigma (hover): x=%.4f y=%.4f m/s" %
      (np.median(np.abs(exh - np.median(exh)))/0.6745, np.median(np.abs(eyh - np.median(eyh)))/0.6745))

# ---- quality dependence ----
sq = df.tlm_flow_squal.values.astype(float)
alt = df.tlm_altitude_tof_m.values
err_mag = np.full(len(df), np.nan)
sel = m
err_mag[sel] = np.sqrt((fvx[sel]-rvx_l[sel])**2 + (fvy[sel]-rvy_l[sel])**2)
print("\n== |err| vs SQUAL bins ==")
for lo, hi in [(50, 70), (70, 80), (80, 90), (90, 100), (100, 135)]:
    mm = sel & (sq >= lo) & (sq < hi)
    if mm.sum() > 10:
        e = err_mag[mm]
        print("SQUAL[%3d-%3d): n=%4d  rms=%.4f  p50=%.4f" % (lo, hi, mm.sum(), np.sqrt(np.mean(e**2)), np.median(e)))
print("== |err| vs ToF alt bins ==")
for lo, hi in [(0.08, 0.15), (0.15, 0.25), (0.25, 0.32), (0.32, 0.45)]:
    mm = sel & (alt >= lo) & (alt < hi)
    if mm.sum() > 10:
        e = err_mag[mm]
        print("alt[%.2f-%.2f): n=%4d  rms=%.4f  p50=%.4f" % (lo, hi, mm.sum(), np.sqrt(np.mean(e**2)), np.median(e)))

# yaw-rate dependence (gyro comp for p/q; r not compensated -> omega x r arm)
rr = np.abs(df.tlm_r_rad_s.values)
pq = np.sqrt(df.tlm_p_rad_s.values**2 + df.tlm_q_rad_s.values**2)
print("== |err| vs |r| bins ==")
for lo, hi in [(0, 0.1), (0.1, 0.3), (0.3, 0.8)]:
    mm = sel & (rr >= lo) & (rr < hi)
    if mm.sum() > 10:
        e = err_mag[mm]
        print("|r|[%.1f-%.1f): n=%4d  rms=%.4f  p50=%.4f" % (lo, hi, mm.sum(), np.sqrt(np.mean(e**2)), np.median(e)))
print("== |err| vs |pq| bins ==")
for lo, hi in [(0, 0.1), (0.1, 0.3), (0.3, 1.0), (1.0, 5.0)]:
    mm = sel & (pq >= lo) & (pq < hi)
    if mm.sum() > 10:
        e = err_mag[mm]
        print("|pq|[%.1f-%.1f): n=%4d  rms=%.4f  p50=%.4f" % (lo, hi, mm.sum(), np.sqrt(np.mean(e**2)), np.median(e)))

# turn window
turn = sel & (t >= 39) & (t <= 48)
print("\nturn window t=39-48: n=%d rms|err|=%.4f  vs hover-outside rms=%.4f" %
      (turn.sum(), np.sqrt(np.nanmean(err_mag[turn]**2)),
       np.sqrt(np.nanmean(err_mag[sel & ~((t >= 39) & (t <= 48))]**2))))

# ---- band-limited comparison (both signals LPF'd to ~1.5Hz, fair signal-band compare) ----
def lpf(x, win_s=0.66):
    n = int(win_s * 50) | 1
    k = np.hanning(n); k /= k.sum()
    xg = np.interp(tg, t, x)
    yg = np.convolve(xg, k, mode="same")
    return np.interp(t, tg, yg)

fvx_l_ = lpf(np.where(valid | ~new_tlm, fvx, np.nan) * 0 + fvx)  # flow held values ok
fvy_l_ = lpf(fvy)
rvx_ll = lpf(rvx_l); rvy_ll = lpf(rvy_l)
print("\n== band-limited (<~1.5Hz) comparison, t=4.5-53 ==")
for f_, r_, nm in [(fvx_l_, rvx_ll, "vx"), (fvy_l_, rvy_ll, "vy")]:
    corr = np.corrcoef(f_[m], r_[m])[0, 1]
    scale = np.dot(f_[m], r_[m]) / np.dot(r_[m], r_[m])
    rms = np.sqrt(np.mean((f_[m]-r_[m])**2))
    print("%s: corr=%.4f scale=%.3f rms=%.4f m/s  (sig std ref=%.4f flow=%.4f)" %
          (nm, corr, scale, rms, r_[m].std(), f_[m].std()))

# ---- 2s-window averaging: effective sigma of windowed mean error (mechanism A granularity) ----
print("\n== 2s window mean error (flow - ref), stride 1s ==")
wins = []
for t0 in np.arange(5, 51, 1.0):
    mm = m & (t >= t0) & (t < t0 + 2)
    if mm.sum() > 30:
        wins.append([fvx[mm].mean()-rvx_l[mm].mean(), fvy[mm].mean()-rvy_l[mm].mean(),
                     np.abs(Vb[mm]).mean()])
wins = np.array(wins)
print("n_windows=%d  sigma of window-mean err: x=%.4f y=%.4f m/s  (|mean|max x=%.3f y=%.3f)" %
      (len(wins), wins[:,0].std(), wins[:,1].std(), np.abs(wins[:,0]).max(), np.abs(wins[:,1]).max()))
# implied yaw sigma at given speeds: sigma_psi ~ sigma_perp_winmean / v
for v in [0.3, 0.4]:
    sp = np.sqrt(0.5*(wins[:,0].std()**2 + wins[:,1].std()**2))
    print("implied 2s-window sigma_psi at v=%.1f m/s: %.2f deg (using per-axis win-mean sigma %.4f)" %
          (v, np.rad2deg(sp / v), sp))

# ---- autocorrelation time of residual (colored-ness, kappa_color) ----
res = (fvx - rvx_l)
idx2 = np.where(m)[0]
rr_ = res[idx2] - res[idx2].mean()
ac = np.correlate(rr_, rr_, "full")[len(rr_)-1:] / np.dot(rr_, rr_)
tau_i = np.argmax(ac < 1/np.e)
print("residual x autocorr 1/e time: ~%.0f ms (samples are ~43ms apart incl. dup-removal)" % (tau_i*43))

# save intermediates for plotting
np.savez(BASE + "/flow_ref.npz", t=t, fvx=fvx, fvy=fvy, rvx=rvx_l, rvy=rvy_l,
         valid=valid, m=m, spd=spd, sq=sq, alt=alt, rr=df.tlm_r_rad_s.values,
         err_mag=err_mag, phi0=phi0, sgn_psi=sgn_psi, conjV=conjV, lag=lag,
         hover=hover)
print("\nsaved flow_ref.npz  phi0=%.2f deg sgn=%d conjV=%s lag=%.0fms" %
      (np.rad2deg(phi0), sgn_psi, conjV, lag*1000))
