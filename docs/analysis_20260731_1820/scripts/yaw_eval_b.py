#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""20260731_182025: 4系統ヨー精度評価 + EKF1"RMS80°"分解 + b_m挙動 + log_fixed.csv生成."""
import json
import numpy as np
import pandas as pd

SP = '/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731b'

def wrap(d):
    return (np.asarray(d) + 180.0) % 360.0 - 180.0

df = pd.read_csv(SP + '/log_corrected.csv')
t = df['elapsed_time'].values
truth = df['yaw_true_corrected_deg'].values
glitch = df['mocap_glitch'].values.astype(int)
state = df['tlm_state'].values

est = {
    'EKF1':     np.degrees(df['tlm_yaw_est_rad'].values),
    'EKF2':     np.degrees(df['tlm_ekf2_yaw_rad'].values),
    'gyro_int': np.degrees(df['tlm_yaw_gyro_int_rad'].values),
    'Madgwick': np.degrees(df['tlm_yaw_rad'].values),
}

# ---- windows ----
ground_w = (t >= 0.17) & (t <= 2.0)
t_flying0 = t[np.argmax(state == 4)]
t_land = t[np.argmax(state == 5)]
flight_w = (t >= t_flying0) & (t < t_land)

cmd = np.degrees(df['cmd_yaw_ref_rad'].values)
def rate_of(sig_deg):
    u = np.unwrap(np.radians(sig_deg))
    r = np.full_like(u, np.nan)
    dt_ = np.diff(t)
    ok = dt_ > 1e-6
    r[1:][ok] = np.diff(u)[ok] / dt_[ok] * 180/np.pi
    r = pd.Series(r).interpolate(limit_direction='both').rolling(9, center=True, min_periods=1).mean().values
    return r
cmd_rate = rate_of(cmd)
man_raw = np.abs(cmd_rate) > 2.0
man_raw &= flight_w & (t < 52.0)
man = man_raw.copy()
for i in np.where(man_raw)[0]:
    man |= (t >= t[i]) & (t <= t[i] + 2.0)
man &= flight_w
turn_w = man
hover_w = flight_w & ~man
truth_rate = rate_of(truth)

res = {}
errs_adj = {}
errs_raw = {}
for name, e in est.items():
    err = wrap(e - truth)
    off = err[ground_w].mean()
    err_adj = wrap(err - off)
    errs_raw[name] = err
    errs_adj[name] = err_adj
    A = np.polyfit(t[flight_w], err_adj[flight_w], 1)
    res[name] = dict(
        ground_offset_deg=round(float(off), 3),
        raw=dict(rms=float(np.sqrt(np.mean(err[flight_w]**2))),
                 mean=float(err[flight_w].mean()),
                 maxabs=float(np.abs(err[flight_w]).max())),
        adj=dict(rms=float(np.sqrt(np.mean(err_adj[flight_w]**2))),
                 mean=float(err_adj[flight_w].mean()),
                 maxabs=float(np.abs(err_adj[flight_w]).max())),
        drift_deg_per_min=float(A[0]*60.0),
        adj_turn=dict(rms=float(np.sqrt(np.mean(err_adj[turn_w]**2))),
                      maxabs=float(np.abs(err_adj[turn_w]).max())),
        adj_hover=dict(rms=float(np.sqrt(np.mean(err_adj[hover_w]**2))),
                       maxabs=float(np.abs(err_adj[hover_w]).max())),
    )

res['_windows'] = dict(
    t_flying=float(t_flying0), t_landing=float(t_land),
    n_flight=int(flight_w.sum()), n_turn=int(turn_w.sum()),
    n_hover=int(hover_w.sum()),
    turn_spans=[])
# contiguous turn spans
d = np.diff(turn_w.astype(int))
starts = np.where(d == 1)[0] + 1
ends = np.where(d == -1)[0] + 1
if turn_w[0]:
    starts = np.r_[0, starts]
if turn_w[-1]:
    ends = np.r_[ends, len(turn_w)-1]
res['_windows']['turn_spans'] = [[round(float(t[a]), 2), round(float(t[b-1]), 2)]
                                 for a, b in zip(starts, ends)]

# ---- EKF1 "RMS 80.4°" の分解 (汚染mocap_yaw_true列 vs 正解truth) ----
mc_orig = df['mocap_yaw_true_deg'].values  # PC側ロガーの汚染列
dcheck = wrap(mc_orig - truth)
cls_clean = np.abs(dcheck) < 20
cls_flip = np.abs(np.abs(dcheck) - 180.0) < 20
cls_glitch = ~cls_clean & ~cls_flip
e_cont = wrap(est['EKF1'] - mc_orig)  # 一次調査でRMS80.4°を出した誤差
def rmsw(e, w):
    return float(np.sqrt(np.mean(e[w]**2))) if w.sum() else None
res['ekf1_rms80_decomposition'] = dict(
    vs_contaminated_flight_rms=rmsw(e_cont, flight_w),
    vs_corrected_flight_rms=rmsw(errs_raw['EKF1'], flight_w),
    row_classes_flight=dict(
        clean=int((cls_clean & flight_w).sum()),
        flip180=int((cls_flip & flight_w).sum()),
        glitch95=int((cls_glitch & flight_w).sum())),
    rms_by_class=dict(
        clean=rmsw(e_cont, cls_clean & flight_w),
        flip180=rmsw(e_cont, cls_flip & flight_w),
        glitch95=rmsw(e_cont, cls_glitch & flight_w)),
    # RMS^2寄与 (加重)
    var_contrib_pct=dict(
        clean=float(100*np.sum(e_cont[cls_clean & flight_w]**2) / np.sum(e_cont[flight_w]**2)),
        flip180=float(100*np.sum(e_cont[cls_flip & flight_w]**2) / np.sum(e_cont[flight_w]**2)),
        glitch95=float(100*np.sum(e_cont[cls_glitch & flight_w]**2) / np.sum(e_cont[flight_w]**2))),
)

# ---- EKF1実誤差(13°級)の分解: 初期オフセット/回頭遅れ/磁気引き込み ----
bmx = df['tlm_bm_x_ut'].values
bmy = df['tlm_bm_y_ut'].values
bmx2 = df['tlm_ekf2_bm_x_ut'].values
bmy2 = df['tlm_ekf2_bm_y_ut'].values
for name, (bx, by) in [('EKF1', (bmx, bmy)), ('EKF2', (bmx2, bmy2))]:
    y = errs_adj[name][flight_w]
    X = np.column_stack([bx[flight_w], by[flight_w], np.ones(flight_w.sum())])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    yhat = X @ coef
    r2 = 1 - np.sum((y-yhat)**2)/np.sum((y-y.mean())**2)
    sens = float(np.hypot(coef[0], coef[1]))
    res[name]['bm_regression'] = dict(
        coef_x=float(coef[0]), coef_y=float(coef[1]), intercept=float(coef[2]),
        R2=float(r2), sensitivity_deg_per_ut=sens,
        implied_B0h_ut=float((180/np.pi)/sens) if sens > 0 else None,
        bm_final=[float(bx[flight_w][-1]), float(by[flight_w][-1])],
        bm_final_norm=float(np.hypot(bx[flight_w][-1], by[flight_w][-1])),
        bm_maxnorm=float(np.hypot(bx[flight_w], by[flight_w]).max()))

# 回頭遅れ: 2つの大ランプでの実効遅れ
ramp1 = (t >= 23.0) & (t <= 26.5)    # +88.4°
ramp2 = (t >= 36.8) & (t <= 40.2)    # -125.4°
pre1 = (t >= 20.0) & (t <= 22.9)
pre2 = (t >= 34.5) & (t <= 36.7)
for name in ['EKF1', 'EKF2']:
    e = errs_adj[name]
    lag = {}
    for lab, rw, pw in [('turn_pos88', ramp1, pre1), ('turn_neg125', ramp2, pre2)]:
        base = e[pw].mean()
        dd_ = (e[rw] - base) / np.where(np.abs(truth_rate[rw]) > 15, truth_rate[rw], np.nan)
        lag[lab] = dict(peak_delta_deg=float((e[rw]-base)[np.nanargmax(np.abs(e[rw]-base))]),
                        eff_lag_s_median=float(np.nanmedian(np.abs(dd_))))
    res[name]['turn_lag'] = lag

# 5秒毎平均 (磁気引き込みの時間構造)
for name in ['EKF1', 'EKF2']:
    means = {}
    for a in range(5, 55, 5):
        w = (t >= a) & (t < a+5) & flight_w
        if w.sum():
            means[f'{a}-{a+5}'] = round(float(errs_adj[name][w].mean()), 1)
    res[name]['5s_means'] = means

# ---- b_m 対ヘディング軌跡 + 直接残差測定 c_body ----
# 直接測定: mag_lev(t) - R(beta)*B0h(frame0),  beta = truth - truth(frame0)
zx = df['tlm_mag_lev_x_ut'].values
zy = df['tlm_mag_lev_y_ut'].values
b0hx, b0hy = zx[0], zy[0]
beta = np.radians(truth - truth[0])
px = np.cos(beta)*b0hx - np.sin(beta)*b0hy
py = np.sin(beta)*b0hx + np.cos(beta)*b0hy
rx = zx - px
ry = zy - py
w = flight_w & (glitch == 0)
res['ff_residual_direct'] = dict(
    B0h_frame0=[float(b0hx), float(b0hy)],
    c_body_mean=[float(rx[w].mean()), float(ry[w].mean())],
    c_body_norm=float(np.hypot(rx[w].mean(), ry[w].mean())),
    c_body_std=[float(rx[w].std()), float(ry[w].std())],
    # ヘディング依存性: 残差をヘディングbinで見る (真のハードアイアンなら一定のはず)
    by_heading={})
hb = np.floor((truth[w] + 180) / 45).astype(int)
for b in sorted(set(hb)):
    m = hb == b
    if m.sum() >= 20:
        res['ff_residual_direct']['by_heading'][f'{-180+45*b}..{-135+45*b}'] = [
            round(float(rx[w][m].mean()), 2), round(float(ry[w][m].mean()), 2), int(m.sum())]

# b_m成長率
tf = t[flight_w]
nrm1 = np.hypot(bmx, bmy)[flight_w]
nrm2 = np.hypot(bmx2, bmy2)[flight_w]
res['bm_growth'] = dict(
    ekf1_rate_ut_s=float(np.polyfit(tf[tf < 20], nrm1[tf < 20], 1)[0]),
    ekf2_rate_ut_s=float(np.polyfit(tf[tf < 20], nrm2[tf < 20], 1)[0]),
    ekf1_final=[float(bmx[-1]), float(bmy[-1])],
    ekf2_final=[float(bmx2[-1]), float(bmy2[-1])],
    ekf1_final_norm=float(np.hypot(bmx[-1], bmy[-1])),
    ekf2_final_norm=float(np.hypot(bmx2[-1], bmy2[-1])))

# ---- log_fixed.csv 生成 (mocap列を正解値へ置換) + 送信系クリーン列 ----
dfx = df.copy()
dfx['mocap_yaw_true_deg'] = truth
dfx['yaw_obs_sent_clean_rad'] = np.radians(wrap(truth - 90.434))
dfx.to_csv(SP + '/log_fixed.csv', index=False)

with open(SP + '/yaw_eval_stats.json', 'w') as f:
    json.dump(res, f, indent=1, ensure_ascii=False)
print(json.dumps(res, indent=1, ensure_ascii=False))
