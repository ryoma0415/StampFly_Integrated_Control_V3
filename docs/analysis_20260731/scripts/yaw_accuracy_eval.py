#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""20260731 flight: yaw accuracy of EKF1/EKF2/gyro/Madgwick vs corrected mocap truth."""
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

BASE = '/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly_MoCap_System_v2-StampFly-Integrated-Control-V3'
# actual scratchpad path
BASE = '/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a0731'
PLOTS = BASE + '/plots'

def wrap(d):
    return (np.asarray(d) + 180.0) % 360.0 - 180.0

df = pd.read_csv(BASE + '/log_corrected.csv')
df = df.iloc[1:].reset_index(drop=True)  # drop pre-arm stale first row
t = df['elapsed_time'].values
truth = df['yaw_true_corrected_deg'].values

est = {
    'EKF1':     np.degrees(df['tlm_yaw_est_rad'].values),
    'EKF2':     np.degrees(df['tlm_ekf2_yaw_rad'].values),
    'gyro_int': np.degrees(df['tlm_yaw_gyro_int_rad'].values),
    'Madgwick': np.degrees(df['tlm_yaw_rad'].values),
}

# ---- windows ----
state = df['tlm_state'].values
ground_w = (t >= 0.15) & (t <= 1.5)
t_takeoff = t[np.argmax(state == 3)]          # takeoff state begins
t_flying0 = t[np.argmax(state == 4)]          # steady flight begins
t_land = t[np.argmax(state == 5)]             # landing begins
flight_w = (t >= t_flying0) & (t < t_land)    # in-air, pre-landing
air_w = (t >= t_takeoff) & (t < t_land)

# turn/maneuver windows from cmd yaw rate
cmd = np.degrees(df['cmd_yaw_ref_rad'].values)
def rate_of(sig_deg):
    """deg/s, robust to duplicate timestamps."""
    u = np.unwrap(np.radians(sig_deg))
    r = np.full_like(u, np.nan)
    dt_ = np.diff(t)
    ok = dt_ > 1e-6
    r[1:][ok] = np.diff(u)[ok] / dt_[ok] * 180/np.pi
    r = pd.Series(r).interpolate(limit_direction='both').rolling(9, center=True, min_periods=1).mean().values
    return r
cmd_rate = rate_of(cmd)
man_raw = np.abs(cmd_rate) > 2.0
man_raw &= flight_w & (t < 52.0)  # exclude landing-ramp artifact
# dilate by settle margin: 0 s before, 2.0 s after
man = man_raw.copy()
idx = np.where(man_raw)[0]
for i in idx:
    man |= (t >= t[i]) & (t <= t[i] + 2.0)
man &= flight_w
turn_w = man
hover_w = flight_w & ~man
# separate ramps
ramp_up = (t >= 38.9) & (t <= 42.0)
ramp_dn = (t >= 44.4) & (t <= 46.6)

# truth yaw rate
truth_rate = rate_of(truth)

res = {}
errs = {}
errs_adj = {}
for name, e in est.items():
    err = wrap(e - truth)
    off = err[ground_w].mean()
    err_adj = wrap(err - off)
    errs[name] = err
    errs_adj[name] = err_adj
    # drift: linear fit over flight window (offset-removed error)
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

# turn transient: error change relative to pre-turn baseline (t=36-38.9)
pre_turn = (t >= 36.0) & (t <= 38.9)
for name in est:
    base = errs_adj[name][pre_turn].mean()
    d_up = errs_adj[name][ramp_up] - base
    d_dn = errs_adj[name][ramp_dn] - base
    res[name]['turn_transient'] = dict(
        pre_turn_baseline=float(base),
        rampup_peak_delta=float(d_up[np.argmax(np.abs(d_up))]),
        rampdn_peak_delta=float(d_dn[np.argmax(np.abs(d_dn))]),
    )

# effective lag estimate for EKF1 during ramps: delta_err / yaw_rate
for name in ['EKF1', 'EKF2']:
    e1 = errs_adj[name]
    base = e1[pre_turn].mean()
    lag_up = (e1[ramp_up] - base) / np.where(np.abs(truth_rate[ramp_up]) > 5,
                                             truth_rate[ramp_up], np.nan)
    res[name]['eff_lag_s_rampup_median'] = float(np.nanmedian(np.abs(lag_up)))

# peak truth yaw rates
res['_yaw_rates'] = dict(
    peak_up=float(np.nanmax(truth_rate[ramp_up])),
    peak_dn=float(np.nanmin(truth_rate[ramp_dn])),
)

# ---- phase-by-phase EKF error around the turn (degeneracy-break analysis) ----
phases = {
    'pre_turn_30_38.9': (t >= 30.0) & (t < 38.9),
    'ramp_up_38.9_42': ramp_up,
    'hold_42_44.4': (t >= 42.0) & (t < 44.4),
    'ramp_dn_44.4_46.6': ramp_dn,
    'post_turn_48_52.9': (t >= 48.0) & (t < t_land),
}
for name in ['EKF1', 'EKF2', 'gyro_int', 'Madgwick']:
    res[name]['phase_mean_err'] = {k: round(float(errs_adj[name][w].mean()), 2)
                                   for k, w in phases.items()}
    res[name]['phase_minmax'] = {k: [round(float(errs_adj[name][w].min()), 2),
                                     round(float(errs_adj[name][w].max()), 2)]
                                 for k, w in phases.items()}
# b_m norm per phase (EKF1)
res['bm_phase'] = {k: dict(
    ekf1=[round(float(bmx_ := df['tlm_bm_x_ut'].values[w].mean()), 2),
          round(float(bmy_ := df['tlm_bm_y_ut'].values[w].mean()), 2),
          round(float(np.hypot(bmx_, bmy_)), 2)]) for k, w in phases.items()}
# EKF1 vs EKF2 similarity
d12 = wrap(est['EKF1'] - est['EKF2'])
res['ekf1_vs_ekf2'] = dict(rms_diff=float(np.sqrt(np.mean(d12[flight_w]**2))),
                           corr_err=float(np.corrcoef(errs_adj['EKF1'][flight_w],
                                                      errs_adj['EKF2'][flight_w])[0, 1]))

# ---- b_m regression (EKF1) ----
bmx = df['tlm_bm_x_ut'].values
bmy = df['tlm_bm_y_ut'].values
for name, (bx, by) in [('EKF1', (bmx, bmy)),
                       ('EKF2', (df['tlm_ekf2_bm_x_ut'].values, df['tlm_ekf2_bm_y_ut'].values))]:
    y = errs_adj[name][flight_w]
    X = np.column_stack([bx[flight_w], by[flight_w], np.ones(flight_w.sum())])
    coef, *_ = np.linalg.lstsq(X, y, rcond=None)
    yhat = X @ coef
    ss_res = np.sum((y - yhat)**2)
    ss_tot = np.sum((y - y.mean())**2)
    r2 = 1 - ss_res/ss_tot
    sens = float(np.hypot(coef[0], coef[1]))  # deg per uT
    b0h_impl = (180/np.pi)/sens if sens > 0 else np.inf
    res[name]['bm_regression'] = dict(
        coef_x=float(coef[0]), coef_y=float(coef[1]), intercept=float(coef[2]),
        R2=float(r2), sensitivity_deg_per_ut=sens, implied_B0h_ut=float(b0h_impl),
        bm_final=[float(bx[flight_w][-1]), float(by[flight_w][-1])],
        bm_maxnorm=float(np.hypot(bx[flight_w], by[flight_w]).max()),
    )
# hover-only regression for EKF1 (exclude turn transient which is EMA lag, not bm)
y = errs_adj['EKF1'][hover_w]
X = np.column_stack([bmx[hover_w], bmy[hover_w], np.ones(hover_w.sum())])
coef, *_ = np.linalg.lstsq(X, y, rcond=None)
yhat = X @ coef
r2h = 1 - np.sum((y-yhat)**2)/np.sum((y-y.mean())**2)
sens = float(np.hypot(coef[0], coef[1]))
res['EKF1']['bm_regression_hover'] = dict(
    R2=float(r2h), sensitivity_deg_per_ut=sens,
    implied_B0h_ut=float((180/np.pi)/sens))

# ---- yaw control tracking ----
ffs = df['tlm_ff_status'].values.astype(int)
yawctrl = ((ffs >> 5) & 1).astype(bool) & flight_w
e_ctrl = wrap(cmd - est['EKF1'])          # what the controller sees
e_true = wrap(cmd - truth)                # what actually happened
err1_raw = errs['EKF1']                   # ekf1 - truth (raw frame = cmd frame)
# identity: e_true = wrap(e_ctrl + err1_raw)  (cmd-truth = (cmd-ekf1)+(ekf1-truth))
chk = wrap(e_true - (e_ctrl + err1_raw))
res['yaw_ctrl'] = dict(
    n=int(yawctrl.sum()),
    identity_check_max=float(np.abs(chk).max()),
    rms_ctrl_err=float(np.sqrt(np.mean(e_ctrl[yawctrl]**2))),
    rms_true_err=float(np.sqrt(np.mean(e_true[yawctrl]**2))),
    rms_est_err_raw=float(np.sqrt(np.mean(err1_raw[yawctrl]**2))),
    corr_true_vs_est=float(np.corrcoef(e_true[yawctrl], err1_raw[yawctrl])[0, 1]),
    hover_corr_true_vs_est=float(np.corrcoef(e_true[hover_w], err1_raw[hover_w])[0, 1]),
    hover_rms_ctrl=float(np.sqrt(np.mean(e_ctrl[hover_w]**2))),
    hover_rms_true=float(np.sqrt(np.mean(e_true[hover_w]**2))),
    hover_rms_est=float(np.sqrt(np.mean(err1_raw[hover_w]**2))),
    hover_mean_ctrl=float(e_ctrl[hover_w].mean()),
    hover_mean_true=float(e_true[hover_w].mean()),
    turn_rms_ctrl=float(np.sqrt(np.mean(e_ctrl[turn_w]**2))),
    turn_rms_true=float(np.sqrt(np.mean(e_true[turn_w]**2))),
    turn_max_ctrl=float(np.abs(e_ctrl[turn_w]).max()),
    turn_max_true=float(np.abs(e_true[turn_w]).max()),
)

# ---- position control ----
ex = df['error_x'].values
ey = df['error_y'].values
# verify sign convention: error = pos - target?
chk_ex = np.nanmax(np.abs(ex - (df['pos_x'].values - df['target_x'].values)))
chk_ex2 = np.nanmax(np.abs(ex - (df['target_x'].values - df['pos_x'].values)))
res['pos_err_convention'] = dict(pos_minus_target=float(chk_ex),
                                 target_minus_pos=float(chk_ex2))
r_err = np.hypot(ex, ey)
ez = df['pos_z'].values - df['target_z'].values

# steady hover for position: flight, alt settled (t>=t_flying0+2s), full flight too
hover_pos_w = hover_w & (t >= t_flying0 + 2.0)
seg = {}
for label, w in [('flight_all', flight_w), ('steady_hover', hover_pos_w),
                 ('turn', turn_w)]:
    seg[label] = dict(
        n=int(w.sum()),
        rms_x_mm=float(np.sqrt(np.mean(ex[w]**2))*1000),
        rms_y_mm=float(np.sqrt(np.mean(ey[w]**2))*1000),
        rms_r_mm=float(np.sqrt(np.mean(r_err[w]**2))*1000),
        p95_r_mm=float(np.percentile(r_err[w], 95)*1000),
        max_r_mm=float(r_err[w].max()*1000),
        z_mean_m=float(df['pos_z'].values[w].mean()),
        z_err_rms_mm=float(np.sqrt(np.mean(ez[w]**2))*1000),
        z_err_mean_mm=float(ez[w].mean()*1000),
        z_std_mm=float(df['pos_z'].values[w].std()*1000),
    )
res['position'] = seg
# does horizontal error correlate with yaw estimation error? (thrust misdirection)
w = hover_pos_w
res['position']['corr_r_vs_absyawerr_hover'] = float(
    np.corrcoef(r_err[w], np.abs(err1_raw[w]))[0, 1])
# time-split hover halves
tm = 0.5*(t[w].min()+t[w].max())
for lab, ww in [('hover_1st_half', w & (t < tm)), ('hover_2nd_half', w & (t >= tm))]:
    res['position'][lab] = dict(
        rms_r_mm=float(np.sqrt(np.mean(r_err[ww]**2))*1000),
        p95_r_mm=float(np.percentile(r_err[ww], 95)*1000))
# altitude oscillation character
zz = ez[w] - ez[w].mean()
zc = np.where(np.diff(np.sign(zz)) != 0)[0]
res['position']['z_osc'] = dict(
    p2p_mm=float((ez[w].max()-ez[w].min())*1000),
    zero_cross_per_s=float(len(zc)/(t[w].max()-t[w].min())))
res['_windows'] = dict(t_takeoff=float(t_takeoff), t_flying=float(t_flying0),
                       t_landing=float(t_land),
                       turn_span=[float(t[turn_w].min()), float(t[turn_w].max())],
                       n_flight=int(flight_w.sum()), n_turn=int(turn_w.sum()),
                       n_hover=int(hover_w.sum()))

with open(BASE + '/yaw_accuracy_stats.json', 'w') as f:
    json.dump(res, f, indent=1, ensure_ascii=False)
print(json.dumps(res, indent=1, ensure_ascii=False))

# ================= plots =================
import matplotlib
matplotlib.rcParams['font.size'] = 9
C = {'EKF1': 'tab:red', 'EKF2': 'tab:purple', 'gyro_int': 'tab:blue',
     'Madgwick': 'tab:green'}

def shade(ax):
    ax.axvspan(38.9, 42.0, color='orange', alpha=0.12, lw=0)
    ax.axvspan(44.4, 46.6, color='orange', alpha=0.12, lw=0)
    ax.axvline(t_land, color='gray', ls=':', lw=0.8)

# Fig 1: error time series
fig, axs = plt.subplots(3, 1, figsize=(11, 9), sharex=True,
                        gridspec_kw={'height_ratios': [2, 2, 1.3]})
ax = axs[0]
for name in est:
    ax.plot(t, errs_adj[name], color=C[name], lw=1.0, label=name)
ax.axhline(0, color='k', lw=0.5)
shade(ax)
ax.set_ylabel('yaw err (offset-removed) [deg]')
ax.legend(ncol=4, loc='upper left')
ax.set_title('20260731_111906  yaw error vs corrected MoCap truth '
             '(ground-window offset removed; orange = yaw maneuver)')
ax = axs[1]
ax.plot(t, bmx, 'r-', lw=1, label='EKF1 b_m x')
ax.plot(t, bmy, 'r--', lw=1, label='EKF1 b_m y')
ax.plot(t, df['tlm_ekf2_bm_x_ut'], 'm-', lw=0.8, label='EKF2 b_m x')
ax.plot(t, df['tlm_ekf2_bm_y_ut'], 'm--', lw=0.8, label='EKF2 b_m y')
ax2 = ax.twinx()
ax2.plot(t, errs_adj['EKF1'], color='0.4', lw=0.7, alpha=0.7)
ax2.set_ylabel('EKF1 err [deg]', color='0.4')
shade(ax)
ax.set_ylabel('b_m [uT]')
ax.legend(ncol=4, loc='lower left')
ax = axs[2]
ax.plot(t, truth_rate, 'k-', lw=0.7, label='truth yaw rate')
ax.plot(t, df['tlm_current_a'], 'c-', lw=0.8, label='current [A]')
shade(ax)
ax.set_ylabel('deg/s | A')
ax.set_xlabel('t [s]')
ax.legend(ncol=2)
fig.tight_layout()
fig.savefig(PLOTS + '/01_yaw_error_timeseries.png', dpi=140)
plt.close(fig)

# Fig 2: turn zoom
fig, axs = plt.subplots(2, 1, figsize=(11, 7), sharex=True)
ax = axs[0]
ax.plot(t, cmd, 'k--', lw=1.2, label='cmd_yaw_ref')
ax.plot(t, truth, color='0.2', lw=1.5, label='truth (corrected)')
for name in ['EKF1', 'EKF2', 'gyro_int']:
    ax.plot(t, wrap(est[name] - res[name]['ground_offset_deg']),
            color=C[name], lw=1, label=name + ' (offset-rm)')
ax.set_xlim(36, 51)
ax.set_ylabel('yaw [deg]')
ax.legend(ncol=3)
ax.set_title('+90deg yaw maneuver (t=39-48s): tracking & estimator lag')
ax = axs[1]
for name in est:
    ax.plot(t, errs_adj[name], color=C[name], lw=1, label=name)
ax.axhline(0, color='k', lw=0.5)
ax.plot(t, wrap(cmd - est['EKF1']), color='0.5', ls=':', lw=1.2,
        label='ctrl err (cmd-EKF1)')
ax.set_xlim(36, 51)
ax.set_ylim(-35, 25)
ax.set_ylabel('err [deg]')
ax.set_xlabel('t [s]')
ax.legend(ncol=5, fontsize=8)
fig.tight_layout()
fig.savefig(PLOTS + '/02_turn_zoom.png', dpi=140)
plt.close(fig)

# Fig 3: b_m regression
fig, axs = plt.subplots(1, 3, figsize=(13, 4))
reg = res['EKF1']['bm_regression']
yhat_all = reg['coef_x']*bmx + reg['coef_y']*bmy + reg['intercept']
ax = axs[0]
ax.plot(t[flight_w], errs_adj['EKF1'][flight_w], 'r-', lw=1, label='EKF1 err')
ax.plot(t[flight_w], yhat_all[flight_w], 'k--', lw=1,
        label=f"b_m fit (R2={reg['R2']:.3f})")
ax.set_xlabel('t [s]'); ax.set_ylabel('deg'); ax.legend()
ax.set_title('EKF1 err vs b_m linear model')
ax = axs[1]
sc = ax.scatter(yhat_all[flight_w], errs_adj['EKF1'][flight_w], s=3,
                c=t[flight_w], cmap='viridis')
lims = [-30, 15]
ax.plot(lims, lims, 'k--', lw=0.7)
ax.set_xlabel('predicted from b_m [deg]'); ax.set_ylabel('actual err [deg]')
plt.colorbar(sc, ax=ax, label='t [s]')
ax.set_title(f"sens {reg['sensitivity_deg_per_ut']:.2f} deg/uT, "
             f"|B0h| impl {reg['implied_B0h_ut']:.1f} uT")
ax = axs[2]
ax.plot(bmx[flight_w], bmy[flight_w], '-', c='0.6', lw=0.7)
sc = ax.scatter(bmx[flight_w], bmy[flight_w], s=4, c=t[flight_w], cmap='viridis')
ax.scatter([bmx[flight_w][-1]], [bmy[flight_w][-1]], marker='x', c='r', s=60)
ax.set_xlabel('b_m x [uT]'); ax.set_ylabel('b_m y [uT]')
ax.set_title('EKF1 b_m trajectory (x=final)')
ax.axis('equal')
fig.tight_layout()
fig.savefig(PLOTS + '/03_bm_regression.png', dpi=140)
plt.close(fig)

# Fig 4: yaw tracking / control structure
fig, axs = plt.subplots(1, 2, figsize=(11, 4.2))
ax = axs[0]
ax.plot(t[flight_w], e_ctrl[flight_w], color='0.5', lw=0.8,
        label='ctrl err = cmd - EKF1')
ax.plot(t[flight_w], e_true[flight_w], 'b-', lw=1, label='true err = cmd - truth')
ax.plot(t[flight_w], err1_raw[flight_w], 'r--', lw=0.8,
        label='EKF1 est err (raw)')
ax.axhline(0, color='k', lw=0.5)
ax.set_xlabel('t [s]'); ax.set_ylabel('deg'); ax.legend(fontsize=8)
ax.set_title('true tracking err = ctrl err + EKF1 est err')
ax = axs[1]
ax.scatter(err1_raw[hover_w], e_true[hover_w], s=3, c=t[hover_w], cmap='viridis')
lims = [-30, 10]
ax.plot(lims, lims, 'k--', lw=0.7)
ax.set_xlabel('EKF1 est err [deg]'); ax.set_ylabel('true tracking err [deg]')
ax.set_title(f"hover only: corr = {res['yaw_ctrl']['hover_corr_true_vs_est']:.3f}")
fig.tight_layout()
fig.savefig(PLOTS + '/04_yaw_tracking.png', dpi=140)
plt.close(fig)

# Fig 5: position
fig, axs = plt.subplots(1, 3, figsize=(13, 4.2))
ax = axs[0]
w = hover_pos_w
ax.plot(ex[w]*1000, ey[w]*1000, '.', ms=2, c='tab:blue', label='steady hover')
ax.plot(ex[turn_w]*1000, ey[turn_w]*1000, '.', ms=2, c='tab:orange', label='turn')
th = np.linspace(0, 2*np.pi, 100)
for rr, ls in [(seg['steady_hover']['rms_r_mm'], '-'),
               (seg['steady_hover']['p95_r_mm'], '--')]:
    ax.plot(rr*np.cos(th), rr*np.sin(th), 'k'+ls, lw=0.8)
ax.set_xlabel('err x [mm]'); ax.set_ylabel('err y [mm]')
ax.axis('equal'); ax.legend(fontsize=8)
ax.set_title(f"XY err: RMS {seg['steady_hover']['rms_r_mm']:.0f} mm, "
             f"P95 {seg['steady_hover']['p95_r_mm']:.0f} mm (hover)")
ax = axs[1]
ax.plot(t, r_err*1000, 'b-', lw=0.8)
shade(ax)
ax.set_xlabel('t [s]'); ax.set_ylabel('radial err [mm]')
ax.set_title('horizontal error vs time')
ax = axs[2]
ax.plot(t, df['pos_z'], 'g-', lw=1, label='pos_z')
ax.axhline(0.3, color='k', ls='--', lw=0.8, label='target 0.30 m')
shade(ax)
ax.set_xlabel('t [s]'); ax.set_ylabel('z [m]')
ax.legend(fontsize=8)
ax.set_title(f"z: mean {seg['steady_hover']['z_mean_m']:.3f} m, "
             f"err RMS {seg['steady_hover']['z_err_rms_mm']:.0f} mm")
fig.tight_layout()
fig.savefig(PLOTS + '/05_position.png', dpi=140)
plt.close(fig)
print('plots done')
