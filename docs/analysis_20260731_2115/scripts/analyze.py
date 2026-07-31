#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""21:15/21:13 flights (2026-07-31): first successful yaw-obs fusion.
Tasks 1-5 + supporting stats for 6/7. Outputs stats JSON + plots."""
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle

plt.rcParams["font.family"] = ["Hiragino Sans", "Arial Unicode MS", "sans-serif"]
plt.rcParams["axes.unicode_minus"] = False

BASE = ("/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/"
        "StampFly_MoCap_System_v2/StampFly_Integrated_Control_V3")
OUT = ("/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-"
       "Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/"
       "ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a2115")
PLOTS = OUT + "/plots"
LOGS = {"main2115": BASE + "/logs/flight_logs/20260731_211535_position.csv",
        "short2113": BASE + "/logs/flight_logs/20260731_211304_position.csv"}


def wrapd(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def load(name):
    path = LOGS[name]
    df = pd.read_csv(path)
    meta = json.load(open(path.replace(".csv", ".meta.json")))
    d = {}
    d["df"] = df
    d["meta"] = meta
    d["t"] = df["elapsed_time"].values
    d["n"] = len(df)
    st = df["tlm_state"].values
    d["st"] = st
    d["i_arm"] = int(np.argmax(st >= 3))
    d["t_arm"] = float(d["t"][d["i_arm"]])
    d["t_flying"] = float(d["t"][np.argmax(st == 4)])
    d["t_landing"] = float(d["t"][np.argmax(st == 5)])
    d["fly"] = (d["t"] >= d["t_flying"]) & (d["t"] < d["t_landing"])
    tof = df["tlm_altitude_tof_m"].values
    lift = next((i for i in range(d["i_arm"], d["n"] - 10)
                 if np.all(tof[i:i + 10] > 0.08)), None)
    d["t_lift"] = float(d["t"][lift]) if lift else None
    d["anchor_deg"] = float(np.degrees(meta["yaw_ref"]["anchor"]["anchor_rad"]))
    d["yt"] = df["mocap_yaw_true_deg"].values          # PC frame truth
    d["truth_onb"] = wrapd(d["yt"] + d["anchor_deg"])  # onboard frame truth
    d["e1"] = np.degrees(df["tlm_yaw_est_rad"].values)
    d["e2"] = np.degrees(df["tlm_ekf2_yaw_rad"].values)
    d["gy"] = np.degrees(np.unwrap(df["tlm_yaw_gyro_int_rad"].values))
    d["md"] = np.degrees(np.unwrap(df["tlm_yaw_rad"].values))
    d["sent"] = np.degrees(df["yaw_ref_sent_rad"].values)
    d["inn"] = np.degrees(df["tlm_ekf2_yaw_innov_rad"].values)
    d["s2"] = df["tlm_ekf2_status"].astype(int).values
    d["g2"] = df["tlm_ekf2_gate"].astype(int).values
    d["ffg"] = df["tlm_ffg"].astype(int).values
    return d


R = {}

# ================================================================ per flight
for name in ("main2115", "short2113"):
    d = load(name)
    df, t, n = d["df"], d["t"], d["n"]
    res = {}

    # ---------- Task 1a: truth quality vs gyro integration ----------
    yt_u = np.degrees(np.unwrap(np.radians(d["yt"])))
    D = yt_u - d["gy"]
    m_on = np.arange(n) >= d["i_arm"]
    # robust linear detrend (gyro drift) -> residual = shape difference
    A = np.polyfit(t[m_on], D[m_on] - D[m_on][0], 1)
    det = (D - D[m_on][0] - np.polyval(A, t))[m_on]
    # iterate with outlier rejection
    inl_all = np.ones(n, bool)
    for _ in range(3):
        w = m_on & inl_all
        A = np.polyfit(t[w], D[w] - D[m_on][0], 1)
        r_ = D - D[m_on][0] - np.polyval(A, t)
        inl_all = np.abs(r_ - np.median(r_[m_on])) < 25
    det = r_[m_on & inl_all] - np.median(r_[m_on])
    res["truth_vs_gyro"] = dict(
        drift_deg_per_min=float(A[0] * 60),
        detrended_rms_deg=float(np.sqrt(np.mean(det**2))),
        detrended_maxabs_deg=float(np.abs(det).max()),
        n_outlier_rows=int((m_on & ~inl_all).sum()))
    # short-window shape RMS (5s windows, per-window linear fit removed)
    sw = []
    for t0 in np.arange(t[d["i_arm"]], t[-1] - 5, 2.5):
        w = (t >= t0) & (t < t0 + 5)
        if w.sum() < 50:
            continue
        a_ = np.polyfit(t[w], D[w], 1)
        sw.append(np.sqrt(np.mean((D[w] - np.polyval(a_, t[w]))**2)))
    res["truth_vs_gyro"]["shortwin5s_shape_rms_med"] = float(np.median(sw))
    res["truth_vs_gyro"]["shortwin5s_shape_rms_p95"] = float(np.percentile(sw, 95))
    # frame-to-frame jump census (glitch check)
    jump = np.abs(wrapd(np.diff(d["yt"])))
    res["truth_vs_gyro"]["jump_p99_deg_per_row"] = float(np.percentile(jump, 99))
    res["truth_vs_gyro"]["jump_max_deg_per_row"] = float(jump.max())
    res["truth_vs_gyro"]["n_jump_gt20"] = int((jump > 20).sum())
    res["truth_vs_gyro"]["cont_filter_reject_rows"] = int(
        (df["mocap_flip"].astype(int).values & 2).astype(bool).sum())
    res["truth_vs_gyro"]["rb_error_mean_mm"] = float(df["rb_error"].mean() * 1000)
    res["truth_vs_gyro"]["rb_marker_count_hist"] = {
        int(k): int(v) for k, v in
        zip(*np.unique(df["rb_marker_count"].values, return_counts=True))}

    # ---------- Task 1b: anchor operation ----------
    ok = df["yaw_ref_valid"].values > 0
    imp_anchor = wrapd(d["sent"] - d["yt"])           # implied anchor
    res["anchor"] = dict(meta_anchor_deg=round(d["anchor_deg"], 3),
                         meta_yaw_est_deg=round(np.degrees(
                             d["meta"]["yaw_ref"]["anchor"]["yaw_est_rad"]), 3))
    # plateau detection: median implied anchor pre-arm vs post-arm
    pre = ok & (np.arange(n) < d["i_arm"])
    post = ok & (t > d["t_arm"] + 1.5)
    res["anchor"]["implied_pre_arm_med"] = float(np.median(imp_anchor[pre])) if pre.any() else None
    res["anchor"]["implied_post_arm_med"] = float(np.median(imp_anchor[post]))
    res["anchor"]["post_arm_sent_eq_true_plus_anchor_rms"] = float(
        np.sqrt(np.mean(wrapd(d["sent"] - (d["yt"] + d["anchor_deg"]))[post]**2)))
    # latch transition: find where implied anchor settles to meta value
    close = np.abs(wrapd(imp_anchor - d["anchor_deg"])) < 0.5
    i_latch = int(np.argmax(close & (np.arange(n) >= d["i_arm"])))
    res["anchor"]["t_latch_settle"] = float(t[i_latch])
    res["anchor"]["latch_delay_after_arm_s"] = float(t[i_latch] - d["t_arm"])
    # step size at latch
    res["anchor"]["anchor_step_at_arm_deg"] = float(
        wrapd(d["anchor_deg"] - res["anchor"]["implied_pre_arm_med"])) if pre.any() else None
    # sent - yaw_est(EKF1) exactly at latch settle
    res["anchor"]["sent_minus_e1_after_latch"] = dict(
        mean=float(wrapd(d["sent"] - d["e1"])[(t >= t[i_latch]) & (t <= t[i_latch] + 1) & ok].mean()))
    # innov step around arm
    warm = (t >= d["t_arm"] - 0.5) & (t <= d["t_arm"] + 3)
    res["anchor"]["innov_maxabs_arm_window"] = float(np.abs(d["inn"][warm]).max())
    res["anchor"]["innov_maxabs_whole"] = float(np.abs(d["inn"]).max())
    it_max = int(np.argmax(np.abs(d["inn"])))
    res["anchor"]["t_innov_maxabs"] = float(t[it_max])

    # ---------- Task 2: fusion quality ----------
    chg = np.where(np.abs(np.diff(d["inn"])) > 1e-9)[0] + 1
    fused = (d["s2"] & 2) > 0
    fresh = (d["s2"] & 1) > 0
    res["fusion"] = dict(
        fused_frac_all=float(fused.mean()),
        fused_frac_preroll=float(fused[d["st"] == 2].mean()),
        fused_frac_ground_armed=float(fused[(d["st"] == 3)].mean()),
        fused_frac_flight=float(fused[d["fly"]].mean()),
        fused_at_takeoff=bool(fused[np.argmax(d["st"] == 4)]),
        fresh_frac=float(fresh.mean()),
        recapture_rows=int(((d["s2"] & 128) > 0).sum()),
        low_trust_rows=int(((d["s2"] & 64) > 0).sum()),
        obs_n=int(len(chg)),
        obs_rate_hz=float(len(chg) / (t[chg[-1]] - t[chg[0]])))
    io = d["inn"][chg]
    tc = t[chg]
    res["fusion"]["innov_obs"] = dict(
        rms=float(np.sqrt(np.mean(io**2))), mean=float(io.mean()),
        p50_abs=float(np.median(np.abs(io))), p95_abs=float(np.percentile(np.abs(io), 95)),
        p99_abs=float(np.percentile(np.abs(io), 99)), maxabs=float(np.abs(io).max()),
        frac_within_3deg=float((np.abs(io) < 3).mean()),
        gate30_margin_worst=float(1 - np.abs(io).max() / 30.0))
    # innov by phase
    flyc = d["fly"][chg]
    grdc = ~flyc
    res["fusion"]["innov_rms_ground"] = float(np.sqrt(np.mean(io[grdc]**2)))
    res["fusion"]["innov_rms_flight"] = float(np.sqrt(np.mean(io[flyc]**2)))

    # ---------- Task 3: flight anchor ----------
    fa = (d["s2"] & 4) > 0
    i_fa = int(np.argmax(fa))
    t_fa = float(t[i_fa])
    alt_est = df["tlm_altitude_est_m"].values
    alt_ref = df["tlm_alt_ref_m"].values
    cur = df["tlm_current_a"].values
    hold = np.abs(alt_est - alt_ref) < 0.1
    res["flight_anchor"] = dict(
        t_first=t_fa, frac=float(fa.mean()),
        dt_after_flying=float(t_fa - d["t_flying"]),
        dt_after_arm=float(t_fa - d["t_arm"]),
        current_at_anchor=float(cur[max(0, i_fa - 3):i_fa + 3].mean()))
    # alt-hold history before anchor: last reset
    resets = np.where(~hold[:i_fa])[0]
    res["flight_anchor"]["t_last_alt_hold_reset"] = float(t[resets[-1]]) if len(resets) else None
    # condition reconstruction: earliest t where flying>5s & hold2s
    res["flight_anchor"]["alt_hold_frac_prefa"] = float(hold[d["fly"] & (t < t_fa)].mean()) if (d["fly"] & (t < t_fa)).any() else None
    # z gate: ekf2 vs ekf1
    z2 = (d["g2"] & 8) > 0
    z1 = (d["ffg"] & 8) > 0
    pre_fa_fly = d["fly"] & (t < t_fa)
    post_fa = d["fly"] & (t >= t_fa)
    res["flight_anchor"]["z_reject"] = dict(
        ekf2_flight_all=float(z2[d["fly"]].mean()),
        ekf2_pre_anchor=float(z2[pre_fa_fly].mean()) if pre_fa_fly.any() else None,
        ekf2_post_anchor=float(z2[post_fa].mean()),
        ekf1_flight_all=float(z1[d["fly"]].mean()),
        ekf1_pre_anchor=float(z1[pre_fa_fly].mean()) if pre_fa_fly.any() else None,
        ekf1_post_anchor=float(z1[post_fa].mean()),
        ekf1_bm_frozen_frac=float(((d["ffg"] & 32) > 0).mean()))
    # mag z shift: ground B0z proxy vs flight
    magz = df["tlm_mag_cal_z_ut"].values
    gw = (t > 0.5) & (t < d["t_arm"] - 0.3)
    b0z_gnd = float(magz[gw].mean())
    res["flight_anchor"]["mag_cal_z"] = dict(
        ground_mean=b0z_gnd,
        flight_mean=float(magz[d["fly"]].mean()),
        shift_at_flight=float(magz[d["fly"]].mean() - b0z_gnd),
        pre_anchor_mean=float(magz[pre_fa_fly].mean()) if pre_fa_fly.any() else None)
    # mag update acceptance for EKF2 in flight (gate==0 or 1 => accepted-ish)
    res["flight_anchor"]["ekf2_gate_hist_flight"] = {
        int(k): int(v) for k, v in zip(*np.unique(d["g2"][d["fly"]], return_counts=True))}
    res["flight_anchor"]["ekf1_ffg_hist_flight"] = {
        int(k): int(v) for k, v in zip(*np.unique(d["ffg"][d["fly"]], return_counts=True))}
    d["t_fa"] = t_fa
    d["b0z_gnd"] = b0z_gnd

    # ---------- Task 4: b_m ----------
    bm2x = df["tlm_ekf2_bm_x_ut"].values
    bm2y = df["tlm_ekf2_bm_y_ut"].values
    bm1x = df["tlm_bm_x_ut"].values
    bm1y = df["tlm_bm_y_ut"].values
    res["bm"] = dict(
        ekf2_at_start=[float(bm2x[0]), float(bm2y[0])],
        ekf2_at_anchor_reset=[float(bm2x[i_fa]), float(bm2y[i_fa])],
        ekf1_at_start=[float(bm1x[0]), float(bm1y[0])],
        ekf1_norm_max=float(np.hypot(bm1x, bm1y).max()),
        ekf2_norm_max=float(np.hypot(bm2x, bm2y).max()))
    # post-anchor convergence & stability
    pa = post_fa
    if pa.sum() > 100:
        fx = float(np.median(bm2x[d["fly"] & (t > d["t_landing"] - 10)]))
        fy = float(np.median(bm2y[d["fly"] & (t > d["t_landing"] - 10)]))
        res["bm"]["ekf2_final_last10s_med"] = [fx, fy]
        res["bm"]["ekf2_std_post_anchor"] = [float(bm2x[pa].std()), float(bm2y[pa].std())]
        res["bm"]["ekf1_std_flight"] = [float(bm1x[d["fly"]].std()), float(bm1y[d["fly"]].std())]
        # convergence time to within 0.5uT of final after anchor
        dist = np.hypot(bm2x - fx, bm2y - fy)
        idx = np.where(pa)[0]
        conv = None
        for i in idx:
            if dist[i] < 0.5 and np.all(dist[i:min(i + 100, n)][pa[i:min(i + 100, n)]] < 1.0):
                conv = float(t[i] - t_fa)
                break
        res["bm"]["ekf2_conv_time_after_anchor_s"] = conv
    # heading dependence (degeneracy fingerprint): b_m ~ a*cos+b*sin+c
    hd = np.radians(d["truth_onb"])
    X = np.column_stack([np.cos(hd), np.sin(hd), np.ones(n)])
    for nm, bx, by, msk in (("ekf1", bm1x, bm1y, d["fly"]),
                            ("ekf2", bm2x, bm2y, post_fa if post_fa.sum() > 100 else d["fly"])):
        r2s, amps = [], []
        for b_ in (bx, by):
            c_, *_ = np.linalg.lstsq(X[msk], b_[msk], rcond=None)
            pred = X[msk] @ c_
            ss = 1 - np.sum((b_[msk] - pred)**2) / max(np.sum((b_[msk] - b_[msk].mean())**2), 1e-9)
            r2s.append(round(float(ss), 3))
            amps.append(round(float(np.hypot(c_[0], c_[1])), 2))
        res["bm"][f"{nm}_heading_R2_xy"] = r2s
        res["bm"][f"{nm}_heading_amp_uT_xy"] = amps
    # DRIFT_WARN episodes
    dw = (d["g2"] & 64) > 0
    if dw.any():
        edges = np.where(np.diff(dw.astype(int)) != 0)[0] + 1
        spans = []
        i0_ = None
        for i in range(n):
            if dw[i] and i0_ is None:
                i0_ = i
            elif not dw[i] and i0_ is not None:
                spans.append((float(t[i0_]), float(t[i - 1])))
                i0_ = None
        if i0_ is not None:
            spans.append((float(t[i0_]), float(t[-1])))
        # b_m rate in each span (10s window before end)
        bmr = np.hypot(np.gradient(pd.Series(bm2x).rolling(21, center=True, min_periods=1).mean().values, t),
                       np.gradient(pd.Series(bm2y).rolling(21, center=True, min_periods=1).mean().values, t))
        sp_stats = []
        for (a_, b_) in spans:
            w = (t >= a_ - 10) & (t <= b_)
            sp_stats.append(dict(t0=a_, t1=b_, dur=round(b_ - a_, 2),
                                 bm_rate_max=round(float(bmr[w].max()), 3)))
        res["bm"]["drift_warn_spans"] = sp_stats
        res["bm"]["drift_warn_rows"] = int(dw.sum())
    d["bm2"] = (bm2x, bm2y)
    d["bm1"] = (bm1x, bm1y)

    # ---------- Task 5: yaw accuracy 4 systems ----------
    truth = d["truth_onb"]
    truth_u = np.degrees(np.unwrap(np.radians(truth)))
    fly = d["fly"]
    # align gyro/madgwick at post-latch ground window
    gw2 = (t > d["t_arm"] + 1.5) & (t < (d["t_lift"] or d["t_flying"]) - 0.3)
    errs = {}
    err_raw = {}
    for nm, sig in (("EKF1", d["e1"]), ("EKF2", d["e2"]),
                    ("gyro_int", d["gy"]), ("madgwick", d["md"])):
        e_ = wrapd(sig - truth)
        err_raw[nm] = e_
        off = float(np.median(e_[gw2]))
        errs[nm] = e_ if nm in ("EKF1", "EKF2") else wrapd(e_ - off)
        A_ = np.polyfit(t[fly], errs[nm][fly], 1)
        res.setdefault("yaw_err", {})[nm] = dict(
            flight_rms=float(np.sqrt(np.mean(errs[nm][fly]**2))),
            flight_mean=float(errs[nm][fly].mean()),
            flight_maxabs=float(np.abs(errs[nm][fly]).max()),
            drift_deg_per_min=float(A_[0] * 60),
            ground_offset_used=off if nm not in ("EKF1", "EKF2") else 0.0)
    d["errs"] = errs
    # phases from cmd yaw rate
    cmd_u = np.degrees(np.unwrap(df["cmd_yaw_ref_rad"].values))
    dtv = np.gradient(t)
    cmd_rate = pd.Series(np.gradient(cmd_u, t)).rolling(9, center=True, min_periods=1).mean().values
    man = (np.abs(cmd_rate) > 2.0) & fly
    segs = []
    i = 0
    while i < n:
        if man[i]:
            j = i
            while j + 1 < n and man[j + 1]:
                j += 1
            if abs(cmd_u[j] - cmd_u[i]) > 5:
                segs.append(dict(t0=float(t[i]), t1=float(t[j]),
                                 dpsi=round(float(cmd_u[j] - cmd_u[i]), 1)))
            i = j + 1
        else:
            i += 1
    res["turn_segments"] = segs
    turn_mask = np.zeros(n, bool)
    for s_ in segs:
        turn_mask |= (t >= s_["t0"]) & (t <= s_["t1"] + 2)
    hover = fly & ~turn_mask
    res["yaw_err_phases"] = {}
    for nm in ("EKF1", "EKF2", "gyro_int", "madgwick"):
        e_ = errs[nm]
        res["yaw_err_phases"][nm] = {
            "hover": dict(n=int(hover.sum()), rms=float(np.sqrt(np.mean(e_[hover]**2))),
                          maxabs=float(np.abs(e_[hover]).max())),
            "turn": dict(n=int((turn_mask & fly).sum()),
                         rms=float(np.sqrt(np.mean(e_[turn_mask & fly]**2))),
                         maxabs=float(np.abs(e_[turn_mask & fly]).max())) if (turn_mask & fly).any() else None}
    # true yaw rate & lag estimate for EKF2
    tr_rate = pd.Series(np.gradient(truth_u, t)).rolling(9, center=True, min_periods=1).mean().values
    res["true_peak_rate_deg_s"] = float(np.abs(tr_rate[fly]).max())
    e2u = np.degrees(np.unwrap(np.radians(d["e2"])))
    best = None
    for lag in np.arange(-0.1, 0.25, 0.01):
        tv = truth_u  # shift truth back by lag: truth(t - lag)
        tshift = np.interp(t, t + lag, truth_u)
        er = (e2u - tshift)[fly]
        er = er - np.mean(er)
        rmsv = float(np.sqrt(np.mean(er**2)))
        if best is None or rmsv < best[1]:
            best = (round(float(lag), 3), rmsv)
    res["ekf2_lag"] = dict(best_lag_s=best[0], rms_at_best_lag_meanfree=best[1],
                           rms_lag0_meanfree=float(np.sqrt(np.mean(
                               ((e2u - truth_u)[fly] - np.mean((e2u - truth_u)[fly]))**2))))
    # error vs rate correlation
    res["ekf2_err_vs_rate_corr"] = float(np.corrcoef(
        np.abs(tr_rate[fly]), np.abs(errs["EKF2"][fly]))[0, 1])

    # basic timeline
    res["timeline"] = dict(dur=float(t[-1]), t_arm=d["t_arm"], t_lift=d["t_lift"],
                           t_flying=d["t_flying"], t_landing=d["t_landing"],
                           t_flight_anchor=t_fa,
                           volt_end=float(df["tlm_voltage_v"].values[-10:].mean()),
                           firmware_uptime_at_log0_ms=int(df["tlm_elapsed_ms"].values[0]))
    R[name] = res
    d["res"] = res
    if name == "main2115":
        D_MAIN = d
    else:
        D_SHORT = d

# ================================================================ plots
c = dict(t="#333333", e1="#d62728", e2="#1f77b4", gy="#2ca02c", md="#9467bd",
         fa="#ff7f0e", inn="#1f77b4")

# ---- 01 fusion timeline (main) ----
d = D_MAIN
t = d["t"]
fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True,
                         gridspec_kw=dict(height_ratios=[2, 1.2, 1, 1]))
ax = axes[0]
ax.plot(t, d["truth_onb"], color=c["t"], lw=1.2, label="MoCap真値(機体系)")
ax.plot(t, d["e2"], color=c["e2"], lw=0.9, alpha=0.85, label="EKF2")
ax.plot(t, d["e1"], color=c["e1"], lw=0.9, alpha=0.7, label="EKF1")
ax.plot(t, d["sent"], color="#17becf", lw=0.7, alpha=0.6, label="送信基準(sent)")
ax.set_ylabel("ヨー [deg]")
ax.legend(loc="upper left", fontsize=9, ncol=4)
ax.set_title("21:15 メインフライト: ヨー観測融合タイムライン(初の fused=100%)")
ax = axes[1]
chg = np.where(np.abs(np.diff(d["inn"])) > 1e-9)[0] + 1
ax.plot(t[chg], d["inn"][chg], ".", ms=2.5, color=c["inn"], label="innovation(観測毎)")
ax.axhline(30, color="r", ls="--", lw=0.8)
ax.axhline(-30, color="r", ls="--", lw=0.8, label="±30°ゲート")
ax.set_ylim(-12, 12)
ax.set_ylabel("innov [deg]")
ax.legend(loc="upper right", fontsize=9)
ax.text(0.99, 0.05, f"innov RMS(観測毎)= {d['res']['fusion']['innov_obs']['rms']:.2f}°"
        f" / max {d['res']['fusion']['innov_obs']['maxabs']:.1f}°(アーム時アンカー再ラッチ段差)",
        transform=ax.transAxes, ha="right", fontsize=9)
ax = axes[2]
ax.fill_between(t, 0, ((d["s2"] & 2) > 0).astype(int), color=c["e2"], alpha=0.6,
                step="mid", label="fused (bit1)")
ax.fill_between(t, 1.1, 1.1 + ((d["s2"] & 4) > 0).astype(int) * 0.9, color=c["fa"],
                alpha=0.7, step="mid", label="flight_anchor (bit2)")
ax.set_ylim(-0.1, 2.1)
ax.set_yticks([])
ax.legend(loc="center right", fontsize=9)
ax.set_ylabel("状態")
ax = axes[3]
ax.fill_between(t, 0, ((d["g2"] & 8) > 0).astype(int), color="#d62728", alpha=0.7,
                step="mid", label="EKF2 Z_REJECT")
ax.fill_between(t, 1.1, 1.1 + ((d["ffg"] & 8) > 0).astype(int) * 0.9, color="#8c564b",
                alpha=0.6, step="mid", label="EKF1 Z_REJECT (ffg)")
ax.fill_between(t, 2.2, 2.2 + ((d["g2"] & 64) > 0).astype(int) * 0.9, color="#9467bd",
                alpha=0.7, step="mid", label="EKF2 DRIFT_WARN")
ax.set_ylim(-0.1, 3.3)
ax.set_yticks([])
ax.legend(loc="center right", fontsize=9)
ax.set_ylabel("ゲート")
ax.set_xlabel("t [s]")
for ax in axes:
    for tv, lb, cc in ((d["t_arm"], "アーム", "k"), (d["t_lift"], "離陸", "g"),
                       (d["t_fa"], "飛行アンカー", c["fa"]), (d["t_landing"], "着陸", "gray")):
        ax.axvline(tv, color=cc, ls=":", lw=1)
axes[0].annotate("アーム", (d["t_arm"], axes[0].get_ylim()[1] * 0.9), fontsize=8)
axes[0].annotate("飛行アンカー", (d["t_fa"], axes[0].get_ylim()[1] * 0.75), fontsize=8,
                 color=c["fa"])
fig.tight_layout()
fig.savefig(PLOTS + "/01_fusion_timeline_2115.png", dpi=130)
plt.close(fig)

# ---- 02 flight anchor / z-gate (both flights) ----
fig, axes = plt.subplots(3, 2, figsize=(14, 9), sharex="col",
                         gridspec_kw=dict(height_ratios=[1.5, 1, 1]))
for col, (dd, ttl) in enumerate(((D_MAIN, "21:15 メイン"), (D_SHORT, "21:13 ショート"))):
    t_ = dd["t"]
    df_ = dd["df"]
    ax = axes[0][col]
    magz = df_["tlm_mag_cal_z_ut"].values
    ax.plot(t_, magz - dd["b0z_gnd"], lw=0.8, color="#333",
            label="mag_cal_z − 地上B0z")
    ax.axhline(0, color="gray", lw=0.5)
    ax.axhline(-12, color="r", ls="--", lw=0.8, label="±12µT (Z_REJECTしきい値相当)")
    ax.axhline(12, color="r", ls="--", lw=0.8)
    ax.set_ylabel("Δmag_z [µT]")
    ax.set_title(f"{ttl}: 磁気z逸脱と飛行状態再アンカー")
    ax.legend(fontsize=8, loc="lower right")
    ax = axes[1][col]
    ax.fill_between(t_, 0, ((dd["g2"] & 8) > 0).astype(int), color="#d62728",
                    alpha=0.75, step="mid", label="EKF2 Z_REJECT")
    ax.fill_between(t_, 1.1, 1.1 + ((dd["ffg"] & 8) > 0).astype(int) * 0.9,
                    color="#8c564b", alpha=0.6, step="mid", label="EKF1 Z_REJECT")
    ax.fill_between(t_, 2.2, 2.2 + ((dd["ffg"] & 32) > 0).astype(int) * 0.9,
                    color="#e377c2", alpha=0.6, step="mid", label="EKF1 BM_FROZEN")
    ax.set_ylim(-0.1, 3.3)
    ax.set_yticks([])
    ax.legend(fontsize=8, loc="center right")
    ax.set_ylabel("ゲート")
    ax = axes[2][col]
    ax.plot(t_, df_["tlm_current_a"].values, lw=0.8, color="#2ca02c", label="電流 [A]")
    ax.plot(t_, np.abs(df_["tlm_altitude_est_m"].values - df_["tlm_alt_ref_m"].values) * 10,
            lw=0.8, color="#1f77b4", label="|alt_est−alt_ref|×10 [m]")
    ax.axhline(1.0, color="#2ca02c", ls="--", lw=0.7)
    ax.axhline(0.1 * 10, color="#1f77b4", ls="--", lw=0.7)
    ax.set_ylabel("発動条件")
    ax.legend(fontsize=8)
    ax.set_xlabel("t [s]")
    for r_ in range(3):
        for tv, cc in ((dd["t_arm"], "k"), (dd["t_flying"], "g"),
                       (dd["t_fa"], "#ff7f0e"), (dd["t_landing"], "gray")):
            axes[r_][col].axvline(tv, color=cc, ls=":", lw=1)
    axes[0][col].annotate("飛行アンカー発動", (dd["t_fa"], axes[0][col].get_ylim()[0] * 0.8),
                          fontsize=9, color="#ff7f0e")
fig.tight_layout()
fig.savefig(PLOTS + "/02_flight_anchor_zgate.png", dpi=130)
plt.close(fig)

# ---- 03 b_m EKF1 vs EKF2 (main) ----
d = D_MAIN
t = d["t"]
fig, axes = plt.subplots(3, 1, figsize=(13, 9), sharex=True,
                         gridspec_kw=dict(height_ratios=[1.4, 1.4, 1]))
ax = axes[0]
ax.plot(t, d["bm2"][0], color=c["e2"], lw=1.1, label="EKF2 b_m,x(融合下=真のFF残差)")
ax.plot(t, d["bm1"][0], color=c["e1"], lw=0.9, alpha=0.7, label="EKF1 b_m,x(縮退)")
ax.set_ylabel("b_m x [µT]")
ax.legend(fontsize=9)
ax.set_title("21:15: b_m 可観測化の実証 — EKF2(ヨー観測融合)vs EKF1(磁気のみ)")
ax = axes[1]
ax.plot(t, d["bm2"][1], color=c["e2"], lw=1.1, label="EKF2 b_m,y")
ax.plot(t, d["bm1"][1], color=c["e1"], lw=0.9, alpha=0.7, label="EKF1 b_m,y")
ax.set_ylabel("b_m y [µT]")
ax.legend(fontsize=9)
ax = axes[2]
ax.plot(t, d["truth_onb"], color=c["t"], lw=0.9, label="ヨー真値")
ax.set_ylabel("ヨー [deg]")
ax.set_xlabel("t [s]")
ax.legend(fontsize=9, loc="upper left")
dw = (d["g2"] & 64) > 0
for ax in axes:
    yl = ax.get_ylim()
    ax.fill_between(t, yl[0], yl[1], where=dw, color="#9467bd", alpha=0.15)
    ax.axvline(d["t_fa"], color=c["fa"], ls=":", lw=1.2)
    ax.axvline(d["t_flying"], color="g", ls=":", lw=0.8)
    ax.axvline(d["t_landing"], color="gray", ls=":", lw=0.8)
    ax.set_ylim(yl)
axes[0].annotate("飛行アンカー(b_m←0リセット)", (d["t_fa"] + 0.5, axes[0].get_ylim()[1] * 0.8),
                 fontsize=9, color=c["fa"])
axes[0].annotate("紫帯=DRIFT_WARN", (60, axes[0].get_ylim()[0] * 0.9), fontsize=9,
                 color="#9467bd")
fig.tight_layout()
fig.savefig(PLOTS + "/03_bm_ekf1_vs_ekf2.png", dpi=130)
plt.close(fig)

# ---- 04 yaw error 4 systems + short flight ----
fig = plt.figure(figsize=(14, 10))
gs = fig.add_gridspec(3, 2, height_ratios=[1.6, 1, 1])
ax = fig.add_subplot(gs[0, :])
d = D_MAIN
t = d["t"]
for nm, cc in (("EKF1", c["e1"]), ("EKF2", c["e2"]), ("gyro_int", c["gy"]),
               ("madgwick", c["md"])):
    ax.plot(t, d["errs"][nm], color=cc, lw=0.9,
            label=f"{nm} (飛行RMS {d['res']['yaw_err'][nm]['flight_rms']:.2f}°)")
for s_ in d["res"]["turn_segments"]:
    ax.axvspan(s_["t0"], s_["t1"], color="orange", alpha=0.12)
ax.axhline(0, color="gray", lw=0.5)
ax.axvline(d["t_flying"], color="g", ls=":", lw=1)
ax.axvline(d["t_landing"], color="gray", ls=":", lw=1)
ax.set_ylim(-25, 25)
ax.set_ylabel("ヨー誤差 [deg]")
ax.set_title("21:15: ヨー誤差4系統(MoCap真値基準、橙帯=回頭)")
ax.legend(fontsize=9, ncol=4)
ax = fig.add_subplot(gs[1, 0])
names = ["EKF2", "gyro_int", "madgwick", "EKF1"]
ph = d["res"]["yaw_err_phases"]
x = np.arange(len(names))
ax.bar(x - 0.2, [ph[nm]["hover"]["rms"] for nm in names], 0.38, label="ホバ", color="#1f77b4")
ax.bar(x + 0.2, [ph[nm]["turn"]["rms"] for nm in names], 0.38, label="回頭", color="#ff7f0e")
ax.set_xticks(x, names)
ax.set_ylabel("RMS [deg]")
ax.set_title("フェーズ別RMS(21:15)")
ax.legend(fontsize=9)
ax = fig.add_subplot(gs[1, 1])
tr_rate = pd.Series(np.gradient(np.degrees(np.unwrap(np.radians(d["truth_onb"]))), t)
                    ).rolling(9, center=True, min_periods=1).mean().values
ax.plot(np.abs(tr_rate[d["fly"]]), np.abs(d["errs"]["EKF2"][d["fly"]]), ".", ms=2,
        alpha=0.3, color=c["e2"])
ax.set_xlabel("|ヨーレート| [deg/s]")
ax.set_ylabel("|EKF2誤差| [deg]")
ax.set_title(f"EKF2誤差 vs 回頭レート(相関 {d['res']['ekf2_err_vs_rate_corr']:.2f}"
             f"、遅れ≈{d['res']['ekf2_lag']['best_lag_s'] * 1000:.0f}ms)")
d2 = D_SHORT
ax = fig.add_subplot(gs[2, :])
for nm, cc in (("EKF1", c["e1"]), ("EKF2", c["e2"]), ("gyro_int", c["gy"])):
    ax.plot(d2["t"], d2["errs"][nm], color=cc, lw=0.9,
            label=f"{nm} (飛行RMS {d2['res']['yaw_err'][nm]['flight_rms']:.2f}°)")
ax.axhline(0, color="gray", lw=0.5)
ax.axvline(d2["t_flying"], color="g", ls=":", lw=1)
ax.axvline(d2["t_fa"], color=c["fa"], ls=":", lw=1)
ax.set_ylabel("ヨー誤差 [deg]")
ax.set_xlabel("t [s]")
ax.set_title("21:13 ショート: EKF1はBM_FROZEN(全行)+アーム時~20°オフセットのまま — EKF2は融合で追従")
ax.legend(fontsize=9, ncol=3)
fig.tight_layout()
fig.savefig(PLOTS + "/04_yaw_error_4sys.png", dpi=130)
plt.close(fig)

with open(OUT + "/stats_a2115.json", "w") as f:
    json.dump(R, f, indent=1, ensure_ascii=False)
print(json.dumps(R, indent=1, ensure_ascii=False))
