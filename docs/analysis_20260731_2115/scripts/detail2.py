#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""Follow-up details: innov by phase, EKF1 slew at arm, b_m rate at DRIFT_WARN,
b_m trajectory milestones, in-flight innov max."""
import json
import numpy as np
import pandas as pd

BASE = ("/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/"
        "StampFly_MoCap_System_v2/StampFly_Integrated_Control_V3")
LOGS = {"main2115": BASE + "/logs/flight_logs/20260731_211535_position.csv",
        "short2113": BASE + "/logs/flight_logs/20260731_211304_position.csv"}

def wrapd(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0

out = {}
for name, path in LOGS.items():
    df = pd.read_csv(path)
    t = df["elapsed_time"].values
    n = len(df)
    st = df["tlm_state"].values
    i_arm = int(np.argmax(st >= 3))
    t_arm = float(t[i_arm])
    t_fly = float(t[np.argmax(st == 4)])
    t_land = float(t[np.argmax(st == 5)])
    fly = (t >= t_fly) & (t < t_land)
    inn = np.degrees(df["tlm_ekf2_yaw_innov_rad"].values)
    e1 = np.degrees(df["tlm_yaw_est_rad"].values)
    r = {}
    # EKF1 slew around arm
    for lbl, (a, b) in (("pre_arm_1s", (t_arm - 1, t_arm)),
                        ("post_arm_1s", (t_arm, t_arm + 1)),
                        ("arm_pm2s_range", (t_arm - 2, t_arm + 2))):
        w = (t >= a) & (t <= b)
        r[f"e1_{lbl}"] = dict(first=float(e1[w][0]), last=float(e1[w][-1]),
                              rate_deg_s=float((e1[w][-1] - e1[w][0]) / (t[w][-1] - t[w][0])))
    # innov by phase (obs-level)
    chg = np.where(np.abs(np.diff(inn)) > 1e-9)[0] + 1
    cmd_u = np.degrees(np.unwrap(df["cmd_yaw_ref_rad"].values))
    cmd_rate = pd.Series(np.gradient(cmd_u, t)).rolling(9, center=True, min_periods=1).mean().values
    man = (np.abs(cmd_rate) > 2.0) & fly
    io, tc = inn[chg], t[chg]
    flyc = fly[chg]
    manc = man[chg]
    postarm = tc > t_arm + 2
    r["innov_rms_hover"] = float(np.sqrt(np.mean(io[flyc & ~manc]**2)))
    r["innov_rms_turn"] = float(np.sqrt(np.mean(io[flyc & manc]**2))) if (flyc & manc).any() else None
    r["innov_max_flight"] = float(np.abs(io[flyc]).max())
    r["innov_rms_postarm_ground"] = float(np.sqrt(np.mean(io[postarm & ~flyc]**2)))
    # b_m milestones + rate (uniform resample)
    bmx = df["tlm_ekf2_bm_x_ut"].values
    bmy = df["tlm_ekf2_bm_y_ut"].values
    s2 = df["tlm_ekf2_status"].astype(int).values
    g2 = df["tlm_ekf2_gate"].astype(int).values
    i_fa = int(np.argmax((s2 & 4) > 0))
    t_fa = float(t[i_fa])
    tg = np.arange(t[0], t[-1], 0.05)
    bxg = np.interp(tg, t, pd.Series(bmx).rolling(15, center=True, min_periods=1).mean().values)
    byg = np.interp(tg, t, pd.Series(bmy).rolling(15, center=True, min_periods=1).mean().values)
    rate = np.hypot(np.gradient(bxg, tg), np.gradient(byg, tg))
    r["bm_milestones"] = {}
    for dt_ in (5, 10, 20, 40):
        w = (t >= t_fa + dt_ - 1) & (t <= t_fa + dt_ + 1)
        if w.any():
            r["bm_milestones"][f"anchor+{dt_}s"] = [round(float(bmx[w].mean()), 2),
                                                    round(float(bmy[w].mean()), 2)]
    # drift warn spans with rate
    dw = (g2 & 64) > 0
    spans = []
    i0_ = None
    for i in range(n):
        if dw[i] and i0_ is None:
            i0_ = i
        elif not dw[i] and i0_ is not None:
            spans.append((float(t[i0_]), float(t[i - 1])))
            i0_ = None
    r["drift_warn"] = []
    for (a, b) in spans:
        w10 = (tg >= a - 10) & (tg <= b)
        win = (tg >= a - 10) & (tg <= a)
        r["drift_warn"].append(dict(
            t0=a, t1=b,
            bm_rate_mean_prior10s=round(float(rate[win].mean()), 3),
            bm_rate_max_prior10s=round(float(rate[win].max()), 3)))
    # heading rate at drift warn
    yt = df["mocap_yaw_true_deg"].values
    yt_u = np.degrees(np.unwrap(np.radians(yt)))
    hr = pd.Series(np.gradient(np.interp(tg, t, yt_u), tg)).rolling(9, center=True, min_periods=1).mean().values
    for i, (a, b) in enumerate(spans):
        win = (tg >= a - 10) & (tg <= a)
        r["drift_warn"][i]["heading_rate_maxabs_prior10s"] = round(float(np.abs(hr[win]).max()), 1)
    # ekf2 err vs rate corr (redo, was truncated)
    anchor = json.load(open(path.replace(".csv", ".meta.json")))["yaw_ref"]["anchor"]["anchor_rad"]
    truth = wrapd(yt + np.degrees(anchor))
    e2 = np.degrees(df["tlm_ekf2_yaw_rad"].values)
    err2 = wrapd(e2 - truth)
    tr_rate = pd.Series(np.gradient(yt_u, t)).rolling(9, center=True, min_periods=1).mean().values
    r["ekf2_err_vs_rate_corr"] = float(np.corrcoef(np.abs(tr_rate[fly]), np.abs(err2[fly]))[0, 1])
    # regression err vs rate slope: deg err per deg/s
    A = np.polyfit(tr_rate[fly], err2[fly], 1)
    r["ekf2_err_per_rate_slope_ms"] = float(A[0] * 1000)  # ~effective lag ms
    # EKF1 vs EKF2 bm at end for magbias compare
    r["bm_ekf1_final"] = [float(df["tlm_bm_x_ut"].values[-1]), float(df["tlm_bm_y_ut"].values[-1])]
    r["bm_ekf2_final"] = [float(bmx[-1]), float(bmy[-1])]
    # z-reject rows count post-anchor (main: 48 rows where?)
    z2 = (g2 & 8) > 0
    if z2.any():
        r["z2_reject_times"] = [round(float(x), 2) for x in t[z2][:10]] + (
            ["..."] if z2.sum() > 10 else [])
        r["z2_reject_n"] = int(z2.sum())
        r["z2_reject_frac_after_anchor"] = float(z2[(t > t_fa)].mean())
    out[name] = r

print(json.dumps(out, indent=1, ensure_ascii=False))
