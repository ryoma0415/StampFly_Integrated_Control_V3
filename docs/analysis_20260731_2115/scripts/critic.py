#!/usr/bin/env python
"""Critic recomputation to settle A1/A2 contradictions (21:15 / 21:13 flights)."""
import json, math
import numpy as np
import pandas as pd

BASE = "/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/StampFly_MoCap_System_v2/StampFly_Integrated_Control_V3/logs/flight_logs"
OUT = "/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad/a2115/critic"

def wrap(a):
    return (a + 180.0) % 360.0 - 180.0

res = {}

for label, fname in [("2115", "20260731_211535_position.csv"), ("2113", "20260731_211304_position.csv")]:
    df = pd.read_csv(f"{BASE}/{fname}", comment="#")
    t = df["elapsed_time"].values
    state = df["tlm_state"].values
    fly = state == 4
    r = {}

    # ---- Timeline semantics (C4) ----
    r["timeline"] = {
        "t_state3_first": float(t[state == 3][0]) if (state == 3).any() else None,
        "t_state4_first": float(t[fly][0]) if fly.any() else None,
        "alt_first_gt_0p05": float(t[(df["tlm_altitude_est_m"].values > 0.05)][0]),
        "phase_vals": {str(k): int(v) for k, v in df["phase"].value_counts().items()},
    }

    # ---- C1: tracking_valid dropouts vs innov spikes ----
    tv = df["tracking_valid"].values.astype(float)
    inv = tv < 0.5
    runs = []
    i = 0
    n = len(inv)
    while i < n:
        if inv[i]:
            j = i
            while j + 1 < n and inv[j + 1]:
                j += 1
            runs.append((float(t[i]), float(t[j]), float(t[j] - t[i]) + 0.02, j - i + 1))
            i = j + 1
        else:
            i += 1
    r["tracking_invalid"] = {
        "rows": int(inv.sum()), "runs": len(runs),
        "run_list": [(a, b, round(d, 3), c) for a, b, d, c in runs],
    }

    # innovation: dedup consecutive identical values (telemetry hold)
    innov = np.degrees(df["tlm_ekf2_yaw_innov_rad"].values)
    chg = np.ones(n, dtype=bool)
    chg[1:] = np.abs(np.diff(innov)) > 1e-9
    ti, vi = t[chg], innov[chg]
    order = np.argsort(-np.abs(vi))
    top = [(float(ti[k]), float(vi[k])) for k in order[:10]]
    r["innov"] = {
        "rms_obs_all": float(np.sqrt(np.mean(vi ** 2))),
        "maxabs_all": float(np.max(np.abs(vi))),
        "t_maxabs_all": float(ti[np.argmax(np.abs(vi))]),
        "top10_spikes": top,
    }
    # in-flight (state4) innov spikes
    flyi = np.interp(ti, t, fly.astype(float)) > 0.5
    tif, vif = ti[flyi], vi[flyi]
    orf = np.argsort(-np.abs(vif))
    r["innov"]["flight_top5"] = [(float(tif[k]), float(vif[k])) for k in orf[:5]]
    r["innov"]["flight_maxabs"] = float(np.max(np.abs(vif)))
    r["innov"]["flight_rms"] = float(np.sqrt(np.mean(vif ** 2)))

    # do the flight innov spikes coincide with a tracking_invalid run end (recapture)?
    spike_attrib = []
    for ts, vs in [(float(tif[k]), float(vif[k])) for k in orf[:5]]:
        near = None
        for a, b, d, c in runs:
            if a - 0.3 <= ts <= b + 0.6:
                near = (a, b, round(d, 3))
                break
        spike_attrib.append({"t": ts, "innov": round(vs, 2), "occl_run": near})
    r["innov"]["flight_spike_occlusion_attribution"] = spike_attrib

    # position correction at recapture (jump in pos_x/pos_y after run end)
    px, py = df["pos_x"].values, df["pos_y"].values
    jumps = []
    for a, b, d, c in runs:
        idx = np.searchsorted(t, b)
        if idx + 2 < n and idx > 1:
            dp = math.hypot(px[idx + 1] - px[idx - 1], py[idx + 1] - py[idx - 1])
            jumps.append({"t_end": b, "dur": round(d, 3), "pos_jump_m": round(float(dp), 3)})
    jumps.sort(key=lambda x: -x["pos_jump_m"])
    r["recapture_pos_jumps_top3"] = jumps[:3]

    # ---- C3: EKF1/EKF2 error in both frames ----
    truth = df["mocap_yaw_true_deg"].values
    sent = np.degrees(df["yaw_ref_sent_rad"].values)
    e1 = np.degrees(df["tlm_yaw_est_rad"].values)
    e2 = np.degrees(df["tlm_ekf2_yaw_rad"].values)
    anchor_deg = float(np.median(wrap(sent[fly] - truth[fly])))
    for name, est in [("EKF1", e1), ("EKF2", e2)]:
        pc = wrap(est[fly] - truth[fly])              # PC frame (vs raw truth)
        body = wrap(est[fly] - sent[fly])             # control frame (vs truth+anchor)
        r[f"{name}_err"] = {
            "pc_frame_rms": float(np.sqrt(np.mean(pc ** 2))),
            "pc_frame_mean": float(np.mean(pc)),
            "body_frame_rms": float(np.sqrt(np.mean(body ** 2))),
            "body_frame_mean": float(np.mean(body)),
            "body_frame_std": float(np.std(body)),
            "body_frame_maxabs": float(np.max(np.abs(body))),
        }
    r["anchor_implied_deg"] = anchor_deg

    # ---- C2: flight_anchor precondition timeline ----
    st2 = df["tlm_ekf2_status"].values.astype(int)
    fa = (st2 & 4) > 0
    t_fa = float(t[fa][0]) if fa.any() else None
    alt_est = df["tlm_altitude_est_m"].values
    alt_ref = df["tlm_alt_ref_m"].values
    cur = df["tlm_current_a"].values
    in_flight = state >= 3  # state3 (takeoff) onward
    t_if0 = float(t[in_flight][0]) if in_flight.any() else None
    # alt-hold condition |alt_est-alt_ref|<0.1 sustained 2 s: find last violation before t_fa
    ok = np.abs(alt_est - alt_ref) < 0.1
    viol_before = t[(~ok) & (t < (t_fa if t_fa else t[-1]))]
    t_last_viol = float(viol_before[-1]) if len(viol_before) else None
    r["flight_anchor"] = {
        "t_fire": t_fa,
        "t_in_flight_start": t_if0,
        "in_flight_plus5": (t_if0 + 5.0) if t_if0 is not None else None,
        "t_last_althold_violation": t_last_viol,
        "altviol_plus2": (t_last_viol + 2.0) if t_last_viol else None,
        "current_at_fire": float(np.interp(t_fa, t, cur)) if t_fa else None,
        "current_first_gt1A": float(t[cur > 1.0][0]) if (cur > 1.0).any() else None,
        "binding_constraint": None,
    }
    cands = {"in_flight+5s": r["flight_anchor"]["in_flight_plus5"],
             "alt_hold_2s": r["flight_anchor"]["altviol_plus2"]}
    r["flight_anchor"]["binding_constraint"] = max(cands, key=lambda k: cands[k] or 0)
    # alt-hold violations during flight before anchor (settling behaviour)
    if t_fa:
        win = (t > (t_if0 or 0)) & (t < t_fa) & (~ok)
        r["flight_anchor"]["altviol_frac_between_takeoff_and_fire"] = float(win.sum()) / max(1, int(((t > (t_if0 or 0)) & (t < t_fa)).sum()))
        r["flight_anchor"]["alt_err_max_prefire"] = float(np.max(np.abs(alt_est - alt_ref)[(t > (t_if0 or 0)) & (t < t_fa)]))

    # z_reject causality: is z_reject explained by mag_z shift vs ground B0 (not "extra disturbance")?
    magz = df["tlm_mag_cal_z_ut"].values
    gate2 = df["tlm_ekf2_gate"].values.astype(int)
    zrej = (gate2 & 8) > 0
    ground = state <= 1
    r["magz"] = {
        "ground_mean": float(np.mean(magz[ground])) if ground.any() else None,
        "flight_mean_pre_anchor": float(np.mean(magz[fly & (t < (t_fa or 1e9))])) if (fly & (t < (t_fa or 1e9))).any() else None,
        "flight_mean_post_anchor": float(np.mean(magz[fly & (t >= (t_fa or 1e9))])) if t_fa else None,
        "zrej_pre_anchor_frac": float(np.mean(zrej[fly & (t < (t_fa or 1e9))])) if (fly & (t < (t_fa or 1e9))).any() else None,
        "zrej_post_anchor_frac": float(np.mean(zrej[fly & (t >= (t_fa or 1e9))])) if t_fa else None,
    }
    res[label] = r

# ---- C5: 18:20 glitch-rate baseline (two definitions) ----
df18 = pd.read_csv(f"{BASE}/20260731_182025_position.csv", comment="#")
truth18 = df18["mocap_yaw_true_deg"].values
head18 = df18["mocap_heading_deg"].values
raw_true = wrap(-head18 + 88.4)
diff = np.abs(wrap(raw_true - truth18))
n18 = len(df18)
res["glitch_1820_baseline"] = {
    "rows_total": n18,
    "rows_gt45_rawtrue_vs_logged": int((diff > 45).sum()),
    "pct_gt45": round(float((diff > 45).sum()) / n18 * 100, 2),
    "note": "26.4% (728 rows) figure in FLIGHT_ANALYSIS_20260731_1820.md uses the 90-deg-family classifier incl. bridged rows; >45deg raw-vs-logged gives 18.45%",
}

with open(f"{OUT}/critic_results.json", "w") as f:
    json.dump(res, f, indent=1, default=str)
print(json.dumps(res, indent=1, default=str))
