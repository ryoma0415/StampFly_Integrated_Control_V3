"""2026-07-27 flight logs: quantitative yaw-estimation error analysis.

Facts grounded in repo code (read-only):
- tlm_yaw_rad      = Madgwick AHRS yaw (sensor.cpp:250, wrapPi(-Drone_ahrs.getYawRadians()))
- tlm_yaw_est_rad  = active estimator yaw = EKF when ff_status bit2 set (sensor_hub_ff.cpp:412)
- tlm_yaw_gyro_int_rad = plain gyro-z integral (sensor.cpp:240)
- tlm_bm_x/y_ut    = EKF magnetic bias STATE x_[2],x_[3] (yaw_estimator_kf.hpp:70-71),
                     NOT the raw horizontal field. Obs model: z = R_z(psi-psi0) B0h + b_m.
- tlm_db_hat_x/y_ut= applied FF correction dB^ (current-driven); EKF sees mag - dB^.
- mocap_yaw_true_deg = corrected truth yaw; equals yaw_sign*(mocap_heading_deg - 88.6)
  per meta.json (verified numerically below).
Viewer conventions (flight_log_viewer/viewer/loader.py):
  error = wrap_deg(est_deg - mocap_yaw_true_deg); drift = polyfit slope on UNWRAPPED error.
"""
import json
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

REPO = ("/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/"
        "StampFly_MoCap_System_v2/StampFly_Integrated_Control_V2")
LOGS = [f"{REPO}/logs/flight_logs/20260727_164200_position.csv",
        f"{REPO}/logs/flight_logs/20260727_164520_position.csv"]
SCRATCH = ("/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-"
           "Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V2/"
           "e51a3555-1c34-4613-ac37-eff99446eece/scratchpad")
PLOTS = f"{SCRATCH}/plots"

TLM_FLAG_FLYING = 1 << 2
FFG_BITS = ["R_inflate", "NIS_reject", "norm_reject", "z_reject",
            "tilt_skip", "bm_frozen", "drift_warn", "recapture"]
FF_FLAG_BITS = [("est_mode_EKF", 2), ("anchor_valid", 3), ("ffcal_loaded", 4),
                ("yaw_ctrl_active", 5), ("mag_fresh", 6)]


def wrap_deg(a):
    return (np.asarray(a) + 180.0) % 360.0 - 180.0


def unwrap_deg(v):
    v = np.asarray(v, float)
    out = np.full_like(v, np.nan)
    f = np.isfinite(v)
    if f.sum() >= 2:
        out[f] = np.degrees(np.unwrap(np.radians(v[f])))
    return out


def linfit(t, y):
    """slope per unit t, intercept, R^2, rms of residual."""
    f = np.isfinite(t) & np.isfinite(y)
    if f.sum() < 2:
        return np.nan, np.nan, np.nan, np.nan
    p = np.polyfit(t[f], y[f], 1)
    yhat = np.polyval(p, t[f])
    ss_res = np.sum((y[f] - yhat) ** 2)
    ss_tot = np.sum((y[f] - np.mean(y[f])) ** 2)
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else np.nan
    return p[0], p[1], r2, float(np.sqrt(np.mean((y[f] - yhat) ** 2)))


def err_stats(t, err_wrapped, label):
    f = np.isfinite(err_wrapped)
    e = err_wrapped[f]
    tt = t[f]
    eu = unwrap_deg(err_wrapped)[f]
    slope, icpt, r2, rms_resid = linfit(tt, eu)
    init = float(np.mean(e[tt <= tt[0] + 2.0]))          # first 2 s
    final = float(np.mean(e[tt >= tt[-1] - 2.0]))        # last 2 s
    return {
        "series": label, "n": int(f.sum()),
        "mean_deg": float(np.mean(e)),
        "rms_deg": float(np.sqrt(np.mean(e ** 2))),
        "max_abs_deg": float(np.max(np.abs(e))),
        "init_offset_deg(first2s)": init,
        "final_err_deg(last2s)": final,
        "drift_deg_per_min": float(slope * 60.0),
        "fit_r2": float(r2),
        "rms_about_fit_deg": rms_resid,
        "rms_offset_removed_deg": float(np.sqrt(np.mean((e - init) ** 2))),
    }


def pearson(x, y):
    f = np.isfinite(x) & np.isfinite(y)
    if f.sum() < 3:
        return np.nan
    return float(np.corrcoef(x[f], y[f])[0, 1])


def analyze(path):
    name = path.split("/")[-1].replace("_position.csv", "")
    df = pd.read_csv(path)
    for c in df.columns:
        if c not in ("timestamp", "mode", "phase", "data_source"):
            df[c] = pd.to_numeric(df[c], errors="coerce")
    t = df["elapsed_time"].to_numpy(float)
    out = {"log": name, "n_rows": len(df)}

    # ---- 1. basics -------------------------------------------------------
    dt = np.diff(t)
    flags = df["tlm_flags"].to_numpy(float)
    flying = np.isfinite(flags) & ((flags.astype(int) & TLM_FLAG_FLYING) != 0)
    out["duration_s"] = float(t[-1] - t[0])
    out["dt_median_ms"] = float(np.median(dt) * 1e3)
    out["dt_p99_ms"] = float(np.percentile(dt, 99) * 1e3)
    out["dt_max_ms"] = float(np.max(dt) * 1e3)
    out["phase_counts"] = df["phase"].value_counts().to_dict()
    out["flying_time_s"] = float(flying.sum()) / 50.0
    tgx = df["target_x"].to_numpy(float)
    tgy = df["target_y"].to_numpy(float)
    tgz = df["target_z"].to_numpy(float)
    out["target_unique"] = sorted(set(zip(np.round(tgx, 3), np.round(tgy, 3),
                                          np.round(tgz, 3))))
    px, py, pz = (df[c].to_numpy(float) for c in ("pos_x", "pos_y", "pos_z"))
    r = np.hypot(px - tgx, py - tgy)[flying]
    out["hover_radius_mean_m"] = float(np.nanmean(r))
    out["hover_radius_p95_m"] = float(np.nanpercentile(r, 95))
    out["hover_radius_max_m"] = float(np.nanmax(r))
    out["pos_z_mean_m(flying)"] = float(np.nanmean(pz[flying]))
    out["mocap_flip_count"] = int(np.nansum(df["mocap_flip"].to_numpy(float)))

    # verify mocap_yaw_true relation:  true = -(heading - 88.6)  (wrapped)
    head = df["mocap_heading_deg"].to_numpy(float)
    true = df["mocap_yaw_true_deg"].to_numpy(float)
    resid = wrap_deg(true - (-(head - 88.6)))
    out["mocap_true_vs_heading_maxabs_deg"] = float(np.nanmax(np.abs(resid)))

    # ---- 2. yaw errors vs mocap truth -----------------------------------
    mocap = true
    yaw = {
        "EKF(tlm_yaw_est_rad)": np.degrees(df["tlm_yaw_est_rad"].to_numpy(float)),
        "GyroInt(tlm_yaw_gyro_int_rad)": np.degrees(df["tlm_yaw_gyro_int_rad"].to_numpy(float)),
        "Madgwick(tlm_yaw_rad)": np.degrees(df["tlm_yaw_rad"].to_numpy(float)),
    }
    errs = {k: np.asarray(wrap_deg(v - mocap), float) for k, v in yaw.items()}
    out["yaw_err_stats"] = [err_stats(t, e, k) for k, e in errs.items()]

    # ---- 3./7. b_m state (NOT raw field -- EKF bias state) --------------
    bmx = df["tlm_bm_x_ut"].to_numpy(float)
    bmy = df["tlm_bm_y_ut"].to_numpy(float)
    bmn = np.hypot(bmx, bmy)
    cur = df["tlm_current_a"].to_numpy(float)
    out["bm_final_x_ut(last30s_median)"] = float(np.nanmedian(bmx[t >= t[-1] - 30]))
    out["bm_final_y_ut(last30s_median)"] = float(np.nanmedian(bmy[t >= t[-1] - 30]))
    out["bm_norm_max_ut"] = float(np.nanmax(bmn))
    out["bm_norm_final_ut"] = float(np.nanmedian(bmn[t >= t[-1] - 30]))
    out["bm_dir_final_deg"] = float(np.degrees(np.arctan2(
        out["bm_final_y_ut(last30s_median)"], out["bm_final_x_ut(last30s_median)"])))
    # implied worst-case yaw shift if b_m were pure unmodeled disturbance
    # (|B0h| not logged; use nominal 30 uT Japan horizontal field, clearly an estimate)
    out["bm_implied_yaw_shift_deg@30uT"] = float(np.degrees(np.arcsin(
        min(out["bm_norm_final_ut"] / 30.0, 1.0))))
    # convergence: first time |bm - final| stays < 0.5 uT for 5 s
    fin = np.array([out["bm_final_x_ut(last30s_median)"],
                    out["bm_final_y_ut(last30s_median)"]])
    dist = np.hypot(bmx - fin[0], bmy - fin[1])
    conv = np.nan
    inside = dist < 0.5
    for i in range(len(t)):
        if inside[i] and np.all(inside[i:i + 250]):
            conv = t[i]
            break
    out["bm_converge_time_s(<0.5uT,5s)"] = float(conv)

    # correlations of bm with time / current / position (flying only)
    m = flying
    out["corr_bmx"] = {"time": pearson(t[m], bmx[m]), "current": pearson(cur[m], bmx[m]),
                       "pos_x": pearson(px[m], bmx[m]), "pos_y": pearson(py[m], bmx[m])}
    out["corr_bmy"] = {"time": pearson(t[m], bmy[m]), "current": pearson(cur[m], bmy[m]),
                       "pos_x": pearson(px[m], bmy[m]), "pos_y": pearson(py[m], bmy[m])}

    # EKF yaw error correlations
    e_ekf = errs["EKF(tlm_yaw_est_rad)"]
    out["corr_err_ekf"] = {
        "time": pearson(t[m], e_ekf[m]), "current": pearson(cur[m], e_ekf[m]),
        "bm_norm": pearson(bmn[m], e_ekf[m]),
        "bmx": pearson(bmx[m], e_ekf[m]), "bmy": pearson(bmy[m], e_ekf[m]),
        "pos_x": pearson(px[m], e_ekf[m]), "pos_y": pearson(py[m], e_ekf[m]),
    }

    # ---- 4. db_hat -------------------------------------------------------
    dbx = df["tlm_db_hat_x_ut"].to_numpy(float)
    dby = df["tlm_db_hat_y_ut"].to_numpy(float)
    dbn = np.hypot(dbx, dby)
    pre = ~flying & (t < t[flying][0] if flying.any() else True)
    out["db_hat_initial_xy_ut"] = [float(dbx[0]), float(dby[0])]
    out["db_hat_flying_median_xy_ut"] = [float(np.nanmedian(dbx[m])),
                                         float(np.nanmedian(dby[m]))]
    out["db_hat_flying_p5_p95_norm_ut"] = [float(np.nanpercentile(dbn[m], 5)),
                                           float(np.nanpercentile(dbn[m], 95))]
    out["db_hat_norm_median_ut"] = float(np.nanmedian(dbn[m]))
    out["ratio_bm_over_dbhat_median"] = float(np.nanmedian(bmn[m]) /
                                              np.nanmedian(dbn[m]))
    sx, ix, r2x, _ = linfit(cur[m], dbx[m])
    sy, iy, r2y, _ = linfit(cur[m], dby[m])
    out["dbhat_vs_current"] = {"x_slope_uT_per_A": float(sx), "x_r2": float(r2x),
                               "y_slope_uT_per_A": float(sy), "y_r2": float(r2y)}

    # ---- 5. bm vs current regression ------------------------------------
    sx, ix, r2x, _ = linfit(cur[m], bmx[m])
    sy, iy, r2y, _ = linfit(cur[m], bmy[m])
    out["bm_vs_current"] = {"x_slope_uT_per_A": float(sx), "x_icpt": float(ix),
                            "x_r2": float(r2x),
                            "y_slope_uT_per_A": float(sy), "y_icpt": float(iy),
                            "y_r2": float(r2y)}
    # pre-takeoff vs flying jump in bm
    if (~flying).any() and flying.any():
        out["bm_pre_vs_fly"] = {
            "pre_n": int((~flying).sum()),
            "pre_median_xy": [float(np.nanmedian(bmx[~flying])),
                              float(np.nanmedian(bmy[~flying]))],
            "fly_first5s_median_xy": [
                float(np.nanmedian(bmx[m & (t < t[m][0] + 5)])),
                float(np.nanmedian(bmy[m & (t < t[m][0] + 5)]))],
        }
    out["current_stats_a"] = {"pre_median": float(np.nanmedian(cur[~flying]))
                              if (~flying).any() else np.nan,
                              "fly_median": float(np.nanmedian(cur[m])),
                              "fly_p95": float(np.nanpercentile(cur[m], 95))}

    # ---- 6. NIS ----------------------------------------------------------
    nis = df["tlm_nis"].to_numpy(float)
    # telemetry holds last value; zeros occur before first update -> drop exact 0
    nz = nis[np.isfinite(nis) & (nis > 0)]
    out["nis"] = {
        "n_nonzero": int(nz.size),
        "mean": float(np.mean(nz)), "median": float(np.median(nz)),
        "p95": float(np.percentile(nz, 95)), "max": float(np.max(nz)),
        "frac_gt_5.99_pct": float(100 * np.mean(nz > 5.99)),
        "frac_gt_13.8_pct": float(100 * np.mean(nz > 13.8)),
        "chi2_expected": {"mean": 2.0, "gt5.99_pct": 5.0, "gt13.8_pct": 0.1},
    }

    # ---- 8. ffg / ff_status ---------------------------------------------
    ffg = df["tlm_ffg"].to_numpy(float)
    ffg_i = np.where(np.isfinite(ffg), ffg, 0).astype(int)
    out["ffg_bit_rates_pct"] = {nm: float(100 * np.mean((ffg_i >> b) & 1))
                                for b, nm in enumerate(FFG_BITS)}
    st = df["tlm_ff_status"].to_numpy(float)
    st_i = np.where(np.isfinite(st), st, 0).astype(int)
    out["ff_status_values"] = {int(k): int(v) for k, v in
                               pd.Series(st_i).value_counts().items()}
    out["ff_mode_values"] = {int(k): int(v) for k, v in
                             pd.Series(st_i & 0x03).value_counts().items()}
    out["ff_flag_rates_pct"] = {nm: float(100 * np.mean((st_i >> b) & 1))
                                for nm, b in FF_FLAG_BITS}

    # ---------------- figures --------------------------------------------
    mu = unwrap_deg(mocap)
    fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
    ax = axes[0]
    ax.plot(t, mu, "g", lw=1.4, label="MoCap truth (mocap_yaw_true_deg)")
    ax.plot(t, unwrap_deg(yaw["EKF(tlm_yaw_est_rad)"]), "r", lw=1, label="EKF")
    ax.plot(t, unwrap_deg(yaw["Madgwick(tlm_yaw_rad)"]), "orange", lw=1, label="Madgwick")
    ax.plot(t, unwrap_deg(yaw["GyroInt(tlm_yaw_gyro_int_rad)"]), "b", lw=1, label="GyroInt")
    ax.set_ylabel("yaw unwrapped [deg]"); ax.legend(ncol=2, fontsize=8)
    ax.set_title(f"{name}: yaw sources (unwrapped)")
    ax = axes[1]
    for (k, e), c in zip(errs.items(), ["r", "b", "orange"]):
        eu = unwrap_deg(e)
        s, i0, _, _ = linfit(t, eu)
        ax.plot(t, eu, c, lw=0.9,
                label=f"{k.split('(')[0]}: drift {s*60:+.2f} deg/min")
        ax.plot(t, np.polyval([s, i0], t), c, ls="--", lw=0.8, alpha=0.6)
    ax.axhline(0, color="gray", ls=":")
    ax.set_ylabel("error vs MoCap [deg] (unwrapped)"); ax.set_xlabel("t [s]")
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(f"{PLOTS}/{name}_01_yaw_and_error.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(4, 1, figsize=(13, 11), sharex=True)
    axes[0].plot(t, nis, lw=0.7, color="#0284c7")
    axes[0].axhline(5.99, color="orange", ls="--", lw=1, label="5.99 (95%)")
    axes[0].axhline(13.8, color="r", ls="--", lw=1, label="13.8 (99.9%)")
    axes[0].set_ylim(0, max(20, np.nanpercentile(nis, 99.5) * 1.1))
    axes[0].set_ylabel("NIS"); axes[0].legend(fontsize=8)
    axes[0].set_title(f"{name}: EKF diagnostics")
    axes[1].plot(t, bmx, label="b_m x"); axes[1].plot(t, bmy, label="b_m y")
    axes[1].plot(t, bmn, "purple", lw=1.2, label="|b_m|")
    axes[1].axhline(20, color="r", ls="--", lw=1, label="freeze 20uT")
    axes[1].set_ylabel("b_m state [uT]"); axes[1].legend(fontsize=8, ncol=2)
    axes[2].plot(t, dbx, label="db_hat x"); axes[2].plot(t, dby, label="db_hat y")
    axes[2].set_ylabel("db_hat [uT]"); axes[2].legend(fontsize=8)
    axes[3].plot(t, cur, "#f97316", lw=0.8)
    axes[3].set_ylabel("current [A]"); axes[3].set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig(f"{PLOTS}/{name}_02_ekf_diag.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, (v, lbl, sl, r2) in zip(axes, [
            (bmx, "b_m x [uT]", out["bm_vs_current"]["x_slope_uT_per_A"],
             out["bm_vs_current"]["x_r2"]),
            (bmy, "b_m y [uT]", out["bm_vs_current"]["y_slope_uT_per_A"],
             out["bm_vs_current"]["y_r2"]),
            (e_ekf, "EKF yaw err [deg]", np.nan, np.nan)]):
        ax.scatter(cur[m], v[m], s=2, alpha=0.25, c=t[m], cmap="viridis")
        if np.isfinite(sl):
            xx = np.linspace(np.nanmin(cur[m]), np.nanmax(cur[m]), 10)
            p = np.polyfit(cur[m][np.isfinite(v[m])], v[m][np.isfinite(v[m])], 1)
            ax.plot(xx, np.polyval(p, xx), "r--",
                    label=f"slope {sl:+.2f} uT/A, R2={r2:.2f}")
            ax.legend(fontsize=8)
        ax.set_xlabel("current [A]"); ax.set_ylabel(lbl)
    axes[2].set_title(f"corr(err,I)={out['corr_err_ekf']['current']:+.2f} (color=time)")
    fig.suptitle(f"{name}: vs current (flying only)")
    fig.tight_layout(); fig.savefig(f"{PLOTS}/{name}_03_vs_current.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(1, 2, figsize=(12, 4.5))
    axes[0].hist(nz, bins=60, density=True, alpha=0.7, label="NIS (nonzero)")
    xx = np.linspace(0.01, max(15, np.percentile(nz, 99)), 300)
    axes[0].plot(xx, 0.5 * np.exp(-xx / 2), "r", label="chi2(2) pdf")
    axes[0].axvline(5.99, color="orange", ls="--"); axes[0].axvline(13.8, color="r", ls="--")
    axes[0].set_xlabel("NIS"); axes[0].legend(fontsize=8)
    axes[0].set_title(f"{name}: NIS hist  >5.99: {out['nis']['frac_gt_5.99_pct']:.1f}% "
                      f"(exp 5%),  >13.8: {out['nis']['frac_gt_13.8_pct']:.2f}% (exp 0.1%)")
    ax = axes[1]
    ax.plot(px[m], py[m], lw=0.6, alpha=0.8)
    ax.scatter(tgx[m], tgy[m], c="r", marker="x", s=60, label="target")
    ax.set_aspect("equal"); ax.set_xlabel("pos_x [m]"); ax.set_ylabel("pos_y [m]")
    ax.set_title(f"hover r_mean={out['hover_radius_mean_m']*100:.1f}cm "
                 f"p95={out['hover_radius_p95_m']*100:.1f}cm")
    ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(f"{PLOTS}/{name}_04_nis_hist_hover.png", dpi=130)
    plt.close(fig)

    fig, axes = plt.subplots(2, 1, figsize=(13, 6), sharex=True)
    for b, nm in enumerate(FFG_BITS):
        act = ((ffg_i >> b) & 1).astype(bool)
        axes[0].fill_between(t, b, b + 0.8, where=act, step="mid")
    axes[0].set_yticks([b + 0.4 for b in range(8)]); axes[0].set_yticklabels(FFG_BITS, fontsize=7)
    axes[0].set_title(f"{name}: ffg gate bits")
    for row, (nm, b) in enumerate(FF_FLAG_BITS):
        act = ((st_i >> b) & 1).astype(bool)
        axes[1].fill_between(t, row, row + 0.8, where=act, step="mid")
    axes[1].set_yticks([r + 0.4 for r in range(len(FF_FLAG_BITS))])
    axes[1].set_yticklabels([n for n, _ in FF_FLAG_BITS], fontsize=7)
    axes[1].set_xlabel("t [s]"); axes[1].set_title("ff_status flag bits")
    fig.tight_layout(); fig.savefig(f"{PLOTS}/{name}_05_gates.png", dpi=130)
    plt.close(fig)

    # keep arrays for cross-log figure
    out["_t"] = t; out["_e_ekf"] = e_ekf; out["_bmx"] = bmx; out["_bmy"] = bmy
    out["_errs"] = errs
    return out


results = []
for p in LOGS:
    results.append(analyze(p))

# cross-log comparison figure
fig, axes = plt.subplots(2, 1, figsize=(13, 8), sharex=True)
for r, c in zip(results, ["#d62728", "#1f77b4"]):
    axes[0].plot(r["_t"], unwrap_deg(r["_e_ekf"]), c=c, lw=0.9, label=f"{r['log']} EKF err")
    axes[1].plot(r["_t"], r["_bmx"], c=c, lw=0.9, label=f"{r['log']} b_m x")
    axes[1].plot(r["_t"], r["_bmy"], c=c, lw=0.9, ls="--", label=f"{r['log']} b_m y")
axes[0].axhline(0, color="gray", ls=":"); axes[0].set_ylabel("EKF yaw err [deg]")
axes[0].legend(fontsize=8); axes[0].set_title("Cross-log reproducibility")
axes[1].set_ylabel("b_m [uT]"); axes[1].set_xlabel("t [s]"); axes[1].legend(fontsize=8)
fig.tight_layout(); fig.savefig(f"{PLOTS}/compare_00_two_logs.png", dpi=130)
plt.close(fig)

for r in results:
    for k in list(r):
        if k.startswith("_"):
            del r[k]
print(json.dumps(results, indent=1, ensure_ascii=False, default=str))
