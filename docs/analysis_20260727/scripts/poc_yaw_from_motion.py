#!/usr/bin/env python
"""
Proof-of-concept: motion-based yaw estimation for StampFly.

Model:  a_W(t) = e^{i psi} * u_B(t) + noise
  a_W : horizontal world acceleration from MoCap position (2nd derivative, SG-smoothed)
  u_B : tilt-induced specific-force direction in the yaw-less (level body) frame,
        u_B = g * [ s1*tan(pitch), s2*tan(roll)/cos(pitch) ]  (signs resolved from data)
  psi : yaw (heading of body frame in control/world coordinates)

Estimator (window LS, no truth used):  psi_hat = arg( sum A_i * conj(U_i) )
Validation: mocap_yaw_true_deg (MoCap truth, NOT used in the estimator).
"""
import json
import math
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

G = 9.80665
FEAS = "/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V2/e51a3555-1c34-4613-ac37-eff99446eece/scratchpad/analysis/feasibility"
LOGS = {
    "20260727_164200": "/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/StampFly_MoCap_System_v2/StampFly_Integrated_Control_V2/logs/flight_logs/20260727_164200_position.csv",
    "20260727_164520": "/Users/ryoma_nishimura/Code-Projects/StampFly-Project-Develop/Developments/StampFly_MoCap_System_v2/StampFly_Integrated_Control_V2/logs/flight_logs/20260727_164520_position.csv",
}
DT = 0.02  # nominal row period (50 Hz, mean dt = 20.00 ms confirmed)


def savgol(y, win, poly, deriv, dt):
    """Manual Savitzky-Golay filter (no scipy)."""
    half = win // 2
    x = (np.arange(win) - half) * dt
    A = np.vander(x, poly + 1, increasing=True)
    pinv = np.linalg.pinv(A)
    c = pinv[deriv] * math.factorial(deriv)
    # out[i] = sum_j c[j] * y[i - half + j]  -> correlation
    out = np.convolve(y, c[::-1], mode="same")
    out[:half] = np.nan
    out[-half:] = np.nan
    return out


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def movavg(y, win):
    k = np.ones(win) / win
    out = np.convolve(y, k, mode="same")
    h = win // 2
    out[:h] = np.nan
    out[-h:] = np.nan
    return out


def analyze(name, path, sg_win=31, sg_poly=3, hp_win_s=2.0, report=None):
    df = pd.read_csv(path)
    n = len(df)
    t = np.arange(n) * DT  # uniform time base (row rate 50 Hz)

    px = df["pos_x"].to_numpy(float)
    py = df["pos_y"].to_numpy(float)
    roll = df["tlm_roll_rad"].to_numpy(float)
    pitch = df["tlm_pitch_rad"].to_numpy(float)
    psi_true = np.radians(df["mocap_yaw_true_deg"].to_numpy(float))

    # --- world horizontal acceleration from MoCap position (SG 2nd derivative)
    ax = savgol(px, sg_win, sg_poly, 2, DT)
    ay = savgol(py, sg_win, sg_poly, 2, DT)
    vx = savgol(px, sg_win, sg_poly, 1, DT)
    vy = savgol(py, sg_win, sg_poly, 1, DT)
    A = ax + 1j * ay
    V = vx + 1j * vy

    # --- tilt vector candidates, smoothed with the same SG(deriv=0) for bandwidth match
    tp = np.tan(pitch)
    tr = np.tan(roll) / np.cos(pitch)
    tp_s = savgol(tp, sg_win, sg_poly, 0, DT)
    tr_s = savgol(tr, sg_win, sg_poly, 0, DT)

    # --- convention search over the full planar orthogonal group:
    #     U = g * rot * (Z or conj(Z)),  rot in {1, i, -1, -i},  psi sign in {+1,-1}, lag scan
    hp = int(round(hp_win_s / DT)) | 1
    Ah = A - (movavg(A.real, hp) + 1j * movavg(A.imag, hp))
    Z0 = tp_s + 1j * tr_s
    best = None
    for conj in (False, True):
        Zc = np.conj(Z0) if conj else Z0
        for irot, rot in enumerate([1, 1j, -1, -1j]):
            for ps in (1, -1):
                U0 = G * rot * Zc
                for lag in range(-15, 16):
                    Ul = np.roll(U0, lag)
                    Vr = np.exp(1j * ps * psi_true) * Ul
                    Vh = Vr - (movavg(Vr.real, hp) + 1j * movavg(Vr.imag, hp))
                    m = np.isfinite(Ah) & np.isfinite(Vh)
                    if m.sum() < 100:
                        continue
                    num = np.sum(Ah[m] * np.conj(Vh[m]))
                    den = math.sqrt(np.sum(np.abs(Ah[m]) ** 2) * np.sum(np.abs(Vh[m]) ** 2))
                    coh = abs(num) / den
                    resid_ang = math.degrees(np.angle(num))
                    score = coh * math.cos(math.radians(resid_ang))  # want coherent AND near 0 deg
                    if best is None or score > best["score"]:
                        best = dict(conj=conj, rot_deg=90 * irot, ps=ps, lag=lag, coh=coh,
                                    resid_deg=resid_ang, score=score)
    ps, lag = best["ps"], best["lag"]
    rot = [1, 1j, -1, -1j][best["rot_deg"] // 90]
    Zc = np.conj(Z0) if best["conj"] else Z0
    U = np.roll(G * rot * Zc, lag)
    psi_t = ps * psi_true  # truth in estimator convention

    # --- high-passed signals used by the estimator (truth NOT used below)
    Ah = A - (movavg(A.real, hp) + 1j * movavg(A.imag, hp))
    Uh = U - (movavg(U.real, hp) + 1j * movavg(U.imag, hp))
    m = np.isfinite(Ah) & np.isfinite(Uh)

    # --- global complex gain & residual noise (uses truth, for diagnosis only)
    Vh = np.exp(1j * psi_t) * Uh
    k = np.sum(Ah[m] * np.conj(Vh[m])) / np.sum(np.abs(Vh[m]) ** 2)
    resid = Ah[m] - k * Vh[m]
    sig_a = float(np.sqrt(0.5 * (np.var(resid.real) + np.var(resid.imag))))  # per-axis
    # drag check: add velocity term  A = k*V + kd*Vel
    Velh = V - (movavg(V.real, hp) + 1j * movavg(V.imag, hp))
    m2 = m & np.isfinite(Velh)
    X = np.column_stack([Vh[m2], Velh[m2]])
    sol, *_ = np.linalg.lstsq(X, Ah[m2], rcond=None)
    resid2 = Ah[m2] - X @ sol
    sig_a2 = float(np.sqrt(0.5 * (np.var(resid2.real) + np.var(resid2.imag))))
    # residual autocorrelation -> integral time scale
    r = resid.real - resid.real.mean()
    ac = np.correlate(r, r, "full")[len(r) - 1:]
    ac = ac / ac[0]
    zero = np.argmax(ac < 0) if np.any(ac < 0) else len(ac)
    tau_int = DT * (0.5 + float(np.sum(ac[1:zero])))
    n_corr = max(1.0, 2 * tau_int / DT)

    # --- excitation stats
    exc_rms = float(np.sqrt(np.mean(np.abs(Uh[m]) ** 2)))
    tilt_hp_rms_deg = math.degrees(exc_rms / G)

    # --- windowed yaw estimation (estimator: no truth)
    results = {}
    win_list = [5.0, 10.0, 30.0]
    win_data = {}
    for Tw in win_list:
        w = int(round(Tw / DT))
        hop = int(round(1.0 / DT))
        est, tru, tc, exc, pred, est_d = [], [], [], [], [], []
        Velh = V - (movavg(V.real, hp) + 1j * movavg(V.imag, hp))
        for i0 in range(0, n - w, hop):
            sl = slice(i0, i0 + w)
            mm = np.isfinite(Ah[sl]) & np.isfinite(Uh[sl]) & np.isfinite(Velh[sl])
            if mm.sum() < w * 0.8:
                continue
            Aw, Uw = Ah[sl][mm], Uh[sl][mm]
            S = np.sum(Aw * np.conj(Uw))
            psi_hat = np.angle(S)
            E = float(np.sum(np.abs(Uw) ** 2))
            # drag-augmented estimator: A = alpha*U - lam*Vel, alpha complex, lam real
            Vw = Velh[sl][mm]
            Xr = np.zeros((2 * mm.sum(), 3))
            Xr[0::2, 0] = Uw.real; Xr[0::2, 1] = -Uw.imag; Xr[0::2, 2] = -Vw.real
            Xr[1::2, 0] = Uw.imag; Xr[1::2, 1] = Uw.real;  Xr[1::2, 2] = -Vw.imag
            yr = np.zeros(2 * mm.sum())
            yr[0::2] = Aw.real; yr[1::2] = Aw.imag
            th, *_ = np.linalg.lstsq(Xr, yr, rcond=None)
            est_d.append(math.atan2(th[1], th[0]))
            # excitation-weighted circular mean of truth
            wgt = np.abs(Uw) ** 2
            psi_bar = np.angle(np.sum(wgt * np.exp(1j * psi_t[sl][mm])))
            est.append(psi_hat)
            tru.append(psi_bar)
            tc.append(t[i0] + Tw / 2)
            exc.append(E)
            pred.append(sig_a * math.sqrt(n_corr) / math.sqrt(E))
        est, tru, exc, pred, est_d = map(np.array, (est, tru, exc, pred, est_d))
        err = np.degrees(wrap(est - tru))
        err_d = np.degrees(wrap(est_d - tru))
        med_off = float(np.median(err))
        err_c = err - med_off
        results[f"win_{int(Tw)}s"] = dict(
            n_windows=len(err),
            err_raw_deg=dict(median=float(np.median(err)), rms=float(np.sqrt(np.mean(err ** 2))),
                             p50_abs=float(np.median(np.abs(err))), p90_abs=float(np.percentile(np.abs(err), 90)),
                             max_abs=float(np.max(np.abs(err)))),
            err_offset_removed_deg=dict(rms=float(np.sqrt(np.mean(err_c ** 2))),
                                        p50_abs=float(np.median(np.abs(err_c))),
                                        p90_abs=float(np.percentile(np.abs(err_c), 90))),
            err_drag_deg=dict(median=float(np.median(err_d)), rms=float(np.sqrt(np.mean(err_d ** 2))),
                              p50_abs=float(np.median(np.abs(err_d))),
                              p90_abs=float(np.percentile(np.abs(err_d), 90))),
            frac_abs_lt_2deg=float(np.mean(np.abs(err) < 2)),
            frac_abs_lt_5deg=float(np.mean(np.abs(err) < 5)),
            frac_abs_lt_10deg=float(np.mean(np.abs(err) < 10)),
            pred_sigma_deg=dict(median=float(np.degrees(np.median(pred)))),
            excitation_sum=dict(median=float(np.median(exc)), p10=float(np.percentile(exc, 10)),
                                p90=float(np.percentile(exc, 90))),
        )
        win_data[Tw] = dict(tc=np.array(tc), est=est, tru=tru, err=err, exc=exc, pred=pred)
        np.savez(f"{FEAS}/win_{name}_{int(Tw)}s.npz", tc=np.array(tc), est=est, tru=tru,
                 err=err, err_d=err_d, exc=exc, pred=pred)

    out = dict(
        log=name, sg_win=sg_win, sg_poly=sg_poly, hp_win_s=hp_win_s,
        convention=dict(conj=bool(best["conj"]), rot_deg=best["rot_deg"], psi_sign=ps,
                        lag_rows=lag, lag_ms=lag * DT * 1e3,
                        coherence=best["coh"], residual_const_deg=best["resid_deg"]),
        gain=dict(abs=float(abs(k)), phase_deg=float(math.degrees(np.angle(k)))),
        noise=dict(sigma_a_per_axis=sig_a, sigma_a_with_drag_term=sig_a2,
                   tau_int_s=tau_int, n_corr=n_corr,
                   drag_coeff=dict(k_re=float(sol[1].real), k_im=float(sol[1].imag))),
        excitation=dict(u_hp_rms_ms2=exc_rms, tilt_hp_rms_deg=tilt_hp_rms_deg,
                        a_hp_rms_ms2=float(np.sqrt(np.mean(np.abs(Ah[m]) ** 2)))),
        windows=results,
    )
    if report is not None:
        report[name] = out

    # ---- figures
    fig, axs = plt.subplots(3, 1, figsize=(13, 9), sharex=True)
    axs[0].plot(t, Ah.real, lw=0.6, label="a_W x (MoCap 2nd deriv, HP)")
    pred_a = k * np.exp(1j * psi_t) * Uh
    axs[0].plot(t, pred_a.real, lw=0.6, alpha=0.8, label="k*R(psi_true)*u_B x (tilt model)")
    axs[0].set_ylabel("m/s$^2$"); axs[0].legend(loc="upper right", fontsize=8)
    axs[0].set_title(f"{name}: world accel vs tilt-predicted accel (coh={best['coh']:.2f}, |k|={abs(k):.2f})")
    axs[1].plot(t, Ah.imag, lw=0.6, label="a_W y")
    axs[1].plot(t, pred_a.imag, lw=0.6, alpha=0.8, label="model y")
    axs[1].set_ylabel("m/s$^2$"); axs[1].legend(loc="upper right", fontsize=8)
    axs[2].plot(t, np.abs(Uh), lw=0.6, color="tab:green")
    axs[2].set_ylabel("|u_B| HP [m/s$^2$]"); axs[2].set_xlabel("t [s]")
    fig.tight_layout(); fig.savefig(f"{FEAS}/fb_{name}_01_signals.png", dpi=110); plt.close(fig)

    fig, axs = plt.subplots(2, 1, figsize=(13, 7), sharex=True)
    axs[0].plot(t, np.degrees(psi_t), "k-", lw=1.2, label="MoCap true yaw (conv.)")
    colors = {5.0: "tab:red", 10.0: "tab:orange", 30.0: "tab:blue"}
    for Tw in win_list:
        d = win_data[Tw]
        axs[0].plot(d["tc"], np.degrees(d["est"]), ".", ms=3, color=colors[Tw], label=f"motion-yaw {int(Tw)}s win")
    axs[0].set_ylabel("yaw [deg]"); axs[0].legend(fontsize=8)
    axs[0].set_title(f"{name}: windowed motion-based yaw vs MoCap truth")
    for Tw in win_list:
        d = win_data[Tw]
        axs[1].plot(d["tc"], d["err"], ".", ms=3, color=colors[Tw], label=f"{int(Tw)}s err")
    axs[1].axhline(0, color="k", lw=0.5); axs[1].set_ylabel("error [deg]"); axs[1].set_xlabel("t [s]")
    axs[1].set_ylim(-60, 60); axs[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(f"{FEAS}/fb_{name}_02_window_yaw.png", dpi=110); plt.close(fig)

    fig, ax1 = plt.subplots(figsize=(8, 6))
    for Tw in win_list:
        d = win_data[Tw]
        ax1.loglog(np.sqrt(d["exc"]), np.abs(d["err"]) + 1e-3, ".", ms=4, color=colors[Tw], label=f"{int(Tw)}s windows")
    ee = np.logspace(-1, 1.5, 50)
    ax1.loglog(ee, np.degrees(sig_a * math.sqrt(n_corr) / ee), "k--", lw=1,
               label=r"CRLB $\sigma_a\sqrt{N_{corr}}/\sqrt{\Sigma|u|^2}$")
    ax1.set_xlabel(r"$\sqrt{\Sigma|u_{HP}|^2}$  [(m/s$^2$)$\sqrt{samples}$]")
    ax1.set_ylabel("|yaw error| [deg]"); ax1.legend(fontsize=8); ax1.grid(True, which="both", alpha=0.3)
    ax1.set_title(f"{name}: window error vs excitation")
    fig.tight_layout(); fig.savefig(f"{FEAS}/fb_{name}_03_err_vs_excitation.png", dpi=110); plt.close(fig)

    return out


def main():
    report = {}
    for name, path in LOGS.items():
        out = analyze(name, path, report=report)
        c = out["convention"]
        print(f"\n===== {name}")
        print(f" convention: conj={c['conj']} rot={c['rot_deg']}deg psi_sign={c['psi_sign']} lag={c['lag_ms']:.0f}ms "
              f"coh={c['coherence']:.3f} resid_const={c['residual_const_deg']:.1f}deg")
        print(f" gain |k|={out['gain']['abs']:.3f} phase={out['gain']['phase_deg']:.1f}deg")
        nz = out["noise"]
        print(f" sigma_a={nz['sigma_a_per_axis']:.4f} m/s2 (with drag term {nz['sigma_a_with_drag_term']:.4f}), "
              f"tau_int={nz['tau_int_s']:.3f}s n_corr={nz['n_corr']:.1f}")
        ex = out["excitation"]
        print(f" excitation: u_HP rms={ex['u_hp_rms_ms2']:.3f} m/s2 (tilt {ex['tilt_hp_rms_deg']:.2f} deg rms), "
              f"a_HP rms={ex['a_hp_rms_ms2']:.3f}")
        for wname, w in out["windows"].items():
            er, ec, ed = w["err_raw_deg"], w["err_offset_removed_deg"], w["err_drag_deg"]
            print(f" {wname}: n={w['n_windows']} raw[med={er['median']:+.1f} rms={er['rms']:.1f} "
                  f"p50|e|={er['p50_abs']:.1f} p90|e|={er['p90_abs']:.1f}] "
                  f"offrem[rms={ec['rms']:.1f} p50={ec['p50_abs']:.1f} p90={ec['p90_abs']:.1f}] "
                  f"drag[med={ed['median']:+.1f} p50={ed['p50_abs']:.1f} p90={ed['p90_abs']:.1f}] "
                  f"frac<5deg={w['frac_abs_lt_5deg']:.2f} <10deg={w['frac_abs_lt_10deg']:.2f} "
                  f"predsig={w['pred_sigma_deg']['median']:.1f}deg")
    with open(f"{FEAS}/poc_results.json", "w") as f:
        json.dump(report, f, indent=1)
    print(f"\nsaved {FEAS}/poc_results.json")


if __name__ == "__main__":
    main()
