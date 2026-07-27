#!/usr/bin/env python
"""
Task 3: required deliberate motion for target yaw accuracy, from measured noise levels.
Task 4: empirical check that window yaw error scales as 1/sqrt(excitation)  (hover
        observability limit), by binning the PoC window results by excitation.

Noise model (calibrated in poc_yaw_from_motion.py):
   sigma_psi = sigma_a * sqrt(2*tau_int / T) / u_rms      [rad]
   u_rms = g*tan(alpha)         (constant-tilt maneuver)
   u_rms = g*tan(alpha)/sqrt2   (sinusoidal tilt oscillation, amplitude alpha)
Position amplitude of a sinusoidal maneuver at freq f:  x_amp = g*tan(alpha)/(2*pi*f)^2
"""
import json
import math
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

G = 9.80665
FEAS = "/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V2/e51a3555-1c34-4613-ac37-eff99446eece/scratchpad/analysis/feasibility"

with open(f"{FEAS}/poc_results.json") as f:
    res = json.load(f)

# --- noise parameters from both logs (use worst/best range)
params = []
for name, r in res.items():
    params.append((name, r["noise"]["sigma_a_per_axis"], r["noise"]["tau_int_s"]))
    print(f"{name}: sigma_a={r['noise']['sigma_a_per_axis']:.4f} m/s2, tau_int={r['noise']['tau_int_s']:.3f} s")
sig_lo = min(p[1] for p in params)
sig_hi = max(p[1] for p in params)
tau = float(np.mean([p[2] for p in params]))
print(f"-> sigma_a range [{sig_lo:.4f},{sig_hi:.4f}], tau_int={tau:.3f}s\n")


def sigma_psi_deg(sig_a, tau, T, alpha_deg, sinus=False):
    u = G * math.tan(math.radians(alpha_deg))
    if sinus:
        u /= math.sqrt(2)
    return math.degrees(sig_a * math.sqrt(2 * tau / T) / u)


print("== Constant-tilt (accelerating translation) maneuver: predicted yaw sigma [deg] ==")
print("tilt_deg  T=2s          T=5s          T=10s        (range over both logs' noise)")
for a in [0.5, 1, 2, 3, 5, 10]:
    row = f"{a:7.1f} "
    for T in [2, 5, 10]:
        lo = sigma_psi_deg(sig_lo, tau, T, a)
        hi = sigma_psi_deg(sig_hi, tau, T, a)
        row += f"  {lo:4.1f}-{hi:4.1f}   "
    # kinematics if held one-way for T=2s
    acc = G * math.tan(math.radians(a))
    print(row + f" | a={acc:.2f} m/s2 (2s one-way: v_end={acc*2:.1f} m/s, d={0.5*acc*4:.1f} m)")

print("\n== Sinusoidal wiggle maneuver (stay-in-place): predicted yaw sigma [deg] ==")
print("tilt_amp  f=0.5Hz: x_amp     sigma(T=5s)   sigma(T=10s)   | f=1.0Hz: x_amp  sigma(T=5s)")
for a in [1, 2, 3, 5]:
    acc = G * math.tan(math.radians(a))
    row = f"{a:7.1f} "
    for f_hz in [0.5, 1.0]:
        xa = acc / (2 * math.pi * f_hz) ** 2
        s5lo = sigma_psi_deg(sig_lo, tau, 5, a, sinus=True)
        s5hi = sigma_psi_deg(sig_hi, tau, 5, a, sinus=True)
        s10lo = sigma_psi_deg(sig_lo, tau, 10, a, sinus=True)
        s10hi = sigma_psi_deg(sig_hi, tau, 10, a, sinus=True)
        if f_hz == 0.5:
            row += f"  x_amp={xa*100:5.1f}cm  {s5lo:4.1f}-{s5hi:4.1f}deg  {s10lo:4.1f}-{s10hi:4.1f}deg   |"
        else:
            row += f"  x_amp={xa*100:5.1f}cm  {s5lo:4.1f}-{s5hi:4.1f}deg"
    print(row)

print("\n== Required tilt for target accuracy (sinusoidal, T=5s, worst-case noise) ==")
for target in [2.0, 5.0]:
    # sigma = sig_hi*sqrt(2tau/T)/(g tan a /sqrt2)  -> tan a = sig_hi*sqrt(2tau/T)*sqrt2/(g*sigma_rad)
    ta = sig_hi * math.sqrt(2 * tau / 5) * math.sqrt(2) / (G * math.radians(target))
    a = math.degrees(math.atan(ta))
    xa = G * ta / (2 * math.pi * 0.5) ** 2
    print(f" target {target:.0f} deg: tilt_amp={a:.2f} deg, pos_amp(0.5Hz)={xa*100:.1f} cm")

# ---- Task 4: empirical error vs excitation binning
print("\n== Empirical: |yaw err| binned by window excitation (5s and 10s windows, both logs) ==")
for Tw in [5, 10]:
    errs, excs = [], []
    for name in res:
        d = np.load(f"{FEAS}/win_{name}_{Tw}s.npz")
        errs.append(d["err"]); excs.append(d["exc"])
    err = np.abs(np.concatenate(errs)); exc = np.concatenate(excs)
    q = np.percentile(exc, [0, 25, 50, 75, 100])
    print(f" T={Tw}s windows (n={len(err)}): excitation quartiles -> median|err|, p90|err|")
    for i in range(4):
        m = (exc >= q[i]) & (exc <= q[i + 1])
        u_rms_equiv = math.degrees(math.atan(math.sqrt(np.median(exc[m]) / (Tw / 0.02)) / G))
        print(f"   Q{i+1} (exc {q[i]:6.2f}-{q[i+1]:6.2f}, ~tilt_rms {u_rms_equiv:.2f}deg): "
              f"med={np.median(err[m]):5.1f} p90={np.percentile(err[m],90):5.1f} deg")

# ---- figure: predicted sigma vs tilt for several T + empirical overlay
fig, ax = plt.subplots(figsize=(8, 6))
til = np.linspace(0.2, 10, 200)
for T, col in [(2, "tab:red"), (5, "tab:orange"), (10, "tab:blue"), (30, "tab:green")]:
    s = [sigma_psi_deg(sig_hi, tau, T, a, sinus=True) for a in til]
    ax.plot(til, s, color=col, label=f"T={T}s (sinusoidal, worst noise)")
ax.axhline(2, color="k", ls=":", lw=1); ax.axhline(5, color="k", ls="--", lw=1)
ax.text(8.5, 2.1, "2 deg", fontsize=8); ax.text(8.5, 5.2, "5 deg", fontsize=8)
# empirical hover points: tilt_hp_rms from each log with 5/10/30s windows median err
for name, mk in zip(res, ["o", "s"]):
    tilt_rms = res[name]["excitation"]["tilt_hp_rms_deg"]
    for Tw, col in [(5, "tab:orange"), (10, "tab:blue"), (30, "tab:green")]:
        w = res[name]["windows"][f"win_{Tw}s"]
        ax.plot(tilt_rms * math.sqrt(2), w["err_raw_deg"]["p50_abs"], mk, color=col, ms=8,
                mec="k", mew=0.5)
ax.set_xlabel("tilt amplitude [deg]  (markers: hover data, x=tilt_rms*sqrt2)")
ax.set_ylabel("predicted / observed median yaw error [deg]")
ax.set_yscale("log"); ax.set_ylim(0.3, 40); ax.grid(True, which="both", alpha=0.3)
ax.legend(fontsize=8)
ax.set_title("Motion-based yaw: accuracy vs deliberate tilt excitation")
fig.tight_layout(); fig.savefig(f"{FEAS}/fb_04_required_motion.png", dpi=110)
print(f"\nsaved {FEAS}/fb_04_required_motion.png")
