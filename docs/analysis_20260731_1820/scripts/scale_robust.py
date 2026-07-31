#!/usr/bin/env python
"""Robustness battery for the 18:20 per-channel scale estimates."""
import json
import math
import numpy as np
import pandas as pd

SP = ("/private/tmp/claude-501/-Users-ryoma-nishimura-Code-Projects-StampFly-Project-"
      "Develop-Developments-StampFly-MoCap-System-v2-StampFly-Integrated-Control-V3/"
      "ba8cbf47-5d69-4ee1-9e51-9931e40440d0/scratchpad")


def savgol(x, window, poly, deriv=0, delta=1.0):
    half = window // 2
    idx = np.arange(-half, half + 1)
    A = np.vander(idx, poly + 1, increasing=True)
    coef = np.linalg.pinv(A)[deriv] * math.factorial(deriv) / (delta ** deriv)
    xp = np.r_[x[half:0:-1], x, x[-2:-half - 2:-1]]
    return np.convolve(xp, coef[::-1], mode="valid")


def prep(path):
    df = pd.read_csv(path)
    t = df.elapsed_time.values
    tg = np.arange(t[0], t[-1], 0.02)
    fill = lambda c: pd.Series(c).ffill().bfill().values

    def sg_deriv(x, win=9):
        xg = np.interp(tg, t, fill(x))
        return np.interp(t, tg, savgol(xg, win, 2, deriv=1, delta=0.02))

    vw = sg_deriv(df.pos_x.values) + 1j * sg_deriv(df.pos_y.values)
    psi = np.deg2rad(fill(df.yaw_true_corrected_deg.values))
    F = df.tlm_flow_vx_mps.values + 1j * df.tlm_flow_vy_mps.values
    new_tlm = np.r_[True, np.diff(df.tlm_elapsed_ms.values) != 0]
    valid = ((df.tlm_flow_status.values.astype(int) & 4) > 0) & new_tlm
    Vb0 = np.conj(vw) * np.exp(-1j * psi)
    return df, t, F, Vb0, valid


def fit22(F, Vb, m):
    R0 = np.c_[Vb.real[m], Vb.imag[m]]
    F0 = np.c_[F.real[m], F.imag[m]]
    A0, *_ = np.linalg.lstsq(R0, F0, rcond=None)
    A0 = A0.T
    U, S, Vt = np.linalg.svd(A0)
    rot = U @ Vt
    ang = math.degrees(math.atan2(rot[1, 0], rot[0, 0]))
    st = rot.T @ A0
    return A0, ang, (st[0, 0], st[1, 1])   # gain ref-x (->dx ch), ref-y (->dy ch)


df, t, F, Vb0, valid = prep(SP + "/a0731b/log_corrected.csv")
LAG = 0.0377
Vl = np.interp(t - LAG, t, Vb0.real) + 1j * np.interp(t - LAG, t, Vb0.imag)
m_all = valid & (t > 5.5) & (t < 53.0)
turn = np.zeros(len(t), bool)
for a_, b_ in [(23.0, 26.5), (30.9, 33.2), (36.8, 40.2), (43.9, 47.0)]:
    turn |= (t >= a_) & (t <= b_)

out = {}
for name, mm in [
    ("all", m_all),
    ("non_turn", m_all & ~turn),
    ("turn_only", m_all & turn),
    ("first_half", m_all & (t < 29)),
    ("second_half", m_all & (t >= 29)),
    ("v>0.06", m_all & (np.abs(Vl) > 0.06)),
    ("v>0.10", m_all & (np.abs(Vl) > 0.10)),
]:
    if mm.sum() < 50:
        continue
    A0, ang, (gx, gy) = fit22(F, Vl, mm)
    out[name] = {"n": int(mm.sum()), "rot_deg": round(ang, 1),
                 "gain_dx_ch": round(gx, 3), "gain_dy_ch": round(gy, 3)}

# bootstrap on all (block bootstrap, 2s blocks)
idx = np.where(m_all)[0]
tblk = (t[idx] // 2).astype(int)
blocks = np.unique(tblk)
rng = np.random.default_rng(7)
bs = []
for _ in range(400):
    pick = rng.choice(blocks, len(blocks), replace=True)
    sel = np.concatenate([idx[tblk == b] for b in pick])
    msel = np.zeros(len(t), bool); msel[sel] = True   # note: dedup effect ok
    try:
        _, ang, (gx, gy) = fit22(F, Vl, msel)
        bs.append((ang, gx, gy))
    except np.linalg.LinAlgError:
        pass
bs = np.array(bs)
out["bootstrap_2s_blocks"] = {
    "rot_deg_ci90": [round(float(np.percentile(bs[:, 0], p)), 1) for p in (5, 95)],
    "gain_dx_ci90": [round(float(np.percentile(bs[:, 1], p)), 3) for p in (5, 95)],
    "gain_dy_ci90": [round(float(np.percentile(bs[:, 2], p)), 3) for p in (5, 95)]}

# lag sensitivity
for L in (0.0, 0.02, 0.06, 0.10):
    Vl2 = np.interp(t - L, t, Vb0.real) + 1j * np.interp(t - L, t, Vb0.imag)
    _, ang, (gx, gy) = fit22(F, Vl2, m_all)
    out[f"lag_{int(L*1000)}ms"] = {"rot_deg": round(ang, 1),
                                   "gain_dx_ch": round(gx, 3),
                                   "gain_dy_ch": round(gy, 3)}

# SG window sensitivity
for win in (15, 21):
    tg = np.arange(t[0], t[-1], 0.02)
    fill = lambda c: pd.Series(c).ffill().bfill().values
    vw = (np.interp(t, tg, savgol(np.interp(tg, t, fill(df.pos_x.values)), win, 2, 1, 0.02))
          + 1j * np.interp(t, tg, savgol(np.interp(tg, t, fill(df.pos_y.values)), win, 2, 1, 0.02)))
    psi = np.deg2rad(fill(df.yaw_true_corrected_deg.values))
    Vb0w = np.conj(vw) * np.exp(-1j * psi)
    Vlw = np.interp(t - LAG, t, Vb0w.real) + 1j * np.interp(t - LAG, t, Vb0w.imag)
    _, ang, (gx, gy) = fit22(F, Vlw, m_all)
    out[f"sgwin_{win}"] = {"rot_deg": round(ang, 1), "gain_dx_ch": round(gx, 3),
                           "gain_dy_ch": round(gy, 3)}

# ToF altitude stats both flights (v = rate*d common factor check)
alt_b = df.tlm_altitude_tof_m.values[m_all]
out["tof_alt_1820"] = {"mean": round(float(alt_b.mean()), 3),
                       "p10": round(float(np.percentile(alt_b, 10)), 3),
                       "p90": round(float(np.percentile(alt_b, 90)), 3)}
dfa, ta, Fa, Vb0a, valida = prep(SP + "/a0731/log_corrected.csv")
ma = valida & (ta > 4.5) & (ta < 53.0)
alt_a = dfa.tlm_altitude_tof_m.values[ma]
out["tof_alt_1119"] = {"mean": round(float(alt_a.mean()), 3),
                       "p10": round(float(np.percentile(alt_a, 10)), 3),
                       "p90": round(float(np.percentile(alt_a, 90)), 3)}

# 11:19 same battery quick (non-turn etc for comparability)
La = 0.0525
Vla = np.interp(ta - La, ta, Vb0a.real) + 1j * np.interp(ta - La, ta, Vb0a.imag)
for name, mm in [("1119_all", ma), ("1119_v>0.10", ma & (np.abs(Vla) > 0.10))]:
    A0, ang, (gx, gy) = fit22(Fa, Vla, mm)
    out[name] = {"n": int(mm.sum()), "rot_deg": round(ang, 1),
                 "gain_dx_ch": round(gx, 3), "gain_dy_ch": round(gy, 3)}

# SQUAL comparison
out["squal_1820"] = {"mean": round(float(df.tlm_flow_squal.values[m_all].mean()), 1)}
out["squal_1119"] = {"mean": round(float(dfa.tlm_flow_squal.values[ma].mean()), 1)}

print(json.dumps(out, indent=1))
with open(SP + "/a0731b/scale_robust.json", "w") as f:
    json.dump(out, f, indent=1)
