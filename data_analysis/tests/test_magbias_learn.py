#!/usr/bin/env python3
"""magbias_learn の受入テスト (pytest不要、.venv/bin/python で直接実行).

合成データ(既知の Δb を注入した v6 相当ログ)で:
  ① ベクトルフィットの回復精度: 注入 Δb を 0.3 µT 以内で回復すること
     (磁気EMA遅れ τ≈0.456s を合成側でも再現し、遅れ補償込みで検証)。
  ② ヨー励振不足(<45°)で delta_b=null に縮退すること。
  ③ FF電圧/duty回帰: 注入した電圧係数を回復し R² が高いこと。
  ④ v5縮退: v6列なしで delta_b=null + EKF1 b_m 代用 + warnings 明記。
  ⑤ プロファイルJSONが契約 §3 スキーマ(必須キー・allow_nan)を満たすこと。

実行: cd data_analysis && .venv/bin/python tests/test_magbias_learn.py
"""
from __future__ import annotations

import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA_ANALYSIS = HERE.parent
sys.path.insert(0, str(DATA_ANALYSIS))

import numpy as np  # noqa: E402

import magbias_learn as mbl  # noqa: E402

_fail_count = 0


def _check(cond: bool, msg: str) -> None:
    global _fail_count
    if not cond:
        _fail_count += 1
        print(f"  FAIL: {msg}")


# ============================================================ 合成データ ======
def make_synth_log(*, delta_b=(4.0, -6.0), b0h=(24.0, -14.0),
                   yaw_amp_deg=120.0, duration_s=60.0, hz=25.0,
                   volt_coeff=(2.5, -1.5), seed=7,
                   v5_mode=False, noise_ut=0.15) -> dict:
    """v6 相当の飛行ログ dict を合成する。

    mag_lev = EMA( R_z(ψ(t))·B0h' + Δb + ノイズ ) — EMA は実機と同じ
    α=0.18/実効10Hz(magbias_learn の遅れ補償が正しく効くことも検証する)。
    b_m 列 = Δb + 電圧・duty 依存項(ff_supplement 回帰の真値)。
    """
    rng = np.random.default_rng(seed)
    n = int(duration_s * hz)
    t = np.arange(n) / hz
    data: dict = {"_n": n, "elapsed_time": t}

    # 離陸5s後からヨー励振(±yaw_amp/2 の正弦回頭)
    yaw = np.zeros(n)
    exc = t >= 10.0
    yaw[exc] = np.radians(yaw_amp_deg / 2.0) * np.sin(
        2.0 * np.pi * 0.05 * (t[exc] - 10.0))
    data["mocap_yaw_true_deg"] = np.degrees(yaw) + rng.normal(0, 0.1, n)

    flying = (t >= 1.0)
    data["tlm_flags"] = np.where(flying, 4.0, 0.0)

    volt = 4.2 - 0.012 * t
    duty = 0.55 + 0.03 * np.sin(2 * np.pi * 0.03 * t)
    cur = np.where(flying, 3.2 + 0.5 * (duty - 0.55), 0.15)
    data["tlm_voltage_v"] = volt
    data["tlm_current_a"] = cur
    for i, c in enumerate(mbl.DUTY_COLS):
        data[c] = np.where(flying, duty + 0.01 * i, 0.0)

    # b_m 時系列 (回帰真値): Δb + cV·(V−3.8) + ノイズ
    bm_x = delta_b[0] + volt_coeff[0] * (volt - 3.8) + rng.normal(0, 0.05, n)
    bm_y = delta_b[1] + volt_coeff[1] * (volt - 3.8) + rng.normal(0, 0.05, n)

    if v5_mode:
        data["tlm_bm_x_ut"] = bm_x
        data["tlm_bm_y_ut"] = bm_y
        data["tlm_yaw_est_rad"] = yaw
        return data

    data["tlm_ekf2_bm_x_ut"] = bm_x
    data["tlm_ekf2_bm_y_ut"] = bm_y
    data["tlm_ekf2_yaw_rad"] = yaw

    # 観測 (EMA前): z = R_z(ψ)·B0h' + Δb + n
    c, s = np.cos(yaw), np.sin(yaw)
    zx_raw = c * b0h[0] - s * b0h[1] + delta_b[0] + rng.normal(0, noise_ut, n)
    zy_raw = s * b0h[0] + c * b0h[1] + delta_b[1] + rng.normal(0, noise_ut, n)
    # 実機EMA: α=0.18, 実効10Hz fresh → 25Hzログでは 10Hz 間引きで更新・ホールド
    alpha = mbl.MAG_FILTER_ALPHA
    fresh_period = int(round(hz * mbl.MAG_FRESH_PERIOD_S))  # 25Hz中 0.1s 毎
    zx = np.empty(n)
    zy = np.empty(n)
    ex, ey = zx_raw[0], zy_raw[0]
    for k in range(n):
        if k % fresh_period == 0:
            ex = alpha * zx_raw[k] + (1 - alpha) * ex
            ey = alpha * zy_raw[k] + (1 - alpha) * ey
        zx[k] = ex
        zy[k] = ey
    data[mbl.COL_MAG_LEV_X] = zx
    data[mbl.COL_MAG_LEV_Y] = zy

    # mag_cal_z: 地上→ホバで −2.0 µT の z 残差
    data[mbl.COL_MAG_CAL_Z] = np.where(flying, -38.0, -36.0) \
        + rng.normal(0, 0.05, n)
    return data


# ============================================================ ①回復精度 ======
def test_delta_b_recovery() -> None:
    print("[①] 既知 Δb 注入 → ベクトルフィット回復 (許容 0.3 µT)")
    delta_true = (4.0, -6.0)
    data = make_synth_log(delta_b=delta_true)
    profile, internals = mbl.extract_magbias(
        data, name="synth", source_log="synth.csv",
        ff_state_path=Path("/nonexistent/ff_state.json"))
    db = profile["delta_b_ut"]
    _check(db is not None, "delta_b が null (フィット失敗)")
    if db is not None:
        ex = abs(db[0] - delta_true[0])
        ey = abs(db[1] - delta_true[1])
        print(f"  delta_b = [{db[0]:+.3f}, {db[1]:+.3f}] µT "
              f"(真値 [{delta_true[0]:+.1f}, {delta_true[1]:+.1f}], "
              f"誤差 [{ex:.3f}, {ey:.3f}])")
        _check(ex < 0.3, f"Δb_x 誤差 {ex:.3f} > 0.3 µT")
        _check(ey < 0.3, f"Δb_y 誤差 {ey:.3f} > 0.3 µT")
        # z 残差: 合成値 −2.0 µT
        _check(abs(db[2] - (-2.0)) < 0.2, f"Δb_z {db[2]:+.3f} ≠ −2.0±0.2")
    q = profile["quality"]
    _check(q["yaw_excursion_deg"] > 100.0,
           f"励振量 {q['yaw_excursion_deg']:.1f}° が合成値(≈120°)と不整合")
    _check(q["fit_rms_ut"] is not None and q["fit_rms_ut"] < 1.0,
           f"fit_rms {q['fit_rms_ut']} µT が大きすぎる")
    _check(internals["log_generation"] == "v6", "v6ログと判定されていない")


# ============================================================ ②励振不足 ======
def test_insufficient_excursion() -> None:
    print("[②] ヨー励振不足 (<45°) → delta_b=null")
    data = make_synth_log(yaw_amp_deg=20.0)
    profile, _ = mbl.extract_magbias(
        data, name="synth2", source_log="synth2.csv",
        ff_state_path=Path("/nonexistent/ff_state.json"))
    _check(profile["delta_b_ut"] is None, "励振不足でも delta_b が非null")
    _check(any("励振不足" in w or "null" in w
               for w in profile["quality"]["warnings"]),
           "励振不足の警告が無い")
    hov = profile["hover_residual_ut"]["leveled_xy"]
    _check(abs(hov[0] - 4.0) < 1.0 and abs(hov[1] - (-6.0)) < 1.0,
           f"hover_residual {hov} が注入値 [4,−6] から外れている")


# ============================================================ ③FF回帰 ========
def test_ff_supplement_regression() -> None:
    print("[③] FF電圧/duty回帰: 電圧係数の回復と R²")
    vc = (2.5, -1.5)
    data = make_synth_log(volt_coeff=vc)
    profile, _ = mbl.extract_magbias(
        data, name="synth3", source_log="synth3.csv",
        ff_state_path=Path("/nonexistent/ff_state.json"))
    supp = profile["ff_supplement"]
    _check(supp is not None, "ff_supplement が null")
    if supp is not None:
        got = supp["voltage_coeff_ut_per_v"]
        print(f"  voltage_coeff = [{got[0]:+.3f}, {got[1]:+.3f}] "
              f"(真値 [{vc[0]:+.1f}, {vc[1]:+.1f}]), R²={supp['r2']:.3f}")
        # duty と電圧は完全には直交しないため許容は 0.5 µT/V
        _check(abs(got[0] - vc[0]) < 0.5, f"電圧係数x {got[0]:.3f} ≠ {vc[0]}")
        _check(abs(got[1] - vc[1]) < 0.5, f"電圧係数y {got[1]:.3f} ≠ {vc[1]}")
        _check(supp["r2"] > 0.8, f"R² {supp['r2']:.3f} < 0.8")


# ============================================================ ④v5縮退 ========
def test_v5_degraded() -> None:
    print("[④] v5縮退: delta_b=null + EKF1 b_m 代用 + warnings")
    data = make_synth_log(v5_mode=True)
    profile, internals = mbl.extract_magbias(
        data, name="synth5", source_log="synth5.csv",
        ff_state_path=Path("/nonexistent/ff_state.json"))
    _check(internals["log_generation"] == "v5", "v5と判定されていない")
    _check(profile["delta_b_ut"] is None, "v5で delta_b が非null")
    _check(internals["bm_source"] == "ekf1_bm", "EKF1 b_m 代用になっていない")
    _check(any("v5" in w for w in profile["quality"]["warnings"]),
           "v5縮退の警告が無い")
    _check(profile["hover_residual_ut"]["z"] is None, "v5で z 残差が非null")
    hov = profile["hover_residual_ut"]["leveled_xy"]
    _check(abs(hov[0] - 4.0) < 1.0 and abs(hov[1] - (-6.0)) < 1.0,
           f"v5 hover_residual {hov} が注入値から外れている")


# ============================================================ ⑤スキーマ ======
def test_schema() -> None:
    print("[⑤] プロファイルJSONスキーマ (契約 §3)")
    data = make_synth_log()
    profile, _ = mbl.extract_magbias(
        data, name="synth6", source_log="synth6.csv",
        ff_state_path=Path("/nonexistent/ff_state.json"))
    _check(profile["schema"] == "stampfly_magbias_profile", "schema 名不一致")
    _check(profile["version"] == 1, "version ≠ 1")
    for key in ("name", "created_at", "source_logs", "delta_b_ut",
                "hover_residual_ut", "ff_supplement", "binding", "quality"):
        _check(key in profile, f"必須キー {key} が無い")
    for key in ("mag3d_hash", "ffcal_crc"):
        _check(key in profile["binding"], f"binding.{key} が無い")
    for key in ("yaw_excursion_deg", "fit_rms_ut", "warnings"):
        _check(key in profile["quality"], f"quality.{key} が無い")
    hov = profile["hover_residual_ut"]
    for key in ("leveled_xy", "z", "current_a"):
        _check(key in hov, f"hover_residual_ut.{key} が無い")
    try:
        json.dumps(profile, ensure_ascii=False, allow_nan=False)
    except ValueError:
        _check(False, "JSON に NaN/Inf が混入 (allow_nan=False で失敗)")


def main() -> int:
    test_delta_b_recovery()
    test_insufficient_excursion()
    test_ff_supplement_regression()
    test_v5_degraded()
    test_schema()
    if _fail_count:
        print(f"\nNG: {_fail_count} 件の FAIL")
        return 1
    print("\nOK: 全テスト合格")
    return 0


if __name__ == "__main__":
    sys.exit(main())
