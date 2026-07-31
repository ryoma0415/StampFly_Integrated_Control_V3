#!/usr/bin/env python3
"""ekf2_replay の受入テスト (pytest不要、.venv/bin/python で直接実行).

  ① EKF2 単体: ヨー擬似観測ゲート (|y|>30°棄却・連続25回で融合停止)、
     受理時の Δψ クランプ、τ_bm 適応 (RW/準凍結) の状態遷移。
  ② v6 パイプライン (ヨー観測なし): 磁気バイアス注入で EKF1 相当挙動
     (発散せず、ゲート受理、ψ 有限)。
  ③ v6 パイプライン (mocap ヨー観測): ψ が基準を追従 (RMS<2°) し、
     b_m が注入バイアスへ収束 (可観測化 — 契約 §2.1(1) の本体)。
  ④ FFプロファイル再構成: LUT ΔB̂ を正しく引いて補正 (b_m≈0 のまま)。
  ⑤ 飛行状態再アンカー (契約 §2.1(3)): 発動条件成立で B0f 再取得、
     以後 b_m≈0・status bit2。
  ⑥ v5 制限モード: 合成観測+受理マスクで EKF1 ログ軌跡を再現 (RMS<3°)。
  ⑦ ヨー観測ソフト再捕捉 (FF_EKF2_YAW_RECAPTURE_*): 誤基準でラッチ →
     正解化 → 段階2(12観測連続) → 段階3(制限融合 bit7) → 2s 継続で完全解除。
  ⑧ 再捕捉の誤再入防止: 鏡像基準 (sent=−ψ) は回頭中 d(innov)/dt≈2ψ̇ が
     ドリフトゲートで遮断され、静止中はゲート外のまま再入しない。
  ⑨ v6 パイプライン再捕捉: 90°オフセット基準 → t=15s 正解化 → bit7 →
     通常融合復帰 (FLIGHT_ANALYSIS_20260731 の 18:20 飛行シナリオ相当)。

実行: cd data_analysis && .venv/bin/python tests/test_ekf2_replay.py
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
DATA_ANALYSIS = HERE.parent
sys.path.insert(0, str(DATA_ANALYSIS))

import numpy as np  # noqa: E402

from replay import ekf2_replay as rep  # noqa: E402

DEG = math.pi / 180.0
_fail_count = 0


def _check(cond: bool, msg: str) -> None:
    global _fail_count
    if not cond:
        _fail_count += 1
        print(f"  FAIL: {msg}")


# ============================================================ ① 単体 =========
def test_yaw_obs_unit() -> None:
    print("[①] ヨー擬似観測: ゲート・融合停止・Δψクランプ・τ適応")
    kf = rep.YawEstimatorKf2Replay()
    kf.reanchor(0.0, 28.0, 0.0, [28.0, 0.0, -35.0])

    # τ適応: 初期はヨー観測なし → 準凍結側 (status bit3=0)
    kf.predict(0.0, 0.0, 0.0, 0.0, 0.01)
    _check((kf.status_bits() & 0x08) == 0, "観測前から tau_rw_mode が立っている")

    # |y|>30° は棄却
    ok = kf.update_yaw_obs(40.0 * DEG, low_trust=False)
    _check(not ok, "40°イノベーションが受理された")
    _check(kf.yaw_obs_reject_count == 1, "棄却カウンタが増えていない")
    _check(abs(kf.yaw_obs_innov - 40.0 * DEG) < 1e-9,
           "棄却時もイノベーション値を保持するはず")

    # 受理: Δψ は ±3°/更新にクランプ (P0=(10°)² → K≈0.96 で 3° 超になる条件)
    ok = kf.update_yaw_obs(20.0 * DEG, low_trust=False)
    _check(ok, "20°イノベーションが棄却された")
    _check(kf.yaw_obs_reject_count == 0, "受理でカウンタがリセットされない")
    _check(kf.x[0] <= 3.0 * DEG + 1e-9,
           f"Δψ クランプ 3° 超え: {math.degrees(kf.x[0]):.2f}°")
    _check((kf.status_bits() & 0x08) != 0, "受理後 tau_rw_mode が立たない")
    _check((kf.status_bits() & 0x02) != 0, "受理後 yaw_obs_fused が立たない")

    # 連続棄却 25 回で融合停止フラグ
    kf2 = rep.YawEstimatorKf2Replay()
    kf2.reanchor(0.0, 28.0, 0.0, [28.0, 0.0, -35.0])
    for _ in range(rep.FF_EKF2_YAW_GATE_STOP_COUNT):
        kf2.update_yaw_obs(35.0 * DEG, low_trust=False)
    _check(kf2.yaw_obs_stopped, "連続25棄却で融合停止フラグが立たない")
    psi_before = kf2.x[0]
    kf2.update_yaw_obs(1.0 * DEG, low_trust=False)
    _check(kf2.x[0] == psi_before, "融合停止後に状態が動いた")

    # クランプ発動時の制限付き更新 (kf2.cpp updateYawObs): P が膨張+stale 相関
    # ありでゲート内の大イノベーションが来ても、バイアスへ直撃せず P00 も
    # 「全補正済み」に収縮しない (実効ゲイン K0_eff=Δψ_clamp/y)。
    kf3 = rep.YawEstimatorKf2Replay()
    kf3.reanchor(0.0, 28.0, 0.0, [28.0, 0.0, -35.0])
    kf3.P[0][0] = (30.0 * DEG) ** 2  # reject-inflation 相当の膨張
    corr = 0.5 * math.sqrt(kf3.P[0][0] * kf3.P[2][2])  # 磁気更新由来の stale 相関
    kf3.P[0][2] = kf3.P[2][0] = corr
    p00_before = kf3.P[0][0]
    p22_before = kf3.P[2][2]
    ok = kf3.update_yaw_obs(25.0 * DEG, low_trust=False)
    _check(ok, "クランプ発動更新が棄却された")
    _check(abs(kf3.x[0]) <= 3.0 * DEG + 1e-9,
           f"クランプ発動時 Δψ 3° 超え: {math.degrees(kf3.x[0]):.2f}°")
    _check(kf3.x[2] == 0.0 and kf3.x[1] == 0.0,
           f"クランプ発動時にバイアスが動いた: b_g={kf3.x[1]:.3e} "
           f"b_mx={kf3.x[2]:.3e}")
    _check(kf3.P[2][2] == p22_before, "クランプ発動時に P_bm が収縮した")
    _check(kf3.P[0][0] > 0.5 * p00_before,
           f"クランプ発動時に P00 が過収縮: {kf3.P[0][0] / p00_before:.3f}")

    # τ適応: 受理後 1.0s 経過で準凍結へ戻る
    for _ in range(120):
        kf.predict(0.0, 0.0, 0.0, 0.0, 0.01)
    _check((kf.status_bits() & 0x08) == 0, "受理後1.2sで準凍結へ戻らない")


# ============================================================ 合成 v6 ログ ====
B0 = np.array([25.0, -10.0, -35.0])
PSI_TRUE = 0.3  # ファーム座標系の真ヨー (一定)


def make_synth_v6(*, bias=(5.0, -3.0, 0.0), duration_s=40.0,
                  lut_db=None, seed=3) -> dict:
    """定点ホバの v6 相当ログ: 離陸で磁気バイアス(FF残差相当)がステップ注入。

    lut_db を与えると ΔB̂(I) 汚染を b_cal に加える (FFプロファイル再構成の検証用)。
    """
    rng = np.random.default_rng(seed)
    hz = 25.0
    n = int(duration_s * hz)
    t = np.arange(n) / hz
    flying = t >= 3.0
    data: dict = {"_n": n, "elapsed_time": t,
                  "tlm_elapsed_ms": np.round(t * 1000.0)}
    data["tlm_flags"] = np.where(flying, 4.0, 0.0)
    data["tlm_roll_rad"] = np.zeros(n)
    data["tlm_pitch_rad"] = np.zeros(n)
    data["tlm_r_rad_s"] = np.zeros(n)
    data["tlm_p_rad_s"] = np.zeros(n)
    data["tlm_current_a"] = np.where(flying, 3.2, 0.15)
    data["tlm_yaw_est_rad"] = np.full(n, PSI_TRUE)
    data["tlm_bm_x_ut"] = np.zeros(n)
    data["tlm_bm_y_ut"] = np.zeros(n)
    data["tlm_ffg"] = np.zeros(n)
    data["tlm_ff_status"] = np.full(n, 1.0)  # ff_mode=1 (方式A)
    data["tlm_altitude_est_m"] = np.where(flying, 0.3, 0.0)
    data["tlm_alt_ref_m"] = np.full(n, 0.3)
    for c in ("tlm_duty_fl", "tlm_duty_fr", "tlm_duty_rl", "tlm_duty_rr"):
        data[c] = np.where(flying, 0.55, 0.0)
    data["mocap_yaw_true_deg"] = np.full(n, 10.0)  # mocap系は任意の定数オフセット
    data["tlm_db_hat_x_ut"] = np.zeros(n)
    data["tlm_db_hat_y_ut"] = np.zeros(n)

    # b_cal: B0 + 飛行中バイアス + (任意) ΔB̂(I) 汚染。10Hz 相当の fresh 更新
    # (2〜3フレームのホールドで値を保持し、fresh 検出を実機同様に働かせる)。
    mag = np.zeros((n, 3))
    cur_val = None
    for k in range(n):
        if k % 3 == 0 or cur_val is None:
            v = B0.copy()
            if flying[k]:
                v += np.asarray(bias, float)
            if lut_db is not None:
                # LUT線形: ΔB̂ = lut_db · (I − i_idle)/(3.2 − i_idle)
                frac = (data["tlm_current_a"][k] - 0.15) / (3.2 - 0.15)
                v += np.asarray(lut_db, float) * frac
            v = v + rng.normal(0, 0.05, 3)
            cur_val = v
        mag[k] = cur_val
    data["tlm_mag_cal_x_ut"] = mag[:, 0]
    data["tlm_mag_cal_y_ut"] = mag[:, 1]
    data["tlm_mag_cal_z_ut"] = mag[:, 2]
    return data


def make_ff_profile(lut_db) -> dict:
    """②/④用の最小 stampfly_ff_profile: LUT 2点 (idle→3.2A)、差動項なし。"""
    zero = {"c2": 0.0, "c1": 0.0, "c0": 0.0}
    return {
        "schema": "stampfly_ff_profile", "version": 1, "name": "synth",
        "method_a": {"lut": {"i_idle_a": 0.15, "points": [
            {"i_a": 0.15, "db": [0.0, 0.0, 0.0]},
            {"i_a": 3.2, "db": list(lut_db)},
        ]}},
        "method_b": {
            "a_tilde": {m: [0.0, 0.0, 0.0] for m in ("FL", "FR", "RL", "RR")},
            "duty_to_current": {m: dict(zero) for m in ("FL", "FR", "RL", "RR")},
        },
    }


# ============================================================ ②〜⑤ ==========
def test_v6_no_yaw_obs() -> None:
    print("[②] v6 ヨー観測なし: EKF1 相当挙動 (発散なし・受理継続)")
    data = make_synth_v6()
    res = rep.run_replay(data, yaw_obs_mode="none", enable_flight_anchor=False)
    _check(res["mode"] == "v6-full", f"モード {res['mode']} ≠ v6-full")
    psi = res["psi_ekf2_rad"]
    _check(np.isfinite(psi).all(), "ψ に非有限値")
    fly = res["flying"]
    err_deg = np.degrees(np.abs(rep._wrap_arr(psi[fly] - PSI_TRUE)))
    print(f"  最終ψ誤差 {err_deg[-1]:.2f}° / 最大 {err_deg.max():.2f}° "
          f"(バイアス吸収による誤差は許容)")
    _check(err_deg.max() < 30.0, f"ψ誤差 {err_deg.max():.1f}° が異常に大きい")
    _check(res["stats"]["gate_reject_ratio"] < 0.5,
           f"棄却率 {res['stats']['gate_reject_ratio']:.2f} が高すぎる")


def test_v6_with_yaw_obs() -> None:
    print("[③] v6 mocapヨー観測: ψ追従 + b_m 可観測化 (注入バイアス回復)")
    bias = (5.0, -3.0, 0.0)
    data = make_synth_v6(bias=bias)
    res = rep.run_replay(data, yaw_obs_mode="mocap", enable_flight_anchor=False)
    fly = res["flying"]
    psi = res["psi_ekf2_rad"]
    err_deg = np.degrees(np.abs(rep._wrap_arr(psi[fly] - PSI_TRUE)))
    rms = float(np.sqrt(np.mean(err_deg ** 2)))
    print(f"  ψ RMS {rms:.2f}° (目標 <2°)")
    _check(rms < 2.0, f"ψ RMS {rms:.2f}° ≥ 2°")
    bx = res["bm_x_ut"][fly][-1]
    by = res["bm_y_ut"][fly][-1]
    print(f"  b_m 最終 [{bx:+.2f}, {by:+.2f}] µT (注入 [{bias[0]:+.1f}, {bias[1]:+.1f}])")
    _check(abs(bx - bias[0]) < 1.5, f"b_mx {bx:.2f} が注入値 {bias[0]} へ収束せず")
    _check(abs(by - bias[1]) < 1.5, f"b_my {by:.2f} が注入値 {bias[1]} へ収束せず")


def test_v6_ff_profile_reconstruction() -> None:
    print("[④] FFプロファイル再構成: LUT ΔB̂ 補正で b_m≈0")
    lut_db = (12.0, -6.0, 3.0)
    data = make_synth_v6(bias=(0.0, 0.0, 0.0), lut_db=lut_db)
    prof = make_ff_profile(lut_db)
    res = rep.run_replay(data, ff_profile=prof, yaw_obs_mode="mocap",
                         enable_flight_anchor=False)
    fly = res["flying"]
    bx = res["bm_x_ut"][fly][-1]
    by = res["bm_y_ut"][fly][-1]
    print(f"  b_m 最終 [{bx:+.2f}, {by:+.2f}] µT (補正後は ≈0 のはず)")
    _check(abs(bx) < 1.0 and abs(by) < 1.0,
           f"FF再構成後も b_m が残留 [{bx:.2f}, {by:.2f}]")
    # 再構成 ΔB̂ が LUT 通りか (飛行中 ≈ lut_db)
    dbx = res["db_hat_x_ut"]
    m = fly & np.isfinite(dbx)
    _check(m.any() and abs(np.nanmedian(dbx[m]) - lut_db[0]) < 0.3,
           "再構成 ΔB̂_x が LUT 値と不一致")


def test_flight_anchor() -> None:
    print("[⑤] 飛行状態再アンカー (契約 §2.1(3))")
    data = make_synth_v6(bias=(5.0, -3.0, 0.0))
    res = rep.run_replay(data, yaw_obs_mode="mocap", enable_flight_anchor=True)
    st = np.nan_to_num(res["status"], nan=0.0).astype(np.int64)
    _check((st & 0x04).any(), "flight_anchor_done (status bit2) が立たない")
    if (st & 0x04).any():
        k_anchor = int(np.argmax((st & 0x04) != 0))
        t_anchor = res["t_s"][k_anchor] - res["t_s"][0]
        print(f"  再アンカー発動 t={t_anchor:.1f}s")
        _check(t_anchor > 3.0 + rep.FF_EKF2_FLIGHT_ANCHOR_MIN_FLY_S - 0.5,
               f"発動が早すぎる (t={t_anchor:.1f}s)")
        fly = res["flying"]
        bx = res["bm_x_ut"][fly][-1]
        by = res["bm_y_ut"][fly][-1]
        print(f"  再アンカー後 b_m 最終 [{bx:+.2f}, {by:+.2f}] µT")
        _check(abs(bx) < 1.5 and abs(by) < 1.5,
               "再アンカー後も b_m が残留 (B0f が飛行場を吸収していない)")


# ============================================================ ⑥ v5 ==========
def make_synth_v5(*, duration_s=40.0, seed=11) -> dict:
    """v5 相当 (磁気列なし): EKF1 ログ状態のみを持つ合成ログ。"""
    rng = np.random.default_rng(seed)
    hz = 25.0
    n = int(duration_s * hz)
    t = np.arange(n) / hz
    flying = t >= 1.0
    yaw_est = 0.3 + 0.1 * np.sin(2 * np.pi * 0.05 * t)   # ±5.7° のゆらぎ
    bm_x = np.full(n, 2.0)
    bm_y = np.full(n, -1.0)
    # 10Hz 相当で b_m に微小ノイズ → fresh 検出を駆動
    for k in range(0, n, 3):
        bm_x[k:k + 3] = 2.0 + rng.normal(0, 0.02)
        bm_y[k:k + 3] = -1.0 + rng.normal(0, 0.02)
    data: dict = {"_n": n, "elapsed_time": t,
                  "tlm_elapsed_ms": np.round(t * 1000.0)}
    data["tlm_flags"] = np.where(flying, 4.0, 0.0)
    data["tlm_roll_rad"] = np.zeros(n)
    data["tlm_pitch_rad"] = np.zeros(n)
    data["tlm_r_rad_s"] = np.gradient(yaw_est, t)  # 予測が軌跡を追える整合レート
    data["tlm_p_rad_s"] = np.zeros(n)
    data["tlm_current_a"] = np.where(flying, 3.2, 0.15)
    data["tlm_yaw_est_rad"] = yaw_est
    data["tlm_bm_x_ut"] = bm_x
    data["tlm_bm_y_ut"] = bm_y
    data["tlm_ffg"] = np.zeros(n)
    data["tlm_ff_status"] = np.full(n, 6.0)  # ff_mode=2, est=EKF
    data["tlm_altitude_est_m"] = np.where(flying, 0.3, 0.0)
    data["tlm_alt_ref_m"] = np.full(n, 0.3)
    for c in ("tlm_duty_fl", "tlm_duty_fr", "tlm_duty_rl", "tlm_duty_rr"):
        data[c] = np.where(flying, 0.55, 0.0)
    data["mocap_yaw_true_deg"] = np.degrees(yaw_est) + 5.0
    data["tlm_db_hat_x_ut"] = np.full(n, -80.0)
    data["tlm_db_hat_y_ut"] = np.full(n, 60.0)
    return data


def test_v5_restricted() -> None:
    print("[⑥] v5 制限モード: EKF1 ログ軌跡の再現 (受入基準 #6 相当)")
    data = make_synth_v5()
    res = rep.run_replay(data, yaw_obs_mode="none", enable_flight_anchor=False)
    _check(res["mode"] == "v5-restricted", f"モード {res['mode']} ≠ v5-restricted")
    rms = res["stats"]["rms_vs_ekf1_log_deg"]
    print(f"  ψ RMS vs EKF1ログ {rms:.2f}° (目標 <3°)")
    _check(rms is not None and rms < 3.0, f"RMS {rms}° ≥ 3°")
    _check(any("v5制限モード" in w for w in res["warnings"]),
           "v5制限モードの警告が無い")


# ============================================================ ⑦〜⑨ 再捕捉 ====
def test_yaw_recapture() -> None:
    print("[⑦] ソフト再捕捉: 誤基準ラッチ→正解化→制限融合(bit7)→2s継続で解除")
    DT = 0.02  # 実効50Hz相当
    kf = rep.YawEstimatorKf2Replay()
    kf.reanchor(0.0, 28.0, 0.0, [28.0, 0.0, -35.0])

    def step(obs_rad: float) -> bool:
        kf.predict(0.0, 0.0, 0.0, 0.0, DT)
        return kf.update_yaw_obs(obs_rad, low_trust=False)

    # (a) 誤基準 (+90°) を10s継続 → 25連続棄却でラッチ、ψ は動かない
    for _ in range(int(10.0 / DT)):
        step(90.0 * DEG)
    _check(kf.yaw_obs_stopped, "誤基準10sでラッチが立たない")
    _check(not kf.yaw_recapture_active, "誤基準継続中に制限融合へ入った")
    _check(abs(kf.x[0]) < 1.0 * DEG,
           f"ラッチ中に ψ が動いた: {math.degrees(kf.x[0]):.2f}°")

    # (b) 正解化 → 段階2 (M=12 観測連続+ドリフト窓) → 段階3 (制限融合)
    n_to_active = 0
    for i in range(100):
        step(0.5 * DEG)
        if kf.yaw_recapture_active:
            n_to_active = i + 1
            break
    _check(kf.yaw_recapture_active, "正解化後に制限融合モードへ入らない")
    _check(n_to_active == rep.FF_EKF2_YAW_RECAPTURE_M,
           f"制限融合入りが {n_to_active} 観測目 "
           f"(期待 {rep.FF_EKF2_YAW_RECAPTURE_M})")
    _check((kf.status_bits() & 0x80) != 0, "制限融合中に status bit7 が立たない")
    _check(kf.yaw_obs_stopped, "制限融合中は stopped 継続のはず")

    # (c) 制限融合: バイアス行 K=0・「受理」(fused/τ適応) には数えない
    bg0, bmx0 = kf.x[1], kf.x[2]
    step(0.5 * DEG)
    _check(kf.x[1] == bg0 and kf.x[2] == bmx0, "制限融合でバイアスが動いた")
    _check((kf.status_bits() & 0x02) == 0,
           "制限融合が yaw_obs_fused (bit1) に数えられた")

    # (d) ゲート外へ出たら段階1へ戻る (5s 待機やり直し) → 再度正解化で再入
    step(80.0 * DEG)
    _check(not kf.yaw_recapture_active, "ゲート外で制限融合が解除されない")
    _check(kf.yaw_obs_stopped, "ゲート外転落でラッチまで解除された")
    for _ in range(int(2.0 / DT)):  # 5s 未満はゲート内でも再入しない
        step(0.5 * DEG)
    _check(not kf.yaw_recapture_active, "段階1の5s待機を待たず再入した")
    for _ in range(int(3.5 / DT)):
        step(0.5 * DEG)
    _check(kf.yaw_recapture_active, "5s経過+ゲート内連続で再入しない")

    # (e) ゲート内 2s 継続でラッチ完全解除 → 通常融合 (fused bit1)
    for _ in range(int(rep.FF_EKF2_YAW_RECAPTURE_HOLD_S / DT) + 5):
        step(0.5 * DEG)
    _check(not kf.yaw_obs_stopped, "制限融合2s継続後もラッチが解除されない")
    _check(not kf.yaw_recapture_active, "解除後も制限融合フラグが残留")
    step(0.5 * DEG)
    _check((kf.status_bits() & 0x02) != 0, "解除後の通常融合で fused が立たない")


def test_yaw_recapture_no_reentry_wrong_ref() -> None:
    print("[⑧] 再捕捉の誤再入防止: 鏡像基準は回頭中もドリフトゲートで遮断")
    DT = 0.02
    kf = rep.YawEstimatorKf2Replay()
    kf.reanchor(0.0, 28.0, 0.0, [28.0, 0.0, -35.0])
    # まず誤オフセット基準でラッチ (18:20 飛行のフレーム差 −90° 相当)
    for _ in range(int(6.0 / DT)):
        kf.predict(0.0, 0.0, 0.0, 0.0, DT)
        kf.update_yaw_obs(-90.0 * DEG, low_trust=False)
    _check(kf.yaw_obs_stopped, "前段: ラッチが立たない")
    # 鏡像基準 (sent=−ψ_true) のまま ψ̇=30°/s で回頭: innov=wrap(−2ψ) が
    # ゲート帯 (±30°) を 60°/s で横切る間 (≈1s=50観測 ≫ M=12) も、
    # 窓ドリフトレート≈2ψ̇=60°/s > 20°/s がドリフトゲートで遮断される
    psi_rate = 30.0 * DEG
    psi_true = 0.0
    reentered = False
    for _ in range(int(12.0 / DT)):  # 360° 回頭 = ゲート帯通過 2 回
        kf.predict(psi_rate, 0.0, 0.0, 0.0, DT)
        psi_true = rep.wrap_pi(psi_true + psi_rate * DT)
        kf.update_yaw_obs(rep.wrap_pi(-psi_true), low_trust=False)
        reentered |= kf.yaw_recapture_active
    _check(not reentered, "鏡像基準の回頭中に制限融合へ再入した")
    _check(kf.yaw_obs_stopped, "鏡像基準でラッチが解除された")
    # 静止+誤オフセット基準の継続でも再入しない (ゲート外のまま)
    for _ in range(int(6.0 / DT)):
        kf.predict(0.0, 0.0, 0.0, 0.0, DT)
        kf.update_yaw_obs(90.0 * DEG, low_trust=False)
    _check(not kf.yaw_recapture_active and kf.yaw_obs_stopped,
           "静止・誤基準継続で再入した")


def test_yaw_recapture_pipeline() -> None:
    print("[⑨] v6 パイプライン再捕捉: 誤基準→t=15s正解化→bit7→融合復帰")
    data = make_synth_v6(bias=(0.0, 0.0, 0.0), duration_s=40.0)
    t = data["elapsed_time"]
    data["yaw_ref_rad"] = np.where(t < 15.0, PSI_TRUE + 90.0 * DEG, PSI_TRUE)
    res = rep.run_replay(data, yaw_obs_mode="yaw_ref_rad",
                         enable_flight_anchor=False, align_yaw_obs=False)
    st = np.nan_to_num(res["status"], nan=0.0).astype(np.int64)
    tt = res["t_s"] - res["t_s"][0]
    pre = tt < 15.0
    _check(not ((st[pre] & 0x80) != 0).any(), "正解化前に bit7 が立った")
    mid = (tt > 5.0) & (tt < 15.0)
    _check(not ((st[mid] & 0x02) != 0).any(),
           "誤基準期間に fused (bit1) が立ち続けた (ラッチ不発)")
    post = tt >= 15.0
    _check(((st[post] & 0x80) != 0).any(), "正解化後に bit7 が現れない")
    k7 = np.where((st & 0x80) != 0)[0]
    k1 = np.where(post & ((st & 0x02) != 0))[0]
    _check(len(k1) > 0, "正解化後に通常融合が復帰しない")
    if len(k7) and len(k1):
        t7, t1 = tt[k7[0]], tt[k1[0]]
        print(f"  制限融合開始 t={t7:.1f}s → 通常融合復帰 t={t1:.1f}s")
        _check(t1 > t7 + rep.FF_EKF2_YAW_RECAPTURE_HOLD_S - 0.2,
               "ホールド時間を待たずラッチが解除された")
    tail = tt > 20.0
    fused_ratio = float(np.mean((st[tail] & 0x02) != 0))
    print(f"  復帰後 (t>20s) fused 率 {fused_ratio:.2f}")
    _check(fused_ratio > 0.9, f"復帰後 fused 率 {fused_ratio:.2f} が低い")
    fly = res["flying"]
    psi = res["psi_ekf2_rad"]
    err = np.degrees(np.abs(rep._wrap_arr(psi[fly & tail] - PSI_TRUE)))
    _check(err.size > 0 and float(np.sqrt(np.mean(err ** 2))) < 2.0,
           "復帰後の ψ が基準へ収束していない")


def main() -> int:
    test_yaw_obs_unit()
    test_v6_no_yaw_obs()
    test_v6_with_yaw_obs()
    test_v6_ff_profile_reconstruction()
    test_flight_anchor()
    test_v5_restricted()
    test_yaw_recapture()
    test_yaw_recapture_no_reentry_wrong_ref()
    test_yaw_recapture_pipeline()
    if _fail_count:
        print(f"\nNG: {_fail_count} 件の FAIL")
        return 1
    print("\nOK: 全テスト合格")
    return 0


if __name__ == "__main__":
    sys.exit(main())
