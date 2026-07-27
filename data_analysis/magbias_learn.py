#!/usr/bin/env python3
"""飛行ログ CSV → magbias プロファイル JSON (stampfly_magbias_profile v1) 抽出 CLI.

仕様: docs/MAG_AUTOTUNE_DESIGN.md §3(スキーマ)・§4(抽出方式)。
pc_server の /api/magbias extract からサブプロセスとして呼ばれるのと同じ CLI
(make_ff_profile.py と同型の呼び出し契約: 引数→JSON出力→終了コード0/非0)。

抽出内容:
  1. ハードアイアン残差 Δb: ヨー励振区間のベクトルフィット。
       z_k = R_z(ψ_k)·B0h' + Δb_lev   (ψ0 は B0h' に吸収されるため 4未知数の線形LS)
     z=(tlm_mag_lev_x/y_ut)=EKF観測(FF補正+magbias+EMA後レベル化水平)、
     ψ=mocap真値(mocap_yaw_true_deg)または tlm_ekf2_yaw_rad。
     磁気EMA遅れ(α=0.18@実効10Hz → τ≈0.456s)はヨーの時間シフトで補償する。
     ヨー励振 < 45° なら delta_b=null(hover_residual のみ)。
     レベル化水平Δb→b_cal空間3軸への変換はホバ近傍(roll/pitch≈0)近似
     (z成分はホバ/地上の mag_cal_z 差から。近似の限界は quality.warnings に明記)。
  2. ホバ点残差 hover_residual: EKF2 b_m(tlm_ekf2_bm_x/y_ut)の安定ホバ窓中央値。
  3. FF電圧/duty回帰 ff_supplement: b_m 時系列 ×(電圧, duty4本, 電流)の線形回帰+R²
     (診断・将来のFFモデル拡張用。契約 §3)。

v5ログ縮退(2026-07-27 のログ等、ekf2/mag_lev/mag_cal 列なし):
  - ベクトルフィット不能 → delta_b=null。
  - hover_residual / ff_supplement は EKF1 の tlm_bm_x/y_ut で代用(warnings 明記。
    ヨー観測なしの EKF1 b_m は一定ヘディングで不可観測なため信頼度は低い —
    FLIGHT_ANALYSIS_20260727.md §2-3)。

使い方:
  .venv/bin/python magbias_learn.py <flight_log.csv> [-o out.json] [--name NAME]
      [--plots] [--yaw-source auto|mocap|ekf2] [--ff-state <ff_state.json>]

出力: -o 省略時 ../pc_server/data/magbias_profiles/<name>.json
  name 既定: magbias_<ログstem>。--plots で品質図 PNG を graphs/magbias_<name>/ へ。

依存: numpy(必須)、matplotlib(--plots 時のみ)。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
DEFAULT_OUT_DIR = HERE.parent / "pc_server" / "data" / "magbias_profiles"
DEFAULT_FF_STATE = HERE.parent / "pc_server" / "data" / "ff_state.json"
DEFAULT_FF_PROFILE_DIR = HERE.parent / "pc_server" / "data" / "ff_profiles"
GRAPH_DIR = HERE / "graphs"

TOOL_VERSION = "1.0"

# ---- ファームウェア定数の転記(firmware_stampfly/src/yaw_estimation/yaw_config.hpp)----
MAG_FILTER_ALPHA = 0.18          # yaw_config.hpp L40 (補正系EMAと同一)
MAG_FRESH_PERIOD_S = 0.1         # BMM150 実効freshレート ~10Hz (yaw_config.hpp L30-32)
# EMA一次遅れの実効時定数: τ = Ts·(1−α)/α ≈ 0.456s
#   (FLIGHT_ANALYSIS_20260727.md §5 の「磁気EMA遅れ τ≈0.46s」と同一)
MAG_EMA_LAG_S = MAG_FRESH_PERIOD_S * (1.0 - MAG_FILTER_ALPHA) / MAG_FILTER_ALPHA

# ---- 抽出パラメータ(本ツール固有。契約 §3/§4 の方針に基づく)----
YAW_EXCURSION_MIN_DEG = 45.0     # 契約 §4: 励振<45° なら delta_b=null
FIT_SETTLE_AFTER_TAKEOFF_S = 5.0   # 離陸直後の過渡(電流ステップ・アンカー直後)を除外
HOVER_WINDOW_AFTER_TAKEOFF_S = 10.0  # b_m 中央値を取るホバ窓の開始(引き込み完了待ち)
FIT_MAX_YAW_RATE_DEG_S = 90.0    # EMA遅れ補償後も残る高速回頭サンプルの安全除外
TLM_FLAG_FLYING = 1 << 2         # protocol TlmState.FLAG_FLYING

# v6 ログ列名(pc_server ロガー v6 = TLM_STATE フィールド名 + prefix "tlm_"。
# 契約 §3.3。ロガー側の命名が変わった場合はここだけ直す)
COL_MAG_LEV_X = "tlm_mag_lev_x_ut"
COL_MAG_LEV_Y = "tlm_mag_lev_y_ut"
COL_MAG_CAL_Z = "tlm_mag_cal_z_ut"
COL_EKF2_YAW = "tlm_ekf2_yaw_rad"
COL_EKF2_BM_X = "tlm_ekf2_bm_x_ut"
COL_EKF2_BM_Y = "tlm_ekf2_bm_y_ut"
# v5 縮退時の代用列(EKF1)
COL_EKF1_BM_X = "tlm_bm_x_ut"
COL_EKF1_BM_Y = "tlm_bm_y_ut"

DUTY_COLS = ("tlm_duty_fl", "tlm_duty_fr", "tlm_duty_rl", "tlm_duty_rr")  # FL,FR,RL,RR 順


# ============================================================ ログ読み込み ====
def _f(v) -> float:
    try:
        return float(v)
    except (TypeError, ValueError):
        return math.nan


def load_flight_log(path: Path) -> dict:
    """飛行ログ CSV → {列名: np.ndarray} (数値化できない列は object)。"""
    with path.open(encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    if not rows:
        raise ValueError(f"空のCSVです: {path}")
    out: dict = {}
    for key in rows[0].keys():
        if key is None:
            continue
        vals = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
        if not np.isfinite(vals).any():
            # 全行が数値化不能(timestamp/mode/phase 等の文字列列)
            out[key] = np.array([r.get(key) or "" for r in rows], dtype=object)
        else:
            out[key] = vals
    out["_n"] = len(rows)
    return out


def col(data: dict, name: str) -> np.ndarray:
    """列を取得。無ければ全 NaN。"""
    v = data.get(name)
    if v is None or getattr(v, "dtype", None) == object:
        return np.full(data["_n"], np.nan)
    return v


def has_col(data: dict, name: str) -> bool:
    return np.isfinite(col(data, name)).any()


# ============================================================ ユーティリティ ==
def wrap_pi(a: np.ndarray) -> np.ndarray:
    return (np.asarray(a) + np.pi) % (2.0 * np.pi) - np.pi


def unwrap_finite(a: np.ndarray) -> np.ndarray:
    """NaN を保持したまま有限区間を unwrap。"""
    a = np.asarray(a, float)
    out = np.full_like(a, np.nan)
    f = np.isfinite(a)
    if f.sum() >= 2:
        out[f] = np.unwrap(a[f])
    elif f.any():
        out[f] = a[f]
    return out


def _finite_median(a: np.ndarray):
    a = np.asarray(a, float)
    f = np.isfinite(a)
    return float(np.median(a[f])) if f.any() else None


# ============================================================ 抽出コア =======
def fit_hard_iron(t_s: np.ndarray, yaw_rad: np.ndarray,
                  zx: np.ndarray, zy: np.ndarray,
                  mask: np.ndarray,
                  ema_lag_s: float = MAG_EMA_LAG_S) -> dict:
    """ヨー励振区間のベクトルフィット (契約 §4)。

    z_k = R_z(ψ_k)·B0h' + Δb を [B0h'_x, B0h'_y, Δb_x, Δb_y] の線形LSで解く
    (ψ0 は B0h' = R_z(−ψ0)·B0h に吸収される)。磁気EMA遅れは ψ を
    t−ema_lag_s で評価して補償する。戻り値 dict:
      ok, n, dx, dy, b0h_x, b0h_y, rms_ut, yaw_excursion_deg, reason
    """
    t_s = np.asarray(t_s, float)
    yaw_u = unwrap_finite(yaw_rad)
    valid = mask & np.isfinite(t_s) & np.isfinite(yaw_u) & \
        np.isfinite(zx) & np.isfinite(zy)
    res = {"ok": False, "n": int(valid.sum()), "dx": None, "dy": None,
           "b0h_x": None, "b0h_y": None, "rms_ut": None,
           "yaw_excursion_deg": 0.0, "reason": ""}
    if valid.sum() < 20:
        res["reason"] = f"有効サンプル不足 ({int(valid.sum())} < 20)"
        return res

    tv = t_s[valid]
    # EMA遅れ補償: 観測 z(t) は ψ(t−τ) の場に対応する → ψ を過去へシフト評価
    yaw_lag = np.interp(tv - ema_lag_s, tv, yaw_u[valid])
    # 高速回頭サンプルの安全除外(一次遅れ補償の残差が大きい領域)
    if len(tv) >= 3:
        rate = np.gradient(yaw_lag, tv, edge_order=1)
        keep = np.abs(np.degrees(rate)) <= FIT_MAX_YAW_RATE_DEG_S
    else:
        keep = np.ones(len(tv), bool)
    if keep.sum() < 20:
        res["reason"] = "回頭レート除外後のサンプル不足"
        return res
    yaw_fit = yaw_lag[keep]
    zx_fit = zx[valid][keep]
    zy_fit = zy[valid][keep]

    excursion_deg = float(np.degrees(yaw_fit.max() - yaw_fit.min()))
    res["yaw_excursion_deg"] = excursion_deg
    if excursion_deg < YAW_EXCURSION_MIN_DEG:
        res["reason"] = (f"ヨー励振不足 ({excursion_deg:.1f}° < "
                         f"{YAW_EXCURSION_MIN_DEG:.0f}°)")
        return res

    c = np.cos(yaw_fit)
    s = np.sin(yaw_fit)
    n = len(yaw_fit)
    # [zx; zy] = [[c,−s,1,0],[s,c,0,1]]·[Bx,By,dx,dy]
    A = np.zeros((2 * n, 4))
    A[0::2, 0] = c
    A[0::2, 1] = -s
    A[0::2, 2] = 1.0
    A[1::2, 0] = s
    A[1::2, 1] = c
    A[1::2, 3] = 1.0
    b = np.empty(2 * n)
    b[0::2] = zx_fit
    b[1::2] = zy_fit
    sol, *_ = np.linalg.lstsq(A, b, rcond=None)
    resid = A @ sol - b
    res.update(ok=True, n=n,
               b0h_x=float(sol[0]), b0h_y=float(sol[1]),
               dx=float(sol[2]), dy=float(sol[3]),
               rms_ut=float(np.sqrt(np.mean(resid ** 2))))
    return res


def regress_ff_supplement(bm_x: np.ndarray, bm_y: np.ndarray,
                          voltage_v: np.ndarray, duty4: list[np.ndarray],
                          current_a: np.ndarray, mask: np.ndarray) -> dict | None:
    """FF電圧/duty回帰 (契約 §3): b_m 時系列 ×(V, duty4本, I)の線形回帰+R²。

    軸別(x,y)に b_m = c0 + cV·V + Σ cD_m·duty_m + cI·I を最小二乗。
    duty 順は FL,FR,RL,RR(CMD_FF_MOT の idx と同順)。
    戻り値は契約 §3 の ff_supplement dict(+補足キー)。フィット不能なら None。
    """
    feats = [voltage_v] + list(duty4) + [current_a]
    valid = mask & np.isfinite(bm_x) & np.isfinite(bm_y)
    for f in feats:
        valid &= np.isfinite(f)
    n = int(valid.sum())
    if n < 30:
        return None
    A = np.column_stack([np.ones(n)] + [np.asarray(f, float)[valid] for f in feats])
    out = {"voltage_coeff_ut_per_v": [], "duty_coeff": [], "r2": None,
           "current_coeff_ut_per_a": [], "intercept_ut": [], "r2_xy": [],
           "n_samples": n}
    ss_res_sum = 0.0
    ss_tot_sum = 0.0
    for target in (bm_x, bm_y):
        y = np.asarray(target, float)[valid]
        sol, *_ = np.linalg.lstsq(A, y, rcond=None)
        yhat = A @ sol
        ss_res = float(np.sum((y - yhat) ** 2))
        ss_tot = float(np.sum((y - np.mean(y)) ** 2))
        r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
        out["intercept_ut"].append(float(sol[0]))
        out["voltage_coeff_ut_per_v"].append(float(sol[1]))
        out["duty_coeff"].append([float(v) for v in sol[2:6]])
        out["current_coeff_ut_per_a"].append(float(sol[6]))
        out["r2_xy"].append(float(r2))
        ss_res_sum += ss_res
        ss_tot_sum += ss_tot
    # 契約の "r2" はスカラー: x/y プールした決定係数
    out["r2"] = float(1.0 - ss_res_sum / ss_tot_sum) if ss_tot_sum > 0 else 0.0
    out["duty_order"] = ["FL", "FR", "RL", "RR"]
    return out


def _load_binding(ff_state_path: Path, warnings: list[str]) -> dict:
    """依存パラメータ束縛 (契約 §3): 適用中 ffcal の CRC と mag3d ハッシュ。

    飛行ログ自体には mag3d/ffcal が載らないため、pc_server の ff_state.json
    (適用状態)と適用プロファイルJSONの binding から best-effort で引く。
    取れない場合は null(適用時の照合は pc_server 側の責務)。
    """
    binding = {"mag3d_hash": None, "ffcal_crc": None}
    try:
        state = json.loads(ff_state_path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        warnings.append(f"ff_state.json が読めないため binding は null ({ff_state_path.name})")
        return binding
    applied = state.get("applied") or {}
    crc = applied.get("crc")
    if isinstance(crc, str) and crc:
        binding["ffcal_crc"] = crc
    name = applied.get("name")
    if name:
        prof_path = DEFAULT_FF_PROFILE_DIR / f"{name}.json"
        try:
            prof = json.loads(prof_path.read_text(encoding="utf-8"))
            h = (prof.get("binding") or {}).get("mag3d_hash")
            if isinstance(h, str):
                binding["mag3d_hash"] = h
        except (OSError, json.JSONDecodeError):
            warnings.append(f"適用プロファイル {name}.json が読めず mag3d_hash は null")
    if binding["ffcal_crc"] is None:
        warnings.append("ffcal 適用状態が無いため binding.ffcal_crc は null")
    return binding


def extract_magbias(data: dict, name: str, source_log: str,
                    yaw_source: str = "auto",
                    ff_state_path: Path | None = None) -> tuple[dict, dict]:
    """飛行ログ dict → (プロファイル dict, 内部量 dict)。

    v6 列があればベクトルフィット+EKF2 b_m、無ければ v5 縮退
    (delta_b=null、EKF1 tlm_bm 代用、warnings 明記)。
    """
    warnings: list[str] = []
    internals: dict = {}
    n = data["_n"]
    t = col(data, "elapsed_time")
    if not np.isfinite(t).any():
        raise ValueError("elapsed_time 列がありません(飛行ログではない?)")

    flags = col(data, "tlm_flags")
    flying = np.zeros(n, bool)
    if np.isfinite(flags).any():
        fbits = np.nan_to_num(flags, nan=0.0).astype(np.int64)
        flying = np.isfinite(flags) & ((fbits & TLM_FLAG_FLYING) != 0)
    if not flying.any():
        # フォールバック: 電流>1.0A を飛行扱い(古いログ・欠損対策)
        cur = col(data, "tlm_current_a")
        flying = np.isfinite(cur) & (cur > 1.0)
        if flying.any():
            warnings.append("tlm_flags が無いため飛行判定は電流>1.0Aで代用")
    if not flying.any():
        raise ValueError("飛行区間が見つかりません(magbias学習には飛行ログが必要)")
    t_takeoff = float(t[flying][0])
    flight_dur = float(t[flying][-1] - t_takeoff)
    internals["t_takeoff"] = t_takeoff
    internals["flight_duration_s"] = flight_dur

    # ---- ログ世代の判定 ----
    is_v6 = has_col(data, COL_MAG_LEV_X) and has_col(data, COL_MAG_LEV_Y)
    internals["log_generation"] = "v6" if is_v6 else "v5"
    if not is_v6:
        warnings.append(
            "v5ログ縮退: mag_lev/mag_cal/ekf2 列が無いためベクトルフィット不能 "
            "(delta_b=null)。残差は EKF1 tlm_bm_x/y_ut で代用 — ヨー観測なしの "
            "EKF1 b_m は一定ヘディングで不可観測のため参考値 "
            "(FLIGHT_ANALYSIS_20260727.md §2-3)")

    # ---- ヨー基準の選択 ----
    mocap_deg = col(data, "mocap_yaw_true_deg")
    ekf2_yaw = col(data, COL_EKF2_YAW)
    mocap_ok = np.isfinite(mocap_deg[flying]).mean() > 0.8 \
        if flying.any() else False
    if yaw_source == "mocap" or (yaw_source == "auto" and mocap_ok):
        yaw_rad = np.radians(mocap_deg)
        yaw_used = "mocap_yaw_true_deg"
    elif yaw_source == "ekf2" or (yaw_source == "auto" and has_col(data, COL_EKF2_YAW)):
        yaw_rad = ekf2_yaw
        yaw_used = COL_EKF2_YAW
        warnings.append("ヨー基準に ekf2_yaw を使用(mocap真値なし)。"
                        "ヨー観測が融合されていない区間では磁気バイアスの影響を受ける")
    else:
        yaw_rad = col(data, "tlm_yaw_est_rad")
        yaw_used = "tlm_yaw_est_rad"
        warnings.append("ヨー基準に tlm_yaw_est_rad を使用(mocap/ekf2なし)。"
                        "推定誤差がフィットへ混入し得る")
    internals["yaw_source"] = yaw_used

    # ヨー励振量(選択したヨー基準の飛行中 unwrap 幅)
    yaw_u_flight = unwrap_finite(np.where(flying, yaw_rad, np.nan))
    fin = np.isfinite(yaw_u_flight)
    yaw_excursion_deg = float(np.degrees(
        yaw_u_flight[fin].max() - yaw_u_flight[fin].min())) if fin.any() else 0.0

    # ---- 1. ハードアイアン残差ベクトルフィット ----
    delta_b = None
    fit_rms = None
    fit = None
    if is_v6:
        settle = flying & (t >= t_takeoff + FIT_SETTLE_AFTER_TAKEOFF_S)
        fit = fit_hard_iron(t, yaw_rad,
                            col(data, COL_MAG_LEV_X), col(data, COL_MAG_LEV_Y),
                            settle)
        internals["fit"] = fit
        yaw_excursion_deg = max(yaw_excursion_deg, fit["yaw_excursion_deg"])
        if fit["ok"]:
            fit_rms = fit["rms_ut"]
            # z成分: ホバ中 b_cal.z と 地上(モーター停止)b_cal.z の差。
            dz, dz_warn = _z_residual(data, t, flying, t_takeoff)
            if dz_warn:
                warnings.append(dz_warn)
            delta_b = [fit["dx"], fit["dy"], dz if dz is not None else 0.0]
            warnings.append(
                "delta_b の x/y はレベル化水平フィット値を roll/pitch≈0 のホバ近傍"
                "近似で b_cal 機体系へ同一視したもの(大チルト飛行では不正確)。"
                "z成分はホバ/地上の mag_cal_z 差で、環境鉛直勾配(高度依存)を含む"
                "(FLIGHT_ANALYSIS_20260727.md §3.2 注意1)")
        else:
            warnings.append(f"delta_b=null: {fit['reason']}")
    else:
        warnings.append("delta_b=null: v5ログ(mag_lev 列なし)")

    # ---- 2. ホバ点残差 (b_m の安定窓中央値) ----
    if is_v6 and has_col(data, COL_EKF2_BM_X):
        bm_x = col(data, COL_EKF2_BM_X)
        bm_y = col(data, COL_EKF2_BM_Y)
        bm_used = "ekf2_bm"
    else:
        bm_x = col(data, COL_EKF1_BM_X)
        bm_y = col(data, COL_EKF1_BM_Y)
        bm_used = "ekf1_bm"
        if is_v6:
            warnings.append("ekf2_bm 列が無いため hover_residual は EKF1 tlm_bm で代用")
    internals["bm_source"] = bm_used
    hover = flying & (t >= t_takeoff + HOVER_WINDOW_AFTER_TAKEOFF_S)
    if not hover.any():
        hover = flying
        warnings.append(
            f"飛行が{HOVER_WINDOW_AFTER_TAKEOFF_S:.0f}s未満のため "
            "hover_residual は全飛行区間の中央値")
    hov_x = _finite_median(bm_x[hover])
    hov_y = _finite_median(bm_y[hover])
    if hov_x is None or hov_y is None:
        raise ValueError("b_m 列が無く hover_residual を計算できません "
                         f"({COL_EKF2_BM_X} / {COL_EKF1_BM_X})")
    hov_z, dz_warn2 = (None, None)
    if is_v6:
        hov_z, dz_warn2 = _z_residual(data, t, flying, t_takeoff)
        if dz_warn2 and delta_b is None:  # 同文言の重複を避ける
            warnings.append(dz_warn2)
    hover_current = _finite_median(col(data, "tlm_current_a")[hover])
    hover_residual = {"leveled_xy": [hov_x, hov_y], "z": hov_z,
                      "current_a": hover_current}

    # ---- 3. FF電圧/duty回帰 ----
    duty4 = [col(data, c) for c in DUTY_COLS]
    ff_supp = regress_ff_supplement(
        bm_x, bm_y, col(data, "tlm_voltage_v"), duty4,
        col(data, "tlm_current_a"), hover)
    if ff_supp is None:
        warnings.append("ff_supplement=null: 回帰サンプル不足(電圧/duty/電流列の欠損)")
    elif bm_used == "ekf1_bm":
        warnings.append("ff_supplement は EKF1 b_m での回帰(可観測化前) — 参考値")

    # ---- binding ----
    binding = _load_binding(ff_state_path or DEFAULT_FF_STATE, warnings)

    profile = {
        "schema": "stampfly_magbias_profile",
        "version": 1,
        "name": name,
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "source_logs": [source_log],
        "delta_b_ut": delta_b,
        "hover_residual_ut": hover_residual,
        "ff_supplement": ff_supp,
        "binding": binding,
        "quality": {
            "yaw_excursion_deg": yaw_excursion_deg,
            "fit_rms_ut": fit_rms,
            "warnings": warnings,
        },
        "provenance": {
            "tool": "magbias_learn.py",
            "tool_version": TOOL_VERSION,
            "log_generation": internals["log_generation"],
            "yaw_source": yaw_used,
            "bm_source": bm_used,
            "flight_duration_s": flight_dur,
        },
    }
    return profile, internals


def _z_residual(data: dict, t: np.ndarray, flying: np.ndarray,
                t_takeoff: float) -> tuple[float | None, str | None]:
    """z成分残差: ホバ中 mag_cal_z 中央値 − 地上(離陸前・モーター停止)中央値。

    地上窓が無い(ログが飛行中から始まる)場合は None。
    """
    mz = col(data, COL_MAG_CAL_Z)
    if not np.isfinite(mz).any():
        return None, "mag_cal_z 列が無いため z 残差は null"
    duty_off = np.ones(data["_n"], bool)
    for c in DUTY_COLS:
        d = col(data, c)
        duty_off &= ~(np.isfinite(d) & (d > 0.0))
    ground = (~flying) & duty_off & (t < t_takeoff)
    hover = flying & (t >= t_takeoff + HOVER_WINDOW_AFTER_TAKEOFF_S)
    g = _finite_median(mz[ground])
    h = _finite_median(mz[hover if hover.any() else flying])
    if g is None or h is None or int(np.sum(ground)) < 10:
        return None, "離陸前の地上窓が無いため z 残差は null(ログが飛行中から開始)"
    return h - g, ("z 残差はホバ/地上の mag_cal_z 差 — FF z残差と環境鉛直磁場差"
                   "(高度依存)の合算であり分離していない")


# ============================================================ 品質図 =========
def save_plots(data: dict, profile: dict, internals: dict, out_dir: Path) -> None:
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.font_manager as fm
    installed = {f.name for f in fm.fontManager.ttflist}
    for jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic",
               "Yu Gothic", "Noto Sans CJK JP", "IPAexGothic"):
        if jp in installed:
            plt.rcParams["font.family"] = jp
            break
    plt.rcParams["axes.unicode_minus"] = False

    out_dir.mkdir(parents=True, exist_ok=True)
    t = col(data, "elapsed_time")
    fig, axes = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    fit = internals.get("fit")

    ax = axes[0]
    zx = col(data, COL_MAG_LEV_X)
    zy = col(data, COL_MAG_LEV_Y)
    if np.isfinite(zx).any():
        ax.plot(t, zx, lw=0.7, label="mag_lev_x")
        ax.plot(t, zy, lw=0.7, label="mag_lev_y")
        if fit and fit.get("ok"):
            yaw_src = internals.get("yaw_source", "")
            yaw = np.radians(col(data, "mocap_yaw_true_deg")) \
                if yaw_src == "mocap_yaw_true_deg" else col(data, yaw_src)
            c, s = np.cos(yaw), np.sin(yaw)
            ax.plot(t, c * fit["b0h_x"] - s * fit["b0h_y"] + fit["dx"],
                    "--", lw=0.8, label="fit x")
            ax.plot(t, s * fit["b0h_x"] + c * fit["b0h_y"] + fit["dy"],
                    "--", lw=0.8, label="fit y")
    else:
        ax.text(0.5, 0.5, "mag_lev 列なし (v5ログ縮退)", ha="center",
                va="center", transform=ax.transAxes)
    ax.set_ylabel("leveled mag [uT]")
    if ax.get_legend_handles_labels()[0]:
        ax.legend(fontsize=8, ncol=4)
    ax.set_title(f"magbias 品質図: {profile['name']} "
                 f"(励振 {profile['quality']['yaw_excursion_deg']:.1f}°)")

    ax = axes[1]
    src = internals.get("bm_source", "")
    bx = col(data, COL_EKF2_BM_X if src == "ekf2_bm" else COL_EKF1_BM_X)
    by = col(data, COL_EKF2_BM_Y if src == "ekf2_bm" else COL_EKF1_BM_Y)
    ax.plot(t, bx, lw=0.7, label=f"{src}_x")
    ax.plot(t, by, lw=0.7, label=f"{src}_y")
    hov = profile["hover_residual_ut"]["leveled_xy"]
    ax.axhline(hov[0], color="C0", ls=":", lw=1)
    ax.axhline(hov[1], color="C1", ls=":", lw=1)
    ax.set_ylabel("b_m [uT]")
    ax.legend(fontsize=8)

    ax = axes[2]
    ax.plot(t, col(data, "tlm_current_a"), lw=0.7, label="current [A]")
    ax.plot(t, col(data, "tlm_voltage_v"), lw=0.7, label="voltage [V]")
    ax.set_ylabel("I / V")
    ax.set_xlabel("elapsed_time [s]")
    ax.legend(fontsize=8)

    fig.tight_layout()
    png = out_dir / "quality.png"
    fig.savefig(png, dpi=110)
    plt.close(fig)
    print(f"品質図: {png}")


# ============================================================ CLI ============
def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="飛行ログ → magbias プロファイルJSON 抽出 (契約 §3/§4)")
    ap.add_argument("log", help="飛行ログ CSV (logs/flight_logs/*.csv, v6推奨/v5縮退可)")
    ap.add_argument("-o", "--out",
                    help="出力JSONパス (既定 ../pc_server/data/magbias_profiles/<name>.json)")
    ap.add_argument("--name", help="プロファイル名 (既定 magbias_<ログstem>)")
    ap.add_argument("--yaw-source", choices=("auto", "mocap", "ekf2"),
                    default="auto", help="ベクトルフィットのヨー基準 (既定 auto)")
    ap.add_argument("--ff-state", default=str(DEFAULT_FF_STATE),
                    help="binding 取得元の ff_state.json (既定 pc_server/data/)")
    ap.add_argument("--plots", action="store_true", help="品質図PNGを出力")
    args = ap.parse_args(argv)

    log_path = Path(args.log).expanduser()
    if not log_path.is_file():
        print(f"ログが見つかりません: {log_path}", file=sys.stderr)
        return 2
    name = args.name or f"magbias_{log_path.stem}"

    try:
        data = load_flight_log(log_path)
        profile, internals = extract_magbias(
            data, name=name, source_log=log_path.name,
            yaw_source=args.yaw_source, ff_state_path=Path(args.ff_state))
    except ValueError as exc:
        print(f"抽出失敗: {exc}", file=sys.stderr)
        return 1

    out_path = Path(args.out) if args.out else DEFAULT_OUT_DIR / f"{name}.json"
    out_path.parent.mkdir(parents=True, exist_ok=True)
    # ffプロファイルと同じ規約: 非有限値(NaN/Inf)禁止(allow_nan=False)
    out_path.write_text(
        json.dumps(profile, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8")

    q = profile["quality"]
    db = profile["delta_b_ut"]
    print(f"magbias プロファイル生成: {out_path}")
    print(f"  ログ世代: {internals['log_generation']} / "
          f"ヨー基準: {internals['yaw_source']} / b_m: {internals['bm_source']}")
    if db is not None:
        print(f"  delta_b = [{db[0]:+.2f}, {db[1]:+.2f}, {db[2]:+.2f}] µT "
              f"(fit_rms {q['fit_rms_ut']:.2f} µT)")
    else:
        print("  delta_b = null (hover_residual のみ)")
    hov = profile["hover_residual_ut"]
    print(f"  hover_residual xy = [{hov['leveled_xy'][0]:+.2f}, "
          f"{hov['leveled_xy'][1]:+.2f}] µT @ {hov['current_a']:.2f} A")
    supp = profile["ff_supplement"]
    if supp is not None:
        print(f"  ff_supplement R² = {supp['r2']:.3f} (n={supp['n_samples']})")
    print(f"  ヨー励振 {q['yaw_excursion_deg']:.1f}° / 警告 {len(q['warnings'])}件")
    for w in q["warnings"]:
        print(f"    - {w}")

    if args.plots:
        gdir = name if name.startswith("magbias") else f"magbias_{name}"
        save_plots(data, profile, internals, GRAPH_DIR / gdir)
    return 0


if __name__ == "__main__":
    sys.exit(main())
