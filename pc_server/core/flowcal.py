"""オプティカルフロー較正(純回転 2×2 フィット)の記録・推定・適用
(FLIGHT_ANALYSIS_20260731.md §5 / MAG_AUTOTUNE_DESIGN.md §1.4-1.5)。

原理: 機上処理は trans_x = flow_x_rate − q̄、trans_y = flow_y_rate + p̄、
v = trans·d(d=ToF **生スラント距離** sensor_state.Range。firmware
flow_hub.cpp / sensor.cpp と同一)。PC 側は TLM_STATE からセンサーレートを
逆算する:
    flow_x_rate = v_x/d + q̄
    flow_y_rate = v_y/d − p̄
ここで TLM_STATE の altitude_tof はチルト補正後の鉛直高度
(≈ Range·cosφ·cosθ。sensor.cpp tof_tilt_comp_enabled=true 既定)なので、
機上の d(スラント)は d = altitude_tof / (cosφ·cosθ) で復元する
(roll/pitch も記録行に保存。±25° ロッキングで補正なしだと φ0 に
≈0.6°・非対角に ≈8% の系統誤差が乗る — 検証スクリプトで定量済み)。
純回転(手持ちロッキング・並進≈0)中にセンサーが見る角レートは
[q, −p] なので、現在の較正行列 K_cur(CAL_GET で取得。機体が実際に
適用した行列)でカウントレートへ戻し
    counts = K_cur · [flow_x_rate; flow_y_rate]
これを [q̄; −p̄] に対して 2×2 線形回帰(平均除去+残差 2.5σ 棄却×2)
すると新しい K [counts/rad](機体系レート→カウントレート)が得られる。
K=diag(kx,ky)·R(φ0) 形なら取付回転 φ0 も同時に推定される(2 定数の
旧 flowcal では表現できなかった成分 — 7/31 初飛行で φ0=−9.4° を実測)。

受入基準(StampFly_Telemetry の flowcal 手順を踏襲+行列化拡張):
n≥200 / 各軸 r²≥0.80 / 励振 std(p,q)≥0.30 rad/s / 含意スケール
300..600 counts/rad / x/y 比 ≤1.2 / |φ0| ≤15°。

MagbiasManager と同型のマネージャ:
- 記録: session._on_tlm_state から on_tlm_state() で供給(RX スレッド上:
  追記のみ)。開始はモーター停止・非飛行時のみ。
- 適用: CMD_FLOWCAL_SET(17B)→ CAL_GET 読み戻し(140B)4 成分照合 →
  flowcal_state.json 記録。排他はキャリブ/FF プロファイルと同じスロット
  (hub.calprofile_begin/end)を共有する。
"""

from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from . import config as cfg
from .calibration import CAL_VERIFY_TOLERANCE, ack_detail, ack_ok
from .serial_link import SerialLink, SerialLinkError

# ---- 受入基準(StampFly_Telemetry pc_server の FLOWCAL_* を踏襲) ----
FLOWCAL_MIN_SAMPLES = 200            # 最低有効サンプル数(25Hz × 8s 相当)
FLOWCAL_MIN_R2 = 0.80                # 各軸回帰の最低決定係数
FLOWCAL_MIN_EXCITATION_RAD_S = 0.30  # 収集中ジャイロ std の最低値(揺すり不足検出)
FLOWCAL_SCALE_PASS_MIN = 300.0       # 含意スケールの合格範囲 [counts/rad]
FLOWCAL_SCALE_PASS_MAX = 600.0       # (第一原理 ≈477・実機実績 450±10% を包含)
FLOWCAL_SCALE_RATIO_MAX = 1.2        # x/y スケール乖離の上限
FLOWCAL_PHI0_MAX_DEG = 15.0          # 含意取付回転の上限(7/31 実測 −9.4° を包含)

# ---- ファーム受理条件(flow_config.hpp flowcalMatrixValid と同値) ----
FLOWCAL_FW_SCALE_MIN = 100.0         # 対角の受理範囲 [counts/rad]
FLOWCAL_FW_SCALE_MAX = 2000.0        # 非対角は |m| ≤ MAX
FLOWCAL_FW_DET_MIN = FLOWCAL_FW_SCALE_MIN * FLOWCAL_FW_SCALE_MIN

# ---- 記録・頑健化 ----
FLOWCAL_MAX_SAMPLES = 1500           # リングバッファ上限(25Hz × 60s)
FLOWCAL_TOF_MIN_M = 0.05             # v/d 逆算の分母下限(ゼロ割り防護)
FLOWCAL_TILT_COS_MIN = 0.2           # スラント復元 d/(cosφ·cosθ) の分母下限
                                     # (ファームのチルトゲート 0.70rad≈cos0.76
                                     #  より十分下 — 異常姿勢での発散防護のみ)
FLOWCAL_TLM_FRESH_S = 1.0            # 記録開始ゲートの TLM_STATE 鮮度
FLOWCAL_EXP_FRESH_S = 1.0            # 同・TLM_EXP(motors_running)鮮度
FLOWCAL_OUTLIER_SIGMA = 2.5          # 残差棄却の閾値(σ 倍)
FLOWCAL_OUTLIER_PASSES = 2           # 棄却→再フィットの反復回数
FLOWCAL_CORR_MAX_RHO2 = 0.95         # p/q 励振の相関上限(回帰の縮退防止)
FLOWCAL_DEFAULT_SCALE = 450.0        # 機体既定 diag(450,450)(契約 §2.5)

# 有効サンプル条件: vel_valid ∧ range_valid ∧ squal_ok(契約 §1.1)
_FLOW_VALID_MASK = (proto.TlmState.FLOW_STATUS_VEL_VALID
                    | proto.TlmState.FLOW_STATUS_RANGE_VALID
                    | proto.TlmState.FLOW_STATUS_SQUAL_OK)


def flowcal_matrix_fw_valid(m00: float, m01: float,
                            m10: float, m11: float) -> bool:
    """ファームの K 受理条件(flow_config.hpp flowcalMatrixValid と同値)。

    対角 ∈ [100, 2000]・|非対角| ≤ 2000・det ≥ 100²(K⁻¹ の数値安定域)。
    PC 側で事前検査し、機体 NACK になる行列の送信を避ける。
    """
    values = (m00, m01, m10, m11)
    if any(not math.isfinite(v) for v in values):
        return False
    if not (FLOWCAL_FW_SCALE_MIN <= m00 <= FLOWCAL_FW_SCALE_MAX
            and FLOWCAL_FW_SCALE_MIN <= m11 <= FLOWCAL_FW_SCALE_MAX):
        return False
    if abs(m01) > FLOWCAL_FW_SCALE_MAX or abs(m10) > FLOWCAL_FW_SCALE_MAX:
        return False
    return (m00 * m11 - m01 * m10) >= FLOWCAL_FW_DET_MIN


def flowcal_sample_valid(sample: dict[str, Any]) -> bool:
    """フィットに使える行か(vel_valid ∧ range_valid ∧ squal_ok+d 下限)。"""
    status = int(sample.get("status") or 0)
    if (status & _FLOW_VALID_MASK) != _FLOW_VALID_MASK:
        return False
    d = float(sample.get("d") or 0.0)
    return d >= FLOWCAL_TOF_MIN_M


def _implied_params(m00: float, m01: float,
                    m10: float, m11: float) -> dict[str, float]:
    """K の含意パラメータ(kx=|K·e1|, ky=|K·e2|, φ0, x/y 比, det)。

    K=diag(kx,ky)·R(φ0) 形なら列ノルムが kx/ky、φ0 は
    atan2(−m01, m00)(x行)と atan2(m10, m11)(y行)が一致する — 平均を採る。
    """
    kx = math.hypot(m00, m10)
    ky = math.hypot(m01, m11)
    phi_x = math.degrees(math.atan2(-m01, m00))
    phi_y = math.degrees(math.atan2(m10, m11))
    ratio = (max(kx, ky) / min(kx, ky)) if min(kx, ky) > 0.0 else float("inf")
    return {
        "kx": kx, "ky": ky,
        "phi0_deg": 0.5 * (phi_x + phi_y),
        "ratio": ratio,
        "det": m00 * m11 - m01 * m10,
    }


def _solve_2x2_fit(points: list[tuple[float, float, float, float]]
                   ) -> Optional[dict[str, Any]]:
    """平均除去つき最小二乗で counts ≈ K·g を解く(縮退時 None)。

    points: (gx, gy, cx, cy) = ([q, −p] [rad/s], 生カウントレート [counts/s])。
    平均除去は切片(定常並進・センサーバイアス)を吸収する。
    """
    n = len(points)
    if n < 3:
        return None
    gx_m = sum(p[0] for p in points) / n
    gy_m = sum(p[1] for p in points) / n
    cx_m = sum(p[2] for p in points) / n
    cy_m = sum(p[3] for p in points) / n
    sxx = sxy = syy = 0.0
    bxx = bxy = byx = byy = 0.0
    for gx, gy, cx, cy in points:
        dgx = gx - gx_m
        dgy = gy - gy_m
        dcx = cx - cx_m
        dcy = cy - cy_m
        sxx += dgx * dgx
        sxy += dgx * dgy
        syy += dgy * dgy
        bxx += dcx * dgx
        bxy += dcx * dgy
        byx += dcy * dgx
        byy += dcy * dgy
    if sxx <= 0.0 or syy <= 0.0:
        return None                       # 片軸励振ゼロ(回帰不能)
    rho2 = (sxy * sxy) / (sxx * syy)
    if rho2 > FLOWCAL_CORR_MAX_RHO2:
        return None                       # p/q が相関しすぎ(縮退)
    det_s = sxx * syy - sxy * sxy
    m00 = (bxx * syy - bxy * sxy) / det_s
    m01 = (bxy * sxx - bxx * sxy) / det_s
    m10 = (byx * syy - byy * sxy) / det_s
    m11 = (byy * sxx - byx * sxy) / det_s

    # 各軸 r² と残差(棄却用の 2D 残差ノルムも返す)
    ss_res_x = ss_res_y = ss_tot_x = ss_tot_y = 0.0
    residuals: list[float] = []
    for gx, gy, cx, cy in points:
        dgx = gx - gx_m
        dgy = gy - gy_m
        rx = (cx - cx_m) - (m00 * dgx + m01 * dgy)
        ry = (cy - cy_m) - (m10 * dgx + m11 * dgy)
        ss_res_x += rx * rx
        ss_res_y += ry * ry
        ss_tot_x += (cx - cx_m) ** 2
        ss_tot_y += (cy - cy_m) ** 2
        residuals.append(math.hypot(rx, ry))
    r2x = 1.0 - ss_res_x / ss_tot_x if ss_tot_x > 0.0 else 0.0
    r2y = 1.0 - ss_res_y / ss_tot_y if ss_tot_y > 0.0 else 0.0
    return {
        "matrix": (m00, m01, m10, m11),
        "r2x": max(0.0, r2x), "r2y": max(0.0, r2y),
        "std_gx": math.sqrt(sxx / n), "std_gy": math.sqrt(syy / n),
        "residuals": residuals,
    }


def flowcal_fit(samples: list[dict[str, Any]],
                k_cur: tuple[float, float, float, float]) -> dict[str, Any]:
    """記録サンプルから新しい K を推定し、品質判定つきで返す(純関数)。

    samples: on_tlm_state が積む行 {q, p, vx, vy, d, roll, pitch, squal,
    status}(roll/pitch は省略可 — 無ければスラント復元なしで扱う)。
    k_cur: 記録中に機体が適用していた行列 (m00, m01, m10, m11)
    (テレメトリ速度 v は K_cur⁻¹ 適用後なので、生カウントレートへの
    復元に必須 — 記録開始時の CAL_GET スナップショットを渡すこと)。
    """
    result: dict[str, Any] = {"ok": False, "n_total": len(samples),
                              "warnings": []}
    warnings = result["warnings"]
    valid = [s for s in samples if flowcal_sample_valid(s)]
    result["n_valid"] = len(valid)
    result["valid_ratio"] = (len(valid) / len(samples)) if samples else 0.0
    if len(valid) < FLOWCAL_MIN_SAMPLES:
        warnings.append(
            f"有効サンプル不足({len(valid)} < {FLOWCAL_MIN_SAMPLES})— "
            "テクスチャのある面上・ToF 有効域でもっと長く揺すってください")
        return result

    k00, k01, k10, k11 = (float(v) for v in k_cur)
    points: list[tuple[float, float, float, float]] = []
    for s in valid:
        q = float(s["q"])
        p = float(s["p"])
        # 機上の d は生スラント距離だが、TLM の altitude_tof はチルト補正後の
        # 鉛直高度 — roll/pitch から d = alt/(cosφ·cosθ) でスラントへ復元する
        # (旧記録行に roll/pitch キーが無い場合は補正なし=cc=1 で従来どおり)
        cc = (math.cos(float(s.get("roll") or 0.0))
              * math.cos(float(s.get("pitch") or 0.0)))
        d = float(s["d"]) / max(cc, FLOWCAL_TILT_COS_MIN)
        # センサーレート逆算(機上式 v=(rate−gyro)·d の逆)→ 生カウントレート
        fx = float(s["vx"]) / d + q
        fy = float(s["vy"]) / d - p
        cx = k00 * fx + k01 * fy
        cy = k10 * fx + k11 * fy
        points.append((q, -p, cx, cy))

    # ロバスト化: フィット → 残差 2.5σ 棄却 → 再フィット(×2)。
    # 並進混入・スパイク(7/31 実測: 単発 0.4〜0.9 m/s)を落とす。
    used = points
    fit: Optional[dict[str, Any]] = None
    for pass_no in range(FLOWCAL_OUTLIER_PASSES + 1):
        fit = _solve_2x2_fit(used)
        if fit is None:
            warnings.append("回帰が縮退しました(ロールとピッチを別々の"
                            "リズムで揺すってください)")
            return result
        if pass_no >= FLOWCAL_OUTLIER_PASSES:
            break
        residuals = fit["residuals"]
        sigma = math.sqrt(sum(r * r for r in residuals) / len(residuals))
        if sigma <= 0.0:
            break
        keep = [pt for pt, r in zip(used, residuals)
                if r <= FLOWCAL_OUTLIER_SIGMA * sigma]
        if len(keep) == len(used) or len(keep) < FLOWCAL_MIN_SAMPLES:
            break
        used = keep

    m00, m01, m10, m11 = fit["matrix"]
    implied = _implied_params(m00, m01, m10, m11)
    n_used = len(used)
    result.update({
        "n_used": n_used,
        "n_rejected": len(points) - n_used,
        "matrix": [round(v, 3) for v in (m00, m01, m10, m11)],
        "k_cur": [round(float(v), 3) for v in k_cur],
        "r2x": round(fit["r2x"], 4), "r2y": round(fit["r2y"], 4),
        # 励振 std: x 軸フロー↔q、y 軸フロー↔−p(std は符号不変なので p と同値)
        "excitation_q_rad_s": round(fit["std_gx"], 3),
        "excitation_p_rad_s": round(fit["std_gy"], 3),
        "kx": round(implied["kx"], 2), "ky": round(implied["ky"], 2),
        "phi0_deg": round(implied["phi0_deg"], 2),
        "ratio": round(implied["ratio"], 3),
        "det": round(implied["det"], 1),
    })

    # ---- 品質判定(受入基準) ----
    if n_used < FLOWCAL_MIN_SAMPLES:
        warnings.append(f"棄却後サンプル不足({n_used} < {FLOWCAL_MIN_SAMPLES})"
                        "— 並進が多すぎます(手持ちで位置を保ってください)")
    if fit["std_gx"] < FLOWCAL_MIN_EXCITATION_RAD_S:
        warnings.append(f"ピッチ揺すり不足(std {fit['std_gx']:.2f} rad/s "
                        f"< {FLOWCAL_MIN_EXCITATION_RAD_S})— x 軸の傾きが"
                        "信頼できません")
    if fit["std_gy"] < FLOWCAL_MIN_EXCITATION_RAD_S:
        warnings.append(f"ロール揺すり不足(std {fit['std_gy']:.2f} rad/s "
                        f"< {FLOWCAL_MIN_EXCITATION_RAD_S})— y 軸の傾きが"
                        "信頼できません")
    if fit["r2x"] < FLOWCAL_MIN_R2:
        warnings.append(f"x 軸の相関が弱い(r²={fit['r2x']:.2f})— 並進が"
                        "混ざったか床のテクスチャ不足")
    if fit["r2y"] < FLOWCAL_MIN_R2:
        warnings.append(f"y 軸の相関が弱い(r²={fit['r2y']:.2f})— 並進が"
                        "混ざったか床のテクスチャ不足")
    for axis, scale in (("x", implied["kx"]), ("y", implied["ky"])):
        if not (FLOWCAL_SCALE_PASS_MIN <= scale <= FLOWCAL_SCALE_PASS_MAX):
            warnings.append(
                f"{axis} スケール {scale:.0f} counts/rad が合格範囲外"
                f"({FLOWCAL_SCALE_PASS_MIN:.0f}〜{FLOWCAL_SCALE_PASS_MAX:.0f})")
    if implied["ratio"] > FLOWCAL_SCALE_RATIO_MAX:
        warnings.append(f"x/y スケール乖離 {implied['ratio']:.2f} 倍"
                        f"(>{FLOWCAL_SCALE_RATIO_MAX})— 収集やり直し推奨")
    if abs(implied["phi0_deg"]) > FLOWCAL_PHI0_MAX_DEG:
        warnings.append(f"含意回転 φ0={implied['phi0_deg']:+.1f}° が"
                        f"上限 ±{FLOWCAL_PHI0_MAX_DEG:.0f}° を超過")
    if not flowcal_matrix_fw_valid(m00, m01, m10, m11):
        warnings.append("K が機体の受理範囲外です(対角 100..2000・"
                        "|非対角|≤2000・det≥10⁴)")

    result["ok"] = not warnings
    return result


def flowcal_live_metrics(samples: list[dict[str, Any]]) -> Optional[dict]:
    """記録中のライブ指標(UI メーター用。フィットはしない)。"""
    if not samples:
        return None
    valid = [s for s in samples if flowcal_sample_valid(s)]
    n_valid = len(valid)
    metrics: dict[str, Any] = {
        "n_total": len(samples),
        "n_valid": n_valid,
        "valid_ratio": n_valid / len(samples),
        "squal": int(samples[-1].get("squal") or 0),
        "tof_m": float(samples[-1].get("d") or 0.0),
    }
    if n_valid >= 2:
        for key, name in (("p", "std_p_rad_s"), ("q", "std_q_rad_s")):
            vals = [float(s[key]) for s in valid]
            mean = sum(vals) / n_valid
            metrics[name] = math.sqrt(
                sum((v - mean) ** 2 for v in vals) / n_valid)
    else:
        metrics["std_p_rad_s"] = 0.0
        metrics["std_q_rad_s"] = 0.0
    return metrics


class FlowcalManager:
    """フロー較正の記録/フィット/適用/状態(MagbiasManager と同型)。"""

    def __init__(self, link: SerialLink, hub, calibration,
                 notify: Callable[[str], None],
                 tlm_state_provider: Optional[Callable] = None,
                 state_path: Path = cfg.FLOWCAL_STATE_PATH) -> None:
        self.link = link
        self.hub = hub                   # ExperimentHub(モーター状態+排他スロット)
        self.calibration = calibration   # CalibrationManager(CAL_GET 用)
        self.notify = notify
        # session._tlm_state_snapshot: () -> (TlmState|None, age_s|None)
        self.tlm_state_provider = tlm_state_provider
        self.state_path = Path(state_path)

        self._lock = threading.Lock()
        self._collecting = False
        self._started_at = 0.0
        self._samples: list[dict[str, Any]] = []
        # 記録開始時の機体適用行列(CAL_GET スナップショット)と出所
        self._k_cur: tuple[float, float, float, float] = (
            FLOWCAL_DEFAULT_SCALE, 0.0, 0.0, FLOWCAL_DEFAULT_SCALE)
        self._k_cur_source = "default"
        self._fit: Optional[dict[str, Any]] = None
        self.message = ""
        self.busy = False

    # ---- 状態ファイル(magbias と同型) ----

    def _set_message(self, message: str) -> None:
        with self._lock:
            self.message = message

    def _load_state(self) -> dict[str, Any]:
        if not self.state_path.is_file():
            return {}
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.notify(f"[警告] {self.state_path.name} を読めないため"
                        f"未適用として扱います: {exc}")
            return {}
        return data if isinstance(data, dict) else {}

    def _save_state(self, state: dict[str, Any]) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.state_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2),
                       encoding="utf-8")
        os.replace(tmp, self.state_path)

    # ---- TLM_STATE 流入(RX スレッド上: 追記のみ・ブロッキング禁止) ----

    def on_tlm_state(self, tlm: proto.TlmState) -> None:
        with self._lock:
            if not self._collecting:
                return
            self._samples.append({
                "q": tlm.q, "p": tlm.p,
                "vx": tlm.flow_vx_mps, "vy": tlm.flow_vy_mps,
                "d": tlm.altitude_tof,
                # スラント復元 d/(cosφ·cosθ) 用の姿勢(flowcal_fit 参照)
                "roll": tlm.roll, "pitch": tlm.pitch,
                "squal": tlm.flow_squal, "status": tlm.flow_status,
            })
            if len(self._samples) > FLOWCAL_MAX_SAMPLES:
                self._samples = self._samples[-FLOWCAL_MAX_SAMPLES:]

    # ---- 状態 ----

    def status(self) -> dict[str, Any]:
        with self._lock:
            collecting = self._collecting
            samples = list(self._samples)
            started_at = self._started_at
            fit = self._fit
            k_cur = self._k_cur
            k_cur_source = self._k_cur_source
            busy = self.busy
            message = self.message
        state = self._load_state()
        applied = state.get("applied")
        if not isinstance(applied, dict):
            applied = None
        # 機体の現在行列: 直近受信の TLM_CAL_DATA を非ブロッキングで覗く
        # (CAL_GET は記録開始/適用時に発行される — status では要求しない)
        drone = None
        cal, _age = self.calibration.peek_cal_data()
        if cal is not None:
            drone = {
                "valid": bool(cal.valid_flags & proto.TlmCalData.VALID_FLOWCAL),
                "matrix": [cal.flowcal_m00, cal.flowcal_m01,
                           cal.flowcal_m10, cal.flowcal_m11],
            }
        return {
            "collecting": collecting,
            "duration_s": (round(time.time() - started_at, 1)
                           if collecting else None),
            "sample_count": len(samples),
            "live": flowcal_live_metrics(samples),
            "k_cur": {"matrix": list(k_cur), "source": k_cur_source},
            "fit": fit,
            "applied": applied,
            "drone": drone,
            "busy": busy,
            "message": message,
        }

    def _fail(self, message: str, **extra: Any) -> dict[str, Any]:
        self._set_message(message)
        return {"ok": False, "message": message, **extra, **self.status()}

    def _send_ack(self, msg_type: int, payload: bytes) -> tuple[bool, str]:
        try:
            ack = self.link.send_with_ack(msg_type, payload)
        except SerialLinkError as exc:
            return False, f"送信失敗: {exc}"
        return ack_ok(ack), ack_detail(ack)

    # ---- 記録 ----

    def start_record(self) -> dict[str, Any]:
        """記録開始(モーター停止・非飛行・TLM_STATE 受信中のみ)。

        開始時に CAL_GET で機体の現在行列 K_cur を取得して固定する
        (テレメトリ速度は K_cur⁻¹ 適用後 — フィットの生カウント復元に必須。
        記録中に行列を変えると復元が壊れるため、取得はここで一度だけ)。

        注意: self._lock は非リエントラント — status() もこのロックを取る
        ため、ロック保持中に status() を呼んではならない(デッドロック)。
        """
        with self._lock:
            collecting = self._collecting
        if collecting:
            return self._fail("既に記録中です")
        if not self.link.is_connected:
            return self._fail("シリアル未接続です")
        # モーター停止ゲート: PC 側モーターテスト状態+機体 TLM_EXP の実回転
        # フラグの両方を見る(プロペラ風・振動はフロー/ToF を汚す)
        if self.hub.motor_status().get("running"):
            return self._fail("モーター回転中は記録を開始できません"
                              "(先に停止してください)")
        exp_sample, exp_age = self.hub.latest_sample()
        if (exp_sample is not None and exp_age is not None
                and exp_age <= FLOWCAL_EXP_FRESH_S
                and exp_sample.get("motors_running")):
            return self._fail("機体側モーター回転中は記録を開始できません")
        tlm = age = None
        if self.tlm_state_provider is not None:
            tlm, age = self.tlm_state_provider()
        if tlm is None or age is None or age > FLOWCAL_TLM_FRESH_S:
            return self._fail("TLM_STATE を受信していません"
                              "(接続と機体状態を確認してください)")
        if tlm.flags & proto.TlmState.FLAG_FLYING:
            return self._fail("飛行中は記録できません(着陸後に実行)")
        # K_cur スナップショット(未設定なら既定 diag(450,450))
        cal = self.calibration.fetch_cal_data()
        if cal is None:
            return self._fail("機体から TLM_CAL_DATA を取得できませんでした"
                              "(リンク/機体状態を確認)")
        if cal.valid_flags & proto.TlmCalData.VALID_FLOWCAL:
            k_cur = (cal.flowcal_m00, cal.flowcal_m01,
                     cal.flowcal_m10, cal.flowcal_m11)
            source = "drone"
        else:
            k_cur = (FLOWCAL_DEFAULT_SCALE, 0.0, 0.0, FLOWCAL_DEFAULT_SCALE)
            source = "default"
        with self._lock:
            self._samples = []
            self._fit = None
            self._k_cur = k_cur
            self._k_cur_source = source
            self._started_at = time.time()
            self._collecting = True
            self.message = ("記録中: 床/机上 0.15〜0.5m で機体を手持ちし、"
                            "ロール→ピッチの順に ±20〜30° でゆっくり "
                            "20〜30 秒揺すってください(並進・ヨーは入れない)")
        return {"ok": True, **self.status()}

    def stop_and_fit(self) -> dict[str, Any]:
        """記録停止 → 2×2 フィット+品質判定。"""
        with self._lock:
            was_collecting = self._collecting
            self._collecting = False
            samples = list(self._samples)
            k_cur = self._k_cur
        if not was_collecting:
            return self._fail("記録していません")
        fit = flowcal_fit(samples, k_cur)
        with self._lock:
            self._fit = fit
        if fit.get("ok"):
            self._set_message(
                f"フィット完了(合格): kx={fit['kx']:.0f} ky={fit['ky']:.0f} "
                f"counts/rad, φ0={fit['phi0_deg']:+.1f}° "
                f"(r²={fit['r2x']:.3f}/{fit['r2y']:.3f}, "
                f"n={fit['n_used']})")
        else:
            self._set_message("フィットに警告があります: "
                              + " / ".join(fit.get("warnings") or ["不明"]))
        return {"ok": bool(fit.get("ok")), **self.status()}

    # ---- 適用 / クリア ----

    def apply(self, force: bool = False) -> dict[str, Any]:
        """フィット結果を機体へ適用する(NVS 書込み+読み戻し照合)。

        合格フィットのみ適用可(不合格は force で強制)。ただしファーム
        受理条件(flowcalMatrixValid)を満たさない行列は force でも送らない。
        """
        with self._lock:
            collecting = self._collecting
            fit = dict(self._fit) if self._fit else None
        if collecting:
            return self._fail("記録中は適用できません(先に停止&フィット)")
        if not fit or "matrix" not in fit:
            return self._fail("適用できるフィット結果がありません")
        if not fit.get("ok") and not force:
            return self._fail(
                "フィットに警告があります(force で強制適用可): "
                + " / ".join(fit.get("warnings") or []),
                quality_warning=True, warnings=fit.get("warnings") or [])
        m00, m01, m10, m11 = (float(v) for v in fit["matrix"])
        if not flowcal_matrix_fw_valid(m00, m01, m10, m11):
            return self._fail("K が機体の受理範囲外のため適用できません"
                              "(force でも不可 — 収集をやり直してください)")
        busy = self.hub.calprofile_begin()
        if busy:
            return self._fail(busy)
        with self._lock:
            self.busy = True
        try:
            ok, detail = self._send_ack(
                proto.MsgType.CMD_FLOWCAL_SET,
                proto.CmdFlowcalSet(mode=proto.CmdFlowcalSet.MODE_SET,
                                    m00=m00, m01=m01,
                                    m10=m10, m11=m11).to_payload())
            if not ok:
                return self._fail(f"CMD_FLOWCAL_SET が失敗しました: {detail}")
            # CAL_GET 読み戻しで valid bit7+4 成分照合(契約 §1.5)
            data = self.calibration.fetch_cal_data()
            if data is None:
                return self._fail("CAL_GET の応答がありません", verified=False)
            if not (data.valid_flags & proto.TlmCalData.VALID_FLOWCAL):
                return self._fail("読み戻しで flowcal が有効になっていません",
                                  verified=False)
            got = (data.flowcal_m00, data.flowcal_m01,
                   data.flowcal_m10, data.flowcal_m11)
            mismatch = [f"{name}: sent={want:.3f} drone={float(g):.3f}"
                        for name, want, g in zip(
                            ("m00", "m01", "m10", "m11"),
                            (m00, m01, m10, m11), got)
                        if abs(float(g) - want) > CAL_VERIFY_TOLERANCE]
            if mismatch:
                return self._fail(
                    f"読み戻し値が一致しません: {', '.join(mismatch)}",
                    verified=False)
            # 適用状態の永続化(flowcal_state.json)
            self._save_state({
                "applied": {
                    "matrix": [m00, m01, m10, m11],
                    "kx": fit.get("kx"), "ky": fit.get("ky"),
                    "phi0_deg": fit.get("phi0_deg"),
                    "r2x": fit.get("r2x"), "r2y": fit.get("r2y"),
                    "n_used": fit.get("n_used"),
                    "forced": bool(force and not fit.get("ok")),
                    "applied_at": time.time(),
                    "verified": True,
                },
            })
            forced = "(force適用)" if (force and not fit.get("ok")) else ""
            self._set_message(
                f"適用・読み戻し照合OK{forced}: K=[{m00:.1f} {m01:.1f}; "
                f"{m10:.1f} {m11:.1f}] counts/rad "
                f"(φ0={fit.get('phi0_deg', 0):+.1f}°)")
            return {"ok": True, "verified": True, **self.status()}
        finally:
            with self._lock:
                self.busy = False
            self.hub.calprofile_end()

    def clear(self) -> dict[str, Any]:
        """PC 側の記録・フィット結果を破棄する(機体 NVS は変更しない —
        機体側のクリアは /api/flow action=cal_clear)。"""
        with self._lock:
            self._collecting = False
            self._samples = []
            self._fit = None
            self.message = "記録・フィット結果をクリアしました"
        return {"ok": True, **self.status()}
