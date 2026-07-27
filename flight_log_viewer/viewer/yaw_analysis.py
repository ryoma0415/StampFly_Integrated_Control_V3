"""ヨー解析(本プロジェクトの主目的)。

- ヨー比較図: tlm_yaw_rad(Madgwick)/ tlm_yaw_est_rad(EKF)/
  mocap_yaw_true_deg(MoCap 正解Yaw、v5 ログのみ)/ ヨー指令
  (ジャイロ積算は図から除外。統計 compute_yaw_stats は全系統で計算し
  レポートに使う。旧 mocap_yaw_deg は軸違いのため全面的に不使用)
- 対基準誤差の統計(RMS / ドリフト率 [°/min])— レポート・2ログ比較用
- EKF 診断: NIS・磁気バイアス b_m・FF 補正 db_hat・ffg ゲートタイムライン
- ヨー指令追従: cmd_yaw_ref vs 機体適用目標 vs 実測
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from . import jp_font

jp_font.setup_japanese_font()

from .constants import (  # noqa: E402
    BM_FREEZE_THRESHOLD_UT,
    COLORS,
    FF_STATUS_FF_MODE_MASK,
    FF_STATUS_FLAG_BITS,
    FFG_GATE_BITS,
    NIS_EXPECTED,
    NIS_R_INFLATE_THRESHOLD,
    NIS_REJECT_THRESHOLD,
    YAW_SOURCES,
)
from .loader import (  # noqa: E402
    YAW_ESTIMATOR_KEYS,
    FlightLog,
    unwrap_deg,
    wrapped_plot_series,
)
from .style import styled_legend, new_fig, save_fig  # noqa: E402

# 系統キー → (表示名, 色) の索引
_SOURCE_INFO: dict[str, tuple[str, str]] = {
    key: (label, color) for key, _col, label, color, _deg in YAW_SOURCES
}

SECONDS_PER_MINUTE = 60.0


# ---------------------------------------------------------------------------
# 統計
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class YawErrorStat:
    """1 系統の対基準ヨー誤差統計。"""

    key: str                  # 系統キー(madgwick / ekf / gyro_int)
    label: str                # 表示名
    reference: str            # 基準系統キー(mocap / madgwick)
    n: int                    # 有効サンプル数
    rms_deg: float            # RMS 誤差 [deg]
    max_abs_deg: float        # 最大絶対誤差 [deg]
    drift_deg_per_min: float  # ドリフト率 [°/min](アンラップ誤差の線形回帰勾配)


def _linear_drift_deg_per_min(t_s: np.ndarray, err_deg: np.ndarray) -> float:
    """アンラップ済み誤差の線形回帰勾配 [°/min] を返す。"""
    finite = np.isfinite(t_s) & np.isfinite(err_deg)
    if finite.sum() < 2:
        return math.nan
    slope_per_s = np.polyfit(t_s[finite], err_deg[finite], 1)[0]
    return float(slope_per_s * SECONDS_PER_MINUTE)


def compute_yaw_stats(log: FlightLog) -> dict:
    """ヨー統計一式を辞書で返す(レポート・比較で使用)。

    返り値のキー:
      reference: 基準系統キー(mocap / madgwick / None)
      errors: list[YawErrorStat]
      tracking_rms_deg / tracking_max_deg: ヨー指令追従誤差(ON 区間)
      nis_mean / nis_p95 / nis_max, bm_max_ut, gate_rates(名前→(回数, %))
    """
    df = log.df
    result: dict = {
        "reference": None,
        "errors": [],
        "tracking_rms_deg": math.nan,
        "tracking_max_deg": math.nan,
        "nis_mean": math.nan,
        "nis_p95": math.nan,
        "nis_max": math.nan,
        "bm_max_ut": math.nan,
        "gate_rates": {},
    }

    ref = log.yaw_reference()
    if ref is not None:
        ref_key, _ref_series = ref
        result["reference"] = ref_key
        t = log.t
        for key in YAW_ESTIMATOR_KEYS:
            err_col = f"yaw_err_{key}_deg"
            if key == ref_key or err_col not in df.columns:
                continue
            err = df[err_col].to_numpy(dtype=float)
            finite = np.isfinite(err)
            if not finite.any():
                continue
            label, _color = _SOURCE_INFO[key]
            err_unwrapped = unwrap_deg(err)
            result["errors"].append(YawErrorStat(
                key=key,
                label=label,
                reference=ref_key,
                n=int(finite.sum()),
                rms_deg=float(np.sqrt(np.nanmean(err[finite] ** 2))),
                max_abs_deg=float(np.nanmax(np.abs(err[finite]))),
                drift_deg_per_min=_linear_drift_deg_per_min(t, err_unwrapped),
            ))

    # ヨー指令追従(ヨー制御 ON 区間)
    if "yaw_track_err_deg" in df.columns:
        track = df["yaw_track_err_deg"].to_numpy(dtype=float)
        finite = np.isfinite(track)
        if finite.any():
            result["tracking_rms_deg"] = float(np.sqrt(np.mean(track[finite] ** 2)))
            result["tracking_max_deg"] = float(np.max(np.abs(track[finite])))

    # EKF 診断
    if log.has("tlm_nis"):
        nis = df["tlm_nis"].to_numpy(dtype=float)
        nis = nis[np.isfinite(nis)]
        if nis.size:
            result["nis_mean"] = float(np.mean(nis))
            result["nis_p95"] = float(np.percentile(nis, 95))
            result["nis_max"] = float(np.max(nis))
    if log.has("tlm_bm_x_ut") and log.has("tlm_bm_y_ut"):
        bm_norm = np.hypot(df["tlm_bm_x_ut"].to_numpy(dtype=float),
                           df["tlm_bm_y_ut"].to_numpy(dtype=float))
        if np.isfinite(bm_norm).any():
            result["bm_max_ut"] = float(np.nanmax(bm_norm))
    if log.has("tlm_ffg"):
        ffg = df["tlm_ffg"].to_numpy(dtype=float)
        finite = np.isfinite(ffg)
        n_total = int(finite.sum())
        ffg_int = ffg[finite].astype(int)
        for bit, (name, _desc, _color) in enumerate(FFG_GATE_BITS):
            count = int((((ffg_int >> bit) & 1)).sum())
            rate = 100.0 * count / n_total if n_total else 0.0
            result["gate_rates"][name] = (count, rate)

    return result


# ---------------------------------------------------------------------------
# 図
# ---------------------------------------------------------------------------

def _fig_yaw_comparison(log: FlightLog, out_dir: Path) -> Path | None:
    """10: ヨー比較(Madgwick / EKF / MoCap 正解Yaw / 指令)。

    MoCap は v5 の正解Yaw(mocap_yaw_true_deg: 符号/オフセット/フリップ
    補正済み)のみ表示する。旧列 mocap_yaw_deg による「MoCap 真値」は
    軸違いのため 2026-07 に除外し、正解Yaw の導入で復帰した。
    ジャイロ積算は表示しない(参考値のため。統計には含む)。
    """
    available = [
        (key, label, color)
        for key, _col, label, color, _deg in YAW_SOURCES
        if key in ("madgwick", "ekf", "mocap") and log.has(f"yaw_{key}_deg")
    ]
    if not available:
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(13.0, 9.0), sharex=True)

    ax = axes[0]
    for key, label, color in available:
        ax.plot(t, df[f"yaw_{key}_unwrap_deg"], color=color, linewidth=1.1,
                alpha=0.9, label=label)
    if log.has("cmd_yaw_ref_deg"):
        ax.plot(t, unwrap_deg(df["cmd_yaw_ref_deg"].to_numpy(dtype=float)),
                color=COLORS["yaw_cmd"], linewidth=1.0, linestyle="--", alpha=0.8,
                label="ヨー指令(PC)")
    ax.set_title("ヨー比較: Madgwick / EKF / MoCap 正解Yaw / 指令(アンラップ)",
                 fontsize=14)
    ax.set_ylabel("ヨー角 [deg](連続)", fontsize=10)
    styled_legend(ax, ncol=2)

    ax = axes[1]
    for key, label, color in available:
        # ±180 パネルはここでラップして描く(生の deg 列はソース範囲を保持して
        # いる — ジャイロ積算は無制限、旧ログの Madgwick は 0..-360 — ため、
        # そのままでは軸外にクリップされて見えなくなる)。ラップ跨ぎには
        # NaN を挿入して縦線を防ぐ。
        tw, yw = wrapped_plot_series(t, df[f"yaw_{key}_deg"].to_numpy(dtype=float))
        ax.plot(tw, yw, color=color, linewidth=0.9, alpha=0.9, label=label)
    ax.set_ylim(-185.0, 185.0)
    ax.set_ylabel("ヨー角 [deg](±180)", fontsize=10)
    ax.set_xlabel("時間 [s]", fontsize=11)
    styled_legend(ax, ncol=2)
    return save_fig(fig, out_dir, "10_yaw_comparison.png")


def _fig_ekf_diagnostics(log: FlightLog, out_dir: Path) -> Path | None:
    """12: EKF 診断(NIS / b_m / db_hat / ffg ゲートラスタ)。"""
    if not (log.has("tlm_nis") or log.has("tlm_bm_x_ut") or log.has("tlm_ffg")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(4, 1, figsize=(13.0, 12.0), sharex=True,
                        height_ratios=[2.0, 2.0, 1.5, 2.0])

    # NIS
    ax = axes[0]
    if log.has("tlm_nis"):
        nis = df["tlm_nis"].to_numpy(dtype=float)
        ax.plot(t, nis, color=COLORS["nis"], linewidth=0.9, label="NIS")
        ax.axhline(NIS_R_INFLATE_THRESHOLD, ls="--", color="#f59e0b", lw=1,
                   label=f"χ²(2) 95%={NIS_R_INFLATE_THRESHOLD} (R膨張)")
        ax.axhline(NIS_REJECT_THRESHOLD, ls="--", color="#ef4444", lw=1,
                   label=f"χ²(2) 99.9%={NIS_REJECT_THRESHOLD} (棄却)")
        ax.axhline(NIS_EXPECTED, ls=":", color="#6b7280", lw=1,
                   label=f"期待値≈{NIS_EXPECTED:.0f}")
        finite = nis[np.isfinite(nis)]
        top = np.percentile(finite, 99) if finite.size else NIS_REJECT_THRESHOLD
        ax.set_ylim(0, max(NIS_REJECT_THRESHOLD * 1.1, float(top) * 1.2))
    ax.set_title("EKF 診断: NIS(観測整合度)", fontsize=13)
    ax.set_ylabel("NIS", fontsize=10)
    styled_legend(ax, ncol=2)

    # b_m
    ax = axes[1]
    if log.has("tlm_bm_x_ut") and log.has("tlm_bm_y_ut"):
        bm_x = df["tlm_bm_x_ut"].to_numpy(dtype=float)
        bm_y = df["tlm_bm_y_ut"].to_numpy(dtype=float)
        ax.plot(t, bm_x, color=COLORS["bm_x"], linewidth=1.0, label="b_mx")
        ax.plot(t, bm_y, color=COLORS["bm_y"], linewidth=1.0, label="b_my")
        ax.plot(t, np.hypot(bm_x, bm_y), color=COLORS["bm_norm"], linewidth=1.2,
                label="‖b_m‖")
        ax.axhline(BM_FREEZE_THRESHOLD_UT, ls="--", color="#ef4444", lw=1,
                   label=f"凍結しきい値 {BM_FREEZE_THRESHOLD_UT:.0f}µT")
    ax.set_ylabel("b_m [µT]", fontsize=10)
    ax.set_title("EKF 磁気バイアス状態 b_m", fontsize=12)
    styled_legend(ax, ncol=2)

    # db_hat(FF 補正ベクトル)
    ax = axes[2]
    if log.has("tlm_db_hat_x_ut"):
        ax.plot(t, df["tlm_db_hat_x_ut"], color=COLORS["dbhat_x"], linewidth=0.9,
                label="db̂_x")
    if log.has("tlm_db_hat_y_ut"):
        ax.plot(t, df["tlm_db_hat_y_ut"], color=COLORS["dbhat_y"], linewidth=0.9,
                label="db̂_y")
    ax.axhline(y=0, color="gray", linestyle="--", alpha=0.5)
    ax.set_ylabel("db̂ [µT]", fontsize=10)
    ax.set_title("FF 磁気補正ベクトル db̂(電流起因外乱の補正量)", fontsize=12)
    styled_legend(ax)

    # ffg ゲートラスタ
    ax = axes[3]
    if log.has("tlm_ffg"):
        ffg = df["tlm_ffg"].to_numpy(dtype=float)
        ffg_int = np.where(np.isfinite(ffg), ffg, 0).astype(int)
        for bit, (name, _desc, color) in enumerate(FFG_GATE_BITS):
            active = ((ffg_int >> bit) & 1).astype(bool)
            ax.fill_between(t, bit, bit + 0.8, where=active, color=color, step="mid")
        ax.set_yticks([bit + 0.4 for bit in range(len(FFG_GATE_BITS))])
        ax.set_yticklabels([g[0] for g in FFG_GATE_BITS], fontsize=8)
        ax.set_ylim(0, len(FFG_GATE_BITS))
    ax.set_title("ffg ゲート発火(帯 = そのゲートが立っている区間)", fontsize=12)
    ax.set_xlabel("時間 [s]", fontsize=11)
    return save_fig(fig, out_dir, "12_ekf_diagnostics.png")


def _fig_ff_status(log: FlightLog, out_dir: Path) -> Path | None:
    """13: ff_status タイムライン(ff_mode 値+フラグビットのラスタ)。"""
    if not log.has("tlm_ff_status"):
        return None
    t = log.t
    status = log.df["tlm_ff_status"].to_numpy(dtype=float)
    status_int = np.where(np.isfinite(status), status, 0).astype(int)
    fig, axes = new_fig(2, 1, figsize=(13.0, 7.0), sharex=True,
                        height_ratios=[1.0, 2.0])

    ax = axes[0]
    ff_mode = status_int & FF_STATUS_FF_MODE_MASK
    ax.step(t, ff_mode, where="mid", color="#f59e0b", linewidth=1.2, label="ff_mode")
    ax.set_yticks([0, 1, 2])
    ax.set_yticklabels(["0: off", "1: A", "2: B"], fontsize=8)
    ax.set_ylim(-0.3, 2.3)
    ax.set_title("ff_status タイムライン", fontsize=14)
    styled_legend(ax)

    ax = axes[1]
    for row, (name, bit, color) in enumerate(FF_STATUS_FLAG_BITS):
        active = ((status_int >> bit) & 1).astype(bool)
        ax.fill_between(t, row, row + 0.8, where=active, color=color, step="mid")
    ax.set_yticks([row + 0.4 for row in range(len(FF_STATUS_FLAG_BITS))])
    ax.set_yticklabels([b[0] for b in FF_STATUS_FLAG_BITS], fontsize=8)
    ax.set_ylim(0, len(FF_STATUS_FLAG_BITS))
    ax.set_xlabel("時間 [s]", fontsize=11)
    return save_fig(fig, out_dir, "13_ff_status.png")


def _fig_yaw_tracking(log: FlightLog, out_dir: Path, stats: dict) -> Path | None:
    """14: ヨー指令追従(PC 指令 / 機体適用目標 / 実測 / 追従誤差)。"""
    if not (log.has("cmd_yaw_ref_deg") or log.has("tlm_yaw_ref_deg")):
        return None
    t = log.t
    df = log.df
    fig, axes = new_fig(2, 1, figsize=(13.0, 9.0), sharex=True,
                        height_ratios=[2.0, 1.0])

    ax = axes[0]
    # ±180° ラップ+ラップ跨ぎ NaN 挿入で描く(縦線防止)
    if log.has("cmd_yaw_ref_deg"):
        tw, yw = wrapped_plot_series(t, df["cmd_yaw_ref_deg"].to_numpy(dtype=float))
        ax.plot(tw, yw, color=COLORS["yaw_cmd"], linewidth=1.0,
                linestyle="--", alpha=0.9, label="ヨー指令(PC 送信)")
    if log.has("tlm_yaw_ref_deg"):
        tw, yw = wrapped_plot_series(t, df["tlm_yaw_ref_deg"].to_numpy(dtype=float))
        ax.plot(tw, yw, color=COLORS["yaw_ref_applied"],
                linewidth=1.0, alpha=0.9, label="機体適用目標(ラッチ含む)")
    if log.has("yaw_ekf_deg"):
        tw, yw = wrapped_plot_series(t, df["yaw_ekf_deg"].to_numpy(dtype=float))
        ax.plot(tw, yw, color=COLORS["yaw_ekf"], linewidth=1.0,
                alpha=0.9, label="実測(アクティブ推定器)")
    if log.has("yaw_ctrl_on"):
        on = df["yaw_ctrl_on"].to_numpy(dtype=float) > 0
        if on.any():
            ax.fill_between(t, *ax.get_ylim(), where=on, color="#f59e0b",
                            alpha=0.15, step="mid", label="ヨー制御 ON 区間")
    ax.set_title("ヨー指令追従", fontsize=14)
    ax.set_ylabel("ヨー角 [deg]", fontsize=10)
    styled_legend(ax, loc="best")

    ax = axes[1]
    if "yaw_track_err_deg" in df.columns and df["yaw_track_err_deg"].notna().any():
        label = "追従誤差(ON 区間)"
        if math.isfinite(stats.get("tracking_rms_deg", math.nan)):
            label += (f"  RMS={stats['tracking_rms_deg']:.2f}°"
                      f"  最大={stats['tracking_max_deg']:.2f}°")
        ax.plot(t, df["yaw_track_err_deg"], color="#dc2626", linewidth=0.9,
                label=label)
    ax.axhline(y=0, color="gray", linestyle="--", alpha=0.6)
    ax.set_ylabel("誤差 [deg]", fontsize=10)
    ax.set_xlabel("時間 [s]", fontsize=11)
    styled_legend(ax, loc="best")
    return save_fig(fig, out_dir, "14_yaw_tracking.png")


def generate_yaw_figures(log: FlightLog, out_dir: str | Path,
                         stats: dict | None = None) -> list[Path]:
    """ヨー解析図一式を生成し、生成できたパスの一覧を返す。"""
    out_dir = Path(out_dir)
    if stats is None:
        stats = compute_yaw_stats(log)
    print("\nヨー解析図を生成します...")
    saved: list[Path] = []
    for path in (
        _fig_yaw_comparison(log, out_dir),
        _fig_ekf_diagnostics(log, out_dir),
        _fig_ff_status(log, out_dir),
        _fig_yaw_tracking(log, out_dir, stats),
    ):
        if path is not None:
            saved.append(path)
    print(f"ヨー解析図生成完了: {len(saved)} 枚")
    return saved
