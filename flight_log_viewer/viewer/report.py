"""サマリレポート(summary.txt + index.html)と 2 ログ比較レポートの生成。

- 単一ログ: 飛行時間・位置 RMS 誤差・ヨー RMS/最大誤差・ドリフト率・電圧推移
  などの統計をテキストと HTML(図の index)で出力する。
- 比較モード: 2 本のログのヨー安定性(RMS / ドリフト率 / NIS / ゲート発火)と
  位置保持XY統計(RMS X/Y/2D・最大2D)を
  並べて比較する HTML と重ね描き図を出力する。
"""

from __future__ import annotations

import html
import math
from datetime import datetime
from pathlib import Path

import numpy as np

from . import jp_font

jp_font.setup_japanese_font()

from .constants import (  # noqa: E402
    LOW_VOLTAGE_THRESHOLD_V,
    TLM_FLAG_LOW_VOLTAGE,
)
from .loader import FlightLog  # noqa: E402
from .style import styled_legend, new_fig, save_fig  # noqa: E402
from .yaw_analysis import _SOURCE_INFO, compute_yaw_stats  # noqa: E402

# 図キャプション(ファイル名 → 表示名)
_FIGURE_CAPTIONS: dict[str, str] = {
    "01_xy_trajectory.png": "XY 飛行軌跡",
    "02_attitude.png": "姿勢: 指令 vs 実測",
    "03_altitude.png": "高度と昇降速度",
    "04_position_tracking.png": "位置追従(目標 vs 実測)",
    "05_pid_components.png": "XY PID 成分(旧ログ v1〜v3 のみ)",
    "06_duty.png": "モーター duty",
    "07_power.png": "電源(電圧 / 電流)",
    "08_latency_loop_dt.png": "通信レイテンシ / 制御周期",
    "09_mocap_diagnostics.png": "MoCap 診断",
    "10_yaw_comparison.png": "ヨー比較(Madgwick / EKF / EKF2(v6 のみ) / 指令)",
    "12_ekf_diagnostics.png": "EKF 診断(NIS / b_m / db̂ / ゲート)",
    "13_ff_status.png": "ff_status タイムライン",
    "14_yaw_tracking.png": "ヨー指令追従",
    "15_xyz_3d.png": "3D 飛行軌跡(時間カラー)",
    "16_xy_time.png": "XY 軌跡(時間カラー)",
    "17_cmd_echo.png": "指令エコー: 送信指令 vs 機体適用",
    "18_pid_ang_components.png": "角度ループ PID 成分(P+I+D=指令角速度)",
    "19_pid_rate_components.png": "角速度ループ PID 成分",
    "20_rate_tracking.png": "角速度追従(指令 vs 実測)",
    "21_ekf2_diagnostics.png": "EKF2 診断(イノベーション / b_m 比較 / "
                               "status / gate。v6 のみ)",
    "22_mag_observation.png": "磁気観測(b_cal / レベル化観測 z。v6 のみ)",
    "23_flow.png": "オプティカルフロー(vx/vy / SQUAL / status。v6 のみ)",
    "M01_multi_xy.png": "複数機 XY 軌跡(共有)",
}


def _fmt(value: float, digits: int = 2, unit: str = "") -> str:
    """NaN を「—」にする数値フォーマッタ。"""
    if value is None or (isinstance(value, float) and not math.isfinite(value)):
        return "—"
    return f"{value:.{digits}f}{unit}"


# ---------------------------------------------------------------------------
# サマリ統計
# ---------------------------------------------------------------------------

def compute_position_stats(log: FlightLog) -> dict:
    """位置保持(XY)統計を計算する(単一レポート・2ログ比較で共用)。

    error_x/error_y = 目標 − フィルタ済み実位置(制御座標系, m)を、
    閉ループ制御中(control_active=1)の行だけで集計する。
    離陸直後の過渡も含まれる点に注意(区間は START〜制御停止)。
    """
    df = log.df
    stats = {
        "pos_rms_x_m": math.nan,
        "pos_rms_y_m": math.nan,
        "pos_rms_2d_m": math.nan,
        "pos_max_2d_m": math.nan,
    }
    if log.has("error_x") and log.has("error_y"):
        ex = df["error_x"].to_numpy(dtype=float)
        ey = df["error_y"].to_numpy(dtype=float)
        mask = np.isfinite(ex) & np.isfinite(ey)
        if log.has("control_active"):
            active = df["control_active"].to_numpy(dtype=float)
            mask &= np.isfinite(active) & (active > 0)
        if mask.any():
            stats["pos_rms_x_m"] = float(np.sqrt(np.mean(ex[mask] ** 2)))
            stats["pos_rms_y_m"] = float(np.sqrt(np.mean(ey[mask] ** 2)))
            e2d = np.hypot(ex[mask], ey[mask])
            stats["pos_rms_2d_m"] = float(np.sqrt(np.mean(e2d ** 2)))
            stats["pos_max_2d_m"] = float(np.max(e2d))
    return stats


def compute_summary(log: FlightLog) -> dict:
    """レポートに載せる統計一式を計算して辞書で返す。"""
    df = log.df
    summary: dict = {
        "log_name": log.name,
        "mode": log.mode,
        "rows": len(df),
        "duration_s": log.duration_s,
        "flight_time_s": log.flight_time_s(),
        "yaw": compute_yaw_stats(log),
        # 位置
        "pos_rms_x_m": math.nan,
        "pos_rms_y_m": math.nan,
        "pos_rms_2d_m": math.nan,
        "pos_max_2d_m": math.nan,
        # 電源
        "voltage_start_v": math.nan,
        "voltage_end_v": math.nan,
        "voltage_min_v": math.nan,
        "low_voltage_rows": 0,
        "current_mean_a": math.nan,
        "current_max_a": math.nan,
        # 通信 / 周期
        "latency_mean_ms": math.nan,
        "latency_p95_ms": math.nan,
        "loop_dt_mean_us": math.nan,
        "loop_dt_max_us": math.nan,
    }

    # 位置 RMS(閉ループ制御中の行のみ。compute_position_stats と共用)
    summary.update(compute_position_stats(log))

    # 電圧推移
    if log.has("tlm_voltage_v"):
        volts = df["tlm_voltage_v"].to_numpy(dtype=float)
        finite = volts[np.isfinite(volts)]
        if finite.size:
            summary["voltage_start_v"] = float(finite[0])
            summary["voltage_end_v"] = float(finite[-1])
            summary["voltage_min_v"] = float(np.min(finite))
    if log.has("tlm_flags"):
        flags = df["tlm_flags"].to_numpy(dtype=float)
        finite = np.isfinite(flags)
        summary["low_voltage_rows"] = int(
            ((flags[finite].astype(int) & TLM_FLAG_LOW_VOLTAGE) != 0).sum())

    # 電流
    if log.has("tlm_current_a"):
        cur = df["tlm_current_a"].to_numpy(dtype=float)
        finite = cur[np.isfinite(cur)]
        if finite.size:
            summary["current_mean_a"] = float(np.mean(finite))
            summary["current_max_a"] = float(np.max(finite))

    # レイテンシ / 制御周期
    if log.has("feedback_latency_ms"):
        lat = df["feedback_latency_ms"].to_numpy(dtype=float)
        finite = lat[np.isfinite(lat)]
        if finite.size:
            summary["latency_mean_ms"] = float(np.mean(finite))
            summary["latency_p95_ms"] = float(np.percentile(finite, 95))
    if log.has("tlm_loop_dt_us"):
        dt = df["tlm_loop_dt_us"].to_numpy(dtype=float)
        finite = dt[np.isfinite(dt)]
        if finite.size:
            summary["loop_dt_mean_us"] = float(np.mean(finite))
            summary["loop_dt_max_us"] = float(np.max(finite))

    return summary


# ---------------------------------------------------------------------------
# テキストサマリ
# ---------------------------------------------------------------------------

def _summary_lines(summary: dict) -> list[str]:
    """summary.txt 用の行リストを作る。"""
    yaw = summary["yaw"]
    lines: list[str] = []
    lines.append("=" * 64)
    lines.append(f"フライトログ サマリ: {summary['log_name']}")
    lines.append(f"生成日時: {datetime.now().isoformat(timespec='seconds')}")
    lines.append("=" * 64)
    lines.append("")
    lines.append("【セッション】")
    lines.append(f"  モード          : {summary['mode']}")
    lines.append(f"  記録行数        : {summary['rows']} 行 (50Hz)")
    lines.append(f"  記録時間        : {_fmt(summary['duration_s'], 1, ' s')}")
    lines.append(f"  飛行時間        : {_fmt(summary['flight_time_s'], 1, ' s')}")
    lines.append("")

    if math.isfinite(summary["pos_rms_2d_m"]):
        lines.append("【位置制御(閉ループ区間)】")
        lines.append(f"  RMS 誤差 X      : {_fmt(summary['pos_rms_x_m'], 3, ' m')}")
        lines.append(f"  RMS 誤差 Y      : {_fmt(summary['pos_rms_y_m'], 3, ' m')}")
        lines.append(f"  RMS 誤差 2D     : {_fmt(summary['pos_rms_2d_m'], 3, ' m')}")
        lines.append(f"  最大誤差 2D     : {_fmt(summary['pos_max_2d_m'], 3, ' m')}")
        lines.append("")

    ref = yaw.get("reference")
    if ref is not None and yaw["errors"]:
        ref_label = _SOURCE_INFO[ref][0]
        lines.append(f"【ヨー推定(基準: {ref_label})】")
        for stat in yaw["errors"]:
            lines.append(
                f"  {stat.label:<18}: RMS {_fmt(stat.rms_deg, 2, '°'):>9}"
                f" / 最大 {_fmt(stat.max_abs_deg, 2, '°'):>9}"
                f" / ドリフト {_fmt(stat.drift_deg_per_min, 2, '°/min'):>12}"
                f" (n={stat.n})")
        lines.append("")
    if math.isfinite(yaw.get("tracking_rms_deg", math.nan)):
        lines.append("【ヨー指令追従(ヨー制御 ON 区間)】")
        lines.append(f"  RMS 追従誤差    : {_fmt(yaw['tracking_rms_deg'], 2, '°')}")
        lines.append(f"  最大追従誤差    : {_fmt(yaw['tracking_max_deg'], 2, '°')}")
        lines.append("")
    if math.isfinite(yaw.get("nis_mean", math.nan)) or yaw.get("gate_rates"):
        lines.append("【EKF 健全性】")
        lines.append(f"  NIS 平均 / p95 / 最大 : {_fmt(yaw['nis_mean'])} /"
                     f" {_fmt(yaw['nis_p95'])} / {_fmt(yaw['nis_max'])}")
        lines.append(f"  ‖b_m‖ 最大            : {_fmt(yaw['bm_max_ut'], 2, ' µT')}")
        for name, (count, rate) in yaw.get("gate_rates", {}).items():
            if count > 0:
                lines.append(f"  ゲート {name:<12}: {count} 行 ({rate:.1f}%)")
        lines.append("")

    # EKF2 健全性(v6 ログのみ有効値。契約 docs/MAG_AUTOTUNE_DESIGN.md §1.1)
    if (math.isfinite(yaw.get("bm2_max_ut", math.nan))
            or math.isfinite(yaw.get("ekf2_innov_rms_deg", math.nan))):
        lines.append("【EKF2 健全性(v6・シャドー)】")
        lines.append(f"  ‖b_m‖ 最大            : {_fmt(yaw['bm2_max_ut'], 2, ' µT')}")
        lines.append(f"  ヨー観測イノベーション RMS / 最大 :"
                     f" {_fmt(yaw['ekf2_innov_rms_deg'], 2, '°')} /"
                     f" {_fmt(yaw['ekf2_innov_max_deg'], 2, '°')}")
        lines.append(f"  ヨー観測受理率        :"
                     f" {_fmt(yaw['ekf2_fused_rate'], 1, ' %')}")
        for name, (count, rate) in yaw.get("gate2_rates", {}).items():
            if count > 0:
                lines.append(f"  ゲート {name:<12}: {count} 行 ({rate:.1f}%)")
        lines.append("")

    lines.append("【電源】")
    lines.append(f"  電圧 開始→終了  : {_fmt(summary['voltage_start_v'], 2, ' V')}"
                 f" → {_fmt(summary['voltage_end_v'], 2, ' V')}"
                 f" (最低 {_fmt(summary['voltage_min_v'], 2, ' V')})")
    lines.append(f"  低電圧フラグ行  : {summary['low_voltage_rows']} 行"
                 f" (しきい値 {LOW_VOLTAGE_THRESHOLD_V} V)")
    lines.append(f"  電流 平均 / 最大: {_fmt(summary['current_mean_a'], 2, ' A')}"
                 f" / {_fmt(summary['current_max_a'], 2, ' A')}")
    lines.append("")
    lines.append("【通信 / 制御周期】")
    lines.append(f"  レイテンシ 平均 / p95 : {_fmt(summary['latency_mean_ms'], 1, ' ms')}"
                 f" / {_fmt(summary['latency_p95_ms'], 1, ' ms')}")
    lines.append(f"  loop_dt 平均 / 最大   : {_fmt(summary['loop_dt_mean_us'], 0, ' µs')}"
                 f" / {_fmt(summary['loop_dt_max_us'], 0, ' µs')}")
    return lines


# ---------------------------------------------------------------------------
# HTML
# ---------------------------------------------------------------------------

_HTML_STYLE = """
body { background: #ffffff; color: #1f2937; font-family: 'Hiragino Sans',
       'Noto Sans CJK JP', sans-serif; margin: 0; padding: 24px; }
h1 { font-size: 1.4rem; border-bottom: 2px solid #2563eb; padding-bottom: 8px; }
h2 { font-size: 1.1rem; color: #1d4ed8; margin-top: 28px; }
table { border-collapse: collapse; margin: 8px 0 16px; }
th, td { border: 1px solid #d1d5db; padding: 6px 12px; font-size: 0.85rem; }
th { background: #f3f4f6; text-align: left; }
td.num { text-align: right; font-variant-numeric: tabular-nums; }
.figure { margin: 16px 0; }
.figure img { max-width: 100%; border: 1px solid #d1d5db; border-radius: 4px; }
.figure .caption { color: #6b7280; font-size: 0.85rem; margin: 4px 0; }
.warn { color: #b45309; }
.meta { color: #6b7280; font-size: 0.8rem; }
"""


def _html_page(title: str, body: str) -> str:
    return (
        "<!DOCTYPE html>\n<html lang=\"ja\">\n<head>\n<meta charset=\"utf-8\">\n"
        f"<title>{html.escape(title)}</title>\n<style>{_HTML_STYLE}</style>\n"
        f"</head>\n<body>\n{body}\n</body>\n</html>\n"
    )


def _stats_table_html(summary: dict) -> str:
    """統計テーブル群の HTML 断片を作る。"""
    yaw = summary["yaw"]
    parts: list[str] = []

    parts.append("<h2>セッション</h2><table>")
    parts.append(f"<tr><th>モード</th><td>{html.escape(summary['mode'])}</td></tr>")
    parts.append(f"<tr><th>記録行数</th><td class='num'>{summary['rows']}</td></tr>")
    parts.append(f"<tr><th>記録時間</th><td class='num'>{_fmt(summary['duration_s'], 1, ' s')}</td></tr>")
    parts.append(f"<tr><th>飛行時間</th><td class='num'>{_fmt(summary['flight_time_s'], 1, ' s')}</td></tr>")
    parts.append("</table>")

    if math.isfinite(summary["pos_rms_2d_m"]):
        parts.append("<h2>位置制御(閉ループ区間)</h2><table>")
        parts.append("<tr><th>RMS X</th><th>RMS Y</th><th>RMS 2D</th><th>最大 2D</th></tr>")
        parts.append(
            f"<tr><td class='num'>{_fmt(summary['pos_rms_x_m'], 3, ' m')}</td>"
            f"<td class='num'>{_fmt(summary['pos_rms_y_m'], 3, ' m')}</td>"
            f"<td class='num'>{_fmt(summary['pos_rms_2d_m'], 3, ' m')}</td>"
            f"<td class='num'>{_fmt(summary['pos_max_2d_m'], 3, ' m')}</td></tr>")
        parts.append("</table>")

    ref = yaw.get("reference")
    if ref is not None and yaw["errors"]:
        ref_label = _SOURCE_INFO[ref][0]
        parts.append(f"<h2>ヨー推定(基準: {html.escape(ref_label)})</h2><table>")
        parts.append("<tr><th>系統</th><th>RMS [°]</th><th>最大 [°]</th>"
                     "<th>ドリフト [°/min]</th><th>n</th></tr>")
        for stat in yaw["errors"]:
            parts.append(
                f"<tr><td>{html.escape(stat.label)}</td>"
                f"<td class='num'>{_fmt(stat.rms_deg)}</td>"
                f"<td class='num'>{_fmt(stat.max_abs_deg)}</td>"
                f"<td class='num'>{_fmt(stat.drift_deg_per_min, 2)}</td>"
                f"<td class='num'>{stat.n}</td></tr>")
        parts.append("</table>")
        if ref == "madgwick":
            parts.append("<p class='warn'>※ MoCap 真値が無いため Madgwick を基準に"
                         "した相対比較です。</p>")

    if math.isfinite(yaw.get("tracking_rms_deg", math.nan)):
        parts.append("<h2>ヨー指令追従(ON 区間)</h2><table>")
        parts.append(
            f"<tr><th>RMS</th><td class='num'>{_fmt(yaw['tracking_rms_deg'], 2, '°')}</td>"
            f"<th>最大</th><td class='num'>{_fmt(yaw['tracking_max_deg'], 2, '°')}</td></tr>")
        parts.append("</table>")

    parts.append("<h2>電源 / 通信</h2><table>")
    parts.append(
        f"<tr><th>電圧 開始 → 終了(最低)</th><td class='num'>"
        f"{_fmt(summary['voltage_start_v'], 2, ' V')} → "
        f"{_fmt(summary['voltage_end_v'], 2, ' V')}"
        f"({_fmt(summary['voltage_min_v'], 2, ' V')})</td></tr>")
    parts.append(f"<tr><th>低電圧フラグ行</th><td class='num'>{summary['low_voltage_rows']}</td></tr>")
    parts.append(
        f"<tr><th>電流 平均 / 最大</th><td class='num'>"
        f"{_fmt(summary['current_mean_a'], 2, ' A')} / "
        f"{_fmt(summary['current_max_a'], 2, ' A')}</td></tr>")
    parts.append(
        f"<tr><th>レイテンシ 平均 / p95</th><td class='num'>"
        f"{_fmt(summary['latency_mean_ms'], 1, ' ms')} / "
        f"{_fmt(summary['latency_p95_ms'], 1, ' ms')}</td></tr>")
    parts.append(
        f"<tr><th>loop_dt 平均 / 最大</th><td class='num'>"
        f"{_fmt(summary['loop_dt_mean_us'], 0, ' µs')} / "
        f"{_fmt(summary['loop_dt_max_us'], 0, ' µs')}</td></tr>")
    parts.append("</table>")

    if yaw.get("gate_rates"):
        fired = {k: v for k, v in yaw["gate_rates"].items() if v[0] > 0}
        parts.append("<h2>EKF ゲート発火</h2>")
        if fired:
            parts.append("<table><tr><th>ゲート</th><th>行数</th><th>割合</th></tr>")
            for name, (count, rate) in fired.items():
                parts.append(f"<tr><td>{html.escape(name)}</td>"
                             f"<td class='num'>{count}</td>"
                             f"<td class='num'>{rate:.1f}%</td></tr>")
            parts.append("</table>")
        else:
            parts.append("<p>ゲート発火なし(全区間で磁気更新が正常採用)。</p>")

    # EKF2 健全性(v6 ログのみ有効値)
    if (math.isfinite(yaw.get("bm2_max_ut", math.nan))
            or math.isfinite(yaw.get("ekf2_innov_rms_deg", math.nan))):
        parts.append("<h2>EKF2 健全性(v6・シャドー)</h2><table>")
        parts.append(
            f"<tr><th>‖b_m‖ 最大</th><td class='num'>"
            f"{_fmt(yaw['bm2_max_ut'], 2, ' µT')}</td></tr>")
        parts.append(
            f"<tr><th>ヨー観測イノベーション RMS / 最大</th><td class='num'>"
            f"{_fmt(yaw['ekf2_innov_rms_deg'], 2, '°')} / "
            f"{_fmt(yaw['ekf2_innov_max_deg'], 2, '°')}</td></tr>")
        parts.append(
            f"<tr><th>ヨー観測受理率</th><td class='num'>"
            f"{_fmt(yaw['ekf2_fused_rate'], 1, ' %')}</td></tr>")
        parts.append("</table>")
        fired2 = {k: v for k, v in yaw.get("gate2_rates", {}).items()
                  if v[0] > 0}
        if fired2:
            parts.append("<table><tr><th>EKF2 ゲート</th><th>行数</th>"
                         "<th>割合</th></tr>")
            for name, (count, rate) in fired2.items():
                parts.append(f"<tr><td>{html.escape(name)}</td>"
                             f"<td class='num'>{count}</td>"
                             f"<td class='num'>{rate:.1f}%</td></tr>")
            parts.append("</table>")

    return "\n".join(parts)


def generate_report(log: FlightLog, out_dir: str | Path,
                    figure_paths: list[Path] | None = None) -> tuple[Path, Path]:
    """summary.txt と index.html を out_dir に生成する。

    figure_paths を省略した場合は out_dir 内の PNG を番号順に載せる。
    """
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print("\nサマリレポートを生成します...")

    summary = compute_summary(log)

    # summary.txt
    txt_path = out_dir / "summary.txt"
    txt_path.write_text("\n".join(_summary_lines(summary)) + "\n", encoding="utf-8")
    print(f"  保存: {txt_path}")

    # index.html
    if figure_paths is None:
        figure_paths = sorted(out_dir.glob("*.png"))
    body: list[str] = []
    body.append(f"<h1>フライトログレポート: {html.escape(summary['log_name'])}</h1>")
    body.append(f"<p class='meta'>生成日時: {datetime.now().isoformat(timespec='seconds')}"
                f" / 元ファイル: {html.escape(str(log.path))}</p>")
    for warning in log.warnings:
        body.append(f"<p class='warn'>警告: {html.escape(warning)}</p>")
    body.append(_stats_table_html(summary))
    body.append("<h2>グラフ</h2>")
    for path in figure_paths:
        caption = _FIGURE_CAPTIONS.get(path.name, path.stem)
        body.append(
            f"<div class='figure'><div class='caption'>{html.escape(caption)}"
            f" ({html.escape(path.name)})</div>"
            f"<img src='{html.escape(path.name)}' alt='{html.escape(caption)}'></div>")

    html_path = out_dir / "index.html"
    html_path.write_text(
        _html_page(f"フライトログレポート: {summary['log_name']}", "\n".join(body)),
        encoding="utf-8")
    print(f"  保存: {html_path}")
    return txt_path, html_path


# ---------------------------------------------------------------------------
# 複数機(multi)レポート
# ---------------------------------------------------------------------------

def _max_altitude_m(log: FlightLog) -> float:
    """最大高度 [m](推定高度優先、無ければ pos_z)。"""
    for col in ("tlm_altitude_est_m", "pos_z"):
        if log.has(col):
            values = log.df[col].to_numpy(dtype=float)
            finite = values[np.isfinite(values)]
            if finite.size:
                return float(np.max(finite))
    return math.nan


def generate_multi_report(logs: list[FlightLog], out_dir: str | Path) -> Path:
    """複数機グループの index.html を生成する。

    - 全機サマリ表(機体名 / 記録時間 / 飛行時間 / 位置 RMS / 最大高度)
    - 共有図 M01_multi_xy.png(out_dir に生成済みであれば埋め込む)
    - 機体別サブフォルダの index.html へのリンク
      (機体別 index は generate_report で個別に生成する)
    """
    from .multi_plots import MULTI_XY_FILENAME, drone_label  # noqa: PLC0415

    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print("\n複数機レポートを生成します...")

    body: list[str] = []
    group_name = out_dir.name
    body.append(f"<h1>複数機フライトレポート: {html.escape(group_name)}</h1>")
    body.append(f"<p class='meta'>生成日時: "
                f"{datetime.now().isoformat(timespec='seconds')} / 機体数: "
                f"{len(logs)}</p>")
    for log in logs:
        for warning in log.warnings:
            body.append(f"<p class='warn'>警告 [{html.escape(drone_label(log))}]:"
                        f" {html.escape(warning)}</p>")

    # 全機サマリ表(機体別 index.html も同じループで生成する)
    body.append("<h2>全機サマリ</h2><table>")
    body.append("<tr><th>機体名</th><th>記録時間 [s]</th><th>飛行時間 [s]</th>"
                "<th>位置 RMS 2D [m]</th><th>最大高度 [m]</th><th>詳細</th></tr>")
    for log in logs:
        name = drone_label(log)
        sub_dir = out_dir / name
        generate_report(log, sub_dir)  # summary.txt + 機体別 index.html
        summary = compute_summary(log)
        link = f"{name}/index.html"
        body.append(
            f"<tr><td>{html.escape(name)}</td>"
            f"<td class='num'>{_fmt(summary['duration_s'], 1)}</td>"
            f"<td class='num'>{_fmt(summary['flight_time_s'], 1)}</td>"
            f"<td class='num'>{_fmt(summary['pos_rms_2d_m'], 3)}</td>"
            f"<td class='num'>{_fmt(_max_altitude_m(log), 2)}</td>"
            f"<td><a href='{html.escape(link)}' style='color:#1d4ed8'>"
            f"機体別レポート</a></td></tr>")
    body.append("</table>")

    # 共有 XY 図(multi_plots.fig_multi_xy が out_dir に生成済みなら埋め込む)
    m01 = out_dir / MULTI_XY_FILENAME
    if m01.is_file():
        caption = _FIGURE_CAPTIONS.get(m01.name, m01.stem)
        body.append("<h2>共有図</h2>")
        body.append(
            f"<div class='figure'><div class='caption'>{html.escape(caption)}"
            f" ({html.escape(m01.name)})</div>"
            f"<img src='{html.escape(m01.name)}' alt='{html.escape(caption)}'>"
            f"</div>")

    html_path = out_dir / "index.html"
    html_path.write_text(
        _html_page(f"複数機フライトレポート: {group_name}", "\n".join(body)),
        encoding="utf-8")
    print(f"  保存: {html_path}")
    return html_path


# ---------------------------------------------------------------------------
# 2 ログ比較(ヨー安定性)
# ---------------------------------------------------------------------------

def _fig_compare_yaw(log_a: FlightLog, log_b: FlightLog, out_dir: Path) -> Path | None:
    """比較図: 両ログのヨー誤差を重ね描き(A=細実線 / B=太破線)。

    実線と破線が重なると区別しづらいため、B は太線+高不透明度+
    短めのダッシュで強調し、A は細実線・低不透明度で下に敷く
    (描画順も A → B とし、B が上に乗る)。
    """
    keys = ("madgwick", "ekf", "ekf2", "gyro_int")
    has_any = False
    fig, ax = new_fig(figsize=(13.0, 6.0))
    styles = (
        (log_a, "A", {"linestyle": "-", "linewidth": 0.9, "alpha": 0.55}),
        (log_b, "B", {"linestyle": (0, (4, 1.5)), "linewidth": 1.8,
                      "alpha": 0.95}),
    )
    for log, suffix, style in styles:
        for key in keys:
            col = f"yaw_err_{key}_deg"
            if col not in log.df.columns or not log.df[col].notna().any():
                continue
            label_name, color = _SOURCE_INFO[key]
            ax.plot(log.t, log.df[col], color=color,
                    label=f"[{suffix}] {label_name}", **style)
            has_any = True
    if not has_any:
        import matplotlib.pyplot as plt  # noqa: PLC0415
        plt.close(fig)
        return None
    ax.axhline(y=0, color="gray", linestyle="--", alpha=0.6)
    ax.set_title(f"ヨー誤差比較  A={log_a.name} / B={log_b.name}", fontsize=13)
    ax.set_xlabel("時間 [s]", fontsize=11)
    ax.set_ylabel("誤差 [deg]", fontsize=10)
    styled_legend(ax, loc="best", ncol=2, fontsize=8)
    return save_fig(fig, out_dir, "compare_yaw_error.png")


def generate_comparison(log_a: FlightLog, log_b: FlightLog,
                        out_dir: str | Path) -> Path:
    """2 ログの比較レポート(ヨー安定性+位置保持XY。comparison.html)を生成する。"""
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    print(f"\n比較レポートを生成します: A={log_a.name} / B={log_b.name}")

    stats_a = compute_yaw_stats(log_a)
    stats_b = compute_yaw_stats(log_b)
    fig_path = _fig_compare_yaw(log_a, log_b, out_dir)

    def _stat_map(stats: dict) -> dict[str, object]:
        return {s.key: s for s in stats["errors"]}

    map_a, map_b = _stat_map(stats_a), _stat_map(stats_b)
    keys = [k for k in ("madgwick", "ekf", "ekf2", "gyro_int")
            if k in map_a or k in map_b]

    body: list[str] = []
    body.append("<h1>ヨー安定性比較レポート</h1>")
    body.append(f"<p class='meta'>A: {html.escape(str(log_a.path))}<br>"
                f"B: {html.escape(str(log_b.path))}<br>"
                f"生成日時: {datetime.now().isoformat(timespec='seconds')}</p>")

    ref_a = stats_a.get("reference")
    ref_b = stats_b.get("reference")
    ref_note = []
    for name, ref in (("A", ref_a), ("B", ref_b)):
        if ref is not None:
            ref_note.append(f"{name}: {_SOURCE_INFO[ref][0]}")
    body.append(f"<p>誤差基準 — {' / '.join(ref_note) if ref_note else 'なし'}</p>")
    if ref_a != ref_b:
        body.append("<p class='warn'>※ 2 ログで誤差基準が異なるため単純比較には"
                    "注意してください。</p>")

    body.append("<h2>ヨー誤差統計</h2><table>")
    body.append("<tr><th rowspan='2'>系統</th>"
                "<th colspan='3'>A: " + html.escape(log_a.name) + "</th>"
                "<th colspan='3'>B: " + html.escape(log_b.name) + "</th></tr>")
    body.append("<tr><th>RMS [°]</th><th>最大 [°]</th><th>ドリフト [°/min]</th>"
                "<th>RMS [°]</th><th>最大 [°]</th><th>ドリフト [°/min]</th></tr>")
    for key in keys:
        label = _SOURCE_INFO[key][0]
        cells = [f"<td>{html.escape(label)}</td>"]
        for stat_map in (map_a, map_b):
            stat = stat_map.get(key)
            if stat is None:
                cells.append("<td class='num'>—</td>" * 3)
            else:
                cells.append(
                    f"<td class='num'>{_fmt(stat.rms_deg)}</td>"
                    f"<td class='num'>{_fmt(stat.max_abs_deg)}</td>"
                    f"<td class='num'>{_fmt(stat.drift_deg_per_min, 2)}</td>")
        body.append("<tr>" + "".join(cells) + "</tr>")
    body.append("</table>")

    # 位置保持(XY)統計 — ヨー制御あり/なしの位置保持品質比較用。
    # どちらのログにも位置データが無い(Posture 等)場合は表を出さない
    pos_a = compute_position_stats(log_a)
    pos_b = compute_position_stats(log_b)
    if any(math.isfinite(v) for v in (*pos_a.values(), *pos_b.values())):
        body.append("<h2>位置保持(XY)統計</h2><table>")
        body.append("<tr><th>指標</th><th>A</th><th>B</th></tr>")
        body.append(
            f"<tr><td>飛行時間 [s]</td>"
            f"<td class='num'>{_fmt(log_a.flight_time_s(), 1)}</td>"
            f"<td class='num'>{_fmt(log_b.flight_time_s(), 1)}</td></tr>")
        for label, key in (
            ("RMS 誤差 X [m]", "pos_rms_x_m"),
            ("RMS 誤差 Y [m]", "pos_rms_y_m"),
            ("RMS 誤差 2D [m]", "pos_rms_2d_m"),
            ("最大 2D [m]", "pos_max_2d_m"),
        ):
            body.append(
                f"<tr><td>{html.escape(label)}</td>"
                f"<td class='num'>{_fmt(pos_a[key], 3)}</td>"
                f"<td class='num'>{_fmt(pos_b[key], 3)}</td></tr>")
        body.append("</table>")
        body.append("<p class='meta'>位置統計は閉ループ制御中の全行"
                    "(離陸過渡を含む)。誤差 = 目標 − フィルタ済み実位置。"
                    "飛行時間・目標・ヨー以外の操作を揃えた飛行同士で"
                    "比較すること。</p>")

    # EKF2 行(bm2 等)は少なくとも一方のログが v6 のときのみ載せる
    has_ekf2 = any(
        math.isfinite(s.get("bm2_max_ut", math.nan))
        or math.isfinite(s.get("ekf2_innov_rms_deg", math.nan))
        for s in (stats_a, stats_b))
    ekf_rows: list[tuple[str, str, int, str]] = [
        ("NIS 平均", "nis_mean", 2, ""),
        ("NIS p95", "nis_p95", 2, ""),
        ("‖b_m‖ 最大", "bm_max_ut", 2, " µT"),
        ("ヨー追従 RMS", "tracking_rms_deg", 2, "°"),
    ]
    if has_ekf2:
        ekf_rows += [
            ("‖b_m‖ 最大(EKF2)", "bm2_max_ut", 2, " µT"),
            ("EKF2 イノベーション RMS", "ekf2_innov_rms_deg", 2, "°"),
            ("EKF2 ヨー観測受理率", "ekf2_fused_rate", 1, " %"),
        ]
    body.append("<h2>EKF 健全性</h2><table>")
    body.append("<tr><th>指標</th><th>A</th><th>B</th></tr>")
    for label, key, digits, unit in ekf_rows:
        body.append(
            f"<tr><td>{html.escape(label)}</td>"
            f"<td class='num'>{_fmt(stats_a.get(key, math.nan), digits, unit)}</td>"
            f"<td class='num'>{_fmt(stats_b.get(key, math.nan), digits, unit)}</td></tr>")
    body.append("</table>")

    gate_names = sorted(set(stats_a.get("gate_rates", {})) | set(stats_b.get("gate_rates", {})))
    fired = [n for n in gate_names
             if stats_a.get("gate_rates", {}).get(n, (0, 0.0))[0] > 0
             or stats_b.get("gate_rates", {}).get(n, (0, 0.0))[0] > 0]
    if fired:
        body.append("<h2>ゲート発火率</h2><table>")
        body.append("<tr><th>ゲート</th><th>A</th><th>B</th></tr>")
        for name in fired:
            rate_a = stats_a.get("gate_rates", {}).get(name, (0, 0.0))[1]
            rate_b = stats_b.get("gate_rates", {}).get(name, (0, 0.0))[1]
            body.append(f"<tr><td>{html.escape(name)}</td>"
                        f"<td class='num'>{rate_a:.1f}%</td>"
                        f"<td class='num'>{rate_b:.1f}%</td></tr>")
        body.append("</table>")

    if fig_path is not None:
        body.append("<h2>ヨー誤差重ね描き</h2>")
        body.append(f"<div class='figure'><img src='{html.escape(fig_path.name)}'"
                    f" alt='ヨー誤差比較'></div>")

    html_path = out_dir / "comparison.html"
    html_path.write_text(
        _html_page(f"ヨー安定性比較: {log_a.name} vs {log_b.name}", "\n".join(body)),
        encoding="utf-8")
    print(f"  保存: {html_path}")
    return html_path
