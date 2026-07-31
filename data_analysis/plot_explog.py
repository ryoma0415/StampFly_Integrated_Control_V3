#!/usr/bin/env python3
"""StampFly Experiment 計測ログ（explog CSV）のグラフ化.

pc_server の Experiment モードが出力する
pc_server/data/exp_logs/explog_<YYYYMMDD_HHMMSS>.csv（≈25Hz）を読み、
ヨー推定3系統の比較・FF補正の説明力・EKF健全性などを PNG 6〜7枚 + summary.txt に
まとめる。必要な列が無い / 全欠損の図は自動でスキップする。

出力図:
    01_yaw_comparison.png   ヨー3系統（±180°ラップ絶対値）+ 磁気方位(FF補正後) + duty
    02_yaw_error.png        基準に対する各系統の誤差時系列（RMS/最大を凡例に）
    03_current_duty.png     電流 / バッテリ電圧 / duty
    04_mag_ff.png           校正済み磁場変化 Δb_cal と FF推定外乱 db_hat の重畳
    05_ekf_diagnostics.png  NIS（閾値 2.0 / 5.99 / 13.8）+ ffg 8ゲートのタイムライン
    06_ff_status.png        ff_status（ff_mode + フラグビット）のタイムライン
    07_flow.png             オプティカルフロー: 速度 / SQUAL・dt / ToF高度 /
                            flow_status ビット（v2 ログのみ。列が無ければスキップ）

アニメーション（7枠レイアウト・スマホ動画同期）:
    メニュー [2]/[3] または --animation で、時系列6枠をスクロール窓表示
    （既定5秒幅・--window または対話メニューで 5〜60 秒に変更可。
    x軸は常に現在時刻 t を中央にした [t-窓幅/2, t+窓幅/2]。線は時刻<=t の
    履歴のみ描画し、右半分（未来）は常に空白。開始直後は左半分が負時間の
    空白でドットだけが中央にあり、時間とともに履歴が左へ流れる。
    y軸は全期間から固定）した MP4（1920x1080, libx264/yuv420p）を生成する。
    各トレースには現在時刻の補間値位置に線と同色・黒フチの大きなドットを重畳
    （常に x軸中央）。
    スマホ動画（正方形前提。
    非正方形は中央クロップ）は「LED がマゼンタに変わった瞬間 = 計測開始(t_s=0)」
    でカット済みの前提で先頭から同期する。動画は
    ../pc_server/data/exp_logs/videos/ に置くと対話メニューから番号選択できる。
    アニメ生成時は、同一レイアウトで全期間を俯瞰する静止画ボード
    explog_<stem>_overview.png も同時出力する（--start/--end に関わらず全期間）。

使い方:
    python plot_explog.py                    # 対話: [1]静止画 [2]アニメ(動画同期) [3]アニメ(動画なし)
    python plot_explog.py explog_xxx.csv [-o 出力ディレクトリ]      # 静止画（従来）
    python plot_explog.py explog_xxx.csv --animation                # アニメ（動画なし, 20fps）
    python plot_explog.py explog_xxx.csv --video 動画.mp4 [--fps N] [--start S] [--end E] [--window W]
    引数を省略すると explog CSV を新しい順に一覧表示し、番号で1つ選ぶ
    （Enter=最新 / q=中止）。出力先は既定で data_analysis/graphs/explog_<stamp>/。

同じ stamp の explog_<stamp>_meta.json があれば、ff_state
（プロファイル名 / ff_mode / est_mode）を図タイトルとサマリに表示する。

依存: numpy, matplotlib（アニメ MP4 出力に ffmpeg、動画同期時のみ opencv-python）。
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.font_manager as fm

# 日本語フォント（macOS優先）。見つからなければ既定フォントのまま（英字は問題なし）。
_INSTALLED = {f.name for f in fm.fontManager.ttflist}
for _jp in ("Hiragino Sans", "Hiragino Maru Gothic Pro", "YuGothic", "Yu Gothic",
            "Noto Sans CJK JP", "IPAexGothic", "Apple SD Gothic Neo", "Arial Unicode MS"):
    if _jp in _INSTALLED:
        plt.rcParams["font.family"] = _jp
        break
plt.rcParams["axes.unicode_minus"] = False

# ライトテーマ（白背景）
plt.rcParams.update({
    "figure.facecolor": "#ffffff",
    "savefig.facecolor": "#ffffff",
    "axes.facecolor": "#ffffff",
    "axes.edgecolor": "#444444",
    "axes.labelcolor": "#222222",
    "axes.titlecolor": "#222222",
    "text.color": "#222222",
    "xtick.color": "#444444",
    "ytick.color": "#444444",
    "grid.color": "#999999",
    "legend.facecolor": "#ffffff",
    "legend.edgecolor": "#cccccc",
    "legend.labelcolor": "#222222",
})

EXPLOG_GLOB = "explog_*.csv"

# ヨー3系統（列名, 表示ラベル, 色）
YAW_STREAMS = [
    ("yaw_gyro_int_deg", "ジャイロ積分", "#f59e0b"),
    ("yaw_est_deg", "EKF推定 (FF+EKF)", "#0284c7"),
    ("yaw_madgwick_deg", "Madgwick", "#db2777"),
]

# ffg（EKFゲート状態ビット, ファーム側 yaw_estimator_kf と一致）
GATE_BITS = [
    ("R膨張(soft)", "#d97706"),
    ("NIS棄却", "#ef4444"),
    ("norm棄却", "#f97316"),
    ("z棄却", "#a855f7"),
    ("tilt>25°", "#64748b"),
    ("b_m凍結", "#dc2626"),
    ("ドリフト警告", "#0284c7"),
    ("再捕捉中", "#16a34a"),
]

# ff_status: 下位2bit = ff_mode(0=off,1=A,2=B)、bit2-6 = フラグ
FF_MODE_MASK = 0x03
FF_MODE_NAMES = {0: "off", 1: "A", 2: "B", 3: "?(3)"}
FF_STATUS_FLAGS = [  # (bit, ラベル, 色)
    (2, "est=EKF", "#0284c7"),
    (3, "アンカー有効", "#16a34a"),
    (4, "FF係数ロード済", "#7c3aed"),
    (5, "Yaw制御アクティブ", "#ea580c"),
    (6, "磁気fresh", "#db2777"),
]

NIS_EXPECT = 2.0
NIS_SOFT = 5.99
NIS_REJECT = 13.8

# flow_status ビット定義（protocol TlmState.FLOW_STATUS_* と一致。契約 §1.1）
FLOW_STATUS_BITS = [  # (bit, ラベル, 色)
    (0, "SENSOR_OK", "#16a34a"),
    (1, "BURST_OK", "#0284c7"),
    (2, "VEL_VALID", "#7c3aed"),
    (3, "RANGE_VALID", "#d97706"),
    (4, "SQUAL_OK", "#db2777"),
    (5, "INIT_RETRY", "#ef4444"),
]
FLOW_VEL_VALID_BIT = 2


# ---------------------------------------------------------------- load --------
def _f(value) -> float:
    """空欄/欠損は NaN。"""
    if value is None or value == "":
        return math.nan
    try:
        return float(value)
    except ValueError:
        return math.nan


def load_explog(path: Path) -> dict:
    rows = list(csv.DictReader(path.open(encoding="utf-8")))
    if not rows:
        sys.exit(f"空のCSVです: {path}")
    out: dict = {}
    for key in rows[0].keys():
        if key is None:
            continue
        if key == "motors":  # モーター名の文字列列（"FL+FR+RL+RR" 等）
            out[key] = np.array([r.get(key) or "" for r in rows], dtype=object)
            continue
        out[key] = np.array([_f(r.get(key, "")) for r in rows], dtype=float)
    out["_n"] = len(rows)
    return out


def col(data: dict, name: str) -> np.ndarray:
    """列を取得。無ければ全 NaN（自動スキップ用）。"""
    v = data.get(name)
    if v is None or v.dtype == object:
        return np.full(data["_n"], np.nan)
    return v


def usable(data: dict, *names: str) -> bool:
    """全列が存在し、かつ有限値を1つ以上持つか。"""
    return all(np.isfinite(col(data, n)).any() for n in names)


def meta_path_for(log: Path) -> Path:
    return log.with_name(log.stem + "_meta.json")


def load_meta(log: Path) -> dict | None:
    mp = meta_path_for(log)
    if not mp.is_file():
        return None
    try:
        return json.loads(mp.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as e:
        print(f"meta JSON の読み込みに失敗（無視して続行）: {mp} ({e})")
        return None


def ff_state_line(meta: dict | None) -> str:
    """meta の ff_state を1行サマリに（無ければ空文字）。"""
    ff = (meta or {}).get("ff_state") or {}
    if not ff:
        return ""
    parts = []
    if ff.get("name"):
        parts.append(f"プロファイル: {ff['name']}")
    if ff.get("ff_mode") is not None:
        m = ff["ff_mode"]
        parts.append(f"ff_mode={FF_MODE_NAMES.get(m, m) if isinstance(m, int) else m}")
    if ff.get("est_mode") is not None:
        parts.append(f"est_mode={ff['est_mode']}")
    return " / ".join(parts)


# --------------------------------------------------------------- angles -------
def wrap180(deg: np.ndarray) -> np.ndarray:
    """角度差を [-180, 180) に畳む。"""
    return (deg + 180.0) % 360.0 - 180.0


def unwrap_deg(a: np.ndarray) -> np.ndarray:
    """NaN を保ったまま有限値だけを unwrap した deg 系列を返す。"""
    out = np.full(a.shape, np.nan)
    fin = np.isfinite(a)
    if fin.sum() >= 2:
        out[fin] = np.rad2deg(np.unwrap(np.deg2rad(a[fin])))
    elif fin.any():
        out[fin] = a[fin]
    return out


def start_offset(a: np.ndarray, n: int = 10) -> float:
    """開始 n サンプル（有限値のみ）の平均。相対表示の基準。"""
    fin = a[np.isfinite(a)]
    if fin.size == 0:
        return math.nan
    return float(np.mean(fin[:n]))


def relative(a: np.ndarray) -> tuple[np.ndarray, float]:
    """unwrap して開始時点オフセットを引いた相対ヨー系列と、そのオフセット。"""
    u = unwrap_deg(a)
    off = start_offset(u)
    return u - off, off


def circ_mean_deg(a: np.ndarray) -> float:
    """円周平均 [deg]。±180 付近でも破綻しない平均角。"""
    fin = a[np.isfinite(a)]
    if fin.size == 0:
        return math.nan
    r = np.deg2rad(fin)
    return math.degrees(math.atan2(float(np.mean(np.sin(r))),
                                   float(np.mean(np.cos(r)))))


def wrapped_plot_series(t: np.ndarray, deg: np.ndarray,
                        jump_deg: float = 180.0) -> tuple[np.ndarray, np.ndarray]:
    """±180° ラップ表示用の (t, y)。

    wrap180 した系列の隣接差が jump_deg を超える箇所（ラップ跨ぎ）に
    NaN 点を挿入し、プロット時に縦線が入らないよう線を切る。
    """
    w = wrap180(deg)
    if len(w) < 2:
        return t, w
    d = np.abs(np.diff(w))
    jumps = np.where(np.isfinite(d) & (d > jump_deg))[0]
    if jumps.size == 0:
        return t, w
    t2 = np.insert(t.astype(float), jumps + 1, (t[jumps] + t[jumps + 1]) / 2.0)
    y2 = np.insert(w, jumps + 1, np.nan)
    return t2, y2


def tilt_deg_series(data: dict) -> np.ndarray:
    """チルト角 [deg] = acos(cos(roll)·cos(pitch))。roll/pitch 欠損は NaN。"""
    r = np.deg2rad(col(data, "roll_deg"))
    p = np.deg2rad(col(data, "pitch_deg"))
    c = np.clip(np.cos(r) * np.cos(p), -1.0, 1.0)
    return np.degrees(np.arccos(c))


def mag_yaw_deg(data: dict, subtract_ff: bool, tilt_mask_deg: float = 15.0) -> np.ndarray:
    """磁気方位 [deg]（ファーム yaw_estimation と同一規約）。

    ファーム実装（firmware_stampfly/src/yaw_estimation/）:
      - FF補正は水平2軸のみ: b_corr = (bx_cal − db_hat_x, by_cal − db_hat_y, bz_cal)
      - levelMagVectorBody(angle_utils.hpp): 独自符号のチルト補償
            lx = mx·cp + mz·sp
            ly = mx·sr·sp + my·cr + mz·sr·cp
      - yaw_mag = atan2(ly, lx)  （yaw_estimator.cpp:133。ψ とともに増える CCW 規約）
    tilt > tilt_mask_deg の区間は NaN でマスク（レベル化の信頼性低下）。
    """
    bx = col(data, "bx_cal").copy()
    by = col(data, "by_cal").copy()
    bz = col(data, "bz_cal")
    if subtract_ff:
        bx = bx - np.nan_to_num(col(data, "db_hat_x_ut"))
        by = by - np.nan_to_num(col(data, "db_hat_y_ut"))
    r = np.deg2rad(col(data, "roll_deg"))
    p = np.deg2rad(col(data, "pitch_deg"))
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    lx = bx * cp + bz * sp
    ly = bx * sr * sp + by * cr + bz * sr * cp
    yaw = np.degrees(np.arctan2(ly, lx))
    tilt = tilt_deg_series(data)
    yaw[np.isfinite(tilt) & (tilt > tilt_mask_deg)] = np.nan
    return yaw


# --------------------------------------------------------------- derive -------
def time_axis(data: dict) -> np.ndarray:
    """時間軸 [s]（開始=0）。exp_elapsed_ms 優先、無ければ t_s、最後は行番号/25Hz。"""
    el = col(data, "exp_elapsed_ms")
    if np.isfinite(el).any():
        return (el - np.nanmin(el)) / 1000.0
    ts = col(data, "t_s")
    if np.isfinite(ts).any():
        return ts - np.nanmin(ts)
    return np.arange(data["_n"]) / 25.0


def motor_on_mask(data: dict) -> np.ndarray:
    """モーターON区間（duty_cmd > 0、mask 列があれば併用）。"""
    duty = np.nan_to_num(col(data, "duty_cmd"))
    on = duty > 1e-4
    mm = col(data, "motors_mask")
    if np.isfinite(mm).any():
        on &= np.nan_to_num(mm) > 0
    return on


def sample_rate(t: np.ndarray) -> float:
    dt = np.diff(t[np.isfinite(t)])
    dt = dt[dt > 0]
    return float(1.0 / np.median(dt)) if dt.size else math.nan


# -------------------------------------------------------------- figures -------
def save_fig(fig, out: Path, name: str) -> None:
    fig.tight_layout()
    fig.savefig(out / name, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  出力: {name}")


def _motor_spans(ax, t: np.ndarray, on: np.ndarray) -> None:
    """モーターON区間を薄い赤帯で示す。"""
    onl = on.astype(int)
    edges = np.diff(np.concatenate([[0], onl, [0]]))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0] - 1
    label_done = False
    for s, e in zip(starts, ends):
        if s >= len(t):
            continue
        e = min(e, len(t) - 1)
        ax.axvspan(t[s], t[e], color="#ef4444", alpha=0.10,
                   label=None if label_done else "モーターON")
        label_done = True


def _duty_panel(ax, t: np.ndarray, data: dict, on: np.ndarray) -> None:
    """下段共通: duty_cmd + ON区間帯。"""
    _motor_spans(ax, t, on)
    ax.plot(t, col(data, "duty_cmd"), "-", lw=1.2, color="#475569", label="duty_cmd")
    ax.set_ylabel("duty")
    ax.set_xlabel("時間 [s]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)


def fig01_yaw_comparison(data: dict, t: np.ndarray, on: np.ndarray,
                         out: Path, tsuffix: str) -> bool:
    streams = [(k, lab, c) for k, lab, c in YAW_STREAMS if usable(data, k)]
    if not streams:
        print("  スキップ: 01_yaw_comparison（ヨー列が1つも無い/全欠損）")
        return False
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7.5), sharex=True, gridspec_kw={"height_ratios": [3, 1]})
    _motor_spans(ax, t, on)
    for key, lab, color in streams:
        tw, yw = wrapped_plot_series(t, col(data, key))
        ax.plot(tw, yw, "-", lw=1.5, color=color, label=f"{lab} ({key})")

    # 第4トレース: FF補正後の磁気方位（ファーム規約, 開始10サンプルで EKF に位置合わせ）
    if usable(data, "bx_cal", "by_cal", "bz_cal") and usable(data, "yaw_est_deg"):
        ym = mag_yaw_deg(data, subtract_ff=True, tilt_mask_deg=15.0)
        ye = col(data, "yaw_est_deg")
        both = np.isfinite(ym) & np.isfinite(ye)
        idx = np.where(both)[0][:10]
        if idx.size:
            off = circ_mean_deg(wrap180(ye[idx] - ym[idx]))
            tw, yw = wrapped_plot_series(t, ym + off)
            ax.plot(tw, yw, ":", lw=1.0, color="#16a34a", alpha=0.75,
                    label="磁気方位(FF補正後, 開始点合わせ) [tilt>15°マスク]")

    yr = col(data, "yaw_ref_deg")
    if np.isfinite(yr).any():
        tw, yw = wrapped_plot_series(t, yr)
        ax.plot(tw, yw, "--", lw=1.0, color="#111111", alpha=0.6, label="目標 yaw_ref")
    ax.axhline(0, ls=":", lw=0.8, color="#6b7280")
    ax.set_ylim(-190, 190)
    ax.set_yticks(np.arange(-180, 181, 90))
    ax.set_ylabel("Yaw [deg]（±180° ラップ・絶対値）")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_title("① ヨー3系統の比較（±180° ラップ表示・UIと同じ絶対値）" + tsuffix,
                 fontsize=12)
    _duty_panel(ax2, t, data, on)
    save_fig(fig, out, "01_yaw_comparison.png")
    return True


def fig02_yaw_error(data: dict, t: np.ndarray, on: np.ndarray,
                    out: Path, tsuffix: str) -> bool:
    streams = [(k, lab, c) for k, lab, c in YAW_STREAMS if usable(data, k)]
    if not streams:
        print("  スキップ: 02_yaw_error（ヨー列が1つも無い/全欠損）")
        return False
    has_gyro = usable(data, "yaw_gyro_int_deg")
    nrows = 2 if has_gyro and len(streams) >= 2 else 1
    fig, axes = plt.subplots(nrows, 1, figsize=(13, 4.0 * nrows), sharex=True,
                             squeeze=False)
    axes = axes[:, 0]

    def _stats_label(lab: str, err: np.ndarray) -> str:
        fin = err[np.isfinite(err)]
        if fin.size == 0:
            return lab
        rms = float(np.sqrt(np.mean(fin ** 2)))
        return f"{lab}  RMS {rms:.2f}° / 最大 {np.max(np.abs(fin)):.2f}°"

    # 上段: 各系統の「自分の開始10サンプル平均」からのずれ（＝ドリフト）
    ax = axes[0]
    _motor_spans(ax, t, on)
    for key, lab, color in streams:
        rel, _ = relative(col(data, key))
        ax.plot(t, rel, "-", lw=1.3, color=color, label=_stats_label(lab, rel))
    ax.axhline(0, ls=":", lw=0.8, color="#6b7280")
    ax.set_ylabel("誤差 [deg]")
    ax.grid(alpha=0.3)
    ax.legend(loc="best", fontsize=8)
    ax.set_title("② ヨー誤差: 上段 = 開始10サンプル平均基準（各系統のドリフト）" + tsuffix,
                 fontsize=12)

    # 下段: ジャイロ積分を基準にした差（磁気系統の相対誤差）
    if nrows == 2:
        ax2 = axes[1]
        _motor_spans(ax2, t, on)
        ref, _ = relative(col(data, "yaw_gyro_int_deg"))
        for key, lab, color in streams:
            if key == "yaw_gyro_int_deg":
                continue
            rel, _ = relative(col(data, key))
            err = wrap180(rel - ref)
            ax2.plot(t, err, "-", lw=1.3, color=color, label=_stats_label(lab, err))
        ax2.axhline(0, ls=":", lw=0.8, color="#6b7280")
        ax2.set_ylabel("誤差 [deg]")
        ax2.grid(alpha=0.3)
        ax2.legend(loc="best", fontsize=8)
        ax2.set_title("下段 = ジャイロ積分基準（短時間はジャイロ積分が真値に近い）",
                      fontsize=10)
    axes[-1].set_xlabel("時間 [s]")
    save_fig(fig, out, "02_yaw_error.png")
    return True


def fig03_current_duty(data: dict, t: np.ndarray, on: np.ndarray,
                       out: Path, tsuffix: str) -> bool:
    if not (usable(data, "current_a") or usable(data, "vbat_v")
            or usable(data, "duty_cmd")):
        print("  スキップ: 03_current_duty（current_a/vbat_v/duty_cmd が全て無い/全欠損）")
        return False
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [2, 1]})
    _motor_spans(ax, t, on)
    ax.plot(t, col(data, "current_a"), "-", lw=1.3, color="#16a34a", label="総電流 [A]")
    ax.set_ylabel("電流 [A]", color="#16a34a")
    ax.tick_params(axis="y", labelcolor="#16a34a")
    ax.grid(alpha=0.3)
    axv = ax.twinx()
    axv.plot(t, col(data, "vbat_v"), "-", lw=1.2, color="#0284c7", label="vbat [V]")
    axv.set_ylabel("バッテリ電圧 [V]", color="#0284c7")
    axv.tick_params(axis="y", labelcolor="#0284c7")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = axv.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, loc="best", fontsize=8)
    ax.set_title("③ 電流・バッテリ電圧・duty" + tsuffix, fontsize=12)
    _duty_panel(ax2, t, data, on)
    save_fig(fig, out, "03_current_duty.png")
    return True


def fig04_mag_ff(data: dict, t: np.ndarray, on: np.ndarray,
                 out: Path, tsuffix: str) -> bool:
    if not usable(data, "bx_cal", "by_cal", "bz_cal"):
        print("  スキップ: 04_mag_ff（b*_cal が無い/全欠損）")
        return False
    has_ff = usable(data, "db_hat_x_ut") or usable(data, "db_hat_y_ut")
    axes_spec = [
        ("x", col(data, "bx_cal"), col(data, "db_hat_x_ut"), "#2563eb"),
        ("y", col(data, "by_cal"), col(data, "db_hat_y_ut"), "#16a34a"),
        ("z", col(data, "bz_cal"), None, "#dc2626"),  # FF は水平2軸のみ
    ]
    fig, axarr = plt.subplots(3, 1, figsize=(13, 9.5), sharex=True)
    for ax, (a, bcal, dbh, color) in zip(axarr, axes_spec):
        _motor_spans(ax, t, on)
        base = start_offset(bcal)
        db = bcal - base
        ax.plot(t, db, "-", lw=1.3, color=color,
                label=f"Δb{a}_cal（開始基準 {base:.1f}µT からの変化）")
        if dbh is not None and np.isfinite(dbh).any():
            ax.plot(t, dbh, "--", lw=1.3, color="#111111", alpha=0.85,
                    label=f"db_hat_{a}（FF推定外乱）")
            resid = db - dbh
            ax.plot(t, resid, "-", lw=1.0, color="#7c3aed", alpha=0.8,
                    label=f"残差 Δb{a} − db_hat_{a}")
        elif a != "z":
            ax.text(0.01, 0.05, "db_hat 列なし/全欠損", transform=ax.transAxes,
                    fontsize=8, color="#6b7280")
        ax.axhline(0, ls=":", lw=0.8, color="#6b7280")
        ax.set_ylabel(f"B_{a} [µT]")
        ax.grid(alpha=0.3)
        ax.legend(loc="best", fontsize=8)
    axarr[-1].set_xlabel("時間 [s]")
    ttl = "④ 磁場変化 Δb_cal と FF補正 db_hat（一致するほど FF が外乱を説明）"
    if not has_ff:
        ttl = "④ 磁場変化 Δb_cal（db_hat 無し）"
    axarr[0].set_title(ttl + tsuffix, fontsize=12)
    save_fig(fig, out, "04_mag_ff.png")
    return True


def fig05_ekf_diagnostics(data: dict, t: np.ndarray, on: np.ndarray,
                          out: Path, tsuffix: str) -> bool:
    has_nis = usable(data, "nis")
    has_ffg = usable(data, "ffg")
    if not (has_nis or has_ffg):
        print("  スキップ: 05_ekf_diagnostics（nis/ffg が無い/全欠損）")
        return False
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 8), sharex=True, gridspec_kw={"height_ratios": [2, 2]})

    if has_nis:
        nis = col(data, "nis")
        _motor_spans(ax, t, on)
        ax.plot(t, nis, "-", lw=0.9, color="#0284c7", label="NIS")
        ax.axhline(NIS_SOFT, ls="--", color="#d97706", lw=1,
                   label=f"χ²(2) 95%={NIS_SOFT}（R膨張）")
        ax.axhline(NIS_REJECT, ls="--", color="#ef4444", lw=1,
                   label=f"χ²(2) 99.9%={NIS_REJECT}（棄却）")
        ax.axhline(NIS_EXPECT, ls=":", color="#6b7280", lw=1,
                   label=f"期待値≈{NIS_EXPECT}")
        fin = nis[np.isfinite(nis)]
        top = np.percentile(fin, 99) if fin.size else 15.0
        ax.set_ylim(0, max(15.0, float(top) * 1.2))
        ax.set_ylabel("NIS")
        ax.grid(alpha=0.3)
        ax.legend(loc="upper right", fontsize=8, ncol=2)
    else:
        ax.text(0.5, 0.5, "nis 列なし/全欠損", ha="center", va="center",
                transform=ax.transAxes, color="#6b7280")
    ax.set_title("⑤ EKF健全性: NIS と ffg 8ゲート発火タイムライン" + tsuffix, fontsize=12)

    if has_ffg:
        ffg = np.nan_to_num(col(data, "ffg")).astype(int)
        for b, (name, color) in enumerate(GATE_BITS):
            active = ((ffg >> b) & 1).astype(bool)
            ax2.fill_between(t, b + 0.1, b + 0.9, where=active, color=color, step="mid")
        ax2.set_yticks([b + 0.5 for b in range(len(GATE_BITS))])
        ax2.set_yticklabels([g[0] for g in GATE_BITS], fontsize=8)
        ax2.set_ylim(0, len(GATE_BITS))
        ax2.grid(alpha=0.3, axis="x")
        ax2.set_title("ゲート発火（帯 = そのゲートが立っている区間）", fontsize=10)
    else:
        ax2.text(0.5, 0.5, "ffg 列なし/全欠損", ha="center", va="center",
                 transform=ax2.transAxes, color="#6b7280")
    ax2.set_xlabel("時間 [s]")
    save_fig(fig, out, "05_ekf_diagnostics.png")
    return True


def fig06_ff_status(data: dict, t: np.ndarray, on: np.ndarray,
                    out: Path, tsuffix: str) -> bool:
    if not usable(data, "ff_status"):
        print("  スキップ: 06_ff_status（ff_status が無い/全欠損）")
        return False
    st = np.nan_to_num(col(data, "ff_status")).astype(int)
    mode = st & FF_MODE_MASK
    fig, (ax, ax2) = plt.subplots(
        2, 1, figsize=(13, 7), sharex=True, gridspec_kw={"height_ratios": [1, 2]})

    _motor_spans(ax, t, on)
    ax.step(t, mode, where="post", lw=1.5, color="#0284c7")
    ax.set_yticks(sorted(FF_MODE_NAMES.keys()))
    ax.set_yticklabels([f"{k}: {v}" for k, v in sorted(FF_MODE_NAMES.items())],
                       fontsize=8)
    ax.set_ylim(-0.4, 3.4)
    ax.set_ylabel("ff_mode")
    ax.grid(alpha=0.3)
    ax.set_title("⑥ ff_status: ff_mode（下位2bit）と フラグビット（bit2-6）" + tsuffix,
                 fontsize=12)

    for i, (bit, name, color) in enumerate(FF_STATUS_FLAGS):
        active = ((st >> bit) & 1).astype(bool)
        ax2.fill_between(t, i + 0.1, i + 0.9, where=active, color=color, step="mid")
    ax2.set_yticks([i + 0.5 for i in range(len(FF_STATUS_FLAGS))])
    ax2.set_yticklabels([f"bit{b}: {n}" for b, n, _ in FF_STATUS_FLAGS], fontsize=8)
    ax2.set_ylim(0, len(FF_STATUS_FLAGS))
    ax2.grid(alpha=0.3, axis="x")
    ax2.set_xlabel("時間 [s]")
    save_fig(fig, out, "06_ff_status.png")
    return True


def fig07_flow(data: dict, t: np.ndarray, on: np.ndarray,
               out: Path, tsuffix: str) -> bool:
    """オプティカルフロー4段: 速度 / SQUAL・dt / ToF高度 / flow_status ビット。

    v2 ログ（flow_*/alt_tof_m 列あり）でのみ描く。v1 ログは列が無いため
    従来どおり自動スキップ（既存6枚の出力は不変）。
    """
    has_vel = usable(data, "flow_vx_mps") or usable(data, "flow_vy_mps")
    has_squal = usable(data, "flow_squal") or usable(data, "flow_dt_ms")
    has_alt = usable(data, "alt_tof_m")
    has_status = usable(data, "flow_status")
    if not (has_vel or has_squal or has_alt or has_status):
        print("  スキップ: 07_flow（flow_*/alt_tof_m 列が無い/全欠損 — v1 ログ）")
        return False
    fig, (ax1, ax2, ax3, ax4) = plt.subplots(
        4, 1, figsize=(13, 11), sharex=True,
        gridspec_kw={"height_ratios": [2, 1.4, 1.2, 1.2]})

    # vel_valid=0 区間（フロー速度が信頼できない区間）の薄灰帯を全段に引く
    st_col = col(data, "flow_status")
    invalid = None
    if np.isfinite(st_col).any():
        st = np.nan_to_num(st_col).astype(int)
        invalid = ((st >> FLOW_VEL_VALID_BIT) & 1) == 0

    def _invalid_spans(ax) -> None:
        if invalid is None or not invalid.any():
            return
        edges = np.diff(np.concatenate([[0], invalid.astype(int), [0]]))
        starts = np.where(edges == 1)[0]
        ends = np.where(edges == -1)[0] - 1
        label_done = False
        for s, e in zip(starts, ends):
            if s >= len(t):
                continue
            e = min(e, len(t) - 1)
            ax.axvspan(t[s], t[e], color="#9ca3af", alpha=0.15,
                       label=None if label_done else "vel_valid=0")
            label_done = True

    # ① フロー速度（機体系）
    _invalid_spans(ax1)
    _motor_spans(ax1, t, on)
    if usable(data, "flow_vx_mps"):
        ax1.plot(t, col(data, "flow_vx_mps"), "-", lw=1.2, color="#0284c7",
                 label="flow_vx（機体x）")
    if usable(data, "flow_vy_mps"):
        ax1.plot(t, col(data, "flow_vy_mps"), "-", lw=1.2, color="#d97706",
                 label="flow_vy（機体y）")
    ax1.axhline(0, ls=":", lw=0.8, color="#6b7280")
    ax1.set_ylabel("速度 [m/s]")
    ax1.grid(alpha=0.3)
    ax1.legend(loc="best", fontsize=8)
    ax1.set_title("⑦ オプティカルフロー: 速度 / SQUAL・dt / ToF高度 / status"
                  + tsuffix, fontsize=12)

    # ② SQUAL（左軸）+ 実測 dt（右軸）
    _invalid_spans(ax2)
    if usable(data, "flow_squal"):
        ax2.plot(t, col(data, "flow_squal"), "-", lw=1.0, color="#db2777",
                 label="SQUAL")
        ax2.set_ylabel("SQUAL", color="#db2777")
        ax2.tick_params(axis="y", labelcolor="#db2777")
    ax2.grid(alpha=0.3)
    if usable(data, "flow_dt_ms"):
        ax2t = ax2.twinx()
        ax2t.plot(t, col(data, "flow_dt_ms"), "-", lw=0.9, color="#475569",
                  alpha=0.8, label="flow_dt [ms]")
        ax2t.set_ylabel("dt [ms]", color="#475569")
        ax2t.tick_params(axis="y", labelcolor="#475569")
        h1, l1 = ax2.get_legend_handles_labels()
        h2, l2 = ax2t.get_legend_handles_labels()
        ax2.legend(h1 + h2, l1 + l2, loc="best", fontsize=8)
    else:
        ax2.legend(loc="best", fontsize=8)

    # ③ ToF 高度（フロー速度スケーリングの分母）
    _invalid_spans(ax3)
    if has_alt:
        ax3.plot(t, col(data, "alt_tof_m"), "-", lw=1.2, color="#16a34a",
                 label="alt_tof [m]")
        ax3.legend(loc="best", fontsize=8)
    else:
        ax3.text(0.5, 0.5, "alt_tof_m 列なし/全欠損", ha="center", va="center",
                 transform=ax3.transAxes, color="#6b7280")
    ax3.set_ylabel("ToF高度 [m]")
    ax3.grid(alpha=0.3)

    # ④ flow_status ビットのタイムライン（帯 = ビットが立っている区間）
    if has_status:
        st = np.nan_to_num(st_col).astype(int)
        for i, (bit, name, color) in enumerate(FLOW_STATUS_BITS):
            active = ((st >> bit) & 1).astype(bool)
            ax4.fill_between(t, i + 0.1, i + 0.9, where=active, color=color,
                             step="mid")
        ax4.set_yticks([i + 0.5 for i in range(len(FLOW_STATUS_BITS))])
        ax4.set_yticklabels([f"bit{b}: {n}" for b, n, _ in FLOW_STATUS_BITS],
                            fontsize=8)
        ax4.set_ylim(0, len(FLOW_STATUS_BITS))
        ax4.grid(alpha=0.3, axis="x")
    else:
        ax4.text(0.5, 0.5, "flow_status 列なし/全欠損", ha="center", va="center",
                 transform=ax4.transAxes, color="#6b7280")
    ax4.set_xlabel("時間 [s]")
    save_fig(fig, out, "07_flow.png")
    return True


# -------------------------------------------------------------- summary -------
STILL_GYRO_THRESH_RAD_S = 0.05   # |r| < 0.05 rad/s (≈2.9°/s) を静止とみなす
STILL_MIN_SEG_S = 3.0            # ドリフトフィットに使う静止区間の最短長
STILL_ENOUGH_S = 10.0            # これ未満なら「静止区間不足のため参考値」


def stationary_mask(data: dict, on: np.ndarray) -> np.ndarray:
    """静止区間: モーターOFF かつ |r_rad_s| < 閾値（r 欠損時は OFF のみ）。"""
    m = ~on
    r = col(data, "r_rad_s")
    if np.isfinite(r).any():
        m = m & np.isfinite(r) & (np.abs(r) < STILL_GYRO_THRESH_RAD_S)
    return m


def contiguous_runs(mask: np.ndarray) -> list[tuple[int, int]]:
    """True 連続区間の (開始, 終了) index（両端含む）リスト。"""
    m = mask.astype(int)
    edges = np.diff(np.concatenate([[0], m, [0]]))
    starts = np.where(edges == 1)[0]
    ends = np.where(edges == -1)[0] - 1
    return list(zip(starts.tolist(), ends.tolist()))


def stationary_drift_deg_per_min(t: np.ndarray, rel: np.ndarray,
                                 still: np.ndarray) -> tuple[float, float]:
    """静止区間のみのドリフト率 [deg/min] と、使用した静止時間合計 [s]。

    STILL_MIN_SEG_S 以上の静止区間ごとに線形フィットし、区間長で加重平均する
    （意図的回転を含むログで全区間フィットが無意味な値になるのを避ける）。
    """
    slopes: list[float] = []
    weights: list[float] = []
    for s, e in contiguous_runs(still):
        seg_t = t[s:e + 1]
        seg_y = rel[s:e + 1]
        fin = np.isfinite(seg_t) & np.isfinite(seg_y)
        if fin.sum() < 5:
            continue
        dur = float(np.ptp(seg_t[fin]))
        if dur < STILL_MIN_SEG_S:
            continue
        slopes.append(float(np.polyfit(seg_t[fin], seg_y[fin], 1)[0]) * 60.0)
        weights.append(dur)
    if not slopes:
        return math.nan, 0.0
    return float(np.average(slopes, weights=weights)), float(sum(weights))


def write_summary(data: dict, t: np.ndarray, on: np.ndarray, path: Path,
                  meta: dict | None, out: Path) -> None:
    lines: list[str] = []
    add = lines.append
    n = data["_n"]
    dur = float(np.nanmax(t) - np.nanmin(t)) if np.isfinite(t).any() else math.nan
    add("StampFly Experiment 計測ログ サマリ")
    add("=" * 60)
    add(f"入力      : {path}")
    add(f"サンプル数: {n}  /  記録時間: {dur:.1f} s  /  実効レート: {sample_rate(t):.1f} Hz")
    on_ratio = 100.0 * on.mean() if n else math.nan
    add(f"モーターON: {int(on.sum())} サンプル ({on_ratio:.1f}%)")

    ffl = ff_state_line(meta)
    if ffl:
        add(f"FF状態    : {ffl}")
    if meta:
        for k in ("started_at", "ended_at", "aborted"):
            if k in meta:
                add(f"meta.{k}: {meta[k]}")

    add("")
    still = stationary_mask(data, on)
    still_runs = [(s, e) for s, e in contiguous_runs(still)
                  if float(t[e] - t[s]) >= STILL_MIN_SEG_S]
    still_total = float(sum(t[e] - t[s] for s, e in still_runs))
    add(f"--- ヨー各系統のドリフト（静止区間のみ: モーターOFF かつ "
        f"|r|<{math.degrees(STILL_GYRO_THRESH_RAD_S):.1f}°/s・区間毎線形フィット加重平均） ---")
    add(f"静止区間  : {len(still_runs)} 区間 / 合計 {still_total:.1f} s"
        f"（{STILL_MIN_SEG_S:.0f} s 未満の区間は除外）")
    for key, lab, _ in YAW_STREAMS:
        if not usable(data, key):
            add(f"{lab:24s}: 列なし/全欠損")
            continue
        rel, _ = relative(col(data, key))
        fin = rel[np.isfinite(rel)]
        if not fin.size:
            add(f"{lab:24s}: 有効値なし")
            continue
        rate, used_s = stationary_drift_deg_per_min(t, rel, still)
        if math.isnan(rate):
            add(f"{lab:24s}: 静止区間不足のため算出不可 "
                f"(最終値 {fin[-1]:+.2f}° / 最大|ずれ| {np.max(np.abs(fin)):.2f}°)")
            continue
        note = "（静止区間不足のため参考値）" if used_s < STILL_ENOUGH_S else ""
        add(f"{lab:24s}: ドリフト率 {rate:+.3f} °/min{note}  "
            f"(静止 {used_s:.1f} s 使用 / 最終値 {fin[-1]:+.2f}° / "
            f"最大|ずれ| {np.max(np.abs(fin)):.2f}°)")

    # 開始静止 vs 終了静止の生磁気方位差（機体が物理的に元の向きへ戻ったかの指標）
    if usable(data, "bx_cal", "by_cal", "bz_cal"):
        ym_raw = mag_yaw_deg(data, subtract_ff=False, tilt_mask_deg=15.0)
        if len(still_runs) >= 2:
            s0, e0 = still_runs[0]
            s1, e1 = still_runs[-1]
            h0 = circ_mean_deg(ym_raw[s0:e0 + 1])
            h1 = circ_mean_deg(ym_raw[s1:e1 + 1])
            src = (f"開始静止 t={t[s0]:.1f}–{t[e0]:.1f}s vs "
                   f"終了静止 t={t[s1]:.1f}–{t[e1]:.1f}s")
        else:
            h0 = circ_mean_deg(ym_raw[:10])
            h1 = circ_mean_deg(ym_raw[-10:])
            src = "静止区間が2つ未満のため先頭/末尾10サンプルで代用（参考値）"
        if math.isnan(h0) or math.isnan(h1):
            add("生磁気方位差: tilt>15° 等で有効サンプルなし")
        else:
            dh = wrap180(np.array([h1 - h0]))[0]
            add(f"生磁気方位差（{src}）: {dh:+.1f}°"
                f"  ※FF補正なし・ファーム規約 atan2 準拠。0°に近いほど物理的に元の向き")

    add("")
    add("--- 電流 / 電圧 ---")
    cur = col(data, "current_a")
    if np.isfinite(cur).any():
        add(f"電流 [A]  : min {np.nanmin(cur):.3f} / 平均 {np.nanmean(cur):.3f} / "
            f"max {np.nanmax(cur):.3f}")
    else:
        add("電流 [A]  : 列なし/全欠損")
    vb = col(data, "vbat_v")
    if np.isfinite(vb).any():
        add(f"電圧 [V]  : min {np.nanmin(vb):.3f} / 平均 {np.nanmean(vb):.3f} / "
            f"max {np.nanmax(vb):.3f}")
    else:
        add("電圧 [V]  : 列なし/全欠損")

    add("")
    add("--- NIS 統計 ---")
    nis = col(data, "nis")
    fin = nis[np.isfinite(nis)]
    if fin.size:
        add(f"有効 {fin.size} / 平均 {np.mean(fin):.2f}（期待値≈{NIS_EXPECT}） / "
            f"中央値 {np.median(fin):.2f} / p95 {np.percentile(fin, 95):.2f} / "
            f"最大 {np.max(fin):.2f}")
        add(f"NIS > {NIS_SOFT}（R膨張域）: {int((fin > NIS_SOFT).sum())} "
            f"({100.0 * (fin > NIS_SOFT).mean():.1f}%)  /  "
            f"NIS > {NIS_REJECT}（棄却域）: {int((fin > NIS_REJECT).sum())} "
            f"({100.0 * (fin > NIS_REJECT).mean():.1f}%)")
    else:
        add("nis 列なし/全欠損")

    add("")
    add("--- ffg ゲート発火数 ---")
    ffgc = col(data, "ffg")
    if np.isfinite(ffgc).any():
        ffg = np.nan_to_num(ffgc).astype(int)
        for b, (name, _) in enumerate(GATE_BITS):
            cnt = int(((ffg >> b) & 1).sum())
            add(f"bit{b} {name:12s}: {cnt:6d} サンプル ({100.0 * cnt / n:.1f}%)")
    else:
        add("ffg 列なし/全欠損")

    add("")
    add("--- ff_status ---")
    stc = col(data, "ff_status")
    if np.isfinite(stc).any():
        st = np.nan_to_num(stc).astype(int)
        modes = st & FF_MODE_MASK
        seen = ", ".join(f"{FF_MODE_NAMES.get(m, m)}×{int((modes == m).sum())}"
                         for m in sorted(set(modes.tolist())))
        add(f"ff_mode 内訳: {seen}")
        for bit, name, _ in FF_STATUS_FLAGS:
            cnt = int(((st >> bit) & 1).sum())
            add(f"bit{bit} {name:16s}: {cnt:6d} サンプル ({100.0 * cnt / n:.1f}%)")
    else:
        add("ff_status 列なし/全欠損")

    text = "\n".join(lines) + "\n"
    (out / "summary.txt").write_text(text, encoding="utf-8")
    print("  出力: summary.txt")
    print("\n" + text)


# ----------------------------------------------------------- animation --------
# 7枠レイアウト（3行×4列）: 左2×2=①スマホ動画、右上②ヨー ③duty+電流、
# 右中④磁場x ⑤磁場y、下段⑥b_m ⑦IMU温度（幅広）。
# 時系列6枠はスクロール窓表示: x軸は常に現在時刻 t 中心の [t-窓幅/2, t+窓幅/2]
# （開始直後は左半分が負時間の空白）。線は時刻<=t の履歴のみ描画し、右半分
# （未来）は常に空白。y軸はログ全期間から初期化時に一度だけ固定。
# xlim が毎フレーム変わるため blit は使わず、フレーム毎に Line2D の set_data
# （窓スライス・右端は t）+ set_xlim + canvas 再描画を行う。全トレースに現在
# 時刻の補間値位置を示すドット（線と同色・黒フチ・常に x軸中央）を重畳する
# （overview には無し）。
# アニメ生成時は全期間俯瞰の静止画ボード explog_<stem>_overview.png も同時出力。
ANIM_FPS_NO_VIDEO = 20.0        # 動画なし時のフレームレート
ANIM_FIGSIZE = (19.2, 10.8)     # 1920x1080 @ dpi100
ANIM_DPI = 100
ANIM_WINDOW_S = 5.0             # スクロール窓幅の既定 [s]（--window/対話で変更可）
ANIM_WINDOW_MIN_S = 5.0
ANIM_WINDOW_MAX_S = 60.0
VIDEO_PANEL_MAX_PX = 720        # 動画フレームの縮小上限（正方形一辺）
VIDEO_EXTS = (".mp4", ".mov", ".m4v", ".avi", ".mkv")
TIME_TEXT_COLOR = "#111111"


def videos_dir() -> Path:
    """同期用スマホ動画の置き場: pc_server/data/exp_logs/videos/。"""
    return explog_dir() / "videos"


def _import_cv2():
    """cv2 の遅延 import（動画同期時のみ必要）。無ければ日本語で案内して終了。"""
    try:
        import cv2
        return cv2
    except ImportError:
        sys.exit(
            "スマホ動画の同期合成には opencv-python が必要です。\n"
            "  data_analysis/.venv/bin/pip install opencv-python\n"
            "を実行してください（requirements.txt のコメント参照）。")


def _require_ffmpeg() -> None:
    if shutil.which("ffmpeg") is None:
        sys.exit("ffmpeg が見つかりません。MP4 出力には ffmpeg が必要です"
                 "（macOS: brew install ffmpeg）。")


class _VideoReader:
    """スマホ動画を順次デコードし、中央クロップ正方形 RGB フレームを返す。

    動画は「LED マゼンタ点灯 = 計測開始(t_s=0)」でカット済みの前提なので、
    動画フレーム index = round(t × video_fps) の単純対応で同期する。
    順次 grab() で進め、必要フレームのみ retrieve()（全読み込みしない省メモリ構成）。
    """

    def __init__(self, path: Path) -> None:
        cv2 = _import_cv2()
        self._cv2 = cv2
        self._cap = cv2.VideoCapture(str(path))
        if not self._cap.isOpened():
            sys.exit(f"動画ファイルを開けません: {path}")
        self.fps = float(self._cap.get(cv2.CAP_PROP_FPS)) or 30.0
        self.n_frames = int(self._cap.get(cv2.CAP_PROP_FRAME_COUNT))
        self.duration_s = self.n_frames / self.fps if self.fps > 0 else 0.0
        self._pos = 0            # 次に grab されるフレーム index
        self._last: np.ndarray | None = None

    def _process(self, frame_bgr: np.ndarray) -> np.ndarray:
        """中央クロップで正方形化 → 縮小 → RGB 化。"""
        cv2 = self._cv2
        h, w = frame_bgr.shape[:2]
        if w != h:  # 正方形前提。非正方形は中央クロップ
            s = min(h, w)
            y0, x0 = (h - s) // 2, (w - s) // 2
            frame_bgr = frame_bgr[y0:y0 + s, x0:x0 + s]
        if frame_bgr.shape[0] > VIDEO_PANEL_MAX_PX:
            frame_bgr = cv2.resize(frame_bgr,
                                   (VIDEO_PANEL_MAX_PX, VIDEO_PANEL_MAX_PX))
        return cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2RGB)

    def frame_at(self, idx: int) -> np.ndarray | None:
        """フレーム idx を返す（末尾超過は最終フレームを保持。巻き戻しはシーク）。"""
        idx = max(0, min(idx, self.n_frames - 1))
        if idx < self._pos - 1:  # 逆行（--start 変更等）はシーク
            self._cap.set(self._cv2.CAP_PROP_POS_FRAMES, idx)
            self._pos = idx
        while self._pos <= idx:
            if not self._cap.grab():
                return self._last  # デコード終端: 最後のフレームを使い続ける
            self._pos += 1
            if self._pos == idx + 1:
                ok, frame = self._cap.retrieve()
                if ok:
                    self._last = self._process(frame)
        return self._last

    def release(self) -> None:
        self._cap.release()


def _interp_series(t: np.ndarray, v: np.ndarray,
                   frame_times: np.ndarray) -> np.ndarray:
    """有限値のみで線形補間（NaN を拡散させない。範囲外は NaN）。"""
    fin = np.isfinite(t) & np.isfinite(v)
    if fin.sum() < 2:
        return np.full(len(frame_times), np.nan)
    return np.interp(frame_times, t[fin], v[fin], left=np.nan, right=np.nan)


def _dot_interp(tx: np.ndarray, ty: np.ndarray,
                frame_times: np.ndarray) -> np.ndarray:
    """現在位置ドット用の NaN 保存線形補間。

    _interp_series と違い NaN を落とさない: 補間区間の端点いずれかが NaN なら
    NaN（＝ドット非表示）。ヨーのラップ跨ぎに挿入した NaN 切断点を越えて
    +179°→−179° を「0° 付近」と誤補間しないため。範囲外も NaN。
    """
    out = np.full(len(frame_times), np.nan)
    if len(tx) < 2:
        return out
    idx = np.clip(np.searchsorted(tx, frame_times, side="right") - 1,
                  0, len(tx) - 2)
    t0, t1 = tx[idx], tx[idx + 1]
    y0, y1 = ty[idx], ty[idx + 1]
    with np.errstate(invalid="ignore", divide="ignore"):
        frac = np.where(t1 > t0, (frame_times - t0) / (t1 - t0), 0.0)
        val = y0 + frac * (y1 - y0)
    ok = (frame_times >= tx[0]) & (frame_times <= tx[-1])
    out[ok] = val[ok]
    return out


def _fmt_val(v: float, fmt: str = "{:+.1f}") -> str:
    return fmt.format(v) if np.isfinite(v) else "--"


def _window_ylim(ax, t: np.ndarray, arrays: list[np.ndarray],
                 t0: float, t1: float, pad_ratio: float = 0.12) -> None:
    """アニメ範囲 [t0, t1] 内の有限値から固定 y 軸を決める。"""
    win = (t >= t0) & (t <= t1)
    vals: list[np.ndarray] = []
    for a in arrays:
        m = win & np.isfinite(a)
        if m.any():
            vals.append(a[m])
    if not vals:
        return
    allv = np.concatenate(vals)
    lo, hi = float(allv.min()), float(allv.max())
    pad = max((hi - lo) * pad_ratio, 1e-3)
    ax.set_ylim(lo - pad, hi + pad)


def _value_textbox(ax):
    """現在値テキストボックス（左上）を作る。"""
    return ax.text(0.02, 0.965, "", transform=ax.transAxes, va="top", ha="left",
                   fontsize=8, zorder=11,
                   bbox=dict(facecolor="#ffffff", edgecolor="#444444",
                             alpha=0.85, boxstyle="round,pad=0.25"))


def _anim_panel(ax, title: str, ylabel: str, x0: float, x1: float,
                with_text: bool) -> dict:
    """1枠の共通装飾（+ 任意で現在値テキスト）を作り panel dict を返す。

    panel dict:
      ax    : xlim 更新に使う主軸
      lines : (Line2D, t全体, y全体) のリスト。スクロール時に窓スライスを set_data
      text  : 現在値テキスト（with_text=False では None）
      fmt   : フレーム index → 現在値文字列
    """
    ax.set_xlim(x0, x1)
    ax.set_title(title, fontsize=10)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.set_xlabel("時間 [s]", fontsize=8)
    ax.tick_params(labelsize=8)
    ax.grid(alpha=0.3)
    text = _value_textbox(ax) if with_text else None
    return {"ax": ax, "lines": [], "text": text, "fmt": lambda i: ""}


def _panel_line(panel: dict, ax, tx: np.ndarray, ty: np.ndarray, **kw) -> None:
    """panel に Line2D を1本追加し、スクロール set_data 用に全期間データを保持する。

    t が NaN の点は searchsorted（窓スライス）で困るため落としておく。
    """
    tx = np.asarray(tx, dtype=float)
    ty = np.asarray(ty, dtype=float)
    fin_t = np.isfinite(tx)
    tx, ty = tx[fin_t], ty[fin_t]
    (line,) = ax.plot(tx, ty, **kw)
    panel["lines"].append((line, tx, ty))


def _legend_small(ax) -> None:
    ax.legend(loc="upper right", fontsize=7, framealpha=0.7)


def _explog_stem(path: Path) -> str:
    return path.stem[len("explog_"):] if path.stem.startswith("explog_") else path.stem


def _build_board(path: Path, data: dict, t: np.ndarray, tsuffix: str,
                 iv: dict[str, np.ndarray] | None,
                 x0: float, x1: float, title_head: str):
    """アニメ / 全期間俯瞰ボード共通の 3×4 ボード Figure を構築する。

    時系列6枠の初期 xlim は [x0, x1]。y軸レンジは常にログ全期間の有限値から
    一度だけ固定する（スクロール中にフレーム毎へ暴れない）。
    iv=None（俯瞰ボード）では現在値テキスト・fmt を作らない。
    ①動画枠の中身（動画フレーム / サムネ / 「動画なし」）は呼び出し側が設定する。
    戻り値: (fig, ax_video, panels)
    """
    with_text = iv is not None
    yl0 = float(np.nanmin(t))
    yl1 = float(np.nanmax(t))

    fig = plt.figure(figsize=ANIM_FIGSIZE, dpi=ANIM_DPI)
    gs = fig.add_gridspec(3, 4, left=0.045, right=0.985, top=0.90, bottom=0.06,
                          wspace=0.30, hspace=0.55,
                          height_ratios=[1.0, 1.0, 0.9])
    fig.suptitle(f"{title_head}: {path.name}"
                 + (tsuffix.replace("\n", "   ") if tsuffix else ""),
                 fontsize=13)

    # ① スマホ動画（左2×2）
    ax_video = fig.add_subplot(gs[0:2, 0:2])
    ax_video.axis("off")

    panels: list[dict] = []

    # ② Yaw（±180ラップ + ラップ跨ぎ NaN 切断）
    p = _anim_panel(fig.add_subplot(gs[0, 2]), "② ヨー（±180°ラップ）",
                    "Yaw [deg]", x0, x1, with_text)
    for key, lab, color in (("yaw_madgwick_deg", "Madgwick", "#db2777"),
                            ("yaw_est_deg", "EKF推定", "#0284c7")):
        if usable(data, key):
            tw, yw = wrapped_plot_series(t, col(data, key))
            _panel_line(p, p["ax"], tw, yw, ls="-", lw=1.1, color=color, label=lab)
    p["ax"].set_ylim(-190, 190)
    p["ax"].set_yticks(np.arange(-180, 181, 90))
    _legend_small(p["ax"])
    if iv is not None:
        p["fmt"] = lambda i: (f"Mdg {_fmt_val(iv['yaw_madgwick_deg'][i])}° / "
                              f"EKF {_fmt_val(iv['yaw_est_deg'][i])}°")
    panels.append(p)

    # ③ duty + 電流（twinx。x軸は共有なので set_xlim は主軸のみでよい）
    ax3 = fig.add_subplot(gs[0, 3])
    ax3.set_title("③ duty + 電流", fontsize=10)
    ax3.set_xlabel("時間 [s]", fontsize=8)
    ax3.set_ylabel("duty", fontsize=9, color="#475569")
    ax3.tick_params(labelsize=8)
    ax3.tick_params(axis="y", labelcolor="#475569")
    ax3.grid(alpha=0.3)
    ax3.set_xlim(x0, x1)
    p3 = {"ax": ax3, "lines": [], "text": None, "fmt": lambda i: ""}
    _panel_line(p3, ax3, t, col(data, "duty_cmd"), ls="-", lw=1.1,
                color="#475569", label="duty_cmd")
    _window_ylim(ax3, t, [col(data, "duty_cmd")], yl0, yl1)
    ax3t = ax3.twinx()
    _panel_line(p3, ax3t, t, col(data, "current_a"), ls="-", lw=1.1,
                color="#16a34a", label="電流 [A]")
    ax3t.set_ylabel("電流 [A]", fontsize=9, color="#16a34a")
    ax3t.tick_params(labelsize=8, axis="y", labelcolor="#16a34a")
    _window_ylim(ax3t, t, [col(data, "current_a")], yl0, yl1)
    h1, l1 = ax3.get_legend_handles_labels()
    h2, l2 = ax3t.get_legend_handles_labels()
    ax3t.legend(h1 + h2, l1 + l2, loc="upper right", fontsize=7, framealpha=0.7)
    if iv is not None:
        p3["text"] = _value_textbox(ax3t)  # 上に重なる twin 側に置く
        p3["fmt"] = lambda i: (f"duty {_fmt_val(iv['duty_cmd'][i], '{:.3f}')} / "
                               f"{_fmt_val(iv['current_a'][i], '{:.2f}')} A")
    panels.append(p3)

    # ④⑤ 磁場 x/y: b*_cal と FF補正後（b*_cal − db_hat_*）
    for slot, axis_name, bkey, dkey, color in (
            (gs[1, 2], "x", "bx_cal", "db_hat_x_ut", "#2563eb"),
            (gs[1, 3], "y", "by_cal", "db_hat_y_ut", "#16a34a")):
        num = "④" if axis_name == "x" else "⑤"
        p = _anim_panel(fig.add_subplot(slot),
                        f"{num} 磁場{axis_name}（校正済み と FF補正後）",
                        f"B_{axis_name} [µT]", x0, x1, with_text)
        bcal = col(data, bkey)
        _panel_line(p, p["ax"], t, bcal, ls="-", lw=1.1, color=color, label=bkey)
        has_ff = usable(data, dkey)
        if has_ff:
            bcorr = bcal - np.nan_to_num(col(data, dkey))
            _panel_line(p, p["ax"], t, bcorr, ls="-", lw=1.1, color="#111111",
                        alpha=0.85, label=f"{bkey} − db_hat_{axis_name}")
            _window_ylim(p["ax"], t, [bcal, bcorr], yl0, yl1)
        else:
            _window_ylim(p["ax"], t, [bcal], yl0, yl1)
        _legend_small(p["ax"])
        if iv is not None:
            def _fmt_mag(i, _b=bkey, _d=dkey, _has=has_ff):
                raw = iv[_b][i]
                if not _has:
                    return f"cal {_fmt_val(raw)} µT"
                corr = raw - (iv[_d][i] if np.isfinite(iv[_d][i]) else 0.0)
                return f"cal {_fmt_val(raw)} / 補正後 {_fmt_val(corr)} µT"
            p["fmt"] = _fmt_mag
        panels.append(p)

    # ⑥ b_m（EKF 磁気基準ベクトル水平2成分, 下段幅広）
    p = _anim_panel(fig.add_subplot(gs[2, 0:2]), "⑥ b_m（EKF磁気基準・水平2成分）",
                    "b_m [µT]", x0, x1, with_text)
    for key, lab, color in (("bm_x_ut", "bm_x", "#7c3aed"),
                            ("bm_y_ut", "bm_y", "#d97706")):
        if usable(data, key):
            _panel_line(p, p["ax"], t, col(data, key), ls="-", lw=1.1,
                        color=color, label=lab)
    _window_ylim(p["ax"], t, [col(data, "bm_x_ut"), col(data, "bm_y_ut")],
                 yl0, yl1)
    _legend_small(p["ax"])
    if iv is not None:
        p["fmt"] = lambda i: (f"x {_fmt_val(iv['bm_x_ut'][i])} / "
                              f"y {_fmt_val(iv['bm_y_ut'][i])} µT")
    panels.append(p)

    # ⑦ IMU温度（下段幅広）
    p = _anim_panel(fig.add_subplot(gs[2, 2:4]), "⑦ IMU温度", "温度 [°C]",
                    x0, x1, with_text)
    if usable(data, "imu_temp_c"):
        _panel_line(p, p["ax"], t, col(data, "imu_temp_c"), ls="-", lw=1.1,
                    color="#dc2626", label="imu_temp_c")
        _window_ylim(p["ax"], t, [col(data, "imu_temp_c")], yl0, yl1)
        _legend_small(p["ax"])
    if iv is not None:
        p["fmt"] = lambda i: f"{_fmt_val(iv['imu_temp_c'][i], '{:.1f}')} °C"
    panels.append(p)

    return fig, ax_video, panels


def render_overview(path: Path, out: Path, data: dict, t: np.ndarray,
                    tsuffix: str, video_path: Path | None) -> Path:
    """全期間俯瞰の静止画ボード explog_<stem>_overview.png を出力する。

    レイアウトはアニメと同一の 3×4 グリッド（1920x1080 相当）。時系列6枠は
    --start/--end に関わらず常にログ全期間（カーソル・現在値ボックスなし）。
    動画枠はスマホ動画の先頭フレーム（正方形クロップ）のサムネ固定表示。
    動画なし経路では従来どおり「動画なし」表示。
    """
    x0 = float(np.nanmin(t))
    x1 = float(np.nanmax(t))
    fig, ax_video, _panels = _build_board(path, data, t, tsuffix, None, x0, x1,
                                          "Experiment 全期間俯瞰ボード")
    if video_path is not None:
        reader = _VideoReader(video_path)
        first = reader.frame_at(0)
        reader.release()
        if first is None:
            sys.exit("動画の最初のフレームを読み込めませんでした。")
        ax_video.imshow(first)
        ax_video.set_title("① スマホ動画（先頭フレームのサムネイル）", fontsize=10)
    else:
        ax_video.set_title("① スマホ動画", fontsize=10)
        ax_video.text(0.5, 0.5, "動画なし\n（--video または メニュー[2] で同期合成）",
                      transform=ax_video.transAxes, ha="center", va="center",
                      fontsize=13, color="#6b7280")
    out_path = out / f"explog_{_explog_stem(path)}_overview.png"
    fig.savefig(out_path)  # figure dpi のまま保存 = 1920x1080
    plt.close(fig)
    print(f"全期間俯瞰ボード出力: {out_path}")
    return out_path


def render_animation(path: Path, out: Path, data: dict, t: np.ndarray,
                     tsuffix: str, video_path: Path | None,
                     fps_arg: float | None, start_s: float | None,
                     end_s: float | None, window_s: float | None) -> None:
    """スクロール窓方式の 7枠アニメ MP4 を生成する。

    同期規約: 「LED がマゼンタに変わった瞬間 = 計測開始(t_s=0) = 動画のカット位置」。
    動画あり時は出力 fps=動画 fps（既定30）、長さ=min(動画, ログ, --end)。
    データ(≈23.5Hz)は各フレーム時刻へ線形補間して現在値表示に使う。
    時系列6枠は x軸を常に現在時刻 t 中心の [t-窓幅/2, t+窓幅/2] とし、
    線は時刻<=t の履歴のみ描画（右半分=未来は常に空白。開始直後は左半分が
    負時間の空白）。y軸はログ全期間から固定。
    全トレースに現在時刻の補間値位置ドット（線と同色・黒フチ・常に x軸中央・
    NaN 時非表示）を重畳。
    xlim が毎フレーム変わるため blit は使わず、フレーム毎に Line2D の
    set_data（窓スライス・右端は t）+ ドット set_data + set_xlim +
    canvas 再描画を行う。
    """
    _require_ffmpeg()

    t_end_log = float(np.nanmax(t))
    t0 = max(0.0, float(start_s)) if start_s is not None else 0.0
    t1 = min(t_end_log, float(end_s)) if end_s is not None else t_end_log
    window = float(window_s) if window_s is not None else ANIM_WINDOW_S

    reader: _VideoReader | None = None
    if video_path is not None:
        reader = _VideoReader(video_path)
        print(f"動画: {video_path.name}  {reader.fps:.2f}fps / "
              f"{reader.n_frames}フレーム / {reader.duration_s:.1f}s")
        t1 = min(t1, reader.duration_s)  # 長さ = min(動画, ログ, --end)
        fps = float(fps_arg) if fps_arg else reader.fps
    else:
        fps = float(fps_arg) if fps_arg else ANIM_FPS_NO_VIDEO

    if t1 - t0 < 0.5:
        sys.exit(f"アニメ範囲が不正です: start={t0:.2f}s, end={t1:.2f}s")
    n_frames = int((t1 - t0) * fps)
    frame_times = t0 + np.arange(n_frames) / fps

    # --- フレーム時刻への線形補間（現在値テキスト用） ---
    iv: dict[str, np.ndarray] = {}
    for name in ("duty_cmd", "current_a", "bx_cal", "by_cal",
                 "db_hat_x_ut", "db_hat_y_ut", "bm_x_ut", "bm_y_ut",
                 "imu_temp_c"):
        iv[name] = _interp_series(t, col(data, name), frame_times)
    for name in ("yaw_madgwick_deg", "yaw_est_deg"):  # 角度は unwrap 補間→ラップ
        iv[name] = wrap180(_interp_series(t, unwrap_deg(col(data, name)),
                                          frame_times))

    # --- Figure / 7枠レイアウト（初期 xlim = 現在時刻 t0 中心の窓） ---
    half = window / 2.0
    fig, ax_video, panels = _build_board(path, data, t, tsuffix, iv,
                                         t0 - half, t0 + half,
                                         "Experiment 計測アニメーション")
    time_text = fig.text(0.985, 0.975, "", ha="right", va="top", fontsize=12,
                         color=TIME_TEXT_COLOR)

    # --- 現在位置ドット（全トレース。線と同色・黒フチ・自軸に描画） ---
    # panel["lines"] を (line, tx, ty, dot_vals, dot) の5要素に拡張する。
    # dot_vals は各フレーム時刻の NaN 保存補間値（NaN のフレームは非表示）。
    # twinx（電流）は line.axes が twin 側なのでドットも自分の軸に載る。
    for pnl in panels:
        dotted = []
        for line, tx, ty in pnl["lines"]:
            dot_vals = _dot_interp(tx, ty, frame_times)
            (dot,) = line.axes.plot([], [], marker="o", ls="", ms=9,
                                    color=line.get_color(), mec="#111111",
                                    mew=1.2, zorder=10)
            dotted.append((line, tx, ty, dot_vals, dot))
        pnl["lines"] = dotted

    # ① スマホ動画（左2×2）
    ax_video.set_title("① スマホ動画（LEDマゼンタ点灯=計測開始 t=0 でカット済み前提）",
                       fontsize=10)
    im_video = None
    if reader is not None:
        first = reader.frame_at(int(round(t0 * reader.fps)))
        if first is None:
            sys.exit("動画の最初のフレームを読み込めませんでした。")
        im_video = ax_video.imshow(first)
    else:
        ax_video.text(0.5, 0.5, "動画なし\n（--video または メニュー[2] で同期合成）",
                      transform=ax_video.transAxes, ha="center", va="center",
                      fontsize=13, color="#6b7280")

    canvas = fig.canvas
    canvas.draw()
    w_px, h_px = canvas.get_width_height()

    out_name = (f"explog_{_explog_stem(path)}_animation"
                + ("_with_video" if reader is not None else "") + ".mp4")
    out_path = out / out_name
    print(f"\nアニメーション生成: {n_frames}フレーム @ {fps:.2f}fps "
          f"({t0:.1f}–{t1:.1f}s, 窓幅 {window:.1f}s, {w_px}x{h_px}) → {out_path}")

    cmd = ["ffmpeg", "-y", "-loglevel", "error",
           "-f", "rawvideo", "-pix_fmt", "rgba", "-s", f"{w_px}x{h_px}",
           "-r", f"{fps:.6f}", "-i", "-",
           "-an", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "20",
           str(out_path)]
    proc = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    assert proc.stdin is not None
    last_pct = -10
    try:
        for i, tf in enumerate(frame_times):
            # スクロール窓: 常に現在時刻 tf 中心の [tf-窓幅/2, tf+窓幅/2]
            # （開始直後は左半分が負時間の空白）
            wl = tf - half
            wr = tf + half
            for pnl in panels:
                pnl["ax"].set_xlim(wl, wr)  # twinx は x 軸共有のため主軸のみでよい
                for line, tx, ty, dot_vals, dot in pnl["lines"]:
                    i0 = max(int(np.searchsorted(tx, wl)) - 1, 0)
                    # 右端は現在時刻 tf: 未来（tf より先）の線は描かない
                    i1 = int(np.searchsorted(tx, tf, side="right"))
                    line.set_data(tx[i0:i1], ty[i0:i1])
                    v = dot_vals[i]
                    if np.isfinite(v):
                        dot.set_data([tf], [v])
                    else:
                        dot.set_data([], [])  # NaN のときは非表示
                if pnl["text"] is not None:
                    pnl["text"].set_text(pnl["fmt"](i))
            if im_video is not None and reader is not None:
                frame = reader.frame_at(int(round(tf * reader.fps)))
                if frame is not None:
                    im_video.set_data(frame)
            time_text.set_text(f"t = {tf:6.2f} s")
            canvas.draw()
            proc.stdin.write(bytes(canvas.buffer_rgba()))
            pct = int(100 * (i + 1) / n_frames)
            if pct >= last_pct + 10:
                last_pct = pct
                print(f"  進捗: {pct}% ({i + 1}/{n_frames})")
        proc.stdin.close()
        ret = proc.wait()
        if ret != 0:
            sys.exit(f"ffmpeg がエラー終了しました (exit={ret})。")
    except BrokenPipeError:
        proc.wait()
        sys.exit("ffmpeg への書き込みに失敗しました（パイプ切断）。")
    finally:
        if reader is not None:
            reader.release()
        plt.close(fig)
    print(f"アニメーション生成完了: {out_path}")


# ---------------------------------------------------------------- main --------
def explog_dir() -> Path:
    """pc_server が Experiment ログを書き出す exp_logs フォルダ。"""
    return Path(__file__).resolve().parent.parent / "pc_server" / "data" / "exp_logs"


def list_explogs() -> list[Path]:
    """exp_logs 内の explog CSV を新しい順（mtime 降順）に返す。"""
    d = explog_dir()
    if not d.is_dir():
        return []
    return sorted(d.glob(EXPLOG_GLOB), key=lambda p: p.stat().st_mtime, reverse=True)


def _fmt_size(n: int) -> str:
    return f"{n / 1024:.0f} KB" if n < 1024 * 1024 else f"{n / 1024 / 1024:.1f} MB"


def choose_explog(cands: list[Path]) -> Path | None:
    """explog CSV を一覧表示し、番号で1つ選ばせる（会話的選択）。

    Enter は最新（[1]）を選択、q で中止して None を返す。
    パイプ等で対話入力が無い（EOF）場合は最新を自動選択する。
    """
    print(f"\n{explog_dir()} の explog CSV から、グラフ化するものを選んでください:\n")
    for i, p in enumerate(cands, 1):
        st = p.stat()
        mtime = time.strftime("%Y-%m-%d %H:%M", time.localtime(st.st_mtime))
        mark = "  ← 最新" if i == 1 else ""
        print(f"  [{i}] {p.name}   {mtime}   {_fmt_size(st.st_size)}{mark}")
    print()
    while True:
        try:
            raw = input(f"番号を入力 [1-{len(cands)}]（Enter=最新 / q=中止）: ").strip()
        except EOFError:
            print("（対話入力なし → 最新を自動選択）")
            return cands[0]
        if raw == "":
            return cands[0]
        if raw.lower() in ("q", "quit", "exit"):
            return None
        if raw.isdigit() and 1 <= int(raw) <= len(cands):
            return cands[int(raw) - 1]
        print(f"  '{raw}' は無効です。1〜{len(cands)} の番号を入力してください。")


def choose_mode() -> str | None:
    """出力モードを対話選択する。戻り値: 'static' / 'anim_video' / 'anim_only' / None(中止)。

    Enter は [1]（従来の静止画）。EOF（パイプ等）も [1] を自動選択し、
    従来の「引数なし実行=静止画」挙動を保つ。
    """
    print("\n出力の種類を選んでください:\n")
    print("  [1] 静止画グラフ一式（従来: PNG 6〜7枚 + summary.txt）")
    print("  [2] アニメーション（スマホ動画同期, MP4）")
    print("  [3] アニメーション（動画なし, MP4）")
    print()
    modes = {"1": "static", "2": "anim_video", "3": "anim_only"}
    while True:
        try:
            raw = input("番号を入力 [1-3]（Enter=1 / q=中止）: ").strip()
        except EOFError:
            print("（対話入力なし → [1] 静止画を自動選択）")
            return "static"
        if raw == "":
            return "static"
        if raw.lower() in ("q", "quit", "exit"):
            return None
        if raw in modes:
            return modes[raw]
        print(f"  '{raw}' は無効です。1〜3 の番号を入力してください。")


def list_videos() -> list[Path]:
    """videos/ 内の動画を新しい順（mtime 降順）に返す。"""
    d = videos_dir()
    if not d.is_dir():
        return []
    vids = [p for p in d.iterdir()
            if p.is_file() and p.suffix.lower() in VIDEO_EXTS]
    return sorted(vids, key=lambda p: p.stat().st_mtime, reverse=True)


def choose_video() -> Path | None:
    """同期用スマホ動画を番号選択（パス直接入力も可）。None は中止。"""
    cands = list_videos()
    if cands:
        print(f"\n{videos_dir()} の動画から、同期するものを選んでください"
              "（動画ファイルのパス直接入力も可）:\n")
        for i, p in enumerate(cands, 1):
            st = p.stat()
            mtime = time.strftime("%Y-%m-%d %H:%M", time.localtime(st.st_mtime))
            mark = "  ← 最新" if i == 1 else ""
            print(f"  [{i}] {p.name}   {mtime}   {_fmt_size(st.st_size)}{mark}")
        print()
        prompt = f"番号またはパスを入力 [1-{len(cands)}]（Enter=最新 / q=中止）: "
    else:
        print(f"\n{videos_dir()} に動画がありません。"
              "動画ファイルのパスを直接入力してください。")
        prompt = "動画パスを入力（q=中止）: "
    while True:
        try:
            raw = input(prompt).strip()
        except EOFError:
            print("（対話入力なし → 中止）")
            return None
        if raw == "" and cands:
            return cands[0]
        if raw.lower() in ("q", "quit", "exit"):
            return None
        if raw.isdigit() and cands and 1 <= int(raw) <= len(cands):
            return cands[int(raw) - 1]
        p = Path(raw).expanduser()
        if p.is_file():
            return p
        print(f"  '{raw}' は番号でも既存ファイルでもありません。")


def choose_window() -> float | None:
    """スクロール窓幅 [s] を対話入力する。None は中止。

    Enter（および EOF）は既定 ANIM_WINDOW_S。範囲外・非数値は再入力。
    """
    while True:
        try:
            raw = input(f"ウィンドウ幅(秒)を入力 (Enter={ANIM_WINDOW_S:.0f}, "
                        f"{ANIM_WINDOW_MIN_S:.0f}〜{ANIM_WINDOW_MAX_S:.0f}, "
                        f"q で中止): ").strip()
        except EOFError:
            print(f"（対話入力なし → 既定 {ANIM_WINDOW_S:.0f} 秒）")
            return ANIM_WINDOW_S
        if raw == "":
            return ANIM_WINDOW_S
        if raw.lower() in ("q", "quit", "exit"):
            return None
        try:
            v = float(raw)
        except ValueError:
            print(f"  '{raw}' は数値ではありません。")
            continue
        if ANIM_WINDOW_MIN_S <= v <= ANIM_WINDOW_MAX_S:
            return v
        print(f"  {ANIM_WINDOW_MIN_S:.0f}〜{ANIM_WINDOW_MAX_S:.0f} 秒の範囲で"
              f"入力してください: {raw}")


def default_out_dir(path: Path) -> Path:
    """既定の出力先: data_analysis/graphs/explog_<stamp>/。"""
    stem = path.stem
    if stem.startswith("explog_"):
        stem = stem[len("explog_"):]
    return Path(__file__).resolve().parent / "graphs" / f"explog_{stem}"


def main() -> None:
    ap = argparse.ArgumentParser(
        description="StampFly Experiment 計測ログ（explog CSV）のグラフ化"
                    "（PNG 6〜7枚 + summary.txt / アニメ MP4）")
    ap.add_argument("explog", nargs="?",
                    help="explog CSV パス（省略時は exp_logs の一覧から対話選択）")
    ap.add_argument("-o", "--out",
                    help="出力ディレクトリ（省略時は data_analysis/graphs/explog_<stamp>/）")
    ap.add_argument("--animation", action="store_true",
                    help="アニメ MP4 を生成（--video なしは動画なしアニメ 20fps）")
    ap.add_argument("--video", metavar="PATH",
                    help="同期するスマホ動画（指定すると --animation を含意。"
                         "LEDマゼンタ点灯=計測開始でカット済み前提）")
    ap.add_argument("--fps", type=float,
                    help="アニメ出力 fps（既定: 動画あり=動画fps / なし=20）")
    ap.add_argument("--start", type=float, metavar="S",
                    help="アニメ開始時刻 [s]（ログ時間軸基準, 既定0）")
    ap.add_argument("--end", type=float, metavar="S",
                    help="アニメ終了時刻 [s]（既定: ログ末尾。動画があれば min も取る）")
    ap.add_argument("--window", type=float, metavar="W",
                    help=f"アニメ時系列のスクロール窓幅 [s]"
                         f"（既定 {ANIM_WINDOW_S:.0f}、"
                         f"{ANIM_WINDOW_MIN_S:.0f}〜{ANIM_WINDOW_MAX_S:.0f}）")
    args = ap.parse_args()

    if args.window is not None and not (ANIM_WINDOW_MIN_S <= args.window
                                        <= ANIM_WINDOW_MAX_S):
        sys.exit(f"--window は {ANIM_WINDOW_MIN_S:.0f}〜{ANIM_WINDOW_MAX_S:.0f} 秒"
                 f"の範囲で指定してください: {args.window}")

    # --- モード決定（CLI フラグ優先。引数なしは対話メニュー） ---
    video_path = Path(args.video).expanduser() if args.video else None
    interactive = False
    if args.animation or video_path is not None:
        mode = "anim_video" if video_path is not None else "anim_only"
    elif args.explog:
        mode = "static"  # 従来の CLI 挙動を維持
    else:
        interactive = True
        mode = choose_mode()
        if mode is None:
            sys.exit("中止しました。")
    if mode == "static" and (args.fps or args.start is not None
                             or args.end is not None or args.window is not None):
        print("警告: --fps/--start/--end/--window はアニメ専用のため無視します"
              "（--animation を付けてください）。")

    if args.explog:
        path = Path(args.explog)
    else:
        cands = list_explogs()
        if not cands:
            sys.exit(f"explog CSV が見つかりません: {explog_dir()} に {EXPLOG_GLOB} がありません。")
        path = choose_explog(cands)
        if path is None:
            sys.exit("中止しました（CSV が選択されていません）。")
    if not path.is_file():
        sys.exit(f"explog CSV が見つかりません: {path}")

    if mode == "anim_video" and video_path is None:
        video_path = choose_video()
        if video_path is None:
            sys.exit("中止しました（動画が選択されていません）。")
    if video_path is not None and not video_path.is_file():
        sys.exit(f"動画ファイルが見つかりません: {video_path}")

    # 対話メニュー[2][3]では CSV（と動画）選択の後に窓幅を質問する
    # （--window 指定時はそれを優先し質問しない）
    window_s = args.window
    if interactive and mode in ("anim_video", "anim_only") and window_s is None:
        window_s = choose_window()
        if window_s is None:
            sys.exit("中止しました（窓幅が選択されていません）。")

    out = Path(args.out) if args.out else default_out_dir(path)
    out.mkdir(parents=True, exist_ok=True)

    data = load_explog(path)
    t = time_axis(data)
    on = motor_on_mask(data)
    meta = load_meta(path)
    ffl = ff_state_line(meta)
    tsuffix = f"\n{ffl}" if ffl else ""

    print(f"入力: {path}")
    print(f"出力: {out}")
    if meta is None:
        print("meta JSON なし → ff_state 表示は省略します。")
    elif ffl:
        print(f"FF状態: {ffl}")

    if mode in ("anim_video", "anim_only"):
        render_overview(path, out, data, t, tsuffix, video_path)
        render_animation(path, out, data, t, tsuffix, video_path,
                         args.fps, args.start, args.end, window_s)
        return

    n_figs = 0
    n_figs += fig01_yaw_comparison(data, t, on, out, tsuffix)
    n_figs += fig02_yaw_error(data, t, on, out, tsuffix)
    n_figs += fig03_current_duty(data, t, on, out, tsuffix)
    n_figs += fig04_mag_ff(data, t, on, out, tsuffix)
    n_figs += fig05_ekf_diagnostics(data, t, on, out, tsuffix)
    n_figs += fig06_ff_status(data, t, on, out, tsuffix)
    n_figs += fig07_flow(data, t, on, out, tsuffix)
    write_summary(data, t, on, path, meta, out)

    print(f"{n_figs}枚の図 + summary.txt を {out} に出力しました。")


if __name__ == "__main__":
    main()
