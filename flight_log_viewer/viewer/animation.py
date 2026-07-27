"""同期アニメーション MP4 の生成(matplotlib FuncAnimation)。

旧 Drone_Log_Viewer の 7 パネル構成を参考に、V2 列構成で再構築した。
パネル: [スマホ動画(オプション) / XY 軌跡(現在位置+目標位置)] + 高度 +
電源 + ヨー(Madgwick/EKF/指令) + 姿勢 + duty + XY 位置誤差。
スマホ動画との同期合成(OpenCV)はオプションで、動画なしでも生成できる。

- 動画あり: 出力 fps は動画 fps に合わせ、CSV(50Hz)を動画フレーム時刻へ
  線形補間する(旧実装と同じ同期方法。動画は飛行開始と同時に録画開始した
  前提。CSV の長さで動画をカットする)。
- 動画なし: 既定 20fps でログのみのアニメーションを生成。
- ROI 追跡(--track): OpenCV トラッカー(CSRT→KCF→MOSSE→MIL)で機体を
  追跡し赤枠を合成する。ROI 選択ウィンドウが開くため GUI 環境が必要。
"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import pandas as pd

from . import jp_font

jp_font.setup_japanese_font()

import matplotlib.pyplot as plt  # noqa: E402
from matplotlib.animation import FFMpegWriter, FuncAnimation  # noqa: E402

from .constants import (  # noqa: E402
    ANIM_DPI,
    AX_BG,
    COLORS,
    FIG_BG,
    GRID_ALPHA,
    GRID_COLOR,
    MULTI_DRONE_COLORS,
    TEXT_COLOR,
    YAW_SOURCES,
)
from .loader import FlightLog, wrapped_plot_series  # noqa: E402

# 既定の出力設定
DEFAULT_ANIM_FPS = 20.0        # 動画なし時のフレームレート
FIGURE_SIZE = (19.2, 10.8)     # 1920x1080 @ dpi100
TRAIL_WINDOW_S = 5.0           # 時系列パネルの表示窓 [s]
XY_TRAIL_POINTS = 150          # XY 軌跡の尾の点数
VIDEO_MAX_WIDTH_PX = 960       # 事前読み込みする動画フレームの最大幅
BITRATE_KBPS = 8000            # MP4 ビットレート

# 動画同期時に整数として最近傍補間する列(旧実装踏襲)
_INTEGER_COLUMNS = {
    "frame_number", "marker_count", "send_success", "control_active",
    "rb_marker_count", "tracking_valid", "consecutive_outliers",
    "tlm_flags", "tlm_ffg", "tlm_ff_status", "yaw_ctrl_on", "traj_mode",
    "tlm_ctrl_flags",
}


# ---------------------------------------------------------------------------
# OpenCV ヘルパー(動画同期時のみ使用)
# ---------------------------------------------------------------------------

def _import_cv2():
    try:
        import cv2  # noqa: PLC0415
        return cv2
    except ImportError as e:
        raise RuntimeError(
            "スマホ動画の同期合成には opencv-python が必要です: "
            "pip install opencv-python"
        ) from e


def _create_tracker(cv2):
    """利用可能なトラッカーを作成(CSRT → KCF → MOSSE → MIL)。

    OpenCV 4.x は `TrackerXXX_create` 関数、4.5 以降/5.x はクラスの
    `.create()` で生成する(両形式を探索)。opencv-python 5.x の本体
    パッケージには CSRT/KCF/MOSSE が含まれない(contrib へ移動)ため、
    モデルファイル不要で常に使える MIL を最後の受け皿にする
    (Vit/Nano/DaSiamRPN は ONNX モデルの別途配置が必要なため除外)。
    """
    legacy = getattr(cv2, "legacy", None)
    candidates = []
    for name in ("TrackerCSRT", "TrackerKCF", "TrackerMOSSE", "TrackerMIL"):
        for module in (legacy, cv2):
            if module is None:
                continue
            ctor = getattr(module, f"{name}_create", None)
            if ctor is not None:
                candidates.append(ctor)
            cls = getattr(module, name, None)
            if cls is not None and hasattr(cls, "create"):
                candidates.append(cls.create)
    for ctor in candidates:
        try:
            tracker = ctor()
            print(f"トラッカー: {tracker.__class__.__name__} を使用します。")
            return tracker
        except Exception:  # noqa: BLE001 (ビルド差異による生成失敗は次候補へ)
            continue
    print("警告: 利用可能な OpenCV トラッカーが無いため追跡を無効化します。\n"
          "  (高精度な CSRT/KCF を使うには opencv-contrib-python の導入か\n"
          "   opencv-python 4.x への変更を検討してください)")
    return None


def _load_video_frames(video_path: Path, offset_s: float, csv_duration_s: float,
                       track: bool) -> tuple[list[np.ndarray], float]:
    """動画フレームを RGB で読み込む(CSV 長でカット、必要なら縮小・追跡枠)。

    動画はログ記録開始と同時に録画開始した前提。--start による切り出し時は
    offset_s(ログ先頭からの秒数)だけ動画側も読み飛ばして同期を保つ。

    Returns: (frames, video_fps)
    """
    cv2 = _import_cv2()
    cap = cv2.VideoCapture(str(video_path))
    if not cap.isOpened():
        raise ValueError(f"動画ファイルを開けません: {video_path}")

    video_fps = cap.get(cv2.CAP_PROP_FPS) or DEFAULT_ANIM_FPS
    frame_count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    skip_frames = int(round(max(offset_s, 0.0) * video_fps))
    sync_frames = min(int(csv_duration_s * video_fps), frame_count - skip_frames)
    if sync_frames < 2:
        raise ValueError("同期可能な動画フレームが不足しています。")
    print(f"動画情報: {video_fps:.2f}fps, {frame_count}フレーム → "
          f"読み飛ばし {skip_frames} + 同期 {sync_frames}フレーム")

    tracker = None
    if track:
        cap.set(cv2.CAP_PROP_POS_FRAMES, skip_frames)
        ret, first_frame = cap.read()
        if not ret:
            raise ValueError("動画の最初のフレームを読み込めませんでした。")
        print("\n最初のフレームが表示されます。ドラッグしてドローンを囲んでください。")
        roi = cv2.selectROI("ROI選択 (Escでキャンセル)", first_frame,
                            fromCenter=False, showCrosshair=True)
        cv2.destroyAllWindows()
        x, y, w, h = roi
        if w > 0 and h > 0:
            tracker = _create_tracker(cv2)
            if tracker is not None:
                tracker.init(first_frame, roi)
        else:
            print("ROI が指定されなかったため追跡なしで続行します。")

    # 読み出し位置を同期開始フレームに合わせる
    cap.set(cv2.CAP_PROP_POS_FRAMES, skip_frames)

    frames: list[np.ndarray] = []
    for i in range(sync_frames):
        ret, frame = cap.read()
        if not ret:
            break
        if tracker is not None:
            ok, bbox = tracker.update(frame)
            if ok:
                px, py, pw, ph = (int(v) for v in bbox)
                cv2.rectangle(frame, (px, py), (px + pw, py + ph), (0, 0, 255), 2)
                cv2.putText(frame, "Tracking", (px, max(0, py - 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            else:
                cv2.putText(frame, "Tracking lost", (20, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3, cv2.LINE_AA)
        # メモリ節約のため縮小してから RGB 変換
        height, width = frame.shape[:2]
        if width > VIDEO_MAX_WIDTH_PX:
            scale = VIDEO_MAX_WIDTH_PX / width
            frame = cv2.resize(frame, (VIDEO_MAX_WIDTH_PX, int(height * scale)))
        frames.append(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        if i % 150 == 0:
            print(f"  動画フレーム読み込み {i}/{sync_frames}")
    cap.release()
    return frames, float(video_fps)


# ---------------------------------------------------------------------------
# CSV → アニメーション時刻への補間
# ---------------------------------------------------------------------------

def _interpolate_to_frames(df: pd.DataFrame, frame_times: np.ndarray) -> pd.DataFrame:
    """数値列をアニメーションのフレーム時刻へ線形補間する(旧実装踏襲)。

    NaN 区間は補間せず NaN のまま残す(np.interp が NaN を拡散しないよう
    有限値のみで補間する)。
    """
    csv_times = df["elapsed_time"].to_numpy(dtype=float)
    out = pd.DataFrame({"elapsed_time": frame_times})
    for col in df.columns:
        if col == "elapsed_time" or not pd.api.types.is_numeric_dtype(df[col]):
            continue
        values = df[col].to_numpy(dtype=float)
        finite = np.isfinite(values)
        if finite.sum() < 2:
            out[col] = np.nan
            continue
        interp = np.interp(frame_times, csv_times[finite], values[finite],
                           left=np.nan, right=np.nan)
        if col in _INTEGER_COLUMNS:
            interp = np.round(interp)
        out[col] = interp
    return out


# ---------------------------------------------------------------------------
# パネル構築
# ---------------------------------------------------------------------------

def _style_ax(ax) -> None:
    ax.set_facecolor(AX_BG)
    ax.tick_params(colors=TEXT_COLOR, labelsize=8)
    ax.title.set_color(TEXT_COLOR)
    ax.xaxis.label.set_color(TEXT_COLOR)
    ax.yaxis.label.set_color(TEXT_COLOR)
    ax.grid(True, alpha=GRID_ALPHA, color=GRID_COLOR)
    for spine in ax.spines.values():
        spine.set_edgecolor(TEXT_COLOR)


def _legend(ax, **kwargs) -> None:
    kwargs.setdefault("loc", "upper right")
    kwargs.setdefault("fontsize", 7)
    kwargs.setdefault("framealpha", 0.8)
    leg = ax.legend(**kwargs)
    if leg is not None:
        leg.get_frame().set_facecolor(AX_BG)
        leg.get_frame().set_edgecolor("#cccccc")
        for text in leg.get_texts():
            text.set_color(TEXT_COLOR)


def _build_ts_panel(ax, df: pd.DataFrame, ylabel: str,
                    specs: tuple[tuple[str, str, str, str], ...],
                    wrap_angle: bool = False) -> list:
    """時系列パネルの Line2D 群を作る。specs=(列名, 表示名, 色, 線種)。

    データが無い列はスキップし、y 範囲は全データから固定する
    (フレーム毎の再計算を避ける)。単機・複数機アニメで共用。
    wrap_angle=True では値を ±180° にラップし、ラップ跨ぎに NaN を
    挿入して縦線を防ぐ(ヨー系パネル用。線ごとに時間軸が伸びるため、
    戻り値は (Line2D, 時間配列, 値配列) のタプルのリスト)。
    """
    _style_ax(ax)
    ax.set_ylabel(ylabel, fontsize=9)
    ax.set_xlabel("時間 [s]", fontsize=8)
    t = df["elapsed_time"].to_numpy(dtype=float)
    lines = []
    for col, label, color, linestyle in specs:
        if col not in df.columns:
            continue
        values = df[col].to_numpy(dtype=float)
        if not np.isfinite(values).any():
            continue
        if wrap_angle:
            t_line, values = wrapped_plot_series(t, values)
        else:
            t_line = t
        (line,) = ax.plot([], [], color=color, linewidth=1.2, alpha=0.9,
                          linestyle=linestyle, label=label)
        lines.append((line, t_line, values))
    if lines:
        all_values = np.concatenate([v for _, _, v in lines])
        finite = all_values[np.isfinite(all_values)]
        if finite.size:
            low, high = float(np.min(finite)), float(np.max(finite))
            pad = max((high - low) * 0.1, 1e-3)
            ax.set_ylim(low - pad, high + pad)
        _legend(ax, ncol=max(1, len(lines) // 2))
    return lines


def _update_ts_panels(ts_panels, now: float) -> None:
    """時系列パネル群を時刻 now までの表示窓に更新する(単機・複数機共用)。"""
    window_lo = now - TRAIL_WINDOW_S
    for ax, lines in ts_panels:
        for line, t_line, values in lines:
            lo = int(np.searchsorted(t_line, window_lo))
            hi = int(np.searchsorted(t_line, now, side="right"))
            line.set_data(t_line[lo:hi], values[lo:hi])
        ax.set_xlim(window_lo, now + TRAIL_WINDOW_S * 0.1)


class _AnimationBuilder:
    """7 パネルアニメーションの Figure・Artist を構築し、フレーム更新する。"""

    def __init__(self, log: FlightLog, frames_df: pd.DataFrame,
                 video_frames: list[np.ndarray] | None) -> None:
        self.log = log
        self.df = frames_df
        self.video_frames = video_frames
        self.t = frames_df["elapsed_time"].to_numpy(dtype=float)

        self.fig = plt.figure(figsize=FIGURE_SIZE, dpi=ANIM_DPI)
        self.fig.patch.set_facecolor(FIG_BG)
        gs = self.fig.add_gridspec(
            3, 4, left=0.05, right=0.98, top=0.94, bottom=0.06,
            wspace=0.25, hspace=0.35,
            width_ratios=[1.2, 1.0, 1.0, 1.0], height_ratios=[1.0, 1.0, 1.0],
        )

        # --- パネル1: 動画(あれば)/ XY 大画面(なければ) ---
        self.ax_main = self.fig.add_subplot(gs[0:2, 0:2])
        if video_frames:
            self.ax_main.set_title("ドローン飛行映像", fontsize=13, color=TEXT_COLOR)
            self.ax_main.axis("off")
            self.im_video = self.ax_main.imshow(video_frames[0])
            self.ax_xy = self.fig.add_subplot(gs[0, 2])
        else:
            self.im_video = None
            self.ax_xy = self.ax_main

        # --- XY 軌跡パネル ---
        _style_ax(self.ax_xy)
        self.ax_xy.set_title("XY 軌跡", fontsize=11)
        self.ax_xy.set_xlabel("X [m]", fontsize=9)
        self.ax_xy.set_ylabel("Y [m]", fontsize=9)
        self.has_pos = self.log.has("pos_x") and self.log.has("pos_y")
        self.pt_target = None
        if self.has_pos:
            # 目標位置(target_x/target_y)は現在値をマーカーで動かす
            # (軌道線は円軌道以外ではほぼ点になり見えないため廃止)
            self.has_target = ("target_x" in self.df.columns and np.isfinite(
                self.df["target_x"].to_numpy(dtype=float)).any())
            if self.has_target:
                (self.pt_target,) = self.ax_xy.plot(
                    [], [], marker="o", markersize=13, markerfacecolor="none",
                    markeredgecolor=COLORS["target"], markeredgewidth=2.5,
                    linestyle="none", label="目標位置")
            (self.ln_trail,) = self.ax_xy.plot(
                [], [], color=COLORS["trajectory"], linewidth=1.5, alpha=0.9,
                label="軌跡")
            (self.pt_current,) = self.ax_xy.plot(
                [], [], marker="o", markersize=10, color=COLORS["current_pos"],
                markeredgecolor="#111111", linestyle="none", label="現在位置")
            # 軸範囲は実位置+目標位置の全範囲から固定(目標が視野外に
            # 出ないようにする)
            xs = [self.df["pos_x"].to_numpy(dtype=float)]
            ys = [self.df["pos_y"].to_numpy(dtype=float)]
            if self.has_target:
                xs.append(self.df["target_x"].to_numpy(dtype=float))
                ys.append(self.df["target_y"].to_numpy(dtype=float))
            x = np.concatenate(xs)
            y = np.concatenate(ys)
            if np.isfinite(x).any():
                margin = 0.2
                self.ax_xy.set_xlim(np.nanmin(x) - margin, np.nanmax(x) + margin)
                self.ax_xy.set_ylim(np.nanmin(y) - margin, np.nanmax(y) + margin)
            self.ax_xy.set_aspect("equal", adjustable="box")
            _legend(self.ax_xy, loc="upper right")
        else:
            self.ax_xy.text(0.5, 0.5, "位置データなし\n(Posture モード)",
                            transform=self.ax_xy.transAxes, ha="center", va="center",
                            color="gray", fontsize=11)

        # --- 高度パネル ---
        slot_alt = gs[0, 3] if video_frames else gs[0, 2]
        self.ax_alt = self.fig.add_subplot(slot_alt)
        self.alt_lines = self._make_ts_panel(
            self.ax_alt, "高度 [m]",
            (("alt_ref_m", "目標", COLORS["alt_ref"], "--"),
             ("tlm_altitude_est_m", "推定", COLORS["alt_est"], "-"),
             ("tlm_altitude_tof_m", "ToF", COLORS["alt_tof"], "-")))

        # --- 電源パネル ---
        slot_power = gs[1, 2] if video_frames else gs[0, 3]
        self.ax_power = self.fig.add_subplot(slot_power)
        self.power_lines = self._make_ts_panel(
            self.ax_power, "電圧 [V] / 電流 [A]",
            (("tlm_voltage_v", "電圧", COLORS["voltage"], "-"),
             ("tlm_current_a", "電流", COLORS["current"], "-")))

        # --- ヨーパネル(Madgwick / EKF / MoCap 正解Yaw / 指令) ---
        # MoCap は v5 の正解Yaw(mocap_yaw_true_deg)のみ表示(旧列は軸違い
        # のため 2026-07 に除外、正解Yaw 導入で復帰)。ジャイロ積算は参考値の
        # ため表示しない。v4 以前のログでは MoCap 系統は自動スキップされる。
        slot_yaw = gs[1, 3] if video_frames else gs[1, 2:]
        self.ax_yaw = self.fig.add_subplot(slot_yaw)
        # ラップ表示のため補間はアンラップ列で行い、描画時に ±180° へ畳む
        yaw_specs = []
        for key, _col, label, color, _deg in YAW_SOURCES:
            if key not in ("madgwick", "ekf", "mocap"):
                continue
            yaw_specs.append((f"yaw_{key}_unwrap_deg", label, color, "-"))
        yaw_specs.append(("cmd_yaw_ref_deg", "指令", COLORS["yaw_cmd"], "--"))
        self.yaw_lines = self._make_ts_panel(
            self.ax_yaw, "ヨー [deg]（±180）", tuple(yaw_specs),
            wrap_angle=True)

        # --- 姿勢パネル ---
        self.ax_att = self.fig.add_subplot(gs[2, 0:2])
        self.att_lines = self._make_ts_panel(
            self.ax_att, "姿勢 [deg]",
            (("roll_ref_deg", "Roll指令", COLORS["cmd_roll"], "--"),
             ("tlm_roll_deg", "Roll実測", COLORS["meas_roll"], "-"),
             ("pitch_ref_deg", "Pitch指令", COLORS["cmd_pitch"], "--"),
             ("tlm_pitch_deg", "Pitch実測", COLORS["meas_pitch"], "-")))

        # --- duty パネル ---
        self.ax_duty = self.fig.add_subplot(gs[2, 2])
        self.duty_lines = self._make_ts_panel(
            self.ax_duty, "duty (0-1)",
            (("tlm_duty_fl", "FL", COLORS["duty_fl"], "-"),
             ("tlm_duty_fr", "FR", COLORS["duty_fr"], "-"),
             ("tlm_duty_rl", "RL", COLORS["duty_rl"], "-"),
             ("tlm_duty_rr", "RR", COLORS["duty_rr"], "-")))

        # --- XY 位置誤差パネル ---
        # 旧ヨー誤差パネル(対 MoCap 真値)は MoCap ヨーの信頼性が低いため
        # 廃止し、位置制御実験で有用な XY 追従誤差に差し替え(2026-07)
        self.ax_xy_err = self.fig.add_subplot(gs[2, 3])
        self.xy_err_lines = self._make_ts_panel(
            self.ax_xy_err, "XY 位置誤差 [m]",
            (("error_x", "誤差 X", COLORS["err_x"], "-"),
             ("error_y", "誤差 Y", COLORS["err_y"], "-")))
        if self.xy_err_lines:
            self.ax_xy_err.axhline(y=0, color="gray", linestyle="--", alpha=0.5)

        self.ts_panels = (
            (self.ax_alt, self.alt_lines),
            (self.ax_power, self.power_lines),
            (self.ax_yaw, self.yaw_lines),
            (self.ax_att, self.att_lines),
            (self.ax_duty, self.duty_lines),
            (self.ax_xy_err, self.xy_err_lines),
        )
        self.title = self.fig.suptitle("", fontsize=13, color=TEXT_COLOR)

    def _make_ts_panel(self, ax, ylabel: str,
                       specs: tuple[tuple[str, str, str, str], ...],
                       wrap_angle: bool = False) -> list:
        """時系列パネルの Line2D 群を作る。specs=(列名, 表示名, 色, 線種)。"""
        return _build_ts_panel(ax, self.df, ylabel, specs, wrap_angle=wrap_angle)

    def update(self, frame_idx: int):
        """FuncAnimation のフレーム更新コールバック。"""
        now = self.t[frame_idx]

        if self.im_video is not None and frame_idx < len(self.video_frames):
            self.im_video.set_data(self.video_frames[frame_idx])

        if self.has_pos:
            start = max(0, frame_idx - XY_TRAIL_POINTS)
            self.ln_trail.set_data(
                self.df["pos_x"].iloc[start:frame_idx + 1],
                self.df["pos_y"].iloc[start:frame_idx + 1])
            px = self.df["pos_x"].iloc[frame_idx]
            py = self.df["pos_y"].iloc[frame_idx]
            if np.isfinite(px) and np.isfinite(py):
                self.pt_current.set_data([px], [py])
            if self.pt_target is not None:
                tx = self.df["target_x"].iloc[frame_idx]
                ty = self.df["target_y"].iloc[frame_idx]
                if np.isfinite(tx) and np.isfinite(ty):
                    self.pt_target.set_data([tx], [ty])

        _update_ts_panels(self.ts_panels, now)

        self.title.set_text(
            f"{self.log.name}   t={now:6.2f}s   モード: {self.log.mode}")
        return []


# ---------------------------------------------------------------------------
# 生成エントリポイント
# ---------------------------------------------------------------------------

def generate_animation(
    log: FlightLog,
    out_path: str | Path,
    video_path: str | Path | None = None,
    fps: float | None = None,
    start_s: float | None = None,
    end_s: float | None = None,
    track: bool = False,
) -> Path:
    """アニメーション MP4 を生成する。

    Args:
        log: 読み込み済みフライトログ。
        out_path: 出力 MP4 パス。
        video_path: スマホ動画(省略時はログのみ)。
        fps: 出力フレームレート(省略時: 動画あり=動画 fps / なし=20fps)。
        start_s / end_s: ログの切り出し範囲 [s](elapsed_time 基準)。
        track: 動画に ROI 追跡枠を合成する(GUI 必要)。
    """
    if not FFMpegWriter.isAvailable():
        raise RuntimeError(
            "ffmpeg が見つかりません。MP4 出力には ffmpeg のインストールが必要です"
            "(macOS: brew install ffmpeg)。"
        )

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    df = log.df
    t_all = log.t
    t0 = float(t_all[0]) if start_s is None else float(start_s)
    t1 = float(t_all[-1]) if end_s is None else float(end_s)
    if t1 <= t0:
        raise ValueError(f"切り出し範囲が不正です: start={t0}, end={t1}")
    duration = t1 - t0

    video_frames: list[np.ndarray] | None = None
    if video_path is not None:
        # 動画は「ログ記録開始と同時に録画開始」前提でログ長にカットして同期
        offset_s = t0 - float(t_all[0])
        video_frames, video_fps = _load_video_frames(
            Path(video_path), offset_s, duration, track)
        anim_fps = float(fps) if fps else video_fps
    else:
        anim_fps = float(fps) if fps else DEFAULT_ANIM_FPS

    n_frames = int(duration * anim_fps)
    if video_frames is not None:
        n_frames = min(n_frames, len(video_frames))
    if n_frames < 2:
        raise ValueError("アニメーションのフレーム数が不足しています。")
    frame_times = t0 + np.arange(n_frames) / anim_fps

    print(f"\nアニメーション生成: {n_frames}フレーム @ {anim_fps:.1f}fps → {out_path}")
    frames_df = _interpolate_to_frames(df, frame_times)
    builder = _AnimationBuilder(log, frames_df, video_frames)

    writer = FFMpegWriter(
        fps=anim_fps,
        metadata={"title": f"StampFly Flight Log: {log.name}"},
        codec="libx264",
        bitrate=BITRATE_KBPS,
        extra_args=["-pix_fmt", "yuv420p"],
    )
    anim = FuncAnimation(builder.fig, builder.update, frames=n_frames, blit=False)

    last_pct = -1

    def _progress(current: int, total: int) -> None:
        nonlocal last_pct
        pct = int(100 * current / total)
        if pct >= last_pct + 10:
            last_pct = pct
            print(f"  進捗: {pct}% ({current}/{total})")

    anim.save(str(out_path), writer=writer, dpi=ANIM_DPI,
              progress_callback=_progress)
    plt.close(builder.fig)
    print(f"アニメーション生成完了: {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# 複数機同時制御(multi)アニメーション
# ---------------------------------------------------------------------------

class _MultiAnimationBuilder:
    """複数機アニメ: 共有 XY 大パネル + 機体別の高度/ヨー小パネル。

    レイアウトは gridspec(機体数 N × 4 列)。左 2 列を全行ぶち抜きで
    共有 XY パネルに使い、右 2 列に機体 i の高度(col2)/ヨー(col3)を
    行 i に並べる(N=2〜4 を想定)。動画合成は multi では非対応(仕様)。
    """

    def __init__(self, logs: list[FlightLog],
                 frames_dfs: list[pd.DataFrame],
                 frame_times: np.ndarray) -> None:
        self.logs = logs
        self.t = frame_times
        n = len(logs)

        self.fig = plt.figure(figsize=FIGURE_SIZE, dpi=ANIM_DPI)
        self.fig.patch.set_facecolor(FIG_BG)
        gs = self.fig.add_gridspec(
            n, 4, left=0.05, right=0.98, top=0.90, bottom=0.07,
            wspace=0.28, hspace=0.55,
            width_ratios=[1.15, 1.15, 1.0, 1.0],
        )

        # --- 共有 XY パネル(全機の trail+現在位置+目標を機体別色で) ---
        self.ax_xy = self.fig.add_subplot(gs[:, 0:2])
        _style_ax(self.ax_xy)
        self.ax_xy.set_title("複数機 XY 軌跡(共有)", fontsize=12)
        self.ax_xy.set_xlabel("X [m]", fontsize=9)
        self.ax_xy.set_ylabel("Y [m]", fontsize=9)

        # (has_pos, trail, point, frames_df) を機体ごとに保持
        self.xy_artists: list[tuple[bool, object, object, pd.DataFrame]] = []
        all_x: list[np.ndarray] = []
        all_y: list[np.ndarray] = []
        for i, (log, df) in enumerate(zip(logs, frames_dfs)):
            color = MULTI_DRONE_COLORS[i % len(MULTI_DRONE_COLORS)]
            name = log.drone_name or log.name
            has_pos = log.has("pos_x") and log.has("pos_y")
            trail = point = None
            if has_pos:
                if "target_x" in df.columns and np.isfinite(
                        df["target_x"].to_numpy(dtype=float)).any():
                    self.ax_xy.plot(df["target_x"], df["target_y"],
                                    color=color, linewidth=0.9,
                                    linestyle="--", alpha=0.45,
                                    label=f"{name} 目標")
                (trail,) = self.ax_xy.plot(
                    [], [], color=color, linewidth=1.6, alpha=0.9,
                    label=f"{name} 軌跡")
                (point,) = self.ax_xy.plot(
                    [], [], marker="o", markersize=11, color=color,
                    markeredgecolor="#111111", linestyle="none")
                all_x.append(df["pos_x"].to_numpy(dtype=float))
                all_y.append(df["pos_y"].to_numpy(dtype=float))
            self.xy_artists.append((has_pos, trail, point, df))

        if all_x:
            x = np.concatenate(all_x)
            y = np.concatenate(all_y)
            if np.isfinite(x).any():
                margin = 0.2
                self.ax_xy.set_xlim(np.nanmin(x) - margin, np.nanmax(x) + margin)
                self.ax_xy.set_ylim(np.nanmin(y) - margin, np.nanmax(y) + margin)
            self.ax_xy.set_aspect("equal", adjustable="box")
            _legend(self.ax_xy, loc="upper right", ncol=2)
        else:
            self.ax_xy.text(0.5, 0.5, "位置データなし",
                            transform=self.ax_xy.transAxes,
                            ha="center", va="center", color="gray", fontsize=11)

        # --- 機体別の高度/ヨー小パネル ---
        self.ts_panels: list[tuple[object, list]] = []
        for i, (log, df) in enumerate(zip(logs, frames_dfs)):
            color = MULTI_DRONE_COLORS[i % len(MULTI_DRONE_COLORS)]
            name = log.drone_name or log.name

            ax_alt = self.fig.add_subplot(gs[i, 2])
            alt_lines = _build_ts_panel(
                ax_alt, df, "高度 [m]",
                (("alt_ref_m", "目標", COLORS["alt_ref"], "--"),
                 ("tlm_altitude_est_m", "推定", COLORS["alt_est"], "-"),
                 ("tlm_altitude_tof_m", "ToF", COLORS["alt_tof"], "-")))
            ax_alt.set_title(f"{name} 高度", fontsize=10, color=color)

            # ラップ表示のため補間はアンラップ列で行い、描画時に ±180° へ畳む
            ax_yaw = self.fig.add_subplot(gs[i, 3])
            yaw_lines = _build_ts_panel(
                ax_yaw, df, "ヨー [deg]（±180）",
                (("yaw_ekf_unwrap_deg", "EKF", COLORS["yaw_ekf"], "-"),
                 ("yaw_mocap_unwrap_deg", "MoCap", COLORS["yaw_mocap"], "-"),
                 ("cmd_yaw_ref_deg", "指令", COLORS["yaw_cmd"], "--")),
                wrap_angle=True)
            ax_yaw.set_title(f"{name} ヨー", fontsize=10, color=color)

            self.ts_panels += [(ax_alt, alt_lines), (ax_yaw, yaw_lines)]

        self.title = self.fig.suptitle("", fontsize=13, color=TEXT_COLOR)
        self._names = " / ".join(
            (log.drone_name or log.name) for log in logs)

    def update(self, frame_idx: int):
        """FuncAnimation のフレーム更新コールバック。"""
        now = self.t[frame_idx]

        for has_pos, trail, point, df in self.xy_artists:
            if not has_pos:
                continue
            start = max(0, frame_idx - XY_TRAIL_POINTS)
            trail.set_data(df["pos_x"].iloc[start:frame_idx + 1],
                           df["pos_y"].iloc[start:frame_idx + 1])
            px = df["pos_x"].iloc[frame_idx]
            py = df["pos_y"].iloc[frame_idx]
            if np.isfinite(px) and np.isfinite(py):
                point.set_data([px], [py])

        _update_ts_panels(self.ts_panels, now)

        self.title.set_text(
            f"複数機同時制御   t={now:6.2f}s   機体: {self._names}")
        return []


def generate_multi_animation(
    logs: list[FlightLog],
    out_path: str | Path,
    fps: float | None = None,
    start_s: float | None = None,
    end_s: float | None = None,
) -> Path:
    """複数機グループのアニメーション MP4 を生成する(動画合成なし)。

    Args:
        logs: 同一グループの読み込み済みフライトログ(2〜4 機)。
        out_path: 出力 MP4 パス。
        fps: 出力フレームレート(省略時 20fps)。
        start_s / end_s: 切り出し範囲 [s](elapsed_time 基準・全機共通)。
    """
    if not logs:
        raise ValueError("multi アニメーション対象のログがありません。")
    if not FFMpegWriter.isAvailable():
        raise RuntimeError(
            "ffmpeg が見つかりません。MP4 出力には ffmpeg のインストールが必要です"
            "(macOS: brew install ffmpeg)。"
        )

    out_path = Path(out_path)
    out_path.parent.mkdir(parents=True, exist_ok=True)

    # 全機共通の時間軸(既定: 最も早い開始〜最も遅い終了)
    t0 = min(float(log.t[0]) for log in logs) if start_s is None else float(start_s)
    t1 = max(float(log.t[-1]) for log in logs) if end_s is None else float(end_s)
    if t1 <= t0:
        raise ValueError(f"切り出し範囲が不正です: start={t0}, end={t1}")
    duration = t1 - t0

    anim_fps = float(fps) if fps else DEFAULT_ANIM_FPS
    n_frames = int(duration * anim_fps)
    if n_frames < 2:
        raise ValueError("アニメーションのフレーム数が不足しています。")
    frame_times = t0 + np.arange(n_frames) / anim_fps

    print(f"\n複数機アニメーション生成: {len(logs)}機, "
          f"{n_frames}フレーム @ {anim_fps:.1f}fps → {out_path}")
    frames_dfs = [_interpolate_to_frames(log.df, frame_times) for log in logs]
    builder = _MultiAnimationBuilder(logs, frames_dfs, frame_times)

    writer = FFMpegWriter(
        fps=anim_fps,
        metadata={"title": f"StampFly Multi Flight Log: {out_path.stem}"},
        codec="libx264",
        bitrate=BITRATE_KBPS,
        extra_args=["-pix_fmt", "yuv420p"],
    )
    anim = FuncAnimation(builder.fig, builder.update, frames=n_frames, blit=False)

    last_pct = -1

    def _progress(current: int, total: int) -> None:
        nonlocal last_pct
        pct = int(100 * current / total)
        if pct >= last_pct + 10:
            last_pct = pct
            print(f"  進捗: {pct}% ({current}/{total})")

    anim.save(str(out_path), writer=writer, dpi=ANIM_DPI,
              progress_callback=_progress)
    plt.close(builder.fig)
    print(f"複数機アニメーション生成完了: {out_path}")
    return out_path
