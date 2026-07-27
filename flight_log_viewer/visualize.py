#!/usr/bin/env python3
"""StampFly V2 フライトログ可視化ツール(対話式 CLI)。

引数なしで実行すると対話式フローに入る:
  1. 飛行ログモード選択(Posture / Position / Multi)
  2. 出力内容メニュー(モード別)
  3. データ選択(単機: CSV 番号 / Multi: タイムスタンプグループ番号)
  4. 動画同期時: logs/videos/ の動画を番号選択(パス直接入力も可)
  5. ROI 追跡枠の有無(y/N)

引数を与えるとバッチ実行もできる(例は README.md を参照)。
multi グループのバッチ実行は「--group <ts>」または
「--mode multi <代表CSVパス>」で指定する。

出力先: flight_log_viewer/output/<ログ名>/(Multi は output/<ts>_multi/)
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

# flight_log_viewer/ を import パスに追加(どこから実行しても動くように)
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from viewer.loader import (  # noqa: E402
    FlightLog,
    group_timestamp,
    load_group,
    load_log,
)

# 既定パス
DEFAULT_LOGS_DIR = _HERE.parent / "logs"       # V2 リポジトリ直下 logs/
DEFAULT_OUTPUT_DIR = _HERE / "output"

# スマホ動画の拡張子パターン(logs/videos/ の列挙用)
_VIDEO_PATTERNS = ("*.mp4", "*.mov", "*.MOV", "*.avi")

# 対話ステップ1: 飛行ログモード
_MODE_ITEMS = (
    "Posture(姿勢制御ログ)",
    "Position(位置制御ログ)",
    "Multi(複数機同時制御ログ)",
)
_MODE_KEYS = ("posture", "position", "multi")

# 対話ステップ2: 出力内容メニュー(単機)
_SINGLE_MENU = (
    "静止画グラフ一式+ヨー解析+サマリレポート",
    "アニメーション MP4(動画なし)+全区間サマリ静止画",
    "アニメーション MP4(スマホ動画と同期)+全区間サマリ静止画(動画はサムネイル)",
    "すべて生成(静止画+レポート+アニメーション)",
    "2 ログの比較(ヨー安定性+位置保持XY)",
    "インタラクティブ HTML プレーヤー(ブラウザ再生・自己完結 1 ファイル)",
)

# 対話ステップ2: 出力内容メニュー(Multi)
_MULTI_MENU = (
    "静止画+レポート(全機)",
    "複数機アニメーション MP4(共有XY・動画なし)+全区間サマリ静止画",
    "すべて生成(静止画+レポート+複数機アニメーション)",
)


# ---------------------------------------------------------------------------
# ディレクトリ解決
# ---------------------------------------------------------------------------

def _flight_logs_dir(logs_dir: Path) -> Path:
    """飛行ログ置き場: <logs_dir>/flight_logs/。"""
    return logs_dir / "flight_logs"


def _videos_dir(logs_dir: Path) -> Path:
    """スマホ動画置き場: <logs_dir>/videos/。"""
    return logs_dir / "videos"


# ---------------------------------------------------------------------------
# 対話式ヘルパー(旧 Drone_Log_Viewer の選択 UI を踏襲)
# ---------------------------------------------------------------------------

def _select_from_list(items: list[str], label: str) -> int | None:
    """番号選択 UI。選択インデックス(0 始まり)を返す。q でキャンセル。"""
    print(f"\n=== {label} ===")
    for i, item in enumerate(items, start=1):
        print(f"  {i}. {item}")
    while True:
        choice = input(f"\n{label} を選択してください (1-{len(items)}, q で中止): ").strip()
        if choice.lower() == "q":
            return None
        try:
            idx = int(choice) - 1
            if 0 <= idx < len(items):
                return idx
        except ValueError:
            pass
        print("無効な入力です。もう一度入力してください。")


def _ask_path(prompt: str) -> Path | None:
    raw = input(prompt).strip()
    return Path(raw).expanduser() if raw else None


def _collect_single_csvs(logs_dir: Path, mode: str) -> tuple[list[Path], list[str]]:
    """単機モードの候補 CSV とその表示名を返す。

    logs/flight_logs/ を suffix(_posture/_position)でフィルタし、
    旧 logs/ 直下の *.csv も後方互換で候補に含める。
    """
    suffix = f"_{mode}"
    flight_dir = _flight_logs_dir(logs_dir)
    paths: list[Path] = []
    labels: list[str] = []
    if flight_dir.is_dir():
        for p in sorted(flight_dir.glob("*.csv")):
            if p.stem.endswith(suffix):
                paths.append(p)
                labels.append(p.name)
    other = "_position" if mode == "posture" else "_posture"
    if logs_dir.is_dir():
        for p in sorted(logs_dir.glob("*.csv")):
            # 旧 logs/ 直下は後方互換で候補に含める。ただし明らかに別モード
            # (逆 suffix / multi 命名)のものは除く。
            if p.stem.endswith(other) or "_multi_" in p.stem:
                continue
            paths.append(p)
            labels.append(f"{p.name} (旧 logs/ 直下)")
    return paths, labels


def _select_single_csv(logs_dir: Path, mode: str,
                       label: str = "ログファイル") -> Path | None:
    """単機ログの CSV を番号選択させる。パス直接入力にも対応。"""
    paths, labels = _collect_single_csvs(logs_dir, mode)
    if not paths:
        print(f"\n{_flight_logs_dir(logs_dir)} に {mode} ログが見つかりません。"
              "パスを直接入力してください。")
        return _ask_path("CSV パス (空で中止): ")
    idx = _select_from_list(labels + ["(パスを直接入力する)"], label)
    if idx is None:
        return None
    if idx == len(paths):
        return _ask_path("CSV パス (空で中止): ")
    return paths[idx]


def _collect_multi_groups(logs_dir: Path) -> list[tuple[str, list[str]]]:
    """multi グループ一覧 [(ts, [機体名, ...]), ...] を ts 昇順で返す。"""
    flight_dir = _flight_logs_dir(logs_dir)
    groups: dict[str, list[str]] = {}
    if flight_dir.is_dir():
        for p in sorted(flight_dir.glob("*_multi_*.csv")):
            ts = group_timestamp(p)
            if ts is None:
                continue
            name = p.stem.split("_multi_", 1)[1]
            groups.setdefault(ts, []).append(name)
    return sorted(groups.items())


def _select_multi_group(logs_dir: Path) -> str | None:
    """multi グループ(タイムスタンプ)を番号選択させる。"""
    groups = _collect_multi_groups(logs_dir)
    if not groups:
        print(f"\n{_flight_logs_dir(logs_dir)} に multi ログ"
              "(<ts>_multi_<機体名>.csv)が見つかりません。")
        return None
    labels = [f"{ts}(機体: {', '.join(names)})" for ts, names in groups]
    idx = _select_from_list(labels, "Multi グループ")
    if idx is None:
        return None
    return groups[idx][0]


def _select_video(logs_dir: Path) -> Path | None:
    """logs/videos/ の動画を番号選択させる。パス直接入力にも対応。"""
    videos_dir = _videos_dir(logs_dir)
    files: list[Path] = []
    if videos_dir.is_dir():
        seen: set[Path] = set()
        for pattern in _VIDEO_PATTERNS:
            for p in videos_dir.glob(pattern):
                rp = p.resolve()
                if rp not in seen:
                    seen.add(rp)
                    files.append(p)
        files.sort(key=lambda p: p.name)
    if not files:
        print(f"\n{videos_dir} に動画が見つかりません。パスを直接入力してください。")
        return _ask_path("動画パス (空で中止): ")
    labels = [p.name for p in files] + ["(パスを直接入力する)"]
    idx = _select_from_list(labels, "スマホ動画")
    if idx is None:
        return None
    if idx == len(files):
        return _ask_path("動画パス (空で中止): ")
    return files[idx]


def _ask_track() -> bool:
    """ROI 追跡枠の有無(y/N)。"""
    return input("ROI 追跡枠を合成しますか? (y/N): ").strip().lower() == "y"


# ---------------------------------------------------------------------------
# 実行アクション
# ---------------------------------------------------------------------------

def _out_dir_for(log: FlightLog, output_root: Path) -> Path:
    return output_root / log.name


def _multi_base_name(logs: list[FlightLog]) -> str:
    """multi グループの出力ベース名(<ts>_multi、ts 不明時は先頭ログ名)。"""
    ts = group_timestamp(logs[0].path)
    return f"{ts}_multi" if ts else logs[0].name


def run_figures_and_report(log: FlightLog, output_root: Path) -> None:
    """静止画一式+ヨー解析図+サマリレポート。"""
    from viewer import plots, report, yaw_analysis  # noqa: PLC0415

    out_dir = _out_dir_for(log, output_root)
    stats = yaw_analysis.compute_yaw_stats(log)
    figure_paths = plots.generate_static_figures(log, out_dir)
    figure_paths += yaw_analysis.generate_yaw_figures(log, out_dir, stats)
    # 静止画(01-09, 15-17)とヨー解析(10-14)を番号順に並べて index に載せる
    figure_paths.sort(key=lambda p: p.name)
    report.generate_report(log, out_dir, figure_paths)


def run_multi_figures_and_report(logs: list[FlightLog], output_root: Path) -> Path:
    """複数機グループ: M01 共有図+機体別図一式+multi index.html。

    出力先は output_root/<ts>_multi/(グループ ts が取れない場合は先頭ログ名)。
    """
    from viewer import multi_plots, report  # noqa: PLC0415

    out_dir = output_root / _multi_base_name(logs)
    multi_plots.fig_multi_xy(logs, out_dir)
    multi_plots.make_multi_figures(logs, out_dir)
    report.generate_multi_report(logs, out_dir)
    return out_dir


def run_animation(log: FlightLog, output_root: Path,
                  video_path: Path | None, track: bool,
                  fps: float | None, start_s: float | None,
                  end_s: float | None) -> None:
    """アニメーション MP4。"""
    from viewer import animation  # noqa: PLC0415

    out_dir = _out_dir_for(log, output_root)
    suffix = "_with_video" if video_path else ""
    out_path = out_dir / f"{log.name}_animation{suffix}.mp4"
    animation.generate_animation(
        log, out_path, video_path=video_path, fps=fps,
        start_s=start_s, end_s=end_s, track=track)


def run_multi_animation(logs: list[FlightLog], output_root: Path,
                        fps: float | None, start_s: float | None,
                        end_s: float | None) -> Path:
    """複数機アニメーション MP4(共有XY・動画合成なし)。"""
    from viewer import animation  # noqa: PLC0415

    base = _multi_base_name(logs)
    out_path = output_root / base / f"{base}_animation.mp4"
    return animation.generate_multi_animation(
        logs, out_path, fps=fps, start_s=start_s, end_s=end_s)


def run_comparison(log_a: FlightLog, log_b: FlightLog, output_root: Path) -> None:
    """2 ログ比較レポート。"""
    from viewer import report  # noqa: PLC0415

    out_dir = output_root / f"compare_{log_a.name}_vs_{log_b.name}"
    report.generate_comparison(log_a, log_b, out_dir)


def run_interactive_html(log: FlightLog, output_root: Path) -> Path:
    """インタラクティブ HTML プレーヤー(自己完結 1 ファイル)を生成する。"""
    from interactive import export_interactive_html  # noqa: PLC0415

    return export_interactive_html(log, output_root)


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="StampFly V2 フライトログ可視化ツール"
                    "(引数なしで対話モード)")
    parser.add_argument("csv", nargs="?", type=Path,
                        help="対象ログ CSV(省略時は対話選択。--mode multi では"
                             "グループの代表 CSV)")
    parser.add_argument("--mode", choices=_MODE_KEYS, default=None,
                        help="ログモード。対話時はモード選択をスキップ。"
                             "バッチで multi を指定すると CSV をグループの"
                             "代表として全機を処理する")
    parser.add_argument("--group", type=str, default=None,
                        help="multi グループのタイムスタンプ(例 20260101_000000)。"
                             "指定するとバッチで multi グループを処理する")
    parser.add_argument("--figures", action="store_true",
                        help="静止画グラフ+ヨー解析+レポートを生成")
    parser.add_argument("--animation", action="store_true",
                        help="アニメーション MP4 を生成")
    parser.add_argument("--all", action="store_true",
                        help="--figures と --animation の両方")
    parser.add_argument("--interactive", action="store_true",
                        help="インタラクティブ HTML プレーヤー"
                             "(output/<ログ名>/interactive.html)を生成"
                             "(単機のみ)")
    parser.add_argument("--video", type=Path, default=None,
                        help="同期合成するスマホ動画(MP4)。--animation と併用")
    parser.add_argument("--track", action="store_true",
                        help="動画に ROI 追跡枠を合成(GUI 必要・opencv 必要)")
    parser.add_argument("--fps", type=float, default=None,
                        help="アニメーションの出力 fps(既定: 動画fps または 20)")
    parser.add_argument("--start", type=float, default=None,
                        help="アニメーション切り出し開始 [s]")
    parser.add_argument("--end", type=float, default=None,
                        help="アニメーション切り出し終了 [s]")
    parser.add_argument("--compare", type=Path, default=None,
                        help="比較対象の 2 本目のログ CSV(ヨー安定性比較)")
    parser.add_argument("--logs-dir", type=Path, default=DEFAULT_LOGS_DIR,
                        help=f"ログ置き場のルート(既定: {DEFAULT_LOGS_DIR}。"
                             "飛行ログは <logs-dir>/flight_logs/、動画は "
                             "<logs-dir>/videos/ を見る)")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_DIR,
                        help=f"出力ルート(既定: {DEFAULT_OUTPUT_DIR})")
    return parser.parse_args()


def _run_batch(args: argparse.Namespace) -> int:
    """アクションフラグ指定時のバッチ実行(単機)。"""
    log = load_log(args.csv)
    print(f"読み込み完了: {log.path}({len(log.df)}行, モード={log.mode})")

    if args.compare is not None:
        log_b = load_log(args.compare)
        run_comparison(log, log_b, args.output)
        return 0

    did_something = False
    if args.figures or args.all:
        run_figures_and_report(log, args.output)
        did_something = True
    if args.animation or args.all:
        run_animation(log, args.output, args.video, args.track,
                      args.fps, args.start, args.end)
        did_something = True
    if args.interactive:
        run_interactive_html(log, args.output)
        did_something = True
    if not did_something:
        # CSV だけ指定された場合は静止画+レポートを既定動作にする
        run_figures_and_report(log, args.output)
    return 0


def _run_batch_multi(args: argparse.Namespace) -> int:
    """multi グループのバッチ実行(--group <ts> または --mode multi <代表CSV>)。"""
    if args.compare is not None:
        print("エラー: --compare は multi グループでは使えません。")
        return 2
    if args.interactive:
        print("エラー: --interactive は multi グループでは使えません"
              "(初版は単機のみ)。")
        return 2
    if args.video is not None or args.track:
        print("警告: multi では動画合成・ROI 追跡は非対応のため無視します。")

    if args.group is not None:
        logs = load_group(args.group, logs_dir=_flight_logs_dir(args.logs_dir))
    else:
        logs = load_group(args.csv)
    names = ", ".join((log.drone_name or log.name) for log in logs)
    print(f"読み込み完了: multi グループ {len(logs)}機({names})")

    did_something = False
    if args.figures or args.all:
        run_multi_figures_and_report(logs, args.output)
        did_something = True
    if args.animation or args.all:
        run_multi_animation(logs, args.output, args.fps, args.start, args.end)
        did_something = True
    if not did_something:
        run_multi_figures_and_report(logs, args.output)
    return 0


# ---------------------------------------------------------------------------
# 対話モード
# ---------------------------------------------------------------------------

def _interactive_single(args: argparse.Namespace, mode: str) -> int:
    """単機(Posture/Position)の対話フロー(ステップ2〜5)。"""
    # ステップ2: 出力内容
    menu_idx = _select_from_list(list(_SINGLE_MENU), "出力内容")
    if menu_idx is None:
        print("中止しました。")
        return 1

    # ステップ3: データ選択
    csv_path = _select_single_csv(args.logs_dir, mode)
    if csv_path is None:
        print("中止しました。")
        return 1
    log = load_log(csv_path)
    print(f"\n読み込み完了: {log.path}({len(log.df)}行, モード={log.mode})")

    if menu_idx == 4:  # 2 ログのヨー比較
        csv_b = _select_single_csv(args.logs_dir, mode, "比較対象ログ")
        if csv_b is None:
            print("中止しました。")
            return 1
        log_b = load_log(csv_b)
        run_comparison(log, log_b, args.output)
        print(f"\n=== 処理完了 === 出力先: "
              f"{args.output / f'compare_{log.name}_vs_{log_b.name}'}")
        return 0

    if menu_idx == 5:  # インタラクティブ HTML プレーヤー
        html_path = run_interactive_html(log, args.output)
        print(f"\n=== 処理完了 === 出力先: {html_path}")
        return 0

    # ステップ4: 動画選択(動画同期アニメーションのみ)
    video_path: Path | None = None
    track = False
    if menu_idx == 2:
        video_path = _select_video(args.logs_dir)
        if video_path is None:
            print("中止しました。")
            return 1
        if not video_path.is_file():
            print(f"エラー: 動画が見つかりません: {video_path}")
            return 1
        # ステップ5: ROI 追跡枠
        track = _ask_track()

    if menu_idx in (0, 3):
        run_figures_and_report(log, args.output)
    if menu_idx in (1, 2, 3):
        run_animation(log, args.output, video_path, track,
                      args.fps, args.start, args.end)

    print(f"\n=== 処理完了 === 出力先: {_out_dir_for(log, args.output)}")
    return 0


def _interactive_multi(args: argparse.Namespace) -> int:
    """Multi(複数機同時制御)の対話フロー(ステップ2〜3)。"""
    # ステップ2: 出力内容
    menu_idx = _select_from_list(list(_MULTI_MENU), "出力内容")
    if menu_idx is None:
        print("中止しました。")
        return 1

    # ステップ3: グループ選択
    ts = _select_multi_group(args.logs_dir)
    if ts is None:
        print("中止しました。")
        return 1
    logs = load_group(ts, logs_dir=_flight_logs_dir(args.logs_dir))
    names = ", ".join((log.drone_name or log.name) for log in logs)
    print(f"\n読み込み完了: multi グループ {ts}({len(logs)}機: {names})")

    if menu_idx in (0, 2):
        run_multi_figures_and_report(logs, args.output)
    if menu_idx in (1, 2):
        run_multi_animation(logs, args.output, args.fps, args.start, args.end)

    print(f"\n=== 処理完了 === 出力先: {args.output / _multi_base_name(logs)}")
    return 0


def _run_interactive(args: argparse.Namespace) -> int:
    """対話モード(ステップ1: 飛行ログモード選択)。"""
    print("=== StampFly V2 フライトログ可視化ツール ===")
    print(f"飛行ログ置き場: {_flight_logs_dir(args.logs_dir)}")
    print(f"スマホ動画置き場: {_videos_dir(args.logs_dir)}")

    mode = args.mode
    if mode is None:
        mode_idx = _select_from_list(list(_MODE_ITEMS), "飛行ログモード")
        if mode_idx is None:
            print("中止しました。")
            return 1
        mode = _MODE_KEYS[mode_idx]

    if mode == "multi":
        return _interactive_multi(args)
    return _interactive_single(args, mode)


def main() -> int:
    args = _parse_args()
    try:
        if args.group is not None or (args.mode == "multi" and args.csv is not None):
            return _run_batch_multi(args)
        if args.csv is not None:
            return _run_batch(args)
        return _run_interactive(args)
    except KeyboardInterrupt:
        print("\n中止しました。")
        return 130
    except (OSError, ValueError, RuntimeError) as e:
        print(f"エラー: {e}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
