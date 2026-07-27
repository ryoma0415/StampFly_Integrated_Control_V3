#!/usr/bin/env python3
"""インタラクティブ HTML プレーヤーのエクスポータ CLI。

フライトログ CSV 1 本を読み込み、契約の 59 系列を little-endian float32
→ base64 でテンプレート(viewer/interactive_template.html)に埋め込んだ
自己完結 HTML を output/<ログ名>/interactive.html に出力する。

データ契約(59 系列の格納順・単位・注入プレースホルダ)は
INTERACTIVE_VIEWER_SPEC.md「データ契約」に従う。**系列の格納順自体が契約**で
あり、テンプレート側の実装と 1 対 1 で一致させること(名前は JSON に
含めない)。欠損値は float32 の NaN のまま格納する(テンプレートは NaN で
線を切る)。

使い方:
  .venv/bin/python interactive.py <ログCSVパス> [--output <dir>] [--open]

- multi ログは初版では非対応(単機 Posture / Position のみ)。
- 生成後、書き出した HTML から base64 を自分でデコードして系列数・長さ・
  NaN 率を元系列と照合する自己検証を必ず実行し、要約を標準出力に出す。
"""

from __future__ import annotations

import argparse
import base64
import json
import re
import sys
import webbrowser
from datetime import datetime
from pathlib import Path

import numpy as np

# flight_log_viewer/ を import パスに追加(どこから実行しても動くように)
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE))

from viewer.constants import LOG_RATE_HZ  # noqa: E402
from viewer.loader import FlightLog, load_log, wrap_deg  # noqa: E402

# 既定の出力ルート(visualize.py と同じ)
DEFAULT_OUTPUT_DIR = _HERE / "output"

# プレーヤー本体のテンプレート(HTML+CSS+JS 1 ファイル。別途生成される)
TEMPLATE_PATH = _HERE / "viewer" / "interactive_template.html"

# テンプレート内の注入プレースホルダ(契約行の完全一致で置換する。
# 部分文字列だけを対象にするとテンプレート内コメント等の同文字列を
# 誤って置換し得るため、`const EMBED = …;` の行全体を対象にする)
EMBED_PLACEHOLDER = "const EMBED = /*__EMBED_JSON__*/null;"

# EMBED スキーマのバージョン
EMBED_VERSION = 2   # v2: 59系列(yaw_mocap 追加。2026-07)

# ---------------------------------------------------------------------------
# 系列定義(59 系列。この格納順が契約 — INTERACTIVE_VIEWER_SPEC.md)
# ---------------------------------------------------------------------------

SERIES_NAMES: tuple[str, ...] = (
    # 0: 時刻 [s]
    "t",
    # 1-3: 高度 [m]
    "alt_est", "alt_tof", "alt_ref",
    # 4-7: 位置と目標 [m]
    "pos_x", "pos_y", "target_x", "target_y",
    # 8-14: 姿勢と姿勢指令 [deg](yaw 系は ±180)
    "roll", "pitch", "yaw_mad", "yaw_ekf",
    "roll_ref", "pitch_ref", "yaw_ref",
    # 15-17: 姿勢誤差 [deg]
    "err_roll", "err_pitch", "err_yaw",
    # 18-26: 角速度・指令・誤差 [deg/s]
    "p", "q", "r", "p_ref", "q_ref", "r_ref",
    "err_p", "err_q", "err_r",
    # 27-35: 角度ループ PID 成分 [deg/s]
    "pid_ang_roll_p", "pid_ang_roll_i", "pid_ang_roll_d",
    "pid_ang_pitch_p", "pid_ang_pitch_i", "pid_ang_pitch_d",
    "pid_ang_yaw_p", "pid_ang_yaw_i", "pid_ang_yaw_d",
    # 36-44: 角速度ループ PID 成分 [-(出力)]
    "pid_rate_roll_p", "pid_rate_roll_i", "pid_rate_roll_d",
    "pid_rate_pitch_p", "pid_rate_pitch_i", "pid_rate_pitch_d",
    "pid_rate_yaw_p", "pid_rate_yaw_i", "pid_rate_yaw_d",
    # 45-49: duty [0-1]・総電流 [A]
    "duty_fr", "duty_fl", "duty_rr", "duty_rl",
    "current_a",
    # 50-54: FF 補正 ΔB̂ [µT]・EKF 磁気バイアス [µT]
    "ff_x", "ff_y", "ff_norm",
    "bm_x", "bm_y",
    # 55-56: 高度速度と指令 [m/s]
    "alt_vel", "z_dot_ref",
    # 57: 制御フラグ(bit0=xy_onboard, bit1=yaw_ctrl, bit2=flying。NaN=未受信)
    "ctrl_flags",
    # 58: MoCap 正解ヨー [deg ±180](v5 ログ mocap_yaw_true_deg。
    #     v4 以前のログでは全 NaN — テンプレートは線を描かない)
    "yaw_mocap",
)
N_SERIES = len(SERIES_NAMES)
assert N_SERIES == 59, f"系列数が契約(59)と不一致: {N_SERIES}"

# 生成 HTML から EMBED を抽出する正規表現(自己検証用。JSON は 1 行で注入)
_EMBED_RE = re.compile(r"const EMBED = (\{.*\});")


# ---------------------------------------------------------------------------
# 系列構築
# ---------------------------------------------------------------------------

def _column(log: FlightLog, name: str) -> np.ndarray:
    """列を float 配列で返す(無い列は全 NaN — 系列数は常に 59 を保つ)。"""
    if name in log.df.columns:
        return log.df[name].to_numpy(dtype=float)
    return np.full(len(log.df), np.nan)


def _deg(log: FlightLog, name: str) -> np.ndarray:
    """rad 列を deg 単位で返す(NaN 保持)。"""
    return np.degrees(_column(log, name))


def build_series(log: FlightLog) -> dict[str, np.ndarray]:
    """契約の 59 系列を格納順の dict で構築する(float64・NaN 保持)。

    - rad 列は deg に変換、yaw 系は ±180° に折り返す(ラップ跨ぎの線切りは
      テンプレート側の表示処理)。
    - err 系・ff_norm はここで導出する。
    """
    # 姿勢(yaw 系は ±180 に折り返し)
    roll = _deg(log, "tlm_roll_rad")
    pitch = _deg(log, "tlm_pitch_rad")
    yaw_mad = np.asarray(wrap_deg(_deg(log, "tlm_yaw_rad")))
    yaw_ekf = np.asarray(wrap_deg(_deg(log, "tlm_yaw_est_rad")))
    roll_ref = _deg(log, "tlm_roll_ref_rad")
    pitch_ref = _deg(log, "tlm_pitch_ref_rad")
    yaw_ref = np.asarray(wrap_deg(_deg(log, "tlm_yaw_ref_rad")))

    # 角速度と指令
    p = _deg(log, "tlm_p_rad_s")
    q = _deg(log, "tlm_q_rad_s")
    r = _deg(log, "tlm_r_rad_s")
    p_ref = _deg(log, "tlm_roll_rate_ref_rad_s")
    q_ref = _deg(log, "tlm_pitch_rate_ref_rad_s")
    r_ref = _deg(log, "tlm_yaw_rate_ref_rad_s")

    # FF 補正 ΔB̂(ノルムは導出)
    ff_x = _column(log, "tlm_db_hat_x_ut")
    ff_y = _column(log, "tlm_db_hat_y_ut")

    series: dict[str, np.ndarray] = {}
    series["t"] = log.t
    series["alt_est"] = _column(log, "tlm_altitude_est_m")
    series["alt_tof"] = _column(log, "tlm_altitude_tof_m")
    series["alt_ref"] = _column(log, "tlm_alt_ref_m")
    series["pos_x"] = _column(log, "pos_x")
    series["pos_y"] = _column(log, "pos_y")
    series["target_x"] = _column(log, "target_x")
    series["target_y"] = _column(log, "target_y")
    series["roll"] = roll
    series["pitch"] = pitch
    series["yaw_mad"] = yaw_mad
    series["yaw_ekf"] = yaw_ekf
    series["roll_ref"] = roll_ref
    series["pitch_ref"] = pitch_ref
    series["yaw_ref"] = yaw_ref
    series["err_roll"] = roll_ref - roll
    series["err_pitch"] = pitch_ref - pitch
    series["err_yaw"] = np.asarray(wrap_deg(yaw_ref - yaw_ekf))
    series["p"] = p
    series["q"] = q
    series["r"] = r
    series["p_ref"] = p_ref
    series["q_ref"] = q_ref
    series["r_ref"] = r_ref
    series["err_p"] = p_ref - p
    series["err_q"] = q_ref - q
    series["err_r"] = r_ref - r
    # PID 成分(角度ループは rad/s → deg/s、角速度ループは生値)
    for axis in ("roll", "pitch", "yaw"):
        for comp in ("p", "i", "d"):
            series[f"pid_ang_{axis}_{comp}"] = _deg(
                log, f"tlm_pid_{axis}_ang_{comp}")
    for axis in ("roll", "pitch", "yaw"):
        for comp in ("p", "i", "d"):
            series[f"pid_rate_{axis}_{comp}"] = _column(
                log, f"tlm_pid_{axis}_rate_{comp}")
    series["duty_fr"] = _column(log, "tlm_duty_fr")
    series["duty_fl"] = _column(log, "tlm_duty_fl")
    series["duty_rr"] = _column(log, "tlm_duty_rr")
    series["duty_rl"] = _column(log, "tlm_duty_rl")
    series["current_a"] = _column(log, "tlm_current_a")
    series["ff_x"] = ff_x
    series["ff_y"] = ff_y
    series["ff_norm"] = np.hypot(ff_x, ff_y)
    series["bm_x"] = _column(log, "tlm_bm_x_ut")
    series["bm_y"] = _column(log, "tlm_bm_y_ut")
    series["alt_vel"] = _column(log, "tlm_alt_velocity_m_s")
    series["z_dot_ref"] = _column(log, "tlm_z_dot_ref_m_s")
    series["ctrl_flags"] = _column(log, "tlm_ctrl_flags")
    # MoCap 正解ヨー(v5: deg・±180 で記録済み。折り返しは記録側の契約)
    series["yaw_mocap"] = _column(log, "mocap_yaw_true_deg")

    assert tuple(series.keys()) == SERIES_NAMES, "系列の格納順が契約と不一致"
    return series


# ---------------------------------------------------------------------------
# EMBED 構築・テンプレート注入
# ---------------------------------------------------------------------------

def encode_series_b64(series: dict[str, np.ndarray]) -> str:
    """59 系列を格納順に連結した little-endian float32 を base64 化する。"""
    packed = np.concatenate(
        [np.ascontiguousarray(values, dtype="<f4")
         for values in series.values()])
    return base64.b64encode(packed.tobytes()).decode("ascii")


def build_embed(log: FlightLog, series: dict[str, np.ndarray]) -> dict:
    """EMBED スキーマ(version は EMBED_VERSION)の dict を構築する。

    datasets は将来の複数ログ比較を見越した配列(初版は要素 1 のみ)。
    """
    return {
        "version": EMBED_VERSION,
        "log_name": log.name,
        "generated_at": datetime.now().astimezone().isoformat(
            timespec="seconds"),
        "rate_hz": LOG_RATE_HZ,
        "datasets": [
            {
                "label": "A",
                "n": len(log.df),
                "duration_s": round(log.duration_s, 3),
                "mode": log.mode,
                "data_b64": encode_series_b64(series),
            }
        ],
    }


def render_html(embed: dict) -> str:
    """テンプレートのプレースホルダを EMBED JSON リテラルに置換して返す。"""
    if not TEMPLATE_PATH.is_file():
        raise FileNotFoundError(
            f"テンプレートが見つかりません: {TEMPLATE_PATH}"
            "(プレーヤー本体 viewer/interactive_template.html が必要です)")
    template = TEMPLATE_PATH.read_text(encoding="utf-8")
    count = template.count(EMBED_PLACEHOLDER)
    if count != 1:
        raise ValueError(
            f"テンプレートの注入プレースホルダ行 {EMBED_PLACEHOLDER!r} が"
            f" {count} 箇所あります(契約はちょうど1箇所): {TEMPLATE_PATH}")
    payload = json.dumps(embed, ensure_ascii=False, separators=(",", ":"))
    # <script> 内に埋め込むため "</" をエスケープ("<\/" は JSON/JS 両方で
    # "</" と等価。log_name 等に "</script>" が紛れても HTML が壊れない)
    payload = payload.replace("</", "<\\/")
    return template.replace(EMBED_PLACEHOLDER, f"const EMBED = {payload};", 1)


# ---------------------------------------------------------------------------
# 自己検証
# ---------------------------------------------------------------------------

def verify_html(html_path: Path, series: dict[str, np.ndarray]) -> list[str]:
    """書き出した HTML から EMBED を抽出・デコードし、元系列と照合する。

    系列数・長さ・NaN 位置・値(float32)の完全一致を確認する。
    照合に失敗したら ValueError。成功時は標準出力向けの要約行を返す。
    """
    text = html_path.read_text(encoding="utf-8")
    match = _EMBED_RE.search(text)
    if match is None:
        raise ValueError(f"生成 HTML から EMBED を抽出できません: {html_path}")
    embed = json.loads(match.group(1))
    dataset = embed["datasets"][0]
    n = int(dataset["n"])
    raw = base64.b64decode(dataset["data_b64"], validate=True)
    values = np.frombuffer(raw, dtype="<f4")
    if values.size != N_SERIES * n:
        raise ValueError(
            f"埋め込み長が契約と不一致: {values.size} float32 "
            f"!= {N_SERIES}系列 × {n}点")
    decoded = values.reshape(N_SERIES, n)

    nan_total = 0
    all_nan: list[str] = []
    for i, (name, source) in enumerate(series.items()):
        expected = np.ascontiguousarray(source, dtype="<f4")
        if len(expected) != n:
            raise ValueError(
                f"系列 {i} ({name}) の長さが不一致: {len(expected)} != {n}")
        if not np.array_equal(decoded[i], expected, equal_nan=True):
            raise ValueError(
                f"系列 {i} ({name}) のデコード値が元系列と一致しません")
        n_nan = int(np.isnan(decoded[i]).sum())
        nan_total += n_nan
        if n_nan == n:
            all_nan.append(name)

    nan_rate = 100.0 * nan_total / (N_SERIES * n)
    nan_note = f"NaN率 全体 {nan_rate:.2f}%"
    if all_nan:
        nan_note += f"(全NaN系列 {len(all_nan)}本: {', '.join(all_nan)})"
    return [
        f"自己検証: OK({N_SERIES}系列 × {n}点、デコード値は元系列と一致)",
        f"  {nan_note}",
    ]


# ---------------------------------------------------------------------------
# エクスポート本体
# ---------------------------------------------------------------------------

def export_interactive_html(
    log: FlightLog,
    output_root: Path | str = DEFAULT_OUTPUT_DIR,
    *,
    open_browser: bool = False,
) -> Path:
    """FlightLog からインタラクティブ HTML を生成してパスを返す。

    出力先は <output_root>/<ログ名>/interactive.html。
    multi ログは初版では拒否する。
    """
    if log.mode == "multi" or log.drone_name is not None:
        raise ValueError(
            "multi ログは初版では非対応です(単機 Posture / Position のみ)。")

    series = build_series(log)
    embed = build_embed(log, series)
    html = render_html(embed)

    out_dir = Path(output_root) / log.name
    out_dir.mkdir(parents=True, exist_ok=True)
    out_path = out_dir / "interactive.html"
    out_path.write_text(html, encoding="utf-8")

    n = len(log.df)
    b64_chars = len(embed["datasets"][0]["data_b64"])
    print(f"生成完了: {out_path}")
    print(f"  行数: {n} 行 / 時間長: {log.duration_s:.2f} s"
          f"({LOG_RATE_HZ:.0f}Hz, mode={log.mode})")
    print(f"  埋め込み: {N_SERIES}系列 × {n}点 = {N_SERIES * n:,} float32"
          f"(base64 {b64_chars:,} 文字 / HTML {out_path.stat().st_size:,} bytes)")
    for line in verify_html(out_path, series):
        print(line)

    if open_browser:
        webbrowser.open(out_path.resolve().as_uri())
    return out_path


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="フライトログからインタラクティブ HTML プレーヤー"
                    "(自己完結 1 ファイル)を生成する。単機 Posture / "
                    "Position のみ(multi は初版非対応)")
    parser.add_argument("csv", type=Path, help="対象ログ CSV のパス")
    parser.add_argument("--output", type=Path, default=DEFAULT_OUTPUT_DIR,
                        help=f"出力ルート(既定: {DEFAULT_OUTPUT_DIR}。"
                             "実体は <出力ルート>/<ログ名>/interactive.html)")
    parser.add_argument("--open", dest="open_browser", action="store_true",
                        help="生成後に既定ブラウザで開く")
    return parser.parse_args()


def main() -> int:
    args = _parse_args()
    try:
        log = load_log(args.csv)
        print(f"読み込み完了: {log.path}({len(log.df)}行, モード={log.mode})")
        export_interactive_html(log, args.output,
                                open_browser=args.open_browser)
        return 0
    except KeyboardInterrupt:
        print("\n中止しました。")
        return 130
    except (OSError, ValueError, RuntimeError) as e:
        print(f"エラー: {e}")
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
