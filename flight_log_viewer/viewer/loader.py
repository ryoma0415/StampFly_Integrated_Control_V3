"""V2 フライトログ CSV(50Hz・v4 109列〜v6 136列)の読み込みと派生量の計算。

- 破損した末尾(電源断などで途切れた行)は旧 Drone_Log_Viewer と同様に
  切り捨てて読む。
- 列の検証は「警告して続行」とする(pc_server と並行開発のため、列の過不足で
  即エラーにはしない。必須列 elapsed_time が無い場合のみエラー)。
- 角度系の派生列(deg 変換・アンラップ・対真値誤差)をここで一括計算する。
  v4 で CSV から廃止された roll_ref_deg / pitch_ref_deg / cmd_yaw_ref_deg は
  「CSV に列があればそれを使用、無ければ *_rad から派生生成」とし、
  旧ログ v1〜v3 との互換を保つ。
- v6 追加列(EKF2/磁気観測/フロー。docs/MAG_AUTOTUNE_DESIGN.md §1.1・§3)は
  列名ベース読み込みのため v5 以前のログでも従来どおり読める — 列が無い
  ログでは EKF2 系統・新パネルが自動的にスキップされる。
"""

from __future__ import annotations

import io
import math
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import pandas as pd

from .constants import (
    LOG_RATE_HZ,
    N_COLUMNS,
    TEXT_COLUMNS,
    TLM_FLAG_FLYING,
    V4_COLUMNS,
    V6_COLUMNS,
    YAW_SOURCES,
)

# ヨー比較の対象(推定4系統。ekf2 は v6 ログのみ列が存在し、無いログでは
# 自動スキップ)。基準(真値)は mocap があれば mocap、なければ Madgwick
# (その場合 madgwick 自身は比較から除外)。
YAW_ESTIMATOR_KEYS: tuple[str, ...] = ("madgwick", "ekf", "ekf2", "gyro_int")

# 飛行ログの既定保存先: <repo>/logs/flight_logs/
# (viewer/ → flight_log_viewer/ → <repo> の 2 階層上)
DEFAULT_FLIGHT_LOGS_DIR: Path = (
    Path(__file__).resolve().parents[2] / "logs" / "flight_logs"
)


# ---------------------------------------------------------------------------
# 角度ユーティリティ
# ---------------------------------------------------------------------------

def wrap_pi(angle_rad: np.ndarray | float) -> np.ndarray | float:
    """角度を ±π に折り返す。"""
    return (np.asarray(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def wrap_deg(angle_deg: np.ndarray | float) -> np.ndarray | float:
    """角度を ±180° に折り返す。"""
    return (np.asarray(angle_deg) + 180.0) % 360.0 - 180.0


def wrapped_plot_series(t: np.ndarray, deg: np.ndarray,
                        jump_deg: float = 180.0) -> tuple[np.ndarray, np.ndarray]:
    """±180° ラップ表示用の (t, y) を返す(data_analysis/plot_explog.py と同実装)。

    wrap_deg した系列の隣接差が jump_deg を超える箇所(ラップ跨ぎ)に
    NaN 点を挿入し、プロット時に縦線が入らないよう線を切る。
    表示専用: 統計計算(ドリフト率など)には使わないこと。
    """
    t = np.asarray(t, dtype=float)
    w = np.asarray(wrap_deg(np.asarray(deg, dtype=float)))
    if len(w) < 2:
        return t, w
    d = np.abs(np.diff(w))
    jumps = np.where(np.isfinite(d) & (d > jump_deg))[0]
    if jumps.size == 0:
        return t, w
    t2 = np.insert(t, jumps + 1, (t[jumps] + t[jumps + 1]) / 2.0)
    y2 = np.insert(w, jumps + 1, np.nan)
    return t2, y2


def unwrap_deg(values: np.ndarray) -> np.ndarray:
    """NaN を保持したままアンラップする(有限区間のみ np.unwrap)。"""
    values = np.asarray(values, dtype=float)
    result = np.full_like(values, np.nan)
    finite = np.isfinite(values)
    if finite.sum() >= 2:
        result[finite] = np.degrees(np.unwrap(np.radians(values[finite])))
    elif finite.any():
        result[finite] = values[finite]
    return result


# ---------------------------------------------------------------------------
# FlightLog データクラス
# ---------------------------------------------------------------------------

@dataclass
class FlightLog:
    """読み込んだ 1 本のフライトログ。df には派生列も追加済み。"""

    path: Path
    df: pd.DataFrame
    mode: str                       # "posture" / "position" / "multi" / "unknown"
    warnings: list[str] = field(default_factory=list)
    drone_name: str | None = None   # multi ログの機体名(<ts>_multi_<name>.csv)

    @property
    def name(self) -> str:
        return self.path.stem

    @property
    def t(self) -> np.ndarray:
        """経過時間 [s]。"""
        return self.df["elapsed_time"].to_numpy(dtype=float)

    @property
    def duration_s(self) -> float:
        t = self.t
        return float(t[-1] - t[0]) if len(t) >= 2 else 0.0

    def has(self, column: str) -> bool:
        """列が存在し、かつ有効値を1つ以上含むか。"""
        return column in self.df.columns and bool(self.df[column].notna().any())

    def flying_mask(self) -> np.ndarray:
        """飛行中の行マスク(tlm_flags bit2 を優先、無ければ phase=='flying')。"""
        if self.has("tlm_flags"):
            flags = self.df["tlm_flags"].to_numpy(dtype=float)
            mask = np.zeros(len(flags), dtype=bool)
            finite = np.isfinite(flags)
            mask[finite] = (flags[finite].astype(int) & TLM_FLAG_FLYING) != 0
            if mask.any():
                return mask
        if "phase" in self.df.columns:
            return (self.df["phase"].astype(str) == "flying").to_numpy()
        return np.zeros(len(self.df), dtype=bool)

    def flight_time_s(self) -> float:
        """飛行時間 [s](flying 行数 / 50Hz)。"""
        return float(self.flying_mask().sum()) / LOG_RATE_HZ

    def yaw_reference(self) -> tuple[str, np.ndarray] | None:
        """ヨー誤差の基準系統(キー名, アンラップ済み deg 系列)を返す。

        MoCap 正解Yaw(v5: mocap_yaw_true_deg)があればそれを、なければ
        Madgwick を基準とする。どちらも無ければ None。旧列 mocap_yaw_deg は
        機首方位でない(Y-up へ Z-up 前提オイラーを適用した値)ため基準に
        使わない — v4 以前のログは Madgwick 基準になる。
        """
        if self.has("mocap_yaw_true_deg"):
            return "mocap", self.df["yaw_mocap_unwrap_deg"].to_numpy(dtype=float)
        if self.has("tlm_yaw_rad"):
            return "madgwick", self.df["yaw_madgwick_unwrap_deg"].to_numpy(dtype=float)
        return None


# ---------------------------------------------------------------------------
# CSV 読み込み
# ---------------------------------------------------------------------------

def _read_csv_with_recovery(csv_path: Path) -> pd.DataFrame:
    """UTF-8 として壊れたバイトを含む行を除いて読む(電源断ログの救済)。

    注意: pandas の C パーサは 256KiB チャンク単位でデコードするため、
    UnicodeDecodeError.start は**チャンク内**オフセットでありファイル先頭
    からのオフセットではない(旧 Drone_Log_Viewer はこれを切断位置に誤用し、
    破損位置が 256KiB を超えるログで有効行の大半を無警告で失っていた)。
    ここでは e.start を使わず、ファイル全体を errors="ignore" でデコードして
    最後の改行までを読む。不正バイトの除去で列が崩れた行(通常は破損点の
    1 行のみ)は on_bad_lines="skip" で読み飛ばす。
    """
    try:
        return pd.read_csv(csv_path)
    except UnicodeDecodeError as e:
        print(f"警告: CSV 読み込みでエンコードエラー ({e})。"
              "破損バイトを除去して再読み込みします。")
        raw_bytes = Path(csv_path).read_bytes()
        clean_text = raw_bytes.decode("utf-8", errors="ignore")
        if "\n" not in clean_text:
            raise ValueError("CSV の先頭から破損しているため読み込めません。") from e
        clean_text = clean_text.rsplit("\n", 1)[0] + "\n"
        return pd.read_csv(io.StringIO(clean_text), on_bad_lines="skip")


def _add_derived_columns(df: pd.DataFrame) -> pd.DataFrame:
    """deg 変換・ヨーのアンラップ・対基準誤差などの派生列を追加して返す。

    列を 1 本ずつ追加すると pandas がフラグメント化警告を出すため、
    まとめて concat する。
    """
    derived: dict[str, np.ndarray] = {}

    # 姿勢 rad → deg
    for rad_col, deg_col in (
        ("tlm_roll_rad", "tlm_roll_deg"),
        ("tlm_pitch_rad", "tlm_pitch_deg"),
        ("tlm_yaw_ref_rad", "tlm_yaw_ref_deg"),
        ("tlm_roll_ref_rad", "tlm_roll_ref_deg"),
        ("tlm_pitch_ref_rad", "tlm_pitch_ref_deg"),
        ("traj_phase_rad", "traj_phase_deg"),
    ):
        if rad_col in df.columns:
            derived[deg_col] = np.degrees(df[rad_col].to_numpy(dtype=float))

    # 送信指令の deg 列(v4 で CSV から廃止。旧ログ v1〜v3 に列があれば
    # そのまま使い、無ければ rad 列から派生生成する)
    for rad_col, deg_col in (
        ("roll_ref_rad", "roll_ref_deg"),
        ("pitch_ref_rad", "pitch_ref_deg"),
        ("cmd_yaw_ref_rad", "cmd_yaw_ref_deg"),
    ):
        if deg_col not in df.columns and rad_col in df.columns:
            derived[deg_col] = np.degrees(df[rad_col].to_numpy(dtype=float))

    # ヨー4系統: deg 列とアンラップ列を作る
    yaw_deg: dict[str, np.ndarray] = {}
    for key, column, _label, _color, is_deg in YAW_SOURCES:
        if column not in df.columns:
            continue
        values = df[column].to_numpy(dtype=float)
        deg = values if is_deg else np.degrees(values)
        yaw_deg[key] = deg
        derived[f"yaw_{key}_deg"] = deg
        derived[f"yaw_{key}_unwrap_deg"] = unwrap_deg(deg)

    # 対基準ヨー誤差(基準 = mocap があれば mocap、なければ madgwick)
    if "mocap" in yaw_deg and np.isfinite(yaw_deg["mocap"]).any():
        ref_key = "mocap"
    elif "madgwick" in yaw_deg and np.isfinite(yaw_deg["madgwick"]).any():
        ref_key = "madgwick"
    else:
        ref_key = None
    if ref_key is not None:
        ref = yaw_deg[ref_key]
        for key in YAW_ESTIMATOR_KEYS:
            if key == ref_key or key not in yaw_deg:
                continue
            derived[f"yaw_err_{key}_deg"] = wrap_deg(yaw_deg[key] - ref)

    # ヨー指令追従誤差(cmd_yaw_ref と アクティブ推定ヨーの差、ヨー制御 ON 行のみ)
    if "cmd_yaw_ref_deg" in df.columns:
        cmd = df["cmd_yaw_ref_deg"].to_numpy(dtype=float)
    else:
        cmd = derived.get("cmd_yaw_ref_deg")
    if cmd is not None and "ekf" in yaw_deg and "yaw_ctrl_on" in df.columns:
        on = df["yaw_ctrl_on"].to_numpy(dtype=float)
        err = np.asarray(wrap_deg(yaw_deg["ekf"] - cmd), dtype=float)
        err[~(np.isfinite(on) & (on > 0))] = np.nan
        derived["yaw_track_err_deg"] = err

    if not derived:
        return df
    return pd.concat([df, pd.DataFrame(derived, index=df.index)], axis=1)


def load_log(csv_path: str | Path) -> FlightLog:
    """CSV を読み込み、列を検証し、派生列を追加して FlightLog を返す。"""
    csv_path = Path(csv_path)
    if not csv_path.is_file():
        raise FileNotFoundError(f"ログファイルが見つかりません: {csv_path}")

    df = _read_csv_with_recovery(csv_path)
    warnings: list[str] = []

    if "elapsed_time" not in df.columns:
        raise ValueError(f"必須列 elapsed_time がありません: {csv_path}")

    # 列の検証(過不足は警告のみ。契約は docs/LOG_STRUCTURE.md を参照)。
    # 必須は v4 の 109 列。v5/v6 追加列(MoCap 正解ヨー・EKF2/磁気/フロー等)
    # は旧ログに無くて正常なため欠落警告の対象にしない — これらの列が有れば
    # 「契約に無い列」にもしない(V6_COLUMNS 基準で判定)。
    missing = [c for c in V4_COLUMNS if c not in df.columns]
    extra = [c for c in df.columns if c not in V6_COLUMNS]
    if missing:
        warnings.append(
            f"契約({N_COLUMNS}列)に対して欠けている列が {len(missing)} 個あります: "
            + ", ".join(missing[:8]) + ("…" if len(missing) > 8 else "")
        )
    if extra:
        warnings.append(
            f"契約に無い列が {len(extra)} 個あります: "
            + ", ".join(extra[:8]) + ("…" if len(extra) > 8 else "")
        )

    # 数値列を数値化(空欄 → NaN)。文字列列はそのまま。
    for col in df.columns:
        if col not in TEXT_COLUMNS:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # elapsed_time が欠損した行は使えないため除外
    df = df[df["elapsed_time"].notna()].reset_index(drop=True)
    if len(df) == 0:
        raise ValueError(f"有効な行がありません: {csv_path}")

    df = _add_derived_columns(df)

    # モード判定(mode 列 → ファイル名の順)。multi はファイル名
    # <ts>_multi_<機体名>.csv から機体名も取り出す。
    stem = csv_path.stem
    drone_name: str | None = None
    if "_multi_" in stem:
        drone_name = stem.split("_multi_", 1)[1] or None

    mode = "unknown"
    if "mode" in df.columns and df["mode"].notna().any():
        mode = str(df["mode"].dropna().iloc[0])
    elif drone_name is not None:
        mode = "multi"
    else:
        for candidate in ("posture", "position"):
            if stem.endswith(f"_{candidate}"):
                mode = candidate
                break

    for message in warnings:
        print(f"警告: {message}")

    return FlightLog(path=csv_path, df=df, mode=mode, warnings=warnings,
                     drone_name=drone_name)


def group_timestamp(csv_path: str | Path) -> str | None:
    """multi ログのグループキー(<ts>_multi_<name>.csv の <ts>)を返す。

    multi ログでなければ None。
    """
    stem = Path(csv_path).stem
    if "_multi_" not in stem:
        return None
    ts = stem.split("_multi_", 1)[0]
    return ts or None


def load_group(ts_or_path: str | Path,
               logs_dir: str | Path | None = None) -> list[FlightLog]:
    """同一タイムスタンプの multi ログ群をまとめて読み込む。

    Args:
        ts_or_path: グループのタイムスタンプ文字列(例 "20260710_123456")、
            またはグループに属する任意の 1 ファイルのパス。
        logs_dir: タイムスタンプ指定時の検索ディレクトリ
            (既定: <repo>/logs/flight_logs/)。パス指定時はそのファイルの
            ディレクトリを使い、この引数は無視する。

    Returns:
        機体名の昇順に並んだ FlightLog のリスト。

    Raises:
        FileNotFoundError: 該当する multi ログが 1 本も見つからない場合。
        ValueError: パス指定だが multi ログの命名(<ts>_multi_<name>.csv)
            でない場合。
    """
    candidate = Path(ts_or_path)
    if candidate.suffix.lower() == ".csv" or candidate.is_file():
        ts = group_timestamp(candidate)
        if ts is None:
            raise ValueError(
                f"multi ログの命名(<ts>_multi_<機体名>.csv)ではありません: "
                f"{candidate}"
            )
        directory = candidate.resolve().parent
    else:
        ts = str(ts_or_path)
        directory = Path(logs_dir) if logs_dir is not None \
            else DEFAULT_FLIGHT_LOGS_DIR

    paths = sorted(directory.glob(f"{ts}_multi_*.csv"))
    if not paths:
        raise FileNotFoundError(
            f"multi ログが見つかりません: {directory}/{ts}_multi_*.csv"
        )
    return [load_log(path) for path in paths]
