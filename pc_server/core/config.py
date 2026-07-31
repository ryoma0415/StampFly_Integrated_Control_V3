"""設定ファイル(config/*.json)のローダ。

ARCHITECTURE.md「マジックナンバー禁止: PCは config/*.json」に従い、
数値定数はすべて server.json / control.json / airframes.json から供給する。
"""

from __future__ import annotations

import json
import os
import re
from pathlib import Path
from typing import Any

from . import PC_SERVER_DIR, REPO_DIR

CONFIG_DIR = PC_SERVER_DIR / "config"
# 飛行ログ CSV の出力先(logs/ 直下は動画等の他アセットと共有するため
# flight_logs/ サブディレクトリに分離する)
LOGS_DIR = REPO_DIR / "logs" / "flight_logs"

SERVER_CONFIG_PATH = CONFIG_DIR / "server.json"
CONTROL_CONFIG_PATH = CONFIG_DIR / "control.json"
AIRFRAMES_CONFIG_PATH = CONFIG_DIR / "airframes.json"

# v2: 生成データは pc_server/data/ 配下に集約(契約 §6)
DATA_DIR = PC_SERVER_DIR / "data"
SWEEP_RESULTS_DIR = DATA_DIR / "sweep_results"
# 実験モードの計測ログ(EKF/FF 性能評価。ExpRecorder が使用)
EXP_LOGS_DIR = DATA_DIR / "exp_logs"
FF_PROFILES_DIR = DATA_DIR / "ff_profiles"
CALPROFILES_DIR = DATA_DIR / "calibration_profiles"
FF_STATE_PATH = DATA_DIR / "ff_state.json"
# magbias(学習ハードアイアン残差)プロファイル/適用状態(契約 §3.4)
MAGBIAS_PROFILES_DIR = DATA_DIR / "magbias_profiles"
MAGBIAS_STATE_PATH = DATA_DIR / "magbias_state.json"
# フロー較正(純回転 2×2 フィット)の適用状態(FLIGHT_ANALYSIS_20260731.md §5)
FLOWCAL_STATE_PATH = DATA_DIR / "flowcal_state.json"
MAG3D_CALIBRATION_PATH = CONFIG_DIR / "mag3d_calibration.json"
GEOMAG_PROFILES_PATH = CONFIG_DIR / "geomagnetic_profiles.json"
DATA_ANALYSIS_DIR = REPO_DIR / "data_analysis"


def _load_json(path: Path) -> dict[str, Any]:
    with open(path, "r", encoding="utf-8") as fp:
        return json.load(fp)


def load_server_config() -> dict[str, Any]:
    return _load_json(SERVER_CONFIG_PATH)


def load_control_config() -> dict[str, Any]:
    return _load_json(CONTROL_CONFIG_PATH)


def load_airframes() -> list[dict[str, Any]]:
    """機体プロファイル配列を返す(airframes.json の "airframes" キー)。"""
    return _load_json(AIRFRAMES_CONFIG_PATH)["airframes"]


def _save_json_atomic(path: Path, payload: Any) -> None:
    """JSON 設定を原子的に書き込む(共通実装)。

    同一ディレクトリの一時ファイルへ全文を書いてから os.replace で置換する
    (書き込み途中のクラッシュで設定ファイルが壊れることを防ぐ)。
    キー順は呼び出し側の dict 挿入順をそのまま保持する。
    """
    tmp_path = path.with_name(path.name + ".tmp")
    text = json.dumps(payload, ensure_ascii=False, indent=2) + "\n"
    with open(tmp_path, "w", encoding="utf-8") as fp:
        fp.write(text)
        fp.flush()
        os.fsync(fp.fileno())
    os.replace(tmp_path, path)


def save_airframes(airframes: list[dict[str, Any]]) -> None:
    """機体プロファイル配列を airframes.json へ原子的に書き込む。"""
    _save_json_atomic(AIRFRAMES_CONFIG_PATH, {"airframes": airframes})


def save_control_config(control_config: dict[str, Any]) -> None:
    """control.json 全体を原子的に書き込む(MoCap マッピングの実行時変更用)。

    呼び出し側(session.update_mocap_mapping)がメモリ上の control_config
    と適用済みオブジェクト(MocapSource のマッピング)を同期させる責務を
    持つ — ファイル・メモリ・適用状態の三者が食い違わないよう、書き込みは
    必ず session 層のコマンド直列化(_command_lock)の内側で行うこと。
    """
    _save_json_atomic(CONTROL_CONFIG_PATH, control_config)


# MAC 未設定の正準表現は「空文字列」(airframes.json / UI / API で共通)
MAC_UNSET = ""


def mac_is_set(mac_text: Any) -> bool:
    """プロファイルの MAC が設定済みか(空文字列・空白のみ = 未設定)。"""
    return bool(str(mac_text or "").strip())


# MAC の受理形式: 2桁16進オクテット6個を ":" または "-" で区切った文字列のみ。
# オクテットごとの int(p, 16) は "0x1"・" 1"・1桁などの非正準表記も受理して
# しまうため、事前に正規表現で形式を固定する。
_MAC_PATTERN = re.compile(r"[0-9A-Fa-f]{2}([:-][0-9A-Fa-f]{2}){5}")


def parse_mac(mac_text: str) -> bytes:
    """"48:CA:43:38:9C:88" 形式の MAC 文字列を 6 バイトに変換する。

    2桁16進オクテット x6(区切りは ":" / "-"、大文字小文字不問)のみ受理し、
    "0x" 接頭辞・空白混入・桁不足などの非正準表記は ValueError とする。
    """
    if not isinstance(mac_text, str) or _MAC_PATTERN.fullmatch(mac_text) is None:
        raise ValueError(f"invalid MAC address: {mac_text!r}")
    return bytes(int(p, 16) for p in mac_text.replace("-", ":").split(":"))


def format_mac(mac: bytes) -> str:
    return ":".join(f"{b:02X}" for b in mac)
