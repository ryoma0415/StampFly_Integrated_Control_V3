"""キャリブレーション機能(3D磁気 / 加速度6面 / マウント・ヨーゼロ /
地磁気47都道府県 / キャリブレーション・プロファイル)。

旧 yaw側プロジェクト(Yaw_Calibration_and_Estimation。2026-07-27 に削除済み)の
pc_server/server.py の fit_ellipsoid・calprofile 系・geomag 系と、
firmware/src/accel_calibration.cpp の 6面ソルバを
シリアル版(CMD_* + TLM_ACK / CMD_CAL_GET → TLM_CAL_DATA)に移植:
- 保存プロファイルは `stampfly_calibration_profile` v1 スキーマを維持
  (yaw側と同一。保存先は pc_server/data/calibration_profiles/)。
- 適用は accel6 → attmount → mag3d → yawzero の順(accel6 は姿勢参照リセット、
  mag3d はヨー再シードを伴うため順序固定)で送信後、CAL_GET 読み戻しで
  全値照合する(許容 2e-3)。
- 3D磁気の PC 側スナップショットは pc_server/config/mag3d_calibration.json
  (スイープ CSV の b*_cor 計算に使用)。
"""

from __future__ import annotations

import json
import math
import re
import struct
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from . import config as cfg
from .posture import wrap_pi
from .serial_link import SerialLink, SerialLinkError

MAG3D_CALIBRATION_SCHEMA = 2
MAG3D_INPUT_UNITS = "bmm150_trim_compensated_body_uT"
MAG3D_MIN_SAMPLES = 80          # 楕円体フィットに必要な最小サンプル(yaw側と同値)

CALPROFILE_SCHEMA = 1
CAL_VERIFY_TOLERANCE = 2e-3     # float 往復の照合許容(yaw側と同値)

# TLM_ACK status → 表示名(PROTOCOL.md v2 の定義順)
ACK_STATUS_NAMES = {
    proto.TlmAck.STATUS_OK: "ok",
    proto.TlmAck.STATUS_BAD_STATE: "bad_state",
    proto.TlmAck.STATUS_INVALID_ARG: "invalid_arg",
    proto.TlmAck.STATUS_CRC_MISMATCH: "crc_mismatch",
    proto.TlmAck.STATUS_BUSY: "busy",
    proto.TlmAck.STATUS_INCOMPLETE: "incomplete",
}

# 加速度6面キャリブレーション(firmware accel_calibration.cpp から移植。
# 数値・判定は同一 — 面の妥当性ゲートと offset/scale ソルバ)
ACCEL6_FACES = ("x_pos", "x_neg", "y_pos", "y_neg", "z_pos", "z_neg")
ACCEL6_NORM_MIN_G = 0.75
ACCEL6_NORM_MAX_G = 1.25
ACCEL6_AXIS_MIN_G = 0.60
ACCEL6_OTHER_AXES_MAX_G = 0.55
ACCEL6_SCALE_DENOM_MIN = 0.8
ACCEL6_SCALE_DENOM_MAX = 3.2
ACCEL6_SCALE_ABS_MIN = 0.1
ACCEL6_SCALE_ABS_MAX = 10.0

CAL_DATA_POLL_S = 0.05          # TLM_CAL_DATA 待ちのポーリング周期

# ワンクリック・ヨーゼロ自動シーケンス(FF一時off → 設定/クリア → FF復元
# → アンカー再取得)のタイミング定数。テストから monkeypatch で短縮できる
# よう、メソッド内では毎回モジュールグローバルを参照する。
YAWZERO_MODE_TIMEOUT_S = 1.5    # FF off 反映(ff_status 下位2bit==0)待ち上限
YAWZERO_ALIGN_TIMEOUT_S = 1.0   # CF 整列(|yaw_est| < 許容)待ち上限(超過は警告のみ)
YAWZERO_ALIGN_TOL_RAD = math.radians(8.0)
YAWZERO_POLL_S = 0.02           # TLM_STATE ポーリング周期
YAWZERO_ANCHOR_RETRIES = 10     # CMD_FF_ANCHOR の busy リトライ回数(≈3s)
YAWZERO_ANCHOR_RETRY_S = 0.3    # リトライ間隔


class _YawZeroAbort(Exception):
    """ヨーゼロシーケンスの途中失敗(理由メッセージを運ぶ内部例外)。"""


def ack_ok(ack: Optional[proto.TlmAck]) -> bool:
    return ack is not None and ack.status == proto.TlmAck.STATUS_OK

def ack_detail(ack: Optional[proto.TlmAck]) -> str:
    if ack is None:
        return "ACKタイムアウト"
    return ACK_STATUS_NAMES.get(ack.status, f"status={ack.status}")


def sanitize_profile_name(value: Any) -> str:
    """ファイルシステム安全なプロファイル名(パス区切り・制御文字を除去)。"""
    name = str(value or "").strip()
    name = re.sub(r'[\\/:*?"<>|\s\x00-\x1f]+', "_", name).strip("._")
    return name[:40]


def mtime_or_zero(path: Path) -> float:
    """glob() と stat() の間に削除されたファイルに耐えるソートキー。"""
    try:
        return path.stat().st_mtime
    except OSError:
        return 0.0


# ----------------------------------------------------------------------
# 3D磁気キャリブレーション(楕円体フィット、yaw側 fit_ellipsoid 移植)
# ----------------------------------------------------------------------

def fit_ellipsoid(samples: list[list[float]]) -> dict[str, Any]:
    """生磁気サンプル群から楕円体フィット(ハード/ソフトアイアン補正)を得る。"""
    try:
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("3Dキャリブレーションには numpy が必要です。"
                           "`pip install -r requirements.txt` を実行してください"
                           ) from exc

    if len(samples) < MAG3D_MIN_SAMPLES:
        raise ValueError(f"フィットには {MAG3D_MIN_SAMPLES} サンプル以上が必要です")

    points = np.asarray(samples, dtype=float)
    if points.ndim != 2 or points.shape[1] != 3 or not np.all(np.isfinite(points)):
        raise ValueError("3Dキャリブレーションのサンプルは有限の x/y/z 値が必要です")

    origin = np.mean(points, axis=0)
    centered = points - origin
    coordinate_scale = float(np.max(np.std(centered, axis=0)))
    if coordinate_scale <= 1.0e-6:
        raise ValueError("サンプルの動きが不足しています(機体を全方位に回してください)")

    normalized = centered / coordinate_scale
    x = normalized[:, 0]
    y = normalized[:, 1]
    z = normalized[:, 2]
    design = np.column_stack([
        x * x, y * y, z * z,
        2.0 * x * y, 2.0 * x * z, 2.0 * y * z,
        2.0 * x, 2.0 * y, 2.0 * z,
        np.ones(len(points)),
    ])

    _, _, vh = np.linalg.svd(design, full_matrices=False)
    coeff = vh[-1]
    quad = np.array([
        [coeff[0], coeff[3], coeff[4]],
        [coeff[3], coeff[1], coeff[5]],
        [coeff[4], coeff[5], coeff[2]],
    ], dtype=float)
    linear = np.array([coeff[6], coeff[7], coeff[8]], dtype=float)
    constant = float(coeff[9])

    try:
        center_normalized = -np.linalg.solve(quad, linear)
    except np.linalg.LinAlgError as exc:
        raise ValueError("楕円体フィットが特異です。より多様な3Dサンプルを"
                         "収集してください") from exc

    scale = float(center_normalized.T @ quad @ center_normalized - constant)
    if abs(scale) <= 1.0e-9:
        raise ValueError("楕円体フィットのスケールが不正です")

    shape = quad / scale
    eigenvalues, eigenvectors = np.linalg.eigh(shape)
    if np.any(eigenvalues <= 0.0):
        raise ValueError("楕円体フィットが正定値ではありません。より多様な"
                         "3Dサンプルを収集してください")

    center = origin + coordinate_scale * center_normalized
    axes = coordinate_scale / np.sqrt(eigenvalues)
    target_radius = float(np.mean(axes))
    correction = (
        eigenvectors
        @ np.diag(np.sqrt(eigenvalues) * target_radius / coordinate_scale)
        @ eigenvectors.T
    )
    corrected = (points - center) @ correction.T
    norms = np.linalg.norm(corrected, axis=1)
    rms = float(np.sqrt(np.mean((norms - target_radius) ** 2)))
    relative_rms = float(rms / target_radius) if target_radius > 0.0 else 0.0

    return {
        "schema": MAG3D_CALIBRATION_SCHEMA,
        "input_units": MAG3D_INPUT_UNITS,
        "sample_count": int(len(points)),
        "offset": [float(v) for v in center],
        "matrix": [[float(v) for v in row] for row in correction],
        "target_radius": target_radius,
        "rms_error": rms,
        "relative_rms_error": relative_rms,
        "axis_lengths": [float(v) for v in axes],
    }


def is_calibration_compatible(data: Any) -> bool:
    if not isinstance(data, dict):
        return False
    return (data.get("schema") == MAG3D_CALIBRATION_SCHEMA
            and data.get("input_units") == MAG3D_INPUT_UNITS)


def load_saved_calibration(
        path: Path = cfg.MAG3D_CALIBRATION_PATH) -> Optional[dict]:
    if not path.exists():
        return None
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"error": f"保存済みキャリブレーションを読めません: {exc}"}
    if not isinstance(data, dict):
        return {"error": "保存済みキャリブレーションの形式が不正です"}
    if not is_calibration_compatible(data):
        return {"error": "保存済みキャリブレーションが旧形式です。"
                         "クリアして再キャリブレーションしてください"}
    return data


def load_mag3d_correction(
        path: Path = cfg.MAG3D_CALIBRATION_PATH) -> Optional[dict]:
    """保存済み 3D 補正を offset + matrix として返す(なければ None)。"""
    cal = load_saved_calibration(path)
    if not cal or "error" in cal:
        return None
    offset = cal.get("offset")
    matrix = cal.get("matrix")
    if (not isinstance(offset, list) or len(offset) != 3
            or not isinstance(matrix, list) or len(matrix) != 3
            or any(not isinstance(row, list) or len(row) != 3 for row in matrix)):
        return None
    try:
        return {
            "offset": [float(v) for v in offset],
            "matrix": [[float(v) for v in row] for row in matrix],
        }
    except (TypeError, ValueError):
        return None


# ----------------------------------------------------------------------
# 地磁気プロファイル(47都道府県、config/geomagnetic_profiles.json)
# ----------------------------------------------------------------------

def load_geomagnetic_config(
        path: Path = cfg.GEOMAG_PROFILES_PATH) -> dict[str, Any]:
    """選択中の地磁気プロファイルを正規化して返す(yaw側移植)。"""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        return {"error": f"地磁気プロファイルを読めません: {exc}"}
    if not isinstance(data, dict):
        return {"error": "地磁気プロファイルファイルの形式が不正です"}
    selected = data.get("selected")
    profiles = data.get("profiles")
    if not isinstance(selected, str) or not isinstance(profiles, dict):
        return {"error": "地磁気プロファイルファイルには selected と profiles が必要です"}
    profile = profiles.get(selected)
    if not isinstance(profile, dict):
        return {"error": f"選択中の地磁気プロファイルが見つかりません: {selected}"}

    required = ["declination_east_deg", "inclination_deg",
                "horizontal_uT", "vertical_uT", "total_uT"]
    try:
        normalized = {
            "id": selected,
            "label": str(profile.get("label", selected)),
            "latitude_deg": float(profile.get("latitude_deg", 0.0)),
            "longitude_deg": float(profile.get("longitude_deg", 0.0)),
            "epoch": float(profile.get("epoch", 2020.0)),
            "declination_west_deg": float(profile.get(
                "declination_west_deg", -float(profile["declination_east_deg"]))),
            "declination_east_deg": float(profile["declination_east_deg"]),
            "inclination_deg": float(profile["inclination_deg"]),
            "horizontal_uT": float(profile["horizontal_uT"]),
            "vertical_uT": float(profile["vertical_uT"]),
            "total_uT": float(profile["total_uT"]),
        }
    except (KeyError, TypeError, ValueError) as exc:
        missing = ", ".join(key for key in required if key not in profile)
        detail = f" 欠落: {missing}" if missing else f" {exc}"
        return {"error": f"選択中の地磁気プロファイルの値が不正です:{detail}"}

    if normalized["horizontal_uT"] <= 0.0 or normalized["total_uT"] <= 0.0:
        return {"error": "地磁気プロファイルの H と F は正の値が必要です"}
    return {
        "selected": selected,
        "source": data.get("source", ""),
        "profile": normalized,
    }


def geomag_profile_list(path: Path = cfg.GEOMAG_PROFILES_PATH) -> list[dict]:
    """全プロファイルの id/label(都道府県セレクタ用、ファイル順)。"""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return []
    profiles = data.get("profiles")
    if not isinstance(profiles, dict):
        return []
    return [{"id": pid, "label": str((profile or {}).get("label", pid))}
            for pid, profile in profiles.items() if isinstance(profile, dict)]


# ----------------------------------------------------------------------
# 加速度6面キャリブレーション(firmware AccelSixFaceCalibration 移植)
# ----------------------------------------------------------------------

class Accel6Calibrator:
    """6面キャプチャから offset/scale を解く(firmware と同一の判定・数式)。

    スレッド安全ではない。CalibrationManager が排他スロット内で使う。
    """

    def __init__(self) -> None:
        self.reset()

    def reset(self) -> None:
        self.face_mean: dict[str, tuple[float, float, float]] = {}
        self.last_error = ""

    @property
    def captured_faces(self) -> list[str]:
        return [f for f in ACCEL6_FACES if f in self.face_mean]

    @property
    def ready(self) -> bool:
        return len(self.face_mean) == len(ACCEL6_FACES)

    @staticmethod
    def _axis_value(face: str, vec: tuple[float, float, float]) -> float:
        axis = {"x": 0, "y": 1, "z": 2}[face[0]]
        return vec[axis]

    @staticmethod
    def _other_axes_within(face: str, vec: tuple[float, float, float],
                           limit: float) -> bool:
        axis = {"x": 0, "y": 1, "z": 2}[face[0]]
        return all(abs(vec[i]) <= limit for i in range(3) if i != axis)

    def capture_face(self, face: str,
                     mean_raw_body: tuple[float, float, float]) -> bool:
        if face not in ACCEL6_FACES:
            self.last_error = "不正な加速度キャリブレーション面です"
            return False
        if any(not math.isfinite(v) for v in mean_raw_body):
            self.last_error = "加速度サンプルが有限値ではありません"
            return False
        norm = math.sqrt(sum(v * v for v in mean_raw_body))
        if not (ACCEL6_NORM_MIN_G <= norm <= ACCEL6_NORM_MAX_G):
            self.last_error = "加速度ノルムが静止時の想定範囲外です"
            return False
        axis_value = self._axis_value(face, mean_raw_body)
        positive = face.endswith("_pos")
        if (positive and axis_value < ACCEL6_AXIS_MIN_G) \
                or (not positive and axis_value > -ACCEL6_AXIS_MIN_G):
            self.last_error = "選択した面と計測された加速度の向きが一致しません"
            return False
        if not self._other_axes_within(face, mean_raw_body,
                                       ACCEL6_OTHER_AXES_MAX_G):
            self.last_error = "機体が傾きすぎているためキャプチャできません"
            return False
        self.face_mean[face] = tuple(float(v) for v in mean_raw_body)
        self.last_error = ""
        return True

    def solve(self) -> Optional[tuple[tuple[float, float, float],
                                      tuple[float, float, float]]]:
        """全6面から (offset, scale) を解く。失敗時は None(last_error 参照)。"""
        if not self.ready:
            self.last_error = "先に6面すべてをキャプチャしてください"
            return None
        xp, xn = self.face_mean["x_pos"], self.face_mean["x_neg"]
        yp, yn = self.face_mean["y_pos"], self.face_mean["y_neg"]
        zp, zn = self.face_mean["z_pos"], self.face_mean["z_neg"]
        dx = xp[0] - xn[0]
        dy = yp[1] - yn[1]
        dz = zp[2] - zn[2]
        for d in (dx, dy, dz):
            if not (math.isfinite(d)
                    and ACCEL6_SCALE_DENOM_MIN <= d <= ACCEL6_SCALE_DENOM_MAX):
                self.last_error = "キャプチャした面同士が矛盾しています"
                return None
        offset = ((xp[0] + xn[0]) * 0.5, (yp[1] + yn[1]) * 0.5,
                  (zp[2] + zn[2]) * 0.5)
        scale = (2.0 / dx, 2.0 / dy, 2.0 / dz)
        for s in scale:
            if not (math.isfinite(s)
                    and ACCEL6_SCALE_ABS_MIN <= abs(s) <= ACCEL6_SCALE_ABS_MAX):
                self.last_error = "スケールが安全範囲外です"
                return None
        self.last_error = ""
        return offset, scale


# ----------------------------------------------------------------------
# TLM_CAL_DATA ⇔ プロファイル変換
# ----------------------------------------------------------------------

def cal_data_to_profile(name: str, cal: proto.TlmCalData) -> dict[str, Any]:
    """TLM_CAL_DATA を stampfly_calibration_profile v1 JSON に変換する。"""
    m = cal.mag3d_matrix
    return {
        "schema": "stampfly_calibration_profile",
        "version": CALPROFILE_SCHEMA,
        "name": name,
        "saved_at": time.time(),
        "mag3d": {
            "valid": bool(cal.valid_flags & proto.TlmCalData.VALID_MAG3D),
            "offset": [float(v) for v in cal.mag3d_offset],
            "matrix": [[float(m[0]), float(m[1]), float(m[2])],
                       [float(m[3]), float(m[4]), float(m[5])],
                       [float(m[6]), float(m[7]), float(m[8])]],
        },
        "accel6": {
            "valid": bool(cal.valid_flags & proto.TlmCalData.VALID_ACCEL6),
            "offset": [float(v) for v in cal.accel6_offset],
            "scale": [float(v) for v in cal.accel6_scale],
        },
        "attitude_mount": {
            "valid": bool(cal.valid_flags & proto.TlmCalData.VALID_ATTMOUNT),
            "roll_rad": float(cal.attmount_roll_rad),
            "pitch_rad": float(cal.attmount_pitch_rad),
        },
        "yaw_zero": {
            "valid": bool(cal.valid_flags & proto.TlmCalData.VALID_YAWZERO),
            "offset_rad": float(cal.yawzero_offset_rad),
        },
        # 地磁気は情報のみ(都道府県セレクタと機体 NVS が別途管理する)
        "geomag": {
            "valid": bool(cal.valid_flags & proto.TlmCalData.VALID_GEOMAG),
            "declination_east_deg": float(cal.geomag[0]),
            "inclination_deg": float(cal.geomag[1]),
            "horizontal_uT": float(cal.geomag[2]),
            "vertical_uT": float(cal.geomag[3]),
            "total_uT": float(cal.geomag[4]),
        },
    }


def profile_apply_commands(profile: dict[str, Any]
                           ) -> list[tuple[int, bytes]]:
    """プロファイル復元コマンド列(msg_type, payload)を順序どおり構築する。

    順序契約: accel6(姿勢参照リセットを伴う)→ attmount → mag3d
    (ヨー再シードを伴う)→ yawzero。valid=0 はクリアとして送る。
    """
    commands: list[tuple[int, bytes]] = []
    a6 = profile.get("accel6") or {}
    if a6.get("valid"):
        off = a6.get("offset") or [0, 0, 0]
        sc = a6.get("scale") or [1, 1, 1]
        payload = proto.CmdAccel6Set(valid=1, offset=tuple(float(v) for v in off),
                                     scale=tuple(float(v) for v in sc))
    else:
        payload = proto.CmdAccel6Set(valid=0, offset=(0.0, 0.0, 0.0),
                                     scale=(0.0, 0.0, 0.0))
    commands.append((proto.MsgType.CMD_ACCEL6_SET, payload.to_payload()))

    mount = profile.get("attitude_mount") or {}
    if mount.get("valid"):
        att = proto.CmdAttmountSet(valid=1,
                                   roll_rad=float(mount.get("roll_rad", 0.0)),
                                   pitch_rad=float(mount.get("pitch_rad", 0.0)))
    else:
        att = proto.CmdAttmountSet(valid=0, roll_rad=0.0, pitch_rad=0.0)
    commands.append((proto.MsgType.CMD_ATTMOUNT_SET, att.to_payload()))

    mag3d = profile.get("mag3d") or {}
    if mag3d.get("valid"):
        off = mag3d.get("offset") or [0, 0, 0]
        mat = mag3d.get("matrix") or [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
        flat = tuple(float(v) for row in mat for v in row)
        m3 = proto.CmdMag3dSet(valid=1, offset=tuple(float(v) for v in off),
                               matrix=flat)
    else:
        m3 = proto.CmdMag3dSet(valid=0, offset=(0.0, 0.0, 0.0),
                               matrix=(0.0,) * 9)
    commands.append((proto.MsgType.CMD_MAG3D_SET, m3.to_payload()))

    yz = profile.get("yaw_zero") or {}
    if yz.get("valid"):
        yzp = proto.CmdYawzeroSet(valid=1,
                                  offset_rad=float(yz.get("offset_rad", 0.0)))
    else:
        yzp = proto.CmdYawzeroSet(valid=0, offset_rad=0.0)
    commands.append((proto.MsgType.CMD_YAWZERO_SET, yzp.to_payload()))
    return commands


def profile_matches_cal(profile: dict[str, Any], cal: proto.TlmCalData
                        ) -> tuple[bool, list[str]]:
    """プロファイルと CAL_GET 読み戻しを照合する(適用後検証)。"""
    got = cal_data_to_profile(profile.get("name", ""), cal)
    mismatches: list[str] = []

    def close(a: Any, b: Any) -> bool:
        try:
            return abs(float(a) - float(b)) <= CAL_VERIFY_TOLERANCE
        except (TypeError, ValueError):
            return False

    for section, fields in (
        ("mag3d", ("valid", "offset", "matrix")),
        ("accel6", ("valid", "offset", "scale")),
        ("attitude_mount", ("valid", "roll_rad", "pitch_rad")),
        ("yaw_zero", ("valid", "offset_rad")),
    ):
        want = profile.get(section) or {}
        have = got.get(section) or {}
        if bool(want.get("valid")) != bool(have.get("valid")):
            mismatches.append(f"{section}.valid")
            continue
        if not want.get("valid"):
            continue   # 双方クリア済み。値は無関係
        for field in fields:
            if field == "valid":
                continue
            w, g = want.get(field), have.get(field)
            if isinstance(w, list):
                flat_w = ([x for row in w for x in row]
                          if w and isinstance(w[0], list) else w)
                flat_g = ([x for row in g for x in row]
                          if g and isinstance(g[0], list) else g)
                if (len(flat_w) != len(flat_g)
                        or any(not close(a, b) for a, b in zip(flat_w, flat_g))):
                    mismatches.append(f"{section}.{field}")
            elif not close(w, g):
                mismatches.append(f"{section}.{field}")
    return (len(mismatches) == 0), mismatches


# ----------------------------------------------------------------------
# CalibrationManager
# ----------------------------------------------------------------------

class CalibrationManager:
    """キャリブレーション操作の窓口(session 層が保持)。

    hub は ExperimentHub(TLM_EXP サンプル源+排他スロット)。ブロッキング
    メソッドは executor スレッドから呼ぶこと(app.py 参照)。
    """

    def __init__(self, server_config: dict, link: SerialLink, hub,
                 notify: Callable[[str], None],
                 calprofile_dir: Path = cfg.CALPROFILES_DIR,
                 mag3d_path: Path = cfg.MAG3D_CALIBRATION_PATH,
                 geomag_path: Path = cfg.GEOMAG_PROFILES_PATH,
                 tlm_state_provider: Optional[Callable[
                     [Optional[int]], tuple[Any, Optional[float]]]] = None
                 ) -> None:
        exp_cfg = server_config["experiment"]
        self._cal_get_timeout_s: float = exp_cfg["cal_get_timeout_s"]
        self._accel6_capture_s: float = exp_cfg["accel6_capture_s"]
        self._telemetry_fresh_s: float = (
            server_config["freshness"]["telemetry_fresh_s"])
        self.link = link
        self.hub = hub
        self.notify = notify
        # 最新 TLM_STATE のスナップショット (tlm, age_s) を返すプロバイダ
        # (session 層が配線する。yaw_zero の逆算が yaw_est_rad を、
        # attitude_zero が roll/pitch を使う)。node_id 指定で複数機モードの
        # ノード別スナップショットを返す(session._tlm_state_snapshot)。
        self._tlm_state_provider = tlm_state_provider
        self.calprofile_dir = Path(calprofile_dir)
        self.mag3d_path = Path(mag3d_path)
        self.geomag_path = Path(geomag_path)

        self._lock = threading.Lock()
        self._cal_data: Optional[proto.TlmCalData] = None
        self._cal_data_at = 0.0
        # マルチ機体: ノード別 TLM_CAL_DATA スロット {node: (cal, monotonic)}
        self._cal_data_by_node: dict[int, tuple[proto.TlmCalData, float]] = {}
        self._mag3d_fit: Optional[dict] = None
        self._mag3d_error: Optional[str] = None
        self._accel6 = Accel6Calibrator()
        self._calprofile_message = ""

    # ---- TLM_CAL_DATA 流入(RXスレッド上) ----

    def on_cal_data(self, cal: proto.TlmCalData,
                    node_id: Optional[int] = None) -> None:
        """TLM_CAL_DATA の格納(RXスレッド上)。

        node_id 付き(MUX_DOWN 経由)はノード別スロットへ、単機経路は従来の
        共有スロットへ格納する。遅延到着した別ノードの応答が他機の
        fetch_cal_data を満たさないよう、スロットを分離して帰属を保証する。
        """
        with self._lock:
            if node_id is None:
                self._cal_data = cal
                self._cal_data_at = time.monotonic()
            else:
                self._cal_data_by_node[node_id] = (cal, time.monotonic())

    def fetch_cal_data(self, node_id: Optional[int] = None
                       ) -> Optional[proto.TlmCalData]:
        """CMD_CAL_GET → TLM_CAL_DATA を待つ(タイムアウトで None)。

        node_id 指定時はマルチ機体モードのノード宛に要求し、**当該ノード帰属の
        応答のみ**(ノード別スロット+要求時刻ゲート)を受理する。
        """
        requested_at = time.monotonic()
        try:
            if node_id is None:
                self.link.send(proto.MsgType.CMD_CAL_GET)
            else:
                self.link.send_to(node_id, proto.MsgType.CMD_CAL_GET)
        except SerialLinkError:
            return None
        deadline = time.monotonic() + self._cal_get_timeout_s
        while time.monotonic() < deadline:
            with self._lock:
                if node_id is None:
                    if self._cal_data is not None \
                            and self._cal_data_at >= requested_at:
                        return self._cal_data
                else:
                    entry = self._cal_data_by_node.get(node_id)
                    if entry is not None and entry[1] >= requested_at:
                        return entry[0]
            time.sleep(CAL_DATA_POLL_S)
        return None

    def _send_ack(self, msg_type: int, payload: bytes,
                  node_id: Optional[int] = None) -> tuple[bool, str]:
        """コマンド送信+TLM_ACK 待ち。node_id 指定時はノード宛(MUX_UP 経由)。"""
        try:
            if node_id is None:
                ack = self.link.send_with_ack(msg_type, payload)
            else:
                ack = self.link.send_with_ack_to(node_id, msg_type, payload)
        except SerialLinkError as exc:
            return False, f"送信失敗: {exc}"
        return ack_ok(ack), ack_detail(ack)

    # ---- 3D磁気 ----

    def mag3d_status(self) -> dict:
        hub_status = self.hub.cal3d_status()
        with self._lock:
            fit = self._mag3d_fit
            error = self._mag3d_error
        return {**hub_status, "fit": fit, "error": error,
                "saved": load_saved_calibration(self.mag3d_path)}

    def mag3d_start(self) -> dict:
        self.hub.cal3d_start()
        with self._lock:
            self._mag3d_fit = None
            self._mag3d_error = None
        return self.mag3d_status()

    def mag3d_stop(self) -> dict:
        self.hub.cal3d_stop()
        return self.mag3d_status()

    def mag3d_fit(self) -> dict:
        samples = self.hub.cal3d_stop()
        try:
            result = fit_ellipsoid(samples)
        except Exception as exc:
            with self._lock:
                self._mag3d_error = str(exc)
            return self.mag3d_status()
        result["saved_at"] = time.time()
        self._save_mag3d_file(result)
        with self._lock:
            self._mag3d_fit = result
            self._mag3d_error = None
        return self.mag3d_status()

    def mag3d_apply(self) -> dict:
        with self._lock:
            fit = self._mag3d_fit
        if fit is None:
            saved = load_saved_calibration(self.mag3d_path)
            if saved is None or "error" in saved:
                with self._lock:
                    self._mag3d_error = "適用できる 3D キャリブレーションがありません"
                return self.mag3d_status()
            fit = saved
        offset = tuple(float(v) for v in fit["offset"])
        matrix = tuple(float(v) for row in fit["matrix"] for v in row)
        payload = proto.CmdMag3dSet(valid=1, offset=offset, matrix=matrix)
        ok, detail = self._send_ack(proto.MsgType.CMD_MAG3D_SET,
                                    payload.to_payload())
        if not ok:
            with self._lock:
                self._mag3d_error = f"CMD_MAG3D_SET 失敗: {detail}"
            return self.mag3d_status()
        saved = dict(fit)
        saved["applied_at"] = time.time()
        self._save_mag3d_file(saved)
        with self._lock:
            self._mag3d_error = None
        # mag3d 適用は機体側で FF 自動無効(ff_mode=0)+アンカー破棄を伴う契約
        self.notify("3D磁気キャリブレーションを適用しました"
                    "(機体側で FF は自動無効化されます)")
        return {**self.mag3d_status(), "ok": True}

    def mag3d_clear(self) -> dict:
        payload = proto.CmdMag3dSet(valid=0, offset=(0.0, 0.0, 0.0),
                                    matrix=(0.0,) * 9)
        ok, detail = self._send_ack(proto.MsgType.CMD_MAG3D_SET,
                                    payload.to_payload())
        if ok and self.mag3d_path.exists():
            try:
                self.mag3d_path.unlink()
            except OSError:
                pass
        with self._lock:
            self._mag3d_fit = None
            self._mag3d_error = None if ok else f"CMD_MAG3D_SET 失敗: {detail}"
        return {**self.mag3d_status(), "ok": ok}

    def _save_mag3d_file(self, data: dict) -> None:
        try:
            self.mag3d_path.write_text(json.dumps(data, indent=2),
                                       encoding="utf-8")
        except OSError as exc:
            self.notify(f"[警告] mag3d_calibration.json の保存に失敗: {exc}")

    # ---- 加速度6面 ----

    def accel6_status(self) -> dict:
        with self._lock:
            return {"captured": self._accel6.captured_faces,
                    "ready": self._accel6.ready,
                    "error": self._accel6.last_error or None}

    def accel6_start(self) -> dict:
        with self._lock:
            self._accel6.reset()
        return {**self.accel6_status(), "ok": True}

    def accel6_capture(self, face: str) -> dict:
        """指定面の加速度を一定時間平均してキャプチャする(ブロッキング)。"""
        if face not in ACCEL6_FACES:
            return {**self.accel6_status(), "ok": False,
                    "message": f"不正な面です: {face}"}
        samples = self.hub.collect_samples(self._accel6_capture_s)
        if not samples:
            return {**self.accel6_status(), "ok": False,
                    "message": "実験テレメトリ(TLM_EXP)がありません"}
        n = len(samples)
        mean = (sum(s["ax"] for s in samples) / n,
                sum(s["ay"] for s in samples) / n,
                sum(s["az"] for s in samples) / n)
        with self._lock:
            ok = self._accel6.capture_face(face, mean)
            error = self._accel6.last_error
        return {**self.accel6_status(), "ok": ok,
                "message": (f"面 {face} をキャプチャしました" if ok else error)}

    def accel6_apply(self) -> dict:
        with self._lock:
            solved = self._accel6.solve()
            error = self._accel6.last_error
        if solved is None:
            return {**self.accel6_status(), "ok": False, "message": error}
        offset, scale = solved
        payload = proto.CmdAccel6Set(valid=1, offset=offset, scale=scale)
        ok, detail = self._send_ack(proto.MsgType.CMD_ACCEL6_SET,
                                    payload.to_payload())
        message = ("6面キャリブレーションを適用しました(姿勢参照はリセット)"
                   if ok else f"CMD_ACCEL6_SET 失敗: {detail}")
        return {**self.accel6_status(), "ok": ok, "message": message,
                "offset": list(offset), "scale": list(scale)}

    def accel6_clear(self) -> dict:
        payload = proto.CmdAccel6Set(valid=0, offset=(0.0, 0.0, 0.0),
                                     scale=(0.0, 0.0, 0.0))
        ok, detail = self._send_ack(proto.MsgType.CMD_ACCEL6_SET,
                                    payload.to_payload())
        with self._lock:
            self._accel6.reset()
        message = ("6面キャリブレーションをクリアしました" if ok
                   else f"CMD_ACCEL6_SET 失敗: {detail}")
        return {**self.accel6_status(), "ok": ok, "message": message}

    # ---- Attitude 0 / Yaw 0 / Yaw Clear ----

    def attitude_zero(self, node_id: Optional[int] = None) -> dict:
        """現在姿勢(TLM_STATE の Madgwick roll/pitch)をマウントオフセットに設定。

        姿勢ソースは TLM_STATE(TLM_EXP と同じ Madgwick AHRS 由来)のため、
        全制御モードで動作する。node_id 指定時は複数機モードのノード宛。
        """
        reason = self._ground_activity_guard("マウントオフセットを設定")
        if reason is not None:
            return {"ok": False, "message": reason}
        tlm = self._fresh_tlm_state(node_id)
        if tlm is None:
            return {"ok": False,
                    "message": "テレメトリ(TLM_STATE)が新鮮ではありません"}
        payload = proto.CmdAttmountSet(valid=1,
                                       roll_rad=tlm.roll,
                                       pitch_rad=tlm.pitch)
        ok, detail = self._send_ack(proto.MsgType.CMD_ATTMOUNT_SET,
                                    payload.to_payload(), node_id=node_id)
        return {"ok": ok, "message": ("マウントオフセットを設定しました" if ok
                                      else f"CMD_ATTMOUNT_SET 失敗: {detail}")}

    def attitude_zero_clear(self, node_id: Optional[int] = None) -> dict:
        reason = self._ground_activity_guard("マウントオフセットをクリア")
        if reason is not None:
            return {"ok": False, "message": reason}
        payload = proto.CmdAttmountSet(valid=0, roll_rad=0.0, pitch_rad=0.0)
        ok, detail = self._send_ack(proto.MsgType.CMD_ATTMOUNT_SET,
                                    payload.to_payload(), node_id=node_id)
        return {"ok": ok, "message": ("マウントオフセットをクリアしました" if ok
                                      else f"CMD_ATTMOUNT_SET 失敗: {detail}")}

    def yaw_zero(self, node_id: Optional[int] = None) -> dict:
        """現在の推定ヨーが 0 になるヨーゼロオフセットを逆算して設定する
        (ワンクリック自動シーケンス)。

        CMD_YAWZERO_SET はレベル化磁気ヘディング座標系の mag_yaw_offset を
        直接インストールする復元専用 API で、機体側の推定ヨーは
        wrapPi(yaw_mag_raw − offset) になる。Madgwick ヨー(TLM_EXP)は
        磁気を使わず基準も無関係なため、そのまま送っても推定ヨーは 0 に
        ならない。そこで機体の現在オフセット(TLM_CAL_DATA の
        yawzero_offset_rad = 現在の mag_yaw_offset。valid ビットに関わらず
        現行値が返る)と現在の推定ヨー(TLM_STATE の yaw_est_rad)から
        offset_new = wrap_pi(offset_cur + yaw_est) を逆算して送る。

        ff_mode≠0 のときは yaw_est_rad が補正系推定器(補正CF/EKF)の出力に
        なり、リファレンスCFの磁気オフセット座標系と一致しない。そのため
        サーバ側で FF を一時 off(est_mode 維持)→ TLM_STATE で反映確認 →
        リファレンスCF の yaw_est から逆算・設定 → FF 復元 → アンカー再取得
        (EKF を新基準に整列)まで自動オーケストレーションする。

        node_id 指定時は複数機モードのノード宛に同じ順序で実行する。
        """
        return self._yaw_zero_sequence(clear=False, node_id=node_id)

    def yaw_zero_clear(self, node_id: Optional[int] = None) -> dict:
        """ヨーゼロをクリアする(yaw_zero と同じ自動シーケンスのクリア版)。"""
        return self._yaw_zero_sequence(clear=True, node_id=node_id)

    # ---- ヨーゼロ自動シーケンスの内部ヘルパ ----

    def _fresh_tlm_state(self, node_id: Optional[int] = None
                         ) -> Optional[proto.TlmState]:
        """新鮮な TLM_STATE スナップショット(なければ None)。"""
        if self._tlm_state_provider is None:
            return None
        tlm, age = self._tlm_state_provider(node_id)
        if tlm is None or age is None or age > self._telemetry_fresh_s:
            return None
        return tlm

    def _wait_tlm_state(self, predicate: Callable[[proto.TlmState], bool],
                        timeout_s: float,
                        node_id: Optional[int] = None
                        ) -> Optional[proto.TlmState]:
        """新鮮な TLM_STATE が条件を満たすまでポーリングする。"""
        deadline = time.monotonic() + timeout_s
        while True:
            tlm = self._fresh_tlm_state(node_id)
            if tlm is not None and predicate(tlm):
                return tlm
            if time.monotonic() >= deadline:
                return None
            time.sleep(YAWZERO_POLL_S)

    def _ground_activity_guard(self, operation: str) -> Optional[str]:
        """クイック較正共通の地上アクティビティガード。

        計測・スイープ・モーター回転中はキャリブ値が汚れる(振動・傾き変動・
        推定器状態の変化)ため拒否する。拒否理由(日本語)か None を返す。
        experiment モード外ではいずれも自然に False となり素通しする。
        """
        if self.hub.sweep.is_running() or self.hub.sequence.is_running():
            return f"スイープ/シーケンス実行中は{operation}できません"
        if self.hub.recorder.is_recording():
            return (f"実験計測(Experiment ログ)実行中は{operation}できません"
                    "(推定器状態が変わり計測データが汚れます)")
        if self.hub.motor_status().get("running"):
            return f"モーター回転中は{operation}できません(先に停止してください)"
        sample, age = self.hub.latest_sample()
        if (sample is not None and age is not None
                and age <= self.hub.exp_fresh_s
                and sample.get("motors_running")):
            return f"モーター回転中は{operation}できません(先に停止してください)"
        return None

    def _yaw_zero_guards(self, clear: bool,
                         node_id: Optional[int] = None) -> Optional[str]:
        """ヨーゼロ操作の事前ガード。拒否理由(日本語)か None を返す。"""
        if self._tlm_state_provider is None:
            return "TLM_STATE が参照できないためヨーゼロを操作できません"
        tlm = self._fresh_tlm_state(node_id)
        if tlm is None:
            return "テレメトリ(TLM_STATE)が新鮮ではありません"
        if not clear and not (tlm.ff_status & proto.TlmState.FF_STATUS_MAG_FRESH):
            return ("磁気サンプルが新鮮でないためヨーゼロを設定できません"
                    "(磁気センサの状態を確認してください)")
        # 推定器状態を変えるため、計測・スイープ・モーター回転中は拒否する
        # (FF off 中の CF はモーター磁気外乱で狂い、アンカーも busy になる)
        return self._ground_activity_guard("ヨーゼロを操作")

    def _send_ff_mode(self, ff_mode: int, est_mode: int,
                      node_id: Optional[int] = None) -> tuple[bool, str]:
        return self._send_ack(
            proto.MsgType.CMD_FF_MODE,
            proto.CmdFfMode(ff_mode=ff_mode, est_mode=est_mode).to_payload(),
            node_id=node_id)

    def _anchor_after_yawzero(self, steps: list[str], warnings: list[str],
                              node_id: Optional[int] = None) -> None:
        """CMD_FF_ANCHOR を busy リトライ付きで送る(最終失敗は警告のみ)。"""
        busy_name = ACK_STATUS_NAMES[proto.TlmAck.STATUS_BUSY]
        for attempt in range(YAWZERO_ANCHOR_RETRIES):
            if attempt:
                time.sleep(YAWZERO_ANCHOR_RETRY_S)
            ok, detail = self._send_ack(proto.MsgType.CMD_FF_ANCHOR, b"",
                                        node_id=node_id)
            if ok:
                steps.append("アンカー再取得")
                return
            if detail != busy_name:
                warnings.append(f"アンカー再取得に失敗しました({detail})")
                return
        warnings.append("アンカー再取得は保留です"
                        "(busy: 次のモーター始動時に自動再取得されます)")

    def _yaw_zero_sequence(self, *, clear: bool,
                           node_id: Optional[int] = None) -> dict:
        """ヨーゼロ設定/クリアの自動シーケンス本体。

        FF 有効中でも UI ワンクリックで完結するよう、サーバ側で
        1. CAL_GET(ff_mode/est_mode/offset_cur 取得)
        2. ff_mode≠0 なら CMD_FF_MODE(0, est) で FF 一時 off
        3. TLM_STATE.ff_status 下位2bit==0 で反映確認
        4. CMD_YAWZERO_SET(設定はリファレンスCF の yaw_est から逆算)
        5. CF の新基準整列を確認(設定時のみ。超過は警告)
        6. ff_mode≠0 だったら CMD_FF_MODE で復元(失敗は警告+手動復元案内)
        7. CMD_FF_ANCHOR を busy リトライ付きで送信(最終 busy は警告)
        を実行する。途中失敗時は FF モードの復元を試みてから失敗を返す。
        node_id 指定時は全手順をノード宛(MUX_UP/DOWN 経由)で実行する。
        """
        op = "ヨーゼロクリア" if clear else "ヨーゼロ設定"
        reason = self._yaw_zero_guards(clear, node_id)
        if reason:
            return {"ok": False, "message": reason}
        # スイープ/シーケンスとの相互排他+プロファイル操作との直列化
        # (ffprofile と同じ排他スロット。TOCTOU なし)
        busy = self.hub.calprofile_begin()
        if busy:
            return {"ok": False, "message": busy}
        steps: list[str] = []
        warnings: list[str] = []
        try:
            cal = self.fetch_cal_data(node_id=node_id)
            if cal is None:
                return {"ok": False,
                        "message": "機体からキャリブレーションデータを取得"
                                   "できませんでした(リンク/機体状態を確認)"}
            ff_mode_orig = int(cal.ff_mode)
            est_mode_orig = int(cal.est_mode)
            ff_restore_pending = False   # FF off 送信後〜復元前の途中失敗検知
            try:
                if ff_mode_orig != 0:
                    # FF 一時 off(est_mode は維持。機体側で KF 再シード)
                    ff_restore_pending = True
                    ok, detail = self._send_ff_mode(0, est_mode_orig, node_id)
                    if not ok:
                        raise _YawZeroAbort(f"CMD_FF_MODE(off) 失敗: {detail}")
                    steps.append("FF一時off")
                    # TLM_STATE で反映確認(以降の yaw_est = リファレンスCF)
                    if self._wait_tlm_state(
                            lambda t: (t.ff_status
                                       & proto.TlmState.FF_STATUS_FF_MODE_MASK)
                            == 0, YAWZERO_MODE_TIMEOUT_S,
                            node_id) is None:
                        raise _YawZeroAbort(
                            "FF off の反映を TLM_STATE で確認できませんでした")
                    steps.append("FF off反映確認")
                if clear:
                    payload = proto.CmdYawzeroSet(valid=0, offset_rad=0.0)
                else:
                    tlm = self._fresh_tlm_state(node_id)
                    if tlm is None:
                        raise _YawZeroAbort(
                            "テレメトリ(TLM_STATE)が新鮮ではありません")
                    offset_new = wrap_pi(cal.yawzero_offset_rad
                                         + tlm.yaw_est_rad)
                    payload = proto.CmdYawzeroSet(valid=1,
                                                  offset_rad=offset_new)
                ok, detail = self._send_ack(proto.MsgType.CMD_YAWZERO_SET,
                                            payload.to_payload(),
                                            node_id=node_id)
                if not ok:
                    raise _YawZeroAbort(f"CMD_YAWZERO_SET 失敗: {detail}")
                steps.append("ヨーゼロクリア" if clear
                             else f"ヨーゼロ設定(offset={payload.offset_rad:+.3f} rad)")
                if not clear:
                    # CF は次の磁気サンプル(≈0.1s)で新基準に整列する。
                    # 確認できなくても失敗にはしない(警告のみ)
                    if self._wait_tlm_state(
                            lambda t: abs(t.yaw_est_rad)
                            < YAWZERO_ALIGN_TOL_RAD,
                            YAWZERO_ALIGN_TIMEOUT_S,
                            node_id) is not None:
                        steps.append("CF整列確認")
                    else:
                        warnings.append("CF の新基準整列(|yaw|<8°)を確認"
                                        "できませんでした(磁気サンプル待ち"
                                        "の可能性)")
                if ff_mode_orig != 0:
                    # FF モード復元(失敗しても継続するが必ず明記する)
                    ff_restore_pending = False
                    ok, detail = self._send_ff_mode(ff_mode_orig,
                                                    est_mode_orig, node_id)
                    if ok:
                        steps.append(f"FF復元(ff={ff_mode_orig}, "
                                     f"est={est_mode_orig})")
                    else:
                        warnings.append(
                            f"FFモードの復元に失敗しました({detail})。"
                            "FFモードがoffのままです。手動で復元してください")
                # アンカー再取得(EKF の基準を新ヨーゼロに整列)
                self._anchor_after_yawzero(steps, warnings, node_id)
            except _YawZeroAbort as exc:
                message = str(exc)
                if ff_restore_pending:
                    # 途中失敗でも FF モードは必ず復元を試みる
                    ok, detail = self._send_ff_mode(ff_mode_orig,
                                                    est_mode_orig, node_id)
                    if ok:
                        message += "(FFモードは復元しました)"
                    else:
                        message += (f"(FFモードの復元にも失敗: {detail}。"
                                    "FFモードがoffのままです。手動で復元して"
                                    "ください)")
                return {"ok": False,
                        "message": f"{op}に失敗しました: {message}",
                        "steps": steps, "warnings": warnings}
            message = f"{op}を完了しました({' → '.join(steps)})"
            if warnings:
                message += " / 警告: " + " / ".join(warnings)
            self.notify(message)
            return {"ok": True, "message": message,
                    "steps": steps, "warnings": warnings}
        finally:
            self.hub.calprofile_end()

    # ---- 地磁気 ----

    def geomag_status(self) -> dict:
        return {"config": load_geomagnetic_config(self.geomag_path),
                "profiles": geomag_profile_list(self.geomag_path)}

    def geomag_select(self, profile_id: Any) -> dict:
        """選択都道府県を切り替えて永続化し、機体へ適用する。"""
        pid = str(profile_id or "").strip()
        try:
            data = json.loads(self.geomag_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            return {**self.geomag_status(), "ok": False,
                    "message": f"地磁気プロファイルを読めません: {exc}"}
        profiles = data.get("profiles")
        if not isinstance(profiles, dict) or pid not in profiles:
            return {**self.geomag_status(), "ok": False,
                    "message": f"不明な地磁気プロファイルです: {pid}"}
        data["selected"] = pid
        try:
            self.geomag_path.write_text(
                json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
        except OSError as exc:
            return {**self.geomag_status(), "ok": False,
                    "message": f"地磁気プロファイルを保存できません: {exc}"}
        return self.geomag_apply()

    def geomag_apply(self) -> dict:
        config = load_geomagnetic_config(self.geomag_path)
        if "error" in config:
            return {**self.geomag_status(), "ok": False,
                    "message": str(config["error"])}
        p = config["profile"]
        payload = proto.CmdGeomagSet(
            declination_east_deg=p["declination_east_deg"],
            inclination_deg=p["inclination_deg"],
            horizontal_ut=p["horizontal_uT"],
            vertical_ut=p["vertical_uT"],
            total_ut=p["total_uT"])
        ok, detail = self._send_ack(proto.MsgType.CMD_GEOMAG_SET,
                                    payload.to_payload())
        message = (f"地磁気プロファイルを適用しました: {p['label']}" if ok
                   else f"CMD_GEOMAG_SET 失敗: {detail}")
        return {**self.geomag_status(), "ok": ok, "message": message}

    # ---- キャリブレーション・プロファイル ----

    def _set_calprofile_message(self, message: str) -> None:
        with self._lock:
            self._calprofile_message = message

    def calprofile_status(self) -> dict:
        profiles = []
        if self.calprofile_dir.is_dir():
            for path in sorted(self.calprofile_dir.glob("*.json"),
                               key=mtime_or_zero, reverse=True):
                entry: dict[str, Any] = {"name": path.stem}
                try:
                    data = json.loads(path.read_text(encoding="utf-8"))
                    entry["saved_at"] = data.get("saved_at")
                    entry["valid"] = {
                        section: bool((data.get(section) or {}).get("valid"))
                        for section in ("mag3d", "accel6", "attitude_mount",
                                        "yaw_zero")
                    }
                except FileNotFoundError:
                    continue   # glob() と read の間に削除された
                except (OSError, json.JSONDecodeError):
                    entry["error"] = "unreadable"
                profiles.append(entry)
        with self._lock:
            message = self._calprofile_message
        return {"profiles": profiles, "message": message}

    def calprofile_save(self, name: Any) -> dict:
        clean = sanitize_profile_name(name)
        if not clean:
            self._set_calprofile_message("プロファイル名を入力してください")
            return {**self.calprofile_status(), "ok": False}
        busy = self.hub.calprofile_begin()
        if busy:
            self._set_calprofile_message(busy)
            return {**self.calprofile_status(), "ok": False}
        try:
            cal = self.fetch_cal_data()
            if cal is None:
                self._set_calprofile_message(
                    "機体からキャリブレーションデータを取得できませんでした"
                    "(リンク/機体状態を確認)")
                return {**self.calprofile_status(), "ok": False}
            profile = cal_data_to_profile(clean, cal)
            self.calprofile_dir.mkdir(parents=True, exist_ok=True)
            (self.calprofile_dir / f"{clean}.json").write_text(
                json.dumps(profile, ensure_ascii=False, indent=2),
                encoding="utf-8")
            valid_parts = [s for s in ("mag3d", "accel6", "attitude_mount",
                                       "yaw_zero") if profile[s]["valid"]]
            self._set_calprofile_message(
                f"保存しました: {clean}"
                f"(有効: {', '.join(valid_parts) if valid_parts else 'なし'})")
            return {**self.calprofile_status(), "ok": True, "profile": profile}
        finally:
            self.hub.calprofile_end()

    def calprofile_apply(self, name: Any) -> dict:
        clean = sanitize_profile_name(name)
        path = self.calprofile_dir / f"{clean}.json"
        if not clean or not path.is_file():
            self._set_calprofile_message(f"プロファイルが見つかりません: {clean}")
            return {**self.calprofile_status(), "ok": False}
        try:
            profile = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self._set_calprofile_message(f"プロファイル読み込み失敗: {exc}")
            return {**self.calprofile_status(), "ok": False}
        # 送信前に全コマンドを構築・検証する(手編集の不正プロファイルが
        # 途中失敗して機体を中途半端な状態に残さないため)
        try:
            commands = profile_apply_commands(profile)
        except (TypeError, KeyError, IndexError, ValueError,
                AttributeError, struct.error) as exc:
            self._set_calprofile_message(
                f"プロファイル形式が不正です: {clean}"
                f"({type(exc).__name__}: {exc})")
            return {**self.calprofile_status(), "ok": False}
        busy = self.hub.calprofile_begin()
        if busy:
            self._set_calprofile_message(busy)
            return {**self.calprofile_status(), "ok": False}
        try:
            for msg_type, payload in commands:
                ok, detail = self._send_ack(msg_type, payload)
                if not ok:
                    self._set_calprofile_message(
                        f"適用に失敗しました: {clean}"
                        f"({proto.MsgType(msg_type).name}: {detail})")
                    return {**self.calprofile_status(), "ok": False,
                            "verified": False}
            # 読み戻し検証(NVS まで届いたことの確認)
            cal = self.fetch_cal_data()
            if cal is None:
                self._set_calprofile_message(
                    f"適用コマンドは送信しましたが検証応答がありません: {clean}")
                return {**self.calprofile_status(), "ok": False,
                        "verified": False}
            verified, mismatches = profile_matches_cal(profile, cal)
            if verified:
                self._set_calprofile_message(
                    f"適用・検証OK: {clean}(NVSに書き込み済み)")
            else:
                self._set_calprofile_message(
                    f"適用しましたが不一致があります: {clean} → "
                    f"{', '.join(mismatches[:6])}")
            return {**self.calprofile_status(), "ok": verified,
                    "verified": verified, "mismatches": mismatches}
        finally:
            self.hub.calprofile_end()

    def calprofile_delete(self, name: Any) -> dict:
        clean = sanitize_profile_name(name)
        path = self.calprofile_dir / f"{clean}.json"
        busy = self.hub.calprofile_begin()
        if busy:
            self._set_calprofile_message(busy)
            return {**self.calprofile_status(), "ok": False}
        try:
            if not clean or not path.is_file():
                self._set_calprofile_message(f"プロファイルが見つかりません: {clean}")
                return {**self.calprofile_status(), "ok": False}
            path.unlink()
            self._set_calprofile_message(f"削除しました: {clean}")
            return {**self.calprofile_status(), "ok": True}
        finally:
            self.hub.calprofile_end()
