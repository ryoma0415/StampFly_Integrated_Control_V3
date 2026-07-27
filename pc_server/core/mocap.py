"""NatNet(OptiTrack)接続と座標変換。

- vendor/ の NatNet SDK(NatNetClient.py ほか)を無改変のままラップする。
  出自は NatNet SDK 4.3.0 付属の PythonClient(リポジトリ同梱 ../NatNetSDK/ の
  DLLバージョンリソースで確認)。クライアントは旧ビットストリームと後方互換のため
  Motive 本体のバージョンはこれだけでは確定しない(README §4.3)。
- 座標変換は Motive(Y-up)→ 制御座標系。マッピングは control.json の
  "coordinate_transform" で定義する(legacy hovering_controller と同方式)。
  既定: 制御x ← Motive z、制御y ← -Motive x(y軸符号は旧実装の負ゲイン規約を
  座標変換側へ移したもの)、制御z ← Motive y(高度)。
- 位置はリジッドボディのみを使用する。全マーカー重心フォールバックは
  流用禁止(ARCHITECTURE.md)のため実装しない。
- yaw_rad(Z-up 前提オイラー分解)は UI 表示専用の旧値(Motive が Y-up の
  場合は機首方位ではない — 2026-07-27 実測確定)。機首方位は heading_rad
  (前方軸の制御座標方位・生値)と yaw_true_rad(AttitudeMapper で符号/
  オフセット/フリップ補正済みの「正解ヨー」)を使う。
- 座標変換と姿勢マッピングは実行時差し替え可能(set_mapping)。適用は
  地上限定・位置フィルタのリセットとセット(session 層が保証する)。

NatNet 受信コールバック(SDKのスレッド)上では print やブロッキングを行わない。
"""

from __future__ import annotations

import socket
import threading
import time
from math import asin, atan2, copysign, pi
from typing import Callable, Optional

RAD_TO_DEG = 180.0 / pi
DEG_TO_RAD = pi / 180.0

DEFAULT_COORDINATE_TRANSFORM = {
    "x": {"axis": "z", "sign": 1},
    "y": {"axis": "x", "sign": -1},
    "z": {"axis": "y", "sign": 1},
}

# 姿勢マッピング(control.json "attitude_transform")の既定値。
# 既定は従来実装と完全互換: 前方軸 +X の制御座標方位そのまま
# (yaw_sign=1, offset=0 なら yaw_true == heading)。
DEFAULT_ATTITUDE_TRANSFORM = {
    "forward_axis": "+x",     # 機首を向くボディ軸(±x/±y/±z)
    "up_axis": "+y",          # 正立時に鉛直上を向くボディ軸(Motive Y-up 定義の既定)
    "yaw_sign": 1,            # 制御ヨー規約への符号合わせ
    "yaw_offset_deg": 0.0,    # 定数オフセット(フレーム回転・RB定義ずれの吸収)
    "flip_correction": True,  # ソルバ180°反転(マーカー対称性)の補正
    "flip_gate": 0.5,         # 上方軸の鉛直成分がこの絶対値未満の間は反転判定を保留
}

# フリップ補正の構造定数(挙動の性質を決める閾値であり運用調整項ではない)。
# ヨー継続性ガード: フレーム間で |Δyaw| がこれ以上なら 180° 反転とみなす。
# 実機のヨーレートでは 100Hz 1フレームに 120° は物理的に到達不能
# (12000deg/s)なので誤検出しない。
_YAW_JUMP_RAD = pi * 2.0 / 3.0
# 受信ギャップがこれを超えたら継続性を主張しない(蓄積した反転補正を破棄)
_YAW_JUMP_MAX_GAP_S = 0.5
# 前方軸が鉛直に近い(|sin(仰角)| がこれ以上)間は方位が不定のため
# 継続性判定を更新しない(sin 75°)
_YAW_JUMP_ELEV_GATE_SIN = 0.966

# リジッドボディ品質(0-1)算出時、tracking_valid でない場合の減衰率
_QUALITY_INVALID_SCALE = 0.5
_QUALITY_FLOOR = 0.05

# 全ボディインベントリ(紐付け確認用)の保持上限(構造定数。Motive が流す
# リジッドボディ数は高々数個だが、ID の付け直しで無限成長しないための保険)
_INVENTORY_MAX = 64

# vendor NatNetClient.shutdown() を諦めるまでの待ち時間 [s](構造定数)。
# vendor の join はタイムアウト無しのため、看視スレッド側で有限化する。
_SHUTDOWN_JOIN_TIMEOUT_S = 2.0


class _DaemonThread(threading.Thread):
    """vendor NatNetClient が生成する受信スレッドを daemon 化する差し替え。

    vendor のデータ/コマンドスレッドは非デーモンで、multicast データソケットに
    タイムアウトが無い。Motive が配信していない瞬間に shutdown すると
    (macOS/BSD では close() が blocking recvfrom を中断しないため)join が
    無期限化し、UI コマンドのロック(_command_lock)保持と Ctrl-C 不能を
    引き起こす。スレッドを daemon 化して「最悪でもプロセス終了を妨げない」
    ことを構造的に保証する(vendor ファイル自体は無改変のまま、モジュール
    属性 Thread の差し替えのみで実現する)。
    """

    def __init__(self, *args, **kwargs) -> None:
        super().__init__(*args, **kwargs)
        self.daemon = True


def quaternion_rotate_x_axis(qx: float, qy: float, qz: float, qw: float
                             ) -> tuple[float, float, float]:
    """クォータニオンで機体前方軸(ボディ x 軸単位ベクトル)を回した結果。

    回転行列の第1列に相当する。ヨー(方位)を「前方軸の水平面内の向き」
    として座標系変換後に計算するために使う(オイラー角の軸順規約に
    依存しないため、Motive の Y-up 座標系でも安全)。
    """
    return (
        1.0 - 2.0 * (qy * qy + qz * qz),
        2.0 * (qx * qy + qw * qz),
        2.0 * (qx * qz - qw * qy),
    )


def quaternion_body_axis(rotation: tuple, axis_index: int
                         ) -> tuple[float, float, float]:
    """ボディ軸単位ベクトル(0=X, 1=Y, 2=Z)をワールド(Motive)座標で返す。

    回転行列(body→world)の第 axis_index+1 列。axis_index=0 は
    quaternion_rotate_x_axis と同一。前方軸・上方軸を任意のボディ軸に
    選べるようにするための一般化(オイラー角の軸順規約に依存しない)。
    """
    qx, qy, qz, qw = rotation
    if axis_index == 0:
        return (
            1.0 - 2.0 * (qy * qy + qz * qz),
            2.0 * (qx * qy + qw * qz),
            2.0 * (qx * qz - qw * qy),
        )
    if axis_index == 1:
        return (
            2.0 * (qx * qy - qw * qz),
            1.0 - 2.0 * (qx * qx + qz * qz),
            2.0 * (qy * qz + qw * qx),
        )
    return (
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )


def wrap_pi(angle: float) -> float:
    """角度を (-pi, pi] へラップする。"""
    while angle > pi:
        angle -= 2.0 * pi
    while angle <= -pi:
        angle += 2.0 * pi
    return angle


def quaternion_to_euler_xyz(qx: float, qy: float, qz: float, qw: float
                            ) -> tuple[float, float, float]:
    """クォータニオン (x, y, z, w) → (roll, pitch, yaw) [rad]。"""
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (qw * qy - qz * qx)
    if abs(sinp) >= 1.0:
        pitch = copysign(pi / 2.0, sinp)
    else:
        pitch = asin(sinp)

    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = atan2(siny_cosp, cosy_cosp)
    return (roll, pitch, yaw)


class CoordinateTransformer:
    """Motive 座標 (X, Y, Z) を制御座標系へ変換する。"""

    AXIS_INDEX = {"x": 0, "y": 1, "z": 2}

    def __init__(self, transform_config: Optional[dict] = None) -> None:
        config = transform_config or DEFAULT_COORDINATE_TRANSFORM
        self.axis_map: dict[str, tuple[int, int]] = {}
        for axis in ("x", "y", "z"):
            axis_cfg = config.get(axis, DEFAULT_COORDINATE_TRANSFORM[axis])
            src_axis = axis_cfg["axis"]
            if src_axis not in self.AXIS_INDEX:
                raise ValueError(f"coordinate_transform: invalid axis {src_axis!r}")
            sign = -1 if float(axis_cfg["sign"]) < 0 else 1
            self.axis_map[axis] = (self.AXIS_INDEX[src_axis], sign)

    def motive_to_control(self, position) -> Optional[tuple[float, float, float]]:
        """Motive 座標タプル → 制御座標タプル。"""
        if position is None:
            return None
        src = tuple(position)
        x_idx, x_sign = self.axis_map["x"]
        y_idx, y_sign = self.axis_map["y"]
        z_idx, z_sign = self.axis_map["z"]
        return (src[x_idx] * x_sign, src[y_idx] * y_sign, src[z_idx] * z_sign)

    def describe(self) -> dict:
        """設定ファイル形式(control.json "coordinate_transform")の記述を返す。

        API 応答・ログの meta.json に「適用中のマッピング」として記録する。
        """
        return {axis: {"axis": "xyz"[idx], "sign": sign}
                for axis, (idx, sign) in self.axis_map.items()}


# 前方軸/上方軸の指定文字列("+x" / "-z" など)→ (軸index, 符号)
_SIGNED_AXIS = {
    "+x": (0, 1.0), "-x": (0, -1.0),
    "+y": (1, 1.0), "-y": (1, -1.0),
    "+z": (2, 1.0), "-z": (2, -1.0),
}


class AttitudeMapper:
    """クォータニオン → 制御座標系ヨー(heading / yaw_true)の導出。

    調整ノブは「前方軸・上方軸の選択」「符号」「オフセット」のみで、
    ワールド軸の対応は位置と同じ CoordinateTransformer を通すことで
    自動的に整合する(独立の姿勢軸マップは持たない — 位置を直せば
    ヨーも追従する、が本設計の不変条件)。

    フリップ補正(flip_correction)は2層:
    - 上方軸ガード: マーカー配置の対称性によるソルバの180°別解
      (前方軸まわり反転 = 上方軸が下を向く)を検出し、上方軸を
      反転して姿勢の整合を回復する。前方軸自体は不変のためヨーには
      影響しない。ヒステリシス付き(|鉛直成分| < flip_gate では保留)。
    - ヨー継続性ガード: 鉛直軸まわりの180°別解(上方軸では検出不能)
      をフレーム間 |Δyaw| >= _YAW_JUMP_RAD で検出し、180°補正を
      トグルする。受信ギャップ・前方軸が鉛直に近い間は判定しない。

    インスタンスは不変(構築後に設定を書き換えない)。実行時変更は
    MocapSource.set_mapping() でオブジェクトごと差し替える。
    ミュータブルな判定状態(反転フラグ等)は呼び出し側が保持する
    (new_state() で生成し derive() に渡す)。
    """

    def __init__(self, attitude_config: Optional[dict] = None) -> None:
        config = dict(DEFAULT_ATTITUDE_TRANSFORM)
        config.update(attitude_config or {})
        forward = str(config["forward_axis"]).lower()
        up = str(config["up_axis"]).lower()
        if forward not in _SIGNED_AXIS:
            raise ValueError(f"attitude_transform: invalid forward_axis {forward!r}")
        if up not in _SIGNED_AXIS:
            raise ValueError(f"attitude_transform: invalid up_axis {up!r}")
        self._fwd_idx, self._fwd_sign = _SIGNED_AXIS[forward]
        self._up_idx, self._up_sign = _SIGNED_AXIS[up]
        if self._fwd_idx == self._up_idx:
            raise ValueError("attitude_transform: forward_axis と up_axis が同一軸です")
        self._yaw_sign = -1.0 if float(config["yaw_sign"]) < 0 else 1.0
        self._yaw_offset_rad = float(config["yaw_offset_deg"]) * DEG_TO_RAD
        self._flip_correction = bool(config["flip_correction"])
        self._flip_gate = float(config["flip_gate"])
        # describe() 用の正規化済み設定
        self._config = {
            "forward_axis": forward,
            "up_axis": up,
            "yaw_sign": int(self._yaw_sign),
            "yaw_offset_deg": float(config["yaw_offset_deg"]),
            "flip_correction": self._flip_correction,
            "flip_gate": self._flip_gate,
        }

    def describe(self) -> dict:
        """設定ファイル形式(control.json "attitude_transform")の記述を返す。"""
        return dict(self._config)

    @staticmethod
    def new_state() -> dict:
        """リジッドボディ1体ぶんのフリップ判定状態を生成する。"""
        return {"up_flipped": False, "cont_flip": False,
                "last_yaw": None, "last_t": None}

    # フリップフラグ(pose["flip_flags"] / ログ mocap_flip 列のビット)
    FLIP_UP_INVERTED = 0x01    # 上方軸ガードが反転解を補正中
    FLIP_YAW_JUMP = 0x02       # ヨー継続性ガードが180°補正中

    def derive(self, rotation: tuple, transformer: CoordinateTransformer,
               state: dict, now: float,
               tracking_valid: bool = True) -> tuple[float, float, int]:
        """クォータニオン → (heading_rad, yaw_true_rad, flip_flags)。

        heading_rad: 設定された前方軸の制御座標系方位(生値。符号・
          オフセット・フリップ補正を掛けない — 既定設定では従来の
          heading_rad と同値で、CMD_POS_ERR / mocap_heading_deg の
          既存契約を維持する)。
        yaw_true_rad: sign*heading + offset にフリップ補正を適用し
          (-pi, pi] にラップした「正解ヨー」。ログ・表示・将来の制御用。

        tracking_valid=False(オクルージョン等でソルバが外挿中)の間は
        フリップ判定状態を一切更新しない(保持中の補正は適用し続ける)。
        姿勢が信用できないフレームで継続性を更新すると、遮蔽越しに実機が
        回転した場合に恒久的な180°誤補正が残るため、遮蔽はギャップとして
        扱い、再捕捉時に既存のギャップリセット経路で観測を信じ直す。

        NatNet 受信スレッド上で毎フレーム呼ばれる: ロック・ブロッキング
        禁止。state はボディ別 dict(呼び出し側が保持、受信スレッド
        のみが触る)。
        """
        fwd_world = quaternion_body_axis(rotation, self._fwd_idx)
        if self._fwd_sign < 0:
            fwd_world = (-fwd_world[0], -fwd_world[1], -fwd_world[2])
        fwd_ctrl = transformer.motive_to_control(fwd_world)
        heading = atan2(fwd_ctrl[1], fwd_ctrl[0])

        flags = 0
        if self._flip_correction:
            up_world = quaternion_body_axis(rotation, self._up_idx)
            up_ctrl = transformer.motive_to_control(up_world)
            up_vert = up_ctrl[2] * self._up_sign
            # 上方軸ガード(ヒステリシス): 明確に下向き/上向きのときのみ
            # 状態を更新し、鉛直成分が小さい(大姿勢変化中)間は保留する。
            # tracking 無効中は姿勢自体が外挿のため状態を凍結する
            if tracking_valid:
                if up_vert <= -self._flip_gate:
                    state["up_flipped"] = True
                elif up_vert >= self._flip_gate:
                    state["up_flipped"] = False
            if state["up_flipped"]:
                flags |= self.FLIP_UP_INVERTED

        yaw = wrap_pi(self._yaw_sign * heading + self._yaw_offset_rad)
        if self._flip_correction:
            if state["cont_flip"]:
                yaw = wrap_pi(yaw + pi)
            elev_sin = max(-1.0, min(1.0, fwd_ctrl[2]))
            if not tracking_valid:
                # 遮蔽中: 状態を触らない(last_t が経過し、再捕捉が
                # 0.5s 超ならギャップリセットで補正破棄)
                pass
            elif abs(elev_sin) >= _YAW_JUMP_ELEV_GATE_SIN:
                # 前方軸が鉛直に近い: 方位不定のため継続性を明示的に
                # 破棄する(凍結時間の長短に関わらず、復帰フレームは
                # ギャップ扱い → 蓄積補正リセットで観測を信じ直す)
                state["last_yaw"] = None
                state["last_t"] = None
            else:
                last_yaw = state["last_yaw"]
                last_t = state["last_t"]
                gap_ok = (last_t is not None
                          and (now - last_t) <= _YAW_JUMP_MAX_GAP_S)
                if gap_ok and last_yaw is not None:
                    if abs(wrap_pi(yaw - last_yaw)) >= _YAW_JUMP_RAD:
                        state["cont_flip"] = not state["cont_flip"]
                        yaw = wrap_pi(yaw + pi)
                elif not gap_ok:
                    # ギャップ越し(または初回)は継続性を主張できない:
                    # 蓄積した180°補正を破棄して現在の観測を信じる
                    if state["cont_flip"]:
                        state["cont_flip"] = False
                        yaw = wrap_pi(yaw + pi)
                state["last_yaw"] = yaw
                state["last_t"] = now
            if state["cont_flip"]:
                flags |= self.FLIP_YAW_JUMP
        return heading, yaw, flags


def validate_mapping(transform_config: dict,
                     attitude_config: dict) -> Optional[str]:
    """マッピング設定の厳格検証。問題があれば UI 表示用メッセージを返す。

    CoordinateTransformer 単体の検証(軸名のみ)より厳しく、実行時変更
    API(PUT /api/mocap/mapping)の受理条件を規定する:
    - coordinate_transform: x/y/z 全指定、参照元軸は各1回ずつ
      (符号付き置換 = 可逆)、sign は ±1
    - attitude_transform: AttitudeMapper の構築検証+数値範囲
    """
    if not isinstance(transform_config, dict):
        return "coordinate_transform がオブジェクトではありません"
    used: list[str] = []
    for axis in ("x", "y", "z"):
        axis_cfg = transform_config.get(axis)
        if not isinstance(axis_cfg, dict):
            return f"coordinate_transform.{axis} がありません"
        src = axis_cfg.get("axis")
        if src not in ("x", "y", "z"):
            return f"coordinate_transform.{axis}.axis が不正です: {src!r}"
        sign = axis_cfg.get("sign")
        if sign not in (1, -1):
            return (f"coordinate_transform.{axis}.sign は 1 か -1 を"
                    f"指定してください: {sign!r}")
        used.append(src)
    if len(set(used)) != 3:
        return ("coordinate_transform が縮退しています(Motive の各軸は"
                "ちょうど1回ずつ参照してください)")

    if not isinstance(attitude_config, dict):
        return "attitude_transform がオブジェクトではありません"
    merged = dict(DEFAULT_ATTITUDE_TRANSFORM)
    merged.update(attitude_config)
    if str(merged["forward_axis"]).lower() not in _SIGNED_AXIS:
        return f"forward_axis が不正です: {merged['forward_axis']!r}"
    if str(merged["up_axis"]).lower() not in _SIGNED_AXIS:
        return f"up_axis が不正です: {merged['up_axis']!r}"
    if (_SIGNED_AXIS[str(merged["forward_axis"]).lower()][0]
            == _SIGNED_AXIS[str(merged["up_axis"]).lower()][0]):
        return "forward_axis と up_axis に同じ軸は指定できません"
    if merged["yaw_sign"] not in (1, -1):
        return f"yaw_sign は 1 か -1 を指定してください: {merged['yaw_sign']!r}"
    try:
        offset = float(merged["yaw_offset_deg"])
    except (TypeError, ValueError):
        return f"yaw_offset_deg が数値ではありません: {merged['yaw_offset_deg']!r}"
    if not (-360.0 <= offset <= 360.0):
        return "yaw_offset_deg は -360〜360 の範囲で指定してください"
    if not isinstance(merged["flip_correction"], bool):
        return "flip_correction は true/false を指定してください"
    try:
        gate = float(merged["flip_gate"])
    except (TypeError, ValueError):
        return f"flip_gate が数値ではありません: {merged['flip_gate']!r}"
    if not (0.05 <= gate <= 0.95):
        return "flip_gate は 0.05〜0.95 の範囲で指定してください"
    return None


class MocapSource:
    """NatNet クライアントのライフサイクルとリジッドボディ姿勢の抽出。

    on_pose コールバックには制御座標系へ変換済みの pose dict を渡す:
    ``{rigid_body_id, t_mono, frame_number, x, y, z, yaw_rad, heading_rad,
       yaw_true_rad, flip_flags, quat, tracking_valid, error, marker_count,
       quality}``
    (heading_rad は制御座標系での機体前方軸の方位・生値。yaw_true_rad は
    符号/オフセット/フリップ補正済みの「正解ヨー」。quat は Motive 生
    クォータニオン (qx, qy, qz, qw) — ログに残せば任意のマッピングを
    事後再計算できる)
    対象リジッドボディがフレームに存在しない場合はコールバックを呼ばない
    (消費側は pose の経過時間で途絶を検出する)。

    マルチ機体: NatNet は毎フレーム全リジッドボディを配信するため、
    subscribe(id, callback) で ID 別のコールバックを追加登録できる
    (1つの NatNet クライアントで N 機ぶんを賄う)。また全ボディの最新
    pose をインベントリとして保持し、bodies_snapshot() で取り出せる
    (UI の「リジッドボディ紐付け確認」用)。
    """

    def __init__(self, natnet_config: dict, transform_config: dict,
                 attitude_config: Optional[dict] = None,
                 client_factory: Optional[Callable[[], object]] = None) -> None:
        self._server_address: str = natnet_config["server_address"]
        self._client_address: str = natnet_config["client_address"]
        self._use_multicast: bool = natnet_config["use_multicast"]
        self._rigid_body_id: int = natnet_config["rigid_body_id"]
        # マッピング一式は単一タプルで保持する: NatNet スレッドは毎フレーム
        # このタプルを1回読むだけなので、set_mapping() の属性代入1つで
        # 「座標変換・姿勢導出・フリップ判定状態」が不可分に切り替わる
        # (座標変換だけ新・姿勢だけ旧という混在フレームを作らない)。
        # 第3要素はボディID別フリップ判定状態(受信スレッドのみが触る)。
        # 第4要素はマッピング世代: pose に刻印され、set_mapping 直前に旧
        # マッピングで計算中だったフレームを消費側(PositionController)が
        # 世代フロアで破棄できるようにする(旧座標の pose がリセット直後の
        # フィルタをシードする競合の防止)。
        self._mapping: tuple[CoordinateTransformer, AttitudeMapper,
                             dict, int] = (
            CoordinateTransformer(transform_config),
            AttitudeMapper(attitude_config),
            {},
            0,
        )
        self._client_factory = client_factory or self._default_client_factory

        self._client: Optional[object] = None
        self._on_pose: Optional[Callable[[dict], None]] = None
        # start/shutdown の check-then-act を直列化する(パッシブ起動と
        # Position モード起動が別スレッドから同時に来てもクライアントを
        # 二重生成しない)。start() が失敗時に shutdown() を呼ぶため RLock。
        self._lifecycle_lock = threading.RLock()

        self._lock = threading.Lock()
        self._latest_pose: Optional[dict] = None
        self._frames_total = 0
        self._frames_without_rigid_body = 0
        # マルチ機体: ID別コールバック(_lock 保護。NatNetスレッドから配送)
        self._subscriptions: dict[int, Callable[[dict], None]] = {}
        # 全リジッドボディの最新 pose(紐付け確認用インベントリ、_lock 保護)
        self._bodies: dict[int, dict] = {}

    @staticmethod
    def _default_client_factory():
        # vendor/ は core/__init__.py のシムで sys.path に追加済み
        import NatNetClient as natnet_module  # type: ignore
        # vendor 無改変のまま、run() が生成するスレッドだけ daemon 化する
        # (_DaemonThread の docstring 参照)。冪等な差し替え。
        if getattr(natnet_module, "Thread", None) is not _DaemonThread:
            natnet_module.Thread = _DaemonThread
        return natnet_module.NatNetClient()

    # ------------------------------------------------------------------
    # ライフサイクル
    # ------------------------------------------------------------------

    def start(self, on_pose: Optional[Callable[[dict], None]] = None) -> bool:
        """NatNet クライアントを起動する。Returns: 起動成功フラグ。

        on_pose は primary(natnet.rigid_body_id)コールバック。None なら
        パッシブ起動(インベントリ/subscribe のみ使用 — 複数機モードや
        紐付け確認)。既に起動済みの場合、on_pose が指定されていれば
        primary コールバックを差し替える(パッシブ起動が先行しても
        Position モードのコールバック登録が阻害されないように)。
        """
        with self._lifecycle_lock:
            if self._client is not None:
                if on_pose is not None:
                    self._on_pose = on_pose
                return True
            self._on_pose = on_pose
            client = self._client_factory()
            client.set_server_address(self._server_address)
            client.set_client_address(self._client_address)
            client.set_use_multicast(self._use_multicast)
            client.new_frame_with_data_listener = self._receive_frame
            client.set_print_level(0)
            self._client = client
            ok = bool(client.run("d"))
            if not ok:
                self.shutdown()
            return ok

    def connected(self) -> bool:
        client = self._client
        return bool(client and client.connected())

    def shutdown(self) -> None:
        with self._lifecycle_lock:
            client = self._client
            self._client = None
            self._on_pose = None
        if client is not None:
            self._shutdown_client(client)

    @staticmethod
    def _wake_receiver_sockets(client) -> None:
        """recvfrom でブロック中の vendor 受信スレッドを起こす(ベストエフォート)。

        multicast データソケットにはタイムアウトが無く、macOS では close() が
        blocking recvfrom を中断しない。バインドされ得るポートへ空 UDP
        データグラムを送ると recvfrom が返り、ループが stop フラグを見て
        自然終了する(空データグラムは len==0 のためフレーム処理もされない)。
        """
        ports: set[int] = set()
        for attr in ("data_port", "command_port"):
            port = getattr(client, attr, None)
            if isinstance(port, int) and 0 < port < 65536:
                ports.add(port)
        # unicast モードではデータ/コマンドソケットがエフェメラルポートに
        # bind されるため、実際の bind 先ポートもソケットから取得する
        # (close 前に呼ばれる前提 — _shutdown_client がリーパー起動前に呼ぶ)
        for attr in ("data_socket", "command_socket"):
            sock_obj = getattr(client, attr, None)
            if sock_obj is None:
                continue
            try:
                bound_port = sock_obj.getsockname()[1]
            except (OSError, AttributeError, IndexError, TypeError):
                continue
            if isinstance(bound_port, int) and 0 < bound_port < 65536:
                ports.add(bound_port)
        if not ports:
            return
        addresses = {"127.0.0.1"}
        for attr in ("local_ip_address", "multicast_address"):
            address = getattr(client, attr, None)
            if isinstance(address, str) and address:
                addresses.add(address)
        try:
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        except OSError:
            return
        try:
            for address in addresses:
                for port in ports:
                    try:
                        sock.sendto(b"", (address, port))
                    except OSError:
                        continue
        finally:
            sock.close()

    @staticmethod
    def _shutdown_client(client) -> None:
        """vendor クライアントを有限時間で停止する。

        vendor NatNetClient.shutdown() はタイムアウト無しで受信スレッドを
        join するため、Motive が配信していないと(macOS では close() が
        blocking recvfrom を起こさず)無期限にブロックし得る。ここでは
        (1) stop フラグを先に立て、(2) 空データグラムで recvfrom を起こし、
        (3) shutdown 本体は看視スレッドで実行して _SHUTDOWN_JOIN_TIMEOUT_S で
        諦める。諦めた場合も残るのは daemon スレッドのみ(プロセス終了を
        妨げない)。
        """
        try:
            # 起こされたスレッドが必ずループを抜けるよう、先にフラグを立てる
            client.stop_threads = True
        except Exception:
            pass
        MocapSource._wake_receiver_sockets(client)

        def _call_shutdown() -> None:
            try:
                client.shutdown()
            except Exception:
                pass

        reaper = threading.Thread(target=_call_shutdown,
                                  name="natnet-shutdown", daemon=True)
        reaper.start()
        reaper.join(timeout=_SHUTDOWN_JOIN_TIMEOUT_S)
        # join できなくても放置してよい(daemon 化済みのため終了は妨げない)

    # ------------------------------------------------------------------
    # フレーム処理(NatNet スレッド上で実行: ブロッキング・print 禁止)
    # ------------------------------------------------------------------

    def _receive_frame(self, data_dict: dict) -> None:
        mocap_data = data_dict.get("mocap_data")
        if mocap_data is None:
            return
        frame_number = data_dict.get("frame_number", 0)

        # フレーム内の全リジッドボディを一度の走査で pose 化する
        poses: dict[int, dict] = {}
        rigid_body_data = getattr(mocap_data, "rigid_body_data", None)
        rigid_bodies = (getattr(rigid_body_data, "rigid_body_list", None)
                        if rigid_body_data is not None else None)
        for rigid_body in rigid_bodies or []:
            rb_id = getattr(rigid_body, "id_num", None)
            if rb_id is None:
                continue
            pose = self._pose_from_rigid_body(rigid_body, int(rb_id),
                                              frame_number)
            if pose is not None:
                poses[int(rb_id)] = pose

        primary = poses.get(self._rigid_body_id)
        with self._lock:
            self._frames_total += 1
            if primary is None:
                # 対象リジッドボディ非検出。重心フォールバックは行わない(流用禁止)。
                self._frames_without_rigid_body += 1
            else:
                self._latest_pose = primary
            # インベントリ更新(上限超過は最も古いIDから捨てる)
            self._bodies.update(poses)
            if len(self._bodies) > _INVENTORY_MAX:
                excess = len(self._bodies) - _INVENTORY_MAX
                # フリップ判定状態も同時に破棄する(att_states は
                # _bodies と同じ「ID 付け直しで無限成長しない」不変条件を
                # 持つ。ここは受信スレッド上なので att_states への操作は
                # 競合しない — set_mapping と入れ違っても空 dict への
                # pop になるだけで無害)
                att_states = self._mapping[2]
                for stale_id in sorted(
                        self._bodies,
                        key=lambda i: self._bodies[i]["t_mono"])[:excess]:
                    del self._bodies[stale_id]
                    att_states.pop(stale_id, None)
            # 配送対象をロック内で確定し、コールバック呼び出しはロック外で行う
            deliveries = [(cb, poses[rb_id])
                          for rb_id, cb in self._subscriptions.items()
                          if rb_id in poses]

        callback = self._on_pose
        if primary is not None and callback is not None:
            callback(primary)
        for cb, pose in deliveries:
            cb(pose)

    def _pose_from_rigid_body(self, rigid_body, rb_id: int,
                              frame_number: int) -> Optional[dict]:
        """リジッドボディ1体を制御座標系の pose dict に変換する。"""
        # マッピング一式を1回だけ読む(set_mapping() との競合はこの
        # タプル読み出しの前後どちらかに全体が倒れる — 混在しない)
        transformer, attitude, att_states, mapping_gen = self._mapping
        position = transformer.motive_to_control(tuple(rigid_body.pos))
        if position is None:
            return None
        rotation = tuple(rigid_body.rot)
        _, _, yaw_rad = quaternion_to_euler_xyz(*rotation)
        tracking_valid = bool(getattr(rigid_body, "tracking_valid", False))
        # 制御座標系ヨー: 設定された前方軸(既定 +X)を Motive 座標で
        # 回してから位置と同じ軸マップで制御座標系へ変換し、水平面内の
        # 方位角をとる。heading_rad は生値(既存契約: CMD_POS_ERR /
        # mocap_heading_deg)、yaw_true_rad は符号・オフセット・フリップ
        # 補正済みの「正解ヨー」(ログ・表示用)。
        t_mono = time.monotonic()
        state = att_states.get(rb_id)
        if state is None:
            state = AttitudeMapper.new_state()
            att_states[rb_id] = state
        heading_rad, yaw_true_rad, flip_flags = attitude.derive(
            rotation, transformer, state, t_mono,
            tracking_valid=tracking_valid)
        error = getattr(rigid_body, "error", None)
        marker_count = len(getattr(rigid_body, "rb_marker_list", []) or [])

        quality = 1.0
        if error is not None:
            quality = 1.0 / (1.0 + max(0.0, error))
        if not tracking_valid:
            quality *= _QUALITY_INVALID_SCALE
        quality = max(_QUALITY_FLOOR, min(1.0, quality))

        return {
            "rigid_body_id": rb_id,
            "t_mono": t_mono,
            "frame_number": frame_number,
            "x": position[0],
            "y": position[1],
            "z": position[2],
            # 変換前の Motive 生座標(設定タブのマッピング・プレビュー用)
            "motive_pos": tuple(rigid_body.pos),
            "yaw_rad": yaw_rad,
            "heading_rad": heading_rad,
            "yaw_true_rad": yaw_true_rad,
            "flip_flags": flip_flags,
            "quat": rotation,
            "mapping_gen": mapping_gen,
            "tracking_valid": tracking_valid,
            "error": error,
            "marker_count": marker_count,
            "quality": quality,
        }

    # ------------------------------------------------------------------
    # スナップショット
    # ------------------------------------------------------------------

    def latest_pose(self) -> Optional[dict]:
        """最後に検出した pose のコピーを返す(未検出なら None)。"""
        with self._lock:
            return dict(self._latest_pose) if self._latest_pose else None

    # ------------------------------------------------------------------
    # マッピングの参照と実行時差し替え
    # ------------------------------------------------------------------

    def mapping_snapshot(self) -> dict:
        """適用中のマッピング(設定ファイル形式)。API 応答・ログ用。"""
        transformer, attitude, _, _ = self._mapping
        return {
            "coordinate_transform": transformer.describe(),
            "attitude_transform": attitude.describe(),
        }

    @property
    def mapping_generation(self) -> int:
        """適用中マッピングの世代(set_mapping ごとに +1)。

        pose dict の "mapping_gen" と対で使う: 消費側はこの値を世代フロア
        として保持し、フロア未満の pose(差し替え直前に旧マッピングで計算
        されたフレーム)を破棄する。
        """
        return self._mapping[3]

    def set_mapping(self, transform_config: dict,
                    attitude_config: dict) -> None:
        """座標変換・姿勢マッピングを実行時に差し替える。

        タプル1個の属性代入なので NatNet 受信スレッドに対してアトミック
        (ロック不要。受信側は毎フレーム self._mapping を1回読む)。
        フリップ判定状態も同時に空へ差し替え(旧フレームでの判定を
        新マッピングへ持ち越さない)、世代を +1 する。

        注意: 呼び出し側(session 層)の責務 —
        - 事前に validate_mapping() で検証すること(ここでは構築時
          ValueError 以上の検証はしない)
        - 地上(非武装)でのみ呼ぶこと(飛行中の座標系変更は閉ループの
          極性反転になり得る)
        - 呼び出し後に PositionController の世代フロア更新+
          reset_filter() を呼ぶこと(旧フレームのアンカーによる外れ値
          ロックアウト/旧座標 pose の新フィルタへのシードを防ぐ)
        """
        self._mapping = (
            CoordinateTransformer(transform_config),
            AttitudeMapper(attitude_config),
            {},
            self._mapping[3] + 1,
        )

    def stats(self) -> dict:
        with self._lock:
            return {
                "frames_total": self._frames_total,
                "frames_without_rigid_body": self._frames_without_rigid_body,
            }

    # ------------------------------------------------------------------
    # マルチ機体: ID別サブスクリプションとインベントリ
    # ------------------------------------------------------------------

    def subscribe(self, rigid_body_id: int,
                  on_pose: Callable[[dict], None]) -> None:
        """ID 別 pose コールバックを登録する(NatNet スレッド上で呼ばれる。
        ブロッキング・print 禁止)。同一 ID への再登録は置き換え。"""
        with self._lock:
            self._subscriptions[int(rigid_body_id)] = on_pose

    def unsubscribe(self, rigid_body_id: int) -> None:
        with self._lock:
            self._subscriptions.pop(int(rigid_body_id), None)

    def clear_subscriptions(self) -> None:
        with self._lock:
            self._subscriptions.clear()

    def bodies_snapshot(self) -> list[dict]:
        """観測済み全リジッドボディの最新 pose 一覧(紐付け確認 UI 用)。

        各要素は pose dict + ``age_s``(最終観測からの経過秒)。ID 昇順。
        """
        now = time.monotonic()
        with self._lock:
            bodies = [dict(pose) for pose in self._bodies.values()]
        for body in bodies:
            body["age_s"] = now - body["t_mono"]
        return sorted(bodies, key=lambda b: b["rigid_body_id"])
