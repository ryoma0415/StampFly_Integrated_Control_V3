"""StampFly Integrated Control — 通信プロトコル(Python実装)。

docs/PROTOCOL.md v2 が唯一の正(single source of truth)。
C++実装 ``stampfly_protocol.hpp`` とのバイト互換は ``test_vectors.json`` と
``tests/`` により強制される。

ワイヤ仕様の要点:

- 論理フレーム: ``ver(1) | type(1) | seq(4, u32 LE) | len(1) | payload | crc16(2, LE)``
- CRC16-CCITT-FALSE: poly 0x1021 / init 0xFFFF / 非反転 / xorout なし。
  ver〜payload 末尾(crc自身を除く全バイト)に適用。検証ベクタ "123456789" -> 0x29B1。
- シリアル区間: 論理フレーム全体を COBS エンコードし、末尾に 0x00 デリミタを付加。
  受信側は 0x00 区切りで蓄積(上限 256B、超過したら次の 0x00 まで読み捨て)。
  デコード失敗 / CRC不一致 / ver不一致 / len不整合は「フレームごと破棄」(部分回復しない)。
- マルチバイト値はすべてリトルエンディアン。float は IEEE-754 binary32 LE。
"""

from __future__ import annotations

import struct
from dataclasses import dataclass
from enum import IntEnum
from typing import Callable, Optional

# ---------------------------------------------------------------------------
# 定数(PROTOCOL.md 「論理フレーム」「トランスポート」)
# ---------------------------------------------------------------------------

PROTOCOL_VERSION = 0x02

FRAME_HEADER_SIZE = 7          # ver(1) + type(1) + seq(4) + len(1)
FRAME_CRC_SIZE = 2
FRAME_OVERHEAD = FRAME_HEADER_SIZE + FRAME_CRC_SIZE   # = 9
MAX_PAYLOAD_SIZE = 200
MAX_FRAME_SIZE = FRAME_OVERHEAD + MAX_PAYLOAD_SIZE    # = 209

COBS_DELIMITER = 0x00
SERIAL_RX_BUFFER_CAP = 256     # 受信蓄積バッファ上限(超過 → 次の 0x00 まで読み捨て)

MAX_LOG_TEXT_SIZE = 180        # LOG_TEXT の UTF-8 テキスト上限

# --- マルチ機体(リレー多重化)関連 ---
RLY_MAX_PEERS = 4              # RLY_SET_PEERS の最大登録数(=同時制御機体数上限)
RLY_PEER_ENTRY_SIZE = 7        # mac(6) + tlm_state_div(1)
MUX_HEADER_SIZE = 1            # RLY_MUX_UP/DOWN の node_id(1B)
# エンベロープに収まる内側フレームの最大ペイロード長(= 200 - 1 - 9 = 190)。
# 既存の全メッセージ(最大 TLM_STATE 184B)が収まる。
MAX_MUX_INNER_PAYLOAD = MAX_PAYLOAD_SIZE - MUX_HEADER_SIZE - FRAME_OVERHEAD


# ---------------------------------------------------------------------------
# メッセージ型 / enum(PROTOCOL.md 「メッセージ型」「enum定義」)
# ---------------------------------------------------------------------------

class MsgType(IntEnum):
    # 上り(PC -> ドローン): 0x10–0x2F
    CMD_START = 0x10
    CMD_STOP = 0x11
    CMD_SETPOINT = 0x12
    CMD_RESET = 0x13
    CMD_MODE = 0x14
    CMD_MOTOR_RUN = 0x15
    CMD_MOTOR_STOP = 0x16
    CMD_CAL_GET = 0x17
    CMD_MAG3D_SET = 0x18
    CMD_ACCEL6_SET = 0x19
    CMD_ATTMOUNT_SET = 0x1A
    CMD_YAWZERO_SET = 0x1B
    CMD_GEOMAG_SET = 0x1C
    CMD_FF_BEGIN = 0x1D
    CMD_FF_LUT = 0x1E
    CMD_FF_MOT = 0x1F
    CMD_FF_AUX = 0x20
    CMD_FF_COMMIT = 0x21
    CMD_FF_MODE = 0x22
    CMD_FF_ANCHOR = 0x23
    CMD_POS_ERR = 0x24
    CMD_LED_MODE = 0x25
    # 磁気オートチューン/フロー拡張(MAG_AUTOTUNE_DESIGN.md §1.4。追加のみ・
    # ver=0x02 のまま。契約書記載の 0x24-0x26 は既存 CMD_POS_ERR / CMD_LED_MODE
    # と衝突するため、空きの 0x26-0x28 を同順で割当)
    CMD_MAGBIAS_SET = 0x26
    CMD_FLOWCAL_SET = 0x27
    CMD_FLOW_PROBE = 0x28
    # 下り(ドローン -> PC): 0x30–0x4F
    TLM_STATE = 0x30
    TLM_EVENT = 0x31
    TLM_ACK = 0x32
    TLM_EXP = 0x33
    TLM_CAL_DATA = 0x34
    TLM_CTRL = 0x35
    # ログ(リレー/ドローン -> PC)
    LOG_TEXT = 0x40
    # リレー宛/発: 0x50–0x5F
    RLY_SET_TARGET = 0x50
    RLY_TARGET_ACK = 0x51
    RLY_STATS = 0x52
    RLY_PING = 0x53
    RLY_PONG = 0x54
    # マルチ機体拡張(追加のみ・ver=0x02 のまま。単機経路 0x50/0x51 とは排他)
    RLY_SET_PEERS = 0x55
    RLY_PEERS_ACK = 0x56
    RLY_MUX_UP = 0x57
    RLY_MUX_DOWN = 0x58


def is_uplink_type(msg_type: int) -> bool:
    """ドローン行き(リレーが上り ESP-NOW へ転送)の型レンジか。"""
    return 0x10 <= msg_type <= 0x2F


def is_downlink_type(msg_type: int) -> bool:
    """PC行き(リレーが下りシリアルへ転送)の型レンジか。"""
    return 0x30 <= msg_type <= 0x4F


def is_relay_type(msg_type: int) -> bool:
    """リレー自身宛/発の型レンジか。"""
    return 0x50 <= msg_type <= 0x5F


class FlightState(IntEnum):
    INIT = 0
    CALIBRATION = 1
    WAIT = 2
    TAKEOFF = 3
    HOVER = 4
    LANDING = 5
    COMPLETE = 6
    MOTOR_TEST = 7   # v2: モーターテストモード(AUTO_MOTOR_TEST)


class Reason(IntEnum):
    NONE = 0
    START_CMD = 1
    STOP_CMD = 2
    MAX_FLIGHT_TIME = 3
    LOW_VOLTAGE = 4
    START_REJECTED_LOW_VOLTAGE = 5
    LANDED = 6
    OVER_G = 7
    LINK_LOSS = 8
    RESET_CMD = 9
    START_REJECTED_NOT_READY = 10
    MODE_CHANGE = 11   # v2: CMD_MODE による WAIT<->MOTOR_TEST 遷移


class ParseStatus(IntEnum):
    """parse_frame の結果。C++ 側 ParseStatus と同順。"""
    OK = 0
    BAD_VER = 1
    BAD_CRC = 2
    BAD_LEN = 3
    OVERFLOW = 4


# ---------------------------------------------------------------------------
# CRC16-CCITT-FALSE
# ---------------------------------------------------------------------------

def crc16_ccitt_false(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC16-CCITT-FALSE(poly 0x1021, init 0xFFFF, 非反転, xorout なし)。"""
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


# ---------------------------------------------------------------------------
# COBS(Consistent Overhead Byte Stuffing)
# ---------------------------------------------------------------------------

class CobsDecodeError(ValueError):
    """COBS デコード失敗(0x00混入 / ブロック切れ / 空入力)。"""


def cobs_encode(data: bytes) -> bytes:
    """COBS エンコード(デリミタ 0x00 は含まない。送信時は呼び出し元が付加する)。

    本実装は「入力末尾が満杯(254B)ブロックで終わる場合にも終端コード 0x01 を
    出力する」方式。C++ 実装と完全に同一のアルゴリズム構造であり、両実装の
    出力はバイト単位で一致する。
    """
    out = bytearray(b"\x00")   # 先頭コードバイトのプレースホルダ
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code = 1
            code_idx = len(out)
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:
                out[code_idx] = code
                code = 1
                code_idx = len(out)
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    """COBS デコード(入力にデリミタ 0x00 を含めないこと)。

    失敗時は CobsDecodeError。0x00 混入・ブロック長不足を厳格に拒否する。
    """
    n = len(data)
    if n == 0:
        raise CobsDecodeError("empty input")
    out = bytearray()
    i = 0
    while i < n:
        code = data[i]
        if code == 0:
            raise CobsDecodeError("delimiter byte inside data")
        i += 1
        end = i + code - 1
        if end > n:
            raise CobsDecodeError("truncated block")
        block = data[i:end]
        if 0 in block:
            raise CobsDecodeError("zero byte inside block")
        out += block
        i = end
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# ---------------------------------------------------------------------------
# 論理フレーム pack / parse
# ---------------------------------------------------------------------------

_FRAME_HEADER_FMT = "<BBIB"    # ver, type, seq, len
assert struct.calcsize(_FRAME_HEADER_FMT) == FRAME_HEADER_SIZE


@dataclass(frozen=True)
class Frame:
    """検証済み論理フレーム。"""
    type: int
    seq: int
    payload: bytes
    ver: int = PROTOCOL_VERSION


def pack_frame(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """論理フレーム(ver..crc16)を構築する。"""
    msg_type = int(msg_type)
    if not 0 <= msg_type <= 0xFF:
        raise ValueError(f"msg_type out of range: {msg_type}")
    if not 0 <= seq <= 0xFFFFFFFF:
        raise ValueError(f"seq must fit u32: {seq}")
    payload = bytes(payload)
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"payload too long: {len(payload)} > {MAX_PAYLOAD_SIZE}")
    body = struct.pack(_FRAME_HEADER_FMT, PROTOCOL_VERSION, msg_type, seq, len(payload)) + payload
    return body + struct.pack("<H", crc16_ccitt_false(body))


def parse_frame(data: bytes) -> tuple[ParseStatus, Optional[Frame]]:
    """論理フレームを検証・分解する。

    判定順: 構造(BAD_LEN)→ CRC(BAD_CRC)→ バージョン(BAD_VER)。
    CRC を ver 判定より先に行うのは、CRC不一致フレームの ver バイト自体が
    信用できないため(CRC を通過して ver!=2 のものだけが本当の別バージョン。
    v1 機器の混在は ver_errors として可視化される)。
    """
    data = bytes(data)
    if len(data) < FRAME_OVERHEAD:
        return ParseStatus.BAD_LEN, None
    payload_len = data[6]
    if payload_len > MAX_PAYLOAD_SIZE:
        return ParseStatus.BAD_LEN, None
    if len(data) != FRAME_OVERHEAD + payload_len:
        return ParseStatus.BAD_LEN, None
    crc_off = FRAME_HEADER_SIZE + payload_len
    crc_calc = crc16_ccitt_false(data[:crc_off])
    (crc_rx,) = struct.unpack_from("<H", data, crc_off)
    if crc_calc != crc_rx:
        return ParseStatus.BAD_CRC, None
    if data[0] != PROTOCOL_VERSION:
        return ParseStatus.BAD_VER, None
    ver, msg_type, seq, _ = struct.unpack_from(_FRAME_HEADER_FMT, data, 0)
    return ParseStatus.OK, Frame(type=msg_type, seq=seq,
                                 payload=data[FRAME_HEADER_SIZE:crc_off], ver=ver)


def encode_wire(msg_type: int, seq: int, payload: bytes = b"") -> bytes:
    """シリアル区間ワイヤ形式: COBS(論理フレーム) + 0x00 デリミタ。"""
    return cobs_encode(pack_frame(msg_type, seq, payload)) + bytes([COBS_DELIMITER])


# ---------------------------------------------------------------------------
# ペイロード (de)serializer
#
# struct フォーマットはすべて '<'(リトルエンディアン・パディングなし)で明示。
# 各 from_payload は長さを厳格に検証し、不一致は ValueError。
# ---------------------------------------------------------------------------

_CMD_SETPOINT_FMT = "<ffffB"
_CMD_POS_ERR_FMT = "<fffffB"
_CMD_MODE_FMT = "<B"
_CMD_MOTOR_RUN_FMT = "<fB"
_CMD_MAG3D_SET_FMT = "<B3f9f"
_CMD_ACCEL6_SET_FMT = "<B3f3f"
_CMD_ATTMOUNT_SET_FMT = "<Bff"
_CMD_YAWZERO_SET_FMT = "<Bf"
_CMD_GEOMAG_SET_FMT = "<5f"
_CMD_FF_BEGIN_FMT = "<B"
_CMD_FF_LUT_FMT = "<B4f"
_CMD_FF_MOT_FMT = "<B3f3f"
_CMD_FF_AUX_FMT = "<f"
_CMD_FF_COMMIT_FMT = "<I"
_CMD_FF_MODE_FMT = "<BB"
_CMD_LED_MODE_FMT = "<B"
_CMD_MAGBIAS_SET_FMT = "<B3f"
_CMD_FLOWCAL_SET_FMT = "<B4f"
_CMD_FLOW_PROBE_FMT = "<B"
_TLM_EVENT_FMT = "<BBBBf"
_TLM_STATE_FMT = "<IIBBB21fH9fBB9fBB2fBBB"
_TLM_ACK_FMT = "<BIB"
_TLM_EXP_FMT = "<I20fBB"
_TLM_CAL_DATA_FMT = "<B26fBIBB7f"
_TLM_CTRL_FMT = "<I21fB"
_RLY_SET_TARGET_FMT = "<6sB"
_RLY_TARGET_ACK_FMT = "<B6sB"
_RLY_STATS_FMT = "<IIIIII"
_RLY_PONG_FMT = "<I"
_RLY_PEERS_ACK_FMT = "<BBBB"

# PROTOCOL.md 記載のペイロードサイズと一致することを import 時に強制
# (C++ 側の static_assert に対応)
assert struct.calcsize(_CMD_SETPOINT_FMT) == 17
assert struct.calcsize(_CMD_POS_ERR_FMT) == 21
assert struct.calcsize(_CMD_MODE_FMT) == 1
assert struct.calcsize(_CMD_MOTOR_RUN_FMT) == 5
assert struct.calcsize(_CMD_MAG3D_SET_FMT) == 49
assert struct.calcsize(_CMD_ACCEL6_SET_FMT) == 25
assert struct.calcsize(_CMD_ATTMOUNT_SET_FMT) == 9
assert struct.calcsize(_CMD_YAWZERO_SET_FMT) == 5
assert struct.calcsize(_CMD_GEOMAG_SET_FMT) == 20
assert struct.calcsize(_CMD_FF_BEGIN_FMT) == 1
assert struct.calcsize(_CMD_FF_LUT_FMT) == 17
assert struct.calcsize(_CMD_FF_MOT_FMT) == 25
assert struct.calcsize(_CMD_FF_AUX_FMT) == 4
assert struct.calcsize(_CMD_FF_COMMIT_FMT) == 4
assert struct.calcsize(_CMD_FF_MODE_FMT) == 2
assert struct.calcsize(_CMD_LED_MODE_FMT) == 1
assert struct.calcsize(_CMD_MAGBIAS_SET_FMT) == 13
assert struct.calcsize(_CMD_FLOWCAL_SET_FMT) == 17
assert struct.calcsize(_CMD_FLOW_PROBE_FMT) == 1
assert struct.calcsize(_TLM_EVENT_FMT) == 8
assert struct.calcsize(_TLM_STATE_FMT) == 184
assert struct.calcsize(_TLM_ACK_FMT) == 6
assert struct.calcsize(_TLM_EXP_FMT) == 86
assert struct.calcsize(_TLM_CAL_DATA_FMT) == 140
assert struct.calcsize(_TLM_CTRL_FMT) == 89
assert struct.calcsize(_RLY_SET_TARGET_FMT) == 7
assert struct.calcsize(_RLY_TARGET_ACK_FMT) == 8
assert struct.calcsize(_RLY_STATS_FMT) == 24
assert struct.calcsize(_RLY_PONG_FMT) == 4
assert struct.calcsize(_RLY_PEERS_ACK_FMT) == 4


def _check_len(name: str, data: bytes, expected: int) -> None:
    if len(data) != expected:
        raise ValueError(f"{name}: payload length {len(data)} != {expected}")


@dataclass
class CmdSetpoint:
    """0x12 CMD_SETPOINT(17B)— 姿勢+高度+ヨー目標。ハートビートを兼ねる(50Hz)。

    v2: yaw_ref(±π、機体ヨー角目標)と flags bit1(yaw_ref 有効)を追加。
    """
    roll_ref: float = 0.0    # rad
    pitch_ref: float = 0.0   # rad
    alt_ref: float = 0.0     # m
    yaw_ref: float = 0.0     # rad(±π、機体ヨー角目標)
    flags: int = 0           # bit0 = alt_ref 有効、bit1 = yaw_ref 有効(=ヨー角制御ON)

    TYPE = MsgType.CMD_SETPOINT
    PAYLOAD_SIZE = 17
    FLAG_ALT_REF_VALID = 0x01
    FLAG_YAW_REF_VALID = 0x02

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_SETPOINT_FMT, self.roll_ref, self.pitch_ref,
                           self.alt_ref, self.yaw_ref, self.flags)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdSetpoint":
        _check_len("CMD_SETPOINT", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_SETPOINT_FMT, data))


@dataclass
class CmdPosErr:
    """0x24 CMD_POS_ERR(21B)— XY 位置誤差+高度+ヨー目標(機上XY制御モード)。

    CMD_SETPOINT の代替ストリーム(50Hz、ハートビート兼用)。PC は roll/pitch
    角度指令の代わりに制御座標系の位置誤差(目標 − フィルタ済み現在位置)を
    送り、機体側が自身のヨー推定で誤差を機体座標系へ回転(ヨー回転補償)
    してから XY PID を回す。alt_ref / yaw_ref / flags bit0-1 の意味は
    CMD_SETPOINT と同一。

    mocap_yaw は**外部ヨー基準**(制御座標系ヨー、±π。ソースは PC 側設定:
    MoCap 実測ヨー or MoCap 位置から計算する移動ベースヨー。名称は互換のため
    維持)。ファームはソース非依存に消費し、EKF2(est_mode=2)のヨー擬似観測
    ・フレーム整合検証・ログに用いる。bit3 が立っている場合のみ有効。
    bit4(FLAG_YAW_REF_LOW_TRUST)は基準ヨーの信頼度低(移動ベースヨー時に
    PC が立てる)→ファームは R_ψ を低信頼プリセットへ切替(MAG_AUTOTUNE_
    DESIGN.md §1.2)。
    """
    err_x: float = 0.0      # m(制御座標系。target - filtered)
    err_y: float = 0.0      # m
    alt_ref: float = 0.0    # m
    yaw_ref: float = 0.0    # rad(±π、機体ヨー角目標)
    mocap_yaw: float = 0.0  # rad(±π、外部ヨー基準。bit3 有効時のみ)
    flags: int = 0

    TYPE = MsgType.CMD_POS_ERR
    PAYLOAD_SIZE = 21
    FLAG_ALT_REF_VALID = 0x01      # bit0(CMD_SETPOINT と同義)
    FLAG_YAW_REF_VALID = 0x02      # bit1(同上)
    FLAG_XY_ERR_VALID = 0x04       # bit2: err_x/err_y 有効(MoCap 新鮮+閉ループ有効)
    FLAG_MOCAP_YAW_VALID = 0x08    # bit3: mocap_yaw(外部ヨー基準)有効
    FLAG_YAW_REF_LOW_TRUST = 0x10  # bit4: 基準ヨー低信頼(R_ψ 低信頼プリセット)

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_POS_ERR_FMT, self.err_x, self.err_y,
                           self.alt_ref, self.yaw_ref, self.mocap_yaw,
                           self.flags)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdPosErr":
        _check_len("CMD_POS_ERR", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_POS_ERR_FMT, data))


@dataclass
class CmdMode:
    """0x14 CMD_MODE(1B)— WAIT->MOTOR_TEST(mode=1)/ MOTOR_TEST->WAIT(mode=0)。

    他状態では TLM_ACK status=bad_state。
    """
    mode: int = 0

    TYPE = MsgType.CMD_MODE
    PAYLOAD_SIZE = 1
    MODE_FLIGHT = 0
    MODE_MOTOR_TEST = 1

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_MODE_FMT, self.mode)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdMode":
        _check_len("CMD_MODE", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_MODE_FMT, data))


@dataclass
class CmdMotorRun:
    """0x15 CMD_MOTOR_RUN(5B)— MOTOR_TEST 状態のみ。0.4s 周期キープアライブ。

    機体は 1.5s 途絶で自動停止。ソフトスタート 2.0duty/s。
    """
    duty: float = 0.0    # 0–1
    mask: int = 0        # bit0=FL, bit1=FR, bit2=RL, bit3=RR

    TYPE = MsgType.CMD_MOTOR_RUN
    PAYLOAD_SIZE = 5
    MASK_FL = 0x01
    MASK_FR = 0x02
    MASK_RL = 0x04
    MASK_RR = 0x08

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_MOTOR_RUN_FMT, self.duty, self.mask)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdMotorRun":
        _check_len("CMD_MOTOR_RUN", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_MOTOR_RUN_FMT, data))


@dataclass
class CmdMag3dSet:
    """0x18 CMD_MAG3D_SET(49B)— 3D磁気キャリブ設定。valid=0 でクリア。

    適用時: NVS 永続化+FF 自動無効(ff_mode=0)+アンカー破棄+ヨー推定器再シード。
    """
    valid: int = 0
    offset: tuple = (0.0, 0.0, 0.0)   # µT
    matrix: tuple = (0.0,) * 9        # 行優先

    TYPE = MsgType.CMD_MAG3D_SET
    PAYLOAD_SIZE = 49

    def to_payload(self) -> bytes:
        if len(self.offset) != 3 or len(self.matrix) != 9:
            raise ValueError("CMD_MAG3D_SET: offset は 3 要素、matrix は 9 要素")
        return struct.pack(_CMD_MAG3D_SET_FMT, self.valid,
                           *self.offset, *self.matrix)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdMag3dSet":
        _check_len("CMD_MAG3D_SET", data, cls.PAYLOAD_SIZE)
        vals = struct.unpack(_CMD_MAG3D_SET_FMT, data)
        return cls(valid=vals[0], offset=tuple(vals[1:4]), matrix=tuple(vals[4:13]))


@dataclass
class CmdAccel6Set:
    """0x19 CMD_ACCEL6_SET(25B)— 加速度6面キャリブ設定。適用時に姿勢参照リセット。"""
    valid: int = 0
    offset: tuple = (0.0, 0.0, 0.0)   # g
    scale: tuple = (0.0, 0.0, 0.0)

    TYPE = MsgType.CMD_ACCEL6_SET
    PAYLOAD_SIZE = 25

    def to_payload(self) -> bytes:
        if len(self.offset) != 3 or len(self.scale) != 3:
            raise ValueError("CMD_ACCEL6_SET: offset / scale は各 3 要素")
        return struct.pack(_CMD_ACCEL6_SET_FMT, self.valid,
                           *self.offset, *self.scale)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdAccel6Set":
        _check_len("CMD_ACCEL6_SET", data, cls.PAYLOAD_SIZE)
        vals = struct.unpack(_CMD_ACCEL6_SET_FMT, data)
        return cls(valid=vals[0], offset=tuple(vals[1:4]), scale=tuple(vals[4:7]))


@dataclass
class CmdAttmountSet:
    """0x1A CMD_ATTMOUNT_SET(9B)— 姿勢マウントオフセット設定。"""
    valid: int = 0
    roll_rad: float = 0.0
    pitch_rad: float = 0.0

    TYPE = MsgType.CMD_ATTMOUNT_SET
    PAYLOAD_SIZE = 9

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_ATTMOUNT_SET_FMT, self.valid,
                           self.roll_rad, self.pitch_rad)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdAttmountSet":
        _check_len("CMD_ATTMOUNT_SET", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_ATTMOUNT_SET_FMT, data))


@dataclass
class CmdYawzeroSet:
    """0x1B CMD_YAWZERO_SET(5B)— ヨーゼロオフセット設定。valid=0 でクリア。"""
    valid: int = 0
    offset_rad: float = 0.0

    TYPE = MsgType.CMD_YAWZERO_SET
    PAYLOAD_SIZE = 5

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_YAWZERO_SET_FMT, self.valid, self.offset_rad)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdYawzeroSet":
        _check_len("CMD_YAWZERO_SET", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_YAWZERO_SET_FMT, data))


@dataclass
class CmdGeomagSet:
    """0x1C CMD_GEOMAG_SET(20B)— 地磁気プロファイル設定(NVS 永続化)。"""
    declination_east_deg: float = 0.0   # 偏角(東向き正)[deg]
    inclination_deg: float = 0.0        # 伏角 [deg]
    horizontal_ut: float = 0.0          # 水平分力 [µT]
    vertical_ut: float = 0.0            # 鉛直分力 [µT]
    total_ut: float = 0.0               # 全磁力 [µT]

    TYPE = MsgType.CMD_GEOMAG_SET
    PAYLOAD_SIZE = 20

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_GEOMAG_SET_FMT,
                           self.declination_east_deg, self.inclination_deg,
                           self.horizontal_ut, self.vertical_ut, self.total_ut)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdGeomagSet":
        _check_len("CMD_GEOMAG_SET", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_GEOMAG_SET_FMT, data))


@dataclass
class CmdFfBegin:
    """0x1D CMD_FF_BEGIN(1B)— FF 係数ステージング開始(nlut は 4–24)。"""
    nlut: int = 0

    TYPE = MsgType.CMD_FF_BEGIN
    PAYLOAD_SIZE = 1
    NLUT_MIN = 4
    NLUT_MAX = 24

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_FF_BEGIN_FMT, self.nlut)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdFfBegin":
        _check_len("CMD_FF_BEGIN", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_FF_BEGIN_FMT, data))


@dataclass
class CmdFfLut:
    """0x1E CMD_FF_LUT(17B)— FF LUT 点(電流 → 磁気補正ベクトル)。"""
    idx: int = 0         # LUT インデックス(0 <= idx < nlut)
    i_a: float = 0.0     # 電流 [A]
    db_x: float = 0.0    # 磁気補正 x [µT]
    db_y: float = 0.0
    db_z: float = 0.0

    TYPE = MsgType.CMD_FF_LUT
    PAYLOAD_SIZE = 17

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_FF_LUT_FMT, self.idx, self.i_a,
                           self.db_x, self.db_y, self.db_z)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdFfLut":
        _check_len("CMD_FF_LUT", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_FF_LUT_FMT, data))


@dataclass
class CmdFfMot:
    """0x1F CMD_FF_MOT(25B)— FF モーター係数(idx: 0=FL, 1=FR, 2=RL, 3=RR)。"""
    idx: int = 0
    a_tilde: tuple = (0.0, 0.0, 0.0)   # 単位差分磁気ベクトル
    c2: float = 0.0                    # duty->電流 2次係数
    c1: float = 0.0
    c0: float = 0.0

    TYPE = MsgType.CMD_FF_MOT
    PAYLOAD_SIZE = 25
    MOTOR_FL = 0
    MOTOR_FR = 1
    MOTOR_RL = 2
    MOTOR_RR = 3

    def to_payload(self) -> bytes:
        if len(self.a_tilde) != 3:
            raise ValueError("CMD_FF_MOT: a_tilde は 3 要素")
        return struct.pack(_CMD_FF_MOT_FMT, self.idx, *self.a_tilde,
                           self.c2, self.c1, self.c0)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdFfMot":
        _check_len("CMD_FF_MOT", data, cls.PAYLOAD_SIZE)
        vals = struct.unpack(_CMD_FF_MOT_FMT, data)
        return cls(idx=vals[0], a_tilde=tuple(vals[1:4]),
                   c2=vals[4], c1=vals[5], c0=vals[6])


@dataclass
class CmdFfAux:
    """0x20 CMD_FF_AUX(4B)— ベンチ参考アイドル電流。"""
    iid_a: float = 0.0   # [A]

    TYPE = MsgType.CMD_FF_AUX
    PAYLOAD_SIZE = 4

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_FF_AUX_FMT, self.iid_a)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdFfAux":
        _check_len("CMD_FF_AUX", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_FF_AUX_FMT, data))


@dataclass
class CmdFfCommit:
    """0x21 CMD_FF_COMMIT(4B)— CRC-32(IEEE, zlib 互換, float32 LE 連結)照合
    → NVS 永続化。冪等。"""
    crc32: int = 0

    TYPE = MsgType.CMD_FF_COMMIT
    PAYLOAD_SIZE = 4

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_FF_COMMIT_FMT, self.crc32)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdFfCommit":
        _check_len("CMD_FF_COMMIT", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_FF_COMMIT_FMT, data))


@dataclass
class CmdFfMode:
    """0x22 CMD_FF_MODE(2B)— ff_mode / est_mode の実行時切替(NVS 永続化)。

    est_mode の受理範囲は 0..2(2=EKF2、MAG_AUTOTUNE_DESIGN.md §1.3。
    NVS `ffcal/est` に 2 を保存可)。切替時の再シード規約は既存と同じ
    (EKF2 はアクティブ化時、直近のアクティブ推定器ヨーで reseedYaw)。
    """
    ff_mode: int = 0    # 0=off, 1=A, 2=B
    est_mode: int = 0   # 0=補正相補フィルタ, 1=EKF, 2=EKF2(ヨー擬似観測付き)

    TYPE = MsgType.CMD_FF_MODE
    PAYLOAD_SIZE = 2
    FF_MODE_OFF = 0
    FF_MODE_A = 1
    FF_MODE_B = 2
    EST_MODE_COMPLEMENTARY = 0
    EST_MODE_EKF = 1
    EST_MODE_EKF2 = 2

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_FF_MODE_FMT, self.ff_mode, self.est_mode)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdFfMode":
        _check_len("CMD_FF_MODE", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_FF_MODE_FMT, data))


@dataclass
class CmdLedMode:
    """0x25 CMD_LED_MODE(1B)— LED 表示モード切替(計測中インジケータ)。

    mode=1(RECORDING)で MOTOR_TEST 中のみ LED をマゼンタ常灯にする
    (スマホ動画のカット点検出用。「マゼンタに変わった瞬間 = 計測開始」)。
    フェイルセーフ: 最後の mode=1 受信から 3 秒で AUTO へ自動復帰するため、
    PC は計測中およそ 1 秒間隔で再送する(キープアライブ)。MOTOR_TEST
    離脱でも即 AUTO。受理状態はキャリブ系と同じ(WAIT/COMPLETE/MOTOR_TEST)。
    """
    mode: int = 0

    TYPE = MsgType.CMD_LED_MODE
    PAYLOAD_SIZE = 1
    MODE_AUTO = 0       # 通常の状態表示
    MODE_RECORDING = 1  # 計測中(マゼンタ常灯、MOTOR_TEST 中のみ有効)

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_LED_MODE_FMT, self.mode)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdLedMode":
        _check_len("CMD_LED_MODE", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_LED_MODE_FMT, data))


@dataclass
class CmdMagbiasSet:
    """0x26 CMD_MAGBIAS_SET(13B)— 学習ハードアイアン残差 Δb の適用/クリア。

    Δb [µT, b_cal 空間] を NVS `magbias` へ保存し即適用する(mode=0 でクリア)。
    **ff_mode は降格しない**(FF 係数は b_cal 空間のまま有効)。適用時:
    アンカー無効化+窓リセット+補正系再シード(mag3d 変更時と同パターン、
    ff_mode 降格だけしない)。mag3d 変更時は旧 b_cal 空間で無効になるため
    機体側で連動クリアされる。WAIT / COMPLETE / MOTOR_TEST 状態でのみ受理
    (飛行中 NVS 書込み禁止。TLM_ACK 応答)。MAG_AUTOTUNE_DESIGN.md §1.4。
    """
    mode: int = 0    # 0=clear, 1=set
    dx: float = 0.0  # µT(b_cal 空間の Δb x)
    dy: float = 0.0  # µT(同 y)
    dz: float = 0.0  # µT(同 z)

    TYPE = MsgType.CMD_MAGBIAS_SET
    PAYLOAD_SIZE = 13
    MODE_CLEAR = 0
    MODE_SET = 1

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_MAGBIAS_SET_FMT, self.mode,
                           self.dx, self.dy, self.dz)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdMagbiasSet":
        _check_len("CMD_MAGBIAS_SET", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_MAGBIAS_SET_FMT, data))


@dataclass
class CmdFlowcalSet:
    """0x27 CMD_FLOWCAL_SET(17B)— フロー較正行列 K の適用/クリア。

    K [counts/rad] は「機体系レート [rad/s] → センサーカウントレート
    [counts/s]」の 2×2 写像(行優先 m00,m01,m10,m11)。純スケールなら
    K=diag(kx,ky)、取付回転 φ0 込みなら K=diag(kx,ky)·R(φ0)。ファームは
    rate = K⁻¹·counts_rate をジャイロ補償より前に適用し、NVS `flowcal`
    (schema v2)へ保存する(mode=0 でクリア=既定 diag(450,450) に戻る)。
    WAIT / COMPLETE / MOTOR_TEST 状態でのみ受理(TLM_ACK 応答)。
    MAG_AUTOTUNE_DESIGN.md §1.4。

    【改訂 2026-07-31】9B(u8 mode + f32 kx,ky)→ 17B の 2×2 行列化
    (FLIGHT_ANALYSIS_20260731.md §5: 取付回転 φ0 を 2 定数では表現
    できないため)。
    """
    mode: int = 0     # 0=clear, 1=set
    m00: float = 0.0  # K 行列 [counts/rad](行優先)
    m01: float = 0.0
    m10: float = 0.0
    m11: float = 0.0

    TYPE = MsgType.CMD_FLOWCAL_SET
    PAYLOAD_SIZE = 17
    MODE_CLEAR = 0
    MODE_SET = 1

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_FLOWCAL_SET_FMT, self.mode,
                           self.m00, self.m01, self.m10, self.m11)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdFlowcalSet":
        _check_len("CMD_FLOWCAL_SET", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_FLOWCAL_SET_FMT, data))


@dataclass
class CmdFlowProbe:
    """0x28 CMD_FLOW_PROBE(1B)— PMW3901 burst プローブ実行。

    モーター停止時のみ(回転中は status=busy で拒否)。結果は LOG_TEXT
    複数行(agree_ratio 等の要約)で返し、probe 成功で burst_ok ラッチを
    更新する。n_cycles=0 は既定 200 サイクル。WAIT / COMPLETE / MOTOR_TEST
    状態でのみ受理(TLM_ACK 応答)。MAG_AUTOTUNE_DESIGN.md §1.4。
    """
    n_cycles: int = 0  # プローブサイクル数(0=既定 200)

    TYPE = MsgType.CMD_FLOW_PROBE
    PAYLOAD_SIZE = 1
    DEFAULT_CYCLES = 200

    def to_payload(self) -> bytes:
        return struct.pack(_CMD_FLOW_PROBE_FMT, self.n_cycles)

    @classmethod
    def from_payload(cls, data: bytes) -> "CmdFlowProbe":
        _check_len("CMD_FLOW_PROBE", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_CMD_FLOW_PROBE_FMT, data))


@dataclass
class TlmState:
    """0x30 TLM_STATE(184B)— フル状態テレメトリ(25Hz)。

    v2: 末尾追加のみ(既存オフセット 0–96 は v1 と不変。serial_link.py が
    seq_echo を先頭オフセット直読みするため末尾追加限定)。
    磁気オートチューン/フロー拡張(MAG_AUTOTUNE_DESIGN.md §1.1):
    オフセット 135 以降を末尾追加して 135B → 184B。長さ検査は厳格のまま
    (旧 135B フレームは受理しない)。
    """
    seq_echo: int = 0        # 最後に適用した CMD_SETPOINT / CMD_POS_ERR の seq(未受信なら0)
    elapsed_ms: int = 0      # 起動からの経過 [ms]
    state: int = 0           # FlightState
    flags: int = 0           # bit0 low_voltage, bit1 setpoint_fresh(<200ms), bit2 flying
    reason: int = 0          # Reason(直近の遷移理由)
    roll: float = 0.0        # rad(実測姿勢, AHRS)
    pitch: float = 0.0       # rad
    yaw: float = 0.0         # rad
    p: float = 0.0           # rad/s(実測角速度)
    q: float = 0.0           # rad/s
    r: float = 0.0           # rad/s
    roll_ref: float = 0.0    # rad(適用中の指令)
    pitch_ref: float = 0.0   # rad
    alt_ref: float = 0.0     # m(適用中の目標高度)
    altitude_tof: float = 0.0  # m(ToF生値)
    altitude_est: float = 0.0  # m(カルマン推定)
    alt_velocity: float = 0.0  # m/s
    z_dot_ref: float = 0.0     # m/s
    voltage: float = 0.0       # V
    duty_fr: float = 0.0       # 0–1
    duty_fl: float = 0.0
    duty_rr: float = 0.0
    duty_rl: float = 0.0
    ax: float = 0.0            # g(フィルタ後加速度)
    ay: float = 0.0
    az: float = 0.0
    loop_dt_us: int = 0        # µs(直近の実測制御周期)
    # --- v2 追加(オフセット 97 以降) ---
    yaw_est_rad: float = 0.0       # rad(アクティブ推定器ヨー。est_mode=1 なら EKF ψ)
    yaw_gyro_int_rad: float = 0.0  # rad(Z軸角速度の単純積算 400Hz、ahrs_reset でゼロ)
    yaw_ref_rad: float = 0.0       # rad(適用中ヨー目標。ラッチ後含む。制御 off 時 0)
    current_a: float = 0.0         # A(総電流、20Hz 更新)
    db_hat_x_ut: float = 0.0       # µT(FF 補正ベクトル x)
    db_hat_y_ut: float = 0.0       # µT(同 y)
    bm_x_ut: float = 0.0           # µT(EKF 磁気バイアス状態 x)
    bm_y_ut: float = 0.0           # µT(同 y)
    nis: float = 0.0               # 直近 EKF 更新の NIS
    ffg: int = 0                   # EKF ゲート/健全性ビット(yaw側 ffg 定義踏襲)
    ff_status: int = 0             # FF_STATUS_* ビット
    # --- 磁気オートチューン/フロー拡張(オフセット 135 以降。契約 §1.1)---
    mag_cal_x_ut: float = 0.0      # µT(b_cal = mag3d 適用後・FF 補正前、機体系 x)
    mag_cal_y_ut: float = 0.0      # µT(同 y)
    mag_cal_z_ut: float = 0.0      # µT(同 z)
    mag_lev_x_ut: float = 0.0      # µT(FF補正+magbias+EMA後レベル化水平 x = EKF観測 zx)
    mag_lev_y_ut: float = 0.0      # µT(同 y = zy)
    ekf2_yaw_rad: float = 0.0      # rad(EKF2 の ψ。シャドー中も常時)
    ekf2_bm_x_ut: float = 0.0      # µT(EKF2 の b_mx)
    ekf2_bm_y_ut: float = 0.0      # µT(同 b_my)
    ekf2_yaw_innov_rad: float = 0.0  # rad(直近ヨー観測イノベーション。未受信時 0)
    ekf2_status: int = 0           # EKF2_STATUS_* ビット
    ekf2_gate: int = 0             # EKF2 のゲートビット(ffg と同一ビット定義)
    flow_vx_mps: float = 0.0       # m/s(フロー機体系 x 速度。無効時 0)
    flow_vy_mps: float = 0.0       # m/s(同 y)
    flow_squal: int = 0            # SQUAL 生値
    flow_status: int = 0           # FLOW_STATUS_* ビット
    flow_dt_ms: int = 0            # ms(フロー読み実測 dt。0-255 クランプ)

    TYPE = MsgType.TLM_STATE
    PAYLOAD_SIZE = 184
    FLAG_LOW_VOLTAGE = 0x01
    FLAG_SETPOINT_FRESH = 0x02
    FLAG_FLYING = 0x04
    # ff_status ビット定義(v2)
    FF_STATUS_FF_MODE_MASK = 0x03     # bit0-1: ff_mode(0-2)
    FF_STATUS_EST_EKF = 0x04          # bit2: est_mode==1(EKF)
    FF_STATUS_ANCHOR_VALID = 0x08     # bit3
    FF_STATUS_FFCAL_LOADED = 0x10     # bit4
    FF_STATUS_YAW_CTRL_ACTIVE = 0x20  # bit5
    FF_STATUS_MAG_FRESH = 0x40        # bit6
    FF_STATUS_EST_EKF2 = 0x80         # bit7: est_mode==2(EKF2。bit2/bit7 とも 0 なら相補CF)
    # ekf2_status ビット定義(契約 §1.1)
    EKF2_STATUS_YAW_OBS_FRESH = 0x01      # bit0: ヨー観測受信 <1s
    EKF2_STATUS_YAW_OBS_FUSED = 0x02      # bit1: 直近 0.5s 内に受理
    EKF2_STATUS_FLIGHT_ANCHOR_DONE = 0x04 # bit2: 飛行状態再アンカー実施済み
    EKF2_STATUS_TAU_RW_MODE = 0x08        # bit3: q_bm ランダムウォークモード
    EKF2_STATUS_BM_FROZEN = 0x10          # bit4
    EKF2_STATUS_HEALTHY2 = 0x20           # bit5
    EKF2_STATUS_YAW_OBS_LOW_TRUST = 0x40  # bit6
    EKF2_STATUS_YAW_RECAPTURE = 0x80      # bit7: ヨー観測再捕捉中(制限融合モード)
    # flow_status ビット定義(契約 §1.1)
    FLOW_STATUS_SENSOR_OK = 0x01    # bit0
    FLOW_STATUS_BURST_OK = 0x02     # bit1
    FLOW_STATUS_VEL_VALID = 0x04    # bit2
    FLOW_STATUS_RANGE_VALID = 0x08  # bit3
    FLOW_STATUS_SQUAL_OK = 0x10     # bit4
    FLOW_STATUS_INIT_RETRY = 0x20   # bit5
    # bit6-7 予約

    def to_payload(self) -> bytes:
        return struct.pack(
            _TLM_STATE_FMT,
            self.seq_echo, self.elapsed_ms, self.state, self.flags, self.reason,
            self.roll, self.pitch, self.yaw,
            self.p, self.q, self.r,
            self.roll_ref, self.pitch_ref, self.alt_ref,
            self.altitude_tof, self.altitude_est,
            self.alt_velocity, self.z_dot_ref, self.voltage,
            self.duty_fr, self.duty_fl, self.duty_rr, self.duty_rl,
            self.ax, self.ay, self.az,
            self.loop_dt_us,
            self.yaw_est_rad, self.yaw_gyro_int_rad, self.yaw_ref_rad,
            self.current_a,
            self.db_hat_x_ut, self.db_hat_y_ut,
            self.bm_x_ut, self.bm_y_ut,
            self.nis,
            self.ffg, self.ff_status,
            self.mag_cal_x_ut, self.mag_cal_y_ut, self.mag_cal_z_ut,
            self.mag_lev_x_ut, self.mag_lev_y_ut,
            self.ekf2_yaw_rad,
            self.ekf2_bm_x_ut, self.ekf2_bm_y_ut,
            self.ekf2_yaw_innov_rad,
            self.ekf2_status, self.ekf2_gate,
            self.flow_vx_mps, self.flow_vy_mps,
            self.flow_squal, self.flow_status, self.flow_dt_ms)

    @classmethod
    def from_payload(cls, data: bytes) -> "TlmState":
        _check_len("TLM_STATE", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_TLM_STATE_FMT, data))


@dataclass
class TlmEvent:
    """0x31 TLM_EVENT(8B)— 状態遷移時に即時送信+2Hzで定期再送。"""
    state: int = 0           # FlightState
    prev_state: int = 0      # FlightState
    reason: int = 0          # Reason
    flags: int = 0
    voltage: float = 0.0     # V

    TYPE = MsgType.TLM_EVENT
    PAYLOAD_SIZE = 8

    def to_payload(self) -> bytes:
        return struct.pack(_TLM_EVENT_FMT, self.state, self.prev_state,
                           self.reason, self.flags, self.voltage)

    @classmethod
    def from_payload(cls, data: bytes) -> "TlmEvent":
        _check_len("TLM_EVENT", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_TLM_EVENT_FMT, data))


@dataclass
class TlmAck:
    """0x32 TLM_ACK(6B)— 0x14–0x23, 0x25–0x28 コマンドへの応答。"""
    acked_type: int = 0   # 応答対象のメッセージ型
    acked_seq: int = 0    # 応答対象フレームの seq
    status: int = 0       # STATUS_*

    TYPE = MsgType.TLM_ACK
    PAYLOAD_SIZE = 6
    STATUS_OK = 0
    STATUS_BAD_STATE = 1
    STATUS_INVALID_ARG = 2
    STATUS_CRC_MISMATCH = 3
    STATUS_BUSY = 4
    STATUS_INCOMPLETE = 5

    def to_payload(self) -> bytes:
        return struct.pack(_TLM_ACK_FMT, self.acked_type, self.acked_seq,
                           self.status)

    @classmethod
    def from_payload(cls, data: bytes) -> "TlmAck":
        _check_len("TLM_ACK", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_TLM_ACK_FMT, data))


@dataclass
class TlmExp:
    """0x33 TLM_EXP(86B)— 実験テレメトリ。MOTOR_TEST 状態でのみ 25Hz 送出
    (TLM_STATE と 8tick 位相をずらす)。"""
    elapsed_ms: int = 0      # 起動からの経過 [ms]
    current_a: float = 0.0   # A(INA3221 CH2 総電流)
    vbat_v: float = 0.0      # V
    shunt_uv: float = 0.0    # µV
    bx_raw: float = 0.0      # µT(RHALL補償+軸変換後・mag3D 前)
    by_raw: float = 0.0
    bz_raw: float = 0.0
    bx_cal: float = 0.0      # µT(mag3D 後)
    by_cal: float = 0.0
    bz_cal: float = 0.0
    imu_temp_c: float = 0.0  # ℃
    roll: float = 0.0        # rad(Madgwick)
    pitch: float = 0.0
    yaw: float = 0.0
    p: float = 0.0           # rad/s
    q: float = 0.0
    r: float = 0.0
    ax: float = 0.0          # g(フィルタ後)
    ay: float = 0.0
    az: float = 0.0
    duty_cmd: float = 0.0    # モーターテスト指令 duty(0–1)
    motors_mask: int = 0     # CmdMotorRun.MASK_* と同ビット割り
    flags: int = 0           # FLAG_*

    TYPE = MsgType.TLM_EXP
    PAYLOAD_SIZE = 86
    FLAG_CURRENT_VALID = 0x01
    FLAG_MAG_FRESH = 0x02
    FLAG_MOTORS_RUNNING = 0x04

    def to_payload(self) -> bytes:
        return struct.pack(
            _TLM_EXP_FMT,
            self.elapsed_ms,
            self.current_a, self.vbat_v, self.shunt_uv,
            self.bx_raw, self.by_raw, self.bz_raw,
            self.bx_cal, self.by_cal, self.bz_cal,
            self.imu_temp_c,
            self.roll, self.pitch, self.yaw,
            self.p, self.q, self.r,
            self.ax, self.ay, self.az,
            self.duty_cmd,
            self.motors_mask, self.flags)

    @classmethod
    def from_payload(cls, data: bytes) -> "TlmExp":
        _check_len("TLM_EXP", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_TLM_EXP_FMT, data))


@dataclass
class TlmCalData:
    """0x34 TLM_CAL_DATA(140B)— CMD_CAL_GET への応答(キャリブ一括データ)。

    磁気オートチューン/フロー拡張(MAG_AUTOTUNE_DESIGN.md §1.5):
    magbias(Δb [µT])と flowcal(K 行列 [counts/rad])を末尾追加して
    112B → 140B。valid_flags bit6=magbias 有効、bit7=flowcal 有効。

    【改訂 2026-07-31】flowcal 2×2 行列化で 132B → 140B: 既存オフセット
    124/128 の kx/ky は flowcal_m00/flowcal_m11 に改名(対角成分として意味
    維持)、flowcal_m01@132・flowcal_m10@136 を末尾追加。ワイヤ順は
    m00, m11, m01, m10(オフセット互換維持のため行優先ではない)。
    """
    valid_flags: int = 0
    mag3d_offset: tuple = (0.0, 0.0, 0.0)
    mag3d_matrix: tuple = (0.0,) * 9   # 行優先
    accel6_offset: tuple = (0.0, 0.0, 0.0)
    accel6_scale: tuple = (0.0, 0.0, 0.0)
    attmount_roll_rad: float = 0.0
    attmount_pitch_rad: float = 0.0
    yawzero_offset_rad: float = 0.0
    geomag: tuple = (0.0,) * 5         # decl_east_deg, incl_deg, H_uT, V_uT, F_uT
    ff_nlut: int = 0
    ff_crc32: int = 0
    ff_mode: int = 0
    est_mode: int = 0
    magbias: tuple = (0.0, 0.0, 0.0)   # Δb dx, dy, dz [µT, b_cal 空間]
    # flowcal K 行列 [counts/rad](機体系レート→カウントレート)。ワイヤ順
    # m00@124, m11@128, m01@132, m10@136(オフセット互換維持)。
    flowcal_m00: float = 0.0
    flowcal_m11: float = 0.0
    flowcal_m01: float = 0.0
    flowcal_m10: float = 0.0

    TYPE = MsgType.TLM_CAL_DATA
    PAYLOAD_SIZE = 140
    VALID_MAG3D = 0x01
    VALID_ACCEL6 = 0x02
    VALID_ATTMOUNT = 0x04
    VALID_YAWZERO = 0x08
    VALID_GEOMAG = 0x10
    VALID_FFCAL = 0x20
    VALID_MAGBIAS = 0x40   # bit6: magbias 有効(契約 §1.5)
    VALID_FLOWCAL = 0x80   # bit7: flowcal 有効(同上)

    def to_payload(self) -> bytes:
        if (len(self.mag3d_offset) != 3 or len(self.mag3d_matrix) != 9 or
                len(self.accel6_offset) != 3 or len(self.accel6_scale) != 3 or
                len(self.geomag) != 5 or len(self.magbias) != 3):
            raise ValueError("TLM_CAL_DATA: 配列フィールドの要素数が不正")
        return struct.pack(
            _TLM_CAL_DATA_FMT,
            self.valid_flags,
            *self.mag3d_offset, *self.mag3d_matrix,
            *self.accel6_offset, *self.accel6_scale,
            self.attmount_roll_rad, self.attmount_pitch_rad,
            self.yawzero_offset_rad,
            *self.geomag,
            self.ff_nlut, self.ff_crc32, self.ff_mode, self.est_mode,
            *self.magbias,
            self.flowcal_m00, self.flowcal_m11,
            self.flowcal_m01, self.flowcal_m10)

    @classmethod
    def from_payload(cls, data: bytes) -> "TlmCalData":
        _check_len("TLM_CAL_DATA", data, cls.PAYLOAD_SIZE)
        vals = struct.unpack(_TLM_CAL_DATA_FMT, data)
        return cls(
            valid_flags=vals[0],
            mag3d_offset=tuple(vals[1:4]),
            mag3d_matrix=tuple(vals[4:13]),
            accel6_offset=tuple(vals[13:16]),
            accel6_scale=tuple(vals[16:19]),
            attmount_roll_rad=vals[19],
            attmount_pitch_rad=vals[20],
            yawzero_offset_rad=vals[21],
            geomag=tuple(vals[22:27]),
            ff_nlut=vals[27],
            ff_crc32=vals[28],
            ff_mode=vals[29],
            est_mode=vals[30],
            magbias=tuple(vals[31:34]),
            flowcal_m00=vals[34],
            flowcal_m11=vals[35],
            flowcal_m01=vals[36],
            flowcal_m10=vals[37])


@dataclass
class TlmCtrl:
    """0x35 TLM_CTRL(89B)— 制御ループ診断テレメトリ。25Hz(400Hzループの
    16分周、TLM_STATE と 4tick 位相をずらす)で全飛行状態(MOTOR_TEST 中も
    含む)常時送出。

    PID 成分は PID::update() の合成式 m_kp*(err + m_integral + m_differential)
    の3分解(P=kp*err、I=kp*m_integral、D=kp*m_differential。P+I+D=そのPIDの
    出力)。yaw の角度ループ成分はクランプ前の値を記録する(yaw_rate_ref は
    クランプ後 → 差でクランプ発動が分かる)。PID リセット中(非飛行時や、
    ヨー制御OFF時の psi_pid 毎tickリセット等)は成分が 0 になるため、flags で
    有効区間を判別する。
    """
    elapsed_ms: int = 0           # 起動からの経過 [ms](TLM_STATE と同一クロック)
    roll_rate_ref: float = 0.0    # rad/s(角度ループ出力=ロール指令角速度)
    pitch_rate_ref: float = 0.0   # rad/s(同ピッチ)
    yaw_rate_ref: float = 0.0     # rad/s(psi_pid 出力。±yaw_rate_limit_rad_s クランプ後)
    pid_ang: tuple = (0.0,) * 9   # 角度ループPID成分(roll_p,i,d, pitch_p,i,d, yaw_p,i,d)
    pid_rate: tuple = (0.0,) * 9  # 角速度ループPID成分(同順。roll=p_pid, pitch=q_pid, yaw=r_pid)
    flags: int = 0                # FLAG_*

    TYPE = MsgType.TLM_CTRL
    PAYLOAD_SIZE = 89
    FLAG_XY_ONBOARD_ACTIVE = 0x01  # bit0: CMD_POS_ERR 経路で機上XY指令生成中
    FLAG_YAW_CTRL_ACTIVE = 0x02    # bit1
    FLAG_FLYING = 0x04             # bit2

    def to_payload(self) -> bytes:
        if len(self.pid_ang) != 9 or len(self.pid_rate) != 9:
            raise ValueError("TLM_CTRL: pid_ang / pid_rate は各 9 要素")
        return struct.pack(
            _TLM_CTRL_FMT,
            self.elapsed_ms,
            self.roll_rate_ref, self.pitch_rate_ref, self.yaw_rate_ref,
            *self.pid_ang, *self.pid_rate,
            self.flags)

    @classmethod
    def from_payload(cls, data: bytes) -> "TlmCtrl":
        _check_len("TLM_CTRL", data, cls.PAYLOAD_SIZE)
        vals = struct.unpack(_TLM_CTRL_FMT, data)
        return cls(
            elapsed_ms=vals[0],
            roll_rate_ref=vals[1],
            pitch_rate_ref=vals[2],
            yaw_rate_ref=vals[3],
            pid_ang=tuple(vals[4:13]),
            pid_rate=tuple(vals[13:22]),
            flags=vals[22])


@dataclass
class LogText:
    """0x40 LOG_TEXT(1〜181B)— 人間向けメッセージ。生テキスト直書きの代替。"""
    origin: int = 0          # 0=relay, 1=drone
    text: str = ""           # UTF-8 で 180B 以下

    TYPE = MsgType.LOG_TEXT
    ORIGIN_RELAY = 0
    ORIGIN_DRONE = 1

    def to_payload(self) -> bytes:
        encoded = self.text.encode("utf-8")
        if len(encoded) > MAX_LOG_TEXT_SIZE:
            raise ValueError(f"LOG_TEXT: text too long ({len(encoded)}B > {MAX_LOG_TEXT_SIZE}B)")
        if not 0 <= self.origin <= 0xFF:
            raise ValueError(f"LOG_TEXT: origin out of range: {self.origin}")
        return bytes([self.origin]) + encoded

    @classmethod
    def from_payload(cls, data: bytes) -> "LogText":
        if not 1 <= len(data) <= 1 + MAX_LOG_TEXT_SIZE:
            raise ValueError(f"LOG_TEXT: invalid payload length {len(data)}")
        # 表示用途のため、不正な UTF-8 は U+FFFD に置換して受理する
        return cls(origin=data[0], text=bytes(data[1:]).decode("utf-8", errors="replace"))


def utf8_truncate_len(text: bytes, max_len: int) -> int:
    """UTF-8 文字境界を保ったまま text を max_len バイト以下に切り詰めた長さを返す。

    PROTOCOL.md LOG_TEXT: 多バイト文字を分断しない。切り詰め位置が多バイト文字の
    途中になる場合は、その文字の先頭(リード)バイトまで遡って文字ごと落とす。
    末尾がすでに不正な UTF-8 の場合は新たな分断を作らないことのみ保証する
    (C++ 実装 ``stampfly::utf8_truncate_len`` と test_vectors.json で一致を強制)。
    """
    cut = min(len(text), max_len)
    if cut <= 0:
        return 0
    # 末尾文字のリードバイトを探す(0b10xxxxxx = 継続バイトを遡る)
    lead = cut - 1
    while lead > 0 and (text[lead] & 0xC0) == 0x80:
        lead -= 1
    b = text[lead]
    if b < 0x80:                 # ASCII
        expect = 1
    elif (b & 0xE0) == 0xC0:     # 110xxxxx
        expect = 2
    elif (b & 0xF0) == 0xE0:     # 1110xxxx
        expect = 3
    elif (b & 0xF8) == 0xF0:     # 11110xxx
        expect = 4
    else:
        return cut               # 孤立継続バイト等の不正列: 分断回避の対象外
    return cut if lead + expect <= cut else lead


@dataclass
class RlySetTarget:
    """0x50 RLY_SET_TARGET(7B)— ESP-NOW ピア設定。"""
    mac: bytes = b"\x00" * 6
    wifi_channel: int = 0    # 1-13

    TYPE = MsgType.RLY_SET_TARGET
    PAYLOAD_SIZE = 7

    def to_payload(self) -> bytes:
        if len(self.mac) != 6:
            raise ValueError(f"RLY_SET_TARGET: mac must be 6 bytes, got {len(self.mac)}")
        return struct.pack(_RLY_SET_TARGET_FMT, bytes(self.mac), self.wifi_channel)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlySetTarget":
        _check_len("RLY_SET_TARGET", data, cls.PAYLOAD_SIZE)
        mac, channel = struct.unpack(_RLY_SET_TARGET_FMT, data)
        return cls(mac=mac, wifi_channel=channel)


@dataclass
class RlyTargetAck:
    """0x51 RLY_TARGET_ACK(8B)— SET_TARGET への応答。"""
    status: int = 0          # 0=ok, 1=invalid_mac, 2=peer_failed
    mac: bytes = b"\x00" * 6
    channel: int = 0

    TYPE = MsgType.RLY_TARGET_ACK
    PAYLOAD_SIZE = 8
    STATUS_OK = 0
    STATUS_INVALID_MAC = 1
    STATUS_PEER_FAILED = 2

    def to_payload(self) -> bytes:
        if len(self.mac) != 6:
            raise ValueError(f"RLY_TARGET_ACK: mac must be 6 bytes, got {len(self.mac)}")
        return struct.pack(_RLY_TARGET_ACK_FMT, self.status, bytes(self.mac), self.channel)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyTargetAck":
        _check_len("RLY_TARGET_ACK", data, cls.PAYLOAD_SIZE)
        status, mac, channel = struct.unpack(_RLY_TARGET_ACK_FMT, data)
        return cls(status=status, mac=mac, channel=channel)


@dataclass
class RlyStats:
    """0x52 RLY_STATS(24B)— リレー統計(1Hz自動送信)。"""
    up_frames: int = 0
    down_frames: int = 0
    crc_errors: int = 0
    cobs_errors: int = 0
    espnow_send_fail: int = 0
    overflow_drops: int = 0

    TYPE = MsgType.RLY_STATS
    PAYLOAD_SIZE = 24

    def to_payload(self) -> bytes:
        return struct.pack(_RLY_STATS_FMT, self.up_frames, self.down_frames,
                           self.crc_errors, self.cobs_errors,
                           self.espnow_send_fail, self.overflow_drops)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyStats":
        _check_len("RLY_STATS", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_RLY_STATS_FMT, data))


@dataclass
class RlyPong:
    """0x54 RLY_PONG(4B)— RLY_PING 応答(PING の seq をエコー)。"""
    echo_seq: int = 0

    TYPE = MsgType.RLY_PONG
    PAYLOAD_SIZE = 4

    def to_payload(self) -> bytes:
        return struct.pack(_RLY_PONG_FMT, self.echo_seq)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyPong":
        _check_len("RLY_PONG", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_RLY_PONG_FMT, data))


# ---------------------------------------------------------------------------
# マルチ機体拡張(RLY_SET_PEERS / RLY_PEERS_ACK / RLY_MUX_UP / RLY_MUX_DOWN)
#
# エンベロープ(MUX)はシリアル区間のみに存在する。リレーは MUX_UP から
# 内側フレームを取り出して peers[node_id] へそのまま ESP-NOW 送信し、
# ピア発の下りフレームを MUX_DOWN で包んで PC へ送る。ESP-NOW 区間の
# バイト列は単機時と完全に同一のため、機体ファームウェアは無改修。
# ---------------------------------------------------------------------------

@dataclass
class RlyPeer:
    """RLY_SET_PEERS の1エントリ(7B)。"""
    mac: bytes = b"\x00" * 6
    tlm_state_div: int = 1   # TLM_STATE 間引き(1=全転送, n=1/n 転送。0 は 1 扱い)


@dataclass
class RlySetPeers:
    """0x55 RLY_SET_PEERS(可変長 2+7×N B、N=0..4)— マルチ機体ピア設定。

    count=0 でマルチモード解除(ピア表クリア)。設定が受理されると単機
    ターゲット(RLY_SET_TARGET)は無効化される(逆も同様)。1つの無線は
    1チャネルのため、全ピアで wifi_channel を共有する。
    """
    wifi_channel: int = 0    # 1-13(count=0 のときは 0 を許容)
    peers: tuple = ()        # RlyPeer の並び。index がそのまま node_id になる

    TYPE = MsgType.RLY_SET_PEERS
    MIN_PAYLOAD_SIZE = 2
    MAX_PAYLOAD_SIZE = 2 + RLY_PEER_ENTRY_SIZE * RLY_MAX_PEERS   # = 30

    def to_payload(self) -> bytes:
        if len(self.peers) > RLY_MAX_PEERS:
            raise ValueError(
                f"RLY_SET_PEERS: too many peers: {len(self.peers)} > {RLY_MAX_PEERS}")
        out = bytearray([len(self.peers), self.wifi_channel])
        for peer in self.peers:
            if len(peer.mac) != 6:
                raise ValueError(
                    f"RLY_SET_PEERS: mac must be 6 bytes, got {len(peer.mac)}")
            out += bytes(peer.mac)
            out.append(peer.tlm_state_div)
        return bytes(out)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlySetPeers":
        if len(data) < cls.MIN_PAYLOAD_SIZE:
            raise ValueError(f"RLY_SET_PEERS: payload too short: {len(data)}")
        count = data[0]
        if count > RLY_MAX_PEERS:
            raise ValueError(f"RLY_SET_PEERS: count {count} > {RLY_MAX_PEERS}")
        expected = cls.MIN_PAYLOAD_SIZE + RLY_PEER_ENTRY_SIZE * count
        _check_len("RLY_SET_PEERS", data, expected)
        peers = []
        for i in range(count):
            off = cls.MIN_PAYLOAD_SIZE + RLY_PEER_ENTRY_SIZE * i
            peers.append(RlyPeer(mac=bytes(data[off:off + 6]),
                                 tlm_state_div=data[off + 6]))
        return cls(wifi_channel=data[1], peers=tuple(peers))


@dataclass
class RlyPeersAck:
    """0x56 RLY_PEERS_ACK(4B)— SET_PEERS への応答。"""
    status: int = 0
    count: int = 0           # 受理したピア数のエコー
    wifi_channel: int = 0
    failed_index: int = 0xFF  # 失敗したエントリの index(なければ FAILED_NONE)

    TYPE = MsgType.RLY_PEERS_ACK
    PAYLOAD_SIZE = 4
    STATUS_OK = 0
    STATUS_INVALID_MAC = 1
    STATUS_PEER_FAILED = 2
    STATUS_BAD_COUNT = 3
    STATUS_BAD_CHANNEL = 4
    FAILED_NONE = 0xFF

    def to_payload(self) -> bytes:
        return struct.pack(_RLY_PEERS_ACK_FMT, self.status, self.count,
                           self.wifi_channel, self.failed_index)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyPeersAck":
        _check_len("RLY_PEERS_ACK", data, cls.PAYLOAD_SIZE)
        return cls(*struct.unpack(_RLY_PEERS_ACK_FMT, data))


def mux_wrap(node_id: int, inner_frame: bytes) -> bytes:
    """MUX ペイロード(node_id + 内側論理フレーム)を構築する。"""
    if not 0 <= node_id < RLY_MAX_PEERS:
        raise ValueError(f"mux: node_id out of range: {node_id}")
    inner_frame = bytes(inner_frame)
    if len(inner_frame) < FRAME_OVERHEAD:
        raise ValueError(f"mux: inner frame too short: {len(inner_frame)}")
    if MUX_HEADER_SIZE + len(inner_frame) > MAX_PAYLOAD_SIZE:
        raise ValueError(f"mux: inner frame too long: {len(inner_frame)}")
    return bytes([node_id]) + inner_frame


def mux_unwrap(payload: bytes) -> tuple[int, bytes]:
    """MUX ペイロードを (node_id, 内側フレームバイト列) に分解する。

    内側フレームの CRC/構造検証は行わない(受信側が parse_frame で検証)。
    """
    if len(payload) < MUX_HEADER_SIZE + FRAME_OVERHEAD:
        raise ValueError(f"mux: payload too short: {len(payload)}")
    node_id = payload[0]
    if node_id >= RLY_MAX_PEERS:
        raise ValueError(f"mux: node_id out of range: {node_id}")
    return node_id, bytes(payload[MUX_HEADER_SIZE:])


@dataclass
class RlyMuxUp:
    """0x57 RLY_MUX_UP(1+内側フレーム長)— 機体 node_id 宛エンベロープ。"""
    node_id: int = 0
    inner: bytes = b""       # 完全な内側論理フレーム(ver..crc16)

    TYPE = MsgType.RLY_MUX_UP

    def to_payload(self) -> bytes:
        return mux_wrap(self.node_id, self.inner)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyMuxUp":
        node_id, inner = mux_unwrap(data)
        return cls(node_id=node_id, inner=inner)


@dataclass
class RlyMuxDown:
    """0x58 RLY_MUX_DOWN(1+内側フレーム長)— 機体 node_id 発エンベロープ。"""
    node_id: int = 0
    inner: bytes = b""       # 完全な内側論理フレーム(ver..crc16)

    TYPE = MsgType.RLY_MUX_DOWN

    def to_payload(self) -> bytes:
        return mux_wrap(self.node_id, self.inner)

    @classmethod
    def from_payload(cls, data: bytes) -> "RlyMuxDown":
        node_id, inner = mux_unwrap(data)
        return cls(node_id=node_id, inner=inner)


# ペイロードを持たない型(len は常に 0)
EMPTY_PAYLOAD_TYPES = frozenset({
    MsgType.CMD_START, MsgType.CMD_STOP, MsgType.CMD_RESET,
    MsgType.CMD_MOTOR_STOP, MsgType.CMD_CAL_GET, MsgType.CMD_FF_ANCHOR,
    MsgType.RLY_PING,
})

# 型 -> ペイロードクラス(可変長 LOG_TEXT を含む)
PAYLOAD_CLASSES = {
    MsgType.CMD_SETPOINT: CmdSetpoint,
    MsgType.CMD_POS_ERR: CmdPosErr,
    MsgType.CMD_MODE: CmdMode,
    MsgType.CMD_MOTOR_RUN: CmdMotorRun,
    MsgType.CMD_MAG3D_SET: CmdMag3dSet,
    MsgType.CMD_ACCEL6_SET: CmdAccel6Set,
    MsgType.CMD_ATTMOUNT_SET: CmdAttmountSet,
    MsgType.CMD_YAWZERO_SET: CmdYawzeroSet,
    MsgType.CMD_GEOMAG_SET: CmdGeomagSet,
    MsgType.CMD_FF_BEGIN: CmdFfBegin,
    MsgType.CMD_FF_LUT: CmdFfLut,
    MsgType.CMD_FF_MOT: CmdFfMot,
    MsgType.CMD_FF_AUX: CmdFfAux,
    MsgType.CMD_FF_COMMIT: CmdFfCommit,
    MsgType.CMD_FF_MODE: CmdFfMode,
    MsgType.CMD_LED_MODE: CmdLedMode,
    MsgType.CMD_MAGBIAS_SET: CmdMagbiasSet,
    MsgType.CMD_FLOWCAL_SET: CmdFlowcalSet,
    MsgType.CMD_FLOW_PROBE: CmdFlowProbe,
    MsgType.TLM_STATE: TlmState,
    MsgType.TLM_EVENT: TlmEvent,
    MsgType.TLM_ACK: TlmAck,
    MsgType.TLM_EXP: TlmExp,
    MsgType.TLM_CAL_DATA: TlmCalData,
    MsgType.TLM_CTRL: TlmCtrl,
    MsgType.LOG_TEXT: LogText,
    MsgType.RLY_SET_TARGET: RlySetTarget,
    MsgType.RLY_TARGET_ACK: RlyTargetAck,
    MsgType.RLY_STATS: RlyStats,
    MsgType.RLY_PONG: RlyPong,
    MsgType.RLY_SET_PEERS: RlySetPeers,
    MsgType.RLY_PEERS_ACK: RlyPeersAck,
    MsgType.RLY_MUX_UP: RlyMuxUp,
    MsgType.RLY_MUX_DOWN: RlyMuxDown,
}

# 型 -> 期待ペイロード長。None は可変長(LOG_TEXT: 1〜181B)。
# ドローン側受理規則「len==期待値」の判定に使用する。
EXPECTED_PAYLOAD_SIZE: dict[MsgType, Optional[int]] = {
    MsgType.CMD_START: 0,
    MsgType.CMD_STOP: 0,
    MsgType.CMD_SETPOINT: CmdSetpoint.PAYLOAD_SIZE,
    MsgType.CMD_POS_ERR: CmdPosErr.PAYLOAD_SIZE,
    MsgType.CMD_RESET: 0,
    MsgType.CMD_MODE: CmdMode.PAYLOAD_SIZE,
    MsgType.CMD_MOTOR_RUN: CmdMotorRun.PAYLOAD_SIZE,
    MsgType.CMD_MOTOR_STOP: 0,
    MsgType.CMD_CAL_GET: 0,
    MsgType.CMD_MAG3D_SET: CmdMag3dSet.PAYLOAD_SIZE,
    MsgType.CMD_ACCEL6_SET: CmdAccel6Set.PAYLOAD_SIZE,
    MsgType.CMD_ATTMOUNT_SET: CmdAttmountSet.PAYLOAD_SIZE,
    MsgType.CMD_YAWZERO_SET: CmdYawzeroSet.PAYLOAD_SIZE,
    MsgType.CMD_GEOMAG_SET: CmdGeomagSet.PAYLOAD_SIZE,
    MsgType.CMD_FF_BEGIN: CmdFfBegin.PAYLOAD_SIZE,
    MsgType.CMD_FF_LUT: CmdFfLut.PAYLOAD_SIZE,
    MsgType.CMD_FF_MOT: CmdFfMot.PAYLOAD_SIZE,
    MsgType.CMD_FF_AUX: CmdFfAux.PAYLOAD_SIZE,
    MsgType.CMD_FF_COMMIT: CmdFfCommit.PAYLOAD_SIZE,
    MsgType.CMD_FF_MODE: CmdFfMode.PAYLOAD_SIZE,
    MsgType.CMD_FF_ANCHOR: 0,
    MsgType.CMD_LED_MODE: CmdLedMode.PAYLOAD_SIZE,
    MsgType.CMD_MAGBIAS_SET: CmdMagbiasSet.PAYLOAD_SIZE,
    MsgType.CMD_FLOWCAL_SET: CmdFlowcalSet.PAYLOAD_SIZE,
    MsgType.CMD_FLOW_PROBE: CmdFlowProbe.PAYLOAD_SIZE,
    MsgType.TLM_STATE: TlmState.PAYLOAD_SIZE,
    MsgType.TLM_EVENT: TlmEvent.PAYLOAD_SIZE,
    MsgType.TLM_ACK: TlmAck.PAYLOAD_SIZE,
    MsgType.TLM_EXP: TlmExp.PAYLOAD_SIZE,
    MsgType.TLM_CAL_DATA: TlmCalData.PAYLOAD_SIZE,
    MsgType.TLM_CTRL: TlmCtrl.PAYLOAD_SIZE,
    MsgType.LOG_TEXT: None,
    MsgType.RLY_SET_TARGET: RlySetTarget.PAYLOAD_SIZE,
    MsgType.RLY_TARGET_ACK: RlyTargetAck.PAYLOAD_SIZE,
    MsgType.RLY_STATS: RlyStats.PAYLOAD_SIZE,
    MsgType.RLY_PING: 0,
    MsgType.RLY_PONG: RlyPong.PAYLOAD_SIZE,
    MsgType.RLY_SET_PEERS: None,   # 可変長(2+7×N)
    MsgType.RLY_PEERS_ACK: RlyPeersAck.PAYLOAD_SIZE,
    MsgType.RLY_MUX_UP: None,      # 可変長(1+内側フレーム)
    MsgType.RLY_MUX_DOWN: None,    # 可変長(1+内側フレーム)
}


def decode_payload(frame: Frame):
    """検証済みフレームのペイロードを型に応じた dataclass に展開する。

    ペイロードなしの型は None を返す。未知の型・長さ不整合は ValueError。
    """
    msg_type = MsgType(frame.type)   # 未知型は ValueError
    if msg_type in EMPTY_PAYLOAD_TYPES:
        if frame.payload:
            raise ValueError(f"{msg_type.name}: expected empty payload, got {len(frame.payload)}B")
        return None
    return PAYLOAD_CLASSES[msg_type].from_payload(frame.payload)


# ---------------------------------------------------------------------------
# 逐次シリアルレシーバ
# ---------------------------------------------------------------------------

@dataclass
class ReceiverCounters:
    """受信統計。フィールド名は C++ 側 SerialFrameReceiver::Counters と同一。"""
    frames_ok: int = 0
    cobs_errors: int = 0
    crc_errors: int = 0
    ver_errors: int = 0
    len_errors: int = 0
    overflow_drops: int = 0


class SerialFrameReceiver:
    """0x00 区切りのシリアルバイト列から論理フレームを取り出す逐次レシーバ。

    回復則(PROTOCOL.md「トランスポート」):

    - COBSデコード失敗 / CRC不一致 / ver不一致 / len不整合はフレームごと破棄
      (カウンタ加算のみ。部分回復しない)。
    - 蓄積が 256B を超えたら次の 0x00 まで読み捨て(overflow_drops 加算)。

    スレッド安全ではない。単一の読み取りスレッドから使用すること
    (共有が必要な場合は呼び出し側で lock を持つ)。
    """

    def __init__(self, on_frame: Optional[Callable[[Frame], None]] = None) -> None:
        self._buf = bytearray()
        self._dropping = False
        self._on_frame = on_frame
        self.counters = ReceiverCounters()

    def reset(self) -> None:
        """蓄積バッファと読み捨て状態をクリアする(カウンタは維持)。"""
        self._buf.clear()
        self._dropping = False

    def reset_counters(self) -> None:
        self.counters = ReceiverCounters()

    def feed(self, data: bytes) -> list[Frame]:
        """受信バイト列を供給し、完成した有効フレームのリストを返す。

        on_frame コールバックが設定されていればフレームごとに呼ばれる
        (コールバックとリスト返却のどちらでも消費できる)。
        """
        data = bytes(data)
        frames: list[Frame] = []
        pos = 0
        n = len(data)
        while pos < n:
            idx = data.find(COBS_DELIMITER, pos)
            seg_end = idx if idx >= 0 else n
            if seg_end > pos:
                # 非デリミタ区間の蓄積(バイト単位処理と等価なセグメント処理)
                if not self._dropping:
                    if len(self._buf) + (seg_end - pos) > SERIAL_RX_BUFFER_CAP:
                        # 上限超過 → 次の 0x00 まで読み捨て
                        self._buf.clear()
                        self._dropping = True
                        self.counters.overflow_drops += 1
                    else:
                        self._buf += data[pos:seg_end]
            if idx < 0:
                break
            pos = idx + 1
            # --- デリミタ到達: 蓄積分を1フレームとして処理 ---
            if self._dropping:
                self._dropping = False
                continue
            if not self._buf:
                continue   # 連続デリミタ / アイドル状態は無視
            raw = bytes(self._buf)
            self._buf.clear()
            try:
                decoded = cobs_decode(raw)
            except CobsDecodeError:
                self.counters.cobs_errors += 1
                continue
            status, frame = parse_frame(decoded)
            if status is ParseStatus.OK:
                assert frame is not None
                self.counters.frames_ok += 1
                frames.append(frame)
                if self._on_frame is not None:
                    self._on_frame(frame)
            elif status is ParseStatus.BAD_CRC:
                self.counters.crc_errors += 1
            elif status is ParseStatus.BAD_VER:
                self.counters.ver_errors += 1
            else:
                self.counters.len_errors += 1
        return frames
