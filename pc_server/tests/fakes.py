"""テスト用フェイク(serial transport / NatNet client / clock)。"""

from __future__ import annotations

import threading
import time
import types
from typing import Callable, Optional

import stampfly_protocol as proto


def wait_until(predicate: Callable[[], bool], timeout: float = 2.0,
               interval: float = 0.005) -> bool:
    """predicate が True になるまで実時間でポーリングする。"""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


class FakeClock:
    """テストから明示的に進める monotonic クロック。"""

    def __init__(self, start: float = 100.0) -> None:
        self._t = start
        self._lock = threading.Lock()

    def __call__(self) -> float:
        with self._lock:
            return self._t

    def advance(self, dt: float) -> float:
        with self._lock:
            self._t += dt
            return self._t


class FakeTransport:
    """pyserial Serial 互換の最小フェイク。

    - PC が write したバイト列を protocol レシーバでフレーム化して記録する
    - auto_responder(frame) -> [(msg_type, payload), ...] | None で
      リレー側の応答(RLY_TARGET_ACK 等)を模擬できる
    - push()/push_raw() でリレー→PC 方向のバイト列を注入する
    """

    def __init__(self, auto_responder: Optional[Callable] = None) -> None:
        self._cond = threading.Condition()
        self._rx = bytearray()
        self.closed = False
        self.fail_writes = False
        self.auto_responder = auto_responder
        self.raw_written = bytearray()
        self.sent_frames: list[proto.Frame] = []
        self._tx_receiver = proto.SerialFrameReceiver()
        self._push_seq = 10000

    # --- pyserial 互換インターフェース ---

    def write(self, data: bytes) -> int:
        if self.fail_writes:
            raise OSError("simulated write failure")
        self.raw_written += data
        responses: list[tuple[int, bytes]] = []
        for frame in self._tx_receiver.feed(bytes(data)):
            self.sent_frames.append(frame)
            if self.auto_responder is not None:
                reply = self.auto_responder(frame)
                if reply:
                    responses.extend(reply)
        for msg_type, payload in responses:
            self.push(msg_type, payload)
        return len(data)

    def read(self, size: int) -> bytes:
        with self._cond:
            if not self._rx:
                self._cond.wait(timeout=0.01)
            n = min(size, len(self._rx))
            data = bytes(self._rx[:n])
            del self._rx[:n]
            return data

    @property
    def in_waiting(self) -> int:
        with self._cond:
            return len(self._rx)

    def close(self) -> None:
        self.closed = True

    # --- テストヘルパ ---

    def push(self, msg_type: int, payload: bytes = b"",
             seq: Optional[int] = None) -> None:
        """リレー→PC 方向に有効なワイヤフレームを注入する。"""
        if seq is None:
            self._push_seq += 1
            seq = self._push_seq
        self.push_raw(proto.encode_wire(msg_type, seq, payload))

    def push_raw(self, data: bytes) -> None:
        with self._cond:
            self._rx += data
            self._cond.notify_all()

    def frames_of_type(self, msg_type: int) -> list[proto.Frame]:
        return [f for f in self.sent_frames if f.type == msg_type]

    def clear_sent(self) -> None:
        self.sent_frames.clear()


class FakeDroneResponder:
    """v2 上りコマンドに TLM_ACK / TLM_CAL_DATA を返すフェイク機体。

    - リレー: RLY_SET_TARGET → RLY_TARGET_ACK(make_ack_responder 同等)
    - 0x14–0x23 と 0x26–0x28(MAGBIAS/FLOWCAL/FLOW_PROBE): TLM_ACK
      (acked_type / acked_seq エコー、status は ack_status_overrides で
      型ごとに上書き可)。0x24 CMD_POS_ERR(ストリーム)/0x25 CMD_LED_MODE
      は従来どおり応答しない
    - CMD_CAL_GET: TLM_CAL_DATA(self.cal_data)を返す
    - CMD_MAG3D_SET / ACCEL6_SET / ATTMOUNT_SET / YAWZERO_SET / GEOMAG_SET /
      FF_COMMIT / FF_BEGIN / FF_MODE / MAGBIAS_SET / FLOWCAL_SET は
      self.cal_data に反映する(適用の読み戻し検証・CRC 照合を模擬)
    - drop_first_ack_types に入れた型は初回だけ ACK を落とす(リトライ試験)
    """

    def __init__(self) -> None:
        self._relay = make_ack_responder()
        self.cal_data = proto.TlmCalData()
        self.ack_status_overrides: dict[int, int] = {}
        self.drop_first_ack_types: set[int] = set()
        # 状態は更新するが ACK を一切返さない型(commit の ACK ロスト救済試験)
        self.silent_types: set[int] = set()
        self._dropped_once: set[int] = set()
        self._ff_nlut = 0
        # True にすると各コマンド応答に cal_data の ff/est モードを反映した
        # TLM_STATE を追随させる(ヨーゼロ自動シーケンスのモード反映確認と
        # CF 整列確認を模擬)。YAWZERO_SET(valid=1) 受理で yaw_est は 0 に
        # 整列する(次の磁気サンプルで新基準に揃うファーム挙動の近似)
        self.auto_tlm_state = False
        self.yaw_est_rad = 0.0
        self.roll_rad = 0.0    # attitude_zero(TLM_STATE 姿勢ソース)用
        self.pitch_rad = 0.0
        self.tlm_state_status_extra = proto.TlmState.FF_STATUS_MAG_FRESH

    def _update_cal_state(self, frame: proto.Frame) -> None:
        cal = self.cal_data
        t = frame.type
        if t == proto.MsgType.CMD_MAG3D_SET:
            msg = proto.CmdMag3dSet.from_payload(frame.payload)
            if msg.valid:
                cal.valid_flags |= proto.TlmCalData.VALID_MAG3D
                cal.mag3d_offset = tuple(msg.offset)
                cal.mag3d_matrix = tuple(msg.matrix)
            else:
                cal.valid_flags &= ~proto.TlmCalData.VALID_MAG3D
        elif t == proto.MsgType.CMD_ACCEL6_SET:
            msg = proto.CmdAccel6Set.from_payload(frame.payload)
            if msg.valid:
                cal.valid_flags |= proto.TlmCalData.VALID_ACCEL6
                cal.accel6_offset = tuple(msg.offset)
                cal.accel6_scale = tuple(msg.scale)
            else:
                cal.valid_flags &= ~proto.TlmCalData.VALID_ACCEL6
        elif t == proto.MsgType.CMD_ATTMOUNT_SET:
            msg = proto.CmdAttmountSet.from_payload(frame.payload)
            if msg.valid:
                cal.valid_flags |= proto.TlmCalData.VALID_ATTMOUNT
                cal.attmount_roll_rad = msg.roll_rad
                cal.attmount_pitch_rad = msg.pitch_rad
            else:
                cal.valid_flags &= ~proto.TlmCalData.VALID_ATTMOUNT
        elif t == proto.MsgType.CMD_YAWZERO_SET:
            msg = proto.CmdYawzeroSet.from_payload(frame.payload)
            if msg.valid:
                cal.valid_flags |= proto.TlmCalData.VALID_YAWZERO
                cal.yawzero_offset_rad = msg.offset_rad
                self.yaw_est_rad = 0.0   # 新基準へ整列(auto_tlm_state 用)
            else:
                cal.valid_flags &= ~proto.TlmCalData.VALID_YAWZERO
        elif t == proto.MsgType.CMD_GEOMAG_SET:
            msg = proto.CmdGeomagSet.from_payload(frame.payload)
            cal.valid_flags |= proto.TlmCalData.VALID_GEOMAG
            cal.geomag = (msg.declination_east_deg, msg.inclination_deg,
                          msg.horizontal_ut, msg.vertical_ut, msg.total_ut)
        elif t == proto.MsgType.CMD_FF_BEGIN:
            self._ff_nlut = proto.CmdFfBegin.from_payload(frame.payload).nlut
        elif t == proto.MsgType.CMD_FF_COMMIT:
            msg = proto.CmdFfCommit.from_payload(frame.payload)
            cal.valid_flags |= proto.TlmCalData.VALID_FFCAL
            cal.ff_crc32 = msg.crc32
            cal.ff_nlut = self._ff_nlut
        elif t == proto.MsgType.CMD_FF_MODE:
            msg = proto.CmdFfMode.from_payload(frame.payload)
            cal.ff_mode = msg.ff_mode
            cal.est_mode = msg.est_mode
        elif t == proto.MsgType.CMD_MAGBIAS_SET:
            msg = proto.CmdMagbiasSet.from_payload(frame.payload)
            if msg.mode == proto.CmdMagbiasSet.MODE_SET:
                cal.valid_flags |= proto.TlmCalData.VALID_MAGBIAS
                cal.magbias = (msg.dx, msg.dy, msg.dz)
            else:
                cal.valid_flags &= ~proto.TlmCalData.VALID_MAGBIAS
                cal.magbias = (0.0, 0.0, 0.0)
        elif t == proto.MsgType.CMD_FLOWCAL_SET:
            # 2026-07-31 改訂: flowcal 2×2 行列(K [counts/rad])を反映
            msg = proto.CmdFlowcalSet.from_payload(frame.payload)
            if msg.mode == proto.CmdFlowcalSet.MODE_SET:
                cal.valid_flags |= proto.TlmCalData.VALID_FLOWCAL
                cal.flowcal_m00 = msg.m00
                cal.flowcal_m01 = msg.m01
                cal.flowcal_m10 = msg.m10
                cal.flowcal_m11 = msg.m11
            else:
                cal.valid_flags &= ~proto.TlmCalData.VALID_FLOWCAL
                cal.flowcal_m00 = 0.0
                cal.flowcal_m01 = 0.0
                cal.flowcal_m10 = 0.0
                cal.flowcal_m11 = 0.0

    def __call__(self, frame: proto.Frame):
        relay = self._relay(frame)
        if relay:
            return relay
        t = frame.type
        # ACK 対象: キャリブ/FF 系 0x14–0x23+磁気オートチューン/フロー拡張
        # 0x26–0x28(0x24 CMD_POS_ERR / 0x25 CMD_LED_MODE は除外)
        if not (0x14 <= t <= 0x23 or 0x26 <= t <= 0x28):
            return None
        if t in self.drop_first_ack_types and t not in self._dropped_once:
            self._dropped_once.add(t)
            return None
        if t in self.silent_types:
            self._update_cal_state(frame)
            return None
        status = self.ack_status_overrides.get(t, proto.TlmAck.STATUS_OK)
        if status == proto.TlmAck.STATUS_OK:
            self._update_cal_state(frame)
        replies = [(proto.MsgType.TLM_ACK,
                    proto.TlmAck(acked_type=t, acked_seq=frame.seq,
                                 status=status).to_payload())]
        if t == proto.MsgType.CMD_CAL_GET:
            replies.append((proto.MsgType.TLM_CAL_DATA,
                            self.cal_data.to_payload()))
        if self.auto_tlm_state:
            replies.append((proto.MsgType.TLM_STATE,
                            self.make_tlm_state().to_payload()))
        return replies

    def make_tlm_state(self) -> proto.TlmState:
        """cal_data の ff/est モードを反映した TLM_STATE を組み立てる。"""
        ff_status = (self.cal_data.ff_mode
                     & proto.TlmState.FF_STATUS_FF_MODE_MASK)
        if self.cal_data.est_mode:
            ff_status |= proto.TlmState.FF_STATUS_EST_EKF
        ff_status |= self.tlm_state_status_extra
        return proto.TlmState(roll=self.roll_rad, pitch=self.pitch_rad,
                              yaw_est_rad=self.yaw_est_rad,
                              ff_status=ff_status)


def make_tlm_ctrl(elapsed_ms: int = 1000,
                  roll_rate_ref: float = 0.10, pitch_rate_ref: float = -0.20,
                  yaw_rate_ref: float = 0.05,
                  flags: int = proto.TlmCtrl.FLAG_FLYING,
                  pid_ang: tuple = None,
                  pid_rate: tuple = None) -> proto.TlmCtrl:
    """TLM_CTRL(制御ループ診断)ペイロードの組み立てヘルパ。

    pid_ang / pid_rate 省略時は判別可能な等差値(0.01, 0.02, ...)を入れる
    (ログ列の転記順の検証に使える)。
    """
    if pid_ang is None:
        pid_ang = tuple(0.01 * (i + 1) for i in range(9))
    if pid_rate is None:
        pid_rate = tuple(-0.01 * (i + 1) for i in range(9))
    return proto.TlmCtrl(
        elapsed_ms=elapsed_ms,
        roll_rate_ref=roll_rate_ref, pitch_rate_ref=pitch_rate_ref,
        yaw_rate_ref=yaw_rate_ref,
        pid_ang=tuple(pid_ang), pid_rate=tuple(pid_rate), flags=flags)


def make_tlm_exp(current_a: float = 0.2, vbat_v: float = 3.9,
                 b_raw: tuple = (10.0, -5.0, 30.0),
                 b_cal: tuple = (11.0, -4.0, 29.0),
                 roll: float = 0.0, pitch: float = 0.0, yaw: float = 0.0,
                 ax: float = 0.0, ay: float = 0.0, az: float = 1.0,
                 duty_cmd: float = 0.0, motors_mask: int = 0,
                 current_valid: bool = True) -> proto.TlmExp:
    """TLM_EXP ペイロードの組み立てヘルパ。"""
    flags = proto.TlmExp.FLAG_MAG_FRESH
    if current_valid:
        flags |= proto.TlmExp.FLAG_CURRENT_VALID
    if duty_cmd > 0.0:
        flags |= proto.TlmExp.FLAG_MOTORS_RUNNING
    return proto.TlmExp(
        elapsed_ms=0, current_a=current_a, vbat_v=vbat_v, shunt_uv=100.0,
        bx_raw=b_raw[0], by_raw=b_raw[1], bz_raw=b_raw[2],
        bx_cal=b_cal[0], by_cal=b_cal[1], bz_cal=b_cal[2],
        imu_temp_c=32.5, roll=roll, pitch=pitch, yaw=yaw,
        p=0.0, q=0.0, r=0.0, ax=ax, ay=ay, az=az,
        duty_cmd=duty_cmd, motors_mask=motors_mask, flags=flags)


def make_ack_responder(status: int = proto.RlyTargetAck.STATUS_OK,
                       channel_override: Optional[int] = None) -> Callable:
    """RLY_SET_TARGET に RLY_TARGET_ACK を返すレスポンダを作る。"""
    def responder(frame: proto.Frame):
        if frame.type != proto.MsgType.RLY_SET_TARGET:
            return None
        request = proto.RlySetTarget.from_payload(frame.payload)
        ack = proto.RlyTargetAck(
            status=status,
            mac=request.mac,
            channel=(channel_override if channel_override is not None
                     else request.wifi_channel),
        )
        return [(proto.MsgType.RLY_TARGET_ACK, ack.to_payload())]
    return responder


class FakeNatNetClient:
    """vendor/NatNetClient.py の最小フェイク(legacy テストのパターン)。"""

    instances: list["FakeNatNetClient"] = []

    def __init__(self) -> None:
        self.shutdown_called = False
        self.run_called = False
        self.new_frame_with_data_listener = None
        FakeNatNetClient.instances.append(self)

    def set_server_address(self, address) -> None:
        self.server_address = address

    def set_client_address(self, address) -> None:
        self.client_address = address

    def set_use_multicast(self, enabled) -> None:
        self.use_multicast = enabled

    def set_print_level(self, level) -> None:
        self.print_level = level

    def run(self, thread_option) -> bool:
        self.run_called = True
        return True

    def connected(self) -> bool:
        return self.run_called and not self.shutdown_called

    def shutdown(self) -> None:
        self.shutdown_called = True


def make_mocap_frame(rigid_body_id: int = 1,
                     pos: tuple = (0.0, 0.3, 0.0),
                     rot: tuple = (0.0, 0.0, 0.0, 1.0),
                     tracking_valid: bool = True,
                     error: float = 0.001,
                     marker_count: int = 4,
                     frame_number: int = 1) -> dict:
    """NatNet new_frame_with_data_listener 引数の模擬(Motive 座標)。"""
    rigid_body = types.SimpleNamespace(
        id_num=rigid_body_id,
        pos=pos,
        rot=rot,
        tracking_valid=tracking_valid,
        error=error,
        rb_marker_list=list(range(marker_count)),
    )
    mocap_data = types.SimpleNamespace(
        rigid_body_data=types.SimpleNamespace(rigid_body_list=[rigid_body]),
        labeled_marker_data=types.SimpleNamespace(labeled_marker_list=[]),
    )
    return {"mocap_data": mocap_data, "frame_number": frame_number}


def make_mocap_frame_multi(bodies: list[dict], frame_number: int = 1) -> dict:
    """複数リジッドボディ入り NatNet フレームの模擬(マルチ機体用)。

    bodies の各要素は {rigid_body_id, pos, rot, tracking_valid, error,
    marker_count} を任意指定(既定は make_mocap_frame と同じ)。
    """
    rigid_list = [
        types.SimpleNamespace(
            id_num=b.get("rigid_body_id", 1),
            pos=b.get("pos", (0.0, 0.3, 0.0)),
            rot=b.get("rot", (0.0, 0.0, 0.0, 1.0)),
            tracking_valid=b.get("tracking_valid", True),
            error=b.get("error", 0.001),
            rb_marker_list=list(range(b.get("marker_count", 4))),
        )
        for b in bodies
    ]
    mocap_data = types.SimpleNamespace(
        rigid_body_data=types.SimpleNamespace(rigid_body_list=rigid_list),
        labeled_marker_data=types.SimpleNamespace(labeled_marker_list=[]),
    )
    return {"mocap_data": mocap_data, "frame_number": frame_number}


def make_pose(x: float = 0.0, y: float = 0.0, z: float = 0.3,
              t: float = 0.0, yaw_rad: float = 0.0,
              tracking_valid: bool = True, error: float = 0.001,
              marker_count: int = 4, frame_number: int = 1) -> dict:
    """PositionController.on_mocap_pose に渡す制御座標系 pose dict。"""
    return {
        "t_mono": t,
        "frame_number": frame_number,
        "x": x,
        "y": y,
        "z": z,
        "yaw_rad": yaw_rad,
        "tracking_valid": tracking_valid,
        "error": error,
        "marker_count": marker_count,
        "quality": 1.0 / (1.0 + max(0.0, error)),
    }
