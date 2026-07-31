"""test_vectors.json に対する Python 実装のアサーション。

ベクタは tests/generate_vectors.py が Python 実装から生成したものだが、
このテストは「現在の実装がコミット済みベクタと一致し続けること」を保証する
(実装の無意識な変更によるワイヤ互換性破壊の検出)。
"""

from __future__ import annotations

import json

import pytest

import stampfly_protocol as sp
from conftest import VECTORS_PATH


def _load_frames() -> list[dict]:
    return json.loads(VECTORS_PATH.read_text())["frames"]


def _load_corruption() -> list[dict]:
    return json.loads(VECTORS_PATH.read_text())["corruption"]


def _build_message(kind: str, fields: dict):
    """JSON の fields からペイロード dataclass を構築する。"""
    if kind == "NONE":
        return None
    if kind == "CMD_SETPOINT":
        return sp.CmdSetpoint(**fields)
    if kind == "CMD_POS_ERR":
        return sp.CmdPosErr(**fields)
    if kind == "CMD_MODE":
        return sp.CmdMode(**fields)
    if kind == "CMD_MOTOR_RUN":
        return sp.CmdMotorRun(**fields)
    if kind == "CMD_MAG3D_SET":
        return sp.CmdMag3dSet(valid=fields["valid"],
                              offset=tuple(fields["offset"]),
                              matrix=tuple(fields["matrix"]))
    if kind == "CMD_ACCEL6_SET":
        return sp.CmdAccel6Set(valid=fields["valid"],
                               offset=tuple(fields["offset"]),
                               scale=tuple(fields["scale"]))
    if kind == "CMD_ATTMOUNT_SET":
        return sp.CmdAttmountSet(**fields)
    if kind == "CMD_YAWZERO_SET":
        return sp.CmdYawzeroSet(**fields)
    if kind == "CMD_GEOMAG_SET":
        return sp.CmdGeomagSet(**fields)
    if kind == "CMD_FF_BEGIN":
        return sp.CmdFfBegin(**fields)
    if kind == "CMD_FF_LUT":
        return sp.CmdFfLut(**fields)
    if kind == "CMD_FF_MOT":
        return sp.CmdFfMot(idx=fields["idx"], a_tilde=tuple(fields["a_tilde"]),
                           c2=fields["c2"], c1=fields["c1"], c0=fields["c0"])
    if kind == "CMD_FF_AUX":
        return sp.CmdFfAux(**fields)
    if kind == "CMD_FF_COMMIT":
        return sp.CmdFfCommit(**fields)
    if kind == "CMD_FF_MODE":
        return sp.CmdFfMode(**fields)
    if kind == "CMD_LED_MODE":
        return sp.CmdLedMode(**fields)
    if kind == "CMD_MAGBIAS_SET":
        return sp.CmdMagbiasSet(**fields)
    if kind == "CMD_FLOWCAL_SET":
        return sp.CmdFlowcalSet(**fields)
    if kind == "CMD_FLOW_PROBE":
        return sp.CmdFlowProbe(**fields)
    if kind == "TLM_STATE":
        return sp.TlmState(**fields)
    if kind == "TLM_EVENT":
        return sp.TlmEvent(**fields)
    if kind == "TLM_ACK":
        return sp.TlmAck(**fields)
    if kind == "TLM_EXP":
        return sp.TlmExp(**fields)
    if kind == "TLM_CAL_DATA":
        return sp.TlmCalData(
            valid_flags=fields["valid_flags"],
            mag3d_offset=tuple(fields["mag3d_offset"]),
            mag3d_matrix=tuple(fields["mag3d_matrix"]),
            accel6_offset=tuple(fields["accel6_offset"]),
            accel6_scale=tuple(fields["accel6_scale"]),
            attmount_roll_rad=fields["attmount_roll_rad"],
            attmount_pitch_rad=fields["attmount_pitch_rad"],
            yawzero_offset_rad=fields["yawzero_offset_rad"],
            geomag=tuple(fields["geomag"]),
            ff_nlut=fields["ff_nlut"],
            ff_crc32=fields["ff_crc32"],
            ff_mode=fields["ff_mode"],
            est_mode=fields["est_mode"],
            magbias=tuple(fields["magbias"]),
            flowcal_m00=fields["flowcal_m00"],
            flowcal_m11=fields["flowcal_m11"],
            flowcal_m01=fields["flowcal_m01"],
            flowcal_m10=fields["flowcal_m10"])
    if kind == "TLM_CTRL":
        return sp.TlmCtrl(
            elapsed_ms=fields["elapsed_ms"],
            roll_rate_ref=fields["roll_rate_ref"],
            pitch_rate_ref=fields["pitch_rate_ref"],
            yaw_rate_ref=fields["yaw_rate_ref"],
            pid_ang=tuple(fields["pid_ang"]),
            pid_rate=tuple(fields["pid_rate"]),
            flags=fields["flags"])
    if kind == "LOG_TEXT":
        return sp.LogText(**fields)
    if kind == "RLY_SET_TARGET":
        return sp.RlySetTarget(mac=bytes(fields["mac"]),
                               wifi_channel=fields["wifi_channel"])
    if kind == "RLY_TARGET_ACK":
        return sp.RlyTargetAck(status=fields["status"], mac=bytes(fields["mac"]),
                               channel=fields["channel"])
    if kind == "RLY_STATS":
        return sp.RlyStats(**fields)
    if kind == "RLY_PONG":
        return sp.RlyPong(**fields)
    if kind == "RLY_SET_PEERS":
        return sp.RlySetPeers(
            wifi_channel=fields["wifi_channel"],
            peers=tuple(sp.RlyPeer(mac=bytes(p["mac"]),
                                   tlm_state_div=p["tlm_state_div"])
                        for p in fields["peers"]))
    if kind == "RLY_PEERS_ACK":
        return sp.RlyPeersAck(**fields)
    if kind == "RLY_MUX_UP":
        return sp.RlyMuxUp(node_id=fields["node_id"],
                           inner=bytes.fromhex(fields["inner_hex"]))
    if kind == "RLY_MUX_DOWN":
        return sp.RlyMuxDown(node_id=fields["node_id"],
                             inner=bytes.fromhex(fields["inner_hex"]))
    raise AssertionError(f"unknown payload_kind: {kind}")


def test_crc16_vector(vectors):
    crc = vectors["crc16"]
    data = crc["input_ascii"].encode("ascii")
    assert sp.crc16_ccitt_false(data) == crc["expected"]
    assert sp.crc16_ccitt_false(b"123456789") == 0x29B1  # PROTOCOL.md 検証ベクタ


def test_vector_file_metadata(vectors):
    assert vectors["protocol_version"] == sp.PROTOCOL_VERSION == 0x02
    names = [f["name"] for f in vectors["frames"]]
    # PROTOCOL.md §テストベクタの必須項目が存在すること
    assert "cmd_setpoint_seq_0x41424344" in names
    assert "cmd_setpoint_all_zero_payload" in names
    assert "tlm_state_full" in names
    assert "log_text_drone_utf8" in names      # PROTOCOL.md §テストベクタ 6
    assert vectors["utf8_truncate"]["cases"]   # 同上(切り詰めベクタ)
    # v2 新規メッセージ型のベクタが揃っていること
    for required in ("cmd_setpoint_yaw_disabled",
                     "cmd_mode_motor_test", "cmd_motor_run_front_pair",
                     "cmd_motor_stop_empty_payload", "cmd_cal_get_empty_payload",
                     "cmd_mag3d_set_full", "cmd_accel6_set_full",
                     "cmd_attmount_set", "cmd_yawzero_set", "cmd_geomag_set",
                     "cmd_ff_begin_nlut8", "cmd_ff_lut_point", "cmd_ff_mot_fl",
                     "cmd_ff_aux", "cmd_ff_commit", "cmd_ff_mode_b_ekf",
                     "cmd_ff_anchor_empty_payload", "cmd_led_mode_recording",
                     "tlm_ack_ff_commit_ok", "tlm_exp_full", "tlm_cal_data_full",
                     "tlm_ctrl_full"):
        assert required in names, required
    # 磁気オートチューン/フロー拡張(0x26-0x28+CMD_POS_ERR bit4)のベクタ
    for required in ("cmd_pos_err_low_trust", "cmd_magbias_set",
                     "cmd_magbias_clear", "cmd_flowcal_set", "cmd_flow_probe"):
        assert required in names, required
    # マルチ機体拡張(0x55-0x58)のベクタが揃っていること
    for required in ("rly_set_peers_two", "rly_set_peers_clear",
                     "rly_peers_ack_ok",
                     "rly_mux_up_setpoint_node1", "rly_mux_down_event_node3"):
        assert required in names, required
    corruption_names = [c["name"] for c in vectors["corruption"]]
    assert "crc_single_bit_flip" in corruption_names
    assert "missing_delimiter_concatenation" in corruption_names
    assert "oversize_drop_then_valid_frame" in corruption_names
    assert "stale_v1_version_frame" in corruption_names


@pytest.mark.parametrize("vec", _load_frames(), ids=lambda v: v["name"])
def test_frame_vector(vec):
    payload_want = bytes.fromhex(vec["payload_hex"])
    logical_want = bytes.fromhex(vec["logical_hex"])
    wire_want = bytes.fromhex(vec["wire_hex"])

    # 1. ペイロードのシリアライズ一致
    msg = _build_message(vec["payload_kind"], vec["fields"])
    payload_got = b"" if msg is None else msg.to_payload()
    assert payload_got == payload_want

    # 2. 論理フレームの一致
    assert sp.pack_frame(vec["type"], vec["seq"], payload_got) == logical_want

    # 3. ワイヤバイト(COBS + デリミタ)の一致
    assert sp.encode_wire(vec["type"], vec["seq"], payload_got) == wire_want

    # 4. COBS 往復
    assert sp.cobs_decode(wire_want[:-1]) == logical_want
    assert sp.cobs_encode(logical_want) + b"\x00" == wire_want

    # 5. parse 往復
    status, frame = sp.parse_frame(logical_want)
    assert status is sp.ParseStatus.OK
    assert frame is not None
    assert frame.ver == sp.PROTOCOL_VERSION
    assert frame.type == vec["type"]
    assert frame.seq == vec["seq"]
    assert frame.payload == payload_want

    # 6. デシリアライズ往復(バイト保存)
    decoded = sp.decode_payload(frame)
    if vec["payload_kind"] == "NONE":
        assert decoded is None
    else:
        assert decoded is not None
        assert decoded.to_payload() == payload_want

    # 7. レシーバ経由
    rx = sp.SerialFrameReceiver()
    got = rx.feed(wire_want)
    assert got == [frame]
    assert rx.counters.frames_ok == 1


def test_tlm_state_field_offsets(vectors):
    """TLM_STATE の各フィールドが PROTOCOL.md 記載のオフセットに載っていること。"""
    import struct
    vec = next(f for f in vectors["frames"] if f["name"] == "tlm_state_full")
    payload = bytes.fromhex(vec["payload_hex"])
    f = vec["fields"]
    assert len(payload) == 184
    assert struct.unpack_from("<I", payload, 0)[0] == f["seq_echo"]
    assert struct.unpack_from("<I", payload, 4)[0] == f["elapsed_ms"]
    assert payload[8] == f["state"]
    assert payload[9] == f["flags"]
    assert payload[10] == f["reason"]
    offsets = {
        11: "roll", 15: "pitch", 19: "yaw",
        23: "p", 27: "q", 31: "r",
        35: "roll_ref", 39: "pitch_ref",
        43: "alt_ref",
        47: "altitude_tof", 51: "altitude_est",
        55: "alt_velocity",
        59: "z_dot_ref",
        63: "voltage",
        67: "duty_fr", 71: "duty_fl", 75: "duty_rr", 79: "duty_rl",
        83: "ax", 87: "ay", 91: "az",
        # v2 追加(末尾追加のみ。契約 §1.4)
        97: "yaw_est_rad", 101: "yaw_gyro_int_rad", 105: "yaw_ref_rad",
        109: "current_a",
        113: "db_hat_x_ut", 117: "db_hat_y_ut",
        121: "bm_x_ut", 125: "bm_y_ut",
        129: "nis",
        # 磁気オートチューン/フロー拡張(MAG_AUTOTUNE_DESIGN.md §1.1)
        135: "mag_cal_x_ut", 139: "mag_cal_y_ut", 143: "mag_cal_z_ut",
        147: "mag_lev_x_ut", 151: "mag_lev_y_ut",
        155: "ekf2_yaw_rad",
        159: "ekf2_bm_x_ut", 163: "ekf2_bm_y_ut",
        167: "ekf2_yaw_innov_rad",
        173: "flow_vx_mps", 177: "flow_vy_mps",
    }
    for off, key in offsets.items():
        got = struct.unpack_from("<f", payload, off)[0]
        want = struct.unpack("<f", struct.pack("<f", f[key]))[0]
        assert got == want, f"{key} at offset {off}"
    assert struct.unpack_from("<H", payload, 95)[0] == f["loop_dt_us"]
    assert payload[133] == f["ffg"]
    assert payload[134] == f["ff_status"]
    assert payload[171] == f["ekf2_status"]
    assert payload[172] == f["ekf2_gate"]
    assert payload[181] == f["flow_squal"]
    assert payload[182] == f["flow_status"]
    assert payload[183] == f["flow_dt_ms"]


def test_cmd_setpoint_field_offsets(vectors):
    """CMD_SETPOINT(17B)のオフセットが契約どおりであること(v2)。"""
    import struct
    vec = next(f for f in vectors["frames"]
               if f["name"] == "cmd_setpoint_seq_0x41424344")
    payload = bytes.fromhex(vec["payload_hex"])
    f = vec["fields"]
    assert len(payload) == 17
    for off, key in ((0, "roll_ref"), (4, "pitch_ref"),
                     (8, "alt_ref"), (12, "yaw_ref")):
        got = struct.unpack_from("<f", payload, off)[0]
        want = struct.unpack("<f", struct.pack("<f", f[key]))[0]
        assert got == want, f"{key} at offset {off}"
    assert payload[16] == f["flags"]
    assert f["flags"] & sp.CmdSetpoint.FLAG_YAW_REF_VALID  # bit1 = ヨー角制御ON


def test_tlm_cal_data_field_offsets(vectors):
    """TLM_CAL_DATA(140B)のオフセットが契約どおりであること(v2+磁気オートチューン
    拡張+2026-07-31 flowcal 2×2 化)。"""
    import struct
    vec = next(f for f in vectors["frames"] if f["name"] == "tlm_cal_data_full")
    payload = bytes.fromhex(vec["payload_hex"])
    f = vec["fields"]
    assert len(payload) == 140
    assert payload[0] == f["valid_flags"]
    # bit6=magbias / bit7=flowcal(MAG_AUTOTUNE_DESIGN.md §1.5)
    assert f["valid_flags"] & sp.TlmCalData.VALID_MAGBIAS
    assert f["valid_flags"] & sp.TlmCalData.VALID_FLOWCAL

    def f32s(offset: int, count: int) -> tuple:
        return struct.unpack_from(f"<{count}f", payload, offset)

    def want32(values) -> tuple:
        return struct.unpack(f"<{len(values)}f", struct.pack(f"<{len(values)}f", *values))

    assert f32s(1, 3) == want32(f["mag3d_offset"])
    assert f32s(13, 9) == want32(f["mag3d_matrix"])
    assert f32s(49, 3) == want32(f["accel6_offset"])
    assert f32s(61, 3) == want32(f["accel6_scale"])
    assert f32s(73, 1) == want32([f["attmount_roll_rad"]])
    assert f32s(77, 1) == want32([f["attmount_pitch_rad"]])
    assert f32s(81, 1) == want32([f["yawzero_offset_rad"]])
    assert f32s(85, 5) == want32(f["geomag"])
    assert payload[105] == f["ff_nlut"]
    assert struct.unpack_from("<I", payload, 106)[0] == f["ff_crc32"]
    assert payload[110] == f["ff_mode"]
    assert payload[111] == f["est_mode"]
    assert f32s(112, 3) == want32(f["magbias"])
    # flowcal K 行列のワイヤ順は m00@124, m11@128, m01@132, m10@136
    # (既存 kx/ky オフセット互換維持のため行優先ではない)
    assert f32s(124, 1) == want32([f["flowcal_m00"]])
    assert f32s(128, 1) == want32([f["flowcal_m11"]])
    assert f32s(132, 1) == want32([f["flowcal_m01"]])
    assert f32s(136, 1) == want32([f["flowcal_m10"]])


def test_tlm_ctrl_field_offsets(vectors):
    """TLM_CTRL(89B)のオフセットが契約どおりであること(0x35)。"""
    import struct
    vec = next(f for f in vectors["frames"] if f["name"] == "tlm_ctrl_full")
    payload = bytes.fromhex(vec["payload_hex"])
    f = vec["fields"]
    assert len(payload) == 89
    assert struct.unpack_from("<I", payload, 0)[0] == f["elapsed_ms"]

    def f32s(offset: int, count: int) -> tuple:
        return struct.unpack_from(f"<{count}f", payload, offset)

    def want32(values) -> tuple:
        return struct.unpack(f"<{len(values)}f", struct.pack(f"<{len(values)}f", *values))

    assert f32s(4, 1) == want32([f["roll_rate_ref"]])
    assert f32s(8, 1) == want32([f["pitch_rate_ref"]])
    assert f32s(12, 1) == want32([f["yaw_rate_ref"]])
    assert f32s(16, 9) == want32(f["pid_ang"])
    assert f32s(52, 9) == want32(f["pid_rate"])
    assert payload[88] == f["flags"]
    # bit0=xy_onboard_active, bit1=yaw_ctrl_active, bit2=flying
    assert f["flags"] & sp.TlmCtrl.FLAG_XY_ONBOARD_ACTIVE
    assert f["flags"] & sp.TlmCtrl.FLAG_YAW_CTRL_ACTIVE
    assert f["flags"] & sp.TlmCtrl.FLAG_FLYING


def test_zero_heavy_payload_has_no_zero_on_wire(vectors):
    """COBS の保証: ワイヤ上(デリミタ以外)に 0x00 が現れないこと。"""
    vec = next(f for f in vectors["frames"]
               if f["name"] == "cmd_setpoint_all_zero_payload")
    payload = bytes.fromhex(vec["payload_hex"])
    wire = bytes.fromhex(vec["wire_hex"])
    assert payload.count(0) >= 12   # 全ゼロ float x3 + flags=0
    assert wire[:-1].count(0) == 0  # COBS 後はデリミタのみが 0x00
    assert wire[-1] == 0


@pytest.mark.parametrize("vec", _load_corruption(), ids=lambda v: v["name"])
def test_corruption_vector(vec):
    rx = sp.SerialFrameReceiver()
    got = rx.feed(bytes.fromhex(vec["wire_hex"]))
    assert len(got) == vec["expect_frames"]
    assert rx.counters.frames_ok == vec["expect_frames"]
    # 列挙されたカウンタは一致、列挙されていないエラーカウンタは 0
    for key in ("cobs_errors", "crc_errors", "ver_errors", "len_errors",
                "overflow_drops"):
        want = vec["expect_counters"].get(key, 0)
        assert getattr(rx.counters, key) == want, key
    # 生き残るべきフレーム
    expected_logical = vec.get("expect_frame_logical_hex", [])
    assert len(got) == len(expected_logical) or not expected_logical
    for frame, logical_hex in zip(got, expected_logical):
        assert sp.pack_frame(frame.type, frame.seq, frame.payload) == \
            bytes.fromhex(logical_hex)


@pytest.mark.parametrize("vec", _load_corruption(), ids=lambda v: v["name"])
def test_corruption_wire_construction(vec):
    """construct 情報から破損ワイヤを再構築できること(ベクタの自己記述性)。"""
    frames = {f["name"]: f for f in _load_frames()}
    con = vec["construct"]
    if con["kind"] == "crc_bit_flip":
        logical = bytearray(bytes.fromhex(frames[con["base_frame"]]["logical_hex"]))
        logical[-1] ^= con["xor_last_byte"]
        wire = sp.cobs_encode(bytes(logical)) + b"\x00"
    elif con["kind"] == "concat_no_delimiter":
        wire = b"".join(
            sp.cobs_encode(bytes.fromhex(frames[name]["logical_hex"]))
            for name in con["frames"]) + b"\x00"
    elif con["kind"] == "oversize_junk_then_frame":
        wire = (bytes([con["junk_byte"]]) * con["junk_len"] + b"\x00" +
                bytes.fromhex(frames[con["base_frame"]]["wire_hex"]))
    elif con["kind"] == "version_patch":
        import struct
        logical = bytearray(bytes.fromhex(frames[con["base_frame"]]["logical_hex"]))
        logical[0] = con["ver"]
        body = bytes(logical[:-2])
        wire = sp.cobs_encode(
            body + struct.pack("<H", sp.crc16_ccitt_false(body))) + b"\x00"
    else:
        raise AssertionError(f"unknown construct kind: {con['kind']}")
    assert wire == bytes.fromhex(vec["wire_hex"])


def test_vectors_regeneration_is_stable(vectors):
    """generate_vectors.py の再生成結果がコミット済み JSON と一致すること。"""
    from generate_vectors import build_vectors
    assert build_vectors() == vectors
