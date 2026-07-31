#!/usr/bin/env python3
"""test_vectors.json 再生成ツール。

stampfly_protocol.py(PROTOCOL.md 準拠)を正としてベクタを生成し、
書き出す前に Python 実装自身で期待挙動(破損系の破棄・カウンタ)を
セルフチェックする。C++ 側の独立検証は tests/host_test.cpp が行う。

使い方:
    python3 tests/generate_vectors.py        # protocol/test_vectors.json を上書き
"""

from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

PROTOCOL_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROTOCOL_DIR))

import stampfly_protocol as sp  # noqa: E402

OUT_PATH = PROTOCOL_DIR / "test_vectors.json"


def frame_vector(name: str, kind: str, msg_type: int, seq: int,
                 fields: dict, payload: bytes) -> dict:
    logical = sp.pack_frame(msg_type, seq, payload)
    wire = sp.cobs_encode(logical) + bytes([sp.COBS_DELIMITER])
    return {
        "name": name,
        "payload_kind": kind,
        "type": int(msg_type),
        "seq": seq,
        "fields": fields,
        "payload_hex": payload.hex(),
        "logical_hex": logical.hex(),
        "wire_hex": wire.hex(),
    }


def build_vectors() -> dict:
    frames = []

    # --- 2. CMD_SETPOINT seq=0x41424344(旧バグ回帰オマージュ: ペイロード中の
    #     0x41 をヘッダと誤認しないこと)。v2: 17B(yaw_ref+flags bit1)---
    sp1 = sp.CmdSetpoint(roll_ref=0.0524, pitch_ref=-0.0349, alt_ref=0.3,
                         yaw_ref=0.7854, flags=3)
    frames.append(frame_vector(
        "cmd_setpoint_seq_0x41424344", "CMD_SETPOINT",
        sp.MsgType.CMD_SETPOINT, 0x41424344,
        {"roll_ref": 0.0524, "pitch_ref": -0.0349, "alt_ref": 0.3,
         "yaw_ref": 0.7854, "flags": 3},
        sp1.to_payload()))

    # --- 3. payload に 0x00 を多数含むフレーム(alt_ref=0.0 等)の COBS 往復 ---
    sp0 = sp.CmdSetpoint(roll_ref=0.0, pitch_ref=0.0, alt_ref=0.0,
                         yaw_ref=0.0, flags=0)
    frames.append(frame_vector(
        "cmd_setpoint_all_zero_payload", "CMD_SETPOINT",
        sp.MsgType.CMD_SETPOINT, 1,
        {"roll_ref": 0.0, "pitch_ref": 0.0, "alt_ref": 0.0,
         "yaw_ref": 0.0, "flags": 0},
        sp0.to_payload()))

    # --- 3b. v1 互換動作(flags bit1=0、yaw_ref 無効=レートダンピングのみ)---
    sp_v1 = sp.CmdSetpoint(roll_ref=0.02, pitch_ref=0.01, alt_ref=0.4,
                           yaw_ref=0.0, flags=1)
    frames.append(frame_vector(
        "cmd_setpoint_yaw_disabled", "CMD_SETPOINT",
        sp.MsgType.CMD_SETPOINT, 2,
        {"roll_ref": 0.02, "pitch_ref": 0.01, "alt_ref": 0.4,
         "yaw_ref": 0.0, "flags": 1},
        sp_v1.to_payload()))

    # --- 3c. CMD_POS_ERR(v2.1 機上XY制御: 全フラグ有効の代表値)---
    pe1 = sp.CmdPosErr(err_x=0.35, err_y=-0.2, alt_ref=0.5,
                       yaw_ref=1.5708, mocap_yaw=1.62, flags=0x0F)
    frames.append(frame_vector(
        "cmd_pos_err_all_flags", "CMD_POS_ERR",
        sp.MsgType.CMD_POS_ERR, 3,
        {"err_x": 0.35, "err_y": -0.2, "alt_ref": 0.5,
         "yaw_ref": 1.5708, "mocap_yaw": 1.62, "flags": 0x0F},
        pe1.to_payload()))

    # --- 3d. CMD_POS_ERR(XY 無効=MoCap 途絶中の水平指令。bit2=0)---
    pe0 = sp.CmdPosErr(err_x=0.0, err_y=0.0, alt_ref=0.3,
                       yaw_ref=0.0, mocap_yaw=0.0, flags=0x01)
    frames.append(frame_vector(
        "cmd_pos_err_xy_invalid", "CMD_POS_ERR",
        sp.MsgType.CMD_POS_ERR, 4,
        {"err_x": 0.0, "err_y": 0.0, "alt_ref": 0.3,
         "yaw_ref": 0.0, "mocap_yaw": 0.0, "flags": 0x01},
        pe0.to_payload()))

    # --- 3e. CMD_POS_ERR(低信頼外部ヨー基準: bit3+bit4。MAG_AUTOTUNE §1.2)---
    pe_lt = sp.CmdPosErr(err_x=0.1, err_y=-0.05, alt_ref=0.3,
                         yaw_ref=0.5, mocap_yaw=-2.8,
                         flags=(sp.CmdPosErr.FLAG_ALT_REF_VALID |
                                sp.CmdPosErr.FLAG_YAW_REF_VALID |
                                sp.CmdPosErr.FLAG_XY_ERR_VALID |
                                sp.CmdPosErr.FLAG_MOCAP_YAW_VALID |
                                sp.CmdPosErr.FLAG_YAW_REF_LOW_TRUST))
    frames.append(frame_vector(
        "cmd_pos_err_low_trust", "CMD_POS_ERR",
        sp.MsgType.CMD_POS_ERR, 5,
        {"err_x": 0.1, "err_y": -0.05, "alt_ref": 0.3,
         "yaw_ref": 0.5, "mocap_yaw": -2.8, "flags": 0x1F},
        pe_lt.to_payload()))

    # --- 4. TLM_STATE: 全フィールド既知値(184B、磁気オートチューン拡張分を含む)---
    tlm_fields = {
        "seq_echo": 0x01020304,
        "elapsed_ms": 123456,
        "state": int(sp.FlightState.HOVER),
        "flags": sp.TlmState.FLAG_SETPOINT_FRESH | sp.TlmState.FLAG_FLYING,
        "reason": int(sp.Reason.START_CMD),
        "roll": 0.0123, "pitch": -0.0456, "yaw": 1.5708,
        "p": 0.25, "q": -0.5, "r": 0.75,
        "roll_ref": 0.0524, "pitch_ref": -0.0349,
        "alt_ref": 0.5,
        "altitude_tof": 0.48, "altitude_est": 0.51,
        "alt_velocity": -0.05,
        "z_dot_ref": 0.1,
        "voltage": 3.72,
        "duty_fr": 0.41, "duty_fl": 0.42, "duty_rr": 0.43, "duty_rl": 0.44,
        "ax": 0.01, "ay": -0.02, "az": 0.98,
        "loop_dt_us": 2500,
        "yaw_est_rad": 1.5601,
        "yaw_gyro_int_rad": 1.5432,
        "yaw_ref_rad": 1.5708,
        "current_a": 2.85,
        "db_hat_x_ut": 3.2,
        "db_hat_y_ut": -1.1,
        "bm_x_ut": 0.6,
        "bm_y_ut": -0.4,
        "nis": 1.75,
        "ffg": 0x05,
        "ff_status": (2 | sp.TlmState.FF_STATUS_EST_EKF |
                      sp.TlmState.FF_STATUS_ANCHOR_VALID |
                      sp.TlmState.FF_STATUS_FFCAL_LOADED |
                      sp.TlmState.FF_STATUS_YAW_CTRL_ACTIVE |
                      sp.TlmState.FF_STATUS_MAG_FRESH),
        # 磁気オートチューン/フロー拡張(オフセット 135 以降。MAG_AUTOTUNE §1.1)
        "mag_cal_x_ut": 24.5,
        "mag_cal_y_ut": -13.75,
        "mag_cal_z_ut": 38.25,
        "mag_lev_x_ut": 27.9,
        "mag_lev_y_ut": -3.1,
        "ekf2_yaw_rad": 1.5533,
        "ekf2_bm_x_ut": 2.4,
        "ekf2_bm_y_ut": -1.6,
        "ekf2_yaw_innov_rad": -0.021,
        "ekf2_status": (sp.TlmState.EKF2_STATUS_YAW_OBS_FRESH |
                        sp.TlmState.EKF2_STATUS_YAW_OBS_FUSED |
                        sp.TlmState.EKF2_STATUS_TAU_RW_MODE |
                        sp.TlmState.EKF2_STATUS_HEALTHY2),
        "ekf2_gate": 0x03,
        "flow_vx_mps": 0.12,
        "flow_vy_mps": -0.08,
        "flow_squal": 84,
        "flow_status": (sp.TlmState.FLOW_STATUS_SENSOR_OK |
                        sp.TlmState.FLOW_STATUS_BURST_OK |
                        sp.TlmState.FLOW_STATUS_VEL_VALID |
                        sp.TlmState.FLOW_STATUS_SQUAL_OK),
        "flow_dt_ms": 20,
    }
    tlm = sp.TlmState(**tlm_fields)
    payload = tlm.to_payload()
    assert len(payload) == 184
    frames.append(frame_vector(
        "tlm_state_full", "TLM_STATE", sp.MsgType.TLM_STATE, 1000,
        tlm_fields, payload))

    # --- 追加: 全シリアライザのクロス言語検証用ベクタ ---
    ev = sp.TlmEvent(state=int(sp.FlightState.LANDING), prev_state=int(sp.FlightState.HOVER),
                     reason=int(sp.Reason.LINK_LOSS), flags=1, voltage=3.41)
    frames.append(frame_vector(
        "tlm_event_link_loss", "TLM_EVENT", sp.MsgType.TLM_EVENT, 42,
        {"state": 5, "prev_state": 4, "reason": 8, "flags": 1, "voltage": 3.41},
        ev.to_payload()))

    log = sp.LogText(origin=sp.LogText.ORIGIN_RELAY, text="relay: peer set ch=6")
    frames.append(frame_vector(
        "log_text_relay", "LOG_TEXT", sp.MsgType.LOG_TEXT, 7,
        {"origin": 0, "text": "relay: peer set ch=6"},
        log.to_payload()))

    # PROTOCOL.md は LOG_TEXT を UTF-8 と定義する。多バイト文字(3B日本語+4B非BMP)の
    # クロス言語バイト一致を強制する。ensure_ascii=True で書き出すため、JSON 上は
    # \uXXXX(非BMPはサロゲートペア)となり、host_test.cpp の復元パスも検証される。
    utf8_text = "機体🛸: 高度0.50mに到達"
    log_utf8 = sp.LogText(origin=sp.LogText.ORIGIN_DRONE, text=utf8_text)
    frames.append(frame_vector(
        "log_text_drone_utf8", "LOG_TEXT", sp.MsgType.LOG_TEXT, 8,
        {"origin": 1, "text": utf8_text},
        log_utf8.to_payload()))

    mac = bytes([0x24, 0x6F, 0x28, 0xAA, 0xBB, 0xCC])
    st = sp.RlySetTarget(mac=mac, wifi_channel=6)
    frames.append(frame_vector(
        "rly_set_target", "RLY_SET_TARGET", sp.MsgType.RLY_SET_TARGET, 2,
        {"mac": list(mac), "wifi_channel": 6},
        st.to_payload()))

    ack = sp.RlyTargetAck(status=sp.RlyTargetAck.STATUS_OK, mac=mac, channel=6)
    frames.append(frame_vector(
        "rly_target_ack_ok", "RLY_TARGET_ACK", sp.MsgType.RLY_TARGET_ACK, 3,
        {"status": 0, "mac": list(mac), "channel": 6},
        ack.to_payload()))

    stats = sp.RlyStats(up_frames=1000, down_frames=2500, crc_errors=3,
                        cobs_errors=1, espnow_send_fail=2, overflow_drops=0)
    frames.append(frame_vector(
        "rly_stats", "RLY_STATS", sp.MsgType.RLY_STATS, 11,
        {"up_frames": 1000, "down_frames": 2500, "crc_errors": 3,
         "cobs_errors": 1, "espnow_send_fail": 2, "overflow_drops": 0},
        stats.to_payload()))

    pong = sp.RlyPong(echo_seq=77)
    frames.append(frame_vector(
        "rly_pong", "RLY_PONG", sp.MsgType.RLY_PONG, 12,
        {"echo_seq": 77},
        pong.to_payload()))

    # --- マルチ機体拡張(0x55-0x58)---
    peer_macs = [[0x48, 0xCA, 0x43, 0x3A, 0x51, 0x30],
                 [0x48, 0xCA, 0x43, 0x38, 0xA1, 0xCC]]
    set_peers = sp.RlySetPeers(wifi_channel=1, peers=(
        sp.RlyPeer(mac=bytes(peer_macs[0]), tlm_state_div=1),
        sp.RlyPeer(mac=bytes(peer_macs[1]), tlm_state_div=2),
    ))
    frames.append(frame_vector(
        "rly_set_peers_two", "RLY_SET_PEERS", sp.MsgType.RLY_SET_PEERS, 13,
        {"wifi_channel": 1,
         "peers": [{"mac": peer_macs[0], "tlm_state_div": 1},
                   {"mac": peer_macs[1], "tlm_state_div": 2}]},
        set_peers.to_payload()))

    frames.append(frame_vector(
        "rly_set_peers_clear", "RLY_SET_PEERS", sp.MsgType.RLY_SET_PEERS, 14,
        {"wifi_channel": 0, "peers": []},
        sp.RlySetPeers(wifi_channel=0, peers=()).to_payload()))

    peers_ack = sp.RlyPeersAck(status=sp.RlyPeersAck.STATUS_OK, count=2,
                               wifi_channel=1,
                               failed_index=sp.RlyPeersAck.FAILED_NONE)
    frames.append(frame_vector(
        "rly_peers_ack_ok", "RLY_PEERS_ACK", sp.MsgType.RLY_PEERS_ACK, 15,
        {"status": 0, "count": 2, "wifi_channel": 1, "failed_index": 0xFF},
        peers_ack.to_payload()))

    # MUX_UP: node 1 宛の CMD_SETPOINT(内側フレームは既出ベクタと同一バイト)
    mux_inner_up = sp.pack_frame(sp.MsgType.CMD_SETPOINT, 0x41424344,
                                 sp1.to_payload())
    frames.append(frame_vector(
        "rly_mux_up_setpoint_node1", "RLY_MUX_UP", sp.MsgType.RLY_MUX_UP, 16,
        {"node_id": 1, "inner_hex": mux_inner_up.hex()},
        sp.RlyMuxUp(node_id=1, inner=mux_inner_up).to_payload()))

    # MUX_DOWN: node 3 発の TLM_EVENT(内側フレームは既出ベクタと同一バイト)
    mux_inner_down = sp.pack_frame(sp.MsgType.TLM_EVENT, 42, ev.to_payload())
    frames.append(frame_vector(
        "rly_mux_down_event_node3", "RLY_MUX_DOWN", sp.MsgType.RLY_MUX_DOWN, 17,
        {"node_id": 3, "inner_hex": mux_inner_down.hex()},
        sp.RlyMuxDown(node_id=3, inner=mux_inner_down).to_payload()))

    frames.append(frame_vector(
        "cmd_start_empty_payload", "NONE", sp.MsgType.CMD_START, 5, {}, b""))

    # --- v2 新規上りメッセージ(0x14–0x23)---
    mode = sp.CmdMode(mode=sp.CmdMode.MODE_MOTOR_TEST)
    frames.append(frame_vector(
        "cmd_mode_motor_test", "CMD_MODE", sp.MsgType.CMD_MODE, 20,
        {"mode": 1}, mode.to_payload()))

    run = sp.CmdMotorRun(duty=0.35, mask=sp.CmdMotorRun.MASK_FL | sp.CmdMotorRun.MASK_FR)
    frames.append(frame_vector(
        "cmd_motor_run_front_pair", "CMD_MOTOR_RUN", sp.MsgType.CMD_MOTOR_RUN, 21,
        {"duty": 0.35, "mask": 3}, run.to_payload()))

    frames.append(frame_vector(
        "cmd_motor_stop_empty_payload", "NONE", sp.MsgType.CMD_MOTOR_STOP, 22, {}, b""))

    frames.append(frame_vector(
        "cmd_cal_get_empty_payload", "NONE", sp.MsgType.CMD_CAL_GET, 23, {}, b""))

    mag3d_offset = [12.5, -8.25, 3.75]
    mag3d_matrix = [1.02, 0.01, -0.02,
                    0.01, 0.98, 0.03,
                    -0.02, 0.03, 1.05]
    mag3d = sp.CmdMag3dSet(valid=1, offset=tuple(mag3d_offset),
                           matrix=tuple(mag3d_matrix))
    frames.append(frame_vector(
        "cmd_mag3d_set_full", "CMD_MAG3D_SET", sp.MsgType.CMD_MAG3D_SET, 24,
        {"valid": 1, "offset": mag3d_offset, "matrix": mag3d_matrix},
        mag3d.to_payload()))

    accel6_offset = [0.012, -0.008, 0.021]
    accel6_scale = [0.998, 1.002, 0.995]
    accel6 = sp.CmdAccel6Set(valid=1, offset=tuple(accel6_offset),
                             scale=tuple(accel6_scale))
    frames.append(frame_vector(
        "cmd_accel6_set_full", "CMD_ACCEL6_SET", sp.MsgType.CMD_ACCEL6_SET, 25,
        {"valid": 1, "offset": accel6_offset, "scale": accel6_scale},
        accel6.to_payload()))

    attmount = sp.CmdAttmountSet(valid=1, roll_rad=0.015, pitch_rad=-0.022)
    frames.append(frame_vector(
        "cmd_attmount_set", "CMD_ATTMOUNT_SET", sp.MsgType.CMD_ATTMOUNT_SET, 26,
        {"valid": 1, "roll_rad": 0.015, "pitch_rad": -0.022},
        attmount.to_payload()))

    yawzero = sp.CmdYawzeroSet(valid=1, offset_rad=-1.234)
    frames.append(frame_vector(
        "cmd_yawzero_set", "CMD_YAWZERO_SET", sp.MsgType.CMD_YAWZERO_SET, 27,
        {"valid": 1, "offset_rad": -1.234}, yawzero.to_payload()))

    geomag = sp.CmdGeomagSet(declination_east_deg=-7.5, inclination_deg=49.5,
                             horizontal_ut=30.0, vertical_ut=35.1, total_ut=46.2)
    frames.append(frame_vector(
        "cmd_geomag_set", "CMD_GEOMAG_SET", sp.MsgType.CMD_GEOMAG_SET, 28,
        {"declination_east_deg": -7.5, "inclination_deg": 49.5,
         "horizontal_ut": 30.0, "vertical_ut": 35.1, "total_ut": 46.2},
        geomag.to_payload()))

    ff_begin = sp.CmdFfBegin(nlut=8)
    frames.append(frame_vector(
        "cmd_ff_begin_nlut8", "CMD_FF_BEGIN", sp.MsgType.CMD_FF_BEGIN, 29,
        {"nlut": 8}, ff_begin.to_payload()))

    ff_lut = sp.CmdFfLut(idx=3, i_a=1.25, db_x=2.5, db_y=-1.75, db_z=0.5)
    frames.append(frame_vector(
        "cmd_ff_lut_point", "CMD_FF_LUT", sp.MsgType.CMD_FF_LUT, 30,
        {"idx": 3, "i_a": 1.25, "db_x": 2.5, "db_y": -1.75, "db_z": 0.5},
        ff_lut.to_payload()))

    ff_mot_a_tilde = [0.8, -0.6, 0.2]
    ff_mot = sp.CmdFfMot(idx=sp.CmdFfMot.MOTOR_FL, a_tilde=tuple(ff_mot_a_tilde),
                         c2=0.9, c1=1.8, c0=0.05)
    frames.append(frame_vector(
        "cmd_ff_mot_fl", "CMD_FF_MOT", sp.MsgType.CMD_FF_MOT, 31,
        {"idx": 0, "a_tilde": ff_mot_a_tilde, "c2": 0.9, "c1": 1.8, "c0": 0.05},
        ff_mot.to_payload()))

    ff_aux = sp.CmdFfAux(iid_a=0.12)
    frames.append(frame_vector(
        "cmd_ff_aux", "CMD_FF_AUX", sp.MsgType.CMD_FF_AUX, 32,
        {"iid_a": 0.12}, ff_aux.to_payload()))

    ff_commit = sp.CmdFfCommit(crc32=0xDEADBEEF)
    frames.append(frame_vector(
        "cmd_ff_commit", "CMD_FF_COMMIT", sp.MsgType.CMD_FF_COMMIT, 33,
        {"crc32": 0xDEADBEEF}, ff_commit.to_payload()))

    ff_mode = sp.CmdFfMode(ff_mode=sp.CmdFfMode.FF_MODE_B,
                           est_mode=sp.CmdFfMode.EST_MODE_EKF)
    frames.append(frame_vector(
        "cmd_ff_mode_b_ekf", "CMD_FF_MODE", sp.MsgType.CMD_FF_MODE, 34,
        {"ff_mode": 2, "est_mode": 1}, ff_mode.to_payload()))

    frames.append(frame_vector(
        "cmd_ff_anchor_empty_payload", "NONE", sp.MsgType.CMD_FF_ANCHOR, 35, {}, b""))

    # --- v2.2 計測中 LED インジケータ(0x25)---
    led_mode = sp.CmdLedMode(mode=sp.CmdLedMode.MODE_RECORDING)
    frames.append(frame_vector(
        "cmd_led_mode_recording", "CMD_LED_MODE", sp.MsgType.CMD_LED_MODE, 36,
        {"mode": 1}, led_mode.to_payload()))

    # --- 磁気オートチューン/フロー拡張(0x26–0x28。MAG_AUTOTUNE §1.4)---
    magbias = sp.CmdMagbiasSet(mode=sp.CmdMagbiasSet.MODE_SET,
                               dx=13.2, dy=-8.4, dz=2.1)
    frames.append(frame_vector(
        "cmd_magbias_set", "CMD_MAGBIAS_SET", sp.MsgType.CMD_MAGBIAS_SET, 37,
        {"mode": 1, "dx": 13.2, "dy": -8.4, "dz": 2.1},
        magbias.to_payload()))

    magbias_clear = sp.CmdMagbiasSet(mode=sp.CmdMagbiasSet.MODE_CLEAR)
    frames.append(frame_vector(
        "cmd_magbias_clear", "CMD_MAGBIAS_SET", sp.MsgType.CMD_MAGBIAS_SET, 38,
        {"mode": 0, "dx": 0.0, "dy": 0.0, "dz": 0.0},
        magbias_clear.to_payload()))

    # flowcal 2×2 行列(2026-07-31 改訂): K = diag(kx,ky)·R(φ0) 形の
    # 非対角込み代表値(f32 で正確に表現できる 1/4 刻み)
    flowcal = sp.CmdFlowcalSet(mode=sp.CmdFlowcalSet.MODE_SET,
                               m00=452.5, m01=36.25, m10=-33.5, m11=447.25)
    frames.append(frame_vector(
        "cmd_flowcal_set", "CMD_FLOWCAL_SET", sp.MsgType.CMD_FLOWCAL_SET, 39,
        {"mode": 1, "m00": 452.5, "m01": 36.25, "m10": -33.5, "m11": 447.25},
        flowcal.to_payload()))

    flow_probe = sp.CmdFlowProbe(n_cycles=0)  # 0=既定 200 サイクル
    frames.append(frame_vector(
        "cmd_flow_probe", "CMD_FLOW_PROBE", sp.MsgType.CMD_FLOW_PROBE, 40,
        {"n_cycles": 0}, flow_probe.to_payload()))

    # --- v2 新規下りメッセージ(0x32–0x34)---
    ack = sp.TlmAck(acked_type=int(sp.MsgType.CMD_FF_COMMIT), acked_seq=33,
                    status=sp.TlmAck.STATUS_OK)
    frames.append(frame_vector(
        "tlm_ack_ff_commit_ok", "TLM_ACK", sp.MsgType.TLM_ACK, 200,
        {"acked_type": int(sp.MsgType.CMD_FF_COMMIT), "acked_seq": 33, "status": 0},
        ack.to_payload()))

    exp_fields = {
        "elapsed_ms": 654321,
        "current_a": 3.15, "vbat_v": 3.85, "shunt_uv": 1250.0,
        "bx_raw": 21.5, "by_raw": -14.25, "bz_raw": 38.75,
        "bx_cal": 20.1, "by_cal": -13.9, "bz_cal": 37.6,
        "imu_temp_c": 41.5,
        "roll": 0.011, "pitch": -0.024, "yaw": 2.618,
        "p": 0.05, "q": -0.03, "r": 0.02,
        "ax": 0.015, "ay": -0.01, "az": 1.002,
        "duty_cmd": 0.45,
        "motors_mask": 0x0F,
        "flags": (sp.TlmExp.FLAG_CURRENT_VALID | sp.TlmExp.FLAG_MAG_FRESH |
                  sp.TlmExp.FLAG_MOTORS_RUNNING),
    }
    exp = sp.TlmExp(**exp_fields)
    exp_payload = exp.to_payload()
    assert len(exp_payload) == 86
    frames.append(frame_vector(
        "tlm_exp_full", "TLM_EXP", sp.MsgType.TLM_EXP, 201,
        exp_fields, exp_payload))

    cal_fields = {
        "valid_flags": 0xFF,   # bit6=magbias, bit7=flowcal を含む全有効
        "mag3d_offset": mag3d_offset,
        "mag3d_matrix": mag3d_matrix,
        "accel6_offset": accel6_offset,
        "accel6_scale": accel6_scale,
        "attmount_roll_rad": 0.015,
        "attmount_pitch_rad": -0.022,
        "yawzero_offset_rad": -1.234,
        "geomag": [-7.5, 49.5, 30.0, 35.1, 46.2],
        "ff_nlut": 8,
        "ff_crc32": 0xDEADBEEF,
        "ff_mode": 2,
        "est_mode": 2,
        "magbias": [13.2, -8.4, 2.1],
        "flowcal_m00": 452.5,
        "flowcal_m11": 447.25,
        "flowcal_m01": 36.25,
        "flowcal_m10": -33.5,
    }
    cal = sp.TlmCalData(
        valid_flags=cal_fields["valid_flags"],
        mag3d_offset=tuple(cal_fields["mag3d_offset"]),
        mag3d_matrix=tuple(cal_fields["mag3d_matrix"]),
        accel6_offset=tuple(cal_fields["accel6_offset"]),
        accel6_scale=tuple(cal_fields["accel6_scale"]),
        attmount_roll_rad=cal_fields["attmount_roll_rad"],
        attmount_pitch_rad=cal_fields["attmount_pitch_rad"],
        yawzero_offset_rad=cal_fields["yawzero_offset_rad"],
        geomag=tuple(cal_fields["geomag"]),
        ff_nlut=cal_fields["ff_nlut"],
        ff_crc32=cal_fields["ff_crc32"],
        ff_mode=cal_fields["ff_mode"],
        est_mode=cal_fields["est_mode"],
        magbias=tuple(cal_fields["magbias"]),
        flowcal_m00=cal_fields["flowcal_m00"],
        flowcal_m11=cal_fields["flowcal_m11"],
        flowcal_m01=cal_fields["flowcal_m01"],
        flowcal_m10=cal_fields["flowcal_m10"])
    cal_payload = cal.to_payload()
    assert len(cal_payload) == 140
    frames.append(frame_vector(
        "tlm_cal_data_full", "TLM_CAL_DATA", sp.MsgType.TLM_CAL_DATA, 202,
        cal_fields, cal_payload))

    # --- 制御診断拡張(0x35 TLM_CTRL: 全フィールド既知値の89B)---
    ctrl_fields = {
        "elapsed_ms": 246810,
        "roll_rate_ref": 0.85,
        "pitch_rate_ref": -0.65,
        "yaw_rate_ref": 0.5236,
        "pid_ang": [0.55, 0.2, 0.1,
                    -0.42, -0.15, -0.08,
                    0.3, 0.18, 0.04],
        "pid_rate": [0.021, 0.008, 0.003,
                     -0.017, -0.006, -0.002,
                     0.011, 0.004, 0.001],
        "flags": (sp.TlmCtrl.FLAG_XY_ONBOARD_ACTIVE |
                  sp.TlmCtrl.FLAG_YAW_CTRL_ACTIVE |
                  sp.TlmCtrl.FLAG_FLYING),
    }
    ctrl = sp.TlmCtrl(
        elapsed_ms=ctrl_fields["elapsed_ms"],
        roll_rate_ref=ctrl_fields["roll_rate_ref"],
        pitch_rate_ref=ctrl_fields["pitch_rate_ref"],
        yaw_rate_ref=ctrl_fields["yaw_rate_ref"],
        pid_ang=tuple(ctrl_fields["pid_ang"]),
        pid_rate=tuple(ctrl_fields["pid_rate"]),
        flags=ctrl_fields["flags"])
    ctrl_payload = ctrl.to_payload()
    assert len(ctrl_payload) == 89
    frames.append(frame_vector(
        "tlm_ctrl_full", "TLM_CTRL", sp.MsgType.TLM_CTRL, 203,
        ctrl_fields, ctrl_payload))

    by_name = {f["name"]: f for f in frames}

    def logical_of(name: str) -> bytes:
        return bytes.fromhex(by_name[name]["logical_hex"])

    # --- 5. 破損系 ---
    base = "cmd_setpoint_seq_0x41424344"
    second = "cmd_setpoint_all_zero_payload"

    # 5a. CRC 1ビット反転 → bad_crc で破棄
    corrupted = bytearray(logical_of(base))
    corrupted[-1] ^= 0x01  # CRC上位バイト(LE格納の末尾)を1ビット反転
    crc_flip_wire = sp.cobs_encode(bytes(corrupted)) + b"\x00"

    # 5b. デリミタ欠落 → 2フレームが連結され len 不整合 → 両方破棄
    concat_wire = (sp.cobs_encode(logical_of(base)) +
                   sp.cobs_encode(logical_of(second)) + b"\x00")

    # 5c. 256B超 → 次の 0x00 まで読み捨て、その後の正常フレームは受信できる
    oversize_wire = (b"\xaa" * 300 + b"\x00" +
                     bytes.fromhex(by_name[base]["wire_hex"]))

    # 5d. v1 フレーム混入(CRC は正しいが ver=0x01)→ ver_errors として破棄
    #     (新旧混在の可視化。契約 §1「ver_errors として可視化される」)
    v1_body = bytearray(logical_of(base)[:-2])
    v1_body[0] = 0x01
    v1_logical = (bytes(v1_body) +
                  struct.pack("<H", sp.crc16_ccitt_false(bytes(v1_body))))
    stale_v1_wire = sp.cobs_encode(v1_logical) + b"\x00"

    corruption = [
        {
            "name": "crc_single_bit_flip",
            "construct": {"kind": "crc_bit_flip", "base_frame": base,
                          "xor_last_byte": 1},
            "wire_hex": crc_flip_wire.hex(),
            "expect_frames": 0,
            "expect_counters": {"crc_errors": 1},
        },
        {
            "name": "missing_delimiter_concatenation",
            "construct": {"kind": "concat_no_delimiter",
                          "frames": [base, second]},
            "wire_hex": concat_wire.hex(),
            "expect_frames": 0,
            "expect_counters": {"len_errors": 1},
        },
        {
            "name": "oversize_drop_then_valid_frame",
            "construct": {"kind": "oversize_junk_then_frame",
                          "junk_byte": 0xAA, "junk_len": 300,
                          "base_frame": base},
            "wire_hex": oversize_wire.hex(),
            "expect_frames": 1,
            "expect_counters": {"overflow_drops": 1},
            "expect_frame_logical_hex": [by_name[base]["logical_hex"]],
        },
        {
            "name": "stale_v1_version_frame",
            "construct": {"kind": "version_patch", "base_frame": base,
                          "ver": 1},
            "wire_hex": stale_v1_wire.hex(),
            "expect_frames": 0,
            "expect_counters": {"ver_errors": 1},
        },
    ]

    # --- 6. UTF-8 文字境界切り詰め(utf8_truncate_len)のクロス言語ベクタ ---
    # 1B(ASCII)/3B(日本語)/4B(非BMP)文字の混在テキストで全境界を網羅する。
    truncate_bytes = "log: 高度0.5m🛸到達".encode("utf-8")
    utf8_truncate = {
        "text_hex": truncate_bytes.hex(),
        "cases": [
            {"max_len": n, "expect_len": sp.utf8_truncate_len(truncate_bytes, n)}
            for n in range(len(truncate_bytes) + 1)
        ],
    }

    return {
        "protocol_version": sp.PROTOCOL_VERSION,
        "generator": "protocol/tests/generate_vectors.py (stampfly_protocol.py)",
        "crc16": {"input_ascii": "123456789", "expected": 0x29B1},
        "frames": frames,
        "corruption": corruption,
        "utf8_truncate": utf8_truncate,
    }


def self_check(vectors: dict) -> None:
    """書き出し前に Python 実装自身でベクタの整合性を検証する。"""
    # CRC 検証ベクタ
    assert sp.crc16_ccitt_false(b"123456789") == vectors["crc16"]["expected"] == 0x29B1

    for fv in vectors["frames"]:
        logical = bytes.fromhex(fv["logical_hex"])
        wire = bytes.fromhex(fv["wire_hex"])
        # COBS 往復
        assert sp.cobs_decode(wire[:-1]) == logical
        # parse 往復
        status, frame = sp.parse_frame(logical)
        assert status is sp.ParseStatus.OK and frame is not None
        assert frame.type == fv["type"] and frame.seq == fv["seq"]
        assert frame.payload == bytes.fromhex(fv["payload_hex"])
        # レシーバ経由
        rx = sp.SerialFrameReceiver()
        got = rx.feed(wire)
        assert len(got) == 1 and got[0] == frame
        assert rx.counters.frames_ok == 1

    for cv in vectors["corruption"]:
        rx = sp.SerialFrameReceiver()
        got = rx.feed(bytes.fromhex(cv["wire_hex"]))
        assert len(got) == cv["expect_frames"], cv["name"]
        for key, value in cv["expect_counters"].items():
            assert getattr(rx.counters, key) == value, (cv["name"], key)
        for frame, expected_hex in zip(got, cv.get("expect_frame_logical_hex", [])):
            repacked = sp.pack_frame(frame.type, frame.seq, frame.payload)
            assert repacked == bytes.fromhex(expected_hex), cv["name"]

    # UTF-8 切り詰めベクタ: 期待値が「文字を分断せず、高々1文字しか余分に削らない」こと
    text = bytes.fromhex(vectors["utf8_truncate"]["text_hex"])
    for case in vectors["utf8_truncate"]["cases"]:
        cut = case["expect_len"]
        assert cut == sp.utf8_truncate_len(text, case["max_len"])
        assert cut <= case["max_len"]
        assert min(case["max_len"], len(text)) - cut < 4  # 削るのは高々1文字(≦4B)
        text[:cut].decode("utf-8")  # strict: 分断があれば UnicodeDecodeError


def main() -> None:
    vectors = build_vectors()
    self_check(vectors)
    OUT_PATH.write_text(json.dumps(vectors, indent=2, ensure_ascii=True) + "\n")
    print(f"wrote {OUT_PATH} "
          f"({len(vectors['frames'])} frame vectors, "
          f"{len(vectors['corruption'])} corruption vectors)")


if __name__ == "__main__":
    main()
