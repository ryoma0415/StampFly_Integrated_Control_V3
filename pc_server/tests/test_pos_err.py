"""機上XY制御(CMD_POS_ERR)のテスト。

Position モードの XY 指令は CMD_POS_ERR に一本化されている(v2.2 で
旧 PC 側 XY PID 経路を削除。CMD_SETPOINT は Posture モード専用)。

- _emit_setpoint の分岐: Position モードは常に CMD_POS_ERR を送る
- フラグ規約: bit2(XY有効)= 閉ループ有効 かつ データ有効 かつ 非途絶
- 誤差クランプ(pos_err_clamp_m)
- Posture モードでは従来どおり CMD_SETPOINT(バイアス加算つき)
- MoCap 制御座標系ヨー(heading_rad)の算出
"""

from __future__ import annotations

from math import atan2, cos, pi, sin

import pytest
import stampfly_protocol as proto

from core.mocap import CoordinateTransformer, quaternion_rotate_x_axis

from fakes import wait_until


def _position_meta(**overrides) -> dict:
    """Position モードの step() が渡す meta の代表形。"""
    meta = {
        "mode": "position",
        "data_valid": True,
        "control_active": True,
        "mocap_dropout": False,
        "error_x": 0.35,
        "error_y": -0.2,
        "yaw_ref_rad": 0.5,
        "yaw_ctrl_on": True,
        "mocap_heading_rad": 1.62,
        # 連続性フィルタ後の正解ヨー(P0-1 以降のヨー基準ワイヤ経路の入力)
        "mocap_yaw_true_rad": 1.62,
        "mocap_flip": 0,
    }
    meta.update(overrides)
    return meta


def _sent_pos_errs(transport) -> list[proto.CmdPosErr]:
    return [proto.CmdPosErr.from_payload(f.payload)
            for f in transport.sent_frames
            if f.type == proto.MsgType.CMD_POS_ERR]


def _connect_quiet(session, transport):
    """接続して 50Hz 送信スレッドを止める(決定的な emit テスト用)。"""
    assert session.connect("COM-fake")
    session.posture.stop()
    session.position.stop()
    transport.sent_frames.clear()


class TestEmitPosErr:
    def test_position_meta_emits_cmd_pos_err(self, session_factory):
        session, transport, _clock = session_factory()
        # ヨー基準は absolute 整列で決定的に(arm 整列のアンカー挙動は
        # test_yaw_ref.py が検証する)
        assert session.set_yaw_ref_align("absolute")["ok"]
        _connect_quiet(session, transport)

        session._emit_setpoint(0.01, 0.02, 0.5, _position_meta())

        frames = _sent_pos_errs(transport)
        assert len(frames) == 1
        pe = frames[0]
        assert pe.err_x == pytest.approx(0.35)
        assert pe.err_y == pytest.approx(-0.2)
        assert pe.alt_ref == pytest.approx(0.5)
        assert pe.yaw_ref == pytest.approx(0.5)
        assert pe.mocap_yaw == pytest.approx(1.62)   # 連続性フィルタ後 yaw_true
        assert pe.flags & proto.CmdPosErr.FLAG_ALT_REF_VALID
        assert pe.flags & proto.CmdPosErr.FLAG_YAW_REF_VALID
        assert pe.flags & proto.CmdPosErr.FLAG_XY_ERR_VALID
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        # roll/pitch 角度指令(CMD_SETPOINT)は送られない
        assert not any(f.type == proto.MsgType.CMD_SETPOINT
                       for f in transport.sent_frames)

    def test_dropout_clears_xy_valid_but_keeps_real_error(self,
                                                          session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            mocap_dropout=True, data_valid=False))

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_XY_ERR_VALID)
        # 実誤差は保持して送る(機体側 PID の誤差履歴が現実を追い続ける。
        # 0 にすり替えると復帰1サンプル目に D 項スパイクを作る)
        assert pe.err_x == pytest.approx(0.35)
        assert pe.err_y == pytest.approx(-0.2)

    def test_control_inactive_clears_xy_valid(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            control_active=False))

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_XY_ERR_VALID)

    def test_error_clamped(self, session_factory, control_config):
        clamp = control_config["control"]["pos_err_clamp_m"]
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            error_x=10.0, error_y=-10.0))

        pe = _sent_pos_errs(transport)[0]
        assert pe.err_x == pytest.approx(clamp)
        assert pe.err_y == pytest.approx(-clamp)

    def test_yaw_true_missing_clears_mocap_yaw_flag(self, session_factory):
        session, transport, _clock = session_factory()
        assert session.set_yaw_ref_align("absolute")["ok"]
        _connect_quiet(session, transport)

        meta = _position_meta()
        del meta["mocap_yaw_true_rad"]
        session._emit_setpoint(0.0, 0.0, 0.3, meta)

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)
        assert pe.mocap_yaw == 0.0

    def test_yaw_ctrl_off_clears_yaw_flag(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            yaw_ctrl_on=False))

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_YAW_REF_VALID)
        assert pe.yaw_ref == 0.0

    def test_posture_meta_still_emits_cmd_setpoint(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)

        session._emit_setpoint(0.01, 0.02, 0.4, {
            "mode": "posture", "yaw_ref_rad": 0.0, "yaw_ctrl_on": False})

        assert not _sent_pos_errs(transport)
        assert any(f.type == proto.MsgType.CMD_SETPOINT
                   for f in transport.sent_frames)

    def test_end_to_end_position_sender_streams_pos_err(self, session_factory):
        """Position モードの 50Hz 送信スレッドが実際に CMD_POS_ERR を流す。"""
        session, transport, clock = session_factory()
        assert session.connect("COM-fake")
        assert session.set_mode("position")

        # 送信ループは注入クロックでペーシングされるため、待ちながら進める
        def _streamed() -> bool:
            clock.advance(0.02)
            return len(_sent_pos_errs(transport)) >= 3

        assert wait_until(_streamed)


class TestPostureBias:
    def test_posture_emits_cmd_setpoint_with_bias(self, session_factory):
        """Posture の CMD_SETPOINT にはバイアスが加算される(従来どおり)。"""
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)

        session._emit_setpoint(0.01, 0.02, 0.5, {
            "mode": "posture", "yaw_ref_rad": 0.0, "yaw_ctrl_on": False})

        assert not _sent_pos_errs(transport)
        sps = [proto.CmdSetpoint.from_payload(f.payload)
               for f in transport.sent_frames
               if f.type == proto.MsgType.CMD_SETPOINT]
        assert len(sps) == 1
        # バイアス(TEST_AIRFRAMES[0]: roll 2.0deg / pitch -1.5deg)が加算される
        assert sps[0].roll_ref == pytest.approx(0.01 + 2.0 * pi / 180.0)
        assert sps[0].pitch_ref == pytest.approx(0.02 - 1.5 * pi / 180.0)


class TestMocapHeading:
    """制御座標系ヨー(heading_rad)の算出。

    既定の座標変換(制御x←Motive z、制御y←−Motive x、制御z←Motive y)で
    機体前方軸(ボディ x 軸)を写像した方位角になることを検証する。
    """

    TRANSFORM = {
        "x": {"axis": "z", "sign": 1},
        "y": {"axis": "x", "sign": -1},
        "z": {"axis": "y", "sign": 1},
    }

    def _heading(self, quat) -> float:
        transformer = CoordinateTransformer(self.TRANSFORM)
        forward = transformer.motive_to_control(
            quaternion_rotate_x_axis(*quat))
        return atan2(forward[1], forward[0])

    def test_identity_quaternion(self):
        # 前方 = Motive +X → 制御 (0, -1, 0) → heading = -π/2
        assert self._heading((0.0, 0.0, 0.0, 1.0)) == pytest.approx(-pi / 2)

    def test_rotation_about_motive_up_axis(self):
        # Motive Y(上)まわり +90°: 前方 Motive +X → Motive -Z
        # → 制御 (-1, 0, 0) → heading = π
        half = pi / 4.0
        quat = (0.0, sin(half), 0.0, cos(half))
        assert abs(self._heading(quat)) == pytest.approx(pi)

    def test_pose_dict_contains_heading(self, session_factory):
        """pose dict に heading_rad が載り、position の meta に伝播する。"""
        session, _transport, _clock = session_factory()
        pose_holder: dict = {}
        session.mocap._on_pose = pose_holder.update  # 直接差し込み

        class Rb:
            id_num = 1
            pos = (0.0, 0.3, 0.0)
            rot = (0.0, 0.0, 0.0, 1.0)
            tracking_valid = True
            error = 0.001
            rb_marker_list = [1, 2, 3]

        pose = session.mocap._pose_from_rigid_body(Rb(), 1, 1)
        assert pose is not None
        assert pose["heading_rad"] == pytest.approx(-pi / 2)
