"""ヨー基準ソース切替(契約 §3.1)の CMD_POS_ERR 内容検証と v6 ログ列。

- mocap(既定): 従来動作 — heading 有効時 bit3、bit4=0
- off: bit3 を立てない(mocap_yaw=0)
- motion: 推定 valid 時 bit3+bit4(低信頼)、invalid 時 bit3 を落とす。
  送信値は PoC 規約 ψ̂ → yaw_true → heading 相当 → ワイヤ規約の変換
  (session._update_motion_yaw)を経る
- v6 ロガー: yaw_ref_source / yaw_ref_sent_rad / yaw_ref_valid /
  motion_yaw_rad / motion_yaw_J 列が行に載る
"""

from __future__ import annotations

import csv

import pytest
import stampfly_protocol as proto

from core.mocap import wrap_pi
from core.session import YAW_REF_MOCAP, YAW_REF_MOTION, YAW_REF_OFF


def _position_meta(**overrides) -> dict:
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
        "filtered_pos": (0.10, -0.20, 0.30),
    }
    meta.update(overrides)
    return meta


def _sent_pos_errs(transport) -> list[proto.CmdPosErr]:
    return [proto.CmdPosErr.from_payload(f.payload)
            for f in transport.sent_frames
            if f.type == proto.MsgType.CMD_POS_ERR]


def _connect_quiet(session, transport):
    assert session.connect("COM-fake")
    session.posture.stop()
    session.position.stop()
    transport.sent_frames.clear()


def _force_motion_estimate(session, yaw_rad, j=25.0, valid=True):
    """MotionYawEstimator.estimate() を固定値に差し替える(単体は
    test_motion_yaw.py で検証済み — ここでは配線とフラグ規約を見る)。"""
    session.motion_yaw.estimate = lambda: {
        "yaw_rad": yaw_rad if valid else None,
        "j": j, "valid": valid, "n": 400}


class TestYawRefSource:
    def test_default_source_is_mocap_with_bit4_clear(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.yaw_ref_status()["source"] == YAW_REF_MOCAP

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        assert not (pe.flags & proto.CmdPosErr.FLAG_YAW_REF_LOW_TRUST)
        assert pe.mocap_yaw == pytest.approx(1.62)

    def test_off_source_clears_bit3(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_OFF)["ok"]

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)
        assert not (pe.flags & proto.CmdPosErr.FLAG_YAW_REF_LOW_TRUST)
        assert pe.mocap_yaw == 0.0

    def test_motion_source_sets_bit3_and_bit4(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        psi_poc = -0.8   # PoC 規約(ψ̂ ≈ -yaw_true)
        _force_motion_estimate(session, psi_poc)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        assert pe.flags & proto.CmdPosErr.FLAG_YAW_REF_LOW_TRUST
        # 規約変換: yaw_true = -ψ̂ → heading = yaw_sign*(yaw_true - offset)
        # → ワイヤ = heading * machine_wire_y_sign(mocap ソースと同一変換)
        yaw_sign, yaw_offset = session.mocap.attitude_yaw_convention
        wire_sign = session.mocap.machine_wire_y_sign
        expected = wrap_pi(yaw_sign * (-psi_poc - yaw_offset) * wire_sign)
        assert pe.mocap_yaw == pytest.approx(expected)

    def test_motion_source_invalid_clears_bit3(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        _force_motion_estimate(session, None, j=1.0, valid=False)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)
        assert pe.mocap_yaw == 0.0
        # 他フィールドは影響を受けない(既存契約の不変性)
        assert pe.flags & proto.CmdPosErr.FLAG_ALT_REF_VALID
        assert pe.flags & proto.CmdPosErr.FLAG_XY_ERR_VALID

    def test_invalid_source_rejected(self, session_factory):
        session, _transport, _clock = session_factory()
        result = session.set_yaw_ref_source("gps")
        assert result["ok"] is False
        assert session.yaw_ref_status()["source"] == YAW_REF_MOCAP

    def test_snapshot_carries_yaw_ref(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        _force_motion_estimate(session, -0.8)
        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        snapshot = session.get_state_snapshot()
        yaw_ref = snapshot["data"]["session"]["yaw_ref"]
        assert yaw_ref["source"] == YAW_REF_MOTION
        assert yaw_ref["motion_valid"] is True
        assert yaw_ref["motion_yaw_deg"] is not None


class TestV6LogColumns:
    def test_pos_err_row_contains_yaw_ref_columns(self, session_factory,
                                                  tmp_path):
        """v6 ロガー: CMD_POS_ERR 行に PC 側ヨー基準/移動ベースヨー列が載る。"""
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        _force_motion_estimate(session, -0.8, j=25.0)
        session.logger._logs_dir = tmp_path
        path = session.logger.start("position",
                                    metadata=session._log_metadata())

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())
        session.logger.stop()

        with open(path, newline="", encoding="utf-8") as fp:
            rows = list(csv.DictReader(fp))
        assert len(rows) == 1
        row = rows[0]
        pe = _sent_pos_errs(transport)[0]
        assert row["yaw_ref_source"] == YAW_REF_MOTION
        assert float(row["yaw_ref_sent_rad"]) == pytest.approx(
            pe.mocap_yaw, abs=1e-6)
        assert row["yaw_ref_valid"] == "1"
        assert float(row["motion_yaw_rad"]) == pytest.approx(
            pe.mocap_yaw, abs=1e-6)
        assert float(row["motion_yaw_J"]) == pytest.approx(25.0)
        # meta.json は v6 を宣言する
        meta_path = path.with_suffix(".meta.json")
        assert meta_path.is_file()
        assert '"log_columns_version": 6' in meta_path.read_text(
            encoding="utf-8")
