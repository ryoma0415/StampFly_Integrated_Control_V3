"""magbias プロファイル適用/クリア+flowcal/flow probe(契約 §1.4 / §3.4-5)。

FakeDroneResponder が CMD_MAGBIAS_SET / CMD_FLOWCAL_SET を cal_data に
反映するため、CAL_GET 読み戻し照合まで通しで検証できる。
"""

from __future__ import annotations

import json

import pytest

import stampfly_protocol as proto

from core.magbias import magbias_binding_diffs, magbias_delta_b
from core.session import MODE_EXPERIMENT

from conftest import halt_supervisor
from fakes import FakeDroneResponder

DELTA_B = [3.5, -12.25, 0.75]
FFCAL_CRC = 0x1234ABCD


def make_magbias_profile_dict(delta=None) -> dict:
    """契約書 §3 スキーマの最小有効プロファイル。"""
    return {
        "schema": "stampfly_magbias_profile",
        "version": 1,
        "name": "test_magbias",
        "created_at": "2026-07-27T00:00:00",
        "source_logs": ["20260727_164200_position.csv"],
        "delta_b_ut": list(DELTA_B) if delta is None else delta,
        "hover_residual_ut": {"leveled_xy": [1.0, -2.0], "z": 0.5,
                              "current_a": 3.2},
        "binding": {"mag3d_hash": "sha256:dummy",
                    "ffcal_crc": f"{FFCAL_CRC:08x}"},
        "quality": {"yaw_excursion_deg": 120.0, "fit_rms_ut": 0.8,
                    "warnings": []},
    }


@pytest.fixture
def magbias_session(server_config, session_factory, tmp_path):
    server_config["failsafe"]["command_ack_timeout_s"] = 0.1
    responder = FakeDroneResponder()
    # 機体の ffcal をプロファイルの binding と一致させる
    responder.cal_data.valid_flags = proto.TlmCalData.VALID_FFCAL
    responder.cal_data.ff_crc32 = FFCAL_CRC
    session, transport, clock = session_factory(responder=responder)
    session.connect("FAKE")
    halt_supervisor(session)
    assert session.set_mode(MODE_EXPERIMENT)
    mgr = session.magbias
    mgr.profile_dir = tmp_path / "magbias_profiles"
    mgr.state_path = tmp_path / "magbias_state.json"
    mgr.logs_dir = tmp_path / "flight_logs"
    mgr.profile_dir.mkdir(parents=True)
    (mgr.profile_dir / "test_magbias.json").write_text(
        json.dumps(make_magbias_profile_dict()), encoding="utf-8")
    return session, transport, clock, responder, mgr


class TestProfileValidation:
    def test_delta_b_extracted(self):
        assert magbias_delta_b(make_magbias_profile_dict()) == \
            pytest.approx(tuple(DELTA_B))

    def test_null_delta_b_rejected(self):
        """ヨー励振不足(delta_b=null)のプロファイルは適用不能。"""
        with pytest.raises(ValueError):
            magbias_delta_b(make_magbias_profile_dict(delta=[]))
        profile = make_magbias_profile_dict()
        profile["delta_b_ut"] = None
        with pytest.raises(ValueError):
            magbias_delta_b(profile)

    def test_wrong_schema_rejected(self):
        profile = make_magbias_profile_dict()
        profile["schema"] = "other"
        with pytest.raises(ValueError):
            magbias_delta_b(profile)

    def test_binding_diffs(self):
        cal = proto.TlmCalData(valid_flags=proto.TlmCalData.VALID_FFCAL,
                               ff_crc32=FFCAL_CRC)
        assert magbias_binding_diffs(make_magbias_profile_dict(), cal) == []
        cal.ff_crc32 = 0xDEADBEEF
        assert magbias_binding_diffs(make_magbias_profile_dict(), cal)


class TestMagbiasApply:
    def test_apply_sends_set_and_verifies_readback(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        transport.clear_sent()
        result = mgr.apply("test_magbias")
        assert result["ok"], result
        assert result["verified"] is True

        # CMD_MAGBIAS_SET(mode=1, Δb)が送られ、機体状態へ反映されている
        frames = transport.frames_of_type(proto.MsgType.CMD_MAGBIAS_SET)
        assert len(frames) == 1
        msg = proto.CmdMagbiasSet.from_payload(frames[0].payload)
        assert msg.mode == proto.CmdMagbiasSet.MODE_SET
        assert (msg.dx, msg.dy, msg.dz) == pytest.approx(tuple(DELTA_B))
        assert responder.cal_data.valid_flags & proto.TlmCalData.VALID_MAGBIAS
        assert responder.cal_data.magbias == pytest.approx(tuple(DELTA_B))

        # magbias_state.json に適用状態が残る(契約 §3.4)
        state = json.loads(mgr.state_path.read_text(encoding="utf-8"))
        assert state["applied"]["name"] == "test_magbias"
        assert state["applied"]["delta_b_ut"] == pytest.approx(DELTA_B)
        assert state["applied"]["verified"] is True
        assert mgr.status()["applied"]["name"] == "test_magbias"

    def test_apply_binding_mismatch_requires_force(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        responder.cal_data.ff_crc32 = 0xDEADBEEF   # 取得時と不一致
        result = mgr.apply("test_magbias")
        assert result["ok"] is False
        assert result.get("binding_mismatch") is True
        assert result["diffs"]
        result = mgr.apply("test_magbias", force=True)
        assert result["ok"], result

    def test_apply_rejected_ack_fails(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        # 飛行中拒否(bad_state)の模擬
        responder.ack_status_overrides[int(proto.MsgType.CMD_MAGBIAS_SET)] = \
            proto.TlmAck.STATUS_BAD_STATE
        result = mgr.apply("test_magbias")
        assert result["ok"] is False
        assert not (responder.cal_data.valid_flags
                    & proto.TlmCalData.VALID_MAGBIAS)

    def test_clear_sends_mode0_and_verifies(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        assert mgr.apply("test_magbias")["ok"]
        transport.clear_sent()
        result = mgr.clear()
        assert result["ok"], result
        frames = transport.frames_of_type(proto.MsgType.CMD_MAGBIAS_SET)
        msg = proto.CmdMagbiasSet.from_payload(frames[0].payload)
        assert msg.mode == proto.CmdMagbiasSet.MODE_CLEAR
        assert not (responder.cal_data.valid_flags
                    & proto.TlmCalData.VALID_MAGBIAS)
        state = json.loads(mgr.state_path.read_text(encoding="utf-8"))
        assert state["applied"] is None

    def test_apply_null_delta_profile_fails(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        profile = make_magbias_profile_dict()
        profile["delta_b_ut"] = None
        (mgr.profile_dir / "hover_only.json").write_text(
            json.dumps(profile), encoding="utf-8")
        result = mgr.apply("hover_only")
        assert result["ok"] is False
        assert "delta_b" in result["message"]


class TestFlowPassthrough:
    def test_flowcal_set_and_verify(self, magbias_session):
        """2026-07-31 改訂: 2×2 行列(対角優先引数順 m00, m11, m01, m10)。"""
        session, transport, clock, responder, mgr = magbias_session
        result = mgr.flowcal_set(455.0, 447.5, 36.25, -33.5)
        assert result["ok"], result
        assert result["verified"] is True
        assert responder.cal_data.valid_flags & proto.TlmCalData.VALID_FLOWCAL
        assert responder.cal_data.flowcal_m00 == pytest.approx(455.0)
        assert responder.cal_data.flowcal_m11 == pytest.approx(447.5)
        assert responder.cal_data.flowcal_m01 == pytest.approx(36.25)
        assert responder.cal_data.flowcal_m10 == pytest.approx(-33.5)

    def test_flowcal_set_diag_two_args(self, magbias_session):
        """旧 API 互換の 2 引数呼び出しは diag(kx,ky) の適用になること。"""
        session, transport, clock, responder, mgr = magbias_session
        result = mgr.flowcal_set(455.0, 447.5)
        assert result["ok"], result
        assert responder.cal_data.flowcal_m00 == pytest.approx(455.0)
        assert responder.cal_data.flowcal_m11 == pytest.approx(447.5)
        assert responder.cal_data.flowcal_m01 == pytest.approx(0.0)
        assert responder.cal_data.flowcal_m10 == pytest.approx(0.0)

    def test_flowcal_clear(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        assert mgr.flowcal_set(455.0, 447.5)["ok"]
        result = mgr.flowcal_clear()
        assert result["ok"], result
        assert not (responder.cal_data.valid_flags
                    & proto.TlmCalData.VALID_FLOWCAL)

    def test_flowcal_rejects_bad_values(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        assert mgr.flowcal_set("abc", 450.0)["ok"] is False
        assert mgr.flowcal_set(-1.0, 450.0)["ok"] is False
        # det(K) ≤ 0(K⁻¹ 不安定)は拒否
        assert mgr.flowcal_set(450.0, 450.0, 500.0, 500.0)["ok"] is False

    def test_flow_probe_acked(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        transport.clear_sent()
        result = mgr.flow_probe(50)
        assert result["ok"], result
        frames = transport.frames_of_type(proto.MsgType.CMD_FLOW_PROBE)
        assert len(frames) == 1
        assert proto.CmdFlowProbe.from_payload(
            frames[0].payload).n_cycles == 50

    def test_flow_probe_rejects_out_of_range(self, magbias_session):
        session, transport, clock, responder, mgr = magbias_session
        assert mgr.flow_probe(300)["ok"] is False


class TestEstMode2:
    def test_ffprofile_mode_accepts_est2(self, magbias_session):
        """/api/ffprofile の est=2(EKF2)受理(契約 §1.3 / §3)。"""
        session, transport, clock, responder, mgr = magbias_session
        result = session.ffprofile.mode(ff=2, est=2)
        assert result["ok"], result
        assert result["est"] == 2
        assert responder.cal_data.est_mode == 2
