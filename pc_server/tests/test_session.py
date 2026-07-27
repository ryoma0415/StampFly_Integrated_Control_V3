"""session: 接続/プロファイル(バイアス)/単位変換/スナップショット契約。"""

from __future__ import annotations

import json
import math

import pytest

import stampfly_protocol as proto
from core.config import parse_mac
from core.logger import FlightLogger
from core.session import MODE_POSITION, MODE_POSTURE, PHASE_CONNECTED

from conftest import halt_supervisor
from fakes import make_pose, make_tlm_ctrl, wait_until


class TestConnectAndAirframe:
    def test_connect_sends_rly_set_target_with_profile(self, session_factory):
        session, transport, clock = session_factory()
        assert session.connect("FAKE")
        halt_supervisor(session)

        targets = transport.frames_of_type(proto.MsgType.RLY_SET_TARGET)
        assert len(targets) == 1
        request = proto.RlySetTarget.from_payload(targets[0].payload)
        assert bytes(request.mac) == parse_mac("AA:BB:CC:DD:EE:01")
        assert request.wifi_channel == 3
        assert session._relay_target_ok is True
        assert session._phase == PHASE_CONNECTED

    def test_select_airframe_resends_target_when_connected(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        transport.clear_sent()

        assert session.select_airframe("zero-bias")
        targets = transport.frames_of_type(proto.MsgType.RLY_SET_TARGET)
        assert len(targets) == 1
        request = proto.RlySetTarget.from_payload(targets[0].payload)
        assert bytes(request.mac) == parse_mac("AA:BB:CC:DD:EE:02")
        assert request.wifi_channel == 1

    def test_select_unknown_airframe_fails(self, session_factory):
        session, _, _ = session_factory()
        assert session.select_airframe("no-such-frame") is False

    def test_connect_warns_when_relay_does_not_ack(self, fast_server_config,
                                                   control_config, session_factory):
        # 応答しないリレー: 接続自体は成功するがターゲット未設定の警告
        session, transport, clock = session_factory(responder=lambda f: None)
        session.serial._target_ack_timeout_s = 0.05   # テスト高速化
        assert session.connect("FAKE")
        halt_supervisor(session)
        assert session._relay_target_ok is False
        # 初回+最大3回再送 = 4送信(PROTOCOL.md RLY_TARGET_ACK)
        assert len(transport.frames_of_type(proto.MsgType.RLY_SET_TARGET)) == 4


class TestBiasApplication:
    def test_bias_added_to_outgoing_setpoint(self, session_factory):
        """プロファイルのバイアス(deg)は rad 変換のうえ送信指令に加算される。"""
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()   # 手動 step で決定的に
        transport.clear_sent()

        session.posture.step(clock())
        frames = transport.frames_of_type(proto.MsgType.CMD_SETPOINT)
        assert len(frames) == 1
        sent = proto.CmdSetpoint.from_payload(frames[0].payload)
        # ユーザ目標 0 + roll_bias 2.0deg / pitch_bias -1.5deg
        assert sent.roll_ref == pytest.approx(math.radians(2.0), abs=1e-6)
        assert sent.pitch_ref == pytest.approx(math.radians(-1.5), abs=1e-6)
        assert sent.alt_ref == pytest.approx(0.4)   # default_alt_m
        assert sent.flags == proto.CmdSetpoint.FLAG_ALT_REF_VALID

    def test_zero_bias_profile_sends_raw_setpoint(self, session_factory):
        session, transport, clock = session_factory()
        session.select_airframe("zero-bias")
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()
        transport.clear_sent()

        session.posture.step(clock())
        sent = proto.CmdSetpoint.from_payload(
            transport.frames_of_type(proto.MsgType.CMD_SETPOINT)[0].payload)
        assert sent.roll_ref == pytest.approx(0.0)
        assert sent.pitch_ref == pytest.approx(0.0)

    def test_ui_setpoint_converted_deg_to_rad(self, session_factory):
        session, _, _ = session_factory()
        session.set_setpoint_deg(5.0, -3.0, 0.5)
        assert session.posture._target_roll == pytest.approx(math.radians(5.0))
        assert session.posture._target_pitch == pytest.approx(math.radians(-3.0))
        assert session.posture._target_alt == pytest.approx(0.5)


class TestModeSwitch:
    def test_set_mode_switches_active_sender(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.posture.running

        assert session.set_mode(MODE_POSITION)
        assert wait_until(lambda: session.position.running)
        assert not session.posture.running
        assert session.mocap.connected()

        assert session.set_mode(MODE_POSTURE)
        assert wait_until(lambda: session.posture.running)
        assert not session.position.running
        assert not session.mocap.connected()

    def test_set_mode_rejected_while_armed(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.start()
        assert session.set_mode(MODE_POSITION) is False
        assert session._mode == MODE_POSTURE


class TestSnapshotContract:
    def test_state_snapshot_shape(self, session_factory):
        session, transport, clock = session_factory()
        snapshot = session.get_state_snapshot()
        assert snapshot["type"] == "state"
        data = snapshot["data"]
        assert set(data.keys()) == {"drone", "mocap", "session"}

        # TLM_STATE 未受信のあいだ drone は null(ゼロ値を実測値として配らない —
        # UI が「INIT・0.00V」を本物の異常と区別できなくなるため)
        assert data["drone"] is None

        sess = data["session"]
        for key in ("mode", "phase", "serial_connected", "airframe", "logging",
                    "log_file", "target", "setpoint", "latency_ms",
                    "relay_stats", "relay_fresh", "relay_target_ok"):
            assert key in sess, key
        assert sess["phase"] == "idle"
        assert sess["serial_connected"] is False
        assert sess["relay_fresh"] is False   # RLY_STATS 未受信
        assert set(sess["setpoint"].keys()) == {"roll_deg", "pitch_deg",
                                                "alt_m", "yaw_deg"}
        assert data["mocap"] is None       # posture モード
        assert sess["target"] is None      # posture モード

    def test_drone_angles_converted_to_deg(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)

        tlm = proto.TlmState(state=proto.FlightState.HOVER,
                             flags=proto.TlmState.FLAG_FLYING,
                             roll=0.1, pitch=-0.2, yaw=0.3, p=1.0,
                             roll_ref=0.05, voltage=3.72)
        transport.push(proto.MsgType.TLM_STATE, tlm.to_payload())
        assert wait_until(
            lambda: (session.get_state_snapshot()["data"]["drone"] or {}).get("fresh"))

        drone = session.get_state_snapshot()["data"]["drone"]
        # 受信後は TLM_STATE 全フィールド+fresh を持つ(スナップショット契約)
        for key in ("seq_echo", "elapsed_ms", "state", "flags", "reason",
                    "roll", "pitch", "yaw", "p", "q", "r",
                    "roll_ref", "pitch_ref", "alt_ref",
                    "altitude_tof", "altitude_est", "alt_velocity", "z_dot_ref",
                    "voltage", "duty_fr", "duty_fl", "duty_rr", "duty_rl",
                    "ax", "ay", "az", "loop_dt_us", "fresh"):
            assert key in drone, key
        assert drone["roll"] == pytest.approx(math.degrees(0.1))
        assert drone["pitch"] == pytest.approx(math.degrees(-0.2))
        assert drone["yaw"] == pytest.approx(math.degrees(0.3))
        assert drone["p"] == pytest.approx(math.degrees(1.0))
        assert drone["roll_ref"] == pytest.approx(math.degrees(0.05))
        assert drone["voltage"] == pytest.approx(3.72, abs=1e-4)
        assert drone["state_name"] == "HOVER"
        assert drone["flying"] is True

    def test_snapshot_drone_nonfinite_becomes_none_and_json_strict(
            self, session_factory):
        """TLM_STATE の非有限 float は None に落ち、スナップショット全体が
        厳格 JSON(allow_nan=False)で直列化できる(NaN トークンを配ると
        ブラウザ側 JSON.parse が state フレームごと捨てて UI が固まる)。"""
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)

        tlm = proto.TlmState(state=proto.FlightState.HOVER,
                             roll=float("nan"), pitch=0.1,
                             voltage=float("nan"),
                             altitude_est=float("inf"),
                             current_a=float("-inf"))
        transport.push(proto.MsgType.TLM_STATE, tlm.to_payload())
        assert wait_until(
            lambda: session.get_state_snapshot()["data"]["drone"] is not None)

        snapshot = session.get_state_snapshot()
        drone = snapshot["data"]["drone"]
        assert drone["roll"] is None          # NaN(deg 換算後も NaN)
        assert drone["voltage"] is None       # NaN
        assert drone["altitude_est"] is None  # +Inf
        assert drone["current_a"] is None     # -Inf
        assert drone["pitch"] == pytest.approx(math.degrees(0.1))
        json.dumps(snapshot, allow_nan=False)   # 漏れがあれば ValueError

    def test_mocap_snapshot_in_position_mode(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        session.set_mode(MODE_POSITION)
        session.position.on_mocap_pose(
            make_pose(x=0.1, y=-0.2, z=0.35, t=clock(), yaw_rad=math.radians(30)))

        mocap = session.get_state_snapshot()["data"]["mocap"]
        assert mocap is not None
        assert mocap["x"] == pytest.approx(0.1)
        assert mocap["y"] == pytest.approx(-0.2)
        assert mocap["z"] == pytest.approx(0.35)
        assert mocap["yaw_deg"] == pytest.approx(30.0)
        assert mocap["fresh"] is True
        assert 0.0 < mocap["confidence"] <= 1.0

        target = session.get_state_snapshot()["data"]["session"]["target"]
        assert set(target.keys()) == {"x", "y", "z"}

    def test_relay_stats_in_snapshot(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        stats = proto.RlyStats(up_frames=10, down_frames=20, crc_errors=1,
                               cobs_errors=2, espnow_send_fail=3, overflow_drops=4)
        transport.push(proto.MsgType.RLY_STATS, stats.to_payload())
        assert wait_until(lambda: session.get_state_snapshot()
                          ["data"]["session"]["relay_stats"] is not None)
        sess = session.get_state_snapshot()["data"]["session"]
        assert sess["relay_stats"] == {"up_frames": 10, "down_frames": 20,
                                       "crc_errors": 1, "cobs_errors": 2,
                                       "espnow_send_fail": 3, "overflow_drops": 4}
        assert sess["relay_fresh"] is True

    def test_relay_fresh_is_time_based_not_content_based(self, session_factory):
        """リレー鮮度は RLY_STATS の受信時刻で判定する(counter 静止でも fresh)。

        ターゲット未設定中に拒否された上りフレームはどの counter にも計上されない
        (router.cpp 仕様)ため、内容変化ベースの判定では「リレー生存・ターゲット
        未設定」とリレー断が区別できない。
        """
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        fresh_s = session.server_config["freshness"]["relay_stats_fresh_s"]

        stats = proto.RlyStats()   # 全counterゼロ(内容は以後不変)
        transport.push(proto.MsgType.RLY_STATS, stats.to_payload())
        assert wait_until(lambda: session.get_state_snapshot()
                          ["data"]["session"]["relay_fresh"])

        # 閾値超過 → 鮮度落ち
        clock.advance(fresh_s + 0.1)
        assert session.get_state_snapshot()["data"]["session"]["relay_fresh"] is False

        # 同一内容の再受信でも鮮度は回復する(時刻ベース判定の核心)
        transport.push(proto.MsgType.RLY_STATS, stats.to_payload())
        assert wait_until(lambda: session.get_state_snapshot()
                          ["data"]["session"]["relay_fresh"])


class TestEventsAndLogText:
    def test_tlm_event_queued_for_ui(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        self._drain(session)

        event = proto.TlmEvent(state=proto.FlightState.TAKEOFF,
                               prev_state=proto.FlightState.WAIT,
                               reason=proto.Reason.START_CMD, voltage=3.9)
        transport.push(proto.MsgType.TLM_EVENT, event.to_payload())
        assert wait_until(lambda: any(
            m["type"] == "event" for m in self._drain(session, keep=True)))
        messages = [m for m in self._drained if m["type"] == "event"]
        data = messages[-1]["data"]
        assert data["state_name"] == "TAKEOFF"
        assert data["reason_name"] == "START_CMD"
        assert data["voltage"] == pytest.approx(3.9, abs=1e-4)

    def test_tlm_event_nan_voltage_becomes_none(self, session_factory):
        """イベント経路(TLM_EVENT)の voltage も非有限なら None に落ちる。"""
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        self._drain(session)

        event = proto.TlmEvent(state=proto.FlightState.TAKEOFF,
                               prev_state=proto.FlightState.WAIT,
                               reason=proto.Reason.START_CMD,
                               voltage=float("nan"))
        transport.push(proto.MsgType.TLM_EVENT, event.to_payload())
        assert wait_until(lambda: any(
            m["type"] == "event" for m in self._drain(session, keep=True)))
        message = [m for m in self._drained if m["type"] == "event"][-1]
        assert message["data"]["voltage"] is None
        json.dumps(message, allow_nan=False)   # 漏れがあれば ValueError

    def test_log_text_origin_mapping(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        self._drain(session)

        log = proto.LogText(origin=proto.LogText.ORIGIN_DRONE, text="hello 機体")
        transport.push(proto.MsgType.LOG_TEXT, log.to_payload())
        assert wait_until(lambda: any(
            m["type"] == "log" and m["origin"] == "drone"
            for m in self._drain(session, keep=True)))
        message = [m for m in self._drained
                   if m["type"] == "log" and m["origin"] == "drone"][-1]
        assert message["line"] == "hello 機体"

    _drained: list = []

    def _drain(self, session, keep=False):
        if not keep:
            self._drained = []
        while True:
            try:
                self._drained.append(session.events.get_nowait())
            except Exception:
                break
        return self._drained


class TestLoggingIntegration:
    """飛行ログの寿命 = START(CMD_START 受理)〜飛行終了(+自動OFF)。"""

    def test_logging_on_is_reservation_until_start(self, tmp_path,
                                                   session_factory):
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()

        session.set_logging(True)
        assert not session.logger.active       # 接続中でも予約のみ

        assert session.start()                 # CMD_START 受理 → ファイルを開く
        assert session.logger.active
        log_file = session.logger.file_path
        assert log_file is not None
        assert log_file.name.endswith("_posture.csv")

        session.posture.step(clock())          # 1行書かれる
        session.set_logging(False)             # OFF は従来どおり即閉じ
        assert not session.logger.active
        content = log_file.read_text(encoding="utf-8").strip().splitlines()
        assert len(content) == 2               # ヘッダ + 1行

    def test_set_logging_during_flight_opens_immediately(self, tmp_path,
                                                         session_factory):
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()
        assert session.start()
        session._update_phase_from_drone(proto.FlightState.HOVER,
                                         proto.TlmState.FLAG_FLYING)
        assert not session.logger.active

        session.set_logging(True)              # 飛行中の ON → 即開く
        assert session.logger.active

    def test_landing_closes_log_and_auto_off(self, tmp_path, session_factory):
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()
        session.set_logging(True)
        assert session.start()
        session._update_phase_from_drone(proto.FlightState.HOVER,
                                         proto.TlmState.FLAG_FLYING)
        assert session.logger.active

        # 着陸(flying → connected)→ ファイル close + 予約フラグ自動 OFF
        session._update_phase_from_drone(proto.FlightState.WAIT, 0)
        assert session._phase == PHASE_CONNECTED
        assert not session.logger.active
        assert session._logging_enabled is False

    def test_start_grace_expiry_closes_log_and_auto_off(self, tmp_path,
                                                        session_factory):
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()
        session.set_logging(True)
        assert session.start()
        assert session.logger.active

        # START 猶予切れ(armed のまま地上状態が続く)→ close + 自動 OFF
        clock.advance(session._start_grace_s + 0.1)
        session._update_phase_from_drone(proto.FlightState.WAIT, 0)
        assert session._phase == PHASE_CONNECTED
        assert not session.logger.active
        assert session._logging_enabled is False

    def test_full_flight_log_scenario(self, tmp_path, session_factory):
        """一連の動作: 接続→トグルON(未作成)→START(作成+行記録)→着陸(close+自動OFF)。"""
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()

        # トグル ON は予約のみ — ファイルはまだ作られない
        session.set_logging(True)
        assert not session.logger.active
        assert list(tmp_path.glob("*.csv")) == []

        # START(CMD_START 受理)でファイルが開き、送信ごとに行が書かれる
        assert session.start()
        assert session.logger.active
        log_file = session.logger.file_path
        assert log_file is not None and log_file.exists()
        session.posture.step(clock())
        session._update_phase_from_drone(proto.FlightState.HOVER,
                                         proto.TlmState.FLAG_FLYING)
        session.posture.step(clock())

        # 着陸イベント(flying → connected)→ close + 予約フラグ自動 OFF
        session._update_phase_from_drone(proto.FlightState.WAIT, 0)
        assert session._phase == PHASE_CONNECTED
        assert not session.logger.active
        assert session._logging_enabled is False
        lines = log_file.read_text(encoding="utf-8").strip().splitlines()
        assert len(lines) == 3                 # ヘッダ + 2行(armed/flying)

    def test_teardown_closes_log_and_auto_off(self, tmp_path, session_factory):
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()
        session.set_logging(True)
        assert session.start()
        assert session.logger.active

        session.disconnect()                   # teardown 経路
        assert not session.logger.active
        assert session._logging_enabled is False

    def test_tlm_ctrl_columns_written(self, tmp_path, session_factory):
        """最新 TLM_CTRL(制御ループ診断)のスナップショットが行に転記される。"""
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()

        ctrl = make_tlm_ctrl(roll_rate_ref=0.10, pitch_rate_ref=-0.20,
                             yaw_rate_ref=0.05,
                             flags=(proto.TlmCtrl.FLAG_YAW_CTRL_ACTIVE
                                    | proto.TlmCtrl.FLAG_FLYING))
        transport.push(proto.MsgType.TLM_CTRL, ctrl.to_payload())
        assert wait_until(lambda: session._tlm_ctrl is not None)

        session.set_logging(True)
        assert session.start()
        log_file = session.logger.file_path
        session.posture.step(clock())
        session.set_logging(False)

        lines = log_file.read_text(encoding="utf-8").strip().splitlines()
        header = lines[0].split(",")
        assert len(header) == 115              # v5 契約(v4 109列+MoCap 6列)
        row = lines[1].split(",")

        def cell(name: str) -> str:
            return row[header.index(name)]

        assert float(cell("tlm_roll_rate_ref_rad_s")) == pytest.approx(0.10)
        assert float(cell("tlm_pitch_rate_ref_rad_s")) == pytest.approx(-0.20)
        assert float(cell("tlm_yaw_rate_ref_rad_s")) == pytest.approx(0.05)
        assert int(cell("tlm_ctrl_flags")) == int(
            proto.TlmCtrl.FLAG_YAW_CTRL_ACTIVE | proto.TlmCtrl.FLAG_FLYING)
        assert cell("tlm_ctrl_age_ms") != ""
        # PID 成分の転記順(roll_p が先頭、yaw_rate_d が末尾)
        assert float(cell("tlm_pid_roll_ang_p")) == pytest.approx(ctrl.pid_ang[0])
        assert float(cell("tlm_pid_yaw_ang_d")) == pytest.approx(ctrl.pid_ang[8])
        assert float(cell("tlm_pid_roll_rate_p")) == pytest.approx(ctrl.pid_rate[0])
        assert float(cell("tlm_pid_yaw_rate_d")) == pytest.approx(ctrl.pid_rate[8])

    def test_tlm_ctrl_absent_leaves_columns_empty(self, tmp_path,
                                                  session_factory):
        """TLM_CTRL 未受信の間は tlm_ctrl_* 列が空欄になる(旧ファーム互換)。"""
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()
        session.set_logging(True)
        assert session.start()
        log_file = session.logger.file_path
        session.posture.step(clock())
        session.set_logging(False)

        lines = log_file.read_text(encoding="utf-8").strip().splitlines()
        header = lines[0].split(",")
        row = lines[1].split(",")
        assert row[header.index("tlm_ctrl_age_ms")] == ""
        assert row[header.index("tlm_pid_roll_ang_p")] == ""
