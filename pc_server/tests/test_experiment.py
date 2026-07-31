"""v2 実験モード: CMD_MODE 遷移・モーターテスト・ACK 待ち・スイープ/シーケンス。"""

from __future__ import annotations

import json
import math
import threading
import time

import pytest

import stampfly_protocol as proto

import core.experiment as experiment
from core.session import MODE_EXPERIMENT, MODE_POSTURE

from conftest import halt_supervisor
from fakes import FakeDroneResponder, make_tlm_exp, wait_until


@pytest.fixture
def fast_ack_config(server_config):
    """ACK タイムアウト・キープアライブを短縮した設定(実時間テスト用)。"""
    server_config["failsafe"]["command_ack_timeout_s"] = 0.1
    server_config["experiment"]["motor_keepalive_s"] = 0.05
    return server_config


@pytest.fixture
def exp_session(fast_ack_config, session_factory):
    """実験モードに入った接続済みセッション(フェイク機体つき)。"""
    responder = FakeDroneResponder()
    session, transport, clock = session_factory(responder=responder)
    session.connect("FAKE")
    halt_supervisor(session)
    assert session.set_mode(MODE_EXPERIMENT)
    return session, transport, clock, responder


def start_exp_feeder(session, transport, **tlm_kwargs):
    """TLM_EXP を 5ms 周期で注入するフィーダスレッド(25Hz 相当以上)。

    最初のサンプルが hub に届くまで待ってから返す(スイープ開始の
    鮮度チェックがフレーク しないように)。
    """
    stop = threading.Event()

    def run():
        while not stop.is_set():
            transport.push(proto.MsgType.TLM_EXP,
                           make_tlm_exp(**tlm_kwargs).to_payload())
            time.sleep(0.005)

    thread = threading.Thread(target=run, daemon=True)
    thread.start()
    assert wait_until(lambda: session.experiment.exp_age_s() is not None)
    return stop


class TestSendWithAck:
    def test_ack_roundtrip(self, exp_session):
        session, transport, clock, responder = exp_session
        ack = session.serial.send_with_ack(proto.MsgType.CMD_FF_ANCHOR)
        assert ack is not None
        assert ack.acked_type == proto.MsgType.CMD_FF_ANCHOR
        assert ack.status == proto.TlmAck.STATUS_OK

    def test_retry_after_lost_ack(self, exp_session):
        session, transport, clock, responder = exp_session
        responder.drop_first_ack_types.add(int(proto.MsgType.CMD_FF_ANCHOR))
        transport.clear_sent()
        ack = session.serial.send_with_ack(proto.MsgType.CMD_FF_ANCHOR)
        assert ack is not None and ack.status == proto.TlmAck.STATUS_OK
        # 初回 ACK ロスト → 1回再送で成功(計2フレーム)
        assert len(transport.frames_of_type(proto.MsgType.CMD_FF_ANCHOR)) == 2

    def test_timeout_returns_none(self, fast_ack_config, session_factory):
        # 既定レスポンダ(リレーのみ、機体なし)では TLM_ACK が来ない
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        transport.clear_sent()
        ack = session.serial.send_with_ack(proto.MsgType.CMD_FF_ANCHOR)
        assert ack is None
        # 初回+最大2回再送 = 3送信(契約 §3.4 の ACK 一般則)
        assert len(transport.frames_of_type(proto.MsgType.CMD_FF_ANCHOR)) == 3


class TestExperimentMode:
    def test_enter_sends_cmd_mode_and_stops_sender(self, exp_session):
        session, transport, clock, responder = exp_session
        frames = transport.frames_of_type(proto.MsgType.CMD_MODE)
        assert len(frames) == 1
        sent = proto.CmdMode.from_payload(frames[0].payload)
        assert sent.mode == proto.CmdMode.MODE_MOTOR_TEST
        assert session.experiment.active
        assert not session.posture.running   # 50Hz 送信は停止

    def test_enter_rejected_by_bad_state(self, fast_ack_config,
                                         session_factory):
        responder = FakeDroneResponder()
        responder.ack_status_overrides[int(proto.MsgType.CMD_MODE)] = \
            proto.TlmAck.STATUS_BAD_STATE
        session, transport, clock = session_factory(responder=responder)
        session.connect("FAKE")
        halt_supervisor(session)
        assert not session.set_mode(MODE_EXPERIMENT)
        assert session._mode == MODE_POSTURE     # 元モードに復帰
        assert session.posture.running           # 送信再開

    def test_start_rejected_in_experiment(self, exp_session):
        session, transport, clock, responder = exp_session
        assert not session.start()

    def test_exit_returns_to_flight_mode(self, exp_session):
        session, transport, clock, responder = exp_session
        transport.clear_sent()
        assert session.set_mode(MODE_POSTURE)
        frames = transport.frames_of_type(proto.MsgType.CMD_MODE)
        assert len(frames) == 1
        assert proto.CmdMode.from_payload(frames[0].payload).mode \
            == proto.CmdMode.MODE_FLIGHT
        assert not session.experiment.active
        assert session.posture.running

    def test_stop_deactivates_and_reactivate(self, exp_session):
        session, transport, clock, responder = exp_session
        session.experiment.motor_start(0.2, 0x0F)
        session.stop()   # SPACE 緊急停止相当(CMD_STOP で機体は WAIT へ)
        assert not session.experiment.motor_status()["running"]
        assert session._experiment_active is False
        # 再有効化で CMD_MODE(1) を送り直す
        transport.clear_sent()
        assert session.activate_experiment()
        assert session._experiment_active is True


class TestMotorTest:
    def test_motor_run_and_keepalive(self, exp_session):
        session, transport, clock, responder = exp_session
        transport.clear_sent()
        result = session.motor_start(0.3, 0x05)
        assert result["ok"]
        # キープアライブ(0.05s に短縮)による再送を確認
        assert wait_until(lambda: len(
            transport.frames_of_type(proto.MsgType.CMD_MOTOR_RUN)) >= 3)
        sent = proto.CmdMotorRun.from_payload(
            transport.frames_of_type(proto.MsgType.CMD_MOTOR_RUN)[0].payload)
        assert sent.duty == pytest.approx(0.3)
        assert sent.mask == 0x05

    def test_motor_stop_sends_three(self, exp_session):
        session, transport, clock, responder = exp_session
        session.motor_start(0.2, 0x0F)
        transport.clear_sent()
        result = session.motor_stop()
        assert result["ok"]
        assert len(transport.frames_of_type(proto.MsgType.CMD_MOTOR_STOP)) \
            == experiment.MOTOR_STOP_REPEAT

    def test_duty_clamped_to_max(self, exp_session):
        session, transport, clock, responder = exp_session
        result = session.motor_start(5.0, 0x0F)
        assert result["duty"] == pytest.approx(1.0)   # motor_max_duty

    def test_rejected_outside_experiment(self, fast_ack_config,
                                         session_factory):
        session, transport, clock = session_factory(
            responder=FakeDroneResponder())
        session.connect("FAKE")
        halt_supervisor(session)
        assert not session.motor_start(0.2, 0x0F)["ok"]


class TestTlmExpFlow:
    def test_latest_sample_updated(self, exp_session):
        session, transport, clock, responder = exp_session
        transport.push(proto.MsgType.TLM_EXP,
                       make_tlm_exp(current_a=1.25, vbat_v=3.85).to_payload())
        assert wait_until(
            lambda: session.experiment.latest_sample()[0] is not None)
        sample, age = session.experiment.latest_sample()
        assert sample["current_a"] == pytest.approx(1.25)
        assert sample["vbat_v"] == pytest.approx(3.85)
        assert sample["cv"] == 1

    def test_snapshot_exposes_accel(self, exp_session):
        """WS スナップショットの exp に ax/ay/az [g] が載る(6面キャリブUIのライブ表示)。"""
        session, transport, clock, responder = exp_session
        transport.push(proto.MsgType.TLM_EXP,
                       make_tlm_exp(ax=0.25, ay=-0.5, az=1.0).to_payload())
        assert wait_until(
            lambda: session.experiment.latest_sample()[0] is not None)
        exp = session.get_state_snapshot()["data"]["session"]["experiment"]
        assert exp is not None and exp["exp"] is not None
        assert exp["exp"]["ax"] == pytest.approx(0.25)
        assert exp["exp"]["ay"] == pytest.approx(-0.5)
        assert exp["exp"]["az"] == pytest.approx(1.0)

    def test_snapshot_accel_nan_becomes_none(self, exp_session):
        """非有限の加速度は None に落ちる(WS の JSON を壊さない)。"""
        session, transport, clock, responder = exp_session
        transport.push(proto.MsgType.TLM_EXP,
                       make_tlm_exp(ax=float("nan"), ay=float("inf"),
                                    az=1.0).to_payload())
        assert wait_until(
            lambda: session.experiment.latest_sample()[0] is not None)
        exp = session.get_state_snapshot()["data"]["session"]["experiment"]
        assert exp["exp"]["ax"] is None
        assert exp["exp"]["ay"] is None
        assert exp["exp"]["az"] == pytest.approx(1.0)

    def test_snapshot_exp_nonfinite_becomes_none_and_json_strict(
            self, exp_session):
        """exp dict の全 float(電流・電圧・磁気ベクトル・角度・duty)も
        非有限なら None に落ち、スナップショット全体が厳格 JSON で通る。"""
        session, transport, clock, responder = exp_session
        transport.push(proto.MsgType.TLM_EXP,
                       make_tlm_exp(current_a=float("nan"),
                                    vbat_v=float("inf"),
                                    b_raw=(float("nan"), -5.0, 30.0),
                                    b_cal=(11.0, float("-inf"), 29.0),
                                    roll=float("nan"), yaw=0.2,
                                    duty_cmd=float("nan")).to_payload())
        assert wait_until(
            lambda: session.experiment.latest_sample()[0] is not None)
        snapshot = session.get_state_snapshot()
        exp = snapshot["data"]["session"]["experiment"]["exp"]
        assert exp["current_a"] is None
        assert exp["vbat_v"] is None
        assert exp["b_raw"] == [None, pytest.approx(-5.0), pytest.approx(30.0)]
        assert exp["b_cal"] == [pytest.approx(11.0), None, pytest.approx(29.0)]
        assert exp["roll_deg"] is None
        assert exp["yaw_deg"] == pytest.approx(math.degrees(0.2))
        assert exp["duty_cmd"] is None
        json.dumps(snapshot, allow_nan=False)   # 漏れがあれば ValueError

    def test_cal3d_collects_raw_field(self, exp_session):
        session, transport, clock, responder = exp_session
        session.experiment.cal3d_start()
        for _ in range(5):
            transport.push(proto.MsgType.TLM_EXP,
                           make_tlm_exp(b_raw=(1.0, 2.0, 3.0)).to_payload())
        assert wait_until(
            lambda: session.experiment.cal3d_status()["sample_count"] >= 5)
        samples = session.experiment.cal3d_stop()
        assert samples[0] == [1.0, 2.0, 3.0]


@pytest.fixture
def short_sweep(monkeypatch, tmp_path):
    """スイープのタイミング定数を短縮する(状態機械の検証用)。"""
    monkeypatch.setattr(experiment, "SWEEP_DUTY_STEPS", [0.2, 0.4])
    monkeypatch.setattr(experiment, "SWEEP_BASE_S", 0.06)
    monkeypatch.setattr(experiment, "SWEEP_SETTLE_S", 0.04)
    monkeypatch.setattr(experiment, "SWEEP_MEASURE_S", 0.06)
    monkeypatch.setattr(experiment, "SWEEP_GAP_S", 0.08)
    monkeypatch.setattr(experiment, "SWEEP_GAP_SETTLE_S", 0.02)
    monkeypatch.setattr(experiment, "SEQUENCE_COOLDOWN_S", 0.05)
    return tmp_path


class TestSweepRunner:
    def test_full_sweep_writes_csv_and_meta(self, exp_session, short_sweep):
        session, transport, clock, responder = exp_session
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport)
        try:
            result = session.sweep_start(0x0F, "up", {"location": "test"})
            assert result["ok"], result
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=10.0)
        finally:
            feeder.set()
        status = session.experiment.sweep.status()
        assert status["phase"] == "done", status
        last = status["last_result"]
        csv_path = short_sweep / last["samples"]
        meta_path = short_sweep / last["meta"]
        assert csv_path.is_file() and meta_path.is_file()

        import csv as csv_mod
        import json
        with csv_path.open() as fp:
            rows = list(csv_mod.DictReader(fp))
        assert list(rows[0].keys()) == experiment.SweepRunner.SAMPLE_FIELDS
        phases = {r["phase"] for r in rows}
        assert {"base", "settle", "measure", "gap_settle", "baseline"} <= phases
        measure = [r for r in rows if r["phase"] == "measure"]
        assert measure and all(r["dB_cor_x"] != "" for r in measure)

        meta = json.loads(meta_path.read_text())
        assert meta["schema"] == "stampfly_sweep_meta"
        assert meta["version"] == 1
        assert meta["method"] == "bracketed_baseline"
        assert meta["pattern"] == "up"
        assert meta["motors"] == "FL+FR+RL+RR"
        assert meta["idle_current_a"] is not None
        assert meta["sample_count"] == len(rows)

        # モーターは各 duty で駆動され、最後に停止している
        duties = {round(proto.CmdMotorRun.from_payload(f.payload).duty, 2)
                  for f in transport.frames_of_type(proto.MsgType.CMD_MOTOR_RUN)}
        assert {0.2, 0.4} <= duties
        assert transport.frames_of_type(proto.MsgType.CMD_MOTOR_STOP)

        # 受入条件: data_analysis が無改修でこの CSV/meta を読めること
        import sys
        from core import REPO_DIR
        da_dir = str(REPO_DIR / "data_analysis")
        if da_dir not in sys.path:
            sys.path.insert(0, da_dir)
        from ff_params import core as ff_core
        stem = last["samples"][:-len("_samples.csv")]
        run = ff_core.load_run(stem, short_sweep)
        agg = ff_core.aggregate(run)
        assert agg["idle"] == meta["idle_current_a"]
        assert {round(s["duty"], 2) for s in agg["steps"]} == {0.2, 0.4}

    def test_abort_stops_motors(self, exp_session, short_sweep, monkeypatch):
        session, transport, clock, responder = exp_session
        monkeypatch.setattr(experiment, "SWEEP_MEASURE_S", 5.0)
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport)
        try:
            assert session.sweep_start(0x01, "up", None)["ok"]
            assert wait_until(
                lambda: session.experiment.sweep.status()["phase"]
                in ("settle", "measure"), timeout=5.0)
            session.experiment.sweep.abort()
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=5.0)
        finally:
            feeder.set()
        assert session.experiment.sweep.status()["phase"] == "aborted"
        assert transport.frames_of_type(proto.MsgType.CMD_MOTOR_STOP)

    def test_manual_motor_rejected_while_running(self, exp_session,
                                                 short_sweep, monkeypatch):
        """スイープ実行中の motor_start / motor_set はサーバ側で拒否する。

        UI の busy ゲートだけに頼ると、別クライアントからの操作が measure 中の
        duty/mask を上書きし CSV の duty_cmd と実回転が食い違う。
        """
        session, transport, clock, responder = exp_session
        monkeypatch.setattr(experiment, "SWEEP_MEASURE_S", 5.0)
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport)
        try:
            assert session.sweep_start(0x01, "up", None)["ok"]
            assert wait_until(
                lambda: session.experiment.sweep.status()["phase"]
                in ("settle", "measure"), timeout=5.0)
            result = session.motor_start(0.9, 0x0F)
            assert result["ok"] is False
            assert "実行中" in result["message"]
            result = session.motor_apply(0.9)
            assert result["ok"] is False
            assert "実行中" in result["message"]
            # hub の duty/mask はスイープの値のまま(キープアライブが 0.9 を
            # 送らない)
            assert session.experiment.motor_status()["duty"] != 0.9
            # motor_stop(緊急停止経路)は従来どおり無条件で受理される
            assert session.motor_stop()["ok"] is True
            session.experiment.sweep.abort()
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=5.0)
        finally:
            feeder.set()

    def test_undervolt_aborts(self, exp_session, short_sweep):
        session, transport, clock, responder = exp_session
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport, vbat_v=2.9)   # < 3.0V
        try:
            assert session.sweep_start(0x0F, "up", None)["ok"]
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=5.0)
        finally:
            feeder.set()
        status = session.experiment.sweep.status()
        assert status["phase"] == "error"
        assert "過放電" in status["error"]


class TestExpRecorder:
    """実験計測ログ(EKF/FF 性能ログ): CSV/meta 出力・計測中ガード・自動停止。"""

    # 仕様 T1-2 の列順(実装定数のリグレッション検知のため文字通りに固定。
    # v2 でフロー/ToF 列を末尾追加 — stampfly_explog_meta v2)
    EXPECTED_FIELDS = [
        "t_s", "exp_elapsed_ms", "duty_cmd", "motors_mask", "motors",
        "cv", "mag_fresh",
        "current_a", "vbat_v", "shunt_uv",
        "bx_raw", "by_raw", "bz_raw", "bx_cal", "by_cal", "bz_cal",
        "imu_temp_c",
        "roll_deg", "pitch_deg", "yaw_madgwick_deg",
        "p_rad_s", "q_rad_s", "r_rad_s", "ax_g", "ay_g", "az_g",
        "yaw_est_deg", "yaw_gyro_int_deg", "yaw_ref_deg",
        "db_hat_x_ut", "db_hat_y_ut", "bm_x_ut", "bm_y_ut",
        "nis", "ffg", "ff_status", "tlm_state_age_ms",
        "flow_vx_mps", "flow_vy_mps", "flow_squal", "flow_status",
        "flow_dt_ms", "alt_tof_m",
    ]

    @staticmethod
    def _recorder(session, tmp_path):
        session.experiment.recorder.logs_dir = tmp_path
        return session.experiment.recorder

    @staticmethod
    def _push_samples(session, transport, count, **tlm_kwargs):
        before = session.experiment.recorder.status()["samples"]
        for _ in range(count):
            transport.push(proto.MsgType.TLM_EXP,
                           make_tlm_exp(**tlm_kwargs).to_payload())
        assert wait_until(
            lambda: session.experiment.recorder.status()["samples"]
            >= before + count)

    def test_record_writes_csv_and_meta(self, exp_session, tmp_path):
        session, transport, clock, responder = exp_session
        recorder = self._recorder(session, tmp_path)
        # 最新 TLM_STATE スナップショット列の供給源
        tlm_state = proto.TlmState(
            state=proto.FlightState.WAIT, yaw_est_rad=0.5,
            yaw_gyro_int_rad=0.25, yaw_ref_rad=-0.1,
            db_hat_x_ut=1.5, db_hat_y_ut=-2.5, bm_x_ut=3.0, bm_y_ut=-4.0,
            nis=1.25, ffg=3, ff_status=0x15,
            altitude_tof=0.42,
            flow_vx_mps=0.12, flow_vy_mps=-0.34, flow_squal=88,
            flow_status=0x1F, flow_dt_ms=11)
        transport.push(proto.MsgType.TLM_STATE, tlm_state.to_payload())
        assert wait_until(lambda: session._tlm_state_snapshot()[0] is not None)

        result = session.exp_record_start()
        assert result["ok"], result
        file_name = result["file"]
        assert file_name.startswith("explog_") and file_name.endswith(".csv")

        self._push_samples(session, transport, 3, current_a=1.25, vbat_v=3.85,
                           roll=0.1, yaw=0.2, duty_cmd=0.4, motors_mask=0x0F)

        # status payload(experiment.recording)にも状態が載る(T1-6)
        recording = session.get_state_snapshot()["data"]["session"][
            "experiment"]["recording"]
        assert recording["active"] is True
        assert recording["file"] == file_name
        assert recording["samples"] >= 3

        result = session.exp_record_stop()
        assert result["ok"], result
        assert not recorder.is_recording()

        import csv as csv_mod
        with (tmp_path / file_name).open() as fp:
            rows = list(csv_mod.DictReader(fp))
        assert list(rows[0].keys()) == self.EXPECTED_FIELDS
        assert list(rows[0].keys()) == experiment.EXPLOG_FIELDS
        row = rows[0]
        assert float(row["current_a"]) == pytest.approx(1.25)
        assert float(row["vbat_v"]) == pytest.approx(3.85)
        assert row["cv"] == "1" and row["mag_fresh"] == "1"
        assert row["motors_mask"] == "15" and row["motors"] == "FL+FR+RL+RR"
        # 姿勢角は deg、角速度は rad/s のまま(T1-2)
        assert float(row["roll_deg"]) == pytest.approx(math.degrees(0.1))
        assert float(row["yaw_madgwick_deg"]) == pytest.approx(
            math.degrees(0.2))
        assert float(row["p_rad_s"]) == pytest.approx(0.0)
        # TLM_STATE スナップショット列(deg 換算+鮮度)
        assert float(row["yaw_est_deg"]) == pytest.approx(math.degrees(0.5))
        assert float(row["yaw_gyro_int_deg"]) == pytest.approx(
            math.degrees(0.25))
        assert float(row["yaw_ref_deg"]) == pytest.approx(math.degrees(-0.1))
        assert float(row["db_hat_x_ut"]) == pytest.approx(1.5)
        assert float(row["nis"]) == pytest.approx(1.25)
        assert row["ffg"] == "3" and row["ff_status"] == "21"
        assert row["tlm_state_age_ms"] != ""
        # v2: フロー/ToF 列(TLM_STATE スナップショット由来)
        assert float(row["flow_vx_mps"]) == pytest.approx(0.12)
        assert float(row["flow_vy_mps"]) == pytest.approx(-0.34)
        assert row["flow_squal"] == "88" and row["flow_status"] == "31"
        assert row["flow_dt_ms"] == "11"
        assert float(row["alt_tof_m"]) == pytest.approx(0.42)

        meta = json.loads(
            (tmp_path / result["meta"]).read_text(encoding="utf-8"))
        assert meta["schema"] == "stampfly_explog_meta"
        assert meta["version"] == experiment.EXPLOG_META_VERSION
        assert meta["aborted"] is False
        assert meta["sample_count"] == len(rows)
        assert meta["started_at_epoch"] <= meta["ended_at_epoch"]
        for key in ("started_at", "ended_at", "ff_state",
                    "mag3d_file_info", "geomag_profile"):
            assert key in meta, key

    def test_tlm_state_columns_empty_when_missing(self, exp_session, tmp_path):
        """TLM_STATE 未受信なら該当列は空欄(T1-2)。"""
        session, transport, clock, responder = exp_session
        self._recorder(session, tmp_path)
        result = session.exp_record_start()
        assert result["ok"], result
        self._push_samples(session, transport, 1)
        result = session.exp_record_stop()
        assert result["ok"], result

        import csv as csv_mod
        with (tmp_path / result["file"]).open() as fp:
            rows = list(csv_mod.DictReader(fp))
        for key in ("yaw_est_deg", "yaw_ref_deg", "nis", "ffg",
                    "ff_status", "tlm_state_age_ms",
                    "flow_vx_mps", "flow_vy_mps", "flow_squal",
                    "flow_status", "flow_dt_ms", "alt_tof_m"):
            assert rows[0][key] == "", key
        assert rows[0]["current_a"] != ""

    def test_guards_while_recording(self, exp_session, tmp_path):
        """計測中: スイープ/シーケンス開始と部分マスクの回転を拒否(T1-4)。"""
        session, transport, clock, responder = exp_session
        self._recorder(session, tmp_path)
        transport.push(proto.MsgType.TLM_EXP, make_tlm_exp().to_payload())
        assert wait_until(lambda: session.experiment.exp_age_s() is not None)
        assert session.exp_record_start()["ok"]

        result = session.sweep_start(0x0F, "up", None)
        assert result["ok"] is False and "計測" in result["message"]
        result = session.sequence_start([0x1], "up", None, 3.5)
        assert result["ok"] is False and "計測" in result["message"]
        # CMD_MOTOR_RUN は全モーター(0xF)のみ受理
        result = session.motor_start(0.2, 0x01)
        assert result["ok"] is False and "全モーター" in result["message"]
        assert session.motor_start(0.2, 0x0F)["ok"] is True
        # モーター停止は常に許可(緊急停止経路)
        assert session.motor_stop()["ok"] is True
        assert session.exp_record_stop()["ok"]

    def test_full_recording_scenario(self, exp_session, tmp_path):
        """一連の動作: 計測開始→TLM_EXP流入→行記録→スイープ拒否→
        mask=0x3 拒否→0xF 受理→停止→meta 生成(aborted=false)。"""
        session, transport, clock, responder = exp_session
        recorder = self._recorder(session, tmp_path)

        result = session.exp_record_start()
        assert result["ok"], result
        file_name = result["file"]

        # TLM_EXP 流入 → 行が記録される
        self._push_samples(session, transport, 5, duty_cmd=0.3,
                           motors_mask=0x0F)
        assert recorder.status()["samples"] >= 5

        # 計測中の制限(サーバ側が正)
        result = session.sweep_start(0x0F, "up", None)
        assert result["ok"] is False and "計測" in result["message"]
        result = session.motor_start(0.2, 0x03)         # FL+FR のみ → 拒否
        assert result["ok"] is False and "全モーター" in result["message"]
        assert session.motor_start(0.2, 0x0F)["ok"] is True  # 全モーターは受理
        assert session.motor_stop()["ok"] is True

        # 停止 → CSV と meta(aborted=false)が揃う
        result = session.exp_record_stop()
        assert result["ok"], result
        assert not recorder.is_recording()

        import csv as csv_mod
        with (tmp_path / file_name).open() as fp:
            rows = list(csv_mod.DictReader(fp))
        assert len(rows) >= 5
        assert list(rows[0].keys()) == experiment.EXPLOG_FIELDS
        meta = json.loads(
            (tmp_path / result["meta"]).read_text(encoding="utf-8"))
        assert meta["aborted"] is False
        assert meta["sample_count"] == len(rows)

    def test_start_rejected_while_sweep_running(self, exp_session,
                                                short_sweep, monkeypatch):
        session, transport, clock, responder = exp_session
        monkeypatch.setattr(experiment, "SWEEP_MEASURE_S", 5.0)
        session.experiment.sweep.result_dir = short_sweep
        self._recorder(session, short_sweep)
        feeder = start_exp_feeder(session, transport)
        try:
            assert session.sweep_start(0x01, "up", None)["ok"]
            assert wait_until(
                lambda: session.experiment.sweep.status()["phase"]
                in ("settle", "measure"), timeout=5.0)
            result = session.exp_record_start()
            assert result["ok"] is False
            assert "実行中" in result["message"]
            session.experiment.sweep.abort()
            assert wait_until(
                lambda: not session.experiment.sweep.is_running(), timeout=5.0)
        finally:
            feeder.set()

    def test_mode_leave_aborts_recording(self, exp_session, tmp_path):
        """モード離脱(実験無効化)で自動停止し meta に aborted=true(T1-5)。"""
        session, transport, clock, responder = exp_session
        recorder = self._recorder(session, tmp_path)
        result = session.exp_record_start()
        assert result["ok"], result
        self._push_samples(session, transport, 1)
        assert session.set_mode(MODE_POSTURE)
        assert not recorder.is_recording()
        meta_name = result["file"][:-len(".csv")] + "_meta.json"
        meta = json.loads((tmp_path / meta_name).read_text(encoding="utf-8"))
        assert meta["aborted"] is True
        assert meta["sample_count"] >= 1

    def test_space_stop_keeps_recording(self, exp_session, tmp_path):
        """SPACE 緊急停止(CMD_STOP)では計測を止めない(T1-5)。"""
        session, transport, clock, responder = exp_session
        recorder = self._recorder(session, tmp_path)
        assert session.exp_record_start()["ok"]
        session.stop()   # モーター停止+実験無効化(hub.deactivate は通らない)
        assert recorder.is_recording()
        # 停止過渡のサンプルも記録され続ける
        self._push_samples(session, transport, 1)
        assert session.exp_record_stop()["ok"]

    def test_disconnect_aborts_recording(self, exp_session, tmp_path):
        """切断(teardown)で自動停止し meta に aborted=true(T1-5)。"""
        session, transport, clock, responder = exp_session
        recorder = self._recorder(session, tmp_path)
        result = session.exp_record_start()
        assert result["ok"], result
        session.disconnect()
        assert not recorder.is_recording()
        meta_name = result["file"][:-len(".csv")] + "_meta.json"
        meta = json.loads((tmp_path / meta_name).read_text(encoding="utf-8"))
        assert meta["aborted"] is True


class TestRecordingLed:
    """計測中 LED(CMD_LED_MODE): 開始/停止時の送信・キープアライブ・
    送信失敗でも計測継続(機体側3秒フェイルセーフ前提の警告のみ)。"""

    @staticmethod
    def _led_frames(transport):
        return [proto.CmdLedMode.from_payload(f.payload)
                for f in transport.frames_of_type(proto.MsgType.CMD_LED_MODE)]

    def test_start_sends_recording_and_stop_sends_auto(self, exp_session,
                                                       tmp_path):
        session, transport, clock, responder = exp_session
        session.experiment.recorder.logs_dir = tmp_path
        transport.clear_sent()
        assert session.exp_record_start()["ok"]
        modes = [f.mode for f in self._led_frames(transport)]
        assert modes == [proto.CmdLedMode.MODE_RECORDING]

        transport.clear_sent()
        assert session.exp_record_stop()["ok"]
        modes = [f.mode for f in self._led_frames(transport)]
        assert modes == [proto.CmdLedMode.MODE_AUTO]

    def test_keepalive_resends_about_every_second(self, exp_session,
                                                  tmp_path):
        session, transport, clock, responder = exp_session
        session.experiment.recorder.logs_dir = tmp_path
        assert session.exp_record_start()["ok"]
        transport.clear_sent()

        now = clock()
        session.supervise(now)               # 初回は即送信
        assert len(self._led_frames(transport)) == 1
        session.supervise(now + 0.5)         # 1秒未満 → 送らない
        assert len(self._led_frames(transport)) == 1
        session.supervise(now + 1.0)         # 1秒経過 → 再送
        assert len(self._led_frames(transport)) == 2
        assert all(f.mode == proto.CmdLedMode.MODE_RECORDING
                   for f in self._led_frames(transport))

    def test_keepalive_silent_when_not_recording(self, exp_session):
        session, transport, clock, responder = exp_session
        transport.clear_sent()
        now = clock()
        session.supervise(now)
        session.supervise(now + 2.0)
        assert self._led_frames(transport) == []

    def test_send_failure_does_not_stop_recording(self, exp_session,
                                                  tmp_path, monkeypatch):
        """送信失敗/NACK でも計測は開始・継続する(警告のみ、T 相当)。"""
        session, transport, clock, responder = exp_session
        recorder = session.experiment.recorder
        recorder.logs_dir = tmp_path

        from core.serial_link import SerialLinkError

        def raise_send(msg_type, payload=b""):
            raise SerialLinkError("送信不能(テスト)")

        monkeypatch.setattr(session.serial, "send", raise_send)
        result = session.exp_record_start()
        assert result["ok"], result
        assert recorder.is_recording()
        # キープアライブも例外を漏らさない
        session.supervise(clock())
        assert recorder.is_recording()
        result = session.exp_record_stop()
        assert result["ok"], result

    def test_auto_stop_path_sends_auto(self, exp_session, tmp_path):
        """モード離脱の自動停止経路(hub.deactivate → recorder.stop)でも
        CMD_LED_MODE(0) が送られる。"""
        session, transport, clock, responder = exp_session
        session.experiment.recorder.logs_dir = tmp_path
        assert session.exp_record_start()["ok"]
        transport.clear_sent()
        assert session.set_mode(MODE_POSTURE)
        assert not session.experiment.recorder.is_recording()
        modes = [f.mode for f in self._led_frames(transport)]
        assert proto.CmdLedMode.MODE_AUTO in modes


class TestSequenceRunner:
    def test_two_mask_sequence(self, exp_session, short_sweep):
        session, transport, clock, responder = exp_session
        session.experiment.sweep.result_dir = short_sweep
        feeder = start_exp_feeder(session, transport)   # vbat 3.9 ≥ 3.5(電池ガード通過)
        try:
            result = session.sequence_start([0x1, 0x2], "up", None, 3.5)
            assert result["ok"], result
            assert wait_until(
                lambda: not session.experiment.sequence.is_running(),
                timeout=15.0)
        finally:
            feeder.set()
        status = session.experiment.sequence.status()
        assert status["phase"] == "done", status
        assert len(status["results"]) == 2
        import json
        meta = json.loads((short_sweep / status["last_meta"]).read_text())
        assert meta["schema"] == "stampfly_sweep_sequence_meta"
        assert meta["completed"] is True
        assert [r["motors"] for r in meta["runs"]] == ["FL", "FR"]
