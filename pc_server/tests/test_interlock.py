"""プリフライト・インターロック(P1-2)の離陸ゲート検証。

条件(FLIGHT_ANALYSIS_20260731.md §7-B-2): yaw_ref_source ≠ off のとき
「ekf2_status bit1(fused)=1 ∧ |innov| < 10°」が 3s 連続成立するまで:
- est_mode=2(EKF2 制御。ff_status bit7): start() が離陸をブロック
  (force=True の明示的 override でのみ CMD_START 許可)
- est_mode≠2(シャドー): 警告のみでブロックしない
- source=off: 非干渉(ゲートも警告も出さない)
連続成立の計時は _on_tlm_state 相当の TLM 注入で行い、不成立フレーム・
テレメトリ途絶(telemetry_fresh_s 超のフレーム間隔)でリセットされる。
"""

from __future__ import annotations

import math

import pytest
import stampfly_protocol as proto

from core.session import (
    _YAW_INTERLOCK_HOLD_S, YAW_REF_OFF,
)

INNOV_OK_RAD = math.radians(2.0)     # |innov| < 10° の代表値
INNOV_BAD_RAD = math.radians(15.0)   # ゲート外の代表値


def _drain_lines(session) -> list[str]:
    lines = []
    while True:
        try:
            event = session.events.get_nowait()
        except Exception:
            break
        if event.get("type") == "log":
            lines.append(event.get("line") or "")
    return lines


def _connect_quiet(session, transport):
    assert session.connect("COM-fake")
    session.posture.stop()
    session.position.stop()
    transport.sent_frames.clear()


def _inject_tlm(session, clock, *, fused=True, innov_rad=INNOV_OK_RAD,
                est2=True, recapture=False,
                state=proto.FlightState.WAIT):
    """TLM_STATE の注入(_on_tlm_state 相当: 保持+インターロック計時)。"""
    ekf2_status = 0
    if fused:
        ekf2_status |= proto.TlmState.EKF2_STATUS_YAW_OBS_FUSED
    if recapture:
        ekf2_status |= proto.TlmState.EKF2_STATUS_YAW_RECAPTURE
    ff_status = proto.TlmState.FF_STATUS_EST_EKF2 if est2 else 0
    tlm = proto.TlmState(state=int(state), ff_status=ff_status,
                         ekf2_status=ekf2_status,
                         ekf2_yaw_innov_rad=innov_rad)
    now = clock()
    with session._lock:
        prev_t = session._tlm_state_t
        session._tlm_state = tlm
        session._tlm_state_t = now
        session._update_yaw_interlock_locked(tlm, now, prev_t)


def _hold_aligned(session, clock, seconds=None, **kwargs):
    """条件成立フレームを 0.1s 間隔で注入し続ける(途絶リセットを跨がない)。"""
    seconds = _YAW_INTERLOCK_HOLD_S + 0.2 if seconds is None else seconds
    steps = int(seconds / 0.1) + 1
    _inject_tlm(session, clock, **kwargs)
    for _ in range(steps):
        clock.advance(0.1)
        _inject_tlm(session, clock, **kwargs)


class TestInterlockStatus:
    def test_source_off_is_exempt(self, session_factory):
        session, transport, clock = session_factory()
        assert session.set_yaw_ref_source(YAW_REF_OFF)["ok"]
        st = session.yaw_interlock_status()
        assert st["state"] == "off"
        assert st["blocking"] is False

    def test_no_telemetry(self, session_factory):
        session, transport, clock = session_factory()
        st = session.yaw_interlock_status()
        assert st["state"] == "no_telemetry"
        assert st["blocking"] is False   # est_mode 不明の間はブロックしない

    def test_no_fused_blocks_when_est2(self, session_factory):
        session, transport, clock = session_factory()
        _inject_tlm(session, clock, fused=False)
        st = session.yaw_interlock_status()
        assert st["state"] == "no_fused"
        assert st["est2"] is True
        assert st["blocking"] is True

    def test_recapture_reported(self, session_factory):
        session, transport, clock = session_factory()
        _inject_tlm(session, clock, fused=False, recapture=True)
        st = session.yaw_interlock_status()
        assert st["state"] == "no_fused"
        assert st["recapture"] is True

    def test_waiting_then_aligned_after_hold(self, session_factory):
        session, transport, clock = session_factory()
        _inject_tlm(session, clock)
        st = session.yaw_interlock_status()
        assert st["state"] == "waiting"
        assert st["aligned"] is False
        _hold_aligned(session, clock)
        st = session.yaw_interlock_status()
        assert st["state"] == "ok"
        assert st["aligned"] is True
        assert st["blocking"] is False
        assert st["hold_s"] >= _YAW_INTERLOCK_HOLD_S

    def test_innov_out_of_gate_resets_hold(self, session_factory):
        session, transport, clock = session_factory()
        _hold_aligned(session, clock, seconds=2.0)
        _inject_tlm(session, clock, innov_rad=INNOV_BAD_RAD)  # fused でも innov 大
        _inject_tlm(session, clock)
        st = session.yaw_interlock_status()
        assert st["state"] == "waiting"
        assert st["hold_s"] < 1.0        # 計時が仕切り直されている

    def test_telemetry_gap_resets_hold(self, session_factory):
        session, transport, clock = session_factory()
        _hold_aligned(session, clock, seconds=2.0)
        clock.advance(1.0)               # telemetry_fresh_s(0.3s)超の途絶
        _inject_tlm(session, clock)
        st = session.yaw_interlock_status()
        assert st["aligned"] is False
        assert st["hold_s"] < 1.0

    def test_snapshot_carries_interlock(self, session_factory):
        session, transport, clock = session_factory()
        _inject_tlm(session, clock, fused=False)
        snap = session.get_state_snapshot()
        il = snap["data"]["session"]["yaw_interlock"]
        assert il["state"] == "no_fused"
        assert il["blocking"] is True


class TestStartGate:
    def test_blocks_start_when_est2_not_aligned(self, session_factory):
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        _inject_tlm(session, clock, fused=False)
        _drain_lines(session)

        assert session.start() is False
        assert transport.frames_of_type(proto.MsgType.CMD_START) == []
        lines = _drain_lines(session)
        assert any("離陸ブロック" in line for line in lines)

    def test_force_overrides_block(self, session_factory):
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        _inject_tlm(session, clock, fused=False)
        _drain_lines(session)

        assert session.start(force=True) is True
        assert len(transport.frames_of_type(proto.MsgType.CMD_START)) == 1
        lines = _drain_lines(session)
        assert any("force" in line for line in lines)

    def test_aligned_allows_start(self, session_factory):
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        _hold_aligned(session, clock)
        _drain_lines(session)

        assert session.start() is True
        assert len(transport.frames_of_type(proto.MsgType.CMD_START)) == 1
        lines = _drain_lines(session)
        assert not any("インターロック" in line or "整列が未成立" in line
                       for line in lines)

    def test_shadow_warns_but_allows(self, session_factory):
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        _inject_tlm(session, clock, fused=False, est2=False)
        _drain_lines(session)

        assert session.start() is True
        assert len(transport.frames_of_type(proto.MsgType.CMD_START)) == 1
        lines = _drain_lines(session)
        assert any("シャドー" in line for line in lines)

    def test_source_off_no_interference(self, session_factory):
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_OFF)["ok"]
        _inject_tlm(session, clock, fused=False)   # est2 でも off なら非干渉
        _drain_lines(session)

        assert session.start() is True
        assert len(transport.frames_of_type(proto.MsgType.CMD_START)) == 1
        lines = _drain_lines(session)
        assert not any("インターロック" in line or "整列" in line
                       for line in lines)

    def test_hold_survives_status_reads(self, session_factory):
        """status 読み出し(20Hz WS 相当)が計時に副作用を持たないこと。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        _inject_tlm(session, clock)
        for _ in range(5):
            session.yaw_interlock_status()
        _hold_aligned(session, clock)
        assert session.yaw_interlock_status()["aligned"] is True
