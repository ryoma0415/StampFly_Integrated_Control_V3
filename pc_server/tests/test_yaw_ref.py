"""ヨー基準ソース/整列方式(契約 §3.1 + 2026-07-31 P0-1)の CMD_POS_ERR 検証。

- mocap(既定): 連続性フィルタ後の yaw_true(パネル較正変換 = 表示列と
  同一値)が単一情報源。整列方式 yaw_ref.align:
  - arm(既定): sent = wrap(yaw_true + anchor)。anchor は機体 state の
    地上→飛行遷移(アーム遷移)で「yaw_est_rad − yaw_true」の直近 0.5s 窓
    中央値をラッチ(地上でも暫定ラッチ — プリフライト fused チェック用)
  - absolute: sent = yaw_true(較正オフセット方式)
  棄却中(mocap_flip bit1)・途絶中・アンカー未ラッチは bit3 を落とす
- off: bit3 を立てない(mocap_yaw=0)
- motion: 推定 valid かつ整列成立時 bit3+bit4(低信頼)、invalid 時 bit3 を
  落とす。送信値は PoC 規約 ψ̂ → yaw_true_motion=−ψ̂ → mocap ソースと
  同一の整列(arm: +anchor / absolute: そのまま)を適用する(2026-08-03
  改修。旧 heading×wire_sign 経路は P0-1 アンカーを通らず 18:20 型の
  定数オフセット基準が再発するため廃止)
- アンカーは地上ではローリング再ラッチ(毎 tick 窓中央値)、アーム遷移で
  凍結(2026-08-03 改修 — 22:38 の stale anchor 段差 → b_g 誤吸収対策)
- v6 ロガー: yaw_ref_source / yaw_ref_sent_rad / yaw_ref_valid /
  motion_yaw_rad / motion_yaw_J 列が行に載る。meta.json に yaw_ref
  (align・anchor)が記録され、アーム遷移の再ラッチで追記更新される
- 18:20 故障モード(基準がフレーム差 ≈−90° のまま機体へ届き EKF2 の
  30° ゲート全棄却 → 25 連続棄却ラッチ恒久発動)が arm 整列で再現しない
  こと: sent − 機体ヨー初期値 ≈ 0
"""

from __future__ import annotations

import csv
import json
import math

import pytest
import stampfly_protocol as proto

from core.mocap import AttitudeMapper, wrap_pi
from core.session import (
    YAW_ALIGN_ABSOLUTE, YAW_ALIGN_ARM, YAW_REF_MOCAP, YAW_REF_MOTION,
    YAW_REF_OFF,
)


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
        "mocap_yaw_true_rad": 1.62,
        "mocap_flip": 0,
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


def _set_tlm(session, clock, state=proto.FlightState.WAIT,
             yaw_est_rad=0.0, r=0.0):
    """最新 TLM_STATE を直接注入する(RX ハンドラ相当。鮮度 = 現在時刻)。"""
    tlm = proto.TlmState(state=int(state), yaw_est_rad=yaw_est_rad, r=r)
    with session._lock:
        session._tlm_state = tlm
        session._tlm_state_t = clock()


def _force_motion_estimate(session, yaw_rad, j=25.0, valid=True):
    """MotionYawEstimator.estimate() を固定値に差し替える(単体は
    test_motion_yaw.py で検証済み — ここでは配線とフラグ規約を見る)。"""
    session.motion_yaw.estimate = lambda: {
        "yaw_rad": yaw_rad if valid else None,
        "j": j, "valid": valid, "n": 400}


class TestYawRefSource:
    def test_default_source_and_align(self, session_factory):
        session, transport, _clock = session_factory()
        status = session.yaw_ref_status()
        assert status["source"] == YAW_REF_MOCAP
        assert status["align"] == YAW_ALIGN_ARM
        assert status["anchor_deg"] is None

    def test_absolute_align_sends_yaw_true_with_bit4_clear(
            self, session_factory):
        """absolute 整列: 表示列と同一の yaw_true をそのまま送る。"""
        session, transport, _clock = session_factory()
        assert session.set_yaw_ref_align(YAW_ALIGN_ABSOLUTE)["ok"]
        _connect_quiet(session, transport)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        assert not (pe.flags & proto.CmdPosErr.FLAG_YAW_REF_LOW_TRUST)
        assert pe.mocap_yaw == pytest.approx(1.62)

    def test_arm_align_without_anchor_clears_bit3(self, session_factory):
        """arm 整列でアンカー未ラッチ(テレメトリ無し)なら bit3 を落とす。"""
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)
        assert pe.mocap_yaw == 0.0

    def test_off_source_clears_bit3(self, session_factory):
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_OFF)["ok"]

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)
        assert not (pe.flags & proto.CmdPosErr.FLAG_YAW_REF_LOW_TRUST)
        assert pe.mocap_yaw == 0.0

    def test_motion_source_sets_bit3_and_bit4_with_arm_anchor(
            self, session_factory):
        """motion + arm 整列: sent = wrap(-ψ̂ + anchor)(mocap と同一整列)。

        2026-08-03 改修の中核: 旧 heading×wire_sign 経路(P0-1 アンカーを
        通らない)は、実ログ検証で機体制御フレームに対し ≈−90° の定数
        オフセットとなり EKF2 30° ゲート内 0%(→ 全棄却ラッチ)だった。
        """
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        psi_poc = -0.8   # PoC 規約(ψ̂ ≈ -yaw_true)
        _force_motion_estimate(session, psi_poc)
        # 地上でアンカーをラッチ(yaw_est=0.5, yaw_true=1.62 → −1.12)
        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        assert pe.flags & proto.CmdPosErr.FLAG_YAW_REF_LOW_TRUST
        anchor = wrap_pi(0.5 - 1.62)
        expected = wrap_pi(-psi_poc + anchor)
        assert pe.mocap_yaw == pytest.approx(expected, abs=1e-6)

    def test_motion_source_absolute_align_sends_yaw_true_motion(
            self, session_factory):
        """motion + absolute 整列: sent = -ψ̂(較正オフセット方式と同格)。"""
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        assert session.set_yaw_ref_align(YAW_ALIGN_ABSOLUTE)["ok"]
        psi_poc = -0.8
        _force_motion_estimate(session, psi_poc)

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        assert pe.flags & proto.CmdPosErr.FLAG_YAW_REF_LOW_TRUST
        assert pe.mocap_yaw == pytest.approx(wrap_pi(-psi_poc), abs=1e-6)

    def test_motion_source_without_anchor_clears_bit3(self, session_factory):
        """motion + arm 整列でアンカー未ラッチなら bit3 を落とす
        (誤フレームの推定値を送らない — mocap ソースと同一規則)。"""
        session, transport, _clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        _force_motion_estimate(session, -0.8)   # 推定は valid だが…

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        # テレメトリ未受信 → アンカー未ラッチ → bit3 なし
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)
        assert pe.mocap_yaw == 0.0

    def test_motion_source_unsupported_mapping_clears_bit3(
            self, session_factory, monkeypatch):
        """未対応マッピング(machine_wire_y_sign=None)では motion を送らない。

        PoC 規約 ψ̂≈−yaw_true は対応マッピングでのみ実証されており、軸入替
        等の未対応フレームでは LS モデル自体が未検証のため(XY 制御の無効化
        と同じ判定基準)。
        """
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        _force_motion_estimate(session, -0.8)
        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
        monkeypatch.setattr(type(session.mocap), "machine_wire_y_sign",
                            property(lambda self: None))

        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        pe = _sent_pos_errs(transport)[0]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)
        assert pe.mocap_yaw == 0.0

    def test_motion_source_invalid_clears_bit3(self, session_factory):
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
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

    def test_invalid_align_rejected(self, session_factory):
        session, _transport, _clock = session_factory()
        result = session.set_yaw_ref_align("relative")
        assert result["ok"] is False
        assert session.yaw_ref_status()["align"] == YAW_ALIGN_ARM

    def test_snapshot_carries_yaw_ref(self, session_factory):
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        _force_motion_estimate(session, -0.8)
        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())

        snapshot = session.get_state_snapshot()
        yaw_ref = snapshot["data"]["session"]["yaw_ref"]
        assert yaw_ref["source"] == YAW_REF_MOTION
        assert yaw_ref["align"] == YAW_ALIGN_ARM
        assert yaw_ref["motion_valid"] is True
        assert yaw_ref["motion_yaw_deg"] is not None


class TestYawAlignArm:
    """アーム相対アンカー方式(P0-1 の本命経路)。"""

    def test_ground_latch_then_arm_relatch_tracks_mocap(self,
                                                        session_factory):
        """(a) sent はアーム時に機体ヨーへ一致し、以後 mocap 変化を追従する。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)

        # 地上(WAIT): 暫定ラッチ → sent == 機体ヨー(yaw_est)
        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
        for _ in range(5):
            clock.advance(0.02)
            _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
            session._emit_setpoint(0.0, 0.0, 0.3,
                                   _position_meta(mocap_yaw_true_rad=1.3))
        pe = _sent_pos_errs(transport)[-1]
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        assert pe.mocap_yaw == pytest.approx(0.5, abs=1e-6)
        assert session.yaw_ref_status()["anchor_latched"] == "ground"

        # アーム遷移(WAIT→TAKEOFF): 0.5s 窓中央値で再ラッチ
        clock.advance(0.02)
        _set_tlm(session, clock, proto.FlightState.TAKEOFF, yaw_est_rad=0.5)
        session._emit_setpoint(0.0, 0.0, 0.3,
                               _position_meta(mocap_yaw_true_rad=1.3))
        pe = _sent_pos_errs(transport)[-1]
        assert pe.mocap_yaw == pytest.approx(0.5, abs=1e-6)
        assert session.yaw_ref_status()["anchor_latched"] == "arm"

        # 以後 mocap の変化がそのまま sent に乗る(アンカーは固定)
        clock.advance(0.02)
        _set_tlm(session, clock, proto.FlightState.HOVER, yaw_est_rad=0.7)
        session._emit_setpoint(0.0, 0.0, 0.3,
                               _position_meta(mocap_yaw_true_rad=1.3 + 0.4))
        pe = _sent_pos_errs(transport)[-1]
        assert pe.mocap_yaw == pytest.approx(0.5 + 0.4, abs=1e-6)

    def test_arm_median_rejects_window_glitch(self, session_factory):
        """アンカー窓の単発グリッチは中央値で無害化される。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        # 窓に正常 diff(yaw_est=0.2, yaw_true=1.0)を多数+グリッチ1点
        for i in range(9):
            clock.advance(0.04)
            _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.2)
            yaw_true = 1.0 if i != 4 else 1.0 + math.pi / 2.0   # 90°グリッチ
            session._emit_setpoint(0.0, 0.0, 0.3,
                                   _position_meta(mocap_yaw_true_rad=yaw_true))
        clock.advance(0.02)
        _set_tlm(session, clock, proto.FlightState.TAKEOFF, yaw_est_rad=0.2)
        session._emit_setpoint(0.0, 0.0, 0.3,
                               _position_meta(mocap_yaw_true_rad=1.0))
        pe = _sent_pos_errs(transport)[-1]
        # 中央値ラッチ: sent == yaw_est(グリッチ点の diff は棄却される)
        assert pe.mocap_yaw == pytest.approx(0.2, abs=1e-6)

    def test_1820_failure_mode_not_reproduced(self, session_factory):
        """(d) 18:20 故障モード(基準オフセット ≈−90°)が再現しない。

        18:20 飛行では sent−機体ヨーがフレーム差 ≈−90° のまま届き、EKF2 の
        30° ゲートが全棄却 → 25 連続棄却ラッチが離陸前に恒久発動した。
        arm 整列では較正オフセットの値に関わらず sent − 機体ヨー初期値 ≈ 0。
        """
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        yaw_est0 = 0.0
        # パネル較正のオフセット欠落を模擬: yaw_true が機体フレームから
        # −90° ずれた値のまま供給される
        yaw_true_bad = wrap_pi(yaw_est0 - math.pi / 2.0)
        for _ in range(5):
            clock.advance(0.02)
            _set_tlm(session, clock, proto.FlightState.WAIT,
                     yaw_est_rad=yaw_est0)
            session._emit_setpoint(
                0.0, 0.0, 0.3, _position_meta(mocap_yaw_true_rad=yaw_true_bad))
        clock.advance(0.02)
        _set_tlm(session, clock, proto.FlightState.TAKEOFF,
                 yaw_est_rad=yaw_est0)
        session._emit_setpoint(
            0.0, 0.0, 0.3, _position_meta(mocap_yaw_true_rad=yaw_true_bad))
        pe = _sent_pos_errs(transport)[-1]
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        # sent − 機体ヨー初期値 ≈ 0(EKF2 30° ゲートの余裕内どころか一致)
        assert abs(wrap_pi(pe.mocap_yaw - yaw_est0)) < 1e-6

    def test_continuity_reject_drops_bit3(self, session_factory):
        """(c) 連続性フィルタ棄却中(mocap_flip bit1)は bit3 を落とす。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta())
        pe = _sent_pos_errs(transport)[-1]
        assert pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID   # 前提: 有効

        clock.advance(0.02)
        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            mocap_flip=AttitudeMapper.FLIP_YAW_JUMP))
        pe = _sent_pos_errs(transport)[-1]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)
        assert pe.mocap_yaw == 0.0

        # absolute 整列でも同様に落ちる
        assert session.set_yaw_ref_align(YAW_ALIGN_ABSOLUTE)["ok"]
        clock.advance(0.02)
        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            mocap_flip=AttitudeMapper.FLIP_YAW_JUMP))
        pe = _sent_pos_errs(transport)[-1]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)

    def test_dropout_drops_bit3(self, session_factory):
        """MoCap 途絶中は(ステイル値を送り続けず)bit3 を落とす。"""
        session, transport, clock = session_factory()
        assert session.set_yaw_ref_align(YAW_ALIGN_ABSOLUTE)["ok"]
        _connect_quiet(session, transport)
        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            mocap_dropout=True, data_valid=False))
        pe = _sent_pos_errs(transport)[-1]
        assert not (pe.flags & proto.CmdPosErr.FLAG_MOCAP_YAW_VALID)

    def test_stale_anchor_rolls_on_ground_without_arm_step(
            self, session_factory):
        """(e) 22:38 故障モード対策: 地上ローリング再ラッチ。

        前フライトでラッチしたアンカーは、着陸後の地上待機中に機体ヨーが
        ドリフトすると stale になり、次のアーム再ラッチで基準が段差
        ジャンプする(22:38 実測 −8.46° → EKF2 の b_g 誤吸収 → 飛行
        RMS 2.03°)。地上ではアンカーが機体ヨーへ追従(ローリング)し、
        アーム時に段差が生じないことを検証する。
        """
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        # フライト1: 地上ラッチ → アーム(anchor = 0.5 − 1.3 = −0.8)
        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
        session._emit_setpoint(0.0, 0.0, 0.3,
                               _position_meta(mocap_yaw_true_rad=1.3))
        clock.advance(0.02)
        _set_tlm(session, clock, proto.FlightState.TAKEOFF, yaw_est_rad=0.5)
        session._emit_setpoint(0.0, 0.0, 0.3,
                               _position_meta(mocap_yaw_true_rad=1.3))
        # 着陸 → 地上待機中に機体ヨーが 0.5 → 0.35 へドリフト(EKF1
        # 縮退相当)。0.5s 窓が新値で満ちるまでホールドする
        for _ in range(35):
            clock.advance(0.02)
            _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.35)
            session._emit_setpoint(0.0, 0.0, 0.3,
                                   _position_meta(mocap_yaw_true_rad=1.3))
        pe_ground = _sent_pos_errs(transport)[-1]
        # 地上ローリング: sent は現在の機体ヨーへ追従(stale の 0.5 でなく)
        assert pe_ground.mocap_yaw == pytest.approx(0.35, abs=1e-6)
        assert session.yaw_ref_status()["anchor_latched"] == "ground"
        # フライト2 アーム: 再ラッチしても基準に段差が生じない
        clock.advance(0.02)
        _set_tlm(session, clock, proto.FlightState.TAKEOFF, yaw_est_rad=0.35)
        session._emit_setpoint(0.0, 0.0, 0.3,
                               _position_meta(mocap_yaw_true_rad=1.3))
        pe_arm = _sent_pos_errs(transport)[-1]
        assert abs(wrap_pi(pe_arm.mocap_yaw - pe_ground.mocap_yaw)) < 1e-6
        assert session.yaw_ref_status()["anchor_latched"] == "arm"

    def _drain_log_lines(self, session):
        lines = []
        while not session.events.empty():
            event = session.events.get_nowait()
            if event.get("type") == "log":
                lines.append(event.get("line", ""))
        return lines

    def test_ground_rolling_freezes_on_inconsistent_frames(
            self, session_factory):
        """(f) スプレッドガード: 鏡映フレーム模擬(yaw_true が逆方向に流れ
        diff が 2ψ̇ でスイープ)では地上ローリングが凍結し、直前アンカーを
        維持して警告を出す(プリフライトの手回し検査の検出力を回復)。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        # 静止で地上ラッチ(anchor = 0.5 − 1.0 = −0.5)
        for _ in range(30):
            clock.advance(0.02)
            _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
            session._emit_setpoint(0.0, 0.0, 0.3,
                                   _position_meta(mocap_yaw_true_rad=1.0))
        self._drain_log_lines(session)
        # 手回し模擬: 機体ヨー +60°/s、鏡映 yaw_true は −60°/s(3s)
        rate = math.radians(60.0)
        yaw_est, yaw_true = 0.5, 1.0
        for _ in range(150):
            clock.advance(0.02)
            yaw_est += rate * 0.02
            yaw_true -= rate * 0.02
            _set_tlm(session, clock, proto.FlightState.WAIT,
                     yaw_est_rad=wrap_pi(yaw_est))
            session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
                mocap_yaw_true_rad=wrap_pi(yaw_true)))
        # アンカーは回転前の値で凍結(追従していれば大きく動いている)
        status = session.yaw_ref_status()
        assert status["anchor_deg"] == pytest.approx(
            math.degrees(-0.5), abs=3.0)
        # 2s 継続で整合不安定の警告
        assert any("整合が不安定" in line
                   for line in self._drain_log_lines(session))

    def test_ground_rolling_continues_during_consistent_rotation(
            self, session_factory):
        """(g) 正常フレームの手回し(yaw_est と yaw_true が同期変化)は
        diff 一定 → スプレッド小のためローリング継続・警告なし。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        rate = math.radians(60.0)
        yaw_est, yaw_true = 0.5, 1.0
        for _ in range(150):
            clock.advance(0.02)
            yaw_est += rate * 0.02
            yaw_true += rate * 0.02
            _set_tlm(session, clock, proto.FlightState.WAIT,
                     yaw_est_rad=wrap_pi(yaw_est))
            session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
                mocap_yaw_true_rad=wrap_pi(yaw_true)))
        status = session.yaw_ref_status()
        assert status["anchor_latched"] == "ground"
        assert status["anchor_deg"] == pytest.approx(
            math.degrees(-0.5), abs=1.0)
        assert not any("整合が不安定" in line
                       for line in self._drain_log_lines(session))

    def test_ground_rolling_tracks_slow_drift(self, session_factory):
        """(h) 緩慢な機体ヨードリフト(1.5°/s = EKF1 縮退級)は 0.5s 窓の
        スプレッドが閾値未満のため追従が継続する(22:38 対策の維持)。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        yaw_est = 0.5
        drift = math.radians(1.5)
        for _ in range(150):   # 3s で −4.5° … +4.5°分ドリフト
            clock.advance(0.02)
            yaw_est += drift * 0.02
            _set_tlm(session, clock, proto.FlightState.WAIT,
                     yaw_est_rad=yaw_est)
            session._emit_setpoint(0.0, 0.0, 0.3,
                                   _position_meta(mocap_yaw_true_rad=1.0))
        pe = _sent_pos_errs(transport)[-1]
        # sent は現在の機体ヨーへ追従(0.5s 窓中央値の遅れ ≈ 0.4° 以内)
        assert abs(wrap_pi(pe.mocap_yaw - yaw_est)) < math.radians(1.0)
        assert not any("整合が不安定" in line
                       for line in self._drain_log_lines(session))

    def test_realign_count_increase_warns(self, session_factory):
        """連続性フィルタの再整列カウント増加が警告ログになる。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            mocap_yaw_realign=0))
        clock.advance(0.02)
        session._emit_setpoint(0.0, 0.0, 0.3, _position_meta(
            mocap_yaw_realign=1))
        lines = []
        while not session.events.empty():
            event = session.events.get_nowait()
            if event.get("type") == "log":
                lines.append(event.get("line", ""))
        assert any("再整列" in line for line in lines)


class TestV6LogColumns:
    def test_pos_err_row_contains_yaw_ref_columns(self, session_factory,
                                                  tmp_path):
        """v6 ロガー: CMD_POS_ERR 行に PC 側ヨー基準/移動ベースヨー列が載る。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        assert session.set_yaw_ref_source(YAW_REF_MOTION)["ok"]
        _force_motion_estimate(session, -0.8, j=25.0)
        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
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
        # meta.json は v6 を宣言し、ヨー基準(整列方式)を記録する
        meta_path = path.with_suffix(".meta.json")
        assert meta_path.is_file()
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        assert meta["log_columns_version"] == 6
        assert meta["yaw_ref"]["align"] == YAW_ALIGN_ARM
        assert meta["yaw_ref"]["source"] == YAW_REF_MOTION

    def test_meta_json_updated_with_arm_anchor(self, session_factory,
                                               tmp_path):
        """アーム遷移の再ラッチが meta.json へ追記される(P0-1)。"""
        session, transport, clock = session_factory()
        _connect_quiet(session, transport)
        session.logger._logs_dir = tmp_path
        path = session.logger.start("position",
                                    metadata=session._log_metadata())

        _set_tlm(session, clock, proto.FlightState.WAIT, yaw_est_rad=0.5)
        session._emit_setpoint(0.0, 0.0, 0.3,
                               _position_meta(mocap_yaw_true_rad=1.3))
        clock.advance(0.02)
        _set_tlm(session, clock, proto.FlightState.TAKEOFF, yaw_est_rad=0.5)
        session._emit_setpoint(0.0, 0.0, 0.3,
                               _position_meta(mocap_yaw_true_rad=1.3))
        session.logger.stop()

        meta = json.loads(path.with_suffix(".meta.json")
                          .read_text(encoding="utf-8"))
        anchor = meta["yaw_ref"]["anchor"]
        assert anchor["latched"] == "arm"
        assert anchor["anchor_rad"] == pytest.approx(0.5 - 1.3, abs=1e-6)
