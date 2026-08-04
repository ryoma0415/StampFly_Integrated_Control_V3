"""v2 評価シーケンス(スクリプト軌道): 事前検証(不正セグメント・yaw制御OFF
拒否)・セグメント遷移の目標連続性・yaw ランプ・途絶凍結・開始インデックス・
meta traj_mode 切替・metadata 記録。"""

from __future__ import annotations

import json
import math

import pytest

from core.logger import FlightLogger
from core.position import (
    TRAJ_MODE_CIRCLE, TRAJ_MODE_HOVER, TRAJ_MODE_SHUTTLE, PositionController,
)
from core.posture import wrap_pi
from core.session import MODE_POSITION

from conftest import halt_supervisor
from fakes import FakeClock, make_pose

STEP_DT = 0.02


def make_controller(server_config, control_config):
    emitted = []
    clock = FakeClock()
    controller = PositionController(
        server_config, control_config,
        emit=lambda r, p, a, meta: emitted.append((r, p, a, meta)),
        clock=clock)
    return controller, emitted, clock


def feed_pose(controller, clock, x=0.0, y=0.0, z=0.3, yaw_rad=0.0):
    controller.on_mocap_pose(make_pose(x=x, y=y, z=z, t=clock(),
                                       yaw_rad=yaw_rad))


def step_with_pose(controller, clock, n, x=0.0, y=0.0):
    """pose を供給しながら n 周期ぶん step する(途絶させない)。"""
    for _ in range(n):
        clock.advance(STEP_DT)
        feed_pose(controller, clock, x=x, y=y)
        controller.step(clock())


# セグメント定義ヘルパ(control.json trajectory.sequences と同スキーマ)
def seg_hover(duration_s=1.0):
    return {"type": "hover", "duration_s": duration_s}


def seg_circle(radius=0.3, period=8.0, laps=1, clockwise=False):
    return {"type": "circle", "radius_m": radius, "period_s": period,
            "laps": laps, "clockwise": clockwise}


def seg_shuttle(axis=0.0, amp=0.3, period=8.0, cycles=1):
    return {"type": "shuttle", "axis_deg": axis, "amplitude_m": amp,
            "period_s": period, "cycles": cycles}


def seg_yaw(targets=(90.0, 0.0), rate=30.0, hold=1.0):
    return {"type": "yaw", "targets_deg": list(targets), "rate_dps": rate,
            "hold_s": hold}


def start_sequence(controller, clock, segments, alt_m=0.5, start_index=0,
                   name="test_seq"):
    return controller.start_sequence(name, segments, alt_m,
                                     start_index=start_index, now=clock())


class TestSequenceValidation:
    """開始時の事前検証: 全セグメントを既存ガードで検証し、不合格セグメントを
    日本語で特定する。"""

    def test_requires_position(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        ok, error = start_sequence(controller, clock, [seg_hover()])
        assert not ok
        assert "MoCap" in error

    def test_rejects_invalid_segment_with_index(self, server_config,
                                                control_config):
        """不正セグメント(円半径範囲外)は「どのセグメントか」を返して拒否。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(
            controller, clock, [seg_hover(), seg_circle(radius=99.0)])
        assert not ok
        assert "セグメント2" in error and "半径" in error
        assert controller.trajectory_snapshot() == {"mode": "hover"}

    def test_rejects_unknown_type(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock, [{"type": "spiral"}])
        assert not ok
        assert "セグメント1" in error and "不明" in error

    def test_rejects_continuous_shuttle(self, server_config, control_config):
        """シーケンス内 shuttle は終端が必要 → cycles=0(連続)は拒否。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock, [seg_shuttle(cycles=0)])
        assert not ok
        assert "サイクル" in error

    def test_rejects_nonpositive_hover(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock, [seg_hover(0.0)])
        assert not ok
        assert "duration_s" in error

    def test_rejects_yaw_rate_over_slew(self, server_config, control_config):
        """yaw レートは yaw_slew_rate_deg_per_s(45)以下でなければ拒否。"""
        controller, _, clock = make_controller(server_config, control_config)
        controller.set_yaw_control(True)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock, [seg_yaw(rate=60.0)])
        assert not ok
        assert "ヨーレート" in error
        # 境界(45°/s)は許可
        assert start_sequence(controller, clock, [seg_yaw(rate=45.0)])[0]

    def test_rejects_yaw_target_out_of_range(self, server_config,
                                             control_config):
        controller, _, clock = make_controller(server_config, control_config)
        controller.set_yaw_control(True)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock,
                                   [seg_yaw(targets=(200.0,))])
        assert not ok
        assert "ヨー目標角" in error

    def test_yaw_requires_yaw_control_on(self, server_config, control_config):
        """yaw セグメントを含む実行区間はヨー角制御 ON でなければ開始拒否。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock,
                                   [seg_hover(), seg_yaw()])
        assert not ok
        assert "ヨー角制御" in error
        controller.set_yaw_control(True)
        assert start_sequence(controller, clock,
                              [seg_hover(), seg_yaw()])[0]

    def test_yaw_before_start_index_allows_off(self, server_config,
                                               control_config):
        """開始インデックスで yaw を飛ばす場合はヨー角制御 OFF でも開始可。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock,
                                   [seg_yaw(), seg_hover()], start_index=1)
        assert ok, error

    def test_rejects_bad_start_index(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock,
                                   [seg_hover(), seg_hover()], start_index=2)
        assert not ok
        assert "開始セグメント" in error

    def test_alt_guard(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock, [seg_hover()],
                                   alt_m=99.0)
        assert not ok
        assert "高度" in error

    def test_preset_ekf2_eval_v1_is_valid(self, server_config, control_config):
        """control.json のプリセット ekf2_eval_v1 が事前検証を通り開始できる。"""
        segments = control_config["trajectory"]["sequences"]["ekf2_eval_v1"]
        assert len(segments) == 7
        controller, _, clock = make_controller(server_config, control_config)
        controller.set_yaw_control(True)
        feed_pose(controller, clock)
        ok, error = start_sequence(controller, clock, segments, alt_m=0.5)
        assert ok, error
        snap = controller.trajectory_snapshot()
        assert snap["mode"] == "sequence"
        assert snap["seg_count"] == 7
        # 見積り合計: 10 + 2×12 + 5 + 2×(3.25×6) + yaw(4×(3+10)) + 10 = 140s
        # + トランジット見積り(開始 (0,0) → 円周合流 0.35m/0.25 = 1.4s、
        #   X往復端点 (0.35,0) → Y往復合流 (0,0) = 1.4s)= 142.8s
        assert snap["remaining_s"] == pytest.approx(142.8, abs=0.5)


class TestSequenceCircleGuards:
    """シーケンス内 circle への速度・可動域ガード(レビュー指摘: 単発
    circle の広い制限値(中心±2m・R1.5m)で ±0.5m/0.5m/s の評価
    エンベロープを迂回できた)。単発 start_circle の制限値は既存挙動のまま。"""

    def test_rejects_circle_speed_violation(self, server_config,
                                            control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        # 周速 2π·0.4/3 = 0.84 m/s > speed_max_mps(0.5)
        ok, error = start_sequence(
            controller, clock, [seg_circle(radius=0.4, period=3.0)])
        assert not ok and "周速" in error

    def test_rejects_circle_excursion_violation(self, server_config,
                                                control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        # 到達範囲 |0.3|+0.3 = 0.6 m > excursion_abs_max_m(0.5)
        seg = dict(seg_circle(radius=0.3, period=8.0))
        seg["center_x"] = 0.3
        ok, error = start_sequence(controller, clock, [seg])
        assert not ok and "到達範囲" in error

    def test_rejects_nan_segment_values(self, server_config, control_config):
        """NaN は閉区間ガードで拒否される(hover duration の代表ケース)。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_sequence(
            controller, clock, [seg_hover(duration_s=float("nan"))])
        assert not ok and "duration_s" in error


class TestSequenceTransit:
    """セグメント遷移トランジット: 合流点への目標段差(レビュー指摘で最大
    0.35m 級)を 0.25 m/s の等速直線で埋める。"""

    def test_transitions_bounded_target_steps(self, server_config,
                                              control_config):
        controller, _, clock = make_controller(server_config, control_config)
        clock.advance(STEP_DT)
        feed_pose(controller, clock, x=0.3, y=0.3)
        # レビュー指摘の幾何: 開始 (0.3,0.3) → circle → X 往復 → Y 往復。
        # 旧実装では circle 合流で 0.07m、X→Y 遷移で 0.35m の目標段差が出た
        segments = [seg_circle(radius=0.35, period=12.0),
                    seg_shuttle(axis=0.0, amp=0.35, period=6.0),
                    seg_shuttle(axis=90.0, amp=0.35, period=6.0)]
        ok, error = start_sequence(controller, clock, segments, alt_m=0.5)
        assert ok, error
        prev = controller.get_target()
        max_step = 0.0
        saw_transit = False
        finished = False
        for _ in range(2600):
            clock.advance(STEP_DT)
            feed_pose(controller, clock, x=0.3, y=0.3)
            controller.step(clock())
            cur = controller.get_target()
            max_step = max(max_step, math.hypot(cur[0] - prev[0],
                                                cur[1] - prev[1]))
            prev = cur
            snap = controller.trajectory_snapshot()
            if snap.get("mode") == "sequence" and snap.get("transit"):
                saw_transit = True
            if snap.get("mode") == "hover":
                finished = True
                break
        assert finished, "シーケンスが完了しなかった"
        assert saw_transit, "トランジットが一度も挿入されなかった"
        # 1 tick の目標ステップは max(トランジット 0.25, shuttle 頂速
        # 2π·0.35/6 = 0.367) m/s × dt 以下に抑えられている
        assert max_step <= 0.367 * STEP_DT + 1e-6

    def test_remaining_estimate_includes_transits(self, server_config,
                                                  control_config):
        """開始直後の remaining_s にセグメント間トランジット見積りが載る
        (位置連鎖: 開始 (0.3,0.3) → 円周合流 → X往復端点 → Y往復合流)。"""
        controller, _, clock = make_controller(server_config, control_config)
        clock.advance(STEP_DT)
        feed_pose(controller, clock, x=0.3, y=0.3)
        segments = [seg_circle(radius=0.35, period=12.0),
                    seg_shuttle(axis=0.0, amp=0.35, period=6.0),
                    seg_shuttle(axis=90.0, amp=0.35, period=6.0)]
        ok, error = start_sequence(controller, clock, segments, alt_m=0.5)
        assert ok, error
        snap = controller.trajectory_snapshot()
        # 本体見積り: 12 + 1.25×6 + 1.25×6 = 27s
        # トランジット: (√0.18 − 0.35)/0.25 ≈ 0.297s(開始 → 円周)
        #  + 0.35·cos45°/0.25 ≈ 0.990s(円周点 → X軸射影)
        #  + 0.35/0.25 = 1.4s(X端点 (0.35,0) → Y合流 (0,0))
        d0 = math.hypot(0.3, 0.3) - 0.35
        d1 = 0.35 * math.cos(math.radians(45.0))
        expected = 27.0 + (d0 + d1 + 0.35) / 0.25
        assert snap["remaining_s"] == pytest.approx(expected, abs=1e-6)
        # 未入場の現セグメント残りにも入場前トランジット分が載る
        assert snap["seg_remaining_s"] == pytest.approx(12.0 + d0 / 0.25,
                                                        abs=1e-6)


class TestSequenceYawCtrlFreeze:
    """yaw セグメント中のヨー角制御 OFF はシーケンス時計ごと凍結する
    (レビュー指摘: OFF だとファームが yaw_ref を見ない「盲目ランプ」に
    なり、ON 復帰時に計画外の回頭が起きた)。"""

    def test_yaw_segment_freezes_when_yaw_ctrl_off(self, server_config,
                                                   control_config):
        controller, _, clock = make_controller(server_config, control_config)
        controller.set_yaw_control(True)
        clock.advance(STEP_DT)
        feed_pose(controller, clock)
        ok, error = start_sequence(
            controller, clock,
            [seg_yaw(targets=(90.0,), rate=30.0, hold=2.0)])
        assert ok, error
        step_with_pose(controller, clock, 25)   # 0.5s → ランプ ≈15°
        yaw_mid = controller.trajectory_snapshot()["target_yaw_rad"]
        assert yaw_mid == pytest.approx(math.radians(15.0),
                                        abs=math.radians(2.0))
        # OFF: 1 step で凍結が成立し、以後ランプは進まない
        controller.set_yaw_control(False)
        step_with_pose(controller, clock, 1)
        yaw_frozen = controller.trajectory_snapshot()["target_yaw_rad"]
        step_with_pose(controller, clock, 50)   # 1.0s 経過
        snap = controller.trajectory_snapshot()
        assert snap["mode"] == "sequence"
        assert snap["target_yaw_rad"] == pytest.approx(yaw_frozen, abs=1e-12)
        # ON 復帰: 凍結点から連続再開(t0 前送り)
        controller.set_yaw_control(True)
        step_with_pose(controller, clock, 25)   # +0.5s → ≈+15°
        yaw_after = controller.trajectory_snapshot()["target_yaw_rad"]
        assert yaw_after == pytest.approx(
            yaw_frozen + math.radians(15.0), abs=math.radians(2.0))


class TestSequenceMotion:
    def test_target_continuity_and_mode_switch(self, server_config,
                                               control_config):
        """遷移で目標が跳ばない(合流則で連続)+ traj_mode が 0→1→2→0 と
        現セグメントの基礎モードを報告する。"""
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock, x=0.3, y=0.0)
        # (0.3, 0) から: hover→circle(位相0合流)→shuttle(端点合流)→hover
        # は端点どうしが一致し、全遷移が厳密に連続になる幾何
        segments = [seg_hover(1.0), seg_circle(radius=0.3, period=8.0, laps=1),
                    seg_shuttle(axis=0.0, amp=0.3, period=8.0, cycles=1),
                    seg_hover(1.0)]
        ok, error = start_sequence(controller, clock, segments, alt_m=0.5)
        assert ok, error
        # 開始時: 現在位置へスナップ(hover は現在位置ホールド)
        controller.step(clock())
        tx, ty, tz = controller.get_target()
        assert (tx, ty, tz) == pytest.approx((0.3, 0.0, 0.5), abs=1e-6)

        targets = []
        modes = []
        # 合計 1+8+8+1 = 18s(shuttle は π/2 合流 → ちょうど 1 周期)+ 余裕
        for _ in range(940):
            clock.advance(STEP_DT)
            feed_pose(controller, clock, x=0.3, y=0.0)
            controller.step(clock())
            targets.append(controller.get_target())
            modes.append(emitted[-1][3]["traj_mode"])

        # 目標連続性: 1 step の移動が最大軌道速度(2πR/T=0.236m/s)を超えない
        v_max_per_step = (2 * math.pi * 0.3 / 8.0) * STEP_DT
        for (x0, y0, _), (x1, y1, _) in zip(targets, targets[1:]):
            assert math.hypot(x1 - x0, y1 - y0) <= v_max_per_step * 1.2 + 1e-9

        # traj_mode は hover(0)→circle(1)→shuttle(2)→hover(0) の順に切替わる
        compressed = [modes[0]]
        for m in modes[1:]:
            if m != compressed[-1]:
                compressed.append(m)
        assert compressed == [TRAJ_MODE_HOVER, TRAJ_MODE_CIRCLE,
                              TRAJ_MODE_SHUTTLE, TRAJ_MODE_HOVER]

        # 完了: 軌道が外れ、最終目標(shuttle 端点 = (0.3, 0))でホバ復帰
        assert controller.trajectory_snapshot() == {"mode": "hover"}
        assert controller.get_target() == pytest.approx((0.3, 0.0, 0.5),
                                                        abs=1e-6)

    def test_snapshot_progress_fields(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.3, y=0.0)
        segments = [seg_hover(1.0), seg_circle(radius=0.3, period=8.0, laps=1)]
        ok, error = start_sequence(controller, clock, segments)
        assert ok, error
        step_with_pose(controller, clock, 100, x=0.3)   # 2秒 → circle 内
        snap = controller.trajectory_snapshot()
        assert snap["mode"] == "sequence"
        assert snap["name"] == "test_seq"
        assert snap["seg_index"] == 1
        assert snap["seg_count"] == 2
        assert snap["seg_type"] == "circle"
        assert snap["elapsed_s"] == pytest.approx(2.0, abs=0.05)
        # 残り: circle 残 7s(9s 中 2s 消化)
        assert snap["remaining_s"] == pytest.approx(7.0, abs=0.1)
        assert snap["seg_remaining_s"] == pytest.approx(7.0, abs=0.1)

    def test_start_index_skips_segments(self, server_config, control_config):
        """「このセグメントから開始」: 先行セグメントを実行しない。"""
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock)
        segments = [seg_hover(5.0), seg_shuttle(cycles=1), seg_hover(5.0)]
        ok, error = start_sequence(controller, clock, segments, start_index=1)
        assert ok, error
        step_with_pose(controller, clock, 5)
        snap = controller.trajectory_snapshot()
        assert snap["seg_index"] == 1
        assert snap["seg_type"] == "shuttle"
        assert snap["start_index"] == 1
        assert emitted[-1][3]["traj_mode"] == TRAJ_MODE_SHUTTLE

    def test_stop_trajectory_interrupts(self, server_config, control_config):
        """stop_trajectory で即時中断し、現在目標でホバ復帰する。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.3, y=0.0)
        ok, error = start_sequence(
            controller, clock, [seg_circle(radius=0.3, period=8.0, laps=2)])
        assert ok, error
        step_with_pose(controller, clock, 50, x=0.3)
        frozen = controller.get_target()
        controller.stop_trajectory()
        step_with_pose(controller, clock, 10, x=0.3)
        assert controller.get_target() == pytest.approx(frozen)
        assert controller.trajectory_snapshot() == {"mode": "hover"}


class TestSequenceYawRamp:
    def test_yaw_ramp_within_slew_and_holds(self, server_config,
                                            control_config):
        """yaw セグメント: _target_yaw のランプが指定レートで進み、整形出力が
        スルーレート内で追従してホールドする。位置は現在目標ホールド。"""
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        controller.set_yaw_control(True)
        feed_pose(controller, clock, x=0.2, y=0.1)
        ok, error = start_sequence(
            controller, clock, [seg_yaw(targets=(90.0, 0.0), rate=30.0,
                                        hold=1.0)])
        assert ok, error

        yaw_outputs = []
        # ramp 3s + hold 1s + ramp 3s + hold 1s = 8s + 余裕
        for _ in range(420):
            clock.advance(STEP_DT)
            feed_pose(controller, clock, x=0.2, y=0.1)
            controller.step(clock())
            meta = emitted[-1][3]
            yaw_outputs.append(meta["yaw_ref_rad"])
            # 基礎モードは hover(0)・位置目標はホールド
            assert meta["traj_mode"] == TRAJ_MODE_HOVER
            assert meta["target_x"] == pytest.approx(0.2, abs=1e-6)
            assert meta["target_y"] == pytest.approx(0.1, abs=1e-6)

        # 整形出力の 1 step 変化はヨースルーレート(45°/s)以内
        slew_per_step = math.radians(45.0) * STEP_DT
        for y0, y1 in zip(yaw_outputs, yaw_outputs[1:]):
            assert abs(wrap_pi(y1 - y0)) <= slew_per_step + 1e-9

        # +90° 到達はレート 30°/s 準拠(約 3.0s)。3.5s(hold 中)で ±90°
        idx_hold = int(3.5 / STEP_DT)
        assert yaw_outputs[idx_hold] == pytest.approx(math.pi / 2, abs=1e-6)
        # hold 終端(4.0s 直前)まで保持
        idx_hold_end = int(3.98 / STEP_DT)
        assert yaw_outputs[idx_hold_end] == pytest.approx(math.pi / 2,
                                                          abs=1e-6)
        # 完了後: 0° へ戻り、シーケンスは終了(ホバ復帰)
        assert yaw_outputs[-1] == pytest.approx(0.0, abs=1e-6)
        assert controller.trajectory_snapshot() == {"mode": "hover"}


class TestSequenceDropoutFreeze:
    def test_sequence_clock_freezes_during_dropout(self, server_config,
                                                   control_config):
        """MoCap 途絶中はシーケンス時計ごと凍結し、セグメント遷移も起きない。"""
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock, x=0.3, y=0.0)
        segments = [seg_hover(2.0), seg_circle(radius=0.3, period=8.0, laps=1)]
        ok, error = start_sequence(controller, clock, segments)
        assert ok, error
        step_with_pose(controller, clock, 50, x=0.3)   # 1秒(hover 中)
        assert controller.trajectory_snapshot()["seg_index"] == 0

        # pose 供給を止めて 3 秒(壁時計では hover 2s を超える)
        for _ in range(150):
            controller.step(clock.advance(STEP_DT))
        meta = emitted[-1][3]
        assert meta["mocap_dropout"] is True
        # 凍結: セグメントは進まず circle へ遷移しない。経過は途絶検出
        # (0.3s)までのみ進む
        snap = controller.trajectory_snapshot()
        assert snap["mode"] == "sequence"
        assert snap["seg_index"] == 0
        assert 1.0 <= snap["elapsed_s"] <= 1.35
        assert meta["traj_mode"] == TRAJ_MODE_HOVER
        frozen_target = controller.get_target()

        # 復帰: 凍結値から連続再開(復帰 1 フレーム目は frame_hold 超で
        # data_valid=0 のため凍結がもう 1 フレーム続く — 既存仕様)。
        # 残り hover 約 1s 消化後に circle へ遷移する
        step_with_pose(controller, clock, 2, x=0.3)
        assert controller.get_target() == pytest.approx(frozen_target,
                                                        abs=1e-6)
        step_with_pose(controller, clock, 30, x=0.3)   # +0.6s → まだ hover
        assert emitted[-1][3]["traj_mode"] == TRAJ_MODE_HOVER
        step_with_pose(controller, clock, 30, x=0.3)   # +0.6s → circle 遷移済み
        assert emitted[-1][3]["traj_mode"] == TRAJ_MODE_CIRCLE
        assert controller.trajectory_snapshot()["seg_index"] == 1


class TestSessionSequenceCommands:
    """session の traj_sequence_start/stop 配線(shuttle と同型)。"""

    def test_rejected_outside_position_mode(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.traj_sequence_start("ekf2_eval_v1", 0.5) is False
        assert session.position.trajectory_snapshot() == {"mode": "hover"}

    def test_unknown_preset_rejected(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.position.on_mocap_pose(make_pose(t=clock()))
        assert session.traj_sequence_start("no_such_preset", 0.5) is False

    def test_start_and_stop_wiring(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.set_yaw_control(True)
        session.position.on_mocap_pose(make_pose(x=0.0, y=0.0, t=clock()))
        assert session.traj_sequence_start("ekf2_eval_v1", 0.5) is True
        snap = session.position.trajectory_snapshot()
        assert snap["mode"] == "sequence"
        assert snap["name"] == "ekf2_eval_v1"
        assert snap["seg_count"] == 7
        session.traj_sequence_stop()
        assert session.position.trajectory_snapshot() == {"mode": "hover"}

    def test_start_index_passthrough(self, session_factory):
        """開始インデックス指定: yaw 以降を飛ばせばヨー角制御 OFF でも可。"""
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.position.on_mocap_pose(make_pose(t=clock()))
        assert session.traj_sequence_start("ekf2_eval_v1", 0.5,
                                           start_index=6) is True
        snap = session.position.trajectory_snapshot()
        assert snap["seg_index"] == 6
        assert snap["start_index"] == 6

    def test_metadata_recorded_when_logging(self, tmp_path, session_factory):
        """ログ中の開始は trajectory_sequence(定義+開始時刻)を meta.json へ
        追記する。"""
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.set_yaw_control(True)
        session.position.on_mocap_pose(make_pose(t=clock()))
        session.logger.start("position", metadata={"log_columns_version": 6})
        assert session.traj_sequence_start("ekf2_eval_v1", 0.5) is True
        meta_path = session.logger.file_path.with_suffix(".meta.json")
        meta = json.loads(meta_path.read_text(encoding="utf-8"))
        seq = meta["trajectory_sequence"]
        assert seq["name"] == "ekf2_eval_v1"
        assert seq["alt_m"] == pytest.approx(0.5)
        assert seq["start_index"] == 0
        assert len(seq["segments"]) == 7
        assert seq["segments"][0]["type"] == "hover"
        assert "started_at" in seq
        assert meta["log_columns_version"] == 6   # 既存メタは維持
