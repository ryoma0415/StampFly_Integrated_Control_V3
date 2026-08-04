"""v2 軌道モード(円・シャトル): 合流位相・回転方向・接線追従・停止復帰・
自動停止・ログ meta。"""

from __future__ import annotations

import math

import pytest

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


def step_with_pose(controller, clock, n, x=0.3, y=0.0):
    """pose を供給しながら n 周期ぶん step する(途絶させない)。

    MoCap 途絶(>300ms)中は軌道位相が凍結されるため、軌道の時間発展を
    見るテストは pose を供給し続ける必要がある。
    """
    for _ in range(n):
        clock.advance(STEP_DT)
        feed_pose(controller, clock, x=x, y=y)
        controller.step(clock())


class TestCircleStart:
    def test_requires_position(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        ok, error = controller.start_circle(0.0, 0.0, 0.3, 30.0, True, 0.5,
                                            False, now=clock())
        assert not ok
        assert "MoCap" in error

    def test_param_validation(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.5, y=0.0)
        # 半径・周期・中心・高度の範囲外は拒否(control.json trajectory 節)
        assert not controller.start_circle(0.0, 0.0, 99.0, 30.0, True, 0.5,
                                           False, now=clock())[0]
        assert not controller.start_circle(0.0, 0.0, 0.3, 0.5, True, 0.5,
                                           False, now=clock())[0]
        assert not controller.start_circle(9.0, 0.0, 0.3, 30.0, True, 0.5,
                                           False, now=clock())[0]
        assert not controller.start_circle(0.0, 0.0, 0.3, 30.0, True, 99.0,
                                           False, now=clock())[0]

    def test_nan_center_rejected(self, server_config, control_config):
        """NaN 中心は閉区間ガードで拒否される(従来の abs 比較は NaN を
        素通しし、NaN 目標 → クランプ飽和誤差の送信に至った)。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.5, y=0.0)
        ok, error = controller.start_circle(float("nan"), 0.0, 0.3, 30.0,
                                            True, 0.5, False, now=clock())
        assert not ok and "中心" in error

    def test_requires_fresh_position(self, server_config, control_config):
        """MoCap 途絶中(>300ms)は古い位置から合流位相を決めず開始を拒否する。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.5, y=0.0)
        clock.advance(1.0)   # mocap_dropout_level_s(0.3s)超過
        ok, error = controller.start_circle(0.0, 0.0, 0.3, 30.0, True, 0.5,
                                            False, now=clock())
        assert not ok
        assert "新鮮" in error

    def test_face_tangent_requires_feasible_period(self, server_config,
                                                   control_config):
        """接線ヨー角速度(360°/周期)がヨースルーレートを超える周期は拒否。

        yaw_slew_rate_deg_per_s=45 → 最小周期 8s。下回ると整形ヨーが
        鋸歯状に発振するため start_circle で拒否する。
        """
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.3, y=0.0)
        ok, error = controller.start_circle(0.0, 0.0, 0.3, 4.0, False, 0.5,
                                            True, now=clock())
        assert not ok
        assert "周期" in error
        # 境界(8s)は許可。face_tangent OFF なら短周期も従来どおり許可
        assert controller.start_circle(0.0, 0.0, 0.3, 8.0, False, 0.5,
                                       True, now=clock())[0]
        assert controller.start_circle(0.0, 0.0, 0.3, 4.0, False, 0.5,
                                       False, now=clock())[0]

    def test_merges_at_nearest_point(self, server_config, control_config):
        """現在位置から円周最近傍点(同じ方位角)に位相を合わせる。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.5, y=0.5)   # 中心(0,0) から 45° 方向
        ok, error = controller.start_circle(0.0, 0.0, 0.3, 30.0, False, 0.5,
                                            False, now=clock())
        assert ok, error
        controller.step(clock())   # 軌道目標を確定
        tx, ty, tz = controller.get_target()
        # 目標は 45° 方向の円周上の点
        assert tx == pytest.approx(0.3 * math.cos(math.radians(45.0)), abs=1e-6)
        assert ty == pytest.approx(0.3 * math.sin(math.radians(45.0)), abs=1e-6)
        assert tz == pytest.approx(0.5)


class TestCircleMotion:
    def _start(self, server_config, control_config, clockwise,
               face_tangent=False, period_s=8.0):
        controller, emitted, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.3, y=0.0)   # 位相 0 から開始
        ok, error = controller.start_circle(0.0, 0.0, 0.3, period_s, clockwise,
                                            0.5, face_tangent, now=clock())
        assert ok, error
        return controller, emitted, clock

    def test_ccw_phase_advances(self, server_config, control_config):
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=False)
        controller.step(clock())
        # 2秒 = 周期8秒の 1/4 → 位相 +90°
        step_with_pose(controller, clock, 100)
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(0.0, abs=1e-6)
        assert ty == pytest.approx(0.3, abs=1e-6)
        meta = emitted[-1][3]
        assert meta["traj_mode"] == TRAJ_MODE_CIRCLE
        assert meta["traj_phase_rad"] == pytest.approx(math.pi / 2, abs=1e-6)

    def test_cw_phase_recedes(self, server_config, control_config):
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=True)
        controller.step(clock())
        step_with_pose(controller, clock, 100)   # 2秒 → 位相 -90°
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(0.0, abs=1e-6)
        assert ty == pytest.approx(-0.3, abs=1e-6)

    def test_face_tangent_follows_heading(self, server_config, control_config):
        """「進行方向を向く」ON かつヨー制御 ON → yaw_ref が接線方向へ追従。"""
        # 周期 32s(接線方位の角速度 11.25°/s < ヨースルー 45°/s)なら追従できる
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=False,
                                                 face_tangent=True,
                                                 period_s=32.0)
        controller.set_yaw_control(True)
        # 位相 0 のとき CCW の接線方向は +90°。初期ギャップ 90° は
        # (45−11.25)°/s で詰まり、約 2.7 秒で追いつく
        step_with_pose(controller, clock, 150)   # 3秒
        meta = emitted[-1][3]
        phase = meta["traj_phase_rad"]
        expected_heading = wrap_pi(phase + math.pi / 2)
        assert abs(wrap_pi(meta["yaw_ref_rad"] - expected_heading)) \
            < math.radians(2.0)
        assert meta["yaw_ctrl_on"] is True

    def test_face_tangent_ignored_when_yaw_off(self, server_config,
                                               control_config):
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=False,
                                                 face_tangent=True)
        # ヨー制御 OFF のままなら yaw_ref は動かない(flags bit1 も立たない)
        step_with_pose(controller, clock, 50)
        meta = emitted[-1][3]
        assert meta["yaw_ctrl_on"] is False
        assert meta["yaw_ref_rad"] == pytest.approx(0.0)

    def test_stop_returns_to_hover_at_current_target(self, server_config,
                                                     control_config):
        controller, emitted, clock = self._start(server_config, control_config,
                                                 clockwise=False)
        step_with_pose(controller, clock, 50)
        frozen = controller.get_target()
        controller.stop_circle()
        step_with_pose(controller, clock, 10)
        assert controller.get_target() == pytest.approx(frozen)
        meta = emitted[-1][3]
        assert meta["traj_mode"] == TRAJ_MODE_HOVER
        assert meta["traj_phase_rad"] is None
        assert controller.trajectory_snapshot() == {"mode": "hover"}


class TestDropoutPhaseFreeze:
    def test_phase_freezes_during_dropout_and_resumes(self, server_config,
                                                      control_config):
        """MoCap 途絶中は軌道位相を凍結し、復帰後に連続再開する。"""
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock, x=0.3, y=0.0)
        ok, error = controller.start_circle(0.0, 0.0, 0.3, 8.0, False, 0.5,
                                            False, now=clock())
        assert ok, error
        step_with_pose(controller, clock, 50)   # 1秒 → 位相 45°
        phase_before = emitted[-1][3]["traj_phase_rad"]
        assert phase_before == pytest.approx(math.pi / 4, abs=1e-6)

        # pose 供給を止めて 1 秒(>300ms 途絶)→ 位相・目標は凍結される
        for _ in range(50):
            controller.step(clock.advance(STEP_DT))
        meta = emitted[-1][3]
        assert meta["mocap_dropout"] is True
        # 途絶検出(0.3s)までは進む: 位相の上限は 45°+0.3s×45°/s
        assert phase_before <= meta["traj_phase_rad"] \
            <= phase_before + (2 * math.pi / 8.0) * 0.35
        frozen_phase = meta["traj_phase_rad"]
        frozen_target = controller.get_target()
        controller.step(clock.advance(STEP_DT))
        assert emitted[-1][3]["traj_phase_rad"] == pytest.approx(frozen_phase)
        assert controller.get_target() == pytest.approx(frozen_target)

        # 復帰: t0 が前送りされ、位相は凍結値から連続に進む(ジャンプしない)。
        # 復帰後の最初のフレームは frame_hold(300ms)超のギャップで
        # data_valid=0 のため位相凍結がもう1フレーム続く(仕様)。
        # 2フレーム目から再開する
        step_with_pose(controller, clock, 2)
        resumed = emitted[-1][3]["traj_phase_rad"]
        assert abs(wrap_pi(resumed - frozen_phase)) \
            <= (2 * math.pi / 8.0) * STEP_DT + 1e-6
        step_with_pose(controller, clock, 50)   # さらに1秒 → +45°
        assert wrap_pi(emitted[-1][3]["traj_phase_rad"] - resumed) \
            == pytest.approx(math.pi / 4, abs=1e-2)


class TestMocapYawMeta:
    def test_mocap_yaw_deg_in_meta(self, server_config, control_config):
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock, yaw_rad=math.radians(25.0))
        controller.step(clock.advance(STEP_DT))
        meta = emitted[-1][3]
        assert meta["mocap_yaw_deg"] == pytest.approx(25.0)
        assert meta["traj_mode"] == TRAJ_MODE_HOVER


# ----------------------------------------------------------------------
# シャトル(直線往復)軌道
# ----------------------------------------------------------------------


def start_shuttle(controller, clock, center_x=0.0, center_y=0.0,
                  axis_deg=0.0, amplitude_m=0.3, period_s=8.0,
                  cycles=0, alt_m=0.5):
    return controller.start_shuttle(center_x, center_y, axis_deg, amplitude_m,
                                    period_s, cycles, alt_m, now=clock())


class TestShuttleStart:
    def test_requires_position(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        ok, error = start_shuttle(controller, clock)
        assert not ok
        assert "MoCap" in error

    def test_amplitude_guard(self, server_config, control_config):
        """振幅は shuttle_amplitude_min/max_m の範囲外を拒否する。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_shuttle(controller, clock, amplitude_m=0.01)
        assert not ok and "振幅" in error
        ok, error = start_shuttle(controller, clock, amplitude_m=0.9)
        assert not ok and "振幅" in error

    def test_period_guard(self, server_config, control_config):
        """周期は circle と共通の period_min/max_s を使う。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_shuttle(controller, clock, period_s=0.5)
        assert not ok and "周期" in error
        ok, error = start_shuttle(controller, clock, period_s=999.0)
        assert not ok and "周期" in error

    def test_speed_guard(self, server_config, control_config):
        """v_max = 2πA/T が speed_max_mps(0.5)を超える組合せを拒否する。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        # A=0.3, T=3 → v_max = 0.628 m/s > 0.5
        ok, error = start_shuttle(controller, clock, amplitude_m=0.3,
                                  period_s=3.0)
        assert not ok and "速度" in error
        # A=0.3, T=4 → v_max = 0.471 m/s ≤ 0.5 は許可
        assert start_shuttle(controller, clock, amplitude_m=0.3,
                             period_s=4.0)[0]

    def test_excursion_guard(self, server_config, control_config):
        """両端点 center±A·e の |x|,|y| が excursion_abs_max_m を超えたら拒否。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        # 端点 x = 0.4 + 0.2 = 0.6 > 0.5
        ok, error = start_shuttle(controller, clock, center_x=0.4,
                                  amplitude_m=0.2)
        assert not ok and "端点" in error
        # 軸 90° なら端点 y = ±0.2(中心 x=0.4 は範囲内)で許可
        assert start_shuttle(controller, clock, center_x=0.4, axis_deg=90.0,
                             amplitude_m=0.2)[0]

    def test_nan_center_and_axis_rejected(self, server_config,
                                          control_config):
        """NaN の中心/軸方位は閉区間ガードで拒否される(NaN 目標 →
        位置誤差 NaN → クランプ飽和誤差の送信に至るため)。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_shuttle(controller, clock, center_x=float("nan"))
        assert not ok and "端点" in error
        ok, error = start_shuttle(controller, clock, axis_deg=float("nan"))
        assert not ok and "端点" in error

    def test_cycles_and_alt_guard(self, server_config, control_config):
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        ok, error = start_shuttle(controller, clock, cycles=-1)
        assert not ok and "サイクル" in error
        ok, error = start_shuttle(controller, clock, alt_m=99.0)
        assert not ok and "高度" in error

    def test_requires_fresh_position(self, server_config, control_config):
        """MoCap 途絶中(>300ms)は古い位置から合流位相を決めず開始を拒否する。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock)
        clock.advance(1.0)   # mocap_dropout_level_s(0.3s)超過
        ok, error = start_shuttle(controller, clock)
        assert not ok
        assert "新鮮" in error

    def test_requires_valid_data(self, server_config, control_config):
        """トラッキング喪失などデータ無効中は開始を拒否する。"""
        controller, _, clock = make_controller(server_config, control_config)
        controller.on_mocap_pose(make_pose(t=clock(), tracking_valid=False))
        ok, error = start_shuttle(controller, clock)
        assert not ok
        assert "無効" in error

    def test_merges_at_projected_point(self, server_config, control_config):
        """現在位置を軸へ射影した点に位相を合わせる(phase0 = asin(s/A))。"""
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock, x=0.15, y=0.2)   # 軸 0° → 射影 s=0.15
        ok, error = start_shuttle(controller, clock, amplitude_m=0.3)
        assert ok, error
        controller.step(clock())
        tx, ty, tz = controller.get_target()
        assert tx == pytest.approx(0.15, abs=1e-6)   # A·sin(asin(0.5)) = s
        assert ty == pytest.approx(0.0, abs=1e-6)    # 軸外成分は無視
        assert tz == pytest.approx(0.5)
        meta = emitted[-1][3]
        assert meta["traj_mode"] == TRAJ_MODE_SHUTTLE
        assert meta["traj_phase_rad"] == pytest.approx(math.asin(0.5), abs=1e-6)

    def test_merge_projection_clamped(self, server_config, control_config):
        """軸射影が振幅を超える位置からは端点(phase=π/2)に合流する。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.5, y=0.0)
        ok, error = start_shuttle(controller, clock, amplitude_m=0.3)
        assert ok, error
        controller.step(clock())
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(0.3, abs=1e-6)
        assert ty == pytest.approx(0.0, abs=1e-6)

    def test_merge_rotated_axis(self, server_config, control_config):
        """軸 90°(e=(0,1))では y 成分が射影される。"""
        controller, _, clock = make_controller(server_config, control_config)
        feed_pose(controller, clock, x=0.2, y=0.15)
        ok, error = start_shuttle(controller, clock, axis_deg=90.0,
                                  amplitude_m=0.3)
        assert ok, error
        controller.step(clock())
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(0.0, abs=1e-6)
        assert ty == pytest.approx(0.15, abs=1e-6)


class TestShuttleMotion:
    def _start(self, server_config, control_config, **kwargs):
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock, x=kwargs.pop("pose_x", 0.0),
                  y=kwargs.pop("pose_y", 0.0))
        ok, error = start_shuttle(controller, clock, **kwargs)
        assert ok, error
        return controller, emitted, clock

    def test_sinusoidal_motion(self, server_config, control_config):
        """target = center + A·sin(phase)·e。T/4 で +A、3T/4 で −A に達する。"""
        controller, emitted, clock = self._start(server_config, control_config)
        controller.step(clock())
        # pose は開始位置(0,0)のまま供給する(位置ジャンプで外れ値判定→
        # 位相凍結になるのを避ける。位相進行は data_valid のみに依存)
        step_with_pose(controller, clock, 100, x=0.0)   # 2秒 = T/4 → phase π/2
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(0.3, abs=1e-6)
        assert ty == pytest.approx(0.0, abs=1e-6)
        meta = emitted[-1][3]
        assert meta["traj_mode"] == TRAJ_MODE_SHUTTLE
        assert meta["traj_phase_rad"] == pytest.approx(math.pi / 2, abs=1e-6)
        step_with_pose(controller, clock, 200, x=0.0)   # +4秒 → 3π/2(反対端)
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(-0.3, abs=1e-6)
        assert ty == pytest.approx(0.0, abs=1e-6)

    def test_rotated_axis_motion(self, server_config, control_config):
        controller, _, clock = self._start(server_config, control_config,
                                           axis_deg=45.0)
        step_with_pose(controller, clock, 100, x=0.0)   # T/4 → 端点 A·e(45°)
        tx, ty, _ = controller.get_target()
        expected = 0.3 * math.cos(math.radians(45.0))
        assert tx == pytest.approx(expected, abs=1e-6)
        assert ty == pytest.approx(expected, abs=1e-6)

    def test_stop_trajectory_returns_to_hover(self, server_config,
                                              control_config):
        controller, emitted, clock = self._start(server_config, control_config)
        step_with_pose(controller, clock, 50, x=0.0)
        frozen = controller.get_target()
        controller.stop_trajectory()
        step_with_pose(controller, clock, 10, x=0.0)
        assert controller.get_target() == pytest.approx(frozen)
        meta = emitted[-1][3]
        assert meta["traj_mode"] == TRAJ_MODE_HOVER
        assert meta["traj_phase_rad"] is None
        assert controller.trajectory_snapshot() == {"mode": "hover"}

    def test_stop_circle_also_stops_shuttle(self, server_config,
                                            control_config):
        """既存 API stop_circle は共通 stop の別名としてシャトルも止める。"""
        controller, _, clock = self._start(server_config, control_config)
        controller.stop_circle()
        assert controller.trajectory_snapshot() == {"mode": "hover"}

    def test_reset_control_clears_shuttle(self, server_config, control_config):
        controller, _, clock = self._start(server_config, control_config)
        controller.reset_control()
        assert controller.trajectory_snapshot() == {"mode": "hover"}

    def test_snapshot_fields_and_cycles_remaining(self, server_config,
                                                  control_config):
        controller, _, clock = self._start(server_config, control_config,
                                           pose_x=0.1, pose_y=-0.1,
                                           center_x=0.1, center_y=-0.1,
                                           axis_deg=30.0, amplitude_m=0.2,
                                           cycles=2)
        snap = controller.trajectory_snapshot()
        assert snap["mode"] == "shuttle"
        assert snap["center_x"] == pytest.approx(0.1)
        assert snap["center_y"] == pytest.approx(-0.1)
        assert snap["axis_deg"] == pytest.approx(30.0)
        assert snap["amplitude_m"] == pytest.approx(0.2)
        assert snap["period_s"] == pytest.approx(8.0)
        assert snap["cycles"] == 2
        assert snap["alt_m"] == pytest.approx(0.5)
        assert snap["phase_rad"] == pytest.approx(0.0, abs=1e-6)
        assert snap["cycles_remaining"] == pytest.approx(2.0, abs=1e-6)
        # 1周期 = 1サイクル消化(pose は開始位置のまま供給)
        step_with_pose(controller, clock, 400, x=0.1, y=-0.1)
        snap = controller.trajectory_snapshot()
        assert snap["cycles_remaining"] == pytest.approx(1.0, abs=0.02)

    def test_snapshot_continuous_has_no_remaining(self, server_config,
                                                  control_config):
        controller, _, clock = self._start(server_config, control_config,
                                           cycles=0)
        snap = controller.trajectory_snapshot()
        assert snap["cycles"] == 0
        assert snap["cycles_remaining"] is None


class TestShuttleAutoStop:
    def test_stops_at_next_extremum_after_cycles(self, server_config,
                                                 control_config):
        """cycles·2π 消化後、次の極値(速度ゼロ点)で停止し端点をホールドする。

        phase0=0・cycles=1 → 2π 消化時は中点(速度最大)なので止まらず、
        5π/2(端点 +A)まで進んでから停止する。
        """
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock)   # (0,0) → phase0=0
        ok, error = start_shuttle(controller, clock, cycles=1)
        assert ok, error
        # 8.4 秒(位相 2.1π > 2π)ではまだ停止しない(極値未到達)
        step_with_pose(controller, clock, 420, x=0.0)
        assert emitted[-1][3]["traj_mode"] == TRAJ_MODE_SHUTTLE
        # 10 秒(位相 5π/2)で端点に達して自動停止 → hover 復帰
        step_with_pose(controller, clock, 85, x=0.0)
        meta = emitted[-1][3]
        assert meta["traj_mode"] == TRAJ_MODE_HOVER
        assert meta["traj_phase_rad"] is None
        assert controller.trajectory_snapshot() == {"mode": "hover"}
        tx, ty, tz = controller.get_target()
        assert tx == pytest.approx(0.3, abs=1e-6)   # 端点 center+A·e ちょうど
        assert ty == pytest.approx(0.0, abs=1e-6)
        assert tz == pytest.approx(0.5)
        # 以後は端点ホールドのまま
        step_with_pose(controller, clock, 50, x=0.0)
        assert controller.get_target() == pytest.approx((0.3, 0.0, 0.5))

    def test_endpoint_start_stops_after_exact_cycles(self, server_config,
                                                     control_config):
        """端点(phase0=π/2)から開始した場合は cycles·2π ちょうどで停止する。"""
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock, x=0.3)   # 射影 s=A → phase0=π/2
        ok, error = start_shuttle(controller, clock, cycles=1)
        assert ok, error
        step_with_pose(controller, clock, 405)   # 1周期(8s)+余裕
        assert emitted[-1][3]["traj_mode"] == TRAJ_MODE_HOVER
        tx, ty, _ = controller.get_target()
        assert tx == pytest.approx(0.3, abs=1e-6)
        assert ty == pytest.approx(0.0, abs=1e-6)

    def test_cycles_zero_runs_until_manual_stop(self, server_config,
                                                control_config):
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock)
        ok, error = start_shuttle(controller, clock, cycles=0)
        assert ok, error
        step_with_pose(controller, clock, 600, x=0.0)   # 12秒 = 1.5周期でも継続
        assert emitted[-1][3]["traj_mode"] == TRAJ_MODE_SHUTTLE
        assert controller.trajectory_snapshot()["mode"] == "shuttle"


class TestShuttleDropoutPhaseFreeze:
    def test_phase_freezes_during_dropout_and_resumes(self, server_config,
                                                      control_config):
        """MoCap 途絶中は位相を凍結し、復帰後に連続再開する(circle と共通)。"""
        controller, emitted, clock = make_controller(server_config,
                                                     control_config)
        feed_pose(controller, clock)
        ok, error = start_shuttle(controller, clock)   # T=8s, phase0=0
        assert ok, error
        step_with_pose(controller, clock, 50, x=0.0)   # 1秒 → 位相 45°
        phase_before = emitted[-1][3]["traj_phase_rad"]
        assert phase_before == pytest.approx(math.pi / 4, abs=1e-6)

        # pose 供給を止めて 1 秒(>300ms 途絶)→ 位相・目標は凍結される
        for _ in range(50):
            controller.step(clock.advance(STEP_DT))
        meta = emitted[-1][3]
        assert meta["mocap_dropout"] is True
        # 途絶検出(0.3s)までは進む: 位相の上限は 45°+0.3s×45°/s
        assert phase_before <= meta["traj_phase_rad"] \
            <= phase_before + (2 * math.pi / 8.0) * 0.35
        frozen_phase = meta["traj_phase_rad"]
        frozen_target = controller.get_target()
        controller.step(clock.advance(STEP_DT))
        assert emitted[-1][3]["traj_phase_rad"] == pytest.approx(frozen_phase)
        assert controller.get_target() == pytest.approx(frozen_target)

        # 復帰: t0 が前送りされ、位相は凍結値から連続に進む(ジャンプしない)。
        # 復帰後の最初のフレームは frame_hold(300ms)超のギャップで
        # data_valid=0 のため位相凍結がもう1フレーム続く(仕様)。
        step_with_pose(controller, clock, 2, x=0.0)
        resumed = emitted[-1][3]["traj_phase_rad"]
        assert abs(wrap_pi(resumed - frozen_phase)) \
            <= (2 * math.pi / 8.0) * STEP_DT + 1e-6
        step_with_pose(controller, clock, 50, x=0.0)   # さらに1秒 → +45°
        assert wrap_pi(emitted[-1][3]["traj_phase_rad"] - resumed) \
            == pytest.approx(math.pi / 4, abs=1e-2)


class TestSessionShuttleCommands:
    """session の shuttle_start/shuttle_stop 配線(circle_start/stop と同型)。"""

    def test_rejected_outside_position_mode(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        # 既定は posture モード → MODE_POSITION ガードで拒否
        assert session.shuttle_start(0.0, 0.0, 0.0, 0.3, 8.0, 0, 0.5) is False
        assert session.position.trajectory_snapshot() == {"mode": "hover"}

    def test_start_and_stop_wiring(self, session_factory):
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.position.on_mocap_pose(make_pose(x=0.0, y=0.0, t=clock()))
        assert session.shuttle_start(0.0, 0.0, 0.0, 0.3, 8.0, 0, 0.5) is True
        snap = session.position.trajectory_snapshot()
        assert snap["mode"] == "shuttle"
        assert snap["amplitude_m"] == pytest.approx(0.3)
        session.shuttle_stop()
        assert session.position.trajectory_snapshot() == {"mode": "hover"}

    def test_start_reports_position_error(self, session_factory):
        """PositionController の検証エラー(範囲外振幅)は warn になり False。"""
        session, transport, clock = session_factory()
        session.connect("FAKE")
        halt_supervisor(session)
        assert session.set_mode(MODE_POSITION)
        session.position.on_mocap_pose(make_pose(x=0.0, y=0.0, t=clock()))
        assert session.shuttle_start(0.0, 0.0, 0.0, 99.0, 8.0, 0, 0.5) is False
        assert session.position.trajectory_snapshot() == {"mode": "hover"}
