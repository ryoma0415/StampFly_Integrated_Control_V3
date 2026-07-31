"""ヨー連続性フィルタ(P0-2)の単体テスト。

18:20 飛行(2026-07-31)の実測パターンを合成して検証する:
- 約90°別解グリッチ(単発/3.5s 連続。フレーム間ジャンプ中央値 94.9°/frame)
- 180°フリップ(旧 cont_flip の対象 — 同一機構で棄却)
- 正常回頭 92°/s(フレーム間 0.92° ≪ 30° ゲート — 棄却しない)
- 棄却 >5s での計測への再整列(警告カウント)
- 受信ギャップ >0.5s での再シード
- AttitudeMapper / MocapSource 統合(レート供給・フラグ・pose 契約)
"""

from __future__ import annotations

import math

import pytest

from core.mocap import AttitudeMapper, CoordinateTransformer, MocapSource
from core.yaw_continuity import (
    GAP_RESEED_S, GATE_RAD, REALIGN_S, YawContinuityFilter, wrap_pi,
)

DEG = math.pi / 180.0
FRAME_DT = 0.01   # 100Hz


class TestFilterUnit:
    def test_first_sample_seeds_and_accepts(self):
        filt = YawContinuityFilter()
        y, rejected = filt.update(0.3, 0.0)
        assert y == pytest.approx(0.3)
        assert rejected is False

    def test_single_90deg_glitch_rejected_and_recovers(self):
        """単発 90° グリッチ: 棄却してコースト、次フレームで再受理。"""
        filt = YawContinuityFilter()
        filt.update(0.0, 0.0)
        y, rejected = filt.update(90.0 * DEG, FRAME_DT)   # グリッチ
        assert rejected is True
        assert y == pytest.approx(0.0, abs=1e-9)          # 直近受理値でコースト
        y, rejected = filt.update(0.5 * DEG, 2 * FRAME_DT)  # 正常へ復帰
        assert rejected is False
        assert y == pytest.approx(0.5 * DEG)

    def test_180deg_flip_rejected_same_mechanism(self):
        """180° フリップも同一機構で棄却される(旧 cont_flip の統合)。"""
        filt = YawContinuityFilter()
        filt.update(10.0 * DEG, 0.0)
        y, rejected = filt.update(wrap_pi(10.0 * DEG + math.pi), FRAME_DT)
        assert rejected is True
        assert y == pytest.approx(10.0 * DEG)
        y, rejected = filt.update(10.0 * DEG, 2 * FRAME_DT)
        assert rejected is False

    def test_sustained_glitch_coasts_on_rate_and_reaccepts(self):
        """3.5s 連続グリッチ: 回頭中でも r 伝播で追従し、終了後に再受理。

        真値 ψ(t) = 0.5t [rad](≈28.6°/s の回頭)。t∈[1.0, 4.5) の間
        計測に +95° のグリッチが乗る(18:20 実測の最長 3.56s 相当)。
        """
        filt = YawContinuityFilter()
        rate = 0.5
        t = 0.0
        realigned_output_error = []
        while t < 6.0:
            true_yaw = wrap_pi(rate * t)
            glitch = (95.0 * DEG) if 1.0 <= t < 4.5 else 0.0
            y, rejected = filt.update(wrap_pi(true_yaw + glitch), t,
                                      rate_rad_s=rate)
            if 1.0 <= t < 4.5:
                assert rejected is True, f"t={t}: グリッチは棄却されるべき"
                realigned_output_error.append(abs(wrap_pi(y - true_yaw)))
            elif t >= 4.5 + FRAME_DT:
                assert rejected is False, f"t={t}: 復帰後は再受理されるべき"
                assert abs(wrap_pi(y - true_yaw)) < 1e-6
            t += FRAME_DT
        # 棄却中のコースト出力は r 伝播により真値へ追従している
        assert max(realigned_output_error) < 1e-6
        # 3.5s < 5s なので再整列はしていない
        assert filt.realign_count == 0

    def test_normal_turn_92deg_s_accepted_without_rate(self):
        """正常回頭 92°/s(18:20 実測級): レート無しでも棄却しない。

        フレーム間変化 0.92° ≪ 30° ゲートのため、受理のたびに予測が
        計測へ引き直され、ホールド予測でも追従する。
        """
        filt = YawContinuityFilter()
        turn_rate = 92.0 * DEG
        t = 0.0
        while t < 4.0:   # 368° 回る(ラップ跨ぎも含む)
            y, rejected = filt.update(wrap_pi(turn_rate * t), t)
            assert rejected is False, f"t={t}"
            assert y == pytest.approx(wrap_pi(turn_rate * t))
            t += FRAME_DT

    def test_normal_turn_with_rate_accepted(self):
        """レート供給ありの正常回頭も受理継続(伝播の符号整合)。"""
        filt = YawContinuityFilter()
        turn_rate = 92.0 * DEG
        t = 0.0
        while t < 4.0:
            y, rejected = filt.update(wrap_pi(turn_rate * t), t,
                                      rate_rad_s=turn_rate)
            assert rejected is False, f"t={t}"
            t += FRAME_DT

    def test_reject_longer_than_5s_realigns_with_count(self):
        """棄却 >5s: 計測へ再整列し realign_count が増える(真の喪失復帰)。"""
        filt = YawContinuityFilter()
        filt.update(0.0, 0.0)
        t = FRAME_DT
        realigned_at = None
        while t < 6.0:
            y, rejected = filt.update(90.0 * DEG, t)   # 恒久オフセット
            if not rejected:
                realigned_at = t
                break
            t += FRAME_DT
        assert realigned_at is not None
        assert realigned_at > REALIGN_S              # 5s は粘る
        assert realigned_at < REALIGN_S + 0.1        # 直後に再整列
        assert filt.realign_count == 1
        assert y == pytest.approx(90.0 * DEG)
        # 再整列後は新しい基準で受理が続く
        y, rejected = filt.update(90.5 * DEG, realigned_at + FRAME_DT)
        assert rejected is False

    def test_gap_reseeds_from_measurement(self):
        """受信ギャップ >0.5s: 連続性を主張せず計測で再シードする。"""
        filt = YawContinuityFilter()
        filt.update(0.0, 0.0)
        y, rejected = filt.update(120.0 * DEG, GAP_RESEED_S + 0.1)
        assert rejected is False
        assert y == pytest.approx(120.0 * DEG)
        assert filt.realign_count == 0

    def test_invalidate_forces_reseed(self):
        """invalidate()(前方軸鉛直など)後の次フレームは計測で再シード。"""
        filt = YawContinuityFilter()
        filt.update(0.0, 0.0)
        filt.invalidate()
        y, rejected = filt.update(170.0 * DEG, FRAME_DT)
        assert rejected is False
        assert y == pytest.approx(170.0 * DEG)

    def test_gate_boundary(self):
        """ゲートは |innov| < 30°(29.9° 受理 / 30.1° 棄却)。"""
        filt = YawContinuityFilter()
        filt.update(0.0, 0.0)
        _, rejected = filt.update(GATE_RAD - 0.002, FRAME_DT)
        assert rejected is False
        filt2 = YawContinuityFilter()
        filt2.update(0.0, 0.0)
        _, rejected = filt2.update(GATE_RAD + 0.002, FRAME_DT)
        assert rejected is True


# ----------------------------------------------------------------------
# AttitudeMapper / MocapSource 統合
# ----------------------------------------------------------------------

VALID_TRANSFORM = {
    "x": {"axis": "z", "sign": 1},
    "y": {"axis": "x", "sign": -1},
    "z": {"axis": "y", "sign": 1},
}


def quat_about_y(theta):
    return (0.0, math.sin(theta / 2.0), 0.0, math.cos(theta / 2.0))


class _RigidBody:
    def __init__(self, rb_id, pos, rot):
        self.id_num = rb_id
        self.pos = pos
        self.rot = rot
        self.tracking_valid = True
        self.error = 0.001
        self.rb_marker_list = []


def _make_source() -> MocapSource:
    natnet = {"server_address": "127.0.0.1", "client_address": "127.0.0.1",
              "use_multicast": True, "rigid_body_id": 1}
    return MocapSource(natnet, VALID_TRANSFORM, None,
                       client_factory=lambda: None)


def test_derive_uses_rate_for_coast():
    """棄却中の出力がレート伝播でコーストする(derive 経由)。"""
    transformer = CoordinateTransformer(VALID_TRANSFORM)
    mapper = AttitudeMapper(None)
    state = mapper.new_state()
    q = quat_about_y(10.0 * DEG)
    _, y1, _ = mapper.derive(q, transformer, state, 0.00)
    # 90° グリッチ + 実機は 0.5 rad/s で回頭中(レート供給)
    q_glitch = quat_about_y((10.0 + 90.0) * DEG)
    _, y2, f2 = mapper.derive(q_glitch, transformer, state, 0.10,
                              yaw_rate_rad_s=0.5)
    assert f2 & AttitudeMapper.FLIP_YAW_JUMP
    assert y2 == pytest.approx(wrap_pi(y1 + 0.5 * 0.10), abs=1e-9)


def test_pose_carries_reject_flag_and_realign_count():
    """pose dict 契約: 棄却が flip_flags bit1、yaw_cont_realign が載る。"""
    source = _make_source()
    rb1 = _RigidBody(1, (0.0, 0.0, 0.0), quat_about_y(10.0 * DEG))
    pose1 = source._pose_from_rigid_body(rb1, 1, 1)
    assert pose1["flip_flags"] == 0
    assert pose1["yaw_cont_realign"] == 0
    # 直後(dt ≈ µs)に 180° 別解 → 棄却フラグ+ヨーは直前値でコースト
    rb2 = _RigidBody(1, (0.0, 0.0, 0.0),
                     quat_about_y((10.0 + 180.0) * DEG))
    pose2 = source._pose_from_rigid_body(rb2, 1, 2)
    assert pose2["flip_flags"] & AttitudeMapper.FLIP_YAW_JUMP
    assert pose2["yaw_true_rad"] == pytest.approx(pose1["yaw_true_rad"],
                                                  abs=1e-3)
    # heading(生値)は契約どおりフィルタされない(180° 跳んだまま)
    assert abs(wrap_pi(pose2["heading_rad"] - pose1["heading_rad"])
               ) == pytest.approx(math.pi, abs=1e-6)


def test_set_primary_yaw_rate_is_consumed():
    """set_primary_yaw_rate の値が primary の連続性フィルタへ届く。

    静止計測に対しレートだけ与えると、予測が計測から離れて棄却が起きる
    (= レートが実際に伝播へ使われている証拠)。primary 以外には適用
    されない。
    """
    source = _make_source()
    q = quat_about_y(10.0 * DEG)
    source._pose_from_rigid_body(_RigidBody(1, (0, 0, 0), q), 1, 1)
    # 大レート(10 rad/s)を注入 → 0.1s 相当の予測移動 ≈ 57° > 30° ゲート
    source.set_primary_yaw_rate(10.0)
    filt = source._mapping[2][1]["yaw_filter"]
    filt._last_t -= 0.1   # フレーム間 0.1s を擬似(t_mono は実時間のため)
    pose = source._pose_from_rigid_body(_RigidBody(1, (0, 0, 0), q), 1, 2)
    assert pose["flip_flags"] & AttitudeMapper.FLIP_YAW_JUMP
