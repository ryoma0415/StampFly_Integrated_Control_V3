"""移動ベースヨー推定器(core/motion_yaw.py)の合成データ単体テスト。

PoC(docs/analysis_20260727/scripts/poc_yaw_from_motion.py)のモデル
a_W = e^{jψ}·u_B に従う合成データ(正弦傾き+整合する位置軌道)から
既知ヨー ψ を回復できることを検証する。座標規約(U = g·e^{-jπ/2}·
(tanθ + j·tanφ/cosθ))と遅延補償(テレメトリ遅延 90ms: A[n]↔U[n-lag])
も合成データ側で同じ規約に合わせて再現する。
"""

from __future__ import annotations

import cmath
import math

import pytest

from core.motion_yaw import G_MPS2, MotionYawEstimator

DT = 0.02          # 50Hz
DELAY_S = 0.09     # 既定のテレメトリ遅延補償
LAG_N = round(DELAY_S / DT)


def _rot(z: complex) -> complex:
    """PoC 規約の回転(rot=270° = -j)。"""
    return (-1j) * z


def _feed_linear_excitation(est: MotionYawEstimator, psi: float,
                            n_ticks: int, tilt_amp_rad: float = 0.035,
                            freq_hz: float = 0.5) -> None:
    """直線往復励振(pitch のみ正弦、roll=0)で n_ticks ぶん feed する。

    位置は a_W = e^{jψ}·U の厳密解(正弦の2階積分 = -a/ω²)から生成する。
    テレメトリ遅延を再現するため、tick n で渡す roll/pitch は位置より
    LAG_N サンプル古い時刻の姿勢とする(実機ではテレメトリが遅れて届く)。
    直線励振なら 2階差分の半サンプル位相残差はヨーに乗らない(u の方向が
    時不変のため)。
    """
    omega = 2.0 * math.pi * freq_hz
    e_psi = cmath.exp(1j * psi)
    for n in range(n_ticks):
        t = n * DT
        # 姿勢(テレメトリ値): 遅延の再現 → 時刻 t-延滞 の姿勢を渡す
        t_tlm = t - LAG_N * DT
        pitch = tilt_amp_rad * math.sin(omega * t_tlm)
        roll = 0.0
        # 位置: a_W(t) = e^{jψ}·g·rot·(tan(pitch(t)) + j·…) の2階積分。
        # tan の高調波は小傾角では無視できる(tan x ≈ x、誤差 <0.003%)
        u_t = G_MPS2 * _rot(complex(tilt_amp_rad * math.sin(omega * t), 0.0))
        a_w = e_psi * u_t
        p = -a_w / (omega ** 2)
        est.add_sample(p.real, p.imag, roll, pitch)


class TestMotionYawRecovery:
    def test_recovers_known_yaw_from_sine_motion(self):
        """正弦運動(傾き2°・0.5Hz・±3.5cm 相当)から既知ヨーを ±2° で回復。"""
        psi_true = 0.6   # rad(約 34.4°)
        est = MotionYawEstimator(dt_s=DT)
        _feed_linear_excitation(est, psi_true, n_ticks=750)   # 15s
        result = est.estimate()
        assert result["valid"], result
        assert result["j"] >= 9.0
        err = abs((result["yaw_rad"] - psi_true + math.pi)
                  % (2.0 * math.pi) - math.pi)
        assert math.degrees(err) < 2.0, math.degrees(err)

    def test_recovers_negative_yaw(self):
        psi_true = -2.2
        est = MotionYawEstimator(dt_s=DT)
        _feed_linear_excitation(est, psi_true, n_ticks=750)
        result = est.estimate()
        assert result["valid"], result
        err = abs((result["yaw_rad"] - psi_true + math.pi)
                  % (2.0 * math.pi) - math.pi)
        assert math.degrees(err) < 2.0, math.degrees(err)

    def test_no_excitation_is_invalid(self):
        """静止(傾き一定・位置一定)では Fisher情報ゲートで invalid。"""
        est = MotionYawEstimator(dt_s=DT)
        for _ in range(750):
            est.add_sample(0.10, -0.20, 0.01, 0.02)
        result = est.estimate()
        assert not result["valid"]
        assert result["yaw_rad"] is None
        assert result["j"] < 9.0

    def test_gap_resets_and_window_expires(self):
        """有効推定後、長いギャップで窓が期限切れになり invalid へ戻る。"""
        est = MotionYawEstimator(dt_s=DT)
        _feed_linear_excitation(est, 0.6, n_ticks=750)
        assert est.estimate()["valid"]
        for _ in range(500):   # 10s > 窓 8s
            est.add_gap()
        result = est.estimate()
        assert not result["valid"]
        assert result["yaw_rad"] is None

    def test_recovery_after_gap(self):
        """ギャップ後に再励振すれば再び valid になる(フィルタ再整定)。"""
        est = MotionYawEstimator(dt_s=DT)
        _feed_linear_excitation(est, 0.6, n_ticks=400)
        for _ in range(100):
            est.add_gap()
        _feed_linear_excitation(est, 0.6, n_ticks=750)
        result = est.estimate()
        assert result["valid"], result
        err = abs((result["yaw_rad"] - 0.6 + math.pi)
                  % (2.0 * math.pi) - math.pi)
        assert math.degrees(err) < 2.0, math.degrees(err)
