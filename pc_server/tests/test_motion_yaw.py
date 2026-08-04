"""移動ベースヨー推定器(core/motion_yaw.py)の合成データ単体テスト。

PoC(docs/analysis_20260727/scripts/poc_yaw_from_motion.py)のモデル
a_W = e^{jψ}·u_B + λ·v_W に従う合成データから既知ヨー ψ を回復できることを
検証する。座標規約(U = g·e^{-jπ/2}·(tanθ + j·tanφ/cosθ))と遅延補償
(遅い側はフィルタ済 MoCap 位置経路 ≈90ms: A[n]↔U[n-lag])も合成データ側で
同じ規約に合わせて再現する。位置系列を POS_DELAY_N tick 遅らせて与えると、
推定器の2階差分(エポック [n-1] 中心)込みで A と U の対が厳密に一致する。

v2 追加分の検証:
- ジャイロ derotation(回頭中の窓平均ラグ除去。符号は実ログ
  20260731_211535 リプレイで確定済み — motion_yaw.py の _DEROT_SIGN 参照)
- 速度回帰項(A = α·U + λ·V の2回帰子 LS。空力バイアス吸収)
- 遅延補償の「方向」(2軸円運動でのみ検出可能 — 1軸励振は時間シフトの
  向きに不感なため、旧テストは方向誤りを検出できなかった)
- lag_n の half-up 丸め(banker's 丸め回帰の防止)
"""

from __future__ import annotations

import cmath
import math

import pytest

from core.motion_yaw import G_MPS2, MotionYawEstimator

DT = 0.02          # 50Hz
DELAY_S = 0.09     # 既定の経路遅延補償
LAG_N = int(DELAY_S / DT + 0.5)   # = 5(half-up。banker's 丸めだと 4 に落ちる)
# 合成データで位置経路に与える遅延 tick 数。推定器の2階差分はエポック
# [n-1] 中心なので、厳密対合には lag より 1 tick 少ない遅延を与える。
POS_DELAY_N = LAG_N - 1


def _rot(z: complex) -> complex:
    """PoC 規約の回転(rot=270° = -j)。"""
    return (-1j) * z


def _wrap_deg(err_rad: float) -> float:
    """ラップ済み誤差 [deg]。"""
    return math.degrees(abs((err_rad + math.pi) % (2.0 * math.pi) - math.pi))


def _feed_linear_excitation(est: MotionYawEstimator, psi: float,
                            n_ticks: int, tilt_amp_rad: float = 0.035,
                            freq_hz: float = 0.5) -> None:
    """直線往復励振(pitch のみ正弦、roll=0)で n_ticks ぶん feed する。

    位置は a_W = e^{jψ}·U の厳密解(正弦の2階積分 = -a/ω²)から生成する。
    実機ではフィルタ済 MoCap 位置経路が遅いので、tick n で渡す位置は傾きより
    POS_DELAY_N サンプル古い時刻の値とする。直線励振なら遅延の向き・残差は
    ヨーに乗らない(u の方向が時不変のため)。tan の高調波は小傾角では
    無視できる(tan x ≈ x、誤差 <0.003%)。
    """
    omega = 2.0 * math.pi * freq_hz
    e_psi = cmath.exp(1j * psi)
    for n in range(n_ticks):
        t = n * DT
        pitch = tilt_amp_rad * math.sin(omega * t)      # 傾き: 現在時刻(新鮮側)
        t_pos = t - POS_DELAY_N * DT                    # 位置経路: 遅い側
        u_pos = G_MPS2 * _rot(
            complex(tilt_amp_rad * math.sin(omega * t_pos), 0.0))
        a_w = e_psi * u_pos
        p = -a_w / (omega ** 2)
        est.add_sample(p.real, p.imag, 0.0, pitch)


def _feed_circular_excitation(est: MotionYawEstimator, psi: float,
                              n_ticks: int, pos_delay_ticks: int,
                              tilt_amp_rad: float = 0.035,
                              freq_hz: float = 0.5) -> None:
    """円運動励振(傾きベクトルが複素平面で回転)+位置経路遅延の合成データ。

    2軸励振なので、遅延補償の「方向」誤りが位相バイアス 2ω·τ として現れる
    (1軸では検出不可)。pos_delay_ticks に POS_DELAY_N を与えると推定器の
    補償と厳密に整合し、負値を与えると逆方向遅延(補償が仇になる系)を再現。
    """
    omega = 2.0 * math.pi * freq_hz
    e_psi = cmath.exp(1j * psi)
    for n in range(n_ticks):
        t = n * DT
        z = tilt_amp_rad * cmath.exp(1j * omega * t)    # tanθ + j·tanφ/cosθ
        pitch = math.atan(z.real)
        roll = math.atan(z.imag * math.cos(pitch))
        t_pos = t - pos_delay_ticks * DT
        u_pos = G_MPS2 * _rot(tilt_amp_rad * cmath.exp(1j * omega * t_pos))
        a_w = e_psi * u_pos
        p = -a_w / (omega ** 2)
        est.add_sample(p.real, p.imag, roll, pitch)


def _feed_turn(est: MotionYawEstimator, psi0: float, rate_rad_s: float,
               n_hold: int, n_turn: int, tilt_amp_rad: float = 0.035,
               freq_hz: float = 1.1) -> float:
    """一定ヨーレート回頭+並進励振の合成データ。最終 tick の真ヨーを返す。

    ψ(t) は PoC 規約(ψ ≈ -yaw_true)なので、機体レート r=+ρ(yaw_true
    増加方向)で dψ/dt = -ρ。位置は a_k = e^{jψ_k}·u_k を離散2重積分
    (P_k = 2P_{k-1} - P_{k-2} + a_{k-1}·dt²: 2階差分のエポック [k-1] 中心に
    厳密整合)し、位置経路遅延 POS_DELAY_N を掛けて与える。
    """
    total = n_hold + n_turn
    omega = 2.0 * math.pi * freq_hz
    theta = 0.0
    psi_seq = []
    a_seq = []
    z_seq = []
    r_seq = []
    for k in range(total):
        r_k = rate_rad_s if k >= n_hold else 0.0
        theta += r_k * DT           # 推定器の積分順序(add_sample 冒頭)と同じ
        psi_k = psi0 - theta
        z_k = tilt_amp_rad * math.sin(omega * k * DT)
        u_k = G_MPS2 * _rot(complex(z_k, 0.0))
        psi_seq.append(psi_k)
        a_seq.append(cmath.exp(1j * psi_k) * u_k)
        z_seq.append(z_k)
        r_seq.append(r_k)
    # 離散2重積分(P の2階差分[k] = a_{k-1})
    pos = [0.0 + 0.0j] * total
    for k in range(2, total):
        pos[k] = 2.0 * pos[k - 1] - pos[k - 2] + a_seq[k - 1] * (DT ** 2)
    for n in range(total):
        p = pos[max(0, n - POS_DELAY_N)]
        pitch = math.atan(z_seq[n])
        est.add_sample(p.real, p.imag, 0.0, pitch, yaw_rate_rad_s=r_seq[n])
    return psi_seq[-1]


def _feed_hover_drag(est: MotionYawEstimator, psi: float, lam: complex,
                     n_ticks: int) -> None:
    """速度比例の空力項を含むホバ様合成データ: a_W = e^{jψ}·U + λ·V。

    低速円軌道(0.25Hz — V が U と同位相に混じり、1回帰子ではヨーの
    ブロックバイアスになる成分)+高速直線往復(0.7Hz — U と V の共線性を
    崩し2回帰子 LS を可測にする成分)の2成分。U = e^{-jψ}(a - λv) から
    逆算して傾きを与えるので、モデルは厳密に成立する。
    """
    w1 = 2.0 * math.pi * 0.25
    w2 = 2.0 * math.pi * 0.7
    p1 = 0.20                       # 円軌道半径 [m]
    p2 = 0.04                       # 直線振幅 [m]
    c2 = cmath.exp(1j * 0.5)        # 直線の方向(円と独立な向き)
    e_mpsi = cmath.exp(-1j * psi)
    for n in range(n_ticks):
        t = n * DT
        t_pos = t - POS_DELAY_N * DT                    # 位置経路: 遅い側
        p = (p1 * cmath.exp(1j * w1 * t_pos)
             + c2 * p2 * math.sin(w2 * t_pos))
        v = (1j * w1 * p1 * cmath.exp(1j * w1 * t)
             + c2 * p2 * w2 * math.cos(w2 * t))
        a = (-(w1 ** 2) * p1 * cmath.exp(1j * w1 * t)
             - c2 * p2 * (w2 ** 2) * math.sin(w2 * t))
        u = e_mpsi * (a - lam * v)
        z = 1j * u / G_MPS2         # u = g·(-j)·z の逆写像
        pitch = math.atan(z.real)
        roll = math.atan(z.imag * math.cos(pitch))
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
        assert _wrap_deg(result["yaw_rad"] - psi_true) < 2.0

    def test_recovers_negative_yaw(self):
        psi_true = -2.2
        est = MotionYawEstimator(dt_s=DT)
        _feed_linear_excitation(est, psi_true, n_ticks=750)
        result = est.estimate()
        assert result["valid"], result
        assert _wrap_deg(result["yaw_rad"] - psi_true) < 2.0

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
        assert _wrap_deg(result["yaw_rad"] - 0.6) < 2.0


class TestDerotation:
    """ジャイロ derotation(ψ=const 制約の除去)。"""

    def test_tracks_yaw_during_turn(self):
        """一定レート回頭(20°/s・160°)+並進励振で現在ヨーを ±3° 追従。"""
        est = MotionYawEstimator(dt_s=DT)
        psi_end = _feed_turn(est, psi0=0.4, rate_rad_s=0.35,
                             n_hold=500, n_turn=400)
        result = est.estimate()
        assert result["valid"], result
        assert _wrap_deg(result["yaw_rad"] - psi_end) < 3.0, result

    def test_off_shows_circular_mean_lag(self):
        """derotation OFF では窓円平均のラグ(回頭終了時に大誤差)が出る。"""
        est = MotionYawEstimator({"derotate": False}, dt_s=DT)
        psi_end = _feed_turn(est, psi0=0.4, rate_rad_s=0.35,
                             n_hold=500, n_turn=400)
        result = est.estimate()
        assert result["valid"], result
        assert _wrap_deg(result["yaw_rad"] - psi_end) > 30.0, result

    def test_gap_holds_rate(self):
        """短ギャップ中は直前 r で θ をホールドし、復帰後も追従が保たれる。"""
        est = MotionYawEstimator(dt_s=DT)
        _feed_turn(est, psi0=0.4, rate_rad_s=0.35, n_hold=500, n_turn=200)
        for _ in range(25):   # 0.5s のギャップ(回頭は継続している想定)
            est.add_gap()
        # ギャップ後も同レートで回頭継続 → θ ホールドが正しければ整合が続く
        theta_before = 0.35 * (200 + 25) * DT
        est2_psi_end = 0.4 - 0.35 * (200 + 25 + 200) * DT
        omega = 2.0 * math.pi * 1.1
        total_prev = 500 + 200 + 25
        # 手動継続 feed(_feed_turn は連続系列前提のためここで簡易に生成)
        pos_hist = {}
        theta = theta_before
        a_seq = []
        z_seq = []
        for k in range(200):
            theta += 0.35 * DT
            psi_k = 0.4 - theta
            z_k = 0.035 * math.sin(omega * (total_prev + k) * DT)
            a_seq.append(cmath.exp(1j * psi_k) * G_MPS2 * _rot(complex(z_k, 0)))
            z_seq.append(z_k)
        pos = [0.0 + 0.0j] * 200
        for k in range(2, 200):
            pos[k] = 2.0 * pos[k - 1] - pos[k - 2] + a_seq[k - 1] * (DT ** 2)
        for n in range(200):
            p = pos[max(0, n - POS_DELAY_N)]
            est.add_sample(p.real, p.imag, 0.0, math.atan(z_seq[n]),
                           yaw_rate_rad_s=0.35)
        result = est.estimate()
        assert result["valid"], result
        # ギャップ挿入込みでも数度以内(θ ホールドが効いている)
        assert _wrap_deg(result["yaw_rad"] - est2_psi_end) < 6.0, result


class TestVelocityTerm:
    """速度回帰項(空力ブロックバイアスの吸収)。"""

    LAM = complex(-0.7, -0.9)   # PoC 実測 k_re≈0.33/k_im≈0.39 [1/s] の約2倍

    def test_removes_velocity_bias(self):
        """速度比例項を含むデータで velocity_term ON がバイアスを除去。"""
        psi_true = 0.9
        est = MotionYawEstimator(dt_s=DT)
        _feed_hover_drag(est, psi_true, self.LAM, n_ticks=1000)   # 20s
        result = est.estimate()
        assert result["valid"], result
        assert result["vel_term_active"], result
        assert _wrap_deg(result["yaw_rad"] - psi_true) < 2.0, result
        # λ の同定も確認(厳密モデルなのでほぼ一致するはず)
        lam_hat = complex(result["lambda_re"], result["lambda_im"])
        assert abs(lam_hat - self.LAM) < 0.25, lam_hat

    def test_off_leaves_velocity_bias(self):
        """同一データで velocity_term OFF はブロックバイアスが残る。"""
        psi_true = 0.9
        est = MotionYawEstimator({"velocity_term": False}, dt_s=DT)
        _feed_hover_drag(est, psi_true, self.LAM, n_ticks=1000)
        result = est.estimate()
        assert result["valid"], result
        assert not result["vel_term_active"]
        assert result["lambda_re"] is None
        assert _wrap_deg(result["yaw_rad"] - psi_true) > 5.0, result


class TestDelayCompensationDirection:
    """遅延補償の「方向」(2軸円運動でのみ検出可能)。

    1軸励振は時間シフトの向きに不感で、旧テストは方向誤りを検出できない
    欠陥が指摘済み。円運動では方向誤りが位相バイアス 2ω·τ ≈ 29°
    (ω=2π·0.5, τ=0.16s)として現れる。速度項は円運動で U と V が共線に
    なり条件数ガードで無効化されるため、切り分けのため明示 OFF にする。
    """

    def test_correct_direction_is_accurate(self):
        """位置経路が遅い(実機と同じ向き)合成データでは ±3° で回復。"""
        est = MotionYawEstimator({"velocity_term": False}, dt_s=DT)
        _feed_circular_excitation(est, 0.8, n_ticks=750,
                                  pos_delay_ticks=POS_DELAY_N)
        result = est.estimate()
        assert result["valid"], result
        assert _wrap_deg(result["yaw_rad"] - 0.8) < 3.0, result

    def test_reversed_direction_shows_bias(self):
        """逆向き遅延(テレメトリが遅い系)では補償が仇になり大誤差。"""
        est = MotionYawEstimator({"velocity_term": False}, dt_s=DT)
        _feed_circular_excitation(est, 0.8, n_ticks=750,
                                  pos_delay_ticks=-POS_DELAY_N)
        result = est.estimate()
        assert result["valid"], result
        assert _wrap_deg(result["yaw_rad"] - 0.8) > 15.0, result

    def test_collinear_guard_disables_velocity_term(self):
        """純円運動では U∥V の条件数ガードが速度項をフォールバックさせる。"""
        est = MotionYawEstimator(dt_s=DT)   # velocity_term 既定 ON
        _feed_circular_excitation(est, 0.8, n_ticks=750,
                                  pos_delay_ticks=POS_DELAY_N)
        result = est.estimate()
        assert result["valid"], result
        assert not result["vel_term_active"], result
        # フォールバック(1回帰子)でもヨー自体は回復できる
        assert _wrap_deg(result["yaw_rad"] - 0.8) < 3.0, result


class TestLagRounding:
    """lag_n の half-up 丸め(banker's 丸め回帰の防止)。"""

    def test_default_delay_rounds_up(self):
        """既定 0.09s·50Hz = 4.5 tick は 5 tick(100ms)に丸める。

        round() の銀行家丸めでは 4 tick(80ms)に落ちる回帰があった。
        """
        est = MotionYawEstimator(dt_s=DT)
        assert est._lag_n == 5

    def test_exact_delay_unchanged(self):
        est = MotionYawEstimator({"delay_s": 0.08}, dt_s=DT)
        assert est._lag_n == 4

    def test_zero_delay(self):
        est = MotionYawEstimator({"delay_s": 0.0}, dt_s=DT)
        assert est._lag_n == 0
