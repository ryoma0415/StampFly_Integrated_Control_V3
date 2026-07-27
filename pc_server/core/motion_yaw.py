"""移動ベースヨー推定(MAG_AUTOTUNE_DESIGN.md §3.2)。

MoCap 位置の水平加速度と機体テレメトリの roll/pitch(傾き起因の比力方向)
から、8s 滑動窓の最小二乗でヨーを逆算する:

    モデル:      a_W(t) = e^{jψ} · u_B(t) + noise
    推定:        ψ̂ = arg( Σ A_i · conj(U_i) )
    Fisher情報:  J  = Σ |U_i|²   (閾値未満なら invalid)

係数・符号規約・遅延補償は docs/analysis_20260727/scripts/
poc_yaw_from_motion.py の PoC 実証値に従う(7/27 実ログ2本で較正):
- U = g · e^{-jπ/2} · (tan(pitch) + j·tan(roll)/cos(pitch))
  (conj=False, rot=270°。roll/pitch はテレメトリのファーム規約)
- A = MoCap 位置(制御座標系 x+jy)の2階微分
- テレメトリ遅延 ≈90ms(PoC lag=4〜5行@50Hz): A[n] と U[n-lag] を対にする
- 推定 ψ̂ の符号規約は PoC の psi_sign=-1、すなわち **ψ̂ ≈ -yaw_true**
  (yaw_true = AttitudeMapper の「正解ヨー」= 機体ヨー推定と比較可能な規約)。
  ワイヤ規約への変換は session 層が行う(マッピング設定に依存するため)。

PoC の SG 平滑(非因果)の代わりに因果 2Hz biquad LPF + 2階差分を使い、
帯域整合のため位置・傾きの両系列へ**同一の** LPF を適用する(群遅延が
両辺で相殺されヨーには効かない)。トリム除去の HP は因果移動平均の減算
(PoC の hp_win_s=2.0 移動平均と同等の因果版)を両系列へ同一適用する。

50Hz 駆動・逐次更新(毎 tick の計算は O(1): biquad 2本+差分+窓和の
加減算のみ。窓の積は deque に保持し、期限切れを引き算で落とす)。
"""

from __future__ import annotations

import cmath
import math
from collections import deque
from typing import Optional

G_MPS2 = 9.80665

# 既定パラメータ(control.json "motion_yaw" 節で上書き可能)
DEFAULT_CONFIG = {
    "lpf_cutoff_hz": 2.0,   # 因果 LPF(biquad Butterworth)カットオフ
    "hp_window_s": 2.0,     # トリム除去の因果移動平均窓(PoC hp_win_s)
    "window_s": 8.0,        # 滑動窓 LS の窓長(契約 §3.2)
    "delay_s": 0.09,        # テレメトリ遅延補償(PoC 実測 80〜100ms)
    "j_min": 9.0,           # Fisher情報ゲート [(m/s²)²·サンプル]。
                            # PoC の CRLB σψ=σa√Ncorr/√J(σa≈0.05, Ncorr≈27)
                            # で ±5° に相当する J≈9
    "warmup_s": 0.5,        # ギャップ復帰後、積算を再開するまでの整定時間
}

# 窓和の浮動小数ドリフト対策: この tick 数ごとに deque から厳密再計算する
_RESYNC_TICKS = 2048


class _Biquad:
    """RBJ cookbook 2次 Butterworth LPF(実係数。複素入力にもそのまま線形)。"""

    def __init__(self, cutoff_hz: float, fs_hz: float) -> None:
        w0 = 2.0 * math.pi * cutoff_hz / fs_hz
        alpha = math.sin(w0) / (2.0 * (1.0 / math.sqrt(2.0)))
        cos_w0 = math.cos(w0)
        a0 = 1.0 + alpha
        self.b0 = ((1.0 - cos_w0) / 2.0) / a0
        self.b1 = (1.0 - cos_w0) / a0
        self.b2 = ((1.0 - cos_w0) / 2.0) / a0
        self.a1 = (-2.0 * cos_w0) / a0
        self.a2 = (1.0 - alpha) / a0
        self.reset()

    def reset(self) -> None:
        self._x1 = self._x2 = None
        self._y1 = self._y2 = None

    def step(self, x: complex) -> complex:
        """1サンプル濾波。初回はステップ過渡を避けるため定常状態でシード。"""
        if self._x1 is None:
            # 入力が過去ずっと x だった定常状態(LPF ゲイン1)で初期化
            self._x1 = self._x2 = x
            self._y1 = self._y2 = x
        y = (self.b0 * x + self.b1 * self._x1 + self.b2 * self._x2
             - self.a1 * self._y1 - self.a2 * self._y2)
        self._x2, self._x1 = self._x1, x
        self._y2, self._y1 = self._y1, y
        return y


class _CausalMean:
    """因果移動平均(直近 n サンプル)。HP = 入力 − 平均。"""

    def __init__(self, n: int) -> None:
        self._n = n
        self._buf: deque = deque()
        self._sum: complex = 0.0 + 0.0j

    def reset(self) -> None:
        self._buf.clear()
        self._sum = 0.0 + 0.0j

    def highpass(self, x: complex) -> complex:
        self._buf.append(x)
        self._sum += x
        if len(self._buf) > self._n:
            self._sum -= self._buf.popleft()
        return x - self._sum / len(self._buf)


class MotionYawEstimator:
    """8s 滑動窓 LS の移動ベースヨー推定器(50Hz 駆動、単一スレッド前提)。

    使い方(session の 50Hz 送信 tick から):
    - 有効サンプル(MoCap 有効 + テレメトリ新鮮)ごとに add_sample()
    - 無効 tick は add_gap()(窓の時刻を進め、フィルタ連鎖をリセット)
    - estimate() で {"yaw_rad", "j", "valid", "n"} を取得。yaw_rad は
      PoC 規約(ψ̂ ≈ -yaw_true)。ワイヤ規約変換は呼び出し側の責務。
    """

    def __init__(self, config: Optional[dict] = None,
                 dt_s: float = 0.02) -> None:
        cfg = dict(DEFAULT_CONFIG)
        cfg.update(config or {})
        self._dt_s = float(dt_s)
        fs = 1.0 / self._dt_s
        self._lag_n = max(0, int(round(float(cfg["delay_s"]) * fs)))
        self._window_n = max(1, int(round(float(cfg["window_s"]) * fs)))
        self._warmup_n = max(2, int(round(float(cfg["warmup_s"]) * fs)))
        self._j_min = float(cfg["j_min"])
        hp_n = max(1, int(round(float(cfg["hp_window_s"]) * fs)))

        # フィルタ連鎖(位置系列・傾き系列に同一特性の LPF/HP を適用する)
        self._lpf_pos = _Biquad(float(cfg["lpf_cutoff_hz"]), fs)
        self._lpf_tilt = _Biquad(float(cfg["lpf_cutoff_hz"]), fs)
        self._hp_a = _CausalMean(hp_n)
        self._hp_u = _CausalMean(hp_n)

        # 逐次状態
        self._tick = 0                     # gap 込みの通算 tick(窓の期限判定)
        self._since_reset = 0              # リセット後の有効サンプル数
        self._pf_prev: Optional[complex] = None    # LPF後位置 [n-1]
        self._pf_prev2: Optional[complex] = None   # LPF後位置 [n-2]
        self._u_ring: deque = deque(maxlen=self._lag_n + 1)  # 遅延補償リング

        # 滑動窓: (tick, A·conj(U), |U|²) の deque と走り和
        self._products: deque = deque()
        self._sum_s: complex = 0.0 + 0.0j
        self._sum_j: float = 0.0
        self._adds_since_resync = 0

    # ------------------------------------------------------------------

    def _reset_chain(self) -> None:
        """フィルタ連鎖と遅延リングをリセットする(窓の積は自然期限切れ)。"""
        self._lpf_pos.reset()
        self._lpf_tilt.reset()
        self._hp_a.reset()
        self._hp_u.reset()
        self._pf_prev = None
        self._pf_prev2 = None
        self._u_ring.clear()
        self._since_reset = 0

    def _expire(self) -> None:
        floor = self._tick - self._window_n
        while self._products and self._products[0][0] <= floor:
            _, prod, usq = self._products.popleft()
            self._sum_s -= prod
            self._sum_j -= usq

    def _resync_sums(self) -> None:
        """走り和を deque から厳密再計算する(浮動小数ドリフトの掃除)。"""
        s = 0.0 + 0.0j
        j = 0.0
        for _, prod, usq in self._products:
            s += prod
            j += usq
        self._sum_s = s
        self._sum_j = j
        self._adds_since_resync = 0

    # ------------------------------------------------------------------

    def add_gap(self) -> None:
        """無効 tick(MoCap 無効・テレメトリ途絶など)。窓時刻のみ進める。"""
        self._tick += 1
        self._reset_chain()
        self._expire()

    def add_sample(self, pos_x_m: float, pos_y_m: float,
                   roll_rad: float, pitch_rad: float) -> None:
        """有効 tick の1サンプル(制御座標系位置+テレメトリ roll/pitch)。"""
        self._tick += 1

        # --- 傾きベクトル(PoC 規約): U = g · e^{-jπ/2} · (tanθ + j·tanφ/cosθ)
        cos_p = math.cos(pitch_rad)
        if abs(cos_p) < 1e-6:   # 鉛直特異(実飛行では到達しない)
            self._reset_chain()
            self._expire()
            return
        z = complex(math.tan(pitch_rad), math.tan(roll_rad) / cos_p)
        u0 = G_MPS2 * (-1j) * self._lpf_tilt.step(z)

        # --- 位置 → LPF → 2階差分(因果。両系列同一 LPF で帯域整合)
        pf = self._lpf_pos.step(complex(pos_x_m, pos_y_m))
        a0: Optional[complex] = None
        if self._pf_prev is not None and self._pf_prev2 is not None:
            a0 = (pf - 2.0 * self._pf_prev + self._pf_prev2) / (self._dt_s ** 2)
        self._pf_prev2, self._pf_prev = self._pf_prev, pf

        self._since_reset += 1
        if a0 is not None:
            a_hp = self._hp_a.highpass(a0)
            self._u_ring.append(self._hp_u.highpass(u0))
            # 遅延補償: A[n] と U[n-lag] を対にする(テレメトリ遅延 ≈90ms)
            if (len(self._u_ring) > self._lag_n
                    and self._since_reset > self._warmup_n):
                u_hp = self._u_ring[0]
                prod = a_hp * u_hp.conjugate()
                usq = abs(u_hp) ** 2
                self._products.append((self._tick, prod, usq))
                self._sum_s += prod
                self._sum_j += usq
                self._adds_since_resync += 1
                if self._adds_since_resync >= _RESYNC_TICKS:
                    self._resync_sums()
        else:
            # 差分に足りない間も HP/リングは温める(帯域整合のため)
            self._u_ring.append(self._hp_u.highpass(u0))

        self._expire()

    def estimate(self) -> dict:
        """現在の窓推定。yaw_rad は PoC 規約(ψ̂ ≈ -yaw_true)、無効時 None。"""
        j = max(0.0, self._sum_j)
        valid = j >= self._j_min and len(self._products) > 0
        yaw = cmath.phase(self._sum_s) if valid else None
        return {"yaw_rad": yaw, "j": j, "valid": valid,
                "n": len(self._products)}
