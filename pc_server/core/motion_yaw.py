"""移動ベースヨー推定(MAG_AUTOTUNE_DESIGN.md §3.2)。

MoCap 位置の水平加速度と機体テレメトリの roll/pitch(傾き起因の比力方向)
から、8s 滑動窓の最小二乗でヨーを逆算する:

    モデル:      a_W(t) = e^{jψ(t)} · u_B(t) + λ · v_W(t) + noise
    derotation:  θ(t) = ∫ r dt(機体ヨーレート積分。PoC 規約では dψ/dt = -r)。
                 Ũ_i = e^{j·s·θ_i} · U_i(s = _DEROT_SIGN)と置くと
                 A_i = α·Ũ_i + λ·V_i(α = e^{jψ_ref} は窓内定数)の線形 LS
                 になり、回頭中でも窓平均ラグ(実測で窓長/2 ≈ 2〜6.7s)が
                 出ない。現在ヨーは ψ̂_now = arg(α) + s·θ_now。
    速度項:      λ(複素)は速度比例の空力項(PoC 実測 k_re≈0.33 /
                 k_im≈0.39 [1/s])。純ホバでも残る +25° 級のブロック
                 バイアスの主因を吸収する。λ は世界系スカラーなので
                 derotation では U のみ回転すれば整合する。
    Fisher情報:  J  = Σ |Ũ_i|² = Σ |U_i|²(回転はノルム不変。
                 閾値未満なら invalid)

係数・符号規約・遅延補償は docs/analysis_20260727/scripts/
poc_yaw_from_motion.py の PoC 実証値に従う(7/27 実ログ2本で較正):
- U = g · e^{-jπ/2} · (tan(pitch) + j·tan(roll)/cos(pitch))
  (conj=False, rot=270°。roll/pitch はテレメトリのファーム規約)
- A = MoCap 位置(制御座標系 x+jy)の2階微分
- 経路遅延 ≈90ms(PoC lag=4〜5行@50Hz): A[n] と U[n-lag] を対にする。
  遅い側はフィルタ済 MoCap 位置経路(= A の系列)であり、テレメトリが
  遅れているのではない(旧記述「テレメトリ遅延」は因果ラベルの誤り)。
- 推定 ψ̂ の符号規約は PoC の psi_sign=-1、すなわち **ψ̂ ≈ -yaw_true**
  (yaw_true = AttitudeMapper の「正解ヨー」= 機体ヨー推定と比較可能な規約)。
  ワイヤ規約への変換は session 層が行う(マッピング設定に依存するため)。

PoC の SG 平滑(非因果)の代わりに因果 2Hz biquad LPF + 2階差分を使い、
帯域整合のため位置・傾きの両系列へ**同一の** LPF を適用する(群遅延が
両辺で相殺されヨーには効かない)。トリム除去の HP は因果移動平均の減算
(PoC の hp_win_s=2.0 移動平均と同等の因果版)を A・U・V の3系列へ同一
適用する(同一 LTI なので線形関係 A=αU+λV は保存される)。U の HP は
回転**前**(トリムは無回転の傾き系で定数)に掛け、回転してから蓄積する。

50Hz 駆動・逐次更新(毎 tick の計算は O(1): biquad 2本+差分+窓和の
加減算のみ。窓の積は deque に保持し、期限切れを引き算で落とす)。
"""

from __future__ import annotations

import cmath
import math
from collections import deque
from typing import Optional

G_MPS2 = 9.80665

# derotation の符号 s: Ũ_i = e^{j·s·θ_i}·U_i、ψ̂_now = arg(α) + s·θ_now。
# 理論(ψ̂ ≈ -yaw_true、r は yaw_true 増加方向が正 → dψ/dt = -r)では
# s=-1。実ログ 20260731_211535(±100° 回頭ペア。tlm_r と d(yaw_true)/dt
# の回帰勾配 +1.00 で r の向きも同ログで確認)のリプレイで経験的にも確定
# (velocity_term ON、診断のため j_min=0.5。回頭区間の |誤差| mean/peak):
#   +98° 回頭(t=12.6〜20.0s): OFF 46.0°/90.6° → s=-1 9.7°/21.3°、
#                               s=+1 63.0°/179.4°
#   -104° 回頭(t=20.6〜25.8s): OFF 24.4°/60.3° → s=-1 16.4°/28.4°、
#                               s=+1 57.2°/139.2°
#   全飛行 RMS: OFF 26.1° → s=-1 12.0°、s=+1 45.7°
# 正符号 s=-1 で回頭誤差が約 1/5 に激減し、逆符号 s=+1 では OFF の
# 約2倍(位相が逆に回る)になる。
_DEROT_SIGN = -1

# 既定パラメータ(control.json "motion_yaw" 節で上書き可能)
DEFAULT_CONFIG = {
    "lpf_cutoff_hz": 2.0,   # 因果 LPF(biquad Butterworth)カットオフ
    "hp_window_s": 2.0,     # トリム除去の因果移動平均窓(PoC hp_win_s)
    "window_s": 8.0,        # 滑動窓 LS の窓長(契約 §3.2)
    "delay_s": 0.09,        # 経路遅延補償(PoC 実測 80〜100ms。遅い側は
                            # フィルタ済 MoCap 位置経路 = A の系列)
    "j_min": 9.0,           # Fisher情報ゲート [(m/s²)²·サンプル]。
                            # PoC の CRLB σψ=σa√Ncorr/√J(σa≈0.05, Ncorr≈27)
                            # で ±5° に相当する J≈9
    "warmup_s": 0.5,        # ギャップ復帰後、積算を再開するまでの整定時間
    "derotate": True,       # ジャイロ derotation(回頭中の窓平均ラグ除去)
    "velocity_term": True,  # 速度回帰項(空力ブロックバイアスの吸収)
}

# 窓和の浮動小数ドリフト対策: この tick 数ごとに deque から厳密再計算する
_RESYNC_TICKS = 2048

# 2回帰子 LS の条件数ガード: S_uu·S_vv - |S_uv|² がこの比率未満(U と V が
# ほぼ共線 — 例: 純円運動)なら1回帰子 α=S_au/S_uu へフォールバックする
_COND_GUARD = 0.02
# S_vv の絶対下限 [(m/s)²·サンプル](速度励振が実質ゼロのとき)
_SVV_MIN = 1e-9


def _wrap_pi(a: float) -> float:
    """角度を (-π, π] 相当へ折り返す。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


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
      (yaw_rate_rad_s にテレメトリの機体ヨーレート r を渡す —
      25Hz のサンプルホールドで良い)
    - 無効 tick は add_gap()(窓の時刻を進め、フィルタ連鎖をリセット。
      θ 積分は直前 r でホールド — 長断は warmup+窓期限切れが守る)
    - estimate() で {"yaw_rad", "j", "valid", "n"} を取得。yaw_rad は
      PoC 規約(ψ̂ ≈ -yaw_true)。ワイヤ規約変換は呼び出し側の責務。
      診断キー lambda_re/lambda_im(速度項係数)・vel_term_active も返す。
    """

    def __init__(self, config: Optional[dict] = None,
                 dt_s: float = 0.02) -> None:
        cfg = dict(DEFAULT_CONFIG)
        cfg.update(config or {})
        self._dt_s = float(dt_s)
        fs = 1.0 / self._dt_s
        # 四捨五入は half-up(int(x+0.5))。round() は銀行家丸めのため
        # 0.09s·50Hz = 4.5 が 4 tick(80ms)に落ちる回帰があった。
        self._lag_n = max(0, int(float(cfg["delay_s"]) * fs + 0.5))
        self._window_n = max(1, int(round(float(cfg["window_s"]) * fs)))
        self._warmup_n = max(2, int(round(float(cfg["warmup_s"]) * fs)))
        self._j_min = float(cfg["j_min"])
        self._derotate = bool(cfg["derotate"])
        self._vel_term = bool(cfg["velocity_term"])
        self._derot_sign = _DEROT_SIGN
        hp_n = max(1, int(round(float(cfg["hp_window_s"]) * fs)))

        # フィルタ連鎖(位置系列・傾き系列に同一特性の LPF/HP を適用する)
        self._lpf_pos = _Biquad(float(cfg["lpf_cutoff_hz"]), fs)
        self._lpf_tilt = _Biquad(float(cfg["lpf_cutoff_hz"]), fs)
        self._hp_a = _CausalMean(hp_n)
        self._hp_u = _CausalMean(hp_n)
        self._hp_v = _CausalMean(hp_n)

        # 逐次状態
        self._tick = 0                     # gap 込みの通算 tick(窓の期限判定)
        self._since_reset = 0              # リセット後の有効サンプル数
        self._pf_prev: Optional[complex] = None    # LPF後位置 [n-1]
        self._pf_prev2: Optional[complex] = None   # LPF後位置 [n-2]
        self._u_ring: deque = deque(maxlen=self._lag_n + 1)  # 遅延補償リング
        self._theta = 0.0                  # ∫r dt(wrap 保持。derotation 用)
        self._r_last = 0.0                 # gap 中のホールド用の直前 r

        # 滑動窓: (tick, A·conj(Ũ), |Ũ|², A·conj(V), Ũ·conj(V), |V|²) の
        # deque と走り和(2×2 複素正規方程式の増分和)
        self._products: deque = deque()
        self._sum_au: complex = 0.0 + 0.0j   # S_au = Σ A·conj(Ũ)
        self._sum_j: float = 0.0             # S_uu = Σ |Ũ|²(= Fisher情報 J)
        self._sum_av: complex = 0.0 + 0.0j   # S_av = Σ A·conj(V)
        self._sum_uv: complex = 0.0 + 0.0j   # S_uv = Σ Ũ·conj(V)
        self._sum_vv: float = 0.0            # S_vv = Σ |V|²
        self._adds_since_resync = 0

    # ------------------------------------------------------------------

    def _advance_theta(self, yaw_rate_rad_s: float) -> None:
        """θ = ∫r dt を1 tick 進める(wrap 保持で無限成長の精度劣化を防ぐ)。"""
        self._theta = _wrap_pi(self._theta
                               + float(yaw_rate_rad_s) * self._dt_s)

    def _reset_chain(self) -> None:
        """フィルタ連鎖と遅延リングをリセットする(窓の積は自然期限切れ)。"""
        self._lpf_pos.reset()
        self._lpf_tilt.reset()
        self._hp_a.reset()
        self._hp_u.reset()
        self._hp_v.reset()
        self._pf_prev = None
        self._pf_prev2 = None
        self._u_ring.clear()
        self._since_reset = 0

    def _expire(self) -> None:
        floor = self._tick - self._window_n
        while self._products and self._products[0][0] <= floor:
            _, p_au, usq, p_av, p_uv, vsq = self._products.popleft()
            self._sum_au -= p_au
            self._sum_j -= usq
            self._sum_av -= p_av
            self._sum_uv -= p_uv
            self._sum_vv -= vsq

    def _resync_sums(self) -> None:
        """走り和を deque から厳密再計算する(浮動小数ドリフトの掃除)。"""
        s_au = 0.0 + 0.0j
        s_j = 0.0
        s_av = 0.0 + 0.0j
        s_uv = 0.0 + 0.0j
        s_vv = 0.0
        for _, p_au, usq, p_av, p_uv, vsq in self._products:
            s_au += p_au
            s_j += usq
            s_av += p_av
            s_uv += p_uv
            s_vv += vsq
        self._sum_au = s_au
        self._sum_j = s_j
        self._sum_av = s_av
        self._sum_uv = s_uv
        self._sum_vv = s_vv
        self._adds_since_resync = 0

    # ------------------------------------------------------------------

    def add_gap(self) -> None:
        """無効 tick(MoCap 無効・テレメトリ途絶など)。窓時刻のみ進める。

        θ 積分は直前 r でホールドする(短断での derotation 連続性維持。
        長断は warmup+窓期限切れが誤用を防ぐ)。
        """
        self._tick += 1
        self._advance_theta(self._r_last)
        self._reset_chain()
        self._expire()

    def add_sample(self, pos_x_m: float, pos_y_m: float,
                   roll_rad: float, pitch_rad: float,
                   yaw_rate_rad_s: float = 0.0) -> None:
        """有効 tick の1サンプル(制御座標系位置+テレメトリ roll/pitch/r)。"""
        self._tick += 1
        self._advance_theta(yaw_rate_rad_s)
        self._r_last = float(yaw_rate_rad_s)

        # --- 傾きベクトル(PoC 規約): U = g · e^{-jπ/2} · (tanθ + j·tanφ/cosθ)
        cos_p = math.cos(pitch_rad)
        if abs(cos_p) < 1e-6:   # 鉛直特異(実飛行では到達しない)
            self._reset_chain()
            self._expire()
            return
        z = complex(math.tan(pitch_rad), math.tan(roll_rad) / cos_p)
        u0 = G_MPS2 * (-1j) * self._lpf_tilt.step(z)

        # --- 位置 → LPF → 差分(因果。両系列同一 LPF で帯域整合)
        # A は2階差分、V は中心1階差分 — どちらもエポック [n-1] 中心で整合
        pf = self._lpf_pos.step(complex(pos_x_m, pos_y_m))
        a0: Optional[complex] = None
        v0: Optional[complex] = None
        if self._pf_prev is not None and self._pf_prev2 is not None:
            a0 = (pf - 2.0 * self._pf_prev + self._pf_prev2) / (self._dt_s ** 2)
            v0 = (pf - self._pf_prev2) / (2.0 * self._dt_s)
        self._pf_prev2, self._pf_prev = self._pf_prev, pf

        self._since_reset += 1
        # HP は回転前(トリムは無回転傾き系の定数)、derotation 回転は
        # 挿入時に掛ける: Ũ = e^{j·s·θ}·HP(U)
        u_hp = self._hp_u.highpass(u0)
        if self._derotate:
            u_hp = cmath.exp(1j * (self._derot_sign * self._theta)) * u_hp
        self._u_ring.append(u_hp)
        if a0 is not None:
            a_hp = self._hp_a.highpass(a0)
            v_hp = self._hp_v.highpass(v0)
            # 遅延補償: A[n] と U[n-lag] を対にする(フィルタ済 MoCap 位置
            # 経路 = A 側が遅い ≈90ms)。V は A と同一経路なので lag なし。
            if (len(self._u_ring) > self._lag_n
                    and self._since_reset > self._warmup_n):
                u_d = self._u_ring[0]
                p_au = a_hp * u_d.conjugate()
                usq = abs(u_d) ** 2
                p_av = a_hp * v_hp.conjugate()
                p_uv = u_d * v_hp.conjugate()
                vsq = abs(v_hp) ** 2
                self._products.append(
                    (self._tick, p_au, usq, p_av, p_uv, vsq))
                self._sum_au += p_au
                self._sum_j += usq
                self._sum_av += p_av
                self._sum_uv += p_uv
                self._sum_vv += vsq
                self._adds_since_resync += 1
                if self._adds_since_resync >= _RESYNC_TICKS:
                    self._resync_sums()

        self._expire()

    def estimate(self) -> dict:
        """現在の窓推定。yaw_rad は PoC 規約(ψ̂ ≈ -yaw_true)、無効時 None。

        velocity_term 有効時は 2×2 複素正規方程式
            [S_uu  conj(S_uv)] [α]   [S_au]
            [S_uv  S_vv      ] [λ] = [S_av]
        を解いて ψ̂ = arg(α)。U と V がほぼ共線(条件数ガード)なら
        1回帰子 α = S_au/S_uu へフォールバックする。derotation 有効時は
        s·θ_now を加えて現在ヨーへ戻す。
        """
        j = max(0.0, self._sum_j)
        valid = j >= self._j_min and len(self._products) > 0
        yaw: Optional[float] = None
        lam: Optional[complex] = None
        vel_active = False
        if valid:
            alpha: Optional[complex] = None
            if self._vel_term:
                s_vv = max(0.0, self._sum_vv)
                det = j * s_vv - abs(self._sum_uv) ** 2
                if s_vv > _SVV_MIN and det > _COND_GUARD * j * s_vv:
                    alpha = (self._sum_au * s_vv
                             - self._sum_uv.conjugate() * self._sum_av) / det
                    lam = (j * self._sum_av
                           - self._sum_uv * self._sum_au) / det
                    vel_active = True
            if alpha is None:
                alpha = self._sum_au / j   # j >= j_min > 0 は valid で保証
            yaw = cmath.phase(alpha)
            if self._derotate:
                yaw = _wrap_pi(yaw + self._derot_sign * self._theta)
        return {"yaw_rad": yaw, "j": j, "valid": valid,
                "n": len(self._products),
                "lambda_re": (lam.real if lam is not None else None),
                "lambda_im": (lam.imag if lam is not None else None),
                "vel_term_active": vel_active}
