"""Position モード: mocap → フィルタ → 位置誤差 → CMD_POS_ERR 50Hz 送信。

データフロー(機上XY制御。XY PID は機体側 flight_control が実行する):
- NatNet コールバック(on_mocap_pose, NatNetスレッド)で PositionFilter →
  有効性判定 → 最新の位置誤差(目標 − フィルタ済み位置)をキャッシュする。
- 50Hz 送信スレッドは誤差と整形済み alt/yaw を meta に載せて emit する
  (session 層が CMD_POS_ERR に組み立てて送信・ログする)。roll/pitch の
  角度指令は機体側 XY PID が計算するため、この層では生成・整形しない。
- v2 軌道モード: hover(固定目標)/ circle(円軌道)/ shuttle(直線往復)/
  sequence(評価シーケンス: hover/circle/shuttle/yaw セグメントの定型メニューを
  1操作で流すスクリプト軌道。各セグメントは既存の生成ロジックへ委譲する)。
  軌道中は 50Hz 送信
  ループ内で目標 (x, y) を時間更新する(旧 OptiTrack版 NatNet_PID_Controller の
  circling_controller.py の軌道生成を参考。当該フォルダは削除済み)。
  開始時は現在位置から円周
  最近傍点(shuttle は軸への射影点)に位相を合わせ、滑らかに合流する。
- v2 ヨー指令: UI のヨー角スライダ(±180°)+「進行方向を向く」オプション
  (円軌道中かつヨー角制御 ON のとき yaw_ref を接線方向に追従)。
- MoCap の yaw_rad はログ列 mocap_yaw_deg として meta に載せる。CMD_POS_ERR の
  mocap_yaw 欄(外部ヨー基準)は連続性フィルタ後の yaw_true_rad
  (meta["mocap_yaw_true_rad"])を session 層がパネル較正準拠で整形して送る
  (2026-07-31 改修: heading×wire_sign の旧経路は較正がワイヤに乗らず廃止)。
  heading_rad は診断列 mocap_heading_deg 用の生値として残る。

フェイルセーフ(PROTOCOL.md):
- MoCap 途絶 > mocap_dropout_level_s(300ms)→ data_valid を落として送信を
  継続(CMD_POS_ERR flags bit2=0 → 機体側が水平指令+PID減衰。alt_ref は維持)。
  >2s の CMD_STOP は session 層の監視が行う。
- データ無効の持続(受信はあるが data_valid=0 が続く: トラッキング喪失・
  外れ値・低信頼度)→ 同じく bit2=0。警告・自動 CMD_STOP は session 層の
  監視(data_invalid_warn_s / data_invalid_stop_s)が行う。
  基準は data_invalid_age_s。

単位は core 内部規約(rad / m)。座標は制御座標系(mocap.py で変換済み)。
"""

from __future__ import annotations

import threading
import time
from math import asin, atan2, ceil, cos, hypot, pi, sin
from typing import Callable, Optional

from .filter import PositionFilter
from .mocap import DEG_TO_RAD, RAD_TO_DEG
from .posture import (
    SENDER_JOIN_TIMEOUT_S, TWO_PI, SetpointShaper, run_paced_loop, wrap_pi,
)

MS_PER_S = 1000.0

# 軌道モード(ログ列 traj_mode の値。LOG_STRUCTURE v2 契約)
TRAJ_MODE_HOVER = 0
TRAJ_MODE_CIRCLE = 1
TRAJ_MODE_SHUTTLE = 2

# 評価シーケンスのセグメント遷移トランジット: 次セグメントの合流点が現在
# 目標からこの距離を超えて離れている場合、等速直線で目標を運ぶ小フェーズを
# 自動挿入する(レビュー指摘: 幾何によっては遷移で 0.25〜0.35m 級の目標
# 段差が出て、EKF2 評価飛行に計画外のステップ応答が混入した)。
# トランジット中の meta traj_mode は 0(hover)、凍結機構はシーケンス
# 時計と共通。
_SEQ_TRANSIT_EPS_M = 0.05
_SEQ_TRANSIT_SPEED_MPS = 0.25


class PositionController:
    """OptiTrack 位置フィードバックの誤差計算+軌道+50Hz 送信コントローラ。

    emit(roll_rad, pitch_rad, alt_m, meta) は session 層が供給し、
    CMD_POS_ERR の組み立て・送信・CSV ログを担う(XY PID は機体側)。
    roll/pitch 引数は互換のため残るが常に 0 を渡す。
    """

    MODE_NAME = "position"

    def __init__(self, server_config: dict, control_config: dict,
                 emit: Callable[[float, float, float, dict], None],
                 clock: Callable[[], float] = time.monotonic) -> None:
        self._period_s = 1.0 / server_config["rates"]["setpoint_hz"]
        self._dropout_level_s: float = server_config["failsafe"]["mocap_dropout_level_s"]
        self._mocap_fresh_s: float = server_config["freshness"]["mocap_fresh_s"]
        # 接線ヨー追従の実現可能性判定に使う(start_circle のガード)
        self._yaw_slew_rad_per_s: float = (
            server_config["clamps"]["yaw_slew_rate_deg_per_s"] * DEG_TO_RAD)
        # yaw セグメントの目標角ガード(SetpointShaper.shape_yaw と同じ制限)
        self._max_yaw_rad: float = (
            server_config["clamps"]["max_yaw_deg"] * DEG_TO_RAD)
        self._shaper = SetpointShaper(server_config["clamps"])
        self._emit = emit
        self._clock = clock

        self.position_filter = PositionFilter.from_config(control_config["filter"])
        self._confidence_zero_threshold: float = (
            control_config["control"]["confidence_zero_threshold"])
        self._frame_hold_s: float = control_config["control"]["frame_hold_ms"] / MS_PER_S

        target_default = control_config["target_default"]
        traj_cfg = control_config["trajectory"]
        self._traj_radius_min: float = traj_cfg["radius_min_m"]
        self._traj_radius_max: float = traj_cfg["radius_max_m"]
        self._traj_period_min: float = traj_cfg["period_min_s"]
        self._traj_period_max: float = traj_cfg["period_max_s"]
        self._traj_center_abs_max: float = traj_cfg["center_abs_max_m"]
        # シャトル(直線往復)のガード
        self._shuttle_amp_min: float = traj_cfg["shuttle_amplitude_min_m"]
        self._shuttle_amp_max: float = traj_cfg["shuttle_amplitude_max_m"]
        self._traj_speed_max: float = traj_cfg["speed_max_mps"]
        self._traj_excursion_abs_max: float = traj_cfg["excursion_abs_max_m"]
        self._lock = threading.Lock()
        # position_filter は NatNet スレッド・50Hz 送信スレッド・
        # UI(executor)/supervisor スレッドから触られる共有状態のため、
        # 専用ロックで保護する(規約: スレッド共有状態は lock で保護)。
        self._filter_lock = threading.Lock()
        self._target = (target_default["x"], target_default["y"], target_default["z"])

        # NatNet コールバックが更新する最新状態
        self._last_pose: Optional[dict] = None         # mocap.py の pose dict
        self._last_pose_t: Optional[float] = None
        self._last_frame_dt: Optional[float] = None
        self._last_filter_result: Optional[dict] = None
        self._last_errors = (0.0, 0.0)
        self._last_data_valid = False
        # データ無効が連続し始めた時刻(持続的データ無効の監視基準。
        # 有効フレームで None に戻る。START を経ない flying 昇格でも
        # 無効フレーム到着時点から計時が始まる)
        self._invalid_since: Optional[float] = None
        # フィルタ世代: reset_filter / reset_control のたびに +1。
        # NatNet スレッドが旧世代フィルタで計算した結果をリセット後に
        # 書き戻す競合を検出して捨てるために使う
        self._filter_generation = 0
        # MoCap マッピング世代フロア: これ未満の pose["mapping_gen"] を
        # 持つフレームは破棄する。マッピング差し替え(set_mapping)直前に
        # 旧マッピングで計算中だったフレームが、リセット直後のフィルタを
        # 旧座標系でシードする競合(逆向きの世代競合)の防止
        self._mapping_gen_floor = 0
        # 制御座標系の方位角 → 機体ヨー規約への符号(接線ヨー用)。
        # CMD_POS_ERR の yaw_ref は機体の ψ 規約(レガシーフレームの方位角と
        # 一致)で送る契約。UI スライダ由来のヨー目標はもともと機体規約だが、
        # 円軌道の接線ヨーは制御座標系の方位角として計算されるため、
        # 右手系マッピング(machine_wire_y_sign=−1)では符号反転が要る。
        # session 層がマッピング適用時に設定する(既定 +1 = レガシー互換)
        self._yaw_azimuth_wire_sign = 1.0

        # 直近の送信値(バイアス加算前)
        self._last_output = (0.0, 0.0, self._target[2])

        # v2: ヨー指令(UI スライダ由来、rad)と制御トグル
        self._target_yaw = 0.0
        self._yaw_ctrl_on = False
        self._last_yaw_output = 0.0

        # v2: 軌道状態(None = hover)。共通キー:
        # kind("circle"/"shuttle"/"sequence") / alt / t0(開始時刻) /
        # frozen_at(MoCap 途絶による位相凍結の開始時刻。None = 凍結なし)。
        # circle/shuttle 共通: center(x,y) / period_s /
        # phase0(合流点の位相 rad)。
        # circle 固有: radius / clockwise / face_tangent。
        # shuttle 固有: axis_deg / axis_e(軸単位ベクトル) / amplitude /
        # cycles / phase_stop(自動停止位相。None = 連続) /
        # phase_abs(直近の非ラップ位相。残りサイクル計算用)。
        # sequence 固有: name / segments(正規化済み定義) / est_s(見積り秒)
        # / start_index / seg_index / seg_state(現セグメントのランタイム状態。
        # None = 未入場) / seg_elapsed_base(現セグメント開始時点の
        # シーケンス経過秒)。シーケンス時計は t0/frozen_at で途絶凍結される
        self._traj: Optional[dict] = None
        self._traj_phase: Optional[float] = None

        # XY 閉ループの有効フラグ(legacy の control_active に相当)。
        # Start 受理後のみ session 層が True にする。False の間もフィルタは
        # 回し続けるが、CMD_POS_ERR の bit2(XY_ERR_VALID)は立たない
        # (機体側は水平指令+PID減衰)。
        self._control_active = False

        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # UI からの入力(座標は制御座標系の m)
    # ------------------------------------------------------------------

    def set_target(self, x: float, y: float, z: float) -> None:
        with self._lock:
            self._target = (x, y, z)

    def get_target(self) -> tuple[float, float, float]:
        with self._lock:
            return self._target

    def current_setpoint(self) -> tuple[float, float, float]:
        """直近に送信した整形済みセットポイント(バイアス加算前)。"""
        with self._lock:
            return self._last_output

    def set_yaw_setpoint(self, yaw_rad: float) -> None:
        """UI ヨー角スライダ(session 層で deg→rad 変換済み)。"""
        with self._lock:
            self._target_yaw = yaw_rad

    def set_yaw_control(self, enabled: bool) -> None:
        with self._lock:
            self._yaw_ctrl_on = bool(enabled)

    def yaw_setpoint(self) -> tuple[float, bool]:
        """直近に送信した整形済みヨー目標と制御 ON/OFF(スナップショット用)。"""
        with self._lock:
            return (self._last_yaw_output, self._yaw_ctrl_on)

    # ------------------------------------------------------------------
    # v2: 軌道モード(circle / shuttle)
    # ------------------------------------------------------------------

    def _merge_position(self, now: float,
                        traj_name: str) -> tuple[Optional[tuple[float, float]],
                                                 Optional[str]]:
        """軌道開始時の合流基準位置(フィルタ済み優先)を返す。

        鮮度・有効性・取得可否のガードを共通適用する(circle/shuttle 同一
        基準)。戻り値は ((px, py), None) または (None, 日本語エラー)。
        """
        with self._lock:
            filter_result = self._last_filter_result
            pose = self._last_pose
            pose_t = self._last_pose_t
            data_valid = self._last_data_valid
        # 鮮度ガード(session.start と同じ基準値): 途絶中の古い位置から
        # 合流位相を決めると「現在位置から滑らかに合流」(契約 §3.3)が
        # 成立せず、復帰時に目標が実位置から離れた点へジャンプするため拒否。
        age = None if pose_t is None else (now - pose_t)
        if age is None or age > self._dropout_level_s:
            return None, f"MoCap データが新鮮でないため{traj_name}を開始できません"
        # 有効性ガード(session.start と同じ): 無効中のフィルタ位置は
        # 凍結/外挿されたゴーストのため、そこから合流位相を決めない
        if not data_valid:
            return None, (f"MoCap 位置データが無効のため{traj_name}を開始"
                          "できません(トラッキング状態を確認してください)")
        if filter_result is not None:
            px, py, _ = filter_result["filtered_position"]
        elif pose is not None:
            px, py = pose["x"], pose["y"]
        else:
            return None, f"MoCap 位置が取得できないため{traj_name}を開始できません"
        return (px, py), None

    # --- パラメータ検証(start_circle/start_shuttle と評価シーケンスが共用) ---

    def _circle_params_error(self, center_x: float, center_y: float,
                             radius_m: float, period_s: float) -> Optional[str]:
        """円軌道パラメータの範囲検証。不合格なら日本語メッセージを返す。"""
        if not (self._traj_radius_min <= radius_m <= self._traj_radius_max):
            return (f"半径は {self._traj_radius_min}–"
                    f"{self._traj_radius_max} m で指定してください")
        if not (self._traj_period_min <= period_s <= self._traj_period_max):
            return (f"周期は {self._traj_period_min}–"
                    f"{self._traj_period_max} s で指定してください")
        # 閉区間形式(not (lo <= v <= hi))は NaN も拒否する(abs 比較は
        # NaN を素通しし、NaN 目標 → クランプ飽和誤差の送信に至るため)
        max_c = self._traj_center_abs_max
        if not (-max_c <= center_x <= max_c and -max_c <= center_y <= max_c):
            return (f"中心座標は ±{max_c} m 以内で"
                    "指定してください")
        return None

    def _shuttle_params_error(self, center_x: float, center_y: float,
                              axis_deg: float, amplitude_m: float,
                              period_s: float) -> Optional[str]:
        """シャトル軌道パラメータの範囲検証(cycles は呼び出し元が検証)。"""
        if not (self._shuttle_amp_min <= amplitude_m <= self._shuttle_amp_max):
            return (f"振幅は {self._shuttle_amp_min}–"
                    f"{self._shuttle_amp_max} m で指定してください")
        if not (self._traj_period_min <= period_s <= self._traj_period_max):
            return (f"周期は {self._traj_period_min}–"
                    f"{self._traj_period_max} s で指定してください")
        # 最大速度ガード: v_max = A·ω = 2πA/T(正弦往復の中点速度)
        v_max = TWO_PI * amplitude_m / period_s
        if v_max > self._traj_speed_max:
            return (f"最大速度 2πA/T = {v_max:.2f} m/s が上限 "
                    f"{self._traj_speed_max} m/s を超えます。"
                    "振幅を小さくするか周期を長くしてください")
        # 可動域ガード: 両端点 center ± A·e が正方形範囲に収まること
        # (中心は端点の中点なので、このガードが中心範囲も内包する)
        # 閉区間形式は NaN(center/axis_deg 経由の伝播含む)も拒否する
        max_e = self._traj_excursion_abs_max
        theta = axis_deg * DEG_TO_RAD
        ex, ey = cos(theta), sin(theta)
        for sgn in (1.0, -1.0):
            end_x = center_x + sgn * amplitude_m * ex
            end_y = center_y + sgn * amplitude_m * ey
            if not (-max_e <= end_x <= max_e and -max_e <= end_y <= max_e):
                return (f"往復の端点が ±{max_e} m"
                        " の範囲を超えます。中心・振幅・軸方位を"
                        "見直してください")
        return None

    def _alt_error(self, alt_m: float) -> Optional[str]:
        alt_lo, alt_hi = self._shaper.alt_limits
        if not (alt_lo <= alt_m <= alt_hi):
            return f"高度は {alt_lo}–{alt_hi} m で指定してください"
        return None

    def start_circle(self, center_x: float, center_y: float, radius_m: float,
                     period_s: float, clockwise: bool, alt_m: float,
                     face_tangent: bool,
                     now: Optional[float] = None) -> tuple[bool, Optional[str]]:
        """円軌道を開始する。現在位置から円周最近傍点に位相を合わせる。

        戻り値は (ok, error)。error は UI 表示用の日本語メッセージ。
        パラメータ検証は control.json の trajectory 節の制限に従う。
        """
        if now is None:
            now = self._clock()
        error = self._circle_params_error(center_x, center_y, radius_m,
                                          period_s)
        if error is not None:
            return False, error
        if face_tangent:
            # 接線ヨー目標は 360°/period_s で回転する。整形スルーレート
            # (yaw_slew_rate_deg_per_s)を超える周期を許すと、整形済み
            # yaw_ref の遅れが 180° を超えた瞬間に wrap で符号反転し、
            # 鋸歯状の逆回転・発振が起きるため開始を拒否する。
            min_period_s = TWO_PI / self._yaw_slew_rad_per_s
            if period_s < min_period_s:
                return False, (
                    "「進行方向を向く」有効時は、接線ヨーの角速度(360°/周期)が"
                    "ヨースルーレート上限を超えないよう周期を "
                    f"{min_period_s:.1f} s 以上にしてください")

        error = self._alt_error(alt_m)
        if error is not None:
            return False, error
        # 現在位置(フィルタ済み優先)→ 円周最近傍点の位相。位置が未取得・
        # 途絶・無効の場合は開始を拒否する(合流点を決められないため)。
        merge, error = self._merge_position(now, "円軌道")
        if merge is None:
            return False, error
        px, py = merge

        dx, dy = px - center_x, py - center_y
        # 機体が中心に一致している縮退ケースは位相 0 から開始する
        phase0 = atan2(dy, dx) if (dx != 0.0 or dy != 0.0) else 0.0
        with self._lock:
            self._traj = {
                "kind": "circle",
                "center": (center_x, center_y),
                "radius": radius_m,
                "period_s": period_s,
                "clockwise": bool(clockwise),
                "alt": alt_m,
                "face_tangent": bool(face_tangent),
                "phase0": phase0,
                "t0": now,
                # MoCap 途絶による位相凍結の開始時刻(None = 凍結なし)
                "frozen_at": None,
            }
            self._traj_phase = wrap_pi(phase0)
        return True, None

    def start_shuttle(self, center_x: float, center_y: float, axis_deg: float,
                      amplitude_m: float, period_s: float, cycles: int,
                      alt_m: float,
                      now: Optional[float] = None) -> tuple[bool, Optional[str]]:
        """直線往復(シャトル)軌道を開始する。

        目標は target = center + A·sin(phase)·e(θ)、e=(cosθ, sinθ)。
        現在位置を軸へ射影した点に位相を合わせて滑らかに合流する
        (円の最近傍位相合流と同思想)。cycles=0 は連続(手動 stop)、
        >0 は経過位相が cycles·2π に達した後、次の極値(速度ゼロ点)で
        自動停止し、その端点をホールドして hover に復帰する。
        戻り値は (ok, error)。error は UI 表示用の日本語メッセージ。
        """
        if now is None:
            now = self._clock()
        error = self._shuttle_params_error(center_x, center_y, axis_deg,
                                           amplitude_m, period_s)
        if error is not None:
            return False, error
        if cycles < 0:
            return False, "サイクル数は 0 以上で指定してください(0 = 連続)"
        error = self._alt_error(alt_m)
        if error is not None:
            return False, error
        merge, error = self._merge_position(now, "往復軌道")
        if merge is None:
            return False, error
        px, py = merge
        theta = axis_deg * DEG_TO_RAD
        ex, ey = cos(theta), sin(theta)

        # 合流: 現在位置を軸へ射影 s=clamp((p−center)·e, −A, A) →
        # phase0 = asin(s/A) ∈ [−π/2, π/2]。開始時の目標が射影点と一致し、
        # 現在位置から滑らかに合流する
        s = (px - center_x) * ex + (py - center_y) * ey
        s = max(-amplitude_m, min(amplitude_m, s))
        phase0 = asin(s / amplitude_m)
        phase_stop: Optional[float] = None
        if cycles > 0:
            phase_stop = self._shuttle_stop_phase(phase0, cycles)
        with self._lock:
            self._traj = {
                "kind": "shuttle",
                "center": (center_x, center_y),
                "axis_deg": axis_deg,
                "axis_e": (ex, ey),
                "amplitude": amplitude_m,
                "period_s": period_s,
                "cycles": int(cycles),
                "alt": alt_m,
                "phase0": phase0,
                "phase_stop": phase_stop,
                "phase_abs": phase0,
                "t0": now,
                # MoCap 途絶による位相凍結の開始時刻(None = 凍結なし)
                "frozen_at": None,
            }
            self._traj_phase = wrap_pi(phase0)
        return True, None

    @staticmethod
    def _shuttle_stop_phase(phase0: float, cycles: int) -> float:
        """経過位相 cycles·2π 以降で最初の極値(phase ≡ π/2 mod π、速度ゼロ点)。

        基点が極値ちょうどなら同点で止まる(start_shuttle と評価シーケンスの
        shuttle セグメントが共用)。
        """
        base = phase0 + cycles * TWO_PI
        k = ceil((base - pi / 2.0) / pi - 1e-9)
        return pi / 2.0 + k * pi

    # ------------------------------------------------------------------
    # v2: 評価シーケンス(スクリプト軌道。hover/circle/shuttle/yaw セグメント)
    # ------------------------------------------------------------------

    def _yaw_segment_error(self, seg: dict) -> Optional[str]:
        """yaw セグメント(その場回頭)の検証。不合格なら日本語メッセージ。"""
        targets = seg.get("targets_deg")
        if not isinstance(targets, (list, tuple)) or not targets:
            return "targets_deg にヨー目標角のリストを指定してください"
        # 数値検査は閉区間形式(not (lo <= v <= hi))で NaN も拒否する
        max_yaw_deg = self._max_yaw_rad * RAD_TO_DEG
        for value in targets:
            if (not isinstance(value, (int, float))
                    or not (-max_yaw_deg - 1e-9
                            <= float(value) <= max_yaw_deg + 1e-9)):
                return (f"ヨー目標角は ±{max_yaw_deg:.0f}° 以内の数値で"
                        "指定してください")
        # ランプレートは SetpointShaper のヨースルーレート以下でなければ
        # 整形が追従できず、指令ランプが実効レートと乖離する
        rate = seg.get("rate_dps")
        slew_dps = self._yaw_slew_rad_per_s * RAD_TO_DEG
        if (not isinstance(rate, (int, float))
                or not (0.0 < float(rate) <= slew_dps + 1e-9)):
            return (f"ヨーレートは 0 より大きく {slew_dps:.0f}°/s"
                    "(ヨースルーレート上限)以下で指定してください")
        hold_s = seg.get("hold_s", 0.0)
        if not isinstance(hold_s, (int, float)) or not (float(hold_s) >= 0.0):
            return "ホールド時間は 0 以上の秒数で指定してください"
        return None

    def _segment_error(self, seg: dict) -> Optional[str]:
        """1 セグメント定義の検証(既存 circle/shuttle ガードへ委譲)。"""
        seg_type = seg.get("type")
        if seg_type == "hover":
            duration = seg.get("duration_s")
            if (not isinstance(duration, (int, float))
                    or not (float(duration) > 0.0)):
                return "duration_s は正の秒数で指定してください"
            return None
        if seg_type == "circle":
            laps = seg.get("laps")
            if not isinstance(laps, int) or laps < 1:
                return "周回数(laps)は 1 以上の整数で指定してください"
            cx = float(seg.get("center_x", 0.0))
            cy = float(seg.get("center_y", 0.0))
            radius = float(seg.get("radius_m", 0.0))
            period = float(seg.get("period_s", 0.0))
            error = self._circle_params_error(cx, cy, radius, period)
            if error is not None:
                return error
            # シーケンスの想定エンベロープは shuttle と同じ速度・可動域
            # 制限を circle にも課す(単発 start_circle の制限値は既存
            # 挙動のまま。レビュー指摘: circle 経由で ±0.5m/0.5m/s を
            # 迂回できた)。閉区間形式は NaN も拒否する
            v_max = TWO_PI * radius / period if period > 0.0 else float("inf")
            if not (v_max <= self._traj_speed_max):
                return (f"周速 2πR/T = {v_max:.2f} m/s が上限 "
                        f"{self._traj_speed_max} m/s を超えます。"
                        "半径を小さくするか周期を長くしてください")
            max_e = self._traj_excursion_abs_max
            if not (abs(cx) + radius <= max_e and abs(cy) + radius <= max_e):
                return (f"円の到達範囲(|中心|+半径)が ±{max_e} m を"
                        "超えます。中心・半径を見直してください")
            return None
        if seg_type == "shuttle":
            cycles = seg.get("cycles")
            if not isinstance(cycles, int) or cycles < 1:
                return ("サイクル数は 1 以上の整数で指定してください"
                        "(シーケンスでは連続往復は使えません)")
            return self._shuttle_params_error(
                float(seg.get("center_x", 0.0)), float(seg.get("center_y", 0.0)),
                float(seg.get("axis_deg", 0.0)),
                float(seg.get("amplitude_m", 0.0)),
                float(seg.get("period_s", 0.0)))
        if seg_type == "yaw":
            return self._yaw_segment_error(seg)
        return f"不明なセグメント型です: {seg_type!r}"

    @staticmethod
    def _segment_estimate_s(seg: dict) -> float:
        """セグメント所要時間の静的見積り(UI 表示・残り時間表示用)。

        実所要は合流位相・開始ヨーに依存して前後する: shuttle は合流位相 0
        (中心合流)を仮定して端点停止までの +T/4 を上乗せし、yaw は開始
        ヨー 0 を仮定する。
        """
        seg_type = seg.get("type")
        if seg_type == "hover":
            return float(seg["duration_s"])
        if seg_type == "circle":
            return float(seg["laps"]) * float(seg["period_s"])
        if seg_type == "shuttle":
            return (float(seg["cycles"]) + 0.25) * float(seg["period_s"])
        # yaw: ランプ(最短経路)+ホールドの合計
        prev = 0.0
        total = 0.0
        rate = float(seg["rate_dps"]) * DEG_TO_RAD
        hold_s = float(seg.get("hold_s", 0.0))
        for tgt_deg in seg["targets_deg"]:
            tgt = wrap_pi(float(tgt_deg) * DEG_TO_RAD)
            total += abs(wrap_pi(tgt - prev)) / rate + hold_s
            prev = tgt
        return total

    def _sequence_transit_estimates(self, segments: list, start_index: int,
                                    start_xy: tuple[float, float]) -> list:
        """各セグメント入場前トランジットの静的見積り秒(残り時間表示用)。

        開始位置からの位置連鎖で決定的に求まる: hover/yaw は現在点を保持、
        circle は合流点(最近傍円周点)で入場し laps·2π 後に同点へ戻る、
        shuttle は軸射影点で入場し停止極値の端点で終わる
        (_shuttle_stop_phase と同じ規則)。start_index より前は 0。
        """
        px, py = start_xy
        result = [0.0] * len(segments)
        for i in range(start_index, len(segments)):
            seg = segments[i]
            seg_type = seg.get("type")
            if seg_type == "circle":
                cx = float(seg.get("center_x", 0.0))
                cy = float(seg.get("center_y", 0.0))
                radius = float(seg["radius_m"])
                dx, dy = px - cx, py - cy
                norm = hypot(dx, dy)
                if norm > 0.0:
                    mx, my = cx + radius * dx / norm, cy + radius * dy / norm
                else:
                    mx, my = cx + radius, cy   # 縮退: 位相 0 の円周点
                dist = hypot(mx - px, my - py)
                if dist > _SEQ_TRANSIT_EPS_M:
                    result[i] = dist / _SEQ_TRANSIT_SPEED_MPS
                px, py = mx, my               # laps·2π 後は合流点へ戻る
            elif seg_type == "shuttle":
                cx = float(seg.get("center_x", 0.0))
                cy = float(seg.get("center_y", 0.0))
                amplitude = float(seg["amplitude_m"])
                theta = float(seg["axis_deg"]) * DEG_TO_RAD
                ex, ey = cos(theta), sin(theta)
                s = max(-amplitude, min(amplitude,
                                        (px - cx) * ex + (py - cy) * ey))
                mx, my = cx + s * ex, cy + s * ey
                dist = hypot(mx - px, my - py)
                if dist > _SEQ_TRANSIT_EPS_M:
                    result[i] = dist / _SEQ_TRANSIT_SPEED_MPS
                phase_stop = self._shuttle_stop_phase(asin(s / amplitude),
                                                      int(seg["cycles"]))
                s_end = amplitude * sin(phase_stop)
                px, py = cx + s_end * ex, cy + s_end * ey
            # hover/yaw: 現在点を保持(トランジット不要)
        return result

    def start_sequence(self, name: str, segments: list, alt_m: float,
                       start_index: int = 0,
                       now: Optional[float] = None) -> tuple[bool, Optional[str]]:
        """評価シーケンス(スクリプト軌道)を開始する。

        全セグメントを既存ガードで事前検証し、不合格ならどのセグメントが
        不合格かを日本語で返す。開始時は現在位置へ目標をスナップし、
        先頭セグメント(start_index)はそこから既存の合流ロジック
        (circle: 円周最近傍点 / shuttle: 軸への射影点)で滑らかに合流する。
        yaw セグメントを含む実行区間はヨー角制御 ON が前提(OFF だと
        ファームが yaw_ref を消費せず、盲目的なランプになるため開始拒否)。
        戻り値は (ok, error)。
        """
        if now is None:
            now = self._clock()
        if not isinstance(segments, list) or not segments:
            return False, "セグメントが定義されていません"
        if not (0 <= int(start_index) < len(segments)):
            return False, (f"開始セグメントは 1–{len(segments)} の範囲で"
                           "指定してください")
        start_index = int(start_index)
        normalized: list[dict] = []
        for i, seg in enumerate(segments):
            if not isinstance(seg, dict):
                return False, f"セグメント{i + 1}: 定義が不正です"
            error = self._segment_error(seg)
            if error is not None:
                return False, (f"セグメント{i + 1}"
                               f"({seg.get('type', '?')}): {error}")
            normalized.append(dict(seg))
        error = self._alt_error(alt_m)
        if error is not None:
            return False, error
        with self._lock:
            yaw_ctrl_on = self._yaw_ctrl_on
        if (any(seg.get("type") == "yaw"
                for seg in normalized[start_index:]) and not yaw_ctrl_on):
            return False, ("yaw セグメントを含むシーケンスはヨー角制御 ON の"
                           "ときのみ開始できます")
        merge, error = self._merge_position(now, "評価シーケンス")
        if merge is None:
            return False, error
        px, py = merge
        est_s = [self._segment_estimate_s(seg) for seg in normalized]
        # 残り時間表示用: セグメント間トランジットの静的見積り(開始位置
        # からの位置連鎖で決定的に求まる)
        transit_s = self._sequence_transit_estimates(normalized, start_index,
                                                     (px, py))
        with self._lock:
            # 開始位置へ目標をスナップし、先頭セグメントはここから合流する
            # (hover は現在位置ホールド、circle/shuttle は既存合流則)
            self._target = (px, py, alt_m)
            self._traj = {
                "kind": "sequence",
                "name": str(name),
                "alt": alt_m,
                "segments": normalized,
                "est_s": est_s,
                "transit_s": transit_s,
                "start_index": start_index,
                "seg_index": start_index,
                "seg_state": None,        # step() が現在目標/ヨーから遅延入場
                "seg_elapsed_base": 0.0,
                "t0": now,
                # MoCap 途絶によるシーケンス時計凍結の開始時刻(None = 凍結なし)
                "frozen_at": None,
            }
            self._traj_phase = None
        return True, None

    def _segment_enter_locked(self, seg: dict, alt_m: float) -> dict:
        """セグメント入場: 現在目標・現在ヨーから合流するランタイム状態を作る。

        呼び出し元が self._lock を保持していること。合流則は単発の
        start_circle/start_shuttle と同じ(circle: 目標の方位角に位相合わせ、
        shuttle: 目標の軸射影に位相合わせ)だが、基準は「現在目標」
        (直前セグメントの終端)を使い、遷移を決定的にする。
        """
        tx, ty, _ = self._target
        seg_type = seg["type"]
        if seg_type == "hover":
            return {"type": "hover", "duration": float(seg["duration_s"]),
                    "hold": (tx, ty), "alt": alt_m}
        if seg_type == "circle":
            cx = float(seg.get("center_x", 0.0))
            cy = float(seg.get("center_y", 0.0))
            period_s = float(seg["period_s"])
            dx, dy = tx - cx, ty - cy
            # 目標が中心に一致している縮退ケースは位相 0 から開始する
            phase0 = atan2(dy, dx) if (dx != 0.0 or dy != 0.0) else 0.0
            return {"type": "circle",
                    "duration": float(seg["laps"]) * period_s,
                    "center": (cx, cy), "radius": float(seg["radius_m"]),
                    "omega": TWO_PI / period_s,
                    "sign": -1.0 if seg.get("clockwise") else 1.0,
                    "phase0": phase0, "alt": alt_m}
        if seg_type == "shuttle":
            cx = float(seg.get("center_x", 0.0))
            cy = float(seg.get("center_y", 0.0))
            amplitude = float(seg["amplitude_m"])
            omega = TWO_PI / float(seg["period_s"])
            theta = float(seg["axis_deg"]) * DEG_TO_RAD
            ex, ey = cos(theta), sin(theta)
            s = (tx - cx) * ex + (ty - cy) * ey
            s = max(-amplitude, min(amplitude, s))
            phase0 = asin(s / amplitude)
            phase_stop = self._shuttle_stop_phase(phase0, int(seg["cycles"]))
            return {"type": "shuttle",
                    "duration": (phase_stop - phase0) / omega,
                    "center": (cx, cy), "axis_e": (ex, ey),
                    "amplitude": amplitude, "omega": omega,
                    "phase0": phase0, "phase_stop": phase_stop, "alt": alt_m}
        # yaw: 現在の整形済みヨー出力からランプ(最短経路)+ホールドの
        # 区間列を作る。位置は現在目標をホールドする
        legs: list[dict] = []
        prev = self._last_yaw_output
        rate = float(seg["rate_dps"]) * DEG_TO_RAD
        hold_s = float(seg.get("hold_s", 0.0))
        duration = 0.0
        for tgt_deg in seg["targets_deg"]:
            tgt = wrap_pi(float(tgt_deg) * DEG_TO_RAD)
            delta = wrap_pi(tgt - prev)
            ramp_s = abs(delta) / rate
            legs.append({"y0": prev, "y1": tgt, "delta": delta,
                         "ramp_s": ramp_s, "hold_s": hold_s})
            duration += ramp_s + hold_s
            prev = tgt
        return {"type": "yaw", "duration": duration, "hold": (tx, ty),
                "legs": legs, "alt": alt_m}

    @staticmethod
    def _segment_entry_point(state: dict) -> Optional[tuple[float, float]]:
        """セグメント入場時(tau=0)の目標点。hover/yaw は現在目標保持のため
        None(トランジット不要)。"""
        if state["type"] == "circle":
            cx, cy = state["center"]
            return (cx + state["radius"] * cos(state["phase0"]),
                    cy + state["radius"] * sin(state["phase0"]))
        if state["type"] == "shuttle":
            cx, cy = state["center"]
            ex, ey = state["axis_e"]
            s = state["amplitude"] * sin(state["phase0"])
            return (cx + s * ex, cy + s * ey)
        return None

    def _segment_enter_with_transit_locked(self, seg: dict,
                                           alt_m: float) -> dict:
        """セグメント入場(必要ならトランジット小フェーズを前置する)。

        呼び出し元が self._lock を保持していること。合流点(circle: phase0
        の円周点 / shuttle: 軸射影点)が現在目標から _SEQ_TRANSIT_EPS_M を
        超えて離れている場合、_SEQ_TRANSIT_SPEED_MPS の等速直線で目標を
        合流点まで運ぶ transit 状態を挿入する。合流位相は現在目標(トラン
        ジット開始前)基準で先に確定しているため、トランジット終端 =
        セグメント入場点で幾何が厳密に接続する。
        """
        state = self._segment_enter_locked(seg, alt_m)
        entry = self._segment_entry_point(state)
        if entry is None:
            return state
        tx, ty, _ = self._target
        dist = hypot(entry[0] - tx, entry[1] - ty)
        if dist <= _SEQ_TRANSIT_EPS_M:
            return state
        return {"type": "transit", "from": (tx, ty), "to": entry,
                "duration": dist / _SEQ_TRANSIT_SPEED_MPS,
                "alt": alt_m, "next": state}

    def _segment_update_locked(self, state: dict,
                               tau: float) -> tuple[int, Optional[float]]:
        """セグメント経過 tau 秒時点の目標を反映し (traj_mode, 位相) を返す。

        呼び出し元が self._lock を保持していること。生成式は単発軌道の
        step() 内実装と同一(circle: center+R·e^{jφ} / shuttle:
        center+A·sin(φ)·e)。yaw はランプ+ホールドの区分線形で
        _target_yaw を自動操作する(基礎モードは hover 扱い)。
        transit は from→to の等速直線補間(基礎モードは hover 扱い)。
        """
        alt = state["alt"]
        if state["type"] == "transit":
            fx, fy = state["from"]
            gx, gy = state["to"]
            frac = (min(1.0, tau / state["duration"])
                    if state["duration"] > 0.0 else 1.0)
            self._target = (fx + (gx - fx) * frac, fy + (gy - fy) * frac, alt)
            return TRAJ_MODE_HOVER, None
        if state["type"] == "circle":
            phase = state["phase0"] + state["sign"] * state["omega"] * tau
            cx, cy = state["center"]
            radius = state["radius"]
            self._target = (cx + radius * cos(phase),
                            cy + radius * sin(phase), alt)
            return TRAJ_MODE_CIRCLE, wrap_pi(phase)
        if state["type"] == "shuttle":
            phase = min(state["phase0"] + state["omega"] * tau,
                        state["phase_stop"])
            cx, cy = state["center"]
            ex, ey = state["axis_e"]
            s = state["amplitude"] * sin(phase)
            self._target = (cx + s * ex, cy + s * ey, alt)
            return TRAJ_MODE_SHUTTLE, wrap_pi(phase)
        if state["type"] == "yaw":
            tau_leg = tau
            yaw = state["legs"][-1]["y1"] if state["legs"] else None
            for leg in state["legs"]:
                leg_total = leg["ramp_s"] + leg["hold_s"]
                if tau_leg < leg_total:
                    if tau_leg < leg["ramp_s"]:
                        frac = tau_leg / leg["ramp_s"]
                        yaw = wrap_pi(leg["y0"] + leg["delta"] * frac)
                    else:
                        yaw = leg["y1"]
                    break
                tau_leg -= leg_total
            if yaw is not None:
                self._target_yaw = yaw
        hx, hy = state["hold"]
        self._target = (hx, hy, alt)
        return TRAJ_MODE_HOVER, None

    def _segment_finish_locked(self, state: dict) -> None:
        """セグメント終端の厳密な状態を確定する(次セグメントの合流基準)。

        呼び出し元が self._lock を保持していること。circle は laps·2π 後
        = 入場点、shuttle は端点(速度ゼロ点)、yaw は最終目標角、
        transit は合流点(=次状態の入場点)。
        """
        alt = state["alt"]
        if state["type"] == "transit":
            gx, gy = state["to"]
            self._target = (gx, gy, alt)
            return
        if state["type"] == "circle":
            phase = (state["phase0"]
                     + state["sign"] * state["omega"] * state["duration"])
            cx, cy = state["center"]
            radius = state["radius"]
            self._target = (cx + radius * cos(phase),
                            cy + radius * sin(phase), alt)
            return
        if state["type"] == "shuttle":
            cx, cy = state["center"]
            ex, ey = state["axis_e"]
            s = state["amplitude"] * sin(state["phase_stop"])
            self._target = (cx + s * ex, cy + s * ey, alt)
            return
        if state["type"] == "yaw" and state["legs"]:
            self._target_yaw = state["legs"][-1]["y1"]
        hx, hy = state["hold"]
        self._target = (hx, hy, alt)

    def _sequence_step_locked(self, traj: dict,
                              t_eff: float) -> tuple[int, Optional[float]]:
        """シーケンス時計 t_eff でのセグメント進行+目標更新。

        呼び出し元が self._lock を保持していること。消化済みセグメントは
        厳密な終端状態で確定してから次セグメントへ入場する(遷移の目標
        連続性は各セグメントの合流則が担保)。全セグメント消化で軌道を
        外し、最終目標でのホバ復帰になる。戻り値は (traj_mode, 位相)。
        """
        elapsed = t_eff - traj["t0"]
        while True:
            segments = traj["segments"]
            index = traj["seg_index"]
            if index >= len(segments):
                # 完了: 最終目標でホバ復帰(stop_trajectory と同じ形)
                self._traj = None
                self._traj_phase = None
                return TRAJ_MODE_HOVER, None
            state = traj["seg_state"]
            if state is None:
                state = self._segment_enter_with_transit_locked(
                    segments[index], traj["alt"])
                traj["seg_state"] = state
            tau = elapsed - traj["seg_elapsed_base"]
            if tau >= state["duration"]:
                self._segment_finish_locked(state)
                traj["seg_elapsed_base"] += state["duration"]
                if state["type"] == "transit":
                    # トランジット完了 → 同一セグメントの本体へ入場
                    # (合流位相はトランジット開始前の目標基準で確定済み)
                    traj["seg_state"] = state["next"]
                else:
                    traj["seg_index"] = index + 1
                    traj["seg_state"] = None
                continue
            break
        traj_mode, traj_phase = self._segment_update_locked(state, tau)
        self._traj_phase = traj_phase
        return traj_mode, traj_phase

    def stop_trajectory(self) -> None:
        """軌道(circle/shuttle/sequence)を停止し、現在の軌道目標でホバに
        復帰する。

        目標 (self._target) は step() が軌道値で毎周期更新しているため、
        軌道を外すだけで「現在目標へのホバ復帰」になる。
        """
        with self._lock:
            self._traj = None
            self._traj_phase = None

    def stop_circle(self) -> None:
        """円軌道の停止(既存 API 互換。実体は stop_trajectory)。"""
        self.stop_trajectory()

    def trajectory_snapshot(self) -> dict:
        """WebSocket 配信用の軌道状態(UI 単位系)。

        50Hz step スレッドが _traj の可変キー(位相・セグメント状態・凍結
        時刻など)を self._lock 下で更新するため、読み側もロックを保持した
        まま組み立てる(規約: スレッド共有状態は lock で保護。以前は参照
        取得のみロック内で、可変キーの読みが競合し得た)。
        """
        with self._lock:
            traj = self._traj
            phase = self._traj_phase
            if traj is None:
                return {"mode": "hover"}
            if traj.get("kind") == "sequence":
                # シーケンス進行(名前・セグメント番号・残り秒)。残りは現
                # セグメントのランタイム所要+以降セグメントの静的見積り
                # (トランジット分込み — 2026-08-03)。
                # 途絶凍結中はシーケンス時計(frozen_at)基準で残りも凍結する
                now = self._clock()
                t_eff = (traj["frozen_at"] if traj["frozen_at"] is not None
                         else now)
                elapsed = max(0.0, t_eff - traj["t0"])
                segments = traj["segments"]
                count = len(segments)
                index = min(traj["seg_index"], count)
                state = traj["seg_state"]
                transit_s = traj.get("transit_s") or [0.0] * count
                in_transit = state is not None and state["type"] == "transit"
                if index >= count:
                    seg_type = None
                    seg_remaining = 0.0
                else:
                    seg_type = segments[index].get("type")
                    if state is not None:
                        seg_remaining = max(
                            0.0, state["duration"]
                            - (elapsed - traj["seg_elapsed_base"]))
                        if in_transit:
                            # トランジット中はセグメント本体の所要も残りに
                            # 数える(表示が一瞬 0 近くへ落ちるのを防ぐ)
                            seg_remaining += state["next"]["duration"]
                    else:
                        # 未入場(開始直後の step 前)は見積りで代用
                        # (入場前トランジットの見積り込み)
                        seg_remaining = transit_s[index] + traj["est_s"][index]
                remaining = seg_remaining + sum(
                    est + tr for est, tr in zip(traj["est_s"][index + 1:],
                                                transit_s[index + 1:]))
                return {
                    "mode": "sequence",
                    "name": traj["name"],
                    "seg_index": index,
                    "seg_count": count,
                    "seg_type": seg_type,
                    "transit": in_transit,
                    "start_index": traj["start_index"],
                    "alt_m": traj["alt"],
                    "elapsed_s": elapsed,
                    "seg_remaining_s": seg_remaining,
                    "remaining_s": remaining,
                    "phase_rad": phase,
                    # yaw セグメントが自動操作するヨー目標。UI がシーケンス
                    # 実行中のスライダ表示同期に使う(操作イベントは発火
                    # させない — 停止後のスライダ乖離ジャンプ対策)
                    "target_yaw_rad": self._target_yaw,
                }
            if traj.get("kind") == "shuttle":
                cycles = traj["cycles"]
                if cycles > 0:
                    elapsed = (traj["phase_abs"] - traj["phase0"]) / TWO_PI
                    cycles_remaining = max(0.0, cycles - elapsed)
                else:
                    cycles_remaining = None   # 連続(手動 stop)
                return {
                    "mode": "shuttle",
                    "center_x": traj["center"][0],
                    "center_y": traj["center"][1],
                    "axis_deg": traj["axis_deg"],
                    "amplitude_m": traj["amplitude"],
                    "period_s": traj["period_s"],
                    "cycles": cycles,
                    "alt_m": traj["alt"],
                    "phase_rad": phase,
                    "cycles_remaining": cycles_remaining,
                }
            return {
                "mode": "circle",
                "center_x": traj["center"][0],
                "center_y": traj["center"][1],
                "radius_m": traj["radius"],
                "period_s": traj["period_s"],
                "clockwise": traj["clockwise"],
                "alt_m": traj["alt"],
                "face_tangent": traj["face_tangent"],
                "phase_rad": phase,
            }

    def set_control_active(self, active: bool) -> None:
        """XY 閉ループの有効/無効を切り替える(CMD_POS_ERR bit2 に反映)。"""
        with self._lock:
            self._control_active = active

    @property
    def control_active(self) -> bool:
        with self._lock:
            return self._control_active

    # ------------------------------------------------------------------
    # NatNet コールバック(NatNetスレッド上: ブロッキング・print 禁止)
    # ------------------------------------------------------------------

    def on_mocap_pose(self, pose: dict) -> None:
        """新規 mocap フレームでフィルタを更新し、位置誤差をキャッシュする。"""
        t = pose["t_mono"]
        position = (pose["x"], pose["y"], pose["z"])
        mapping_gen = pose.get("mapping_gen")

        with self._lock:
            # マッピング差し替え直前に旧マッピングで計算されたフレームは
            # 破棄する(旧座標系の位置がリセット直後のフィルタのアンカーに
            # なるのを防ぐ。set_mocap_mapping_floor 参照)
            if mapping_gen is not None and mapping_gen < self._mapping_gen_floor:
                return
            prev_t = self._last_pose_t
            frame_dt = None if prev_t is None else (t - prev_t)
            self._last_pose = pose
            self._last_pose_t = t
            self._last_frame_dt = frame_dt
            target_x, target_y, _ = self._target

        # フィルタは NatNet スレッドと reset_control(UI/executor)が共有する
        with self._filter_lock:
            generation = self._filter_generation
            filter_result = self.position_filter.process_position(
                position,
                marker_count=pose["marker_count"],
                current_time=t,
                tracking_valid=pose["tracking_valid"],
                quality_weight=pose["quality"],
                rigid_body_error=pose["error"],
                source="rigid_body",
            )

        confidence = filter_result["confidence"]
        is_data_valid = (
            confidence >= self._confidence_zero_threshold
            and not filter_result["is_outlier"]
            and filter_result["tracking_valid"]
            # 再シード直後の検疫中は追従はするが閉ループには使わない
            and not filter_result.get("probation", False)
        )
        # フレーム間隔が保持時間を超えた場合も無効扱い(legacy と同じ)
        if frame_dt is not None and frame_dt > self._frame_hold_s:
            is_data_valid = False

        fx, fy, _ = filter_result["filtered_position"]
        error_x = target_x - fx
        error_y = target_y - fy

        with self._lock:
            if generation != self._filter_generation:
                # 処理中に reset_filter/reset_control が走った。旧世代
                # フィルタの結果でリセット後のキャッシュを汚さない
                return
            self._last_filter_result = filter_result
            self._last_errors = (error_x, error_y)
            self._last_data_valid = is_data_valid
            if is_data_valid:
                self._invalid_since = None
            elif self._invalid_since is None:
                self._invalid_since = t

    # ------------------------------------------------------------------
    # 50Hz 送信
    # ------------------------------------------------------------------

    def start(self) -> None:
        if self._thread is not None:
            if self._thread.is_alive():
                # 動作中、または stop() で止めきれなかった旧スレッドが残存。
                # SetpointShaper は単一所有者前提のため再起動を拒否する。
                return
            self._thread = None
        self.reset_control()
        # 停止イベントはスレッドごとに新規生成し、ループへ明示的に渡す。
        # join がタイムアウトした旧ループが後から復帰しても、自分専用の
        # (set 済み)イベントを見て必ず終了するため、二重送信は起きない。
        self._stop_event = threading.Event()
        self._thread = threading.Thread(
            target=self._sender_loop, args=(self._stop_event,),
            name="position-sender", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=SENDER_JOIN_TIMEOUT_S)
            if thread.is_alive():
                # ループがブロックしたまま(例: シリアル write 停滞)。
                # 参照を保持して start() の再起動を拒否し、ブロック解除後に
                # 旧ループが新ループと並走する事態を構造的に防ぐ。
                return
        self._thread = None

    @property
    def running(self) -> bool:
        return self._thread is not None and self._thread.is_alive()

    def reset_control(self) -> None:
        """フィルタ・整形状態をリセットする(セッション開始時)。"""
        with self._filter_lock:
            self.position_filter.reset()
            self._filter_generation += 1
        self._shaper.reset()
        with self._lock:
            self._last_pose = None
            self._last_pose_t = None
            self._last_frame_dt = None
            self._last_filter_result = None
            self._last_errors = (0.0, 0.0)
            self._last_data_valid = False
            self._invalid_since = None
            self._control_active = False
            self._traj = None
            self._traj_phase = None
            self._last_yaw_output = 0.0

    def set_yaw_azimuth_wire_sign(self, sign: float) -> None:
        """制御座標系方位角→機体ヨー規約の符号を設定する(接線ヨー用)。

        session 層がマッピング適用時に MocapSource.machine_wire_y_sign から
        設定する(+1 = レガシー/−1 = 右手系。未対応マッピング時は +1 の
        ままでよい — その場合 XY 制御自体が無効化されるため)。
        """
        with self._lock:
            self._yaw_azimuth_wire_sign = 1.0 if float(sign) >= 0 else -1.0

    def set_mocap_mapping_floor(self, generation: int) -> None:
        """MoCap マッピング世代フロアを設定する(マッピング差し替え時)。

        session.update_mocap_mapping が MocapSource.set_mapping の直後・
        reset_filter の前に呼ぶ: 差し替え前に NatNet スレッドが旧マッピングで
        計算し始めていたフレーム(pose["mapping_gen"] < generation)は
        on_mocap_pose で破棄され、リセット済みフィルタが旧座標系の位置で
        シードされることがなくなる。
        """
        with self._lock:
            self._mapping_gen_floor = int(generation)

    def reset_filter(self) -> None:
        """位置フィルタのみ再初期化する(離陸 CMD_START 受理時)。

        フィルタ状態は接続・モード変更をまたがずセッション中ずっと生き続ける
        ため、前の飛行で外れ値ロックアウトに陥ったまま次の飛行へ持ち越される
        (2026-07 の位置固定化障害)。飛行単位で仕切り直す。ポーズキャッシュ
        (_last_pose/_last_pose_t)は消さない(START 直後の途絶誤検知を防ぐ)。
        世代カウンタで NatNet スレッドの書き戻し競合(旧世代フィルタの結果で
        リセット後のキャッシュが汚れる)を防ぐ。
        """
        with self._filter_lock:
            self.position_filter.reset()
            self._filter_generation += 1
        with self._lock:
            self._last_filter_result = None
            self._last_errors = (0.0, 0.0)
            self._last_data_valid = False
            self._invalid_since = None

    def _sender_loop(self, stop_event: threading.Event) -> None:
        run_paced_loop(stop_event, self._clock, self._period_s, self.step)

    def step(self, now: float) -> None:
        """1周期ぶんの軌道更新+途絶判定+整形+送信(テストから直接呼べる)。"""
        # --- 軌道更新(circle/shuttle/sequence 中は目標を時間更新する) ---
        traj_phase: Optional[float] = None
        traj_mode = TRAJ_MODE_HOVER
        yaw_tangent: Optional[float] = None
        with self._lock:
            pose_t = self._last_pose_t
            age = None if pose_t is None else (now - pose_t)
            dropped = age is None or age > self._dropout_level_s
            traj = self._traj
            if traj is not None:
                # MoCap 途絶中・データ無効中は軌道位相の時間更新を凍結する
                # (位置フィードバックを失ったまま目標と接線ヨーだけが進み、
                # 復帰時に XY 誤差が跳ねる・盲目的な回頭指令を出すのを防ぐ。
                # 無効中は on_mocap_pose が指令を水平固定しており、目標だけ
                # 周回させる意味がない)。
                # 復帰時は t0 を凍結時間ぶん前送りして位相を連続に保つ。
                # このロジックは circle/shuttle/sequence 共通(シーケンスは
                # 時計 t0 ごと凍結され、セグメント進行・yaw ランプも止まる)。
                freeze = dropped or not self._last_data_valid
                if not freeze and traj.get("kind") == "sequence":
                    # yaw セグメント実行中にヨー角制御が OFF の間も凍結する:
                    # OFF だと step() が yaw_ref を送らずファームはランプを
                    # 見ない(盲目ランプ)ため、ON 復帰時に整形ヨーが進んだ
                    # ランプ位置へ追従して計画外の回頭が起きる(レビュー
                    # 指摘)。ON 復帰で t0 前送り再開(連続)
                    segs = traj["segments"]
                    idx = traj["seg_index"]
                    if (idx < len(segs) and segs[idx].get("type") == "yaw"
                            and not self._yaw_ctrl_on):
                        freeze = True
                if freeze:
                    if traj["frozen_at"] is None:
                        traj["frozen_at"] = now
                    t_eff = traj["frozen_at"]
                else:
                    if traj["frozen_at"] is not None:
                        traj["t0"] += now - traj["frozen_at"]
                        traj["frozen_at"] = None
                    t_eff = now
                # kind 欠落は circle 扱い(既存テストの直接構築 dict 互換)
                if traj.get("kind") == "sequence":
                    # 評価シーケンス: 現セグメントを hover/circle/shuttle/yaw
                    # の生成ロジックへ委譲する(時計はシーケンス共通 t_eff)
                    traj_mode, traj_phase = self._sequence_step_locked(
                        traj, t_eff)
                elif traj.get("kind") == "shuttle":
                    omega = TWO_PI / traj["period_s"]
                    # 直線往復: target = center + A·sin(phase)·e。位相は常に増加
                    phase = traj["phase0"] + omega * (t_eff - traj["t0"])
                    stop_phase = traj["phase_stop"]
                    stopping = stop_phase is not None and phase >= stop_phase
                    if stopping:
                        # 指定サイクル消化後の最初の極値(速度ゼロ点)で停止。
                        # 位相を極値に丸めて端点を正確にホールドする
                        phase = stop_phase
                    traj["phase_abs"] = phase
                    cx, cy = traj["center"]
                    ex, ey = traj["axis_e"]
                    s = traj["amplitude"] * sin(phase)
                    self._target = (cx + s * ex, cy + s * ey, traj["alt"])
                    if stopping:
                        # 自動停止: 端点目標のまま hover に復帰する
                        self._traj = None
                        self._traj_phase = None
                    else:
                        traj_phase = wrap_pi(phase)
                        self._traj_phase = traj_phase
                        traj_mode = TRAJ_MODE_SHUTTLE
                else:
                    omega = TWO_PI / traj["period_s"]
                    # 回転方向: CCW = 位相増加(制御座標系の数学正方向)、
                    # CW = 減少
                    sign = -1.0 if traj["clockwise"] else 1.0
                    phase = traj["phase0"] + sign * omega * (t_eff - traj["t0"])
                    cx, cy = traj["center"]
                    radius = traj["radius"]
                    self._target = (cx + radius * cos(phase),
                                    cy + radius * sin(phase),
                                    traj["alt"])
                    traj_phase = wrap_pi(phase)
                    self._traj_phase = traj_phase
                    traj_mode = TRAJ_MODE_CIRCLE
                    if traj["face_tangent"]:
                        # 接線方向(速度ベクトルの向き)= 位相 + 回転方向×90°。
                        # これは制御座標系の方位角なので、機体ヨー規約へ
                        # _yaw_azimuth_wire_sign で変換して指令にする
                        # (レガシーフレームでは +1 = 従来と同一)
                        yaw_tangent = wrap_pi(
                            self._yaw_azimuth_wire_sign
                            * (phase + sign * (pi / 2.0)))
            error_x, error_y = self._last_errors
            data_valid = self._last_data_valid
            filter_result = self._last_filter_result
            pose = self._last_pose
            frame_dt = self._last_frame_dt
            target = self._target
            control_active = self._control_active
            yaw_target = self._target_yaw
            yaw_ctrl_on = self._yaw_ctrl_on

        if dropped:
            # MoCap 途絶 >300ms: data_valid を落とす(CMD_POS_ERR bit2=0 →
            # 機体側が水平指令+PID減衰。alt_ref は維持)
            data_valid = False

        # roll/pitch の角度指令は機体側 XY PID が計算するため整形しない
        # (定数 0 を通し、alt のクランプ+スルーレート制限のみ適用する)
        _, _, alt = self._shaper.shape(0.0, 0.0, target[2], now)
        # ヨー: 「進行方向を向く」ON かつヨー角制御 ON のときのみ接線追従、
        # それ以外は UI スライダ目標。MoCap 途絶中は軌道位相が凍結される
        # ため接線ヨー目標も直近値で止まり、整形済みヨーを保持したまま
        # 送り続ける(CMD_POS_ERR 自体の途絶時は機体側が推定ヨーを
        # ラッチする契約)。
        if yaw_tangent is not None and yaw_ctrl_on:
            yaw_target = yaw_tangent
        yaw = self._shaper.shape_yaw(yaw_target, now)
        with self._lock:
            self._last_output = (0.0, 0.0, alt)
            self._last_yaw_output = yaw

        meta = {
            "mode": self.MODE_NAME,
            "data_valid": data_valid,
            "control_active": control_active,
            "mocap_dropout": dropped,
            "mocap_age_ms": None if age is None else age * MS_PER_S,
            "error_x": error_x,
            "error_y": error_y,
            "target_x": target[0],
            "target_y": target[1],
            "target_z": target[2],
            "frame_dt_ms": None if frame_dt is None else frame_dt * MS_PER_S,
            "yaw_ref_rad": yaw,
            "yaw_ctrl_on": yaw_ctrl_on,
            "traj_mode": traj_mode,
            "traj_phase_rad": traj_phase,
        }
        meta["data_source"] = "rigid_body" if pose is not None else "none"
        if pose is not None:
            meta["frame_number"] = pose["frame_number"]
            meta["marker_count"] = pose["marker_count"]
            meta["rb_error"] = pose["error"]
            meta["tracking_valid"] = pose["tracking_valid"]
            meta["raw_pos"] = (pose["x"], pose["y"], pose["z"])
            meta["mocap_yaw_deg"] = pose["yaw_rad"] * RAD_TO_DEG
            # 制御座標系ヨー(機上XY制御 CMD_POS_ERR の mocap_yaw 欄と
            # フレーム整合検証ログ mocap_heading_deg に使う)
            heading = pose.get("heading_rad")
            if heading is not None:
                meta["mocap_heading_rad"] = heading
                meta["mocap_heading_deg"] = heading * RAD_TO_DEG
            # 正解ヨー(符号/オフセット補正+連続性フィルタ後)+生クォータ
            # ニオン(ログ列 mocap_yaw_true_deg / mocap_flip / mocap_q*)。
            # rad 値は CMD_POS_ERR のヨー基準ワイヤ経路(session._emit_pos_err)
            # が使う — 表示列と同一値(単一情報源)
            yaw_true = pose.get("yaw_true_rad")
            if yaw_true is not None:
                meta["mocap_yaw_true_deg"] = yaw_true * RAD_TO_DEG
                meta["mocap_yaw_true_rad"] = yaw_true
            flip_flags = pose.get("flip_flags")
            if flip_flags is not None:
                meta["mocap_flip"] = flip_flags
            realign = pose.get("yaw_cont_realign")
            if realign is not None:
                meta["mocap_yaw_realign"] = realign
            quat = pose.get("quat")
            if quat is not None:
                (meta["mocap_qx"], meta["mocap_qy"],
                 meta["mocap_qz"], meta["mocap_qw"]) = quat
        if filter_result is not None:
            meta["filtered_pos"] = tuple(filter_result["filtered_position"])
            meta["is_outlier"] = filter_result["is_outlier"]
            meta["used_prediction"] = filter_result["used_prediction"]
            meta["confidence"] = filter_result["confidence"]
            meta["consecutive_outliers"] = filter_result["consecutive_outliers"]
            meta["filter_threshold"] = filter_result["threshold"]
        self._emit(0.0, 0.0, alt, meta)

    # ------------------------------------------------------------------
    # session 層向けスナップショット
    # ------------------------------------------------------------------

    def xy_error_m(self) -> Optional[float]:
        """直近の XY 位置誤差ノルム [m](mocap 未受信なら None)。

        supervise 層の発散検知用。on_mocap_pose が更新した「目標 −
        フィルタ済み位置」の誤差キャッシュに基づく(closed loop の
        ON/OFF に関わらず更新される)。
        """
        with self._lock:
            if self._last_filter_result is None:
                return None
            error_x, error_y = self._last_errors
        return (error_x ** 2 + error_y ** 2) ** 0.5

    def mocap_age_s(self, now: Optional[float] = None) -> Optional[float]:
        """最後に有効な mocap pose を受けてからの経過秒(未受信なら None)。"""
        if now is None:
            now = self._clock()
        with self._lock:
            pose_t = self._last_pose_t
        return None if pose_t is None else (now - pose_t)

    def data_valid(self) -> bool:
        """直近フレームの位置データ有効性(信頼度・外れ値・トラッキング)。

        受信鮮度(mocap_age_s)とは独立。START ゲートと UI 表示が使う。
        """
        with self._lock:
            return self._last_data_valid

    def data_invalid_age_s(self, now: Optional[float] = None) -> Optional[float]:
        """データ無効が連続し始めてからの経過秒(現在有効なら None)。

        supervise 層の「受信はあるがデータ無効が続く」フェイルセーフ用。
        無効フレームの到着で計時が始まるため、START を経ずに flying へ
        昇格したケース(飛行中の再接続)でも機能する。
        """
        if now is None:
            now = self._clock()
        with self._lock:
            t = self._invalid_since
        return None if t is None else (now - t)

    def mocap_snapshot(self, now: Optional[float] = None) -> Optional[dict]:
        """WebSocket の "mocap" フィールド用スナップショット(deg/m)。"""
        if now is None:
            now = self._clock()
        with self._lock:
            pose = self._last_pose
            pose_t = self._last_pose_t
            filter_result = self._last_filter_result
            data_valid = self._last_data_valid
        if pose is None or pose_t is None:
            return None
        position = (filter_result["filtered_position"]
                    if filter_result is not None else (pose["x"], pose["y"], pose["z"]))
        confidence = (filter_result["confidence"] if filter_result is not None
                      else pose["quality"])
        snapshot = {
            "x": float(position[0]),
            "y": float(position[1]),
            "z": float(position[2]),
            "yaw_deg": pose["yaw_rad"] * RAD_TO_DEG,
            "confidence": float(confidence),
            "fresh": (now - pose_t) <= self._mocap_fresh_s,
            # フィルタ有効性(受信鮮度と独立)。UI は fresh かつ valid で
            # 「受信中」緑、fresh だが invalid は警告色にする
            "valid": bool(data_valid),
        }
        # 正解ヨー(符号/オフセット/フリップ補正済み)。yaw_deg(旧オイラー
        # 分解・表示専用)と別掲する — UI のヨーモニタと設定タブが使う
        yaw_true = pose.get("yaw_true_rad")
        if yaw_true is not None:
            snapshot["yaw_true_deg"] = yaw_true * RAD_TO_DEG
        flip_flags = pose.get("flip_flags")
        if flip_flags is not None:
            snapshot["flip"] = int(flip_flags)
        return snapshot
