"""Position モード: mocap → フィルタ → 位置誤差 → CMD_POS_ERR 50Hz 送信。

データフロー(機上XY制御。XY PID は機体側 flight_control が実行する):
- NatNet コールバック(on_mocap_pose, NatNetスレッド)で PositionFilter →
  有効性判定 → 最新の位置誤差(目標 − フィルタ済み位置)をキャッシュする。
- 50Hz 送信スレッドは誤差と整形済み alt/yaw を meta に載せて emit する
  (session 層が CMD_POS_ERR に組み立てて送信・ログする)。roll/pitch の
  角度指令は機体側 XY PID が計算するため、この層では生成・整形しない。
- v2 軌道モード: hover(固定目標)/ circle(円軌道)。円軌道は 50Hz 送信
  ループ内で目標 (x, y) を時間更新する(旧 OptiTrack版 NatNet_PID_Controller の
  circling_controller.py の軌道生成を参考。当該フォルダは削除済み)。
  開始時は現在位置から円周
  最近傍点に位相を合わせ、滑らかに合流する。
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
from math import atan2, cos, pi, sin
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

        # v2: 円軌道状態(None = hover)。dict キー:
        # center(x,y) / radius / period_s / clockwise / alt / face_tangent /
        # phase0(合流点の位相 rad) / t0(開始時刻) /
        # frozen_at(MoCap 途絶による位相凍結の開始時刻。None = 凍結なし)
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
    # v2: 円軌道モード
    # ------------------------------------------------------------------

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
        if not (self._traj_radius_min <= radius_m <= self._traj_radius_max):
            return False, (f"半径は {self._traj_radius_min}–"
                           f"{self._traj_radius_max} m で指定してください")
        if not (self._traj_period_min <= period_s <= self._traj_period_max):
            return False, (f"周期は {self._traj_period_min}–"
                           f"{self._traj_period_max} s で指定してください")
        if (abs(center_x) > self._traj_center_abs_max
                or abs(center_y) > self._traj_center_abs_max):
            return False, (f"中心座標は ±{self._traj_center_abs_max} m 以内で"
                           "指定してください")
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

        # 現在位置(フィルタ済み優先)→ 円周最近傍点の位相。位置が未取得の
        # 場合は開始を拒否する(合流点を決められないため)。
        with self._lock:
            filter_result = self._last_filter_result
            pose = self._last_pose
            pose_t = self._last_pose_t
            data_valid = self._last_data_valid
            alt_lo, alt_hi = self._shaper.alt_limits
        if not (alt_lo <= alt_m <= alt_hi):
            return False, f"高度は {alt_lo}–{alt_hi} m で指定してください"
        # 鮮度ガード(session.start と同じ基準値): 途絶中の古い位置から
        # 合流位相を決めると「現在位置から滑らかに合流」(契約 §3.3)が
        # 成立せず、復帰時に目標が実位置から離れた点へジャンプするため拒否。
        age = None if pose_t is None else (now - pose_t)
        if age is None or age > self._dropout_level_s:
            return False, "MoCap データが新鮮でないため円軌道を開始できません"
        # 有効性ガード(session.start と同じ): 無効中のフィルタ位置は
        # 凍結/外挿されたゴーストのため、そこから合流位相を決めない
        if not data_valid:
            return False, ("MoCap 位置データが無効のため円軌道を開始できません"
                           "(トラッキング状態を確認してください)")
        if filter_result is not None:
            px, py, _ = filter_result["filtered_position"]
        elif pose is not None:
            px, py = pose["x"], pose["y"]
        else:
            return False, "MoCap 位置が取得できないため円軌道を開始できません"

        dx, dy = px - center_x, py - center_y
        # 機体が中心に一致している縮退ケースは位相 0 から開始する
        phase0 = atan2(dy, dx) if (dx != 0.0 or dy != 0.0) else 0.0
        with self._lock:
            self._traj = {
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

    def stop_circle(self) -> None:
        """円軌道を停止し、現在の軌道目標でホバリングに復帰する。

        目標 (self._target) は step() が軌道値で毎周期更新しているため、
        軌道を外すだけで「現在目標へのホバ復帰」になる。
        """
        with self._lock:
            self._traj = None
            self._traj_phase = None

    def trajectory_snapshot(self) -> dict:
        """WebSocket 配信用の軌道状態(UI 単位系)。"""
        with self._lock:
            traj = self._traj
            phase = self._traj_phase
        if traj is None:
            return {"mode": "hover"}
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
        # --- 軌道更新(circle 中は目標 (x, y, z) を時間更新する) ---
        traj_phase: Optional[float] = None
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
                if dropped or not self._last_data_valid:
                    if traj["frozen_at"] is None:
                        traj["frozen_at"] = now
                    t_eff = traj["frozen_at"]
                else:
                    if traj["frozen_at"] is not None:
                        traj["t0"] += now - traj["frozen_at"]
                        traj["frozen_at"] = None
                    t_eff = now
                omega = TWO_PI / traj["period_s"]
                # 回転方向: CCW = 位相増加(制御座標系の数学正方向)、CW = 減少
                sign = -1.0 if traj["clockwise"] else 1.0
                phase = traj["phase0"] + sign * omega * (t_eff - traj["t0"])
                cx, cy = traj["center"]
                radius = traj["radius"]
                self._target = (cx + radius * cos(phase),
                                cy + radius * sin(phase),
                                traj["alt"])
                traj_phase = wrap_pi(phase)
                self._traj_phase = traj_phase
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
            "traj_mode": (TRAJ_MODE_CIRCLE if traj_phase is not None
                          else TRAJ_MODE_HOVER),
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
