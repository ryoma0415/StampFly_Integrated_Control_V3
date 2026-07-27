"""SessionManager: pc_server の状態と全コンポーネントを一元管理する。

責務(ARCHITECTURE.md):
- 接続/切断、モード切替(posture/position)、Start/Stop/Reset、ログ ON/OFF
- 機体プロファイル適用: RLY_SET_TARGET(MAC/チャネル)送信+バイアス(deg)を
  rad に変換して送信 roll/pitch 指令へ「加算」
- 単位変換の境界: UI/WebSocket は deg/m、protocol と core 内部は rad/m。
  変換はこの層でのみ行う。
- フェイルセーフ(PROTOCOL.md 規範):
  - MoCap 途絶 >300ms(Position)→ 水平固定+UI警告(固定は position.py)
  - MoCap 途絶 >2s(Position)→ CMD_STOP 送信(自動着陸)
  - STOP 送信後 600ms 以内に LANDING/WAIT イベントなし → 再送(最大3回)+UI警告
  - シリアル切断 → UI 赤色警告(serial_connected=false)

公開メソッドはブロッキングし得る(set_relay_target 最大4秒)。asyncio から
呼ぶ場合は to_thread 等で executor に逃がすこと(app.py 参照)。
UI コマンドは _command_lock で直列化される(phase チェックとそれに続く
動作を原子化し、複数クライアントからの同時コマンドの交錯を防ぐ)。
"""

from __future__ import annotations

import functools
import math
import queue
import threading
import time
from typing import Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from . import config as cfg
from .calibration import CalibrationManager, ack_detail, ack_ok
from .experiment import ExperimentHub
from .ffprofile import FfProfileManager
from . import logger as logger_mod
from .logger import FlightLogger
from .mocap import (DEFAULT_ATTITUDE_TRANSFORM, DEG_TO_RAD, RAD_TO_DEG,
                    MocapSource, validate_mapping)
from .multi import MultiControlManager
from .position import PositionController
from .posture import PostureController, run_paced_loop
from .serial_link import SerialLink, SerialLinkError

MODE_POSTURE = "posture"
MODE_POSITION = "position"
MODE_EXPERIMENT = "experiment"   # v2: モーターテスト/スイープ/キャリブ実験
MODE_MULTI = "multi"             # 複数機同時位置制御(2〜4機、MoCap)

_ALL_MODES = (MODE_POSTURE, MODE_POSITION, MODE_EXPERIMENT, MODE_MULTI)

PHASE_IDLE = "idle"
PHASE_CONNECTED = "connected"
PHASE_ARMED = "armed"
PHASE_FLYING = "flying"

# 飛行中とみなす FlightState(LANDING 中も「flying」フェーズとして扱う)
_IN_FLIGHT_STATES = frozenset({
    proto.FlightState.TAKEOFF, proto.FlightState.HOVER, proto.FlightState.LANDING,
})
_ON_GROUND_STATES = frozenset({
    proto.FlightState.INIT, proto.FlightState.CALIBRATION,
    proto.FlightState.WAIT, proto.FlightState.COMPLETE,
})
_START_REJECT_REASONS = frozenset({
    proto.Reason.START_REJECTED_LOW_VOLTAGE,
    proto.Reason.START_REJECTED_NOT_READY,
})

# shutdown(Ctrl-C 経路)が _command_lock を待つ上限 [s](構造定数)。
# FF 転送など長時間ロックを保持する操作の最中でも、プロセス終了を無期限に
# 待たせない(超過時は構成要素を直接畳む強制経路へ落ちる)。
_SHUTDOWN_LOCK_TIMEOUT_S = 5.0

# テレメトリ途絶時のリレーターゲット再設定で _command_lock を待つ上限 [s]。
# UI コマンド実行中なら今回は見送り、次の監視周期で再試行する。
_REASSERT_LOCK_TIMEOUT_S = 0.5

# WebSocket の "log" メッセージの origin 値(LOG_TEXT 由来以外はサーバ発)
LOG_ORIGIN_NAMES = {proto.LogText.ORIGIN_RELAY: "relay",
                    proto.LogText.ORIGIN_DRONE: "drone"}
LOG_ORIGIN_SERVER = "server"


def _state_name(state: int) -> str:
    try:
        return proto.FlightState(state).name
    except ValueError:
        return f"UNKNOWN({state})"


def _reason_name(reason: int) -> str:
    try:
        return proto.Reason(reason).name
    except ValueError:
        return f"UNKNOWN({reason})"


def _json_safe(value):
    """非有限 float(NaN/Inf)を再帰的に None へ落とす。

    json.dumps は NaN/Infinity を不正な JSON トークンとして出力し
    (allow_nan=True 既定)、ブラウザ側 JSON.parse がフレームごと黙って
    捨てて UI が固まるため、WS へ渡す dict(スナップショット・イベント)は
    すべてこれを通す。TLM 由来の float は struct.unpack が NaN を素通し
    するので、個別フィールドではなくこの一括変換で漏れなく守る。"""
    if isinstance(value, float):
        return value if math.isfinite(value) else None
    if isinstance(value, dict):
        return {k: _json_safe(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(v) for v in value]
    return value


def _ui_command(method):
    """UI コマンドを self._command_lock で直列化するデコレータ。

    phase チェック(self._lock 内)とそれに基づく動作(送信スレッドの
    起動/停止・CMD 送信など)の間に別コマンドが割り込む TOCTOU を防ぐ。
    RLock のため supervisor のフェイルセーフ(stop 呼び出し)とも安全。
    """
    @functools.wraps(method)
    def wrapper(self, *args, **kwargs):
        with self._command_lock:
            return method(self, *args, **kwargs)
    return wrapper


class SessionManager:
    """サーバ全体のセッション状態(唯一のインスタンスを app.py が保持)。"""

    def __init__(self,
                 server_config: Optional[dict] = None,
                 control_config: Optional[dict] = None,
                 airframes: Optional[list[dict]] = None,
                 transport_factory: Optional[Callable] = None,
                 natnet_client_factory: Optional[Callable] = None,
                 clock: Callable[[], float] = time.monotonic) -> None:
        self.server_config = server_config or cfg.load_server_config()
        self.control_config = control_config or cfg.load_control_config()
        self.airframes = airframes if airframes is not None else cfg.load_airframes()
        # 旧形式(rigid_body_id なし)のプロファイルをメモリ上で正規化する
        # (update_airframes の同一性比較・複数機モードの参照を一貫させる)
        for profile in self.airframes:
            profile.setdefault("rigid_body_id", None)

        failsafe = self.server_config["failsafe"]
        self._stop_ack_timeout_s: float = failsafe["stop_ack_timeout_s"]
        self._stop_max_retries: int = failsafe["stop_max_retries"]
        self._start_grace_s: float = failsafe["start_grace_s"]
        self._mocap_dropout_level_s: float = failsafe["mocap_dropout_level_s"]
        self._mocap_dropout_stop_s: float = failsafe["mocap_dropout_stop_s"]
        # データ無効の持続監視(受信はあるが data_valid=0 が続く:
        # トラッキング喪失・外れ値・低信頼度。2026-07 位置固定化障害の教訓)。
        # 追加キーは旧形式の設定でも動くよう .get で読む(tlm_stale_s と同じ)
        self._data_invalid_warn_s: float = failsafe.get("data_invalid_warn_s", 0.5)
        self._data_invalid_stop_s: float = failsafe.get("data_invalid_stop_s", 2.0)
        # XY 誤差の持続的発散(単機。multi.py の divergence 検知と同じ規範:
        # RB 取り違え・偽ソースへの再シードで閉ループが幻の誤差を追う事故の
        # 最終防衛線)
        self._divergence_error_m: float = failsafe.get("divergence_error_m", 1.0)
        self._divergence_hold_s: float = failsafe.get("divergence_hold_s", 1.0)
        self._telemetry_fresh_s: float = self.server_config["freshness"]["telemetry_fresh_s"]
        # 単機テレメトリ途絶監視(リレー再起動などのサイレント断の検出)。
        # 既定値は server.json failsafe 節と同値(旧形式の設定でも動くよう
        # .get で読む)。
        self._tlm_stale_s: float = failsafe.get("tlm_stale_s", 3.0)
        self._relay_reassert_interval_s: float = failsafe.get(
            "relay_reassert_interval_s", 10.0)
        # RLY_STATS(1Hz)の鮮度閾値。受信「時刻」で判定する(counter の内容変化では
        # 判定しない — ターゲット未設定で上りを拒否中は全counterが静止し得るため)
        self._relay_stats_fresh_s: float = self.server_config["freshness"]["relay_stats_fresh_s"]
        self._supervisor_period_s: float = 1.0 / self.server_config["rates"]["supervisor_hz"]

        self._clock = clock

        # UI への即時メッセージ(event / log)。app.py が drain する。
        self.events: queue.Queue = queue.Queue()

        # 機上XY制御(CMD_POS_ERR)の位置誤差クランプ(control.json "control" 節)。
        # Position/Multi の XY 指令は CMD_POS_ERR に一本化されている
        # (旧 PC 側 XY PID 経路は v2.2 で削除。CMD_SETPOINT は Posture 専用)
        control_cfg = self.control_config.get("control", {})
        self._pos_err_clamp_m: float = control_cfg.get("pos_err_clamp_m", 2.0)

        # 構成要素
        self.serial = SerialLink(self.server_config,
                                 transport_factory=transport_factory,
                                 on_disconnect=self._on_serial_disconnect)
        self.mocap = MocapSource(self.control_config["natnet"],
                                 self.control_config["coordinate_transform"],
                                 self.control_config.get("attitude_transform"),
                                 client_factory=natnet_client_factory)
        self.posture = PostureController(self.server_config, self._emit_setpoint,
                                         clock=clock)
        self.position = PositionController(self.server_config, self.control_config,
                                           self._emit_setpoint, clock=clock)
        self.logger = FlightLogger(
            flush_every_rows=self.server_config["logging"]["flush_every_rows"])

        # v2: 実験モード(モーターテスト/スイープ)+キャリブ/FF プロファイル
        self.experiment = ExperimentHub(self.server_config, self.serial,
                                        notify=self.warn)
        self.calibration = CalibrationManager(
            self.server_config, self.serial, self.experiment, notify=self.info,
            tlm_state_provider=self._tlm_state_snapshot)
        self.ffprofile = FfProfileManager(self.serial, self.experiment,
                                          self.calibration, notify=self.warn)
        # 実験計測ログ(ExpRecorder)への供給線: 最新 TLM_STATE スナップ
        # ショットと FF 適用状態(meta 用)。hub 構築時には ffprofile が
        # まだ無いため、ここで配線する(依存注入)。
        self.experiment.recorder.tlm_state_provider = self._tlm_state_snapshot
        self.experiment.recorder.ff_state_provider = self._ff_state_snapshot
        # 複数機同時制御(MODE_MULTI)。ノード付き受信ハンドラは
        # MultiControlManager 自身が serial に登録する。
        self.multi = MultiControlManager(
            self.server_config, self.control_config, self.serial, self.mocap,
            notify_info=self.info, notify_warn=self.warn,
            # WS 行きの dict は非有限 float を必ず None 化する(規約)
            events_put=lambda event: self.events.put(_json_safe(event)),
            clock=clock)

        # 受信ディスパッチ登録(ハンドラは RX スレッド上: ブロッキング禁止)
        self.serial.register_handler(proto.MsgType.TLM_STATE, self._on_tlm_state)
        self.serial.register_handler(proto.MsgType.TLM_CTRL, self._on_tlm_ctrl)
        self.serial.register_handler(proto.MsgType.TLM_EVENT, self._on_tlm_event)
        self.serial.register_handler(proto.MsgType.LOG_TEXT, self._on_log_text)
        self.serial.register_handler(proto.MsgType.RLY_STATS, self._on_rly_stats)
        self.serial.register_handler(proto.MsgType.TLM_EXP, self._on_tlm_exp)
        self.serial.register_handler(proto.MsgType.TLM_CAL_DATA,
                                     self._on_tlm_cal_data)
        # マルチ機体: ノード帰属つき TLM_CAL_DATA(機体別 FF 適用の読み戻しが
        # 別ノードの遅延応答で汚染されないよう、ノード別スロットへ振り分ける)
        self.serial.register_node_handler(proto.MsgType.TLM_CAL_DATA,
                                          self._on_tlm_cal_data_node)

        # UI コマンドの直列化(_ui_command デコレータが使用)。supervisor の
        # stop()/teardown とも共有する。self._lock より外側で取得すること
        # (逆順で取るとデッドロックする)。
        self._command_lock = threading.RLock()

        # セッション状態(self._lock で保護)
        self._lock = threading.Lock()
        self._mode = MODE_POSTURE
        self._phase = PHASE_IDLE
        self._airframe: Optional[dict] = None
        self._bias_roll_rad = 0.0
        self._bias_pitch_rad = 0.0
        self._relay_target_ok = False
        self._logging_enabled = False
        self._armed_since: Optional[float] = None
        self._link_lost = False
        # 実験モード: 機体が MOTOR_TEST 状態であることを ACK で確認済みか
        self._experiment_active = False

        # STOP 再送管理: None または {"deadline": t, "resends": n}
        self._stop_pending: Optional[dict] = None

        # MoCap 途絶警告のエピソード管理
        self._mocap_warned = False
        self._mocap_stop_sent = False

        # データ無効(受信はあるが data_valid=0 継続)警告のエピソード管理
        self._data_invalid_warned = False
        self._data_invalid_stop_sent = False
        # XY 誤差が divergence_error_m を超え続けている開始時刻(発散検知)
        self._error_high_since: Optional[float] = None

        # 単機テレメトリ途絶(リレー再起動等のサイレント断)監視の状態
        self._connected_at: Optional[float] = None
        self._tlm_stale_warned = False
        self._relay_reassert_at: Optional[float] = None
        self._relay_reassert_busy = False

        # 最新テレメトリ
        self._tlm_state: Optional[proto.TlmState] = None
        self._tlm_state_t: Optional[float] = None
        self._tlm_ctrl: Optional[proto.TlmCtrl] = None   # 制御ループ診断(25Hz)
        self._tlm_ctrl_t: Optional[float] = None
        self._rly_stats: Optional[proto.RlyStats] = None
        self._rly_stats_t: Optional[float] = None   # 受信時刻(鮮度判定用)

        # 監視スレッド
        self._supervisor_thread: Optional[threading.Thread] = None
        self._supervisor_stop = threading.Event()

        # 既定の機体プロファイル(UI が select_airframe するまでの初期値)。
        # MAC 未設定のプロファイルは選択できないため、「MAC が設定済みの最初の
        # プロファイル」を既定とする。1件もなければ未選択のまま起動する
        # (connect 時に「機体プロファイルが選択されていません」と案内される)。
        default_profile = next(
            (p for p in self.airframes if cfg.mac_is_set(p.get("mac"))), None)
        if default_profile is not None:
            self._apply_airframe(default_profile)

    # ==================================================================
    # UI コマンド(app.py から executor 経由で呼ばれる。ブロッキング可)
    # ==================================================================

    @_ui_command
    def connect(self, port: str) -> bool:
        """シリアル接続し、リレーに機体ターゲットを設定、送信を開始する。"""
        with self._lock:
            if self._phase != PHASE_IDLE:
                self._warn_locked("既に接続済みです")
                return False
            airframe = self._airframe
        if airframe is None:
            self.warn("機体プロファイルが選択されていません")
            return False

        try:
            self.serial.connect(port)
        except SerialLinkError as exc:
            self.warn(f"シリアル接続失敗: {exc}")
            return False

        with self._lock:
            self._phase = PHASE_CONNECTED
            self._link_lost = False
            self._stop_pending = None
            self._tlm_state = None
            self._tlm_state_t = None
            self._tlm_ctrl = None
            self._tlm_ctrl_t = None
            self._rly_stats = None
            self._rly_stats_t = None
            self._connected_at = self._clock()
            self._tlm_stale_warned = False
            self._relay_reassert_at = None

        # リレーへ ESP-NOW ピア設定(各1.0s 待ち、初回+最大3回再送=最大4回。
        # 完了まで上り転送はリレー側で拒否されるため、送信スレッド起動より先に行う)
        self._configure_relay_target(airframe)

        with self._lock:
            mode = self._mode
        if mode == MODE_EXPERIMENT:
            # 実験モードで接続: 50Hz 送信は開始せず CMD_MODE(1)+ACK
            self._enter_experiment()
        elif mode == MODE_MULTI:
            pass   # 複数機モード: multi_select(RLY_SET_PEERS)まで送信なし
        else:
            self._start_active_sender()
        self._supervisor_stop.clear()
        self._supervisor_thread = threading.Thread(
            target=self._supervisor_loop, name="session-supervisor", daemon=True)
        self._supervisor_thread.start()
        self.info(f"接続しました: {port}")
        return True

    @_ui_command
    def disconnect(self) -> None:
        """送信停止・mocap停止・ログ停止のうえシリアルを閉じる。"""
        with self._lock:
            was_active = self._phase != PHASE_IDLE
        self._supervisor_stop.set()
        thread = self._supervisor_thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=1.0)
        self._supervisor_thread = None
        if not was_active and not self.serial.is_connected:
            return   # もともと未接続
        self._teardown_session("切断しました")

    @_ui_command
    def select_airframe(self, name: str) -> bool:
        """機体プロファイルを選択し、接続中なら RLY_SET_TARGET を再送する。"""
        profile = next((p for p in self.airframes if p["name"] == name), None)
        if profile is None:
            self.warn(f"機体プロファイルが見つかりません: {name}")
            return False
        if not cfg.mac_is_set(profile.get("mac")):
            # MAC 未設定プロファイルは接続中・未接続を問わず選択不可
            # (リレーへ設定すべきターゲットが存在しないため)
            self.warn(f"機体「{name}」は MAC が未設定のため選択できません。"
                      "機体を USB 接続し起動ログ『ESP-NOW ready: MAC=...』で MAC を"
                      "確認して、機体プロファイル編集画面で設定してください")
            return False
        with self._lock:
            if self._phase in (PHASE_ARMED, PHASE_FLYING):
                self._warn_locked("飛行中は機体プロファイルを変更できません")
                return False
        if self.multi.active:
            # RLY_SET_TARGET はリレーのマルチピア表をクリアするため、選択中は
            # 地上でも拒否する(飛行中に許すと全機への上り経路が切断される)
            self.warn("複数機モードの機体選択中は機体プロファイルを変更できません"
                      "(先に複数機の選択を解除してください)")
            return False
        self._apply_airframe(profile)
        if self.serial.is_connected:
            self._configure_relay_target(profile)
        self.info(f"機体プロファイル: {name}")
        return True

    @_ui_command
    def update_airframes(self, new_airframes) -> tuple[bool, Optional[str]]:
        """機体プロファイル一覧を検証・保存し、セッションへ反映する。

        戻り値は (ok, error)。error は UI 表示用の日本語メッセージ。

        ポリシー(本実装の規範):
        - 検証: 名前は非空かつ一意 / mac は空(未設定)または 6 オクテット16進 /
          wifi_channel・バイアス・default_alt_m は config の制限内 / notes は文字列。
          件数と name/notes の文字数は airframe_limits の上限内
          (max_profiles / name_max_chars / notes_max_chars)。
        - 飛行ガード: phase が armed/flying のとき、選択中プロファイルが
          変更・削除される更新は拒否する(飛行中の機体の挙動を変えないため)。
          選択中プロファイルが同一内容のままなら、他プロファイルの編集は許可。
        - 受理時: airframes.json へ原子的に保存し self.airframes を更新する。
        - 選択中プロファイルが残存し bias / default_alt_m が変わった場合
          (飛行中でないときのみ到達し得る)は即座に再適用する。
        - 選択中プロファイルが削除 / MAC 未設定化された場合は選択を解除し
          relay_target_ok を false にする(未選択のままの start は拒否される)。
        - MAC / wifi_channel の変更は次の select_airframe / connect で反映する。
          RLY_SET_TARGET の自動再送はしない(接続中はオペレータへ選び直しを促す
          ログを出す)。
        """
        ok, error, normalized = self._validate_airframes(new_airframes)
        if not ok:
            return False, error

        with self._lock:
            phase = self._phase
            current = self._airframe

        # --- 飛行ガード: 選択中プロファイルの変更/削除を拒否 ---
        if phase in (PHASE_ARMED, PHASE_FLYING) and current is not None:
            replacement = next(
                (p for p in normalized if p["name"] == current["name"]), None)
            if replacement != current:
                return False, (f"飛行中のため、選択中の機体プロファイル"
                               f"「{current['name']}」の変更・削除はできません。"
                               "着陸後にやり直してください")

        # --- 飛行ガード(複数機): 飛行中スロットのプロファイル変更/削除を拒否 ---
        for name, profile in self.multi.flying_profiles().items():
            replacement = next(
                (p for p in normalized if p["name"] == name), None)
            if replacement != profile:
                return False, (f"複数機制御で飛行中のため、機体プロファイル"
                               f"「{name}」の変更・削除はできません。"
                               "着陸後にやり直してください")

        # --- 永続化(原子的書き込み)→ メモリ反映 ---
        try:
            cfg.save_airframes(normalized)
        except OSError as exc:
            return False, f"airframes.json の保存に失敗しました: {exc}"
        self.airframes = normalized

        self._refresh_selected_airframe(current)
        self.info(f"機体プロファイルを更新しました({len(normalized)}件)")
        return True, None

    def _refresh_selected_airframe(self, current: Optional[dict]) -> None:
        """update_airframes 受理後、選択中プロファイルを新リストへ追従させる。"""
        if current is None:
            return
        replacement = next(
            (p for p in self.airframes if p["name"] == current["name"]), None)

        if replacement is None or not cfg.mac_is_set(replacement["mac"]):
            # 削除された / MAC が未設定になった → 選択解除(飛行ガード通過済み =
            # 非飛行時のみ到達)。バイアスもゼロへ戻す。リレーのピア設定自体は
            # 残る(STOP は届く)が「選択中プロファイルに対応する設定」ではなく
            # なったため relay_target_ok を落とし、UI のリレー表示で選び直しを促す。
            with self._lock:
                self._airframe = None
                self._bias_roll_rad = 0.0
                self._bias_pitch_rad = 0.0
                self._relay_target_ok = False
            reason = ("削除された" if replacement is None else "MAC が未設定になった")
            self.warn(f"選択中の機体プロファイル「{current['name']}」が{reason}ため"
                      "選択を解除しました。機体を選び直してください")
            return

        bias_changed = (
            replacement["roll_bias_deg"] != current["roll_bias_deg"]
            or replacement["pitch_bias_deg"] != current["pitch_bias_deg"]
            or replacement["default_alt_m"] != current["default_alt_m"])
        target_changed = (replacement["mac"] != current["mac"]
                          or replacement["wifi_channel"] != current["wifi_channel"])

        with self._lock:
            self._airframe = replacement
            if bias_changed:
                # 飛行ガード通過済みのため、ここに来るのは非飛行時のみ。
                # 次の 50Hz 送信から新バイアスが加算される。
                self._bias_roll_rad = replacement["roll_bias_deg"] * DEG_TO_RAD
                self._bias_pitch_rad = replacement["pitch_bias_deg"] * DEG_TO_RAD
        if bias_changed:
            self.posture.set_default_alt(replacement["default_alt_m"])
            self.info(f"機体「{replacement['name']}」のバイアス/初期高度を"
                      "再適用しました")
        if target_changed and self.serial.is_connected:
            # RLY_SET_TARGET は黙って再送しない(契約)。選び直しで反映させる。
            self.warn(f"機体「{replacement['name']}」の MAC/チャネル変更は"
                      "まだリレーに反映されていません。機体プロファイルを"
                      "選び直してください")

    def _validate_airframes(self, new_airframes) \
            -> tuple[bool, Optional[str], list[dict]]:
        """プロファイル配列を検証し、正規化済みリストを返す。

        正規化: キーを正準順(name, mac, wifi_channel, roll_bias_deg,
        pitch_bias_deg, default_alt_m, rigid_body_id, notes)に揃え、
        数値型・MAC 表記("AA:BB:..." 大文字)を統一する。制限値は
        server.json(clamps.max_roll_pitch_deg / airframe_limits)から取る。
        rigid_body_id は任意(複数機モードで必須。null = 未設定)。
        """
        limits = self.server_config["airframe_limits"]
        ch_min, ch_max = limits["wifi_channel_min"], limits["wifi_channel_max"]
        alt_min, alt_max = limits["default_alt_min_m"], limits["default_alt_max_m"]
        max_profiles = limits["max_profiles"]
        name_max = limits["name_max_chars"]
        notes_max = limits["notes_max_chars"]
        bias_limit = self.server_config["clamps"]["max_roll_pitch_deg"]
        required_keys = ("name", "mac", "wifi_channel", "roll_bias_deg",
                         "pitch_bias_deg", "default_alt_m", "notes")
        optional_keys = ("rigid_body_id",)   # 複数機モード用(null = 未設定)

        if not isinstance(new_airframes, list) or not new_airframes:
            return False, "機体プロファイルは1件以上の配列で指定してください", []
        if len(new_airframes) > max_profiles:
            # 設定ファイル・UI プルダウンの肥大化を防ぐ上限
            return False, (f"機体プロファイルは最大 {max_profiles} 件までです"
                           f"({len(new_airframes)} 件指定されました)"), []

        normalized: list[dict] = []
        names: set[str] = set()
        for index, entry in enumerate(new_airframes):
            label = f"{index + 1}件目"
            if not isinstance(entry, dict):
                return False, f"{label}: プロファイルはオブジェクトで指定してください", []
            missing = [k for k in required_keys if k not in entry]
            if missing:
                return False, f"{label}: キーが不足しています: {', '.join(missing)}", []
            unknown = [k for k in entry
                       if k not in required_keys and k not in optional_keys]
            if unknown:
                return False, f"{label}: 不明なキーがあります: {', '.join(unknown)}", []

            name = entry["name"]
            if not isinstance(name, str) or not name.strip():
                return False, f"{label}: 機体名が空です", []
            name = name.strip()
            if len(name) > name_max:
                return False, (f"{label}: 機体名は {name_max} 文字以内で"
                               "指定してください"), []
            label = f"「{name}」"
            if name in names:
                return False, f"機体名が重複しています: {label}", []
            names.add(name)

            mac_raw = entry["mac"]
            if not isinstance(mac_raw, str):
                return False, f"{label}: MAC は文字列で指定してください", []
            if cfg.mac_is_set(mac_raw):
                try:
                    mac = cfg.format_mac(cfg.parse_mac(mac_raw.strip()))
                except ValueError:
                    return False, (f"{label}: MAC の形式が不正です: {mac_raw!r}"
                                   "(AA:BB:CC:DD:EE:FF 形式、未設定なら空欄)"), []
            else:
                mac = cfg.MAC_UNSET

            channel = entry["wifi_channel"]
            if isinstance(channel, bool) or not isinstance(channel, int) \
                    or not (ch_min <= channel <= ch_max):
                return False, (f"{label}: wifi_channel は {ch_min}–{ch_max} の"
                               "整数で指定してください"), []

            numbers: dict[str, float] = {}
            for key, low, high, jp in (
                    ("roll_bias_deg", -bias_limit, bias_limit, "Roll バイアス"),
                    ("pitch_bias_deg", -bias_limit, bias_limit, "Pitch バイアス"),
                    ("default_alt_m", alt_min, alt_max, "初期高度")):
                value = entry[key]
                if isinstance(value, bool) or not isinstance(value, (int, float)):
                    return False, f"{label}: {jp}({key})は数値で指定してください", []
                if not (low <= value <= high):
                    return False, (f"{label}: {jp}({key})は {low}–{high} の"
                                   "範囲で指定してください"), []
                numbers[key] = float(value)

            notes = entry["notes"]
            if not isinstance(notes, str):
                return False, f"{label}: notes は文字列で指定してください", []
            if len(notes) > notes_max:
                return False, (f"{label}: notes は {notes_max} 文字以内で"
                               "指定してください"), []

            rigid_body_id = entry.get("rigid_body_id")
            if rigid_body_id is not None:
                if isinstance(rigid_body_id, bool) \
                        or not isinstance(rigid_body_id, int) \
                        or rigid_body_id < 1:
                    return False, (f"{label}: rigid_body_id は 1 以上の整数"
                                   "(未設定なら null)で指定してください"), []

            normalized.append({
                "name": name,
                "mac": mac,
                "wifi_channel": channel,
                "roll_bias_deg": numbers["roll_bias_deg"],
                "pitch_bias_deg": numbers["pitch_bias_deg"],
                "default_alt_m": numbers["default_alt_m"],
                "rigid_body_id": rigid_body_id,
                "notes": notes,
            })
        return True, None, normalized

    @_ui_command
    def set_mode(self, mode: str) -> bool:
        """posture/position/experiment モードを切り替える(飛行中は不可)。

        experiment 開始(契約 §3.1): 飛行中は拒否 → 50Hz セットポイント送信
        停止 → CMD_MODE(1) 送信+ACK 確認。experiment 終了: モーター停止確認 →
        CMD_MODE(0)+ACK → posture/position に復帰(50Hz 送信再開)。
        未接続時はモードのみ切り替え、CMD_MODE は次の connect で送る。
        """
        if mode not in _ALL_MODES:
            self.warn(f"不明なモード: {mode}")
            return False
        with self._lock:
            if self._phase in (PHASE_ARMED, PHASE_FLYING):
                self._warn_locked("飛行中はモードを変更できません")
                return False
            if self._mode == mode:
                return True
            previous = self._mode
        if self.multi.any_armed_or_flying():
            self.warn("複数機制御中はモードを変更できません")
            return False
        if previous == MODE_EXPERIMENT:
            # 実験モードからの離脱: モーター停止確認 → CMD_MODE(0)+ACK
            self._exit_experiment()
        if previous == MODE_MULTI:
            # 複数機モードからの離脱: スロット解放+単機ターゲットへ復帰
            self._exit_multi()
        self._stop_active_sender()
        with self._lock:
            self._mode = mode
        if self.serial.is_connected:
            if mode == MODE_EXPERIMENT:
                if not self._enter_experiment():
                    # 機体が MOTOR_TEST に入れなかった → 元のモードへ戻す
                    with self._lock:
                        self._mode = previous
                    if previous not in (MODE_EXPERIMENT, MODE_MULTI):
                        self._start_active_sender()
                    return False
            elif mode == MODE_MULTI:
                pass   # multi_select(RLY_SET_PEERS)まで送信なし
            else:
                self._start_active_sender()
        self.info(f"モード: {mode}")
        return True

    @_ui_command
    def activate_experiment(self) -> bool:
        """実験モードの再有効化(CMD_STOP 等で機体が WAIT に戻った後の再開)。"""
        with self._lock:
            mode = self._mode
        if mode != MODE_EXPERIMENT:
            self.warn("実験モードではありません")
            return False
        if not self.serial.is_connected:
            self.warn("未接続のため実験モードを有効化できません")
            return False
        return self._enter_experiment()

    def _enter_experiment(self) -> bool:
        """CMD_MODE(1) を送信し ACK を確認して実験機能を有効化する。"""
        payload = proto.CmdMode(mode=proto.CmdMode.MODE_MOTOR_TEST).to_payload()
        try:
            ack = self.serial.send_with_ack(proto.MsgType.CMD_MODE, payload)
        except SerialLinkError as exc:
            self.warn(f"CMD_MODE(MOTOR_TEST) 送信失敗: {exc}")
            return False
        if not ack_ok(ack):
            self.warn(f"実験モードに入れませんでした"
                      f"(CMD_MODE ACK: {ack_detail(ack)})。"
                      "機体が WAIT 状態か確認してください")
            return False
        with self._lock:
            self._experiment_active = True
        self.experiment.activate()
        self.info("実験モード開始(機体: MOTOR_TEST)")
        return True

    def _exit_multi(self) -> None:
        """複数機モードの離脱: スロット解放+リレーを単機ターゲットへ戻す。"""
        with self._lock:
            airframe = self._airframe
        if self.serial.is_connected and airframe is not None \
                and cfg.mac_is_set(airframe.get("mac")):
            # ピア表は RLY_SET_TARGET が上書きクリアするため個別クリア不要
            self.multi.deactivate(clear_peers=False)
            self._configure_relay_target(airframe)
        else:
            self.multi.deactivate(clear_peers=True)

    def _exit_experiment(self) -> None:
        """実験機能を停止し、接続中なら CMD_MODE(0) で WAIT に戻す。"""
        self.experiment.deactivate()   # スイープ中断+モーター停止を含む
        with self._lock:
            was_active = self._experiment_active
            self._experiment_active = False
        if not (was_active and self.serial.is_connected):
            return
        payload = proto.CmdMode(mode=proto.CmdMode.MODE_FLIGHT).to_payload()
        try:
            ack = self.serial.send_with_ack(proto.MsgType.CMD_MODE, payload)
        except SerialLinkError as exc:
            self.warn(f"CMD_MODE(FLIGHT) 送信失敗: {exc}")
            return
        if not ack_ok(ack):
            # 機体側フェイルセーフ(モーター1.5s途絶停止)に任せ、警告のみ
            self.warn(f"実験モード終了の ACK が確認できません"
                      f"({ack_detail(ack)})。機体状態を確認してください")
        else:
            self.info("実験モード終了(機体: WAIT)")

    @_ui_command
    def start(self) -> bool:
        """離陸開始(CMD_START)。connected フェーズかつ機体選択中のみ受け付ける。"""
        with self._lock:
            phase = self._phase
            mode = self._mode
            airframe = self._airframe
        if phase != PHASE_CONNECTED:
            self.warn(f"開始できません(phase={phase})")
            return False
        if mode == MODE_EXPERIMENT:
            # 実験モード中は START 不可(契約 §3.1。機体側も MOTOR_TEST 中の
            # CMD_START を reason=10 で拒否する)
            self.warn("実験モード中は離陸できません")
            return False
        if mode == MODE_MULTI:
            # 複数機モードは専用の一斉開始(multi_start)を使う
            self.warn("複数機モードでは「一斉スタート」を使用してください")
            return False
        if airframe is None:
            # 接続後でも PUT /api/airframes で選択中プロファイルが削除/MAC 未設定化
            # されると選択は解除される(_refresh_selected_airframe)。プロファイル
            # 未選択のまま ARM しない(connect() と同じガード)。
            self.warn("機体プロファイルが選択されていません。"
                      "機体プロファイルを選び直してください")
            return False
        if mode == MODE_POSITION:
            age = self.position.mocap_age_s(self._clock())
            if age is None or age > self._mocap_dropout_level_s:
                self.warn("MoCap データが新鮮でないため開始できません")
                return False
            # 受信していてもフィルタ/トラッキングが無効なら閉ループを開始
            # しない(位置固定化状態での盲目離陸を防ぐ。フィルタは連続外れ値
            # から自動再シードするため、健全なら数百 ms 以内に有効へ戻る)
            if not self.position.data_valid():
                self.warn("MoCap 位置データが無効のため開始できません"
                          "(トラッキング状態を確認し、数秒待って再試行)")
                return False
        try:
            self.serial.send(proto.MsgType.CMD_START)
        except SerialLinkError as exc:
            self.warn(f"CMD_START 送信失敗: {exc}")
            return False
        with self._lock:
            self._phase = PHASE_ARMED
            self._armed_since = self._clock()
            logging_enabled = self._logging_enabled
        if logging_enabled:
            # 飛行ログの寿命は「START 受理 〜 飛行終了」(_finish_flight_log)
            self._open_flight_log()
        if mode == MODE_POSITION:
            # 飛行単位でフィルタを仕切り直す(前飛行の外れ値ロックアウトや
            # 古い速度推定を持ち越さない)。次フレームで即再シードされる
            self.position.reset_filter()
            self.position.set_control_active(True)
            # 前飛行のフェイルセーフ・エピソードを持ち越さない
            with self._lock:
                self._data_invalid_warned = False
                self._data_invalid_stop_sent = False
                self._error_high_since = None
        self.info("CMD_START 送信")
        return True

    @_ui_command
    def stop(self) -> bool:
        """即時着陸(CMD_STOP)。全飛行状態で受け付け、応答なしなら再送する。"""
        if not self.serial.is_connected:
            self.warn("未接続のため停止コマンドを送れません")
            return False
        self.position.set_control_active(False)
        with self._lock:
            mode = self._mode
        if mode == MODE_MULTI:
            # 複数機モード: 全機へ CMD_STOP(スロットごとの再送監視つき)
            return self.multi.stop_all()
        if mode == MODE_EXPERIMENT:
            # 緊急停止: スイープ/シーケンス中断+モーター停止(キープアライブ
            # が回し続けないよう PC 側状態も落とす)。CMD_STOP で機体は
            # MOTOR_TEST→WAIT に遷移するため、実験の再開には再有効化が必要。
            self.experiment.sequence.abort_if_running()
            self.experiment.sweep.abort_if_running()
            self.experiment.motor_stop()
            with self._lock:
                self._experiment_active = False
        # 再送監視は送信「前」に仕掛ける: 送信直後に届く LANDING イベント
        # (RXスレッド)が _stop_pending を消す方が常に後勝ちになるようにし、
        # 着陸成功後の偽の再送・「応答なし」警告を防ぐ。
        with self._lock:
            self._stop_pending = {
                "deadline": self._clock() + self._stop_ack_timeout_s,
                "resends": 0,
            }
        ok = self._send_stop()
        if not ok:
            # 送信できていないなら応答を待つ意味がない(再送は次の stop で)
            with self._lock:
                self._stop_pending = None
        return ok

    def emergency_stop(self) -> None:
        """SPACE 緊急停止の優先経路(_command_lock を**経由しない**)。

        低速コマンド(select_airframe の RLY_SET_TARGET ACK 待ち最大約4秒、
        connect 等)が _command_lock を保持していても、モーターキープアライブの
        停止と CMD_STOP の送出を先行させる。放置するとキープアライブが
        CMD_MOTOR_RUN を 0.4s 周期で送り続け、機体側 1.5s 途絶フェイルセーフも
        発動しないまま緊急停止が最大約4秒遅れる。

        ここで触るのは hub 内部ロック・serial の TX ロックのみで完結する
        操作に限る(フェーズ遷移や STOP 再送監視などの状態管理は、続けて
        呼ばれる通常の stop() が _command_lock 下で行う)。
        """
        self.position.set_control_active(False)
        # 複数機モード: 全機へ CMD_STOP を先行送出(状態管理は後続の stop())
        self.multi.emergency_stop_all()
        # スイープ/シーケンス中断+モーター停止+キープアライブ停止
        # (CMD_MOTOR_STOP は飛行状態では機体側が bad_state で無害に破棄する)
        self.experiment.sequence.abort_if_running()
        self.experiment.sweep.abort_if_running()
        self.experiment.motor_stop()
        if self.serial.is_connected and not self.multi.active:
            # 単機経路の CMD_STOP(マルチ中は非エンベロープ上りをリレーが
            # 拒否するため送らない — 全機分は emergency_stop_all が送出済み)
            self._send_stop()

    @_ui_command
    def reset(self) -> bool:
        """CMD_RESET(COMPLETE からの復帰)。受理条件はファームが検証する。"""
        if not self.serial.is_connected:
            self.warn("未接続のためリセットできません")
            return False
        try:
            self.serial.send(proto.MsgType.CMD_RESET)
        except SerialLinkError as exc:
            self.warn(f"CMD_RESET 送信失敗: {exc}")
            return False
        self.info("CMD_RESET 送信")
        return True

    @_ui_command
    def set_logging(self, enabled: bool) -> None:
        """CSV ログ予約の ON/OFF。

        _logging_enabled は「予約フラグ」: ON にしても即座にはファイルを
        開かず、START(CMD_START 受理)/一斉開始(multi_start)でファイルを
        開く。飛行中(単機 armed/flying、または複数機スロットの
        armed/flying)に ON を受けたときのみ即座に開く(途中からの記録)。
        OFF は従来どおり即座に閉じる。飛行終了時は _finish_flight_log が
        ファイルを閉じてフラグを自動 OFF する。
        """
        with self._lock:
            self._logging_enabled = bool(enabled)
            phase = self._phase
        if enabled:
            if phase in (PHASE_ARMED, PHASE_FLYING):
                self._open_flight_log()   # 飛行中の途中から記録
            elif self.multi.any_armed_or_flying():
                # 複数機の飛行中 ON → 機体別ログを即開く(途中からの記録)
                for name in self.multi.open_flight_logs():
                    self.info(f"ログ開始: {name}")
            else:
                self.info("ログ予約: 次の飛行開始(START)から記録します")
        else:
            self.logger.stop()
            self.multi.close_flight_logs()
            self.info("ログ停止")

    def set_setpoint_deg(self, roll_deg: float, pitch_deg: float, alt_m: float,
                         yaw_deg: Optional[float] = None) -> None:
        """Posture モードの UI setpoint(deg/m)。rad へ変換して渡す。"""
        self.posture.set_setpoint(
            roll_deg * DEG_TO_RAD, pitch_deg * DEG_TO_RAD, alt_m,
            yaw_rad=None if yaw_deg is None else yaw_deg * DEG_TO_RAD)

    def set_target(self, x: float, y: float, z: float) -> None:
        """Position モードの目標位置(制御座標系 m)。"""
        self.position.set_target(x, y, z)

    def set_yaw_setpoint_deg(self, yaw_deg: float) -> None:
        """UI ヨー角スライダ(±180°)。両モードのコントローラへ反映する。"""
        yaw_rad = yaw_deg * DEG_TO_RAD
        self.posture.set_setpoint_yaw_only(yaw_rad)
        self.position.set_yaw_setpoint(yaw_rad)

    @_ui_command
    def set_yaw_control(self, enabled: bool) -> None:
        """ヨー角制御 ON/OFF(CMD_SETPOINT flags bit1)。"""
        self.posture.set_yaw_control(enabled)
        self.position.set_yaw_control(enabled)
        self.info(f"ヨー角制御: {'ON' if enabled else 'OFF'}")

    # ------------------------------------------------------------------
    # v2: 円軌道モード(Position タブ)
    # ------------------------------------------------------------------

    @_ui_command
    def circle_start(self, center_x: float, center_y: float, radius_m: float,
                     period_s: float, clockwise: bool, alt_m: float,
                     face_tangent: bool) -> bool:
        with self._lock:
            mode = self._mode
        if mode != MODE_POSITION:
            self.warn("円軌道は Position モードでのみ使用できます")
            return False
        ok, error = self.position.start_circle(
            center_x, center_y, radius_m, period_s, clockwise, alt_m,
            face_tangent, now=self._clock())
        if not ok:
            self.warn(f"円軌道を開始できません: {error}")
            return False
        direction = "CW" if clockwise else "CCW"
        self.info(f"円軌道開始: 中心=({center_x:.2f}, {center_y:.2f}) "
                  f"r={radius_m:.2f}m 周期={period_s:.1f}s {direction} "
                  f"高度={alt_m:.2f}m"
                  + ("(進行方向を向く)" if face_tangent else ""))
        return True

    @_ui_command
    def circle_stop(self) -> None:
        self.position.stop_circle()
        self.info("円軌道停止: 現在目標でホバリングに復帰します")

    # ------------------------------------------------------------------
    # 複数機同時制御(Multi タブ)
    # ------------------------------------------------------------------

    @_ui_command
    def multi_select(self, names: list[str]) -> bool:
        """複数機モードの機体選択(RLY_SET_PEERS + スロット構築)。"""
        with self._lock:
            mode = self._mode
        if mode != MODE_MULTI:
            self.warn("複数機モードではありません")
            return False
        ok, message = self.multi.select(names, self.airframes)
        if not ok:
            self.warn(f"機体選択不可: {message}")
        return ok

    @_ui_command
    def multi_target(self, name: str, x: float, y: float, z: float) -> bool:
        """機体別の目標位置(制御座標系 m)。

        _command_lock で multi_start と直列化する(一斉開始の目標間隔検証と
        目標変更の TOCTOU を防ぐ。UI はボタン押下時のみ送るため低頻度)。
        """
        ok, message = self.multi.set_target(name, x, y, z)
        if not ok:
            self.warn(f"目標設定不可: {message}")
        return ok

    @_ui_command
    def multi_start(self) -> bool:
        """選択済み全機の一斉離陸(ログ予約 ON なら機体別ログを開く)。"""
        with self._lock:
            mode = self._mode
        if mode != MODE_MULTI:
            self.warn("複数機モードではありません")
            return False
        ok, message = self.multi.start_all()
        if not ok:
            self.warn(f"一斉開始不可: {message}")
            return False
        with self._lock:
            logging_enabled = self._logging_enabled
        if logging_enabled:
            # 飛行ログの寿命は単機と同じ「開始〜飛行終了」。閉じ側は
            # 全機着陸の検知(supervise → _finish_flight_log)が担う
            for name in self.multi.open_flight_logs():
                self.info(f"ログ開始: {name}")
        return True

    def multi_yaw(self, name: str, enabled=None, yaw_deg=None) -> bool:
        """機体別ヨー角制御 ON/OFF・ヨー目標 [deg](複数機モード)。"""
        ok, message = self.multi.set_yaw(
            name,
            enabled=None if enabled is None else bool(enabled),
            yaw_deg=None if yaw_deg is None else float(yaw_deg))
        if not ok:
            self.warn(f"ヨー設定不可: {message}")
        return ok

    def _multi_ff_slot(self, drone: str) -> tuple[Optional[dict], str]:
        """機体別 FF 操作のスロット解決+ガード(選択済み・全機地上のみ)。

        FF 転送は _command_lock を数十秒保持し得るため、**いずれかの機体が
        飛行中の間は全面拒否**する(飛行中に stop() の再送監視が
        _command_lock 待ちでブロックされる事故を防ぐ)。
        """
        with self._lock:
            mode = self._mode
        if mode != MODE_MULTI:
            return None, "複数機モードではありません"
        if self.multi.any_armed_or_flying():
            return None, ("飛行中の機体があるため FF 操作できません"
                          "(全機着陸後にやり直してください)")
        info = self.multi.slot_info(drone)
        if info is None:
            return None, f"機体「{drone}」は選択されていません"
        return info, ""

    @_ui_command
    def multi_ff_apply(self, drone: str, name, ff=None, est=None,
                       force: bool = False) -> dict:
        """機体別 FF プロファイル適用(複数機モード。ノード宛+MAC 別状態)。"""
        info, reason = self._multi_ff_slot(drone)
        if info is None:
            self.warn(f"FF適用不可: {reason}")
            return {"ok": False, "message": reason}
        return self.ffprofile.apply(name, ff=ff, est=est, force=force,
                                    node_id=info["node_id"], mac=info["mac"])

    @_ui_command
    def multi_ff_mode(self, drone: str, ff, est) -> dict:
        """機体別 ff_mode / est_mode の実行時切替(複数機モード)。"""
        info, reason = self._multi_ff_slot(drone)
        if info is None:
            self.warn(f"FFモード変更不可: {reason}")
            return {"ok": False, "message": reason}
        return self.ffprofile.mode(ff, est, node_id=info["node_id"],
                                   mac=info["mac"])

    @_ui_command
    def multi_ff_anchor(self, drone: str) -> dict:
        """機体別アンカー再取得(複数機モード)。"""
        info, reason = self._multi_ff_slot(drone)
        if info is None:
            self.warn(f"アンカー再取得不可: {reason}")
            return {"ok": False, "message": reason}
        return self.ffprofile.anchor(node_id=info["node_id"])

    # ------------------------------------------------------------------
    # クイック較正(全モード対応。Multi は対象機体を指定)
    # ------------------------------------------------------------------

    def _quickcal_target(self, drone) -> tuple[Optional[int], Optional[str]]:
        """quickcal の対象解決+飛行中ガード。(node_id, 拒否理由) を返す。

        単機モード: drone は無視し、phase が armed/flying なら拒否。
        Multi モード: drone(機体名)必須。_multi_ff_slot と同じ規範で
        **いずれかの機体が飛行中なら全面拒否**する(yaw_zero シーケンスは
        _command_lock を数秒〜10秒保持し得るため、飛行中に stop() の再送監視が
        ロック待ちでブロックされる事故を防ぐ)。
        """
        with self._lock:
            mode = self._mode
            phase = self._phase
        if mode == MODE_MULTI:
            if not drone:
                return None, ("複数機モード中は対象機体(drone)を"
                              "指定してください")
            if self.multi.any_armed_or_flying():
                return None, ("飛行中の機体があるためクイック較正できません"
                              "(全機着陸後にやり直してください)")
            info = self.multi.slot_info(str(drone))
            if info is None:
                return None, f"機体「{drone}」は選択されていません"
            return info["node_id"], None
        # 単機モード: drone は無視(ffprofile の "drone" 方式に合わせた契約)
        if phase in (PHASE_ARMED, PHASE_FLYING):
            return None, ("飛行中はクイック較正できません"
                          "(着陸後にやり直してください)")
        return None, None

    @_ui_command
    def quickcal(self, action: str, drone=None) -> dict:
        """クイック較正(Attitude 0 / Attitude Clear / Yaw 0 / Yaw Clear)。

        全制御モード(Posture/Position/Multi/Experiment)で使用可能。
        Multi モード中は drone(選択適用済みの機体名)でノード宛に送る。
        既存ガード(モーター回転中・スイープ/シーケンス/計測中・磁気鮮度)は
        CalibrationManager 側で維持される(experiment 外では自然に no-op)。
        """
        handlers = {
            "attitude_zero": self.calibration.attitude_zero,
            "attitude_clear": self.calibration.attitude_zero_clear,
            "yaw_zero": self.calibration.yaw_zero,
            "yaw_clear": self.calibration.yaw_zero_clear,
        }
        handler = handlers.get(action)
        if handler is None:
            return {"ok": False, "message": "不明な action です"}
        node_id, reason = self._quickcal_target(drone)
        if reason is not None:
            self.warn(f"クイック較正不可: {reason}")
            return {"ok": False, "message": reason}
        return handler(node_id=node_id)

    def mocap_bodies(self) -> dict:
        """現在観測中の全リジッドボディ一覧(紐付け確認 UI 用)。

        NatNet 未接続なら接続を試みる(primary コールバックは単機 Position
        モード専用のため no-op で起動する。既に起動済みなら何もしない)。
        """
        connected = self.mocap.connected()
        if not connected:
            connected = self.mocap.start()   # パッシブ起動(primary は触らない)
        return {
            "connected": bool(connected),
            "bodies": _json_safe(self.mocap.bodies_snapshot()),
        }

    # ------------------------------------------------------------------
    # MoCap マッピング(設定タブ: 座標変換・姿勢の実行時変更)
    # ------------------------------------------------------------------

    def _mapping_apply_blocked(self) -> Optional[str]:
        """マッピング変更を拒否すべき状態なら理由を返す(地上限定ガード)。

        座標変換の変更は位置誤差とヨーの両方を不連続にステップさせる。
        飛行中に許すと符号変更が閉ループの極性反転(正帰還)になり、
        発散フェイルセーフが効くまで機体が加速し続けるため、単機・
        複数機のいずれかが armed/flying の間は一切受け付けない。
        ログ記録中も拒否する(1ファイル内に2つの座標系が混ざり、
        サイドカー meta.json の記録と矛盾するため)。
        """
        with self._lock:
            phase = self._phase
        if phase in (PHASE_ARMED, PHASE_FLYING):
            return "飛行中は MoCap マッピングを変更できません。着陸後にやり直してください"
        if self.multi.any_armed_or_flying():
            return ("複数機制御で飛行中のため MoCap マッピングを変更できません。"
                    "全機着陸後にやり直してください")
        # 単機ロガーに加えて複数機の機体別ログも検査する(全機着陸検知〜
        # supervise がログを閉じるまでの短い窓でもファイルは開いている)
        if self.logger.active or self.multi.logging_active:
            return "ログ記録中は MoCap マッピングを変更できません"
        return None

    def mocap_mapping(self) -> dict:
        """適用中のマッピングと適用可否(設定タブの GET 用)。"""
        blocked = self._mapping_apply_blocked()
        mapping = self.mocap.mapping_snapshot()
        return {
            "ok": True,
            "mapping": mapping,
            "can_apply": blocked is None,
            "blocked_reason": blocked,
            "primary_rigid_body_id":
                self.control_config["natnet"]["rigid_body_id"],
        }

    @_ui_command
    def update_mocap_mapping(self, payload) -> dict:
        """MoCap マッピング(座標変換+姿勢)を検証・適用・永続化する。

        受理条件: validate_mapping の厳格検証(符号付き置換など)を通過し、
        かつ地上(非武装・非記録中)であること。受理時は
        (1) control.json へ原子的保存 → (2) メモリの control_config 同期 →
        (3) MocapSource へアトミック差し替え → (4) 位置フィルタ再初期化
        (単機+複数機全スロット)の順で適用する。保存に失敗した場合は
        適用もしない(ファイル・メモリ・適用状態の三者一致を維持)。
        """
        if not isinstance(payload, dict):
            return {"ok": False, "message": "リクエスト形式が不正です"}
        transform_cfg = payload.get("coordinate_transform")
        attitude_cfg = payload.get("attitude_transform") or {}
        error = validate_mapping(transform_cfg, attitude_cfg)
        if error is not None:
            return {"ok": False, "message": error}
        blocked = self._mapping_apply_blocked()
        if blocked is not None:
            self.warn(f"MoCap マッピング変更不可: {blocked}")
            return {"ok": False, "message": blocked}

        # 正規化(欠損キーへの既定値充填)してから保存・適用する
        merged_attitude = dict(DEFAULT_ATTITUDE_TRANSFORM)
        merged_attitude.update(attitude_cfg)
        normalized_transform = {
            axis: {"axis": transform_cfg[axis]["axis"],
                   "sign": transform_cfg[axis]["sign"]}
            for axis in ("x", "y", "z")
        }

        # 保存はコピーで先に行い、失敗時はメモリ・適用状態を一切変えない。
        # 成功後のメモリ反映は「同一 dict への in-place 更新」とする
        # (MultiControlManager が control_config の生参照を保持しており、
        # dict ごと差し替えると参照が分裂するため)。
        old_transform = self.control_config["coordinate_transform"]
        old_attitude = self.control_config.get(
            "attitude_transform", dict(DEFAULT_ATTITUDE_TRANSFORM))
        new_config = dict(self.control_config)
        new_config["coordinate_transform"] = normalized_transform
        new_config["attitude_transform"] = merged_attitude
        try:
            cfg.save_control_config(new_config)
        except OSError as exc:
            return {"ok": False,
                    "message": f"control.json の保存に失敗しました: {exc}"}
        self._apply_mapping(normalized_transform, merged_attitude)

        # 適用後の再検査: 地上ガードは RX スレッドの飛行昇格(TLM による
        # CONNECTED→FLYING 等)とは非同期のため、チェック→適用の間に
        # 昇格が起き得る。ここで再検査し、昇格していたら旧マッピングへ
        # 巻き戻す(ファイルも戻す)。巻き戻し後の飛行は従来どおりの
        # 座標系で継続する。
        blocked = self._mapping_apply_blocked()
        if blocked is not None:
            try:
                revert_config = dict(self.control_config)
                revert_config["coordinate_transform"] = old_transform
                revert_config["attitude_transform"] = old_attitude
                cfg.save_control_config(revert_config)
            except OSError:
                pass   # ファイルが新設定のまま残っても適用状態は旧に戻す
            self._apply_mapping(old_transform, old_attitude)
            self.warn(f"MoCap マッピング変更を取り消しました: {blocked}")
            return {"ok": False,
                    "message": f"適用と並行して状態が変化したため取り消しました: {blocked}"}

        self.info("MoCap マッピングを更新しました(位置フィルタ再初期化・"
                  "目標リセット)")
        return self.mocap_mapping()

    def _apply_mapping(self, transform: dict, attitude: dict) -> None:
        """マッピング一式をメモリ・MocapSource・下流状態へ適用する。

        _command_lock 内(update_mocap_mapping)からのみ呼ぶ。順序:
        (1) メモリ設定を in-place 更新 → (2) MocapSource へアトミック差し替え
        → (3) 世代フロア設定+位置フィルタ再初期化(単機+複数機全スロット。
        差し替え直前に旧マッピングで計算中だったフレームのシード防止)→
        (4) 旧座標系の絶対値が残る状態のリセット(単機目標を既定へ、
        円軌道解除、複数機スロット目標のクリア)。
        """
        self.control_config["coordinate_transform"] = transform
        self.control_config["attitude_transform"] = attitude
        self.mocap.set_mapping(transform, attitude)
        floor = self.mocap.mapping_generation
        self.position.set_mocap_mapping_floor(floor)
        self.position.reset_filter()
        self.position.stop_circle()
        target_default = self.control_config["target_default"]
        self.position.set_target(target_default["x"], target_default["y"],
                                 target_default["z"])
        self.multi.reset_for_mapping_change(floor)

    # ------------------------------------------------------------------
    # v2: モーターテスト(Experiment タブ)
    # ------------------------------------------------------------------

    def _experiment_ready(self) -> Optional[str]:
        """モーター操作の前提チェック。問題があれば理由を返す。"""
        with self._lock:
            mode = self._mode
            active = self._experiment_active
        if mode != MODE_EXPERIMENT:
            return "実験モードではありません"
        if not self.serial.is_connected:
            return "未接続です"
        if not active:
            return "実験モードが有効化されていません(再有効化してください)"
        return None

    @_ui_command
    def motor_start(self, duty: float, mask: int) -> dict:
        reason = self._experiment_ready()
        if reason is not None:
            self.warn(f"モーター開始不可: {reason}")
            return {"ok": False, "message": reason}
        result = self.experiment.motor_start(duty, mask)
        if not result.get("ok") and result.get("message"):
            # hub 側の拒否(スイープ/シーケンス実行中など)も UI へ通知する
            self.warn(f"モーター開始不可: {result['message']}")
        return result

    @_ui_command
    def motor_apply(self, duty: float) -> dict:
        reason = self._experiment_ready()
        if reason is not None:
            self.warn(f"duty 変更不可: {reason}")
            return {"ok": False, "message": reason}
        result = self.experiment.motor_apply(duty)
        if not result.get("ok") and result.get("message"):
            self.warn(f"duty 変更不可: {result['message']}")
        return result

    def motor_stop(self) -> dict:
        # 停止は前提チェックなしで常に受け付ける(SPACE 緊急停止経路)。
        # 意図的に _ui_command(_command_lock)を経由しない: 低速コマンドの
        # 背後で緊急停止が待たされないよう、hub 内部ロックのみで完結させる
        # (キープアライブ停止 → CMD_MOTOR_STOP 冗長送出)。
        if not self.serial.is_connected:
            return {"ok": False, "message": "未接続です"}
        return self.experiment.motor_stop()

    def sweep_start(self, mask, pattern, notes) -> dict:
        reason = self._experiment_ready()
        if reason is not None:
            return {"ok": False, "message": reason,
                    **self.experiment.sweep.status()}
        return self.experiment.sweep.start(mask, pattern=pattern, notes=notes)

    def sequence_start(self, masks, pattern, notes, min_start_vbat) -> dict:
        reason = self._experiment_ready()
        if reason is not None:
            return {"ok": False, "message": reason,
                    **self.experiment.sequence.status()}
        return self.experiment.sequence.start(
            masks, pattern=pattern, notes=notes, min_start_vbat=min_start_vbat)

    @_ui_command
    def exp_record_start(self) -> dict:
        """実験計測ログ(EKF/FF 性能ログ)の開始(T1-6)。

        mode==experiment かつ MOTOR_TEST 有効のときのみ。スイープ/シーケンス
        との排他は ExpRecorder.start が start_gate 下で最終判定する。
        """
        reason = self._experiment_ready()
        if reason is not None:
            self.warn(f"計測開始不可: {reason}")
            return {"ok": False, "message": reason,
                    **self.experiment.recorder.status()}
        result = self.experiment.recorder.start()
        if result.get("ok"):
            self.info(f"計測開始: {result.get('file')}")
        elif result.get("message"):
            self.warn(f"計測開始不可: {result['message']}")
        return result

    @_ui_command
    def exp_record_stop(self) -> dict:
        """実験計測ログの手動停止(meta は aborted=false)。

        前提チェックなしで常に受け付ける(モード離脱後などの取り残しも
        確実に閉じられるように。未計測なら ok=False で無害)。
        """
        result = self.experiment.recorder.stop()
        if result.get("ok"):
            self.info(f"計測終了: {result.get('file')}"
                      f"({result.get('samples')}サンプル)")
        elif result.get("message"):
            self.warn(f"計測停止不可: {result['message']}")
        return result

    def _ff_state_snapshot(self) -> Optional[dict]:
        """FF 適用状態の抜粋(ExpRecorder の meta 用。取れなければ None)。"""
        applied = self.ffprofile.status().get("applied")
        if not isinstance(applied, dict):
            return None
        return {"profile": applied.get("name"),
                "ff_mode": applied.get("ff"),
                "est_mode": applied.get("est")}

    def shutdown(self) -> None:
        """サーバ終了時の後始末(Ctrl-C の lifespan 経路)。

        _command_lock が長時間保持されていても(FF 転送は数十秒保持し得る)
        プロセス終了を無期限に待たせない: タイムアウト付きでロックを取得し、
        取れなければ構成要素を直接(各自の有界な停止処理で)畳む。
        """
        acquired = self._command_lock.acquire(timeout=_SHUTDOWN_LOCK_TIMEOUT_S)
        try:
            if acquired:
                self.disconnect()   # RLock 再入のため通常経路のまま
            else:
                self.warn("コマンド実行中のため強制シャットダウンします")
                self._supervisor_stop.set()
                self._stop_active_sender()
                self._finish_flight_log()
                self.serial.disconnect()
        finally:
            if acquired:
                self._command_lock.release()
        # パッシブ起動(mocap_bodies)の NatNet はどの teardown 経路にも
        # 属さないため、終了時に必ず畳む(daemon 化済みだが行儀として)
        self.mocap.shutdown()
        self.experiment.shutdown()

    # ==================================================================
    # 内部: 接続まわり
    # ==================================================================

    def _apply_airframe(self, profile: dict) -> None:
        with self._lock:
            self._airframe = profile
            # バイアス(deg)→ rad。送信時に roll/pitch 指令へ加算される。
            self._bias_roll_rad = profile["roll_bias_deg"] * DEG_TO_RAD
            self._bias_pitch_rad = profile["pitch_bias_deg"] * DEG_TO_RAD
            self._relay_target_ok = False
        self.posture.set_default_alt(profile["default_alt_m"])

    def _configure_relay_target(self, profile: dict,
                                quiet_ok: bool = False) -> None:
        """RLY_SET_TARGET を送信して ACK を確認する。

        quiet_ok=True で成功時の info を抑制する(テレメトリ途絶時の
        周期的な自動再設定がログを埋めないように。失敗警告は常に出す)。
        """
        try:
            mac = cfg.parse_mac(profile["mac"])
            ok, ack = self.serial.set_relay_target(mac, profile["wifi_channel"])
        except (SerialLinkError, ValueError) as exc:
            self.warn(f"RLY_SET_TARGET 失敗: {exc}")
            ok, ack = False, None
        with self._lock:
            self._relay_target_ok = ok
        if ok:
            if not quiet_ok:
                self.info(f"リレーターゲット設定完了: {profile['mac']} "
                          f"ch{profile['wifi_channel']}")
        else:
            status = ack.status if ack is not None else "no_ack"
            self.warn(f"リレーターゲット設定失敗(status={status})。"
                      "ドローン宛コマンドは転送されません")

    def _start_active_sender(self) -> None:
        with self._lock:
            mode = self._mode
        if mode == MODE_POSITION:
            if not self.mocap.connected():
                if not self.mocap.start(self.position.on_mocap_pose):
                    self.warn("NatNet 接続に失敗しました")
            self.position.start()
        else:
            self.posture.start()

    def _stop_active_sender(self) -> None:
        self.posture.stop()
        self.position.stop()
        self.mocap.shutdown()

    def _open_flight_log(self) -> None:
        """単機の飛行ログファイルを開く(START 受理時/飛行中のログ ON)。"""
        with self._lock:
            mode = self._mode
        path = self.logger.start(mode, metadata=self._log_metadata())
        self.info(f"ログ開始: {path.name}")

    def _log_metadata(self) -> dict:
        """飛行ログのサイドカー(*.meta.json)に残すマッピング情報。

        CSV の位置・ヨー列がどの座標マッピングで記録されたかを恒久記録
        する(マッピングが実行時変更可能になったため、列契約だけでは
        フレームが自明でなくなった)。
        """
        return {
            "log_columns_version": 5,
            "mocap_mapping": self.mocap.mapping_snapshot(),
        }

    def _finish_flight_log(self) -> None:
        """飛行ログを閉じ、予約フラグを自動 OFF する(飛行終了の一元フック)。

        armed/flying → connected へ戻る全経路(着陸、START 猶予切れ、
        START 拒否)と teardown(切断/シリアル断/シャットダウン)、および
        複数機モードの全機着陸検知(supervise)から呼ぶ。
        RX スレッドからも安全(logger.stop() は内部ロックのみ、
        events.put はブロックしない)。複数機(T3-5)の機体別ロガーも
        ここで閉じる(このフックに集約)。
        """
        path = self.logger.file_path
        self.logger.stop()
        multi_names = self.multi.close_flight_logs()
        with self._lock:
            self._logging_enabled = False
        if path is not None:
            self.info(f"ログ終了: {path.name}(自動OFF)")
        for name in multi_names:
            self.info(f"ログ終了: {name}(自動OFF)")

    def _teardown_session(self, message: str) -> None:
        self._stop_active_sender()
        # ログ(単機+複数機の機体別)を閉じ、予約フラグも下ろす。
        # multi.deactivate より先に呼ぶ(ファイル名つきの終了通知のため。
        # 閉じた後の 50Hz 送信スレッドの log_row は無視される)
        self._finish_flight_log()
        # 複数機スロットを解放(シリアルはこの後閉じるためピア解除は送らない。
        # 各機体はセットポイント 500ms 途絶で自動着陸する)
        self.multi.deactivate(clear_peers=False)
        # 実験機能を停止(シリアルはこの後閉じるため CMD_MODE は送らない。
        # 機体側はモーターコマンド 1.5s 途絶で自動停止する)
        self.experiment.deactivate()
        self.serial.disconnect()
        with self._lock:
            self._phase = PHASE_IDLE
            self._stop_pending = None
            self._relay_target_ok = False
            self._armed_since = None
            self._mocap_warned = False
            self._mocap_stop_sent = False
            self._data_invalid_warned = False
            self._data_invalid_stop_sent = False
            self._error_high_since = None
            self._experiment_active = False
            self._connected_at = None
            self._tlm_stale_warned = False
            self._relay_reassert_at = None
        self.position.set_control_active(False)
        self.info(message)

    def _on_serial_disconnect(self, reason: str) -> None:
        """SerialLink からの切断通知(RX/TXスレッド上: フラグのみ立てる)。"""
        with self._lock:
            self._link_lost = True
        self.warn(f"シリアル切断: {reason}")

    # ==================================================================
    # 内部: 送信(50Hz スレッドから。バイアス加算・送信・ログ)
    # ==================================================================

    def _emit_setpoint(self, roll_rad: float, pitch_rad: float, alt_m: float,
                       meta: dict) -> None:
        """整形済みセットポイントにバイアスを加算し、送信してログする。

        v2: ヨー目標は meta("yaw_ref_rad" / "yaw_ctrl_on")で受け取り、
        ON のとき flags bit1(FLAG_YAW_REF_VALID)を立てて 17B で送信する。
        OFF のときは yaw_ref=0 / bit1=0(機体は V1 と同一のレートダンピング)。

        v2.2: Position モードは常に位置誤差(CMD_POS_ERR)を送る
        (_emit_pos_err。機体側でヨー回転補償+XY PID)。
        CMD_SETPOINT は Posture モード専用。
        """
        with self._lock:
            bias_roll = self._bias_roll_rad
            bias_pitch = self._bias_pitch_rad
            phase = self._phase
        if meta.get("mode") == "position":
            self._emit_pos_err(alt_m, meta, phase)
            return
        yaw_ctrl_on = bool(meta.get("yaw_ctrl_on"))
        yaw_ref = float(meta.get("yaw_ref_rad") or 0.0) if yaw_ctrl_on else 0.0
        flags = proto.CmdSetpoint.FLAG_ALT_REF_VALID
        if yaw_ctrl_on:
            flags |= proto.CmdSetpoint.FLAG_YAW_REF_VALID
        setpoint = proto.CmdSetpoint(
            roll_ref=roll_rad + bias_roll,
            pitch_ref=pitch_rad + bias_pitch,
            alt_ref=alt_m,
            yaw_ref=yaw_ref,
            flags=flags,
        )
        seq: Optional[int] = None
        send_success = False
        try:
            seq = self.serial.send_setpoint(setpoint)
            send_success = True
        except SerialLinkError:
            pass   # 切断検知は _on_serial_disconnect → supervisor が処理

        if self.logger.active:
            row = self._build_log_row(setpoint, seq, send_success, phase, meta)
            self.logger.log_row(row)

    def _emit_pos_err(self, alt_m: float, meta: dict, phase: str) -> None:
        """機上XY制御: CMD_POS_ERR(位置誤差ストリーム)を送信してログする。

        roll/pitch 指令は機体側 XY PID が計算するため送らない(トリム
        バイアスも送らない — 定常オフセットは機体側 I 項が吸収する)。
        bit2(XY_ERR_VALID)は「閉ループ有効 かつ データ有効 かつ MoCap
        非途絶」。無効時も**実誤差(クランプ後)をそのまま送る**: 機体側が
        bit2=0 で水平指令+PID減衰を行い、PID の誤差履歴(prev_error)は
        現実を追い続ける(誤差を 0 にすり替えると復帰1サンプル目に
        D 項スパイクを作る)。alt_ref / yaw_ref の整形は既存の
        SetpointShaper 出力(呼び出し元)を使う。
        """
        yaw_ctrl_on = bool(meta.get("yaw_ctrl_on"))
        yaw_ref = float(meta.get("yaw_ref_rad") or 0.0) if yaw_ctrl_on else 0.0
        xy_valid = (bool(meta.get("control_active"))
                    and bool(meta.get("data_valid"))
                    and not bool(meta.get("mocap_dropout")))
        clamp = self._pos_err_clamp_m
        err_x = max(-clamp, min(clamp, float(meta.get("error_x") or 0.0)))
        err_y = max(-clamp, min(clamp, float(meta.get("error_y") or 0.0)))
        heading = meta.get("mocap_heading_rad")
        flags = proto.CmdPosErr.FLAG_ALT_REF_VALID
        if yaw_ctrl_on:
            flags |= proto.CmdPosErr.FLAG_YAW_REF_VALID
        if xy_valid:
            flags |= proto.CmdPosErr.FLAG_XY_ERR_VALID
        if heading is not None:
            flags |= proto.CmdPosErr.FLAG_MOCAP_YAW_VALID
        pos_err = proto.CmdPosErr(
            err_x=err_x, err_y=err_y, alt_ref=alt_m, yaw_ref=yaw_ref,
            mocap_yaw=float(heading or 0.0), flags=flags)
        seq: Optional[int] = None
        send_success = False
        try:
            seq = self.serial.send_pos_err(pos_err)
            send_success = True
        except SerialLinkError:
            pass   # 切断検知は _on_serial_disconnect → supervisor が処理

        if self.logger.active:
            # 共有列(alt/yaw 等)はゼロ姿勢の CmdSetpoint 形で埋める
            # (roll/pitch 指令は機体側で計算され、tlm_roll_ref_rad /
            # tlm_pitch_ref_rad 列に現れる)。
            row = self._build_log_row(
                proto.CmdSetpoint(roll_ref=0.0, pitch_ref=0.0, alt_ref=alt_m,
                                  yaw_ref=yaw_ref, flags=flags & 0x03),
                seq, send_success, phase, meta)
            row["cmd_err_x_m"] = err_x
            row["cmd_err_y_m"] = err_y
            row["cmd_xy_valid"] = xy_valid
            self.logger.log_row(row)

    def _build_log_row(self, setpoint: proto.CmdSetpoint, seq: Optional[int],
                       send_success: bool, phase: str, meta: dict) -> dict:
        row = {
            "mode": meta.get("mode"),
            "phase": phase,
            "command_sequence": seq,
            "send_success": send_success,
            "feedback_latency_ms": self.serial.latency_ms,
            "roll_ref_rad": setpoint.roll_ref,
            "pitch_ref_rad": setpoint.pitch_ref,
            "alt_ref_m": setpoint.alt_ref,
            "roll_bias_deg": self._bias_roll_rad * RAD_TO_DEG,
            "pitch_bias_deg": self._bias_pitch_rad * RAD_TO_DEG,
            # v2: ヨー指令(送信したヨー目標)
            "cmd_yaw_ref_rad": setpoint.yaw_ref,
            "yaw_ctrl_on": bool(setpoint.flags
                                & proto.CmdSetpoint.FLAG_YAW_REF_VALID),
        }

        # Position モードの診断列(meta 由来。logger の共通ヘルパで転記)
        row.update(logger_mod.meta_to_row(meta))

        # 最新テレメトリのスナップショット(TLM_STATE / TLM_CTRL とも 25Hz)
        with self._lock:
            tlm = self._tlm_state
            tlm_t = self._tlm_state_t
            ctrl = self._tlm_ctrl
            ctrl_t = self._tlm_ctrl_t
        if tlm is not None and tlm_t is not None:
            row.update(logger_mod.tlm_state_to_row(tlm, self._clock() - tlm_t))
        if ctrl is not None and ctrl_t is not None:
            row.update(logger_mod.tlm_ctrl_to_row(ctrl, self._clock() - ctrl_t))
        return row

    # ==================================================================
    # 内部: 受信ハンドラ(RXスレッド上: 状態更新とキュー投入のみ)
    # ==================================================================

    def _tlm_state_snapshot(self, node_id: Optional[int] = None
                            ) -> tuple[Optional[proto.TlmState],
                                       Optional[float]]:
        """最新 TLM_STATE と受信からの経過秒(CalibrationManager が参照)。

        node_id 指定時は複数機モードの当該スロットのスナップショットを返す
        (quickcal のノード宛対応。未選択/未受信は (None, None))。
        """
        if node_id is None:
            with self._lock:
                tlm = self._tlm_state
                tlm_t = self._tlm_state_t
        else:
            tlm, tlm_t = self.multi.tlm_state_of(node_id)
        age = None if tlm_t is None else (self._clock() - tlm_t)
        return tlm, age

    def _on_tlm_state(self, frame: proto.Frame) -> None:
        try:
            tlm = proto.TlmState.from_payload(frame.payload)
        except ValueError:
            return
        with self._lock:
            self._tlm_state = tlm
            self._tlm_state_t = self._clock()
        self._update_phase_from_drone(tlm.state, tlm.flags)

    def _on_tlm_ctrl(self, frame: proto.Frame) -> None:
        """TLM_CTRL(制御ループ診断 25Hz)の最新値保持(RXスレッド上)。"""
        try:
            ctrl = proto.TlmCtrl.from_payload(frame.payload)
        except ValueError:
            return
        with self._lock:
            self._tlm_ctrl = ctrl
            self._tlm_ctrl_t = self._clock()

    def _on_tlm_event(self, frame: proto.Frame) -> None:
        try:
            event = proto.TlmEvent.from_payload(frame.payload)
        except ValueError:
            return
        # STOP 再送待ちの解除(LANDING / WAIT イベント)
        if event.state in (proto.FlightState.LANDING, proto.FlightState.WAIT):
            with self._lock:
                self._stop_pending = None
        # START 拒否 → armed を解除(飛行終了扱い: ログも閉じる)
        if event.reason in _START_REJECT_REASONS:
            disarmed = False
            with self._lock:
                if self._phase == PHASE_ARMED:
                    self._phase = PHASE_CONNECTED
                    self._armed_since = None
                    disarmed = True
            if disarmed:
                self._finish_flight_log()
            self.warn(f"離陸拒否: {_reason_name(event.reason)}")
        self._update_phase_from_drone(event.state, 0)
        self.events.put(_json_safe({
            "type": "event",
            "data": {
                "state": event.state,
                "state_name": _state_name(event.state),
                "prev_state": event.prev_state,
                "prev_state_name": _state_name(event.prev_state),
                "reason": event.reason,
                "reason_name": _reason_name(event.reason),
                "flags": event.flags,
                "voltage": event.voltage,
            },
        }))

    def _on_log_text(self, frame: proto.Frame) -> None:
        try:
            log = proto.LogText.from_payload(frame.payload)
        except ValueError:
            return
        origin = LOG_ORIGIN_NAMES.get(log.origin, f"origin{log.origin}")
        self.events.put({"type": "log", "origin": origin, "line": log.text})

    def _on_rly_stats(self, frame: proto.Frame) -> None:
        try:
            stats = proto.RlyStats.from_payload(frame.payload)
        except ValueError:
            return
        with self._lock:
            self._rly_stats = stats
            self._rly_stats_t = self._clock()

    def _on_tlm_exp(self, frame: proto.Frame) -> None:
        """TLM_EXP(実験テレメトリ)→ ExperimentHub へ(RXスレッド上)。"""
        try:
            tlm = proto.TlmExp.from_payload(frame.payload)
        except ValueError:
            return
        self.experiment.on_tlm_exp(tlm, frame.seq)

    def _on_tlm_cal_data(self, frame: proto.Frame) -> None:
        """TLM_CAL_DATA → CalibrationManager へ(RXスレッド上)。"""
        try:
            cal = proto.TlmCalData.from_payload(frame.payload)
        except ValueError:
            return
        self.calibration.on_cal_data(cal)

    def _on_tlm_cal_data_node(self, node_id: int, frame: proto.Frame) -> None:
        """ノード帰属つき TLM_CAL_DATA(MUX_DOWN 経由。RXスレッド上)。"""
        try:
            cal = proto.TlmCalData.from_payload(frame.payload)
        except ValueError:
            return
        self.calibration.on_cal_data(cal, node_id=node_id)

    def _update_phase_from_drone(self, state: int, flags: int) -> None:
        """ドローンの報告状態から armed/flying/connected フェーズを更新する。

        armed/flying → connected へ戻る遷移(着陸・START 猶予切れ)は
        「飛行終了」の一元検知点でもあり、_finish_flight_log を呼ぶ
        (飛行ログの寿命 = START〜飛行終了の閉じ側)。
        """
        flying_flag = bool(flags & proto.TlmState.FLAG_FLYING)
        in_flight = flying_flag or state in _IN_FLIGHT_STATES
        flight_ended = False
        with self._lock:
            if self._phase == PHASE_ARMED:
                if in_flight:
                    self._phase = PHASE_FLYING
                    self._armed_since = None
                elif (state in _ON_GROUND_STATES
                      and self._armed_since is not None
                      and self._clock() - self._armed_since > self._start_grace_s):
                    # START 後も離陸へ遷移しない → 受理されなかったとみなす
                    self._phase = PHASE_CONNECTED
                    self._armed_since = None
                    flight_ended = True
            elif self._phase == PHASE_FLYING and not in_flight \
                    and state in _ON_GROUND_STATES:
                self._phase = PHASE_CONNECTED
                flight_ended = True
            elif self._phase == PHASE_CONNECTED and in_flight:
                # 接続先の機体が既に飛行中(例: ホバリング中に PC サーバを再起動
                # して再接続)。connected のままだと飛行ガード(モード/プロファイル
                # 変更・start の拒否)が素通りするため flying へ昇格する。
                # START 猶予(_armed_since)の判定は armed フェーズ限定なので
                # この昇格とは干渉しない。
                self._phase = PHASE_FLYING
        if flight_ended:
            self.position.set_control_active(False)
            self._finish_flight_log()

    # ==================================================================
    # 内部: 監視スレッド(STOP再送 / MoCap途絶 / リンク断)
    # ==================================================================

    def _supervisor_loop(self) -> None:
        run_paced_loop(self._supervisor_stop, self._clock,
                       self._supervisor_period_s, self.supervise)

    def supervise(self, now: float) -> None:
        """フェイルセーフ監視の1周期(テストから直接呼べる)。"""
        # --- シリアル切断: セッションを安全に畳む ---
        with self._lock:
            link_lost = self._link_lost
        if link_lost:
            with self._lock:
                self._link_lost = False
            self._supervisor_stop.set()
            # UI コマンドと teardown が交錯しないよう直列化する
            with self._command_lock:
                self._teardown_session("シリアル切断によりセッションを終了しました")
            return

        # --- STOP 再送(600ms 以内に LANDING/WAIT イベントなし) ---
        resend_stop = False
        with self._lock:
            pending = self._stop_pending
            if pending is not None and now >= pending["deadline"]:
                if pending["resends"] < self._stop_max_retries:
                    pending["resends"] += 1
                    pending["deadline"] = now + self._stop_ack_timeout_s
                    resend_stop = True
                    resend_count = pending["resends"]
                else:
                    self._stop_pending = None
                    self._warn_locked("CMD_STOP への応答がありません(再送上限到達)")
        if resend_stop:
            self.warn(f"CMD_STOP 応答なし → 再送 "
                      f"({resend_count}/{self._stop_max_retries})")
            self._send_stop()

        # --- MoCap 途絶ポリシー(Position モード) ---
        with self._lock:
            mode = self._mode
            phase = self._phase
        if mode == MODE_POSITION and phase in (PHASE_ARMED, PHASE_FLYING):
            age = self.position.mocap_age_s(now)
            dropped = age is None or age > self._mocap_dropout_level_s
            warn_dropout = False
            send_stop = False
            with self._lock:
                if dropped:
                    if not self._mocap_warned:
                        self._mocap_warned = True
                        warn_dropout = True
                    if (age is None or age > self._mocap_dropout_stop_s) \
                            and not self._mocap_stop_sent:
                        self._mocap_stop_sent = True
                        send_stop = True
                else:
                    self._mocap_warned = False
                    self._mocap_stop_sent = False
            if warn_dropout:
                self.warn("MoCap 途絶 >300ms: セットポイントを水平に固定します")
            if send_stop:
                self.warn("MoCap 途絶 >2s: CMD_STOP を送信します(自動着陸)")
                self.stop()

            # --- データ無効の持続(受信はあるが data_valid=0 が続く)---
            # トラッキング喪失・外れ値・低信頼度の間、on_mocap_pose は
            # roll/pitch を水平へ強制している(位置閉ループは事実上停止)。
            # フレームは届き続けるため上の途絶ポリシーでは検出できない。
            # 途絶中は判定しない(無効時間は途絶時間を包含するため二重警告
            # になる。途絶側のポリシーに委ねる)
            if not dropped:
                invalid_age = self.position.data_invalid_age_s(now)
                warn_invalid = False
                send_stop_invalid = False
                with self._lock:
                    if (invalid_age is not None
                            and invalid_age > self._data_invalid_warn_s):
                        if not self._data_invalid_warned:
                            self._data_invalid_warned = True
                            warn_invalid = True
                        if invalid_age > self._data_invalid_stop_s \
                                and not self._data_invalid_stop_sent:
                            self._data_invalid_stop_sent = True
                            send_stop_invalid = True
                    else:
                        self._data_invalid_warned = False
                        self._data_invalid_stop_sent = False
                if warn_invalid:
                    self.warn(
                        f"MoCap 位置データ無効 >{self._data_invalid_warn_s:.1f}s"
                        "(トラッキング喪失/外れ値): セットポイントを水平に"
                        "固定しています")
                if send_stop_invalid:
                    self.warn(
                        f"MoCap 位置データ無効 >{self._data_invalid_stop_s:.0f}s: "
                        "CMD_STOP を送信します(自動着陸)")
                    self.stop()

            # --- XY 誤差の持続的発散(flying かつ閉ループ中のみ) ---
            # multi.supervise の発散検知と同じ規範(単機版)。RB 取り違えや
            # 偽ソース(反射)への再シードで、フィルタ上は「有効」なまま
            # 閉ループが幻の誤差を追い続ける事故の最終防衛線。途絶中は誤差が
            # 古いため判定しない(途絶フェイルセーフに委ねる)。
            error_m = self.position.xy_error_m()
            control_active = self.position.control_active
            diverged = False
            with self._lock:
                if (phase == PHASE_FLYING and control_active and not dropped
                        and error_m is not None
                        and error_m > self._divergence_error_m):
                    if self._error_high_since is None:
                        self._error_high_since = now
                    elif now - self._error_high_since > self._divergence_hold_s:
                        self._error_high_since = None
                        diverged = True
                else:
                    self._error_high_since = None
            if diverged:
                self.warn(
                    f"XY 位置誤差 >{self._divergence_error_m:.1f}m が "
                    f"{self._divergence_hold_s:.1f}s 継続: 発散とみなし "
                    "CMD_STOP を送信します(rigid_body_id の対応と"
                    "トラッキング状態を確認してください)")
                self.stop()

        # --- 計測中 LED キープアライブ(CMD_LED_MODE(1) の約1秒周期再送)---
        # 機体側は最後の mode=1 から3秒で通常表示へ自動復帰するため、計測中
        # (ExpRecorder)は supervisor に相乗りして再送し続ける(専用スレッド
        # なし。非計測時は即 return する軽量チェックのみ)。
        self.experiment.recorder.service_led_keepalive(now)

        # --- 単機テレメトリ途絶(リレー再起動等のサイレント断)---
        # リレーはターゲット設定を RAM のみに保持するため、リレーの再起動で
        # 「シリアルは生きているが機体との転送だけ死んだ」ゾンビ接続になり得る
        # (受信は空読み継続でリンク断とは検出されない)。TLM_STATE の途絶で
        # 検出し、警告 + RLY_SET_TARGET の自動再設定で自己修復を試みる。
        # 複数機モードはスロット別監視(multi.supervise)側の責務。
        self._supervise_tlm_staleness(now)

        # --- 複数機モード(スロットごとの STOP 再送 / MoCap 途絶 / 猶予) ---
        self.multi.supervise(now)

        # --- 複数機の飛行終了検知(T3-5): 全機着陸で全ログ close + 自動OFF ---
        # スロット phase は RX スレッド(TLM)と multi.supervise(途絶解放)
        # が更新するため、multi.supervise の後に判定する。stop_all /
        # 緊急停止後も着陸(全機 idle)を検知するまで記録は続く。
        if self.multi.logging_active and not self.multi.any_armed_or_flying():
            self._finish_flight_log()

    def _supervise_tlm_staleness(self, now: float) -> None:
        """単機経路のテレメトリ鮮度監視1周期(supervise から毎周期呼ばれる)。"""
        if not self.serial.is_connected or self.multi.active:
            return
        with self._lock:
            airframe = self._airframe
            tlm_t = self._tlm_state_t
            reference = tlm_t if tlm_t is not None else self._connected_at
            stale = (reference is not None
                     and now - reference > self._tlm_stale_s)
            can_reassert = (airframe is not None
                            and cfg.mac_is_set(airframe.get("mac")))
            warn_stale = stale and not self._tlm_stale_warned
            recovered = (not stale) and self._tlm_stale_warned
            if warn_stale:
                self._tlm_stale_warned = True
            if recovered:
                self._tlm_stale_warned = False
            reassert = (stale and can_reassert
                        and not self._relay_reassert_busy
                        and (self._relay_reassert_at is None
                             or now - self._relay_reassert_at
                             >= self._relay_reassert_interval_s))
            if reassert:
                self._relay_reassert_busy = True
                self._relay_reassert_at = now
        if warn_stale:
            if can_reassert:
                self.warn(f"機体テレメトリが {self._tlm_stale_s:.0f}s 以上"
                          "途絶しています。リレーターゲットを自動再設定します"
                          "(リレー再起動などで設定が失われた可能性。機体の"
                          "電源が入っていない場合はこの警告のままになります)")
            else:
                self.warn(f"機体テレメトリが {self._tlm_stale_s:.0f}s 以上"
                          "途絶しています(機体プロファイルが未選択のため"
                          "自動再設定はできません)")
        if recovered:
            self.info("機体テレメトリが回復しました")
        if reassert:
            # set_relay_target は ACK 待ちで最大約4秒ブロックするため、
            # supervisor 本体(STOP 再送等の監視)を止めない専用スレッドで行う
            threading.Thread(target=self._reassert_relay_target,
                             args=(airframe,),
                             name="relay-reassert", daemon=True).start()

    def _reassert_relay_target(self, airframe: dict) -> None:
        """テレメトリ途絶時の RLY_SET_TARGET 再送(supervisor 起点の別スレッド)。

        UI コマンド(connect / select_airframe)と交錯しないよう _command_lock を
        短いタイムアウト付きで取得し、取れなければ今回は見送る(次周期に再試行)。
        ロック待ちの間に機体選択が変わり得るため、取得後にスナップショットが
        現在の選択と同一であることを確認してから送る(古いターゲットへ
        黙って戻さない)。
        """
        acquired = self._command_lock.acquire(timeout=_REASSERT_LOCK_TIMEOUT_S)
        try:
            if acquired and self.serial.is_connected and not self.multi.active:
                with self._lock:
                    still_current = self._airframe is airframe
                if still_current:
                    self._configure_relay_target(airframe, quiet_ok=True)
        finally:
            if acquired:
                self._command_lock.release()
            with self._lock:
                self._relay_reassert_busy = False

    def _send_stop(self) -> bool:
        try:
            self.serial.send(proto.MsgType.CMD_STOP)
        except SerialLinkError as exc:
            self.warn(f"CMD_STOP 送信失敗: {exc}")
            return False
        self.info("CMD_STOP 送信")
        return True

    # ==================================================================
    # UI への通知 / スナップショット
    # ==================================================================

    def info(self, message: str) -> None:
        self.events.put({"type": "log", "origin": LOG_ORIGIN_SERVER,
                         "line": message})

    def warn(self, message: str) -> None:
        self.events.put({"type": "log", "origin": LOG_ORIGIN_SERVER,
                         "line": f"[警告] {message}"})

    def _warn_locked(self, message: str) -> None:
        """self._lock 保持中でも安全な警告(queue.put はブロックしない)。"""
        self.events.put({"type": "log", "origin": LOG_ORIGIN_SERVER,
                         "line": f"[警告] {message}"})

    def get_state_snapshot(self) -> dict:
        """WebSocket 20Hz 配信用の状態スナップショット(UI 単位系: deg/m)。"""
        now = self._clock()
        with self._lock:
            tlm = self._tlm_state
            tlm_t = self._tlm_state_t
            mode = self._mode
            phase = self._phase
            airframe = self._airframe
            logging_enabled = self._logging_enabled
            relay_target_ok = self._relay_target_ok
            rly_stats = self._rly_stats
            rly_stats_t = self._rly_stats_t
            mocap_warned = self._mocap_warned
            experiment_active = self._experiment_active

        # drone: TLM_STATE 全フィールド(角度・角速度は deg 換算)+ fresh。
        # 一度も受信していない間は null — ゼロ値を実測値として配ると UI 側で
        # 「INIT・0.00V(危険表示)」と本物の異常が区別できなくなるため。
        if tlm is None or tlm_t is None:
            drone = None
        else:
            drone = {
                "seq_echo": tlm.seq_echo,
                "elapsed_ms": tlm.elapsed_ms,
                "state": tlm.state,
                "state_name": _state_name(tlm.state),
                "flags": tlm.flags,
                "low_voltage": bool(tlm.flags & proto.TlmState.FLAG_LOW_VOLTAGE),
                "setpoint_fresh": bool(tlm.flags & proto.TlmState.FLAG_SETPOINT_FRESH),
                "flying": bool(tlm.flags & proto.TlmState.FLAG_FLYING),
                "reason": tlm.reason,
                "reason_name": _reason_name(tlm.reason),
                "roll": tlm.roll * RAD_TO_DEG,
                "pitch": tlm.pitch * RAD_TO_DEG,
                "yaw": tlm.yaw * RAD_TO_DEG,
                "p": tlm.p * RAD_TO_DEG,
                "q": tlm.q * RAD_TO_DEG,
                "r": tlm.r * RAD_TO_DEG,
                "roll_ref": tlm.roll_ref * RAD_TO_DEG,
                "pitch_ref": tlm.pitch_ref * RAD_TO_DEG,
                "alt_ref": tlm.alt_ref,
                "altitude_tof": tlm.altitude_tof,
                "altitude_est": tlm.altitude_est,
                "alt_velocity": tlm.alt_velocity,
                "z_dot_ref": tlm.z_dot_ref,
                "voltage": tlm.voltage,
                "duty_fr": tlm.duty_fr,
                "duty_fl": tlm.duty_fl,
                "duty_rr": tlm.duty_rr,
                "duty_rl": tlm.duty_rl,
                "ax": tlm.ax,
                "ay": tlm.ay,
                "az": tlm.az,
                "loop_dt_us": tlm.loop_dt_us,
                "fresh": (now - tlm_t) <= self._telemetry_fresh_s,
                # v2: ヨー推定/FF 診断(角度は deg 換算、UI 単位規約)
                "yaw_est": tlm.yaw_est_rad * RAD_TO_DEG,
                "yaw_gyro_int": tlm.yaw_gyro_int_rad * RAD_TO_DEG,
                "yaw_ref": tlm.yaw_ref_rad * RAD_TO_DEG,
                "current_a": tlm.current_a,
                "db_hat_x_ut": tlm.db_hat_x_ut,
                "db_hat_y_ut": tlm.db_hat_y_ut,
                "bm_x_ut": tlm.bm_x_ut,
                "bm_y_ut": tlm.bm_y_ut,
                "nis": tlm.nis,
                "ffg": tlm.ffg,
                "ff_status": tlm.ff_status,
                "ff_mode": tlm.ff_status & proto.TlmState.FF_STATUS_FF_MODE_MASK,
                "est_mode_ekf": bool(tlm.ff_status
                                     & proto.TlmState.FF_STATUS_EST_EKF),
                "anchor_valid": bool(tlm.ff_status
                                     & proto.TlmState.FF_STATUS_ANCHOR_VALID),
                "ffcal_loaded": bool(tlm.ff_status
                                     & proto.TlmState.FF_STATUS_FFCAL_LOADED),
                "yaw_ctrl_active": bool(
                    tlm.ff_status & proto.TlmState.FF_STATUS_YAW_CTRL_ACTIVE),
                "mag_fresh": bool(tlm.ff_status
                                  & proto.TlmState.FF_STATUS_MAG_FRESH),
            }

        # mocap: Position モード時のみ(リジッドボディ未検出なら null)
        mocap = self.position.mocap_snapshot(now) if mode == MODE_POSITION else None

        # setpoint: 現在の整形済みセットポイント(バイアス加算前、UI単位)
        trajectory = None
        if mode == MODE_POSITION:
            roll_rad, pitch_rad, alt_m = self.position.current_setpoint()
            yaw_rad, yaw_ctrl_on = self.position.yaw_setpoint()
            tx, ty, tz = self.position.get_target()
            target = {"x": tx, "y": ty, "z": tz}
            trajectory = self.position.trajectory_snapshot()
        else:
            roll_rad, pitch_rad, alt_m = self.posture.current_setpoint()
            yaw_rad, yaw_ctrl_on = self.posture.yaw_setpoint()
            target = None

        relay_stats = None
        if rly_stats is not None:
            relay_stats = {
                "up_frames": rly_stats.up_frames,
                "down_frames": rly_stats.down_frames,
                "crc_errors": rly_stats.crc_errors,
                "cobs_errors": rly_stats.cobs_errors,
                "espnow_send_fail": rly_stats.espnow_send_fail,
                "overflow_drops": rly_stats.overflow_drops,
            }

        # logger.file_path はプロパティを2回読むと(各読みが別個にロックを
        # 取るため)stop() と競合して None になり得る。一度だけ読んで使う
        # (20Hz 配信タスクを AttributeError で殺さないため)。
        log_path = self.logger.file_path
        # 複数機の機体別ログ記録中は「先頭ファイル名 ×N機」を表示する(T3-6)
        multi_log_names = self.multi.log_file_names()
        if multi_log_names:
            log_file = f"{multi_log_names[0]} ×{len(multi_log_names)}機"
        else:
            log_file = log_path.name if log_path else None

        session = {
            "mode": mode,
            "phase": phase,
            "serial_connected": self.serial.is_connected,
            "airframe": airframe["name"] if airframe else None,
            "relay_target_ok": relay_target_ok,
            "logging": logging_enabled,
            "log_file": log_file,
            "target": target,
            "setpoint": {
                "roll_deg": roll_rad * RAD_TO_DEG,
                "pitch_deg": pitch_rad * RAD_TO_DEG,
                "alt_m": alt_m,
                "yaw_deg": yaw_rad * RAD_TO_DEG,
            },
            "yaw_ctrl_on": yaw_ctrl_on,
            "trajectory": trajectory,
            "latency_ms": self.serial.latency_ms,
            "relay_stats": relay_stats,
            # リレー鮮度: RLY_STATS(1Hz)の受信時刻ベース。counter が静止していても
            # フレームが届いている限り true(UI のリレーリンク表示が使用)
            "relay_fresh": (rly_stats_t is not None
                            and (now - rly_stats_t) <= self._relay_stats_fresh_s),
            "link_stats": self.serial.stats(),
            "mocap_dropout": mocap_warned,
        }

        # 実験モードの状態(UI の Experiment タブが 20Hz で参照)
        experiment = None
        if mode == MODE_EXPERIMENT:
            exp_sample, exp_age = self.experiment.latest_sample()
            experiment = {
                "active": experiment_active,
                "motor": self.experiment.motor_status(),
                "sweep": self.experiment.sweep.status(),
                "sequence": self.experiment.sequence.status(),
                "cal3d": self.experiment.cal3d_status(),
                # 計測ログ(EKF/FF 性能ログ)の状態(T1-6)
                "recording": self.experiment.recorder.status(),
                "exp_age_s": exp_age,
                "exp": None if exp_sample is None else {
                    "current_a": exp_sample["current_a"],
                    "vbat_v": exp_sample["vbat_v"],
                    "cv": exp_sample["cv"],
                    "b_raw": exp_sample["b_raw"],
                    "b_cal": exp_sample["b_cal"],
                    "imu_temp_c": exp_sample["imu_temp_c"],
                    # 加速度 [g](フィルタ後・較正前 = 6面キャリブの入力そのもの)
                    "ax": exp_sample["ax"],
                    "ay": exp_sample["ay"],
                    "az": exp_sample["az"],
                    "roll_deg": exp_sample["roll_rad"] * RAD_TO_DEG,
                    "pitch_deg": exp_sample["pitch_rad"] * RAD_TO_DEG,
                    "yaw_deg": exp_sample["yaw_rad"] * RAD_TO_DEG,
                    "duty_cmd": exp_sample["duty_cmd_fw"],
                    "motors_mask": exp_sample["motors_mask_fw"],
                    "mag_fresh": exp_sample["mag_fresh"],
                    "motors_running": exp_sample["motors_running"],
                },
            }
        session["experiment"] = experiment

        # 複数機モードの状態(UI の複数機タブが 20Hz で参照)
        session["multi"] = (self.multi.snapshot(now)
                            if mode == MODE_MULTI else None)

        # TLM 由来の非有限 float を一括で None 化(WS の JSON を壊さない)
        return _json_safe({
            "type": "state",
            "data": {"drone": drone, "mocap": mocap, "session": session}})
