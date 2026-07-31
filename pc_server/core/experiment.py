"""実験モード(モーターテスト/電流×磁場スイープ/加算性シーケンス)。

旧 yaw側プロジェクト(Yaw_Calibration_and_Estimation。2026-07-27 に削除済み)の
pc_server/server.py の TelemetryHub(モーター部)/ SweepRunner / SequenceRunner を
シリアル版に移植:
- モーター駆動: UDP motor_run → CMD_MOTOR_RUN(0.4s キープアライブ再送、
  機体側 1.5s 途絶で自動停止)。停止は CMD_MOTOR_STOP を3回送出。
- サンプル源: UDP JSON テレメトリ → TLM_EXP(25Hz、MOTOR_TEST 状態のみ)。
- 出力 CSV・meta JSON は yaw側スキーマと完全同一(samples 列、
  `stampfly_sweep_meta` v1)。これにより data_analysis/ が無改修で動く。
- 保存先: pc_server/data/sweep_results/。

v2 追加: ExpRecorder(実験計測ログ、EKF/FF 性能評価)。
- 行 = TLM_EXP 受信ごと(≈25Hz)+最新 TLM_STATE のヨー推定/FF 診断列。
- 出力: pc_server/data/exp_logs/explog_<ts>.csv + explog_<ts>_meta.json
  (`stampfly_explog_meta` v2。v2 でフロー/ToF 列を末尾追加)。
- 計測中はスイープ/シーケンス開始と部分マスクの CMD_MOTOR_RUN を拒否する
  (サーバ側が正。相互チェックは start_gate で原子化)。

スイープのタイミング定数・前後ブラケット基準・duty パターンは yaw側の値を
そのまま踏襲する(計測プロトコル定数。変更すると data_analysis の前提と
過去データとの比較が壊れるため、チューニング値ではない = config に置かない)。
"""

from __future__ import annotations

import csv
import json
import math
import queue
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from . import config as cfg
from .calibration import load_geomagnetic_config, load_mag3d_correction
from .mocap import RAD_TO_DEG
from .serial_link import SerialLink, SerialLinkError

# ----------------------------------------------------------------------
# モーター定義(yaw側 server.py / firmware motor.cpp のビット順を踏襲)
# bit0=FL, bit1=FR, bit2=RL, bit3=RR
# ----------------------------------------------------------------------
MOTOR_NAMES = ["FL", "FR", "RL", "RR"]
MOTOR_MASK_ALL = 0x0F

# CMD_MOTOR_STOP の送出回数(1フレーム欠落でモーターが回り続けないための
# 冗長送出。最終防壁は機体側 1.5s 途絶フェイルセーフ — yaw側と同じ3層構成)
MOTOR_STOP_REPEAT = 3

# ----------------------------------------------------------------------
# スイープ定数(yaw側 server.py から踏襲。数値変更禁止 —
# stampfly_sweep_meta v1 スキーマ・data_analysis の前提と対になっている)
# ----------------------------------------------------------------------
SWEEP_DUTY_STEPS = [round(0.1 * i, 2) for i in range(1, 11)]  # 0.1 .. 1.0(up leg)
SWEEP_PATTERNS = ("updown", "up")
SWEEP_DEFAULT_PATTERN = "updown"
SWEEP_SETTLE_S = 1.5
SWEEP_MEASURE_S = 2.5
SWEEP_GAP_S = 2.5          # duty 間のモーター停止ギャップ(前後ブラケット基準)
SWEEP_GAP_SETTLE_S = 1.0   # ギャップ先頭の破棄時間(スピンダウン/残留過渡)
SWEEP_BASE_S = 2.0         # 初期基準(duty=0 参照+アイドル電流)
SWEEP_LINK_TIMEOUT_S = 2.0
SWEEP_BASELINE_JUMP_WARN_UT = 2.0
SWEEP_UNDERVOLT_ABORT_V = 3.0   # 過放電防止の最終しきい値(LiPo 保護)
SWEEP_OVERCURRENT_ABORT_A = 12.0

# 加算性シーケンス(FL→FR→RL→RR 自動4本)
SEQUENCE_DEFAULT_PLAN = [0x1, 0x2, 0x4, 0x8]  # FL, FR, RL, RR(単機)
SEQUENCE_MIN_START_VBAT_V = 3.5
SEQUENCE_COOLDOWN_S = 10.0
SEQUENCE_VBAT_CLAMP_MIN_V = 3.0   # UI 指定しきい値の受理範囲
SEQUENCE_VBAT_CLAMP_MAX_V = 4.4

# TLM_EXP 配信キューの深さ(25Hz × 実験クライアント。溢れたら黙って捨てる)
EXP_CLIENT_QUEUE_DEPTH = 200

# 3D磁気キャリブレーションの収集サンプル上限(yaw側と同値)
CAL3D_MAX_SAMPLES = 6000

# ----------------------------------------------------------------------
# 実験計測ログ(ExpRecorder)の CSV 列(stampfly_explog_meta v2 と対の契約。
# 解析スクリプトの前提になるため順序変更禁止。追加は末尾のみ —
# v2 でフロー/ToF 列(flow_* / alt_tof_m。TLM_STATE スナップショット由来)を
# 末尾追加した。旧ログ(v1)は該当列なしとして data_analysis 側が自動スキップ)
# ----------------------------------------------------------------------
EXPLOG_FIELDS = [
    "t_s", "exp_elapsed_ms", "duty_cmd", "motors_mask", "motors",
    "cv", "mag_fresh",
    "current_a", "vbat_v", "shunt_uv",
    "bx_raw", "by_raw", "bz_raw", "bx_cal", "by_cal", "bz_cal", "imu_temp_c",
    "roll_deg", "pitch_deg", "yaw_madgwick_deg",
    "p_rad_s", "q_rad_s", "r_rad_s", "ax_g", "ay_g", "az_g",
    "yaw_est_deg", "yaw_gyro_int_deg", "yaw_ref_deg",
    "db_hat_x_ut", "db_hat_y_ut", "bm_x_ut", "bm_y_ut",
    "nis", "ffg", "ff_status", "tlm_state_age_ms",
    # v2 追加(オプティカルフロー/ToF。契約 §1.1 の TLM_STATE 末尾拡張)
    "flow_vx_mps", "flow_vy_mps", "flow_squal", "flow_status", "flow_dt_ms",
    "alt_tof_m",
]

# stampfly_explog_meta のスキーマ版数(2: フロー/ToF 列の末尾追加)
EXPLOG_META_VERSION = 2

MS_PER_S = 1000.0   # 単位変換(tlm_state_age_ms 列)

# 計測中 LED(CMD_LED_MODE mode=1、マゼンタ常灯)のキープアライブ再送間隔 [s]。
# 機体側は最後の mode=1 受信から 3 秒(config.hpp led_recording_failsafe_ms)
# で AUTO へ自動復帰するため、その 1/3 の周期で再送してリンク断に備える。
# 再送は session の supervisor(20Hz)に相乗りし、専用スレッドは作らない。
LED_KEEPALIVE_INTERVAL_S = 1.0


def clamp_motor_mask(value: Any, default: int = MOTOR_MASK_ALL) -> int:
    """リクエスト値を 4bit のモーターマスクに矯正する。"""
    try:
        mask = int(value)
    except (TypeError, ValueError):
        return default
    return mask & MOTOR_MASK_ALL


def motors_label(mask: int) -> str:
    """モーターマスクの表示名(例: 2 → 'FR'、5 → 'FL+RL')。"""
    mask = int(mask) & MOTOR_MASK_ALL
    names = [MOTOR_NAMES[i] for i in range(4) if mask & (1 << i)]
    return "+".join(names) if names else "NONE"


def clamp_sweep_pattern(value: Any) -> str:
    pattern = str(value).strip().lower() if value is not None else ""
    return pattern if pattern in SWEEP_PATTERNS else SWEEP_DEFAULT_PATTERN


def build_duty_sequence(pattern: str) -> list[tuple[float, str]]:
    """実行 duty 列を (duty, leg) で返す。updown: 0.1..1.0 → 0.9..0.1
    (ピークは繰り返さない。up leg に属する)。"""
    up = [(d, "up") for d in SWEEP_DUTY_STEPS]
    if pattern == "up":
        return up
    return up + [(d, "down") for d in reversed(SWEEP_DUTY_STEPS[:-1])]


def sanitize_notes(value: Any) -> dict[str, str]:
    """既知の自由記述フィールドのみ残す(トリム+上限200字)。"""
    out: dict[str, str] = {}
    if isinstance(value, dict):
        for key in ("location", "orientation", "memo", "sequence"):
            raw = value.get(key)
            if isinstance(raw, (str, int, float)):
                text = str(raw).strip()
                if text:
                    out[key] = text[:200]
    return out


def apply_mag3d(raw_xyz: list[float], correction: Optional[dict]) -> list[float]:
    """corrected = matrix @ (raw - offset)(ファームと同じパイプライン)。"""
    if correction is None:
        return [float(raw_xyz[0]), float(raw_xyz[1]), float(raw_xyz[2])]
    off = correction["offset"]
    mat = correction["matrix"]
    x = raw_xyz[0] - off[0]
    y = raw_xyz[1] - off[1]
    z = raw_xyz[2] - off[2]
    return [
        mat[0][0] * x + mat[0][1] * y + mat[0][2] * z,
        mat[1][0] * x + mat[1][1] * y + mat[1][2] * z,
        mat[2][0] * x + mat[2][1] * y + mat[2][2] * z,
    ]


def mag3d_file_info() -> Optional[dict]:
    """mag3d_calibration.json の品質メタ抜粋(sweep / explog の meta 用)。"""
    try:
        data = json.loads(
            cfg.MAG3D_CALIBRATION_PATH.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        return None
    if not isinstance(data, dict):
        return None
    return {key: data.get(key) for key in
            ("sample_count", "target_radius", "rms_error",
             "relative_rms_error", "saved_at", "applied_at")}


def _mean_vec(vectors: list[list[float]]) -> list[float]:
    if not vectors:
        return [0.0, 0.0, 0.0]
    count = len(vectors)
    return [sum(vec[a] for vec in vectors) / count for a in range(3)]


class SweepAborted(Exception):
    """実行中スイープの中断要求(内部制御フロー用)。"""


class SweepError(Exception):
    """スイープ継続不能(リンク途絶・低電圧・過電流など)。"""


class ExperimentHub:
    """実験モードの中枢: TLM_EXP 配信・モーターテスト・排他スロット。

    yaw側 TelemetryHub のシリアル版。TLM_EXP は session 層の RX ハンドラから
    on_tlm_exp() で流入し、latest 保持+クライアントキュー配信+3D磁気収集を
    行う(RXスレッド上: ブロッキング禁止のため put_nowait のみ)。
    """

    def __init__(self, server_config: dict, link: SerialLink,
                 notify: Callable[[str], None],
                 sweep_result_dir: Path = cfg.SWEEP_RESULTS_DIR) -> None:
        exp_cfg = server_config["experiment"]
        self._keepalive_s: float = exp_cfg["motor_keepalive_s"]
        self._max_duty: float = exp_cfg["motor_max_duty"]
        self.exp_fresh_s: float = exp_cfg["exp_fresh_s"]
        self.link = link
        self.notify = notify

        self.lock = threading.Lock()
        self._active = False               # 機体が MOTOR_TEST 状態(ACK 確認済み)
        self.latest: Optional[dict] = None  # 直近の TLM_EXP サンプル dict
        self.last_exp_time = 0.0            # time.time()(スイープの t 基準と共用)
        self.clients: list[queue.Queue] = []

        # モーターテスト状態(キープアライブスレッドが参照)
        self.motor_running = False
        self.motor_duty = 0.0
        self.motor_mask = MOTOR_MASK_ALL

        # 3D磁気キャリブレーション収集(mag3D 前の生値 b_raw)
        self.cal3d_collecting = False
        self.cal3d_samples: list[list[float]] = []

        # キャリブ/FF プロファイル操作の排他スロット(yaw側の
        # calprofile_lock + start_gate + inflight フラグの no-TOCTOU パターン)
        self.calprofile_lock = threading.Lock()
        self.calprofile_inflight = False
        self.start_gate = threading.Lock()

        self.sweep = SweepRunner(self, sweep_result_dir)
        self.sequence = SequenceRunner(self)
        # 実験計測ログ(EKF/FF 性能評価)。tlm_state_provider / ff_state_provider
        # は session 層が構築後に配線する(循環依存を避けるため)
        self.recorder = ExpRecorder(
            self, flush_every_rows=server_config["logging"]["flush_every_rows"])

        self._keepalive_stop = threading.Event()
        self._keepalive_thread = threading.Thread(
            target=self._keepalive_loop, name="motor-keepalive", daemon=True)
        self._keepalive_thread.start()

    # ------------------------------------------------------------------
    # 活性化(session 層が CMD_MODE の ACK 確認後に呼ぶ)
    # ------------------------------------------------------------------

    def activate(self) -> None:
        with self.lock:
            self._active = True

    def deactivate(self) -> None:
        """実験モード終了: スイープ/シーケンス中断+モーター停止+計測中断。

        実験無効化・モード離脱・切断の全経路がここを通るため、計測ログの
        自動停止(meta に aborted=true)もここで行う(T1-5)。SPACE 緊急停止
        (session.stop / emergency_stop)は deactivate を呼ばないため、
        計測は継続する(モーターだけ止まる — 停止過渡も記録対象)。
        """
        self.sequence.abort_if_running()
        self.sweep.abort_if_running()
        self.motor_stop()
        stopped = self.recorder.stop(aborted=True)
        if stopped.get("ok"):
            self.notify(f"計測を中断しました: {stopped.get('file')}"
                        "(実験モード終了のため)")
        with self.lock:
            self._active = False
            self.cal3d_collecting = False

    @property
    def active(self) -> bool:
        with self.lock:
            return self._active

    def shutdown(self) -> None:
        self.deactivate()
        self._keepalive_stop.set()

    # ------------------------------------------------------------------
    # TLM_EXP 流入(RXスレッド上: ブロッキング禁止)
    # ------------------------------------------------------------------

    def on_tlm_exp(self, tlm: proto.TlmExp, seq: int) -> None:
        now = time.time()
        b_raw = [tlm.bx_raw, tlm.by_raw, tlm.bz_raw]
        b_cal = [tlm.bx_cal, tlm.by_cal, tlm.bz_cal]
        sample = {
            "seq": seq,
            "recv_t": now,
            "elapsed_ms": tlm.elapsed_ms,
            "current_a": tlm.current_a,
            "vbat_v": tlm.vbat_v,
            "shunt_uv": tlm.shunt_uv,
            "cv": 1 if (tlm.flags & proto.TlmExp.FLAG_CURRENT_VALID) else 0,
            "b_raw": b_raw,
            "b_cal": b_cal,
            "imu_temp_c": tlm.imu_temp_c,
            "roll_rad": tlm.roll,
            "pitch_rad": tlm.pitch,
            "yaw_rad": tlm.yaw,
            "roll_rate": tlm.p,
            "pitch_rate": tlm.q,
            "yaw_rate": tlm.r,
            "ax": tlm.ax,
            "ay": tlm.ay,
            "az": tlm.az,
            "duty_cmd_fw": tlm.duty_cmd,
            "motors_mask_fw": tlm.motors_mask,
            "mag_fresh": bool(tlm.flags & proto.TlmExp.FLAG_MAG_FRESH),
            "motors_running": bool(tlm.flags & proto.TlmExp.FLAG_MOTORS_RUNNING),
        }
        with self.lock:
            self.latest = sample
            self.last_exp_time = now
            if self.cal3d_collecting:
                self.cal3d_samples.append(list(b_raw))
                if len(self.cal3d_samples) > CAL3D_MAX_SAMPLES:
                    self.cal3d_samples = self.cal3d_samples[-CAL3D_MAX_SAMPLES:]
            clients = list(self.clients)
        # 計測ログ(バッファ付き writerow のみ — RXスレッドをブロックしない)
        self.recorder.on_tlm_exp(tlm, now)
        for client in clients:
            try:
                client.put_nowait(sample)
            except queue.Full:
                pass

    def add_client(self) -> queue.Queue:
        client: queue.Queue = queue.Queue(maxsize=EXP_CLIENT_QUEUE_DEPTH)
        with self.lock:
            self.clients.append(client)
        return client

    def remove_client(self, client: queue.Queue) -> None:
        with self.lock:
            if client in self.clients:
                self.clients.remove(client)

    def exp_age_s(self) -> Optional[float]:
        with self.lock:
            last = self.last_exp_time
        return None if last == 0.0 else (time.time() - last)

    def latest_sample(self) -> tuple[Optional[dict], Optional[float]]:
        with self.lock:
            sample = self.latest
            last = self.last_exp_time
        age = None if last == 0.0 else (time.time() - last)
        return sample, age

    # ------------------------------------------------------------------
    # 3D磁気キャリブレーション収集(fit は calibration.py)
    # ------------------------------------------------------------------

    def cal3d_start(self) -> None:
        with self.lock:
            self.cal3d_collecting = True
            self.cal3d_samples = []

    def cal3d_stop(self) -> list[list[float]]:
        with self.lock:
            self.cal3d_collecting = False
            return list(self.cal3d_samples)

    def cal3d_status(self) -> dict:
        with self.lock:
            return {"collecting": self.cal3d_collecting,
                    "sample_count": len(self.cal3d_samples)}

    def collect_samples(self, duration_s: float) -> list[dict]:
        """duration_s の間 TLM_EXP サンプルを収集して返す(accel6 面平均用)。

        ブロッキングするため executor スレッドから呼ぶこと。
        """
        client = self.add_client()
        collected: list[dict] = []
        deadline = time.time() + duration_s
        try:
            while True:
                remaining = deadline - time.time()
                if remaining <= 0:
                    break
                try:
                    collected.append(client.get(timeout=min(remaining, 0.5)))
                except queue.Empty:
                    continue
        finally:
            self.remove_client(client)
        return collected

    # ------------------------------------------------------------------
    # モーターテスト(CMD_MOTOR_RUN / CMD_MOTOR_STOP)
    # ------------------------------------------------------------------

    def _clamp_motor_duty(self, duty: Any) -> float:
        try:
            value = float(duty)
        except (TypeError, ValueError):
            return 0.0
        if not math.isfinite(value):
            return 0.0
        return max(0.0, min(self._max_duty, value))

    def _send_motor_run(self, duty: float, mask: int) -> dict:
        payload = proto.CmdMotorRun(duty=duty, mask=mask & MOTOR_MASK_ALL)
        try:
            self.link.send(proto.MsgType.CMD_MOTOR_RUN, payload.to_payload())
        except SerialLinkError as exc:
            # リンク一時断。機体側は 1.5s 途絶で自動停止するため報告のみ。
            return {"ok": False, "running": True, "duty": duty, "mask": mask,
                    "error": str(exc)}
        return {"ok": True, "running": True, "duty": duty, "mask": mask,
                "motors": motors_label(mask)}

    def _runner_busy(self) -> bool:
        """スイープ/シーケンス実行中か(start_gate 保持中に呼ぶこと)。"""
        return self.sweep.is_running() or self.sequence.is_running()

    _RUNNER_BUSY_MESSAGE = "スイープ/シーケンス実行中は手動モーター操作できません"

    def motor_start(self, duty: Any, mask: Any = MOTOR_MASK_ALL,
                    _internal: bool = False) -> dict:
        if not self.active:
            return {"ok": False, "running": False,
                    "message": "実験モードが有効ではありません"}
        clamped = self._clamp_motor_duty(duty)
        mask_value = clamp_motor_mask(mask)
        # 手動操作はスイープ/シーケンス実行中は拒否する(UI の busy ゲート
        # だけに頼らない — 別クライアント/エコー遅れの操作が measure 中の
        # duty/mask を上書きし、CSV の duty_cmd 列と実回転が食い違うのを防ぐ)。
        # 判定と状態セットは start_gate 下で原子化(sweep/sequence.start と
        # 同じ no-TOCTOU パターン)。スイープ内部からの駆動(_internal)は対象外。
        with self.start_gate:
            if not _internal and self._runner_busy():
                return {"ok": False, "running": False,
                        "message": self._RUNNER_BUSY_MESSAGE}
            # 計測中(ExpRecorder)は全モーター駆動のみ受理する(T1-4:
            # 部分マスクの回転を混ぜると EKF/FF 性能ログの前提が崩れる)。
            # 停止(motor_stop)は常に許可(緊急停止経路)。
            if not _internal and self.recorder.is_recording() \
                    and mask_value != MOTOR_MASK_ALL:
                return {"ok": False, "running": False,
                        "message": "計測中は全モーター(FL+FR+RL+RR)のみ"
                                   "回転できます"}
            with self.lock:
                self.motor_running = True
                self.motor_duty = clamped
                self.motor_mask = mask_value
        return self._send_motor_run(clamped, mask_value)

    def motor_apply(self, duty: Any, _internal: bool = False) -> dict:
        clamped = self._clamp_motor_duty(duty)
        with self.start_gate:
            if not _internal and self._runner_busy():
                return {"ok": False, "duty": clamped,
                        "message": self._RUNNER_BUSY_MESSAGE}
            with self.lock:
                running = self.motor_running
                self.motor_duty = clamped
                mask_value = self.motor_mask
        if not running:
            return {"ok": False, "running": False, "duty": clamped,
                    "message": "モーターが回転していません。先に Start してください"}
        return self._send_motor_run(clamped, mask_value)

    def motor_stop(self) -> dict:
        with self.lock:
            self.motor_running = False
            self.motor_duty = 0.0
            self.motor_mask = MOTOR_MASK_ALL
        # 冗長送出: フレーム1発の欠落でモーターが回り続けないようにする
        # (最終防壁は機体側 1.5s 途絶フェイルセーフ)
        error: Optional[str] = None
        for _ in range(MOTOR_STOP_REPEAT):
            try:
                self.link.send(proto.MsgType.CMD_MOTOR_STOP)
            except SerialLinkError as exc:
                error = str(exc)
        result: dict = {"ok": error is None, "running": False, "duty": 0.0}
        if error is not None:
            result["error"] = error
        return result

    def motor_status(self) -> dict:
        with self.lock:
            return {"running": self.motor_running, "duty": self.motor_duty,
                    "mask": self.motor_mask,
                    "motors": motors_label(self.motor_mask)}

    def _keepalive_loop(self) -> None:
        """0.4s 周期の CMD_MOTOR_RUN 再送(yaw側 run_motor_keepalive 踏襲)。"""
        while not self._keepalive_stop.wait(self._keepalive_s):
            with self.lock:
                running = self.motor_running and self._active
                duty = self.motor_duty
                mask = self.motor_mask
            if running and self.link.is_connected:
                payload = proto.CmdMotorRun(duty=duty, mask=mask)
                try:
                    self.link.send(proto.MsgType.CMD_MOTOR_RUN,
                                   payload.to_payload())
                except SerialLinkError:
                    # 一時的なリンク断。機体側フェイルセーフに任せて継続。
                    pass

    # ------------------------------------------------------------------
    # 計測中 LED(CMD_LED_MODE)
    # ------------------------------------------------------------------

    def send_led_mode(self, mode: int) -> bool:
        """CMD_LED_MODE を送る(fire-and-forget)。成功なら True。

        計測中インジケータ(mode=1: MOTOR_TEST 中マゼンタ常灯=動画カット点、
        mode=0: 通常表示へ復帰)。送信失敗や NACK でも計測は止めない —
        機体側が 3 秒フェイルセーフで AUTO へ自動復帰するため、ACK 待ちは
        せず、失敗の扱い(警告するか黙るか)は呼び出し側に委ねる。
        """
        payload = proto.CmdLedMode(mode=mode)
        try:
            self.link.send(proto.MsgType.CMD_LED_MODE, payload.to_payload())
        except SerialLinkError:
            return False
        return True

    # ------------------------------------------------------------------
    # キャリブ/FF プロファイル操作の排他スロット
    # ------------------------------------------------------------------

    def calprofile_begin(self) -> Optional[str]:
        """プロファイル操作スロットを取る。不可なら理由(日本語)を返す。

        スイープ/シーケンスの実行チェックと inflight のセットを start_gate
        の同一区間で行い、双方が同じゲートで相互確認する(TOCTOU なし)。
        """
        if not self.calprofile_lock.acquire(blocking=False):
            return "別のプロファイル操作が実行中です"
        with self.start_gate:
            if self.sweep.is_running() or self.sequence.is_running():
                self.calprofile_lock.release()
                return "スイープ/シーケンス実行中はプロファイル操作できません"
            self.calprofile_inflight = True
        return None

    def calprofile_end(self) -> None:
        with self.start_gate:
            self.calprofile_inflight = False
        self.calprofile_lock.release()


def _iso_localtime(epoch: float) -> str:
    """epoch 秒 → ISO8601 ローカル時刻(sweep meta の created_at と同形式)。"""
    return time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(epoch))


class ExpRecorder:
    """実験モードの計測レコーダ(EKF/FF 性能評価ログ、T1-1〜T1-5)。

    寿命: exp_record_start(手動)〜 exp_record_stop(手動、aborted=false)
    または実験無効化・モード離脱・切断(hub.deactivate 経由、aborted=true)。
    SPACE 緊急停止では止めない(モーターだけ止まる — 停止過渡も記録対象)。

    行 = TLM_EXP フレーム受信ごと(≈25Hz、RXスレッド上の on_tlm_exp 経由。
    バッファ付き writerow のみでブロックしない)。TLM_EXP の生値に加え、
    最新 TLM_STATE のヨー推定/FF 診断列を並記する(tlm_state_age_ms が
    その鮮度。TLM_STATE 未受信なら該当列は空欄)。

    開始条件はサーバ側が正(T1-4): スイープ/シーケンス実行中は開始不可。
    計測中はスイープ/シーケンス開始と部分マスクの CMD_MOTOR_RUN が拒否
    される(相互チェックは hub.start_gate の同一区間で原子化 — no-TOCTOU)。
    mode==experiment かつ MOTOR_TEST 有効の判定は session 層
    (_experiment_ready)と hub.active の二段で行う。

    tlm_state_provider / ff_state_provider は session 層が構築後に配線する
    (CalibrationManager の tlm_state_provider と同じ依存注入パターン)。
    """

    def __init__(self, hub: ExperimentHub, logs_dir: Path = cfg.EXP_LOGS_DIR,
                 flush_every_rows: int = 25) -> None:
        self.hub = hub
        self.logs_dir = Path(logs_dir)   # テストから差し替え可
        self._flush_every_rows = flush_every_rows
        # session 層が配線する供給線(未配線でも該当列/項目が空になるだけ)
        self.tlm_state_provider: Optional[Callable] = None   # -> (TlmState|None, age_s|None)
        self.ff_state_provider: Optional[Callable] = None    # -> dict|None
        self._lock = threading.Lock()
        self._file = None
        self._writer: Optional[csv.DictWriter] = None
        self._file_name: Optional[str] = None
        self._stamp = ""
        self._started_at = 0.0        # time.time()(t_s 列と meta の基準)
        self._sample_count = 0
        self._rows_since_flush = 0
        # 計測中 LED キープアライブの最終送信時刻(supervisor のクロック基準。
        # None = 未送信 → 次の supervise 周期で即送信)
        self._led_keepalive_at: Optional[float] = None

    def is_recording(self) -> bool:
        with self._lock:
            return self._writer is not None

    def status(self) -> dict:
        """status payload の experiment.recording(UI 表示用)。"""
        with self._lock:
            return {"active": self._writer is not None,
                    "file": self._file_name,
                    "samples": self._sample_count}

    def service_led_keepalive(self, now: float) -> None:
        """計測中 LED(mode=1)のキープアライブ再送1周期。

        session の supervisor(20Hz)から毎周期呼ばれ、計測中のみ
        LED_KEEPALIVE_INTERVAL_S(1s)間隔に間引いて CMD_LED_MODE(1) を
        再送する(機体側は 3s 途絶で AUTO 復帰するため)。now は supervisor
        のクロック基準 — time.time() と混ぜず、この関数内の比較のみに使う。
        送信失敗は黙って次周期に任せる(モーターキープアライブと同じ扱い。
        1Hz で警告を出し続けない)。
        """
        with self._lock:
            if self._writer is None:
                return
            last = self._led_keepalive_at
            if last is not None and now - last < LED_KEEPALIVE_INTERVAL_S:
                return
            self._led_keepalive_at = now
        self.hub.send_led_mode(proto.CmdLedMode.MODE_RECORDING)

    # ------------------------------------------------------------------
    # 開始 / 停止
    # ------------------------------------------------------------------

    def start(self) -> dict:
        if not self.hub.active:
            return {"ok": False,
                    "message": "実験モードが有効ではありません", **self.status()}
        # start_gate 下で「スイープ/シーケンス非実行の確認」と「計測中への
        # 遷移」を原子化する(SweepRunner/SequenceRunner.start 側の計測中
        # チェックと同じゲートで相互確認 — no-TOCTOU)
        with self.hub.start_gate:
            if self.hub.sweep.is_running() or self.hub.sequence.is_running():
                return {"ok": False,
                        "message": "スイープ/シーケンス実行中は計測を"
                                   "開始できません", **self.status()}
            with self._lock:
                if self._writer is not None:
                    return {"ok": False, "message": "計測は既に実行中です",
                            "active": True, "file": self._file_name,
                            "samples": self._sample_count}
                try:
                    self.logs_dir.mkdir(parents=True, exist_ok=True)
                    self._started_at = time.time()
                    self._stamp = time.strftime(
                        "%Y%m%d_%H%M%S", time.localtime(self._started_at))
                    name = f"explog_{self._stamp}.csv"
                    self._file = (self.logs_dir / name).open(
                        "w", newline="", encoding="utf-8")
                except OSError as exc:
                    self._file = None
                    return {"ok": False,
                            "message": f"計測ファイルを作成できません: {exc}",
                            "active": False, "file": None, "samples": 0}
                self._writer = csv.DictWriter(self._file,
                                              fieldnames=EXPLOG_FIELDS)
                self._writer.writeheader()
                self._file.flush()
                self._file_name = name
                self._sample_count = 0
                self._rows_since_flush = 0
                self._led_keepalive_at = None
        # 計測中 LED(MOTOR_TEST 中マゼンタ常灯 = 動画のカット点)を点ける。
        # 送信失敗でも計測は止めない(警告のみ): 以後は supervisor の
        # 約1秒キープアライブ(service_led_keepalive)が再送し続ける。
        if not self.hub.send_led_mode(proto.CmdLedMode.MODE_RECORDING):
            self.hub.notify("計測中LED(CMD_LED_MODE)の送信に失敗しました"
                            "(計測は継続します。キープアライブで再送します)")
        return {"ok": True, "message": "計測を開始しました", **self.status()}

    def stop(self, aborted: bool = False) -> dict:
        """計測を停止し meta JSON を書く(未計測なら ok=False で無害)。"""
        with self._lock:
            file = self._file
            name = self._file_name
            stamp = self._stamp
            started_at = self._started_at
            count = self._sample_count
            self._file = None
            self._writer = None
            self._file_name = None
        if file is None:
            return {"ok": False, "message": "計測は実行されていません",
                    **self.status()}
        try:
            file.close()
        except OSError:
            pass
        # LED を通常表示へ戻す。手動停止・自動停止(hub.deactivate 経由の
        # モード離脱/切断)の全経路がここを通る。送信できなくても機体側の
        # 3秒フェイルセーフで AUTO へ自動復帰するため警告のみ。
        if not self.hub.send_led_mode(proto.CmdLedMode.MODE_AUTO):
            self.hub.notify("LED復帰(CMD_LED_MODE)の送信に失敗しました"
                            "(機体側フェイルセーフで約3秒後に復帰します)")
        result = {"ok": True, "message": "計測を終了しました",
                  "file": name, "samples": count, "aborted": bool(aborted),
                  "active": False}
        try:
            result["meta"] = self._write_meta(stamp, started_at, count,
                                              aborted)
        except OSError as exc:
            result["meta"] = None
            result["message"] = f"計測を終了しました(meta 書込失敗: {exc})"
        return result

    def _write_meta(self, stamp: str, started_at: float, count: int,
                    aborted: bool) -> str:
        ended_at = time.time()
        geomag = load_geomagnetic_config()
        meta = {
            "schema": "stampfly_explog_meta",
            "version": EXPLOG_META_VERSION,
            "started_at": _iso_localtime(started_at),
            "started_at_epoch": started_at,
            "ended_at": _iso_localtime(ended_at),
            "ended_at_epoch": ended_at,
            "sample_count": count,
            "aborted": bool(aborted),
            "ff_state": self._ff_state_snapshot(),
            "mag3d_file_info": mag3d_file_info(),
            "geomag_profile": (geomag.get("profile")
                               if isinstance(geomag, dict)
                               and "error" not in geomag else None),
        }
        meta_name = f"explog_{stamp}_meta.json"
        (self.logs_dir / meta_name).write_text(
            json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        return meta_name

    def _ff_state_snapshot(self) -> Optional[dict]:
        provider = self.ff_state_provider
        if provider is None:
            return None
        try:
            return provider()
        except Exception:   # 防御的バックストップ(meta 欠落 < 停止失敗)
            return None

    # ------------------------------------------------------------------
    # 行の書き込み(RXスレッド上: hub.on_tlm_exp から)
    # ------------------------------------------------------------------

    def on_tlm_exp(self, tlm: proto.TlmExp, now: float) -> None:
        if not self.is_recording():
            return
        row = self._build_row(tlm)
        with self._lock:
            if self._writer is None:   # 直前に停止された(この行は捨てる)
                return
            row["t_s"] = round(now - self._started_at, 4)
            self._writer.writerow(row)
            self._sample_count += 1
            self._rows_since_flush += 1
            if self._rows_since_flush >= self._flush_every_rows:
                self._file.flush()
                self._rows_since_flush = 0

    def _build_row(self, tlm: proto.TlmExp) -> dict:
        """TLM_EXP + 最新 TLM_STATE スナップショット → EXPLOG_FIELDS の行。"""
        row = {
            "exp_elapsed_ms": tlm.elapsed_ms,
            "duty_cmd": tlm.duty_cmd,
            "motors_mask": tlm.motors_mask,
            "motors": motors_label(tlm.motors_mask),
            "cv": 1 if (tlm.flags & proto.TlmExp.FLAG_CURRENT_VALID) else 0,
            "mag_fresh": 1 if (tlm.flags & proto.TlmExp.FLAG_MAG_FRESH) else 0,
            "current_a": tlm.current_a,
            "vbat_v": tlm.vbat_v,
            "shunt_uv": tlm.shunt_uv,
            "bx_raw": tlm.bx_raw, "by_raw": tlm.by_raw, "bz_raw": tlm.bz_raw,
            "bx_cal": tlm.bx_cal, "by_cal": tlm.by_cal, "bz_cal": tlm.bz_cal,
            "imu_temp_c": tlm.imu_temp_c,
            # 姿勢角は deg(UI/解析単位)、角速度は rad/s のまま(T1-2)
            "roll_deg": tlm.roll * RAD_TO_DEG,
            "pitch_deg": tlm.pitch * RAD_TO_DEG,
            "yaw_madgwick_deg": tlm.yaw * RAD_TO_DEG,
            "p_rad_s": tlm.p, "q_rad_s": tlm.q, "r_rad_s": tlm.r,
            "ax_g": tlm.ax, "ay_g": tlm.ay, "az_g": tlm.az,
        }
        provider = self.tlm_state_provider
        state, age_s = provider() if provider is not None else (None, None)
        if state is not None and age_s is not None:
            row.update({
                "yaw_est_deg": state.yaw_est_rad * RAD_TO_DEG,
                "yaw_gyro_int_deg": state.yaw_gyro_int_rad * RAD_TO_DEG,
                "yaw_ref_deg": state.yaw_ref_rad * RAD_TO_DEG,
                "db_hat_x_ut": state.db_hat_x_ut,
                "db_hat_y_ut": state.db_hat_y_ut,
                "bm_x_ut": state.bm_x_ut,
                "bm_y_ut": state.bm_y_ut,
                "nis": state.nis,
                "ffg": state.ffg,
                "ff_status": state.ff_status,
                "tlm_state_age_ms": age_s * MS_PER_S,
                # v2: オプティカルフロー/ToF(TLM_STATE 末尾拡張。契約 §1.1)
                "flow_vx_mps": state.flow_vx_mps,
                "flow_vy_mps": state.flow_vy_mps,
                "flow_squal": state.flow_squal,
                "flow_status": state.flow_status,
                "flow_dt_ms": state.flow_dt_ms,
                "alt_tof_m": state.altitude_tof,
            })
        return row


class SweepRunner:
    """電流×磁場 duty スイープ(前後ブラケット基準方式)をバックグラウンド
    スレッドで実行する(yaw側 SweepRunner のシリアル移植。ロジック不変)。

    フェーズ: idle → base(モーター停止・duty=0 基準)→ 各 duty
    (settle → measure → モーター停止ギャップ = この duty の後基準かつ
    次 duty の前基準)→ done。後処理で各 measure サンプルからブラケット対の
    時間補間基準を引き、ドリフト除去済みの dB_cor を記録する。
    """

    # CSV 列(yaw側 SAMPLE_FIELDS と完全同一 — data_analysis 互換の生命線)
    SAMPLE_FIELDS = [
        "t_s", "phase", "motors", "duty_cmd", "step_idx", "leg", "seq", "cv",
        "current_a", "vbat_v", "shunt_uv",
        "bx_raw", "by_raw", "bz_raw",
        "bx_cor", "by_cor", "bz_cor",
        "imu_temp_c",
        "roll_deg", "pitch_deg", "yaw_deg",
        "roll_rate", "pitch_rate", "yaw_rate",
        "mag_total_uT",
        "bx_base_cor", "by_base_cor", "bz_base_cor",
        "dB_cor_x", "dB_cor_y", "dB_cor_z",
    ]

    def __init__(self, hub: ExperimentHub, result_dir: Path) -> None:
        self.hub = hub
        self.result_dir = Path(result_dir)
        self.lock = threading.Lock()
        self.thread: Optional[threading.Thread] = None
        self.abort_event = threading.Event()
        self._reset_state()

    def _reset_state(self) -> None:
        self.running = False
        self.phase = "idle"
        self.step_index = 0
        self.pattern = SWEEP_DEFAULT_PATTERN
        self.duty_sequence = build_duty_sequence(self.pattern)
        self.total_steps = len(self.duty_sequence)
        self.duty = 0.0
        self.motor_mask = MOTOR_MASK_ALL
        self.notes: dict[str, str] = {}
        self.message = ""
        self.error: Optional[str] = None
        self.started_at = 0.0
        self.base_raw: Optional[list[float]] = None
        self.last_result: Optional[dict] = None
        self._cur_step_idx: Any = ""
        self._cur_leg = ""
        self._baseline_jumps: list[dict] = []
        self._baseline_flags: list[dict] = []

    # ---- 公開 API(HTTP/WS ハンドラのスレッドから) ----

    def is_running(self) -> bool:
        with self.lock:
            return self.running

    def abort_if_running(self) -> None:
        if self.is_running():
            self.abort()

    def start(self, mask: Any = MOTOR_MASK_ALL, pattern: Any = None,
              notes: Any = None, _internal: bool = False) -> dict:
        mask_value = clamp_motor_mask(mask)
        if mask_value == 0:
            return {**self.status(), "ok": False,
                    "message": "回転させるモーターを1つ以上選択してください"}
        if not self.hub.active:
            return {**self.status(), "ok": False,
                    "message": "実験モードが有効ではありません"}
        age = self.hub.exp_age_s()
        if age is None or age > SWEEP_LINK_TIMEOUT_S:
            return {**self.status(), "ok": False,
                    "message": "実験テレメトリ(TLM_EXP)がありません"}
        # start_gate 下で running 再確認と状態セットを原子化(HTTP の同時
        # リクエストや、シーケンスと手動開始の競合で二重起動しない)
        with self.hub.start_gate:
            if not _internal and self.hub.sequence.is_running():
                return {**self.status(), "ok": False,
                        "message": "シーケンス実行中は単発スイープを開始できません"}
            if self.hub.recorder.is_recording():
                # 計測(EKF/FF性能ログ)とスイープは排他(T1-4。相互チェックは
                # この start_gate 区間で原子化 — ExpRecorder.start 側も同じ
                # ゲート下でスイープ実行を確認する)
                return {**self.status(), "ok": False,
                        "message": "計測(EKF/FF性能ログ)の実行中はスイープを"
                                   "開始できません。先に計測を停止してください"}
            if self.hub.calprofile_inflight:
                return {**self.status(), "ok": False,
                        "message": "キャリブレーション/FFプロファイル操作の実行中は"
                                   "スイープを開始できません"}
            with self.lock:
                if self.running:
                    return {**self._status_locked(), "ok": False,
                            "message": "スイープは既に実行中です"}
                self.abort_event.clear()
                self._reset_state()
                self.motor_mask = mask_value
                self.pattern = clamp_sweep_pattern(pattern)
                self.duty_sequence = build_duty_sequence(self.pattern)
                self.total_steps = len(self.duty_sequence)
                self.notes = sanitize_notes(notes)
                self.running = True
                self.phase = "starting"
                self.message = "スイープ開始"
                self.started_at = time.time()
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()
        return {**self.status(), "ok": True, "message": "スイープを開始しました"}

    def abort(self) -> dict:
        self.abort_event.set()
        self.hub.motor_stop()
        with self.lock:
            if self.running:
                self.message = "中断要求を受け付けました"
        return {**self.status(), "ok": True}

    def status(self) -> dict:
        with self.lock:
            state = self._status_locked()
            duty_sequence = [d for d, _leg in self.duty_sequence]
        state["config"] = {
            "duty_steps": SWEEP_DUTY_STEPS,
            "duty_sequence": duty_sequence,
            "patterns": list(SWEEP_PATTERNS),
            "default_pattern": SWEEP_DEFAULT_PATTERN,
            "settle_s": SWEEP_SETTLE_S,
            "measure_s": SWEEP_MEASURE_S,
            "gap_s": SWEEP_GAP_S,
            "gap_settle_s": SWEEP_GAP_SETTLE_S,
            "base_s": SWEEP_BASE_S,
            "baseline_jump_warn_uT": SWEEP_BASELINE_JUMP_WARN_UT,
            "method": "bracketed_baseline",
        }
        return state

    def _status_locked(self) -> dict:
        return {
            "running": self.running,
            "phase": self.phase,
            "step_index": self.step_index,
            "total_steps": self.total_steps,
            "pattern": self.pattern,
            "duty": self.duty,
            "motor_mask": self.motor_mask,
            "motors": motors_label(self.motor_mask),
            "notes": self.notes,
            "elapsed_s": (time.time() - self.started_at) if self.started_at else 0.0,
            "message": self.message,
            "error": self.error,
            "base_set": self.base_raw is not None,
            "last_result": self.last_result,
        }

    # ---- スレッド本体 ----

    def _set_phase(self, phase: str, duty: Optional[float] = None,
                   step_index: Optional[int] = None,
                   message: Optional[str] = None) -> None:
        with self.lock:
            self.phase = phase
            if duty is not None:
                self.duty = duty
            if step_index is not None:
                self.step_index = step_index
            if message is not None:
                self.message = message

    def _finish(self, phase: str, message: str, result: Optional[dict],
                error: Optional[str] = None) -> None:
        with self.lock:
            self.running = False
            self.phase = phase
            self.message = message
            self.error = error
            self.duty = 0.0
            if result is not None:
                self.last_result = result

    def _run(self) -> None:
        client = self.hub.add_client()
        samples_all: list[dict] = []
        # measure_groups[i] = duty ステップ i の measure サンプル群。
        # baselines[0] は初期 base、baselines[i+1] はステップ i 後のギャップ
        # (= ステップ i の後基準かつステップ i+1 の前基準)。
        measure_groups: list[list[dict]] = []
        baselines: list[dict] = []
        correction = load_mag3d_correction()
        try:
            self._drain(client)
            self.hub.motor_stop()
            self._cur_step_idx, self._cur_leg = "", ""
            self._set_phase("base", duty=0.0, step_index=0,
                            message="基準磁場(duty=0)を測定中(モーター停止)")
            base = self._collect(client, SWEEP_BASE_S, "base", 0.0,
                                 samples_all, correction)
            if not base:
                raise SweepError("基準測定でサンプルを取得できませんでした")
            with self.lock:
                self.base_raw = _mean_vec([s["b_raw"] for s in base])
            baselines.append(self._baseline_summary(base, 0.0))
            for i, (duty, leg) in enumerate(self.duty_sequence):
                self._check_abort()
                self._cur_step_idx, self._cur_leg = i, leg
                leg_tag = "↓" if leg == "down" else "↑"
                self._set_phase("settle", duty=duty, step_index=i + 1,
                                message=f"duty {duty:.2f}{leg_tag} 整定中")
                self.hub.motor_start(duty, self.motor_mask, _internal=True)
                self._collect(client, SWEEP_SETTLE_S, "settle", duty,
                              samples_all, correction)
                self._check_abort()
                self._set_phase("measure", duty=duty, step_index=i + 1,
                                message=f"duty {duty:.2f}{leg_tag} 計測中")
                step = self._collect(client, SWEEP_MEASURE_S, "measure", duty,
                                     samples_all, correction)
                measure_groups.append(step)
                # モーター停止ギャップ = duty ごとの基準。先頭は破棄
                # (スピンダウン・電流減衰・停止直後の磁場過渡)。
                self._set_phase("gap", duty=duty, step_index=i + 1,
                                message=f"duty {duty:.2f}{leg_tag} 後の基準を測定中"
                                        "(モーター停止)")
                self.hub.motor_stop()
                self._collect(client, SWEEP_GAP_SETTLE_S, "gap_settle", duty,
                              samples_all, correction)
                gap = self._collect(client,
                                    max(SWEEP_GAP_S - SWEEP_GAP_SETTLE_S, 0.1),
                                    "baseline", duty, samples_all, correction)
                baselines.append(self._baseline_summary(gap, duty))
            # 後処理: 各 measure サンプルから時間補間ブラケット基準を減算
            self._apply_bracket_baseline(measure_groups, baselines)
            self._baseline_jumps, self._baseline_flags = \
                self._baseline_quality(baselines)
            result = self._write_outputs(samples_all, baselines, correction,
                                         aborted=False)
            done_msg = "完了しました"
            if self._baseline_flags:
                done_msg += (f"(注意: 基準ジャンプ>{SWEEP_BASELINE_JUMP_WARN_UT:g}µT が "
                             f"{len(self._baseline_flags)}ステップ — meta の"
                             "baseline_flags 参照)")
            self._finish("done", done_msg, result)
        except SweepAborted:
            self.hub.motor_stop()
            result = self._best_effort_outputs(samples_all, measure_groups,
                                               baselines, correction)
            self._finish("aborted", "中断しました", result)
        except SweepError as exc:
            self.hub.motor_stop()
            # 安全中断(低電圧/過電流/リンク断)でも収集済みデータは残す
            result = self._best_effort_outputs(samples_all, measure_groups,
                                               baselines, correction)
            self._finish("error", str(exc), result, error=str(exc))
        except Exception as exc:  # 防御的バックストップ
            self.hub.motor_stop()
            self._finish("error", f"想定外のエラー: {exc}", None, error=str(exc))
        finally:
            self.hub.motor_stop()
            self.hub.remove_client(client)

    def _best_effort_outputs(self, samples_all, measure_groups, baselines,
                             correction) -> Optional[dict]:
        if not samples_all:
            return None
        try:
            self._apply_bracket_baseline(measure_groups, baselines)
            self._baseline_jumps, self._baseline_flags = \
                self._baseline_quality(baselines)
            return self._write_outputs(samples_all, baselines, correction,
                                       aborted=True)
        except OSError:
            return None

    @staticmethod
    def _baseline_summary(samples: list[dict], duty: float) -> dict:
        """基準窓のモーター停止磁場平均+平均時刻(ブラケット点1つ)。"""
        if not samples:
            return {"duty": duty, "t": None, "cor": None, "raw": None, "n": 0}
        return {
            "duty": duty,
            "t": sum(s["t_s"] for s in samples) / len(samples),
            "cor": _mean_vec([s["b_cor"] for s in samples]),
            "raw": _mean_vec([s["b_raw"] for s in samples]),
            "n": len(samples),
        }

    @staticmethod
    def _apply_bracket_baseline(measure_groups: list[list[dict]],
                                baselines: list[dict]) -> None:
        """各 measure サンプルにブラケット対(前後基準)の時間補間値を適用。"""
        for i, group in enumerate(measure_groups):
            before = baselines[i] if i < len(baselines) else None
            after = baselines[i + 1] if (i + 1) < len(baselines) else None
            # ブラケット点欠落時(中断ランなど)は片側にフォールバック
            if before is None or before.get("cor") is None:
                before = after
            if after is None or after.get("cor") is None:
                after = before
            if before is None or before.get("cor") is None:
                continue  # 基準が全くない場合は dB 空欄のまま
            b0, t0 = before["cor"], before["t"]
            b1, t1 = after["cor"], after["t"]
            for s in group:
                if t0 is not None and t1 is not None and t1 > t0:
                    frac = (s["t_s"] - t0) / (t1 - t0)
                    frac = 0.0 if frac < 0.0 else (1.0 if frac > 1.0 else frac)
                else:
                    frac = 0.5
                base_vec = [b0[a] + (b1[a] - b0[a]) * frac for a in range(3)]
                s["b_base_cor"] = base_vec
                s["dB_cor"] = [s["b_cor"][a] - base_vec[a] for a in range(3)]

    def _baseline_quality(self, baselines: list[dict]
                          ) -> tuple[list[dict], list[dict]]:
        """隣接ブラケット点のジャンプ量(全件, 閾値超過)を返す。"""
        jumps: list[dict] = []
        flags: list[dict] = []
        for i in range(len(baselines) - 1):
            b0, b1 = baselines[i], baselines[i + 1]
            if not b0.get("cor") or not b1.get("cor"):
                continue
            vec = [b1["cor"][a] - b0["cor"][a] for a in range(3)]
            jump = max(abs(v) for v in vec)
            duty, leg = (self.duty_sequence[i]
                         if i < len(self.duty_sequence) else (None, ""))
            entry = {
                "step_idx": i,
                "duty": duty,
                "leg": leg,
                "jump_uT": round(jump, 3),
                "jump_vec_uT": [round(v, 3) for v in vec],
            }
            jumps.append(entry)
            if jump > SWEEP_BASELINE_JUMP_WARN_UT:
                flags.append(entry)
        return jumps, flags

    def _check_abort(self) -> None:
        if self.abort_event.is_set():
            raise SweepAborted()

    @staticmethod
    def _drain(client: queue.Queue) -> None:
        try:
            while True:
                client.get_nowait()
        except queue.Empty:
            return

    def _collect(self, client: queue.Queue, duration: float, phase: str,
                 duty: float, samples_all: list[dict],
                 correction: Optional[dict]) -> list[dict]:
        collected: list[dict] = []
        seen_seq: set = set()
        deadline = time.time() + duration
        while True:
            self._check_abort()
            remaining = deadline - time.time()
            if remaining <= 0:
                break
            try:
                incoming = client.get(timeout=min(remaining, 0.5))
            except queue.Empty:
                age = self.hub.exp_age_s()
                if age is None or age > SWEEP_LINK_TIMEOUT_S:
                    raise SweepError("実験テレメトリが途絶しました")
                continue
            sample = dict(incoming)   # クライアント間共有 dict を汚さない
            seq = sample.get("seq")
            if seq is not None:
                if seq in seen_seq:
                    continue
                seen_seq.add(seq)
            self._safety_check(sample)
            sample["b_cor"] = apply_mag3d(sample["b_raw"], correction)
            sample["t_s"] = sample["recv_t"] - self.started_at
            sample["phase"] = phase
            sample["motors"] = motors_label(self.motor_mask)
            sample["duty_cmd"] = duty
            sample["step_idx"] = self._cur_step_idx
            sample["leg"] = self._cur_leg
            samples_all.append(sample)
            collected.append(sample)
        return collected

    @staticmethod
    def _safety_check(sample: dict) -> None:
        if sample.get("cv") == 1:
            vbat = sample.get("vbat_v", 0.0)
            # 最終の過放電防止のみ(早期の低電圧警告は機体側 LED が担当)
            if 0.5 < vbat < SWEEP_UNDERVOLT_ABORT_V:
                raise SweepError(
                    f"過放電防止の最終しきい値 {SWEEP_UNDERVOLT_ABORT_V:.2f}V を"
                    f"下回りました({vbat:.2f}V)。バッテリー保護のため停止しました")
            if abs(sample.get("current_a", 0.0)) > SWEEP_OVERCURRENT_ABORT_A:
                raise SweepError(
                    f"過電流 {sample.get('current_a', 0.0):.2f}A を検出したため"
                    "中断しました")

    def _write_outputs(self, samples_all, baselines, correction,
                       aborted: bool) -> dict:
        """サンプル CSV+meta JSON を書き出す(yaw側スキーマと完全同一)。"""
        self.result_dir.mkdir(parents=True, exist_ok=True)
        stamp = time.strftime("%Y%m%d_%H%M%S",
                              time.localtime(self.started_at or time.time()))
        samples_name = f"sweep_{stamp}_samples.csv"
        with (self.result_dir / samples_name).open(
                "w", newline="", encoding="utf-8") as fp:
            writer = csv.DictWriter(fp, fieldnames=self.SAMPLE_FIELDS)
            writer.writeheader()
            for s in samples_all:
                base_vec = s.get("b_base_cor")
                dB_vec = s.get("dB_cor")
                writer.writerow({
                    "t_s": round(s["t_s"], 4),
                    "phase": s["phase"],
                    "motors": s["motors"],
                    "duty_cmd": s["duty_cmd"],
                    "step_idx": s.get("step_idx", ""),
                    "leg": s.get("leg", ""),
                    "seq": s["seq"],
                    "cv": s["cv"],
                    "current_a": s["current_a"],
                    "vbat_v": s["vbat_v"],
                    "shunt_uv": s["shunt_uv"],
                    "bx_raw": s["b_raw"][0], "by_raw": s["b_raw"][1],
                    "bz_raw": s["b_raw"][2],
                    "bx_cor": s["b_cor"][0], "by_cor": s["b_cor"][1],
                    "bz_cor": s["b_cor"][2],
                    "imu_temp_c": s.get("imu_temp_c", ""),
                    # 姿勢角は yaw側テレメトリ(deg)と同じ単位で記録する
                    "roll_deg": s["roll_rad"] * RAD_TO_DEG,
                    "pitch_deg": s["pitch_rad"] * RAD_TO_DEG,
                    "yaw_deg": s["yaw_rad"] * RAD_TO_DEG,
                    # 角速度は yaw側と同じ rad/s
                    "roll_rate": s["roll_rate"], "pitch_rate": s["pitch_rate"],
                    "yaw_rate": s["yaw_rate"],
                    "mag_total_uT": math.sqrt(sum(v * v for v in s["b_cal"])),
                    "bx_base_cor": base_vec[0] if base_vec else "",
                    "by_base_cor": base_vec[1] if base_vec else "",
                    "bz_base_cor": base_vec[2] if base_vec else "",
                    "dB_cor_x": dB_vec[0] if dB_vec else "",
                    "dB_cor_y": dB_vec[1] if dB_vec else "",
                    "dB_cor_z": dB_vec[2] if dB_vec else "",
                })
        meta_name = f"sweep_{stamp}_meta.json"
        meta = self._build_meta(samples_all, baselines, correction, aborted,
                                samples_name)
        (self.result_dir / meta_name).write_text(
            json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        return {
            "dir": str(self.result_dir),
            "samples": samples_name,
            "meta": meta_name,
            "sample_count": len(samples_all),
            "baseline_flag_count": len(self._baseline_flags),
            "aborted": aborted,
        }

    def _build_meta(self, samples_all, baselines, correction, aborted: bool,
                    samples_name: str) -> dict:
        valid = [s for s in samples_all if s.get("cv") == 1]
        vbats = [s["vbat_v"] for s in valid if s.get("vbat_v", 0.0) > 0.5]
        temps = [s["imu_temp_c"] for s in samples_all
                 if isinstance(s.get("imu_temp_c"), float)
                 and math.isfinite(s["imu_temp_c"])]
        idle_rows = [s["current_a"] for s in valid if s.get("phase") == "base"]
        geomag = load_geomagnetic_config()
        return {
            "schema": "stampfly_sweep_meta",
            "version": 1,
            "method": "bracketed_baseline",
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "started_at_epoch": self.started_at,
            "aborted": aborted,
            "samples_csv": samples_name,
            "motors_mask": self.motor_mask,
            "motors": motors_label(self.motor_mask),
            "pattern": self.pattern,
            "duty_sequence": [
                {"step_idx": i, "duty": d, "leg": leg}
                for i, (d, leg) in enumerate(self.duty_sequence)
            ],
            "timing_s": {
                "base": SWEEP_BASE_S,
                "settle": SWEEP_SETTLE_S,
                "measure": SWEEP_MEASURE_S,
                "gap": SWEEP_GAP_S,
                "gap_settle": SWEEP_GAP_SETTLE_S,
            },
            "abort_thresholds": {
                "undervolt_v": SWEEP_UNDERVOLT_ABORT_V,
                "overcurrent_a": SWEEP_OVERCURRENT_ABORT_A,
                "link_timeout_s": SWEEP_LINK_TIMEOUT_S,
            },
            "notes": self.notes,
            "battery": {
                "vbat_start_v": round(vbats[0], 3) if vbats else None,
                "vbat_end_v": round(vbats[-1], 3) if vbats else None,
                "vbat_min_v": round(min(vbats), 3) if vbats else None,
            },
            "idle_current_a": (round(sum(idle_rows) / len(idle_rows), 4)
                               if idle_rows else None),
            "imu_temp_c": {
                "start": round(temps[0], 2) if temps else None,
                "end": round(temps[-1], 2) if temps else None,
                "min": round(min(temps), 2) if temps else None,
                "max": round(max(temps), 2) if temps else None,
            },
            # 本ランで b_cor に実際に適用した mag3d スナップショット。後から
            # mag3d を取り直しても、この snapshot と raw 列から dB_cor を再現可能。
            "mag3d": correction,
            "mag3d_file_info": mag3d_file_info(),
            "geomag_profile": (geomag.get("profile")
                               if isinstance(geomag, dict) and "error" not in geomag
                               else ({"error": geomag.get("error")}
                                     if isinstance(geomag, dict) else None)),
            "baseline_points": [
                {"index": i, "duty": b.get("duty"), "t_s": b.get("t"),
                 "n": b.get("n"), "cor": b.get("cor"), "raw": b.get("raw")}
                for i, b in enumerate(baselines)
            ],
            "baseline_jump_warn_uT": SWEEP_BASELINE_JUMP_WARN_UT,
            "baseline_jumps": self._baseline_jumps,
            "baseline_flags": self._baseline_flags,
            "sample_count": len(samples_all),
        }

class SequenceRunner:
    """複数マスクのスイープを連続実行する薄いオーケストレーション層
    (yaw側 SequenceRunner 移植)。各スイープ自体は完全に無変更で、
    (1) モーター停止クールダウン (2) 電池ガード(vbat ≥ しきい値、
    不足時は交換待ち一時停止 → 明示的な再開)(3) SweepRunner 実行と完了待ち
    (4) 生成ファイルを束ねる sequence meta JSON の書き出し、のみを行う。"""

    def __init__(self, hub: ExperimentHub) -> None:
        self.hub = hub
        self.lock = threading.Lock()
        self.thread: Optional[threading.Thread] = None
        self.abort_event = threading.Event()
        self.resume_event = threading.Event()
        self._reset_state()

    def _reset_state(self) -> None:
        self.running = False
        self.phase = "idle"  # idle/cooldown/waiting_battery/sweeping/done/aborted/error
        self.plan: list[int] = list(SEQUENCE_DEFAULT_PLAN)
        self.index = 0
        self.pattern = SWEEP_DEFAULT_PATTERN
        self.notes: dict[str, str] = {}
        self.min_start_vbat = SEQUENCE_MIN_START_VBAT_V
        self.run_id = ""
        self.message = ""
        self.error: Optional[str] = None
        self.started_at = 0.0
        self.results: list[dict] = []
        self.last_meta: Optional[str] = None
        self._force_once = False

    # ---- 公開 API ----

    def is_running(self) -> bool:
        with self.lock:
            return self.running

    def abort_if_running(self) -> None:
        if self.is_running():
            self.abort()

    def start(self, masks: Any = None, pattern: Any = None, notes: Any = None,
              min_start_vbat: Any = None) -> dict:
        plan = self._clamp_plan(masks)
        if not plan:
            return {**self.status(), "ok": False,
                    "message": "有効なモーターマスクがありません"}
        if not self.hub.active:
            return {**self.status(), "ok": False,
                    "message": "実験モードが有効ではありません"}
        age = self.hub.exp_age_s()
        if age is None or age > SWEEP_LINK_TIMEOUT_S:
            return {**self.status(), "ok": False,
                    "message": "実験テレメトリ(TLM_EXP)がありません"}
        try:
            threshold = float(min_start_vbat)
        except (TypeError, ValueError):
            threshold = SEQUENCE_MIN_START_VBAT_V
        threshold = max(SEQUENCE_VBAT_CLAMP_MIN_V,
                        min(SEQUENCE_VBAT_CLAMP_MAX_V, threshold))
        with self.hub.start_gate:
            if self.hub.sweep.is_running():
                return {**self.status(), "ok": False,
                        "message": "単発スイープの実行中はシーケンスを開始できません"}
            if self.hub.recorder.is_recording():
                # 計測(EKF/FF性能ログ)とシーケンスは排他(T1-4)
                return {**self.status(), "ok": False,
                        "message": "計測(EKF/FF性能ログ)の実行中はシーケンスを"
                                   "開始できません。先に計測を停止してください"}
            if self.hub.calprofile_inflight:
                return {**self.status(), "ok": False,
                        "message": "キャリブレーション/FFプロファイル操作の実行中は"
                                   "シーケンスを開始できません"}
            with self.hub.lock:
                manual_motor = self.hub.motor_running
            if manual_motor:
                return {**self.status(), "ok": False,
                        "message": "手動モーター回転中はシーケンスを開始できません。"
                                   "先に Stop してください"}
            with self.lock:
                if self.running:
                    return {**self._status_locked(), "ok": False,
                            "message": "シーケンスは既に実行中です"}
                self.abort_event.clear()
                self.resume_event.clear()
                self._reset_state()
                self.plan = plan
                self.pattern = clamp_sweep_pattern(pattern)
                self.notes = sanitize_notes(notes)
                self.min_start_vbat = threshold
                self.running = True
                self.phase = "starting"
                self.started_at = time.time()
                self.run_id = time.strftime("%Y%m%d_%H%M%S",
                                            time.localtime(self.started_at))
                self.message = "シーケンス開始"
                self.thread = threading.Thread(target=self._run, daemon=True)
                self.thread.start()
        return {**self.status(), "ok": True, "message": "シーケンスを開始しました"}

    def abort(self) -> dict:
        self.abort_event.set()
        self.resume_event.set()   # 電池交換待ちのブロック解除
        if self.hub.sweep.is_running():
            self.hub.sweep.abort()
        else:
            self.hub.motor_stop()
        with self.lock:
            if self.running:
                self.message = "中断要求を受け付けました"
        return {**self.status(), "ok": True}

    def resume(self, force: bool = False) -> dict:
        with self.lock:
            if not self.running or self.phase != "waiting_battery":
                return {**self._status_locked(), "ok": False,
                        "message": "電池交換待ちではありません"}
            self._force_once = bool(force)
        self.resume_event.set()
        return {**self.status(), "ok": True, "message": "電圧を再確認します"}

    def status(self) -> dict:
        with self.lock:
            state = self._status_locked()
        state["config"] = {
            "default_plan": [{"mask": m, "motors": motors_label(m)}
                             for m in SEQUENCE_DEFAULT_PLAN],
            "min_start_vbat_v": SEQUENCE_MIN_START_VBAT_V,
            "cooldown_s": SEQUENCE_COOLDOWN_S,
        }
        return state

    def _status_locked(self) -> dict:
        return {
            "running": self.running,
            "phase": self.phase,
            "index": self.index,
            "total": len(self.plan),
            "plan": [{"mask": m, "motors": motors_label(m)} for m in self.plan],
            "pattern": self.pattern,
            "min_start_vbat_v": self.min_start_vbat,
            "run_id": self.run_id,
            "elapsed_s": (time.time() - self.started_at) if self.started_at else 0.0,
            "message": self.message,
            "error": self.error,
            "results": self.results,
            "last_meta": self.last_meta,
        }

    @staticmethod
    def _clamp_plan(masks: Any) -> list[int]:
        if not isinstance(masks, list) or not masks:
            return list(SEQUENCE_DEFAULT_PLAN)
        plan = []
        for value in masks[:16]:
            mask = clamp_motor_mask(value, default=0)
            if mask:
                plan.append(mask)
        return plan

    # ---- スレッド本体 ----

    def _set_phase(self, phase: str, index: Optional[int] = None,
                   message: Optional[str] = None) -> None:
        with self.lock:
            self.phase = phase
            if index is not None:
                self.index = index
            if message is not None:
                self.message = message

    def _finish(self, phase: str, message: str,
                error: Optional[str] = None) -> None:
        with self.lock:
            self.running = False
            self.phase = phase
            self.message = message
            self.error = error

    def _check_abort(self) -> None:
        if self.abort_event.is_set():
            raise SweepAborted()

    def _wait_abortable(self, duration: float) -> None:
        if self.abort_event.wait(timeout=duration):
            raise SweepAborted()

    def _live_vbat(self) -> tuple[Optional[float], bool]:
        sample, age = self.hub.latest_sample()
        link = age is not None and age < SWEEP_LINK_TIMEOUT_S
        if not link or sample is None:
            return None, link
        if sample.get("cv") != 1:
            return None, link
        vbat = float(sample.get("vbat_v", 0.0))
        return (vbat if vbat > 0.5 else None), link

    def _battery_gate(self, k: int, mask: int) -> None:
        """vbat ≥ しきい値(電池交換済み)/強制再開/中断までブロックする。"""
        while True:
            self._check_abort()
            vbat, link = self._live_vbat()
            if vbat is not None and vbat >= self.min_start_vbat:
                with self.lock:
                    self._force_once = False
                return
            with self.lock:
                force = self._force_once
                self._force_once = False
            if force and vbat is not None:
                return
            if vbat is None:
                detail = ("テレメトリ(電圧)待ち…" if link
                          else "リンク待ち…(電池交換中はテレメトリが途切れます)")
            else:
                detail = f"電圧 {vbat:.2f}V < {self.min_start_vbat:.2f}V"
            self._set_phase(
                "waiting_battery", index=k,
                message=f"{k + 1}本目({motors_label(mask)})開始前の電池ガード: "
                        f"{detail} — 満充電に交換して「再開」を押してください")
            self.resume_event.wait(timeout=0.5)
            self.resume_event.clear()

    def _wait_sweep_done(self) -> dict:
        while True:
            if self.abort_event.is_set() and self.hub.sweep.is_running():
                self.hub.sweep.abort()
            status = self.hub.sweep.status()
            if not status.get("running"):
                # スレッド起動前の "starting" スナップショットを誤検知しない
                if status.get("phase") in {"done", "aborted", "error"}:
                    return status
            time.sleep(0.3)

    def _run(self) -> None:
        try:
            # クールダウン/電池ガードの前に手動回転を必ず停止(ガードは
            # 数分待ち得るため、その間プロペラは停止していなければならない)
            self.hub.motor_stop()
            total = len(self.plan)
            for k, mask in enumerate(self.plan):
                self._check_abort()
                if k > 0 and SEQUENCE_COOLDOWN_S > 0:
                    self._set_phase(
                        "cooldown", index=k,
                        message=f"モーター冷却待ち {SEQUENCE_COOLDOWN_S:.0f}s"
                                f"({k}/{total} 完了)")
                    self._wait_abortable(SEQUENCE_COOLDOWN_S)
                self._battery_gate(k, mask)
                self._check_abort()
                self._set_phase("sweeping", index=k,
                                message=f"スイープ {k + 1}/{total}: {motors_label(mask)}")
                notes = dict(self.notes)
                notes["sequence"] = f"{self.run_id} {k + 1}/{total}"
                started = self.hub.sweep.start(mask, pattern=self.pattern,
                                               notes=notes, _internal=True)
                if not started.get("ok"):
                    raise SweepError(started.get("message")
                                     or "スイープを開始できませんでした")
                final = self._wait_sweep_done()
                result = dict(final.get("last_result") or {})
                result.update({"mask": mask, "motors": motors_label(mask),
                               "phase": final.get("phase")})
                with self.lock:
                    self.results.append(result)
                if final.get("phase") == "aborted":
                    raise SweepAborted()
                if final.get("phase") != "done":
                    raise SweepError(final.get("error") or final.get("message")
                                     or "スイープが失敗しました")
            meta_name = self._write_meta(completed=True)
            self._finish("done", f"シーケンス完了({total}本、{meta_name})")
        except SweepAborted:
            try:
                self._write_meta(completed=False)
            except OSError:
                pass
            self._finish("aborted", "シーケンスを中断しました")
        except SweepError as exc:
            try:
                self._write_meta(completed=False)
            except OSError:
                pass
            self._finish("error", str(exc), error=str(exc))
        except Exception as exc:  # 防御的バックストップ
            self.hub.motor_stop()
            self._finish("error", f"想定外のエラー: {exc}", error=str(exc))

    def _write_meta(self, completed: bool) -> str:
        result_dir = self.hub.sweep.result_dir
        result_dir.mkdir(parents=True, exist_ok=True)
        meta_name = f"sequence_{self.run_id}_meta.json"
        with self.lock:
            results = list(self.results)
        meta = {
            "schema": "stampfly_sweep_sequence_meta",
            "version": 1,
            "created_at": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
            "run_id": self.run_id,
            "completed": completed,
            "pattern": self.pattern,
            "notes": self.notes,
            "min_start_vbat_v": self.min_start_vbat,
            "cooldown_s": SEQUENCE_COOLDOWN_S,
            "plan": [{"mask": m, "motors": motors_label(m)} for m in self.plan],
            "runs": results,
        }
        (result_dir / meta_name).write_text(
            json.dumps(meta, ensure_ascii=False, indent=2), encoding="utf-8")
        with self.lock:
            self.last_meta = meta_name
        return meta_name
