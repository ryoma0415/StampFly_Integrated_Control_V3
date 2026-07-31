"""CSV フライトログ(ログ ON 時のみ)。

- 出力先: リポジトリ直下 logs/flight_logs/YYYYMMDD_HHMMSS_<mode>.csv
- 寿命: START(CMD_START 受理)〜飛行終了(着陸/START 猶予切れ/切断)。
  開閉は session.py(_open_flight_log / _finish_flight_log)が管理する。
  複数機モード(mode="multi")はスロットごとに1インスタンスを持ち、
  MultiControlManager(open_flight_logs / close_flight_logs)が管理する。
- 50Hz の制御行(CMD_POS_ERR / CMD_SETPOINT 送信ごとに1行)+最新テレメトリ
  (TLM_STATE / TLM_CTRL とも 25Hz)/mocap スナップショットの結合。
  列定義は docs/LOG_STRUCTURE.md(現行 v6 = 136列)と1対1で対応させること。
- 値が未取得の列は空文字。0/1 フラグは文字列 "0"/"1"。
- 経過時間は time.monotonic() 基準。timestamp 列のみ壁時計(ISO8601)。
- プリロール(P2b、2026-07-31): ファイル閉鎖中に届いた行を直近 pre_roll_s
  (既定3s)ぶんリングバッファへ保持し、start() 時に先頭へフラッシュする
  (アーム前区間 — 地上アンカー・I_idle・Δb_z 復元に必須 — が必ずログに
  入る。FLIGHT_ANALYSIS_20260731.md §7-B-3)。elapsed_time の基点(0)は
  プリロール先頭行の時刻、START トリガー時刻は meta.json の pre_roll_s。
"""

from __future__ import annotations

import csv
import json
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path
from typing import Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from .config import LOGS_DIR

# 列定義 v6(136列 = v5 の115列+磁気オートチューン/フロー拡張21列。
# 末尾追加のみ: TLM_STATE 184B 拡張の tlm_* 16列(MAG_AUTOTUNE_DESIGN.md
# §1.1)+PC側ヨー基準/移動ベースヨー5列(同 §3)。
# docs/LOG_STRUCTURE.md と1対1で対応させること)
COLUMNS: tuple[str, ...] = (
    # --- セッション / タイミング(7) ---
    "timestamp", "elapsed_time", "mode", "phase",
    "command_sequence", "send_success", "feedback_latency_ms",
    # --- 目標と位置(制御座標系 m。Position/Multi のみ)(11) ---
    "target_x", "target_y", "target_z",
    "pos_x", "pos_y", "pos_z",
    "raw_pos_x", "raw_pos_y", "raw_pos_z",
    "error_x", "error_y",
    # --- 送信指令(PC→機体)(10) ---
    "cmd_err_x_m", "cmd_err_y_m", "cmd_xy_valid",
    "roll_ref_rad", "pitch_ref_rad", "alt_ref_m",
    "cmd_yaw_ref_rad", "yaw_ctrl_on",
    "roll_bias_deg", "pitch_bias_deg",
    # --- 機体実測: 姿勢・角速度・加速度(TLM_STATE)(10) ---
    "tlm_age_ms",
    "tlm_roll_rad", "tlm_pitch_rad", "tlm_yaw_rad",
    "tlm_p_rad_s", "tlm_q_rad_s", "tlm_r_rad_s",
    "tlm_ax_g", "tlm_ay_g", "tlm_az_g",
    # --- 機体計算指令(TLM_STATE + TLM_CTRL)(8) ---
    "tlm_roll_ref_rad", "tlm_pitch_ref_rad", "tlm_yaw_ref_rad",
    "tlm_ctrl_age_ms", "tlm_ctrl_flags",
    "tlm_roll_rate_ref_rad_s", "tlm_pitch_rate_ref_rad_s",
    "tlm_yaw_rate_ref_rad_s",
    # --- 姿勢PID成分(TLM_CTRL: 角度ループ9+角速度ループ9)(18) ---
    "tlm_pid_roll_ang_p", "tlm_pid_roll_ang_i", "tlm_pid_roll_ang_d",
    "tlm_pid_pitch_ang_p", "tlm_pid_pitch_ang_i", "tlm_pid_pitch_ang_d",
    "tlm_pid_yaw_ang_p", "tlm_pid_yaw_ang_i", "tlm_pid_yaw_ang_d",
    "tlm_pid_roll_rate_p", "tlm_pid_roll_rate_i", "tlm_pid_roll_rate_d",
    "tlm_pid_pitch_rate_p", "tlm_pid_pitch_rate_i", "tlm_pid_pitch_rate_d",
    "tlm_pid_yaw_rate_p", "tlm_pid_yaw_rate_i", "tlm_pid_yaw_rate_d",
    # --- 高度系(TLM_STATE)(5) ---
    "tlm_alt_ref_m", "tlm_altitude_tof_m", "tlm_altitude_est_m",
    "tlm_alt_velocity_m_s", "tlm_z_dot_ref_m_s",
    # --- ヨー推定・FF診断(TLM_STATE)(10) ---
    "tlm_yaw_est_rad", "tlm_yaw_gyro_int_rad",
    "tlm_current_a", "tlm_db_hat_x_ut", "tlm_db_hat_y_ut",
    "tlm_bm_x_ut", "tlm_bm_y_ut",
    "tlm_nis", "tlm_ffg", "tlm_ff_status",
    # --- モータ・電源(TLM_STATE)(5) ---
    "tlm_duty_fr", "tlm_duty_fl", "tlm_duty_rr", "tlm_duty_rl",
    "tlm_voltage_v",
    # --- 機体状態・システム(TLM_STATE)(6) ---
    "tlm_state", "tlm_flags", "tlm_reason", "tlm_seq_echo",
    "tlm_elapsed_ms", "tlm_loop_dt_us",
    # --- MoCap 実測ヨー・軌道(4) ---
    "mocap_yaw_deg", "mocap_heading_deg", "traj_mode", "traj_phase_rad",
    # --- フィルタ状態(Position/Multi のみ)(10) ---
    "data_valid", "control_active", "mocap_dropout", "is_outlier",
    "used_prediction",
    "confidence", "consecutive_outliers", "data_source",
    "filter_threshold", "tracking_valid",
    # --- リジッドボディ / フレーム診断(5) ---
    "rb_error", "rb_marker_count", "frame_number", "frame_dt_ms",
    "mocap_age_ms",
    # --- MoCap 正解ヨー・生クォータニオン(2026-07 追加。互換のため既存列の
    #     末尾に追加のみ: 旧ログは列不足として区別できる)(6) ---
    # mocap_yaw_true_deg: 符号/オフセット/フリップ補正済みの正解ヨー
    #   (-180, 180]。ビューアの「正解Yaw」系列はこの列を使う。
    # mocap_flip: フリップ補正フラグ(bit0=上方軸反転補正, bit1=ヨー180°補正)
    # mocap_q*: Motive 生クォータニオン — 任意のマッピングを事後再計算できる
    "mocap_yaw_true_deg", "mocap_flip",
    "mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw",
    # --- v6: 磁気オートチューン/フロー拡張(2026-07 追加。末尾追加のみ)---
    # TLM_STATE 135B→184B 拡張(契約 §1.1)の全フィールド(16)
    "tlm_mag_cal_x_ut", "tlm_mag_cal_y_ut", "tlm_mag_cal_z_ut",
    "tlm_mag_lev_x_ut", "tlm_mag_lev_y_ut",
    "tlm_ekf2_yaw_rad", "tlm_ekf2_bm_x_ut", "tlm_ekf2_bm_y_ut",
    "tlm_ekf2_yaw_innov_rad", "tlm_ekf2_status", "tlm_ekf2_gate",
    "tlm_flow_vx_mps", "tlm_flow_vy_mps",
    "tlm_flow_squal", "tlm_flow_status", "tlm_flow_dt_ms",
    # PC側: ヨー基準ソース/送信値/有効+移動ベースヨー(契約 §3)(5)
    # yaw_ref_source: off|mocap|motion。yaw_ref_sent_rad: CMD_POS_ERR の
    #   mocap_yaw 欄に実際に送った値(ワイヤ規約)。yaw_ref_valid: bit3。
    # motion_yaw_rad: 移動ベースヨー(ワイヤ規約変換済み。ソースが mocap の
    #   間も常時計算・記録する — 切替前の妥当性検証用)。motion_yaw_J:
    #   Fisher情報 Σ|u_B|²(ゲート判定値)。
    "yaw_ref_source", "yaw_ref_sent_rad", "yaw_ref_valid",
    "motion_yaw_rad", "motion_yaw_J",
)

FLOAT_DECIMALS = 6   # CSV 上の float 桁数

MS_PER_S = 1000.0

# プリロール既定値 [s](P2b)。server.json logging.pre_roll_s で上書き可。
# ≥3s: 地上アンカー(EKF2)・I_idle・Δb_z 復元に必要なアーム前区間
# (FLIGHT_ANALYSIS_20260731.md §7-B-3「ログはアーム前2s以上から開始」+余裕)
PRE_ROLL_DEFAULT_S = 3.0

# TLM_CTRL の PID 成分列の軸/項の並び(§2 契約: roll,pitch,yaw × p,i,d)
_PID_AXES = ("roll", "pitch", "yaw")
_PID_TERMS = ("p", "i", "d")

# PositionController の meta からそのままキー名一致で転記する診断列
# (session._build_log_row と multi の機体別ログで共通)
META_PASSTHROUGH_KEYS: tuple[str, ...] = (
    "error_x", "error_y", "target_x", "target_y", "target_z",
    "data_valid", "control_active", "mocap_dropout",
    "is_outlier", "used_prediction", "confidence",
    "consecutive_outliers", "data_source", "filter_threshold",
    "tracking_valid", "rb_error", "frame_number",
    "frame_dt_ms", "mocap_age_ms",
    "mocap_yaw_deg", "mocap_heading_deg",
    "mocap_yaw_true_deg", "mocap_flip",
    "mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw",
    "traj_mode", "traj_phase_rad",
)


def meta_to_row(meta: dict) -> dict:
    """PositionController の meta 辞書 → 位置/フィルタ診断列(欠損は省略)。"""
    row: dict = {}
    filtered = meta.get("filtered_pos")
    if filtered is not None:
        row["pos_x"], row["pos_y"], row["pos_z"] = filtered
    raw = meta.get("raw_pos")
    if raw is not None:
        row["raw_pos_x"], row["raw_pos_y"], row["raw_pos_z"] = raw
    for key in META_PASSTHROUGH_KEYS:
        if key in meta:
            row[key] = meta[key]
    if "marker_count" in meta:
        row["rb_marker_count"] = meta["marker_count"]
    return row


def tlm_state_to_row(tlm: proto.TlmState, age_s: float) -> dict:
    """最新 TLM_STATE のスナップショット → tlm_* 列(age_s は受信からの経過秒)。"""
    return {
        "tlm_age_ms": age_s * MS_PER_S,
        "tlm_seq_echo": tlm.seq_echo,
        "tlm_elapsed_ms": tlm.elapsed_ms,
        "tlm_state": tlm.state,
        "tlm_flags": tlm.flags,
        "tlm_reason": tlm.reason,
        "tlm_roll_rad": tlm.roll,
        "tlm_pitch_rad": tlm.pitch,
        "tlm_yaw_rad": tlm.yaw,
        "tlm_p_rad_s": tlm.p,
        "tlm_q_rad_s": tlm.q,
        "tlm_r_rad_s": tlm.r,
        "tlm_roll_ref_rad": tlm.roll_ref,
        "tlm_pitch_ref_rad": tlm.pitch_ref,
        "tlm_alt_ref_m": tlm.alt_ref,
        "tlm_altitude_tof_m": tlm.altitude_tof,
        "tlm_altitude_est_m": tlm.altitude_est,
        "tlm_alt_velocity_m_s": tlm.alt_velocity,
        "tlm_z_dot_ref_m_s": tlm.z_dot_ref,
        "tlm_voltage_v": tlm.voltage,
        "tlm_duty_fr": tlm.duty_fr,
        "tlm_duty_fl": tlm.duty_fl,
        "tlm_duty_rr": tlm.duty_rr,
        "tlm_duty_rl": tlm.duty_rl,
        "tlm_ax_g": tlm.ax,
        "tlm_ay_g": tlm.ay,
        "tlm_az_g": tlm.az,
        "tlm_loop_dt_us": tlm.loop_dt_us,
        # v2: TLM_STATE 末尾拡張(ヨー推定/FF 診断)
        "tlm_yaw_est_rad": tlm.yaw_est_rad,
        "tlm_yaw_gyro_int_rad": tlm.yaw_gyro_int_rad,
        "tlm_yaw_ref_rad": tlm.yaw_ref_rad,
        "tlm_current_a": tlm.current_a,
        "tlm_db_hat_x_ut": tlm.db_hat_x_ut,
        "tlm_db_hat_y_ut": tlm.db_hat_y_ut,
        "tlm_bm_x_ut": tlm.bm_x_ut,
        "tlm_bm_y_ut": tlm.bm_y_ut,
        "tlm_nis": tlm.nis,
        "tlm_ffg": tlm.ffg,
        "tlm_ff_status": tlm.ff_status,
        # v6: 磁気オートチューン/フロー拡張(TLM_STATE 184B。契約 §1.1)
        "tlm_mag_cal_x_ut": tlm.mag_cal_x_ut,
        "tlm_mag_cal_y_ut": tlm.mag_cal_y_ut,
        "tlm_mag_cal_z_ut": tlm.mag_cal_z_ut,
        "tlm_mag_lev_x_ut": tlm.mag_lev_x_ut,
        "tlm_mag_lev_y_ut": tlm.mag_lev_y_ut,
        "tlm_ekf2_yaw_rad": tlm.ekf2_yaw_rad,
        "tlm_ekf2_bm_x_ut": tlm.ekf2_bm_x_ut,
        "tlm_ekf2_bm_y_ut": tlm.ekf2_bm_y_ut,
        "tlm_ekf2_yaw_innov_rad": tlm.ekf2_yaw_innov_rad,
        "tlm_ekf2_status": tlm.ekf2_status,
        "tlm_ekf2_gate": tlm.ekf2_gate,
        "tlm_flow_vx_mps": tlm.flow_vx_mps,
        "tlm_flow_vy_mps": tlm.flow_vy_mps,
        "tlm_flow_squal": tlm.flow_squal,
        "tlm_flow_status": tlm.flow_status,
        "tlm_flow_dt_ms": tlm.flow_dt_ms,
    }


def tlm_ctrl_to_row(ctrl: proto.TlmCtrl, age_s: float) -> dict:
    """最新 TLM_CTRL のスナップショット → tlm_ctrl_* / tlm_pid_* 列。

    25Hz(制御行 50Hz の半分)のため、連続する2行が同じスナップショットを
    共有し得る(tlm_ctrl_age_ms で判別可能)。
    """
    row = {
        "tlm_ctrl_age_ms": age_s * MS_PER_S,
        "tlm_ctrl_flags": ctrl.flags,
        "tlm_roll_rate_ref_rad_s": ctrl.roll_rate_ref,
        "tlm_pitch_rate_ref_rad_s": ctrl.pitch_rate_ref,
        "tlm_yaw_rate_ref_rad_s": ctrl.yaw_rate_ref,
    }
    for a, axis in enumerate(_PID_AXES):
        for t, term in enumerate(_PID_TERMS):
            row[f"tlm_pid_{axis}_ang_{term}"] = ctrl.pid_ang[a * 3 + t]
            row[f"tlm_pid_{axis}_rate_{term}"] = ctrl.pid_rate[a * 3 + t]
    return row


def _format_cell(value) -> str:
    """セル値を CSV 文字列に変換する(None → 空、bool → 0/1)。"""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "1" if value else "0"
    if isinstance(value, float):
        return f"{value:.{FLOAT_DECIMALS}f}"
    return str(value)


class FlightLogger:
    """CSV フライトロガー。log_row() はスレッド安全(50Hz送信スレッドから呼ぶ)。"""

    def __init__(self, logs_dir: Path = LOGS_DIR, flush_every_rows: int = 50,
                 pre_roll_s: float = PRE_ROLL_DEFAULT_S) -> None:
        self._logs_dir = Path(logs_dir)
        self._flush_every_rows = flush_every_rows
        self._pre_roll_s = max(0.0, float(pre_roll_s))
        self._lock = threading.Lock()
        self._file = None
        self._writer: Optional[csv.writer] = None
        self._file_path: Optional[Path] = None
        self._t0: Optional[float] = None
        self._rows_since_flush = 0
        # プリロール・リングバッファ(P2b): ファイル閉鎖中に届いた行を
        # (iso_timestamp, t_mono, row) で直近 pre_roll_s ぶん保持する。
        # stop() では消さない(トリガー前区間の保持が目的のため)
        self._preroll: deque = deque()
        # meta.json サイドカーの内容(update_metadata の追記ベース)
        self._metadata: Optional[dict] = None

    @property
    def active(self) -> bool:
        with self._lock:
            return self._file is not None

    @property
    def file_path(self) -> Optional[Path]:
        with self._lock:
            return self._file_path

    def start(self, mode: str, stamp: Optional[str] = None,
              metadata: Optional[dict] = None) -> Path:
        """ログファイルを作成しヘッダを書く。既に開いていれば閉じてから開く。

        stamp を指定すると そのタイムスタンプ文字列でファイル名を作る
        (複数機モードで全機のファイル名を同一時刻に揃えるため)。

        metadata を指定すると同名の ``*.meta.json`` サイドカーに書き出す
        (適用中の MoCap マッピング等 — 「このログはどのフレームで取った
        ものか」を CSV 列契約を変えずに恒久記録する)。書き込み失敗は
        ログ本体を妨げない(サイドカーは診断情報であり必須ではない)。

        P2b: 閉鎖中にバッファされたプリロール行(直近 pre_roll_s 以内)を
        ヘッダ直後へフラッシュする。elapsed_time の基点(0)はプリロール
        先頭行の時刻となり、開始トリガー時点は metadata の ``pre_roll_s``
        (トリガーまでの秒数)として記録される(プリロール行なしなら 0)。
        """
        self.stop()
        self._logs_dir.mkdir(parents=True, exist_ok=True)
        if stamp is None:
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        path = self._logs_dir / f"{stamp}_{mode}.csv"
        with self._lock:
            self._file = open(path, "w", newline="", encoding="utf-8")
            self._writer = csv.writer(self._file)
            self._writer.writerow(COLUMNS)
            # P2b: プリロール行のフラッシュ(期限切れは捨てる。log_row 側の
            # 追記時 evict は「最後の追記から」の経過を見ないため、ここでの
            # トリガー時刻基準の再フィルタが正)
            trigger_mono = time.monotonic()
            preroll = [entry for entry in self._preroll
                       if trigger_mono - entry[1] <= self._pre_roll_s]
            self._preroll.clear()
            t0 = preroll[0][1] if preroll else trigger_mono
            for iso_ts, t_mono, row in preroll:
                values = dict(row)
                values.setdefault("timestamp", iso_ts)
                values.setdefault("elapsed_time", t_mono - t0)
                self._writer.writerow(
                    [_format_cell(values.get(col)) for col in COLUMNS])
            self._file.flush()
            self._file_path = path
            self._t0 = t0
            self._rows_since_flush = 0
            if metadata is not None:
                metadata = dict(metadata)
                metadata["pre_roll_s"] = round(trigger_mono - t0, 3)
            self._metadata = dict(metadata) if metadata is not None else None
        if metadata is not None:
            self._write_metadata(path.with_suffix(".meta.json"), metadata)
        return path

    @staticmethod
    def _write_metadata(meta_path: Path, metadata: dict) -> None:
        """サイドカー書き出し。失敗はログ本体を妨げない(診断情報のため)。"""
        try:
            with open(meta_path, "w", encoding="utf-8") as fp:
                json.dump(metadata, fp, ensure_ascii=False, indent=2)
                fp.write("\n")
        except OSError:
            pass

    def update_metadata(self, updates: dict) -> None:
        """開いているログの meta.json サイドカーへキーを追記・上書きする。

        ログ開始後に確定する値(例: アーム相対ヨーアンカーはアーム遷移で
        ラッチされる — 開始時点では未定)を恒久記録するための追記経路。
        トップレベルキーの浅いマージで全体を書き直す。ログが閉じていれば
        何もしない。50Hz 送信スレッドから呼ばれ得るが、書くのは小さな
        JSON 1個で頻度はイベント時のみ(log_row の flush と同等の負荷)。
        """
        with self._lock:
            if self._file is None or self._file_path is None:
                return
            if self._metadata is None:
                self._metadata = {}
            self._metadata.update(updates)
            metadata = dict(self._metadata)
            meta_path = self._file_path.with_suffix(".meta.json")
        self._write_metadata(meta_path, metadata)

    def stop(self) -> None:
        with self._lock:
            file = self._file
            self._file = None
            self._writer = None
            self._file_path = None
            self._t0 = None
            self._metadata = None
        if file is not None:
            try:
                file.close()
            except OSError:
                pass

    def log_row(self, row: dict) -> None:
        """1行を書き込む。row は COLUMNS のキーを持つ dict(欠損キーは空欄)。

        timestamp / elapsed_time は本メソッドが自動付与する。
        ファイル閉鎖中はプリロール・リングバッファへ保持する(P2b。
        直近 pre_roll_s ぶん。次の start() で先頭へフラッシュされる)。
        """
        with self._lock:
            if self._writer is None or self._t0 is None:
                if self._pre_roll_s <= 0.0:
                    return
                t_mono = time.monotonic()
                self._preroll.append(
                    (datetime.now().isoformat(timespec="milliseconds"),
                     t_mono, dict(row)))
                while (self._preroll
                       and t_mono - self._preroll[0][1] > self._pre_roll_s):
                    self._preroll.popleft()
                return
            values = dict(row)
            values.setdefault("timestamp", datetime.now().isoformat(timespec="milliseconds"))
            values.setdefault("elapsed_time", time.monotonic() - self._t0)
            self._writer.writerow([_format_cell(values.get(col)) for col in COLUMNS])
            self._rows_since_flush += 1
            if self._rows_since_flush >= self._flush_every_rows:
                self._file.flush()
                self._rows_since_flush = 0
