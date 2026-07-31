"""V2 フライトログの列定義・ビット定義・描画スタイル定数。

列定義は docs/LOG_STRUCTURE.md(v4・109列)= pc_server/core/logger.py の
COLUMNS と 1 対 1 で対応させること。ffg のビット定義は
`firmware_stampfly/src/yaw_estimation/yaw_estimator_kf.hpp`(出自は yaw 側
Yaw_Calibration_and_Estimation、同フォルダは削除済み)を、
ff_status のビット定義はプロトコル v2(TLM_STATE 末尾拡張)を、
tlm_ctrl_flags のビット定義は TLM_CTRL(プロトコル 0x35)を踏襲する。
"""

from __future__ import annotations

# ---------------------------------------------------------------------------
# 列定義(v4・109列。順序も logger.py と同一)
# v3(100列)からの変更: deg 重複列・PC 側 XY PID 列・xy_cmd_mode 等 14 列を
# 削除し、TLM_CTRL 由来の指令角速度+姿勢 PID 成分 23 列を追加、論理順に再編。
# ---------------------------------------------------------------------------

V4_COLUMNS: tuple[str, ...] = (
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
    "tlm_roll_rate_ref_rad_s", "tlm_pitch_rate_ref_rad_s", "tlm_yaw_rate_ref_rad_s",
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
    "tlm_current_a", "tlm_db_hat_x_ut", "tlm_db_hat_y_ut", "tlm_bm_x_ut", "tlm_bm_y_ut",
    "tlm_nis", "tlm_ffg", "tlm_ff_status",
    # --- モータ・電源(TLM_STATE)(5) ---
    "tlm_duty_fr", "tlm_duty_fl", "tlm_duty_rr", "tlm_duty_rl", "tlm_voltage_v",
    # --- 機体状態・システム(TLM_STATE)(6) ---
    "tlm_state", "tlm_flags", "tlm_reason", "tlm_seq_echo", "tlm_elapsed_ms",
    "tlm_loop_dt_us",
    # --- MoCap 実測ヨー・軌道(4) ---
    "mocap_yaw_deg", "mocap_heading_deg", "traj_mode", "traj_phase_rad",
    # --- フィルタ状態(Position/Multi のみ)(10) ---
    "data_valid", "control_active", "mocap_dropout", "is_outlier", "used_prediction",
    "confidence", "consecutive_outliers", "data_source",
    "filter_threshold", "tracking_valid",
    # --- リジッドボディ / フレーム診断(5) ---
    "rb_error", "rb_marker_count", "frame_number", "frame_dt_ms", "mocap_age_ms",
)
N_COLUMNS = len(V4_COLUMNS)
assert N_COLUMNS == 109, f"列数が契約(109列)と不一致: {N_COLUMNS}"

# v5 で末尾に追加された列(2026-07: MoCap 正解ヨー・生クォータニオン)。
# 読み込みは列名ベース(pandas)のため v4 ログもそのまま読める — これらの
# 列が無いログでは対応する系統・図が自動的にスキップされる。
V5_APPENDED_COLUMNS: tuple[str, ...] = (
    "mocap_yaw_true_deg", "mocap_flip",
    "mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw",
)
V5_COLUMNS: tuple[str, ...] = V4_COLUMNS + V5_APPENDED_COLUMNS
assert len(V5_COLUMNS) == 115, f"列数が契約(115列)と不一致: {len(V5_COLUMNS)}"

# v6 で末尾に追加された列(2026-07: 磁気オートチューン/EKF2/フロー。
# 契約は docs/MAG_AUTOTUNE_DESIGN.md §1.1(TLM_STATE 184B 拡張)+§3
# (PC 側列)。TLM 由来列は protocol のフィールド名に tlm_ prefix。
# v5 以前のログでは列が無く、対応する系統・図が自動スキップされる。
V6_APPENDED_COLUMNS: tuple[str, ...] = (
    # --- TLM_STATE v6 拡張(契約 §1.1 のワイヤ順)(16) ---
    "tlm_mag_cal_x_ut", "tlm_mag_cal_y_ut", "tlm_mag_cal_z_ut",
    "tlm_mag_lev_x_ut", "tlm_mag_lev_y_ut",
    "tlm_ekf2_yaw_rad", "tlm_ekf2_bm_x_ut", "tlm_ekf2_bm_y_ut",
    "tlm_ekf2_yaw_innov_rad", "tlm_ekf2_status", "tlm_ekf2_gate",
    "tlm_flow_vx_mps", "tlm_flow_vy_mps",
    "tlm_flow_squal", "tlm_flow_status", "tlm_flow_dt_ms",
    # --- PC 側列(契約 §3: ヨー基準ソース/移動ベースヨー)(5) ---
    "yaw_ref_source", "yaw_ref_sent_rad", "yaw_ref_valid",
    "motion_yaw_rad", "motion_yaw_J",
)
V6_COLUMNS: tuple[str, ...] = V5_COLUMNS + V6_APPENDED_COLUMNS
assert len(V6_COLUMNS) == 136, f"列数が契約(136列)と不一致: {len(V6_COLUMNS)}"

# 数値変換しない(文字列のままにする)列
# (tlm_state_name / tlm_reason_name / xy_cmd_mode は v4 で廃止された列だが、
#  旧ログ v1〜v3 の読み込み互換のため文字列扱いを維持する)
TEXT_COLUMNS: frozenset[str] = frozenset(
    {"timestamp", "mode", "phase", "tlm_state_name", "tlm_reason_name",
     "data_source", "xy_cmd_mode", "yaw_ref_source"}
)

# ログの行レート [Hz](CMD_SETPOINT 送信ごとに1行)
LOG_RATE_HZ = 50.0

# ---------------------------------------------------------------------------
# ビット定義
# ---------------------------------------------------------------------------

# tlm_flags(TLM_STATE flags)のビット
TLM_FLAG_LOW_VOLTAGE = 1 << 0   # 低電圧
TLM_FLAG_SETPOINT_FRESH = 1 << 1  # セットポイント新鮮
TLM_FLAG_FLYING = 1 << 2        # 飛行中

# tlm_ctrl_flags(TLM_CTRL flags)のビット
TLM_CTRL_FLAG_XY_ONBOARD = 1 << 0  # 機上XY指令生成中(CMD_POS_ERR 経路)
TLM_CTRL_FLAG_YAW_CTRL = 1 << 1    # ヨー角制御アクティブ
TLM_CTRL_FLAG_FLYING = 1 << 2      # 飛行中

# tlm_ffg(EKF ゲート状態ビット。yaw 側 yaw_estimator_kf.hpp と一致)
# (表示名, 説明, 描画色) を bit0 から順に並べる
FFG_GATE_BITS: tuple[tuple[str, str, str], ...] = (
    ("R膨張(soft)", "NIS>5.99 / norm 8-20µT → R膨張して採用", "#d97706"),
    ("NIS棄却", "NIS>13.8 → 磁気更新スキップ→ジャイロ滑走", "#ef4444"),
    ("norm棄却", "|‖B_corr‖−‖B0‖|>20µT → スキップ", "#f97316"),
    ("z棄却", "|B_corr.z−B0.z|>12µT → スキップ", "#a855f7"),
    ("tilt>25°", "傾き過大 → スキップ", "#64748b"),
    ("b_m凍結", "‖b_m‖>20µT → 磁気更新凍結(要再アンカー)", "#dc2626"),
    ("ドリフト警告", "|db_m/dt|>0.3µT/s 10s継続", "#0ea5e9"),
    ("再捕捉中", "NIS棄却5s超継続 → Δψ≤3°/更新の制限付き更新で引き込み中", "#22c55e"),
)

# tlm_ff_status(プロトコル v2 TLM_STATE offset134)のビット
FF_STATUS_FF_MODE_MASK = 0x03     # bit0-1: ff_mode(0=off,1=A,2=B)
FF_STATUS_EST_EKF = 1 << 2        # bit2: est_mode(1=EKF)
FF_STATUS_ANCHOR_VALID = 1 << 3   # bit3: アンカー有効
FF_STATUS_FFCAL_LOADED = 1 << 4   # bit4: FF 係数ロード済み
FF_STATUS_YAW_CTRL_ACTIVE = 1 << 5  # bit5: ヨー角制御アクティブ
FF_STATUS_MAG_FRESH = 1 << 6      # bit6: 磁気サンプル新鮮
FF_STATUS_EST_EKF2 = 1 << 7       # bit7: est_mode==2(EKF2。契約 §1.1)

# ff_status のフラグビット(表示名, ビット位置, 描画色)。ff_mode は別途値表示。
FF_STATUS_FLAG_BITS: tuple[tuple[str, int, str], ...] = (
    ("est_mode=EKF", 2, "#0ea5e9"),
    ("anchor_valid", 3, "#22c55e"),
    ("ffcal_loaded", 4, "#a855f7"),
    ("yaw_ctrl_active", 5, "#f59e0b"),
    ("mag_fresh", 6, "#64748b"),
    ("est_mode=EKF2", 7, "#9333ea"),
)

# tlm_ekf2_status(v6・契約 §1.1)のビット
EKF2_STATUS_YAW_OBS_FRESH = 1 << 0  # ヨー観測受信 <1s
EKF2_STATUS_YAW_OBS_FUSED = 1 << 1  # 直近 0.5s 内に受理

# tlm_ekf2_status のビット(表示名, 説明, 描画色)を
# bit0 から順に並べる
EKF2_STATUS_BITS: tuple[tuple[str, str, str], ...] = (
    ("yaw_obs_fresh", "ヨー観測受信 <1s", "#22c55e"),
    ("yaw_obs_fused", "直近 0.5s 内に受理", "#0ea5e9"),
    ("flight_anchor", "飛行状態再アンカー実施済み", "#a855f7"),
    ("tau_rw_mode", "q_bm ランダムウォークモード", "#f59e0b"),
    ("bm_frozen", "b_m 凍結", "#dc2626"),
    ("healthy2", "EKF2 健全", "#16a34a"),
    ("low_trust", "ヨー観測低信頼(R_ψ 低信頼プリセット)", "#64748b"),
    ("yaw_recapture", "ヨー観測再捕捉中(制限融合モード)", "#ec4899"),
)

# tlm_flow_status(v6・契約 §1.1)のビット(表示名, 説明, 描画色)。
# bit6-7 は予約
FLOW_STATUS_BITS: tuple[tuple[str, str, str], ...] = (
    ("sensor_ok", "PMW3901 応答あり", "#22c55e"),
    ("burst_ok", "burst 読み成立(不成立ならフロー恒久無効)", "#0ea5e9"),
    ("vel_valid", "速度有効", "#16a34a"),
    ("range_valid", "レンジ有効", "#a855f7"),
    ("squal_ok", "SQUAL 良好", "#f59e0b"),
    ("init_retry", "初期化リトライ中", "#ef4444"),
)

# ---------------------------------------------------------------------------
# 解析しきい値(yaw 側 EKF ゲート・機体側フェイルセーフの定義値)
# ---------------------------------------------------------------------------

NIS_EXPECTED = 2.0            # NIS 期待値(2自由度)
NIS_R_INFLATE_THRESHOLD = 5.99   # χ²(2) 95% → R 膨張
NIS_REJECT_THRESHOLD = 13.8      # χ²(2) 99.9% → 棄却
BM_FREEZE_THRESHOLD_UT = 20.0    # ‖b_m‖ 凍結しきい値 [µT]
LOW_VOLTAGE_THRESHOLD_V = 3.34   # 機体の低電圧判定 [V]
LOOP_DT_NOMINAL_US = 2500.0      # 400Hz 制御ループの公称周期 [µs]

# ---------------------------------------------------------------------------
# 描画スタイル(白背景ライトテーマ)
# ---------------------------------------------------------------------------

FIG_BG = "#ffffff"   # Figure 背景色(白)
AX_BG = "#ffffff"    # Axes 背景色(白)
TEXT_COLOR = "#222222"  # 文字・軸ラベル色(濃色)
GRID_COLOR = "gray"
GRID_ALPHA = 0.3
FIG_DPI = 150        # 静止画の解像度
ANIM_DPI = 100       # アニメーションの解像度

COLORS: dict[str, str] = {
    # PID 成分
    "p": "#dc2626",
    "i": "#0d9488",
    "d": "#16a34a",
    # 軌跡・位置
    "trajectory": "#3498db",
    "raw_trajectory": "#7f8c8d",
    "target": "#ca8a04",
    "start": "#16a34a",
    "end": "#e74c3c",
    "current_pos": "#e74c3c",
    # XY 位置誤差
    "err_x": "#dc2626",
    "err_y": "#0d9488",
    # 姿勢(指令 vs 実測)
    "cmd_roll": "#dc2626",
    "cmd_pitch": "#0d9488",
    "meas_roll": "#ca8a04",
    "meas_pitch": "#9B59B6",
    "meas_yaw": "#2563eb",
    # PID 成分合計(=指令角速度)
    "pid_sum": "#111111",
    # ヨー4系統
    "yaw_madgwick": "#ca8a04",   # Madgwick(tlm_yaw_rad)
    "yaw_ekf": "#e74c3c",        # EKF(tlm_yaw_est_rad)
    "yaw_gyro": "#3498db",       # ジャイロ積算(tlm_yaw_gyro_int_rad)
    "yaw_mocap": "#16a34a",      # MoCap 正解Yaw(mocap_yaw_true_deg)
    "yaw_cmd": "#111111",        # PC ヨー指令(cmd_yaw_ref)
    "yaw_ref_applied": "#ea580c",  # 機体適用ヨー目標(tlm_yaw_ref_rad)
    # 高度
    "alt_ref": "#111111",
    "alt_tof": "#64748b",
    "alt_est": "#0284c7",
    # duty(FL/FR/RL/RR)
    "duty_fl": "#dc2626",
    "duty_fr": "#0d9488",
    "duty_rl": "#16a34a",
    "duty_rr": "#9333ea",
    # 電源
    "voltage": "#0284c7",
    "current": "#f97316",
    # 診断
    "nis": "#0284c7",
    "bm_x": "#2563eb",
    "bm_y": "#16a34a",
    "bm_norm": "#7c3aed",
    "dbhat_x": "#d97706",
    "dbhat_y": "#db2777",
    "latency": "#0284c7",
    "loop_dt": "#d97706",
    "marker": "#16a34a",
    # v6: EKF2 / 磁気観測 / フロー
    "yaw_ekf2": "#9333ea",       # EKF2 シャドーヨー(tlm_ekf2_yaw_rad)
    "ekf2_innov": "#db2777",     # EKF2 ヨー観測イノベーション
    "mag_cal_x": "#dc2626",      # b_cal 機体系 X
    "mag_cal_y": "#0d9488",      # 同 Y
    "mag_cal_z": "#2563eb",      # 同 Z
    "mag_lev_x": "#d97706",      # レベル化観測 zx
    "mag_lev_y": "#db2777",      # 同 zy
    "flow_vx": "#dc2626",        # フロー機体系 X 速度
    "flow_vy": "#0d9488",        # 同 Y
    "flow_squal": "#7c3aed",     # SQUAL
    "flow_dt": "#64748b",        # フロー読み実測 dt
}

# 複数機同時制御(multi)の機体別カラー(最大4機。M01 共有 XY 図などで
# 機体 index 順に割り当てる。既存 trajectory/end/start/target 系と調和する色)
MULTI_DRONE_COLORS: tuple[str, ...] = (
    "#3498db",  # 機体1: 青(trajectory と同系)
    "#e74c3c",  # 機体2: 赤
    "#16a34a",  # 機体3: 緑
    "#ca8a04",  # 機体4: 黄(白背景で読める濃色の黄)
)

# ヨー5系統の (キー名, 列名(rad or deg), 表示名, 色, 単位が deg か)
# mocap は v5 の mocap_yaw_true_deg(符号/オフセット/フリップ補正済みの
# 正解ヨー)。旧列 mocap_yaw_deg は Z-up 前提オイラー分解で Motive(Y-up)
# では機首方位でないことが 2026-07-27 実測で確定したため使わない
# (v4 以前のログでは mocap 系統は自動的に非表示となり、基準は Madgwick に
# フォールバックする)。ekf2 は v6 の EKF2 シャドー推定
# (tlm_ekf2_yaw_rad。契約 §1.1)— v5 以前のログでは自動スキップ。
YAW_SOURCES: tuple[tuple[str, str, str, str, bool], ...] = (
    ("madgwick", "tlm_yaw_rad", "Madgwick", COLORS["yaw_madgwick"], False),
    ("ekf", "tlm_yaw_est_rad", "EKF (アクティブ推定器)", COLORS["yaw_ekf"], False),
    ("ekf2", "tlm_ekf2_yaw_rad", "EKF2 (シャドー)", COLORS["yaw_ekf2"], False),
    ("gyro_int", "tlm_yaw_gyro_int_rad", "ジャイロ積算", COLORS["yaw_gyro"], False),
    ("mocap", "mocap_yaw_true_deg", "MoCap 正解Yaw", COLORS["yaw_mocap"], True),
)
