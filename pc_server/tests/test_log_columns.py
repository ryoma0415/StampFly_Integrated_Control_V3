"""ログ列契約 v5: 115 列(順序は docs/LOG_STRUCTURE.md と 1 対 1)。

v3 までの「末尾追加のみ」契約は v4 で破棄し、列の削除+追加+論理的な
並び替えを行った(TLM_CTRL 追加に伴う全面再編)。v5 は v4 の 109 列を
不変のまま、MoCap 正解ヨー/生クォータニオン 6 列を末尾に追加した
(旧ログとは列数で判別できる)。本テストが順序の正。
"""

from __future__ import annotations

import pytest

from core.logger import COLUMNS, tlm_ctrl_to_row

from fakes import make_tlm_ctrl

# v4 の完全な列順(仕様 §2。これと COLUMNS の完全一致が契約)
V4_COLUMNS = (
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
)

# v5 で末尾に追加された MoCap 正解ヨー/生クォータニオン列(6)。
# mocap_yaw_true_deg はフリップ補正+sign/offset 適用済みの (-180,180]、
# mocap_q* は Motive 生クォータニオン(任意マッピングの事後再計算用)。
V5_APPENDED_COLUMNS = (
    "mocap_yaw_true_deg", "mocap_flip",
    "mocap_qx", "mocap_qy", "mocap_qz", "mocap_qw",
)

# v4 で削除された v3 列(残存すれば契約違反)
V3_REMOVED_COLUMNS = (
    "roll_ref_deg", "pitch_ref_deg", "cmd_yaw_ref_deg", "marker_count",
    "tlm_state_name", "tlm_reason_name", "cmd_mocap_yaw_deg",
    "pid_x_p", "pid_x_i", "pid_x_d", "pid_y_p", "pid_y_i", "pid_y_d",
    "xy_cmd_mode",
)


def test_column_count_is_115():
    assert len(V4_COLUMNS) == 109
    assert len(COLUMNS) == 115


def test_columns_match_v5_contract_order():
    """v5 = v4 の 109 列(順序不変)+末尾6列。旧ツールは先頭 109 列を
    そのまま読める(末尾追加のみ)。"""
    assert COLUMNS == V4_COLUMNS + V5_APPENDED_COLUMNS


def test_removed_v3_columns_absent():
    for name in V3_REMOVED_COLUMNS:
        assert name not in COLUMNS, name


def test_tlm_ctrl_to_row_covers_all_ctrl_columns():
    """TLM_CTRL 由来の 23 列がヘルパで漏れなく・正順で転記される。"""
    ctrl = make_tlm_ctrl()
    row = tlm_ctrl_to_row(ctrl, age_s=0.012)
    assert row["tlm_ctrl_age_ms"] == pytest.approx(12.0)
    assert row["tlm_ctrl_flags"] == ctrl.flags
    assert row["tlm_roll_rate_ref_rad_s"] == pytest.approx(0.10)
    assert row["tlm_pitch_rate_ref_rad_s"] == pytest.approx(-0.20)
    assert row["tlm_yaw_rate_ref_rad_s"] == pytest.approx(0.05)
    # pid_ang / pid_rate は roll_p,i,d → pitch → yaw の順で 9 要素ずつ
    for i, (axis, term) in enumerate(
            (a, t) for a in ("roll", "pitch", "yaw") for t in ("p", "i", "d")):
        assert row[f"tlm_pid_{axis}_ang_{term}"] == \
            pytest.approx(ctrl.pid_ang[i]), (axis, term)
        assert row[f"tlm_pid_{axis}_rate_{term}"] == \
            pytest.approx(ctrl.pid_rate[i]), (axis, term)
    # 転記キーはすべて COLUMNS に存在する
    for key in row:
        assert key in COLUMNS, key
