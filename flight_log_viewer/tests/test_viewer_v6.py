"""v5/v6 ログの読み込み・ヨー解析図・レポートの回帰テスト。

- v6(136列。docs/MAG_AUTOTUNE_DESIGN.md §1.1/§3): EKF2 系統がヨー比較・
  統計に加わり、新パネル(21 EKF2 診断 / 22 磁気観測 / 23 フロー)が生成される。
- v5(115列): 従来どおり読めて、EKF2 系統・新パネルが出ない(デグレなし)。

合成 CSV は tests/make_dummy_log.py の generate_rows / write_csv を使う
(write_csv に V5_COLUMNS を渡すと v6 追加列を落とした v5 相当になる)。

実行: cd flight_log_viewer && .venv/bin/python -m pytest tests/ -q
"""

from __future__ import annotations

import os
import sys
from pathlib import Path

os.environ.setdefault("MPLBACKEND", "Agg")  # ヘッドレス環境でも図を生成できるように

# flight_log_viewer/ を import パスに追加(make_dummy_log と同じ流儀)
_HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(_HERE.parent))

import pytest  # noqa: E402

from make_dummy_log import generate_rows, write_csv  # noqa: E402
from viewer.constants import V5_COLUMNS, V6_COLUMNS  # noqa: E402
from viewer.loader import FlightLog, load_log  # noqa: E402
from viewer.report import generate_comparison, generate_report  # noqa: E402
from viewer.yaw_analysis import (  # noqa: E402
    compute_yaw_stats,
    generate_yaw_figures,
)

DURATION_S = 8.0  # 短い合成ログ(全フェーズを含む最小限)

# v6 新パネルのファイル名
V6_FIGURES = ("21_ekf2_diagnostics.png", "22_mag_observation.png",
              "23_flow.png")


@pytest.fixture(scope="module")
def rows() -> list[dict]:
    return generate_rows("position", DURATION_S)


@pytest.fixture(scope="module")
def v6_log(rows, tmp_path_factory) -> FlightLog:
    path = tmp_path_factory.mktemp("v6") / "20260727_000000_position.csv"
    write_csv(rows, path)  # 既定 = V6_COLUMNS
    return load_log(path)


@pytest.fixture(scope="module")
def v5_log(rows, tmp_path_factory) -> FlightLog:
    path = tmp_path_factory.mktemp("v5") / "20260727_000001_position.csv"
    write_csv(rows, path, columns=V5_COLUMNS)
    return load_log(path)


# ---------------------------------------------------------------------------
# 読み込み(loader)
# ---------------------------------------------------------------------------

class TestLoader:
    def test_v6_loads_without_warnings(self, v6_log):
        """v6 の全列が契約内(「契約に無い列」警告なし)で読めること。"""
        assert v6_log.warnings == []
        for col in V6_COLUMNS:
            assert col in v6_log.df.columns, f"v6 列が欠落: {col}"

    def test_v6_derives_ekf2_columns(self, v6_log):
        """EKF2 の deg/アンラップ/対基準誤差の派生列が生成されること。"""
        for col in ("yaw_ekf2_deg", "yaw_ekf2_unwrap_deg", "yaw_err_ekf2_deg"):
            assert col in v6_log.df.columns
            assert v6_log.df[col].notna().any()

    def test_v6_text_column_yaw_ref_source(self, v6_log):
        """yaw_ref_source は文字列のまま保持されること(数値化しない)。"""
        values = v6_log.df["yaw_ref_source"].dropna().unique().tolist()
        assert values == ["mocap"]

    def test_v5_loads_without_warnings(self, v5_log):
        """v5 ログが従来どおり警告なしで読めること(デグレなし)。"""
        assert v5_log.warnings == []

    def test_v5_has_no_ekf2_columns(self, v5_log):
        """v5 では EKF2 系統の列・派生列が生成されないこと。"""
        for col in ("tlm_ekf2_yaw_rad", "yaw_ekf2_deg", "yaw_err_ekf2_deg"):
            assert col not in v5_log.df.columns


# ---------------------------------------------------------------------------
# 統計(compute_yaw_stats)
# ---------------------------------------------------------------------------

class TestYawStats:
    def test_v6_stats_include_ekf2(self, v6_log):
        stats = compute_yaw_stats(v6_log)
        keys = [s.key for s in stats["errors"]]
        assert "ekf2" in keys
        assert "ekf" in keys  # 既存系統は不変
        # EKF2 診断(v6 のみ有効値)
        assert stats["bm2_max_ut"] == stats["bm2_max_ut"]  # not NaN
        assert stats["ekf2_innov_rms_deg"] == stats["ekf2_innov_rms_deg"]
        assert 0.0 < stats["ekf2_fused_rate"] < 100.0
        assert stats["gate2_rates"]  # ffg と同一ビット定義で集計される

    def test_v5_stats_have_no_ekf2(self, v5_log):
        stats = compute_yaw_stats(v5_log)
        keys = [s.key for s in stats["errors"]]
        assert "ekf2" not in keys
        assert stats["bm2_max_ut"] != stats["bm2_max_ut"]  # NaN
        assert stats["gate2_rates"] == {}
        # 既存統計はデグレなし
        assert stats["reference"] == "mocap"
        assert stats["bm_max_ut"] == stats["bm_max_ut"]


# ---------------------------------------------------------------------------
# 図(generate_yaw_figures)
# ---------------------------------------------------------------------------

class TestYawFigures:
    def test_v6_generates_new_panels(self, v6_log, tmp_path):
        paths = generate_yaw_figures(v6_log, tmp_path)
        names = {p.name for p in paths}
        for fig_name in V6_FIGURES:
            assert fig_name in names, f"v6 新パネルが生成されない: {fig_name}"
        assert "10_yaw_comparison.png" in names

    def test_v5_skips_new_panels(self, v5_log, tmp_path):
        paths = generate_yaw_figures(v5_log, tmp_path)
        names = {p.name for p in paths}
        for fig_name in V6_FIGURES:
            assert fig_name not in names, f"v5 で v6 パネルが生成された: {fig_name}"
        # 既存パネルはデグレなし
        for fig_name in ("10_yaw_comparison.png", "12_ekf_diagnostics.png",
                         "13_ff_status.png", "14_yaw_tracking.png"):
            assert fig_name in names


# ---------------------------------------------------------------------------
# レポート(generate_report / generate_comparison)
# ---------------------------------------------------------------------------

class TestReport:
    def test_v6_report_includes_ekf2(self, v6_log, tmp_path):
        txt_path, html_path = generate_report(v6_log, tmp_path)
        txt = txt_path.read_text(encoding="utf-8")
        assert "EKF2 (シャドー)" in txt      # ヨー統計表の系統行
        assert "EKF2 健全性" in txt          # EKF2 健全性ブロック
        html = html_path.read_text(encoding="utf-8")
        assert "EKF2 健全性" in html

    def test_v5_report_has_no_ekf2(self, v5_log, tmp_path):
        txt_path, _html_path = generate_report(v5_log, tmp_path)
        txt = txt_path.read_text(encoding="utf-8")
        assert "EKF2" not in txt

    def test_comparison_v6_vs_v5(self, v6_log, v5_log, tmp_path):
        """v6 と v5 の比較レポートが生成でき、EKF2 行が出ること。"""
        html_path = generate_comparison(v6_log, v5_log, tmp_path)
        html = html_path.read_text(encoding="utf-8")
        assert "EKF2" in html
