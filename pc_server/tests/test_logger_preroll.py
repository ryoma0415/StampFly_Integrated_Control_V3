"""飛行ログのプリロール(P2b)検証。

- FlightLogger: ファイル閉鎖中の log_row をリングバッファ(直近 pre_roll_s)
  に保持し、start() でヘッダ直後へフラッシュする。elapsed_time の基点は
  プリロール先頭行、meta.json にトリガーまでの秒数 pre_roll_s を記録。
- session 統合: ログ予約(set_logging ON)中の 50Hz 行が START 前でも
  組み立てられ、START で開いたファイルの先頭に入る(アーム前区間 —
  地上アンカー・I_idle・Δb_z 復元 — の確保。FLIGHT_ANALYSIS §7-B-3)。
"""

from __future__ import annotations

import csv
import json
import time

import pytest

from core.logger import COLUMNS, FlightLogger

from conftest import halt_supervisor


def _row(seq: int) -> dict:
    return {"mode": "posture", "phase": "connected", "command_sequence": seq}


def _read_rows(path) -> list[dict]:
    with open(path, newline="", encoding="utf-8") as fp:
        return list(csv.DictReader(fp))


class TestFlightLoggerPreroll:
    def test_preroll_rows_precede_live_rows(self, tmp_path):
        logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        logger.log_row(_row(1))
        logger.log_row(_row(2))
        path = logger.start("posture", metadata={"log_columns_version": 6})
        logger.log_row(_row(3))
        logger.stop()

        rows = _read_rows(path)
        assert [r["command_sequence"] for r in rows] == ["1", "2", "3"]
        elapsed = [float(r["elapsed_time"]) for r in rows]
        assert elapsed[0] == pytest.approx(0.0, abs=1e-6)   # 基点=プリロール先頭
        assert elapsed == sorted(elapsed)                   # 単調非減少
        assert all(r["timestamp"] for r in rows)
        # meta.json: 既存キー+pre_roll_s(トリガーまでの秒数)
        meta = json.loads(path.with_suffix(".meta.json")
                          .read_text(encoding="utf-8"))
        assert meta["log_columns_version"] == 6
        assert 0.0 <= meta["pre_roll_s"] <= 3.0

    def test_preroll_evicts_stale_rows(self, tmp_path):
        logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1,
                              pre_roll_s=0.05)
        logger.log_row(_row(1))
        time.sleep(0.12)                     # 保持窓(0.05s)を超えて経過
        logger.log_row(_row(2))
        path = logger.start("posture", metadata={})
        logger.stop()

        rows = _read_rows(path)
        assert [r["command_sequence"] for r in rows] == ["2"]

    def test_preroll_not_reused_after_flush(self, tmp_path):
        logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        logger.log_row(_row(1))
        path1 = logger.start("posture", stamp="20260731_000001")
        logger.stop()
        path2 = logger.start("posture", stamp="20260731_000002")
        logger.stop()

        assert [r["command_sequence"] for r in _read_rows(path1)] == ["1"]
        assert _read_rows(path2) == []       # バッファは1回のフラッシュで消費

    def test_preroll_disabled(self, tmp_path):
        logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1,
                              pre_roll_s=0.0)
        logger.log_row(_row(1))
        path = logger.start("posture", metadata={})
        logger.log_row(_row(2))
        logger.stop()

        rows = _read_rows(path)
        assert [r["command_sequence"] for r in rows] == ["2"]
        meta = json.loads(path.with_suffix(".meta.json")
                          .read_text(encoding="utf-8"))
        assert meta["pre_roll_s"] == 0.0

    def test_header_unchanged(self, tmp_path):
        """プリロールは列契約(v6・136列)を変えない。"""
        logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        logger.log_row(_row(1))
        path = logger.start("posture")
        logger.stop()
        header = path.read_text(encoding="utf-8").splitlines()[0].split(",")
        assert tuple(header) == COLUMNS


class TestSessionPreroll:
    def test_reserved_rows_flushed_on_start(self, tmp_path, session_factory):
        """ログ予約中(START 前)の行がプリロールとして先頭に入る。"""
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()

        session.set_logging(True)              # 予約のみ(ファイル未作成)
        session.posture.step(clock())          # → プリロールへ
        session.posture.step(clock())
        assert not session.logger.active
        assert list(tmp_path.glob("*.csv")) == []

        assert session.start()                 # START でフラッシュ
        log_file = session.logger.file_path
        session.posture.step(clock())          # 通常記録 1 行
        session.set_logging(False)

        rows = _read_rows(log_file)
        assert len(rows) == 3                  # プリロール2行+通常1行
        elapsed = [float(r["elapsed_time"]) for r in rows]
        assert elapsed[0] == pytest.approx(0.0, abs=1e-6)
        assert elapsed == sorted(elapsed)
        meta = json.loads(log_file.with_suffix(".meta.json")
                          .read_text(encoding="utf-8"))
        assert meta["pre_roll_s"] >= 0.0
        assert meta["log_columns_version"] == 6   # 既存メタは維持

    def test_logging_off_rows_not_captured(self, tmp_path, session_factory):
        """予約 OFF の間の行は保持しない(常時バッファリングはしない)。"""
        session, transport, clock = session_factory()
        session.logger = FlightLogger(logs_dir=tmp_path, flush_every_rows=1)
        session.connect("FAKE")
        halt_supervisor(session)
        session.posture.stop()

        session.posture.step(clock())          # 予約 OFF → 捨てられる
        session.set_logging(True)
        assert session.start()
        log_file = session.logger.file_path
        session.posture.step(clock())
        session.set_logging(False)

        rows = _read_rows(log_file)
        assert len(rows) == 1                  # 予約前の行は入らない
