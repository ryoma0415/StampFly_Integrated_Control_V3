"""magbias(学習ハードアイアン残差)プロファイルの抽出・適用・状態管理と
flowcal / flow probe のパススルー(MAG_AUTOTUNE_DESIGN.md §1.4 / §3.4-5)。

FfProfileManager と同型:
- 抽出: サブプロセスで
  `data_analysis/magbias_learn.py <ログCSV> -o <出力JSON> --name <名前>` を
  実行(CLI 契約: 終了コード 0 = 成功。数値挙動は data_analysis 側の
  責務 — サーバは起動と結果検証のみ)。
  出力は pc_server/data/magbias_profiles/<name>.json
  (schema "stampfly_magbias_profile" v1、契約書 §3 の JSON)。
- 適用: binding 照合(profile.binding.ffcal_crc と機体の現 ff_crc32。
  不一致は警告+force 可)→ CMD_MAGBIAS_SET(mode=1, Δb)→ CAL_GET
  読み戻しで valid bit6+値照合 → magbias_state.json 記録。
  機体側は WAIT/COMPLETE/MOTOR_TEST のみ受理(飛行中 NVS 書込み禁止)。
- clear: CMD_MAGBIAS_SET(mode=0)→ 読み戻しで bit6 消灯を確認。
- flowcal set/clear: CMD_FLOWCAL_SET → 読み戻しで bit7+値照合。
- flow probe: CMD_FLOW_PROBE(結果は機体からの LOG_TEXT 複数行で届く —
  UI コンソールに流れるためここでは ACK 確認のみ)。

排他はキャリブ/FF プロファイルと同じスロット(hub.calprofile_begin/end)を
共有する(スイープ実行中の NVS 書込み操作を構造的に防ぐ)。
"""

from __future__ import annotations

import json
import math
import os
import subprocess
import threading
import time
from pathlib import Path
from typing import Any, Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from . import config as cfg
from .calibration import (
    CAL_VERIFY_TOLERANCE, ack_detail, ack_ok, mtime_or_zero,
    sanitize_profile_name,
)
from .serial_link import SerialLink, SerialLinkError

MAGBIAS_PROFILE_SCHEMA = "stampfly_magbias_profile"
MAGBIAS_EXTRACT_TIMEOUT_S = 120.0


def magbias_delta_b(profile: dict[str, Any]) -> tuple[float, float, float]:
    """プロファイル JSON から適用する Δb [µT] を取り出し検証する。

    delta_b_ut はヨー励振不足の抽出では null になり得る(契約書 §3 の
    プロファイル JSON)— その場合は適用不能として ValueError。
    """
    if profile.get("schema") != MAGBIAS_PROFILE_SCHEMA:
        raise ValueError(f"schema が {MAGBIAS_PROFILE_SCHEMA} ではありません")
    delta = profile.get("delta_b_ut")
    if delta is None:
        raise ValueError("delta_b_ut が null です(ヨー励振不足の抽出 — "
                         "hover_residual のみのプロファイルは適用できません)")
    if not isinstance(delta, list) or len(delta) != 3:
        raise ValueError("delta_b_ut は f32×3 が必要です")
    values = tuple(float(v) for v in delta)
    if any(not math.isfinite(v) for v in values):
        raise ValueError("delta_b_ut に非有限値があります")
    return values


def magbias_binding_diffs(profile: dict[str, Any],
                          cal: proto.TlmCalData) -> list[str]:
    """binding 照合: profile.binding.ffcal_crc と機体の現 ff_crc32。

    magbias の Δb は「FF 補正後の残差」なので FF プロファイルに束縛される
    (FF を差し替えたら残差の意味が変わる)。mag3d_hash はハッシュ定義が
    抽出側(data_analysis)にありワイヤから再計算できないため照合しない
    (mag3d 変更時は機体側が magbias を連動クリアする — 契約 §2.5)。
    """
    binding = profile.get("binding") or {}
    crc_hex = binding.get("ffcal_crc")
    if not crc_hex:
        return []   # binding なしのプロファイルは照合対象外
    diffs: list[str] = []
    if not (cal.valid_flags & proto.TlmCalData.VALID_FFCAL):
        diffs.append("機体側 ffcal が未設定です")
        return diffs
    got_hex = f"{cal.ff_crc32:08x}"
    if str(crc_hex).lower() != got_hex:
        diffs.append(f"ffcal_crc: profile={crc_hex} drone={got_hex}")
    return diffs


class MagbiasManager:
    """magbias プロファイルの抽出/適用/クリア/状態+flowcal/probe。"""

    def __init__(self, link: SerialLink, hub, calibration,
                 notify: Callable[[str], None],
                 profile_dir: Path = cfg.MAGBIAS_PROFILES_DIR,
                 state_path: Path = cfg.MAGBIAS_STATE_PATH,
                 logs_dir: Path = cfg.LOGS_DIR,
                 data_analysis_dir: Path = cfg.DATA_ANALYSIS_DIR) -> None:
        self.link = link
        self.hub = hub
        self.calibration = calibration   # CalibrationManager(cal_data 取得用)
        self.notify = notify
        self.profile_dir = Path(profile_dir)
        self.state_path = Path(state_path)
        self.logs_dir = Path(logs_dir)
        self.data_analysis_dir = Path(data_analysis_dir)

        self._lock = threading.Lock()
        self.message = ""
        self.busy = False

    # ---- 状態ファイル(ffprofile と同型) ----

    def _set_message(self, message: str) -> None:
        with self._lock:
            self.message = message

    def _load_state(self) -> dict[str, Any]:
        if not self.state_path.is_file():
            return {}
        try:
            data = json.loads(self.state_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            self.notify(f"[警告] {self.state_path.name} を読めないため"
                        f"未適用として扱います: {exc}")
            return {}
        return data if isinstance(data, dict) else {}

    def _save_state(self, state: dict[str, Any]) -> None:
        self.state_path.parent.mkdir(parents=True, exist_ok=True)
        tmp = self.state_path.with_suffix(".json.tmp")
        tmp.write_text(json.dumps(state, ensure_ascii=False, indent=2),
                       encoding="utf-8")
        os.replace(tmp, self.state_path)

    def status(self) -> dict[str, Any]:
        profiles: list[dict[str, Any]] = []
        if self.profile_dir.is_dir():
            for path in sorted(self.profile_dir.glob("*.json"),
                               key=mtime_or_zero, reverse=True):
                entry: dict[str, Any] = {"name": path.stem,
                                         "created_at": None,
                                         "delta_b_ut": None,
                                         "warnings_count": 0}
                try:
                    data = json.loads(path.read_text(encoding="utf-8"))
                    entry["created_at"] = data.get("created_at")
                    entry["delta_b_ut"] = data.get("delta_b_ut")
                    entry["warnings_count"] = len(
                        (data.get("quality") or {}).get("warnings") or [])
                except FileNotFoundError:
                    continue
                except (OSError, json.JSONDecodeError):
                    entry["error"] = "unreadable"
                profiles.append(entry)
        state = self._load_state()
        applied = state.get("applied")
        if not isinstance(applied, dict):
            applied = None
        # 抽出元候補: 飛行ログ CSV(新しい順)
        logs: list[str] = []
        if self.logs_dir.is_dir():
            logs = [p.name for p in sorted(self.logs_dir.glob("*.csv"),
                                           key=mtime_or_zero, reverse=True)]
        with self._lock:
            busy = self.busy
            message = self.message
        return {"profiles": profiles, "applied": applied, "logs": logs,
                "busy": busy, "message": message}

    # ---- 排他(キャリブ/FF プロファイルとスロット共有) ----

    def _begin(self) -> Optional[str]:
        busy = self.hub.calprofile_begin()
        if busy is None:
            with self._lock:
                self.busy = True
        return busy

    def _end(self) -> None:
        with self._lock:
            self.busy = False
        self.hub.calprofile_end()

    def _fail(self, message: str, **extra: Any) -> dict[str, Any]:
        self._set_message(message)
        return {"ok": False, "message": message, **extra, **self.status()}

    def _send_ack(self, msg_type: int, payload: bytes) -> tuple[bool, str]:
        """1コマンドを ACK 待ち+リトライ付きで送る(serial_link 側で再送)。"""
        try:
            ack = self.link.send_with_ack(msg_type, payload)
        except SerialLinkError as exc:
            return False, f"送信失敗: {exc}"
        return ack_ok(ack), ack_detail(ack)

    def _read_back(self) -> Optional[proto.TlmCalData]:
        return self.calibration.fetch_cal_data()

    # ---- 抽出 ----

    def extract(self, log: Any, name: Any = None) -> dict[str, Any]:
        """magbias_learn.py をサブプロセス実行してプロファイルを作る。

        log: logs/flight_logs/ 直下の CSV ファイル名(v6 ログ)。
        """
        script = self.data_analysis_dir / "magbias_learn.py"
        if not script.is_file():
            return self._fail(f"抽出ツールが見つかりません: {script}")
        log_name = str(log or "").strip()
        if not log_name or "/" in log_name or "\\" in log_name \
                or log_name.startswith("."):
            return self._fail(f"不正なログ名です: {log_name!r}")
        log_path = self.logs_dir / log_name
        if not log_path.is_file():
            return self._fail(f"ログが見つかりません: {log_name}")
        profile_name = sanitize_profile_name(name)
        if not profile_name:
            # 既定名: magbias_<ログ stem>(拡張子除去)
            profile_name = sanitize_profile_name(f"magbias_{log_path.stem}")
        if not profile_name:
            return self._fail("プロファイル名が決められませんでした")
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            venv_python = self.data_analysis_dir / ".venv" / "bin" / "python"
            python_exe = str(venv_python) if venv_python.exists() else "python3"
            self.profile_dir.mkdir(parents=True, exist_ok=True)
            out_path = self.profile_dir / f"{profile_name}.json"
            command = [python_exe, str(script), str(log_path),
                       "-o", str(out_path), "--name", profile_name]
            try:
                proc = subprocess.run(command, capture_output=True, text=True,
                                      timeout=MAGBIAS_EXTRACT_TIMEOUT_S,
                                      cwd=str(self.data_analysis_dir))
            except subprocess.TimeoutExpired:
                return self._fail(
                    f"抽出がタイムアウトしました({MAGBIAS_EXTRACT_TIMEOUT_S:.0f}s)")
            except OSError as exc:
                return self._fail(f"抽出プロセスを起動できませんでした: {exc}")
            if proc.returncode != 0 or not out_path.is_file():
                detail = (proc.stderr or proc.stdout or "").strip().splitlines()
                tail = " / ".join(detail[-3:]) if detail else f"exit={proc.returncode}"
                return self._fail(f"抽出に失敗しました: {tail}")
            try:
                data = json.loads(out_path.read_text(encoding="utf-8"))
                if data.get("schema") != MAGBIAS_PROFILE_SCHEMA:
                    raise ValueError(
                        f"schema が {MAGBIAS_PROFILE_SCHEMA} ではありません")
            except (OSError, json.JSONDecodeError, ValueError) as exc:
                return self._fail(f"抽出結果が不正です: {exc}")
            quality = data.get("quality") or {}
            warnings = list(quality.get("warnings") or [])
            delta = data.get("delta_b_ut")
            note = "" if delta is not None \
                else "(ヨー励振不足: delta_b なし=適用不可、hover_residual のみ)"
            self._set_message(
                f"抽出完了: {profile_name}{note}"
                + (f"(警告 {len(warnings)}件)" if warnings else ""))
            return {"ok": True, "name": profile_name, "quality": quality,
                    "warnings": warnings, "delta_b_ut": delta, **self.status()}
        finally:
            self._end()

    # ---- 適用 / クリア ----

    def apply(self, name: Any, force: bool = False) -> dict[str, Any]:
        """magbias プロファイルを機体へ適用する(NVS 書込み+読み戻し照合)。"""
        clean = sanitize_profile_name(name)
        path = self.profile_dir / f"{clean}.json"
        if not clean or not path.is_file():
            return self._fail(f"magbias プロファイルが見つかりません: {clean}")
        try:
            profile = json.loads(path.read_text(encoding="utf-8"))
            delta = magbias_delta_b(profile)
        except (OSError, json.JSONDecodeError) as exc:
            return self._fail(f"magbias プロファイル読み込み失敗: {exc}")
        except (ValueError, TypeError) as exc:
            return self._fail(f"magbias プロファイル形式が不正です: {clean}({exc})")
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            # 1. binding 照合(Δb は適用時の FF プロファイルに束縛される)
            cal = self._read_back()
            if cal is None:
                return self._fail("機体から TLM_CAL_DATA を取得できませんでした"
                                  "(リンク/機体状態を確認)")
            diffs = magbias_binding_diffs(profile, cal)
            if diffs and not force:
                return self._fail(
                    f"ffcal が取得時と一致しません(force で強制適用可能): {clean}",
                    diffs=diffs, binding_mismatch=True)
            # 2. CMD_MAGBIAS_SET(mode=1)。機体側は即適用+アンカー無効化+
            #    補正系再シード(ff_mode は降格しない — 契約 §1.4)
            ok, detail = self._send_ack(
                proto.MsgType.CMD_MAGBIAS_SET,
                proto.CmdMagbiasSet(mode=proto.CmdMagbiasSet.MODE_SET,
                                    dx=delta[0], dy=delta[1],
                                    dz=delta[2]).to_payload())
            if not ok:
                return self._fail(f"CMD_MAGBIAS_SET が失敗しました: {detail}")
            # 3. 読み戻しで valid bit6+値を照合
            data = self._read_back()
            if data is None:
                return self._fail("CAL_GET の応答がありません", verified=False)
            if not (data.valid_flags & proto.TlmCalData.VALID_MAGBIAS):
                return self._fail("読み戻しで magbias が有効になっていません",
                                  verified=False)
            mismatch = [f"{axis}: sent={want:.3f} drone={float(got):.3f}"
                        for axis, want, got in zip("xyz", delta, data.magbias)
                        if abs(float(got) - want) > CAL_VERIFY_TOLERANCE]
            if mismatch:
                return self._fail(
                    f"読み戻し値が一致しません: {', '.join(mismatch)}",
                    verified=False)
            # 4. 適用状態の永続化(magbias_state.json — 契約 §3.4)
            self._save_state({
                "applied": {"name": clean,
                            "delta_b_ut": list(delta),
                            "applied_at": time.time(),
                            "verified": True},
            })
            forced = "(force適用)" if (diffs and force) else ""
            self._set_message(
                f"適用・読み戻し照合OK: {clean}{forced} "
                f"Δb=({delta[0]:+.2f}, {delta[1]:+.2f}, {delta[2]:+.2f})µT")
            return {"ok": True, "verified": True, "name": clean,
                    **self.status()}
        finally:
            self._end()

    def clear(self) -> dict[str, Any]:
        """機体の magbias をクリアする(NVS 消去+読み戻し確認)。"""
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            ok, detail = self._send_ack(
                proto.MsgType.CMD_MAGBIAS_SET,
                proto.CmdMagbiasSet(
                    mode=proto.CmdMagbiasSet.MODE_CLEAR).to_payload())
            if not ok:
                return self._fail(f"CMD_MAGBIAS_SET(clear) が失敗しました: {detail}")
            data = self._read_back()
            if data is None:
                return self._fail("CAL_GET の応答がありません", verified=False)
            if data.valid_flags & proto.TlmCalData.VALID_MAGBIAS:
                return self._fail("読み戻しで magbias が消えていません",
                                  verified=False)
            self._save_state({"applied": None})
            self._set_message("magbias をクリアしました")
            return {"ok": True, "verified": True, **self.status()}
        finally:
            self._end()

    def delete(self, name: Any) -> dict[str, Any]:
        clean = sanitize_profile_name(name)
        path = self.profile_dir / f"{clean}.json"
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            if not clean or not path.is_file():
                return self._fail(f"magbias プロファイルが見つかりません: {clean}")
            path.unlink()
            self._set_message(f"削除しました: {clean}")
            return {"ok": True, **self.status()}
        finally:
            self._end()

    # ---- flowcal / flow probe パススルー(契約 §3.5) ----

    def flowcal_set(self, m00: Any, m11: Any, m01: Any = 0.0,
                    m10: Any = 0.0) -> dict[str, Any]:
        """フロー較正行列 K [counts/rad] を機体 NVS へ保存する(読み戻し照合)。

        【2026-07-31 改訂】kx/ky 2 定数 → 2×2 行列化(契約 §1.4)。K は
        「機体系レート [rad/s] → センサーカウントレート [counts/s]」の写像で、
        純スケールなら K=diag(kx,ky)、取付回転 φ0 込みなら K=diag(kx,ky)·R(φ0)。
        引数順は対角優先 (m00, m11, m01, m10) — 旧 API の 2 引数呼び出し
        flowcal_set(kx, ky) は diag(kx,ky) の適用として後方互換で動く。
        """
        try:
            m00_v = float(m00)
            m11_v = float(m11)
            m01_v = float(m01)
            m10_v = float(m10)
        except (TypeError, ValueError):
            return self._fail(
                f"K 行列が数値ではありません: {m00!r}, {m01!r}, {m10!r}, {m11!r}")
        if not all(math.isfinite(v) for v in (m00_v, m01_v, m10_v, m11_v)) \
                or m00_v <= 0.0 or m11_v <= 0.0:
            return self._fail(
                f"対角 m00/m11 は正の有限値が必要です: {m00_v}, {m11_v}")
        det = m00_v * m11_v - m01_v * m10_v
        if det <= 0.0:
            return self._fail(f"det(K) が正ではありません(K⁻¹ 不安定): {det}")
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            ok, detail = self._send_ack(
                proto.MsgType.CMD_FLOWCAL_SET,
                proto.CmdFlowcalSet(mode=proto.CmdFlowcalSet.MODE_SET,
                                    m00=m00_v, m01=m01_v,
                                    m10=m10_v, m11=m11_v).to_payload())
            if not ok:
                return self._fail(f"CMD_FLOWCAL_SET が失敗しました: {detail}")
            data = self._read_back()
            if data is None:
                return self._fail("CAL_GET の応答がありません", verified=False)
            if not (data.valid_flags & proto.TlmCalData.VALID_FLOWCAL):
                return self._fail("読み戻しで flowcal が有効になっていません",
                                  verified=False)
            got_matrix = (data.flowcal_m00, data.flowcal_m01,
                          data.flowcal_m10, data.flowcal_m11)
            mismatch = [f"{name}: sent={want:.2f} drone={float(got):.2f}"
                        for name, want, got in zip(
                            ("m00", "m01", "m10", "m11"),
                            (m00_v, m01_v, m10_v, m11_v), got_matrix)
                        if abs(float(got) - want) > CAL_VERIFY_TOLERANCE]
            if mismatch:
                return self._fail(
                    f"読み戻し値が一致しません: {', '.join(mismatch)}",
                    verified=False)
            self._set_message(
                f"flowcal を適用しました(K=[{m00_v:.1f} {m01_v:.1f}; "
                f"{m10_v:.1f} {m11_v:.1f}])")
            return {"ok": True, "verified": True, **self.status()}
        finally:
            self._end()

    def flowcal_clear(self) -> dict[str, Any]:
        """フロー較正行列をクリアする(機体は既定 diag(450,450) に戻る)。"""
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            ok, detail = self._send_ack(
                proto.MsgType.CMD_FLOWCAL_SET,
                proto.CmdFlowcalSet(
                    mode=proto.CmdFlowcalSet.MODE_CLEAR).to_payload())
            if not ok:
                return self._fail(f"CMD_FLOWCAL_SET(clear) が失敗しました: {detail}")
            self._set_message("flowcal をクリアしました(既定値に戻ります)")
            return {"ok": True, **self.status()}
        finally:
            self._end()

    def flow_probe(self, n_cycles: Any = 0) -> dict[str, Any]:
        """PMW3901 burst プローブを実行する(モーター停止時のみ機体が受理)。

        結果の要約は機体からの LOG_TEXT 複数行として UI コンソールへ届く。
        """
        try:
            cycles = int(n_cycles or 0)
        except (TypeError, ValueError):
            return self._fail(f"n_cycles が整数ではありません: {n_cycles!r}")
        if not (0 <= cycles <= 255):
            return self._fail(f"n_cycles は 0〜255 が必要です: {cycles}")
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            ok, detail = self._send_ack(
                proto.MsgType.CMD_FLOW_PROBE,
                proto.CmdFlowProbe(n_cycles=cycles).to_payload())
            if not ok:
                return self._fail(f"CMD_FLOW_PROBE が失敗しました: {detail}")
            self._set_message("flow probe を開始しました(結果はログに届きます)")
            return {"ok": True, **self.status()}
        finally:
            self._end()
