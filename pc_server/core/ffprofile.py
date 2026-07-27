"""電流フィードフォワード(FF)プロファイルの抽出・適用・状態管理。

旧 yaw側プロジェクト(Yaw_Calibration_and_Estimation。2026-07-27 に削除済み)の
pc_server/server.py の FfProfileManager をシリアル版に移植:
- 抽出: サブプロセスで data_analysis/make_ff_profile.py --folder … を実行
  (数値挙動は data_analysis 側の受入テストで固定。サーバは起動するだけ)。
- 適用: mag3d binding 照合(要素毎 2e-3、不一致は警告+force 可)→
  CMD_FF_BEGIN → CMD_FF_LUT×n → CMD_FF_MOT×4 → CMD_FF_AUX →
  CMD_FF_COMMIT(crc32)。各コマンド TLM_ACK 1.0s 待ち+リトライ2回。
  commit は冪等(ACK ロスト時は CAL_GET 読み戻しで CRC を照合して救済)。
- CRC-32: IEEE(zlib 互換)、float32 LE 連結(LUT 各点 ia,dx,dy,dz →
  モーター FL,FR,RL,RR 各 ax,ay,az,c2,c1,c0 → iid)。yaw側 crc32Of と同一定義。
- 適用状態は pc_server/data/ff_state.json(yaw側スキーマ踏襲、crc は hex8)。
"""

from __future__ import annotations

import json
import math
import os
import re
import struct
import subprocess
import threading
import time
import zlib
from pathlib import Path
from typing import Any, Callable, Optional

import stampfly_protocol as proto  # sys.path シム(core/__init__.py)経由

from . import config as cfg
from .calibration import (
    CAL_VERIFY_TOLERANCE, ack_detail, ack_ok, mtime_or_zero,
    sanitize_profile_name,
)
from .serial_link import SerialLink, SerialLinkError

FF_PROFILE_SCHEMA = "stampfly_ff_profile"
FF_EXTRACT_TIMEOUT_S = 120.0
FF_MAG3D_TOLERANCE = CAL_VERIFY_TOLERANCE   # binding 照合(要素毎 2e-3)
FF_LUT_MIN_POINTS = proto.CmdFfBegin.NLUT_MIN
FF_LUT_MAX_POINTS = proto.CmdFfBegin.NLUT_MAX

# CMD_FF_MOT の idx 順(プロトコル定義: 0=FL,1=FR,2=RL,3=RR)
FF_MOTOR_ORDER = ("FL", "FR", "RL", "RR")

# 既定の ff_mode / est_mode(yaw側の既定と同じ: 方式B + EKF)
FF_MODE_DEFAULT = proto.CmdFfMode.FF_MODE_B
EST_MODE_DEFAULT = proto.CmdFfMode.EST_MODE_EKF


def ff_float32(value: Any) -> float:
    """float32 に丸める(CRC 計算と送信値を機体の float と一致させる)。"""
    return struct.unpack("<f", struct.pack("<f", float(value)))[0]


def ff_crc32(lut_rows: list[tuple[float, ...]],
             mot_rows: list[tuple[float, ...]], iid: float) -> int:
    """CRC-32(zlib)— float32 LE 連結(§4 定義、yaw側 ff_crc_hex と同一)。"""
    buf = bytearray()
    for row in lut_rows:
        for v in row:
            buf += struct.pack("<f", float(v))
    for row in mot_rows:
        for v in row:
            buf += struct.pack("<f", float(v))
    buf += struct.pack("<f", float(iid))
    return zlib.crc32(bytes(buf)) & 0xFFFFFFFF


def ff_profile_wire_values(profile: dict[str, Any]) -> dict[str, Any]:
    """プロファイル JSON から実際に送信する値を取り出し検証する。

    戻り値: {"lut": [(ia,dx,dy,dz)...], "mot": [(ax,ay,az,c2,c1,c0)×4
    FL,FR,RL,RR 順], "iid": float, "crc": int}。不正なら送信前に ValueError
    (calprofile_apply と同じ validate-first ポリシー — 機体を中途半端な
    ステージング状態に残さない)。
    """
    if profile.get("schema") != FF_PROFILE_SCHEMA:
        raise ValueError(f"schema が {FF_PROFILE_SCHEMA} ではありません")
    lut = (profile.get("method_a") or {}).get("lut") or {}
    points = lut.get("points")
    if not isinstance(points, list) \
            or not (FF_LUT_MIN_POINTS <= len(points) <= FF_LUT_MAX_POINTS):
        count = len(points) if isinstance(points, list) else 0
        raise ValueError(
            f"method_a.lut.points は {FF_LUT_MIN_POINTS}〜{FF_LUT_MAX_POINTS}"
            f"点が必要です(現在 {count})")
    lut_rows: list[tuple[float, ...]] = []
    prev_ia: Optional[float] = None
    for k, point in enumerate(points):
        ia = ff_float32(point["i_a"])
        db = point["db"]
        row = (ia, ff_float32(db[0]), ff_float32(db[1]), ff_float32(db[2]))
        if any(not math.isfinite(v) for v in row):
            raise ValueError(f"LUT 点 {k} に非有限値があります")
        if prev_ia is not None and ia <= prev_ia:
            raise ValueError(f"LUT 点は電流昇順が必要です(点 {k})")
        prev_ia = ia
        lut_rows.append(row)
    method_b = profile.get("method_b") or {}
    a_tilde = method_b.get("a_tilde") or {}
    duty_to_current = method_b.get("duty_to_current") or {}
    mot_rows: list[tuple[float, ...]] = []
    for name in FF_MOTOR_ORDER:
        ax = a_tilde[name]
        quad = duty_to_current[name]
        row = (ff_float32(ax[0]), ff_float32(ax[1]), ff_float32(ax[2]),
               ff_float32(quad["c2"]), ff_float32(quad["c1"]),
               ff_float32(quad["c0"]))
        if any(not math.isfinite(v) for v in row):
            raise ValueError(f"モーター係数 {name} に非有限値があります")
        mot_rows.append(row)
    iid = ff_float32(lut.get("i_idle_a"))
    if not math.isfinite(iid):
        raise ValueError("method_a.lut.i_idle_a が有限値ではありません")
    return {"lut": lut_rows, "mot": mot_rows, "iid": iid,
            "crc": ff_crc32(lut_rows, mot_rows, iid)}


def ff_mag3d_diffs(profile: dict[str, Any], cal: proto.TlmCalData) -> list[str]:
    """binding 照合: profile.binding.mag3d と機体の現 mag3d(TLM_CAL_DATA)を
    要素毎 FF_MAG3D_TOLERANCE で比較する(§6.1-1 踏襲)。"""
    mag3d = (profile.get("binding") or {}).get("mag3d") or {}
    offset = mag3d.get("offset")
    matrix = mag3d.get("matrix")
    if (not isinstance(offset, list) or len(offset) != 3
            or not isinstance(matrix, list) or len(matrix) != 3
            or any(not isinstance(row, list) or len(row) != 3 for row in matrix)):
        return ["binding.mag3d がプロファイルにありません"]
    keys = ["ox", "oy", "oz",
            "m00", "m01", "m02", "m10", "m11", "m12", "m20", "m21", "m22"]
    want = ([float(v) for v in offset]
            + [float(v) for row in matrix for v in row])
    got = list(cal.mag3d_offset) + list(cal.mag3d_matrix)
    diffs: list[str] = []
    for key, expected, actual in zip(keys, want, got):
        if abs(float(actual) - expected) > FF_MAG3D_TOLERANCE:
            diffs.append(f"{key}: profile={expected:.6f} drone={float(actual):.6f}")
    if not (cal.valid_flags & proto.TlmCalData.VALID_MAG3D):
        diffs.insert(0, "機体側 mag3d が未設定です")
    return diffs


class FfProfileManager:
    """FF プロファイルの抽出/適用/モード切替/アンカー/削除/状態。

    排他はキャリブプロファイルと同じスロット(hub.calprofile_begin/end)を
    共有する: スイープ/シーケンス実行中は FF 操作を拒否し、FF 操作中は
    スイープ開始を拒否する(yaw側の no-TOCTOU パターン踏襲)。
    """

    def __init__(self, link: SerialLink, hub, calibration,
                 notify: Callable[[str], None],
                 profile_dir: Path = cfg.FF_PROFILES_DIR,
                 state_path: Path = cfg.FF_STATE_PATH,
                 sweep_result_dir: Path = cfg.SWEEP_RESULTS_DIR,
                 data_analysis_dir: Path = cfg.DATA_ANALYSIS_DIR) -> None:
        self.link = link
        self.hub = hub
        self.calibration = calibration   # CalibrationManager(cal_data 取得用)
        self.notify = notify
        self.profile_dir = Path(profile_dir)
        self.state_path = Path(state_path)
        self.sweep_result_dir = Path(sweep_result_dir)
        self.data_analysis_dir = Path(data_analysis_dir)

        self._lock = threading.Lock()
        self.message = ""
        self.busy = False

    # ---- 状態ファイル ----

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
        # 原子的書き込み(tmp + os.replace): 書き込み途中クラッシュで
        # 壊れた ff_state.json が「未適用」として読まれる事故を防ぐ
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
                entry: dict[str, Any] = {"name": path.stem, "memo": "",
                                         "created_at": None,
                                         "warnings_count": 0}
                try:
                    data = json.loads(path.read_text(encoding="utf-8"))
                    entry["memo"] = str(data.get("memo", ""))
                    entry["created_at"] = data.get("created_at")
                    entry["warnings_count"] = len(
                        (data.get("quality") or {}).get("warnings") or [])
                except FileNotFoundError:
                    continue
                except (OSError, json.JSONDecodeError):
                    entry["error"] = "unreadable"
                profiles.append(entry)
        folders: list[str] = []
        loose_stems: list[str] = []
        if self.sweep_result_dir.is_dir():
            folders = sorted(p.name for p in self.sweep_result_dir.iterdir()
                             if p.is_dir())
            loose_stems = sorted(p.name[:-len("_meta.json")]
                                 for p in self.sweep_result_dir.glob(
                                     "sweep_*_meta.json"))
        state = self._load_state()
        applied = state.get("applied")
        if isinstance(applied, dict):
            applied = {**applied, "verified": bool(state.get("verified"))}
        else:
            applied = None
        # マルチ機体: 機体(MAC)別の適用状態(verified は各エントリ内に持つ)
        applied_by_mac = state.get("applied_by_mac")
        if not isinstance(applied_by_mac, dict):
            applied_by_mac = {}
        with self._lock:
            busy = self.busy
            message = self.message
        return {"profiles": profiles, "folders": folders,
                "loose_stems": loose_stems, "applied": applied,
                "applied_by_mac": applied_by_mac,
                "busy": busy, "message": message}

    # ---- 排他(キャリブプロファイルとスロット共有) ----

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

    # ---- 抽出 ----

    def _default_stems_name(self, stems: list[str]) -> str:
        """ファイル選択モードの既定名: ff_<最初の全機ランの日付>。"""
        for stem in stems:
            meta_path = self.sweep_result_dir / f"{stem}_meta.json"
            try:
                meta = json.loads(meta_path.read_text(encoding="utf-8"))
            except (OSError, json.JSONDecodeError):
                continue
            if meta.get("motors") == "FL+FR+RL+RR":
                match = re.match(r"sweep_(\d{8})_", stem)
                if match:
                    return f"ff_{match.group(1)}"
        return "ff_profile"

    def extract(self, folder: Any = None, stems: Any = None,
                name: Any = None, memo: Any = None) -> dict[str, Any]:
        script = self.data_analysis_dir / "make_ff_profile.py"
        if not script.is_file():
            return self._fail(f"抽出ツールが見つかりません: {script}")
        stems_list: list[str] = []
        folder_name = ""
        if folder:
            folder_name = str(folder).strip()
            if "/" in folder_name or "\\" in folder_name \
                    or folder_name.startswith("."):
                return self._fail(f"不正なフォルダ名です: {folder_name}")
            if not (self.sweep_result_dir / folder_name).is_dir():
                return self._fail(f"フォルダが見つかりません: {folder_name}")
            profile_name = (sanitize_profile_name(name)
                            or sanitize_profile_name(folder_name))
        else:
            if not isinstance(stems, list) or len(stems) != 8:
                return self._fail("ファイル選択モードは sweep を8本指定してください")
            stems_list = [str(s).strip() for s in stems]
            missing = [s for s in stems_list
                       if not re.fullmatch(r"sweep_\d{8}_\d{6}", s)
                       or not (self.sweep_result_dir / f"{s}_meta.json").is_file()]
            if missing:
                return self._fail(f"見つからない/不正な stem: {', '.join(missing[:4])}")
            profile_name = (sanitize_profile_name(name)
                            or self._default_stems_name(stems_list))
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
            command = [python_exe, str(script)]
            if folder_name:
                command += ["--folder", str(self.sweep_result_dir / folder_name)]
            else:
                command += ["--stems", *stems_list,
                            "--results-dir", str(self.sweep_result_dir)]
            command += ["--name", profile_name, "-o", str(out_path)]
            if memo:
                command += ["--memo", str(memo)[:200]]
            try:
                proc = subprocess.run(command, capture_output=True, text=True,
                                      timeout=FF_EXTRACT_TIMEOUT_S,
                                      cwd=str(self.data_analysis_dir))
            except subprocess.TimeoutExpired:
                return self._fail(
                    f"抽出がタイムアウトしました({FF_EXTRACT_TIMEOUT_S:.0f}s)")
            except OSError as exc:
                return self._fail(f"抽出プロセスを起動できませんでした: {exc}")
            if proc.returncode != 0 or not out_path.is_file():
                detail = (proc.stderr or proc.stdout or "").strip().splitlines()
                tail = " / ".join(detail[-3:]) if detail else f"exit={proc.returncode}"
                return self._fail(f"抽出に失敗しました: {tail}")
            try:
                data = json.loads(out_path.read_text(encoding="utf-8"))
                ff_profile_wire_values(data)   # 健全性: 出力は適用可能であること
            except (OSError, json.JSONDecodeError, ValueError, KeyError,
                    TypeError, IndexError) as exc:
                return self._fail(f"抽出結果が不正です: {exc}")
            quality = data.get("quality") or {}
            warnings = list(quality.get("warnings") or [])
            self._set_message(
                f"抽出完了: {profile_name}"
                + (f"(警告 {len(warnings)}件)" if warnings else ""))
            return {"ok": True, "name": profile_name, "quality": quality,
                    "warnings": warnings, **self.status()}
        finally:
            self._end()

    # ---- 適用 ----

    def _send_step(self, msg_type: int, payload: bytes,
                   node_id: Optional[int] = None) -> tuple[bool, str]:
        """1コマンドを ACK 待ち+リトライ付きで送る(リトライは serial_link
        側の send_with_ack が行う: 1.0s × 最大2回再送)。

        node_id 指定時はマルチ機体モードのノード宛(RLY_MUX_UP 経由)。
        """
        try:
            if node_id is None:
                ack = self.link.send_with_ack(msg_type, payload)
            else:
                ack = self.link.send_with_ack_to(node_id, msg_type, payload)
        except SerialLinkError as exc:
            return False, f"送信失敗: {exc}"
        return ack_ok(ack), ack_detail(ack)

    @staticmethod
    def _clamp_ff(value: Any, default: int = FF_MODE_DEFAULT) -> int:
        try:
            ff = int(value)
        except (TypeError, ValueError):
            return default
        return ff if ff in (proto.CmdFfMode.FF_MODE_OFF,
                            proto.CmdFfMode.FF_MODE_A,
                            proto.CmdFfMode.FF_MODE_B) else default

    @staticmethod
    def _clamp_est(value: Any, default: int = EST_MODE_DEFAULT) -> int:
        try:
            est = int(value)
        except (TypeError, ValueError):
            return default
        return est if est in (proto.CmdFfMode.EST_MODE_COMPLEMENTARY,
                              proto.CmdFfMode.EST_MODE_EKF) else default

    def _read_back_ff(self, node_id: Optional[int] = None
                      ) -> Optional[proto.TlmCalData]:
        return self.calibration.fetch_cal_data(node_id)

    def apply(self, name: Any, ff: Any = None, est: Any = None,
              force: bool = False, node_id: Optional[int] = None,
              mac: Optional[str] = None) -> dict[str, Any]:
        """FF プロファイルを適用する。

        node_id / mac 指定時はマルチ機体モードのノード宛に適用し、適用状態は
        ff_state.json の applied_by_mac[mac] に記録する(単機の "applied" は
        触らない)。機体別の較正・FF は各機体の NVS に永続化されるため、
        本メソッドの分割転送+CRC 照合の手順は単機と完全に同一。
        """
        clean = sanitize_profile_name(name)
        path = self.profile_dir / f"{clean}.json"
        if not clean or not path.is_file():
            return self._fail(f"FFプロファイルが見つかりません: {clean}")
        try:
            profile = json.loads(path.read_text(encoding="utf-8"))
            wire = ff_profile_wire_values(profile)
        except (OSError, json.JSONDecodeError) as exc:
            return self._fail(f"FFプロファイル読み込み失敗: {exc}")
        except (ValueError, KeyError, TypeError, IndexError) as exc:
            return self._fail(f"FFプロファイル形式が不正です: {clean}({exc})")
        ff_value = self._clamp_ff(ff)
        est_value = self._clamp_est(est)
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            # 1. binding 照合: プロファイルは取得時の mag3d 空間に固有
            #    (dB は補正後フレームで計測されている)
            cal = self._read_back_ff(node_id)
            if cal is None:
                return self._fail("機体から TLM_CAL_DATA を取得できませんでした"
                                  "(リンク/機体状態を確認)")
            diffs = ff_mag3d_diffs(profile, cal)
            if diffs and not force:
                return self._fail(
                    f"mag3d が取得時と一致しません(force で強制適用可能): {clean}",
                    diffs=diffs, mag3d_mismatch=True)
            # 2. 分割転送(各コマンド ACK 待ち+リトライ)
            crc = wire["crc"]
            steps: list[tuple[int, bytes]] = [
                (proto.MsgType.CMD_FF_BEGIN,
                 proto.CmdFfBegin(nlut=len(wire["lut"])).to_payload())]
            for k, (ia, dx, dy, dz) in enumerate(wire["lut"]):
                steps.append((proto.MsgType.CMD_FF_LUT,
                              proto.CmdFfLut(idx=k, i_a=ia, db_x=dx, db_y=dy,
                                             db_z=dz).to_payload()))
            for m, (ax, ay, az, c2, c1, c0) in enumerate(wire["mot"]):
                steps.append((proto.MsgType.CMD_FF_MOT,
                              proto.CmdFfMot(idx=m, a_tilde=(ax, ay, az),
                                             c2=c2, c1=c1, c0=c0).to_payload()))
            steps.append((proto.MsgType.CMD_FF_AUX,
                          proto.CmdFfAux(iid_a=wire["iid"]).to_payload()))
            steps.append((proto.MsgType.CMD_FF_COMMIT,
                          proto.CmdFfCommit(crc32=crc).to_payload()))
            for msg_type, payload in steps:
                ok, detail = self._send_step(msg_type, payload, node_id)
                if not ok:
                    if msg_type == proto.MsgType.CMD_FF_COMMIT:
                        # commit は ACK ロストでも成功している可能性がある
                        # (再送は「ステージングなし」で失敗し得る)。
                        # 読み戻して CRC を照合してから諦める。
                        data = self._read_back_ff(node_id)
                        if (data is not None
                                and (data.valid_flags
                                     & proto.TlmCalData.VALID_FFCAL)
                                and data.ff_crc32 == crc):
                            continue   # 機体側で commit 済み → 検証へ
                    return self._fail(
                        f"{proto.MsgType(msg_type).name} が失敗しました: {detail}")
            # 3. 読み戻して valid + CRC を検証
            data = self._read_back_ff(node_id)
            if data is None:
                return self._fail("CAL_GET の応答がありません", verified=False)
            got_valid = bool(data.valid_flags & proto.TlmCalData.VALID_FFCAL)
            if not got_valid or data.ff_crc32 != crc:
                got_hex = f"{data.ff_crc32:08x}" if got_valid else "無効"
                return self._fail(
                    f"CRC照合に失敗しました(送信 {crc:08x} / 機体 {got_hex})",
                    verified=False)
            ok, detail = self._send_step(
                proto.MsgType.CMD_FF_MODE,
                proto.CmdFfMode(ff_mode=ff_value,
                                est_mode=est_value).to_payload(), node_id)
            if not ok:
                return self._fail(f"CMD_FF_MODE が失敗しました: {detail}",
                                  verified=True, crc=f"{crc:08x}")
            # 4. 適用状態の永続化(crc は yaw側スキーマと同じ hex8 文字列)。
            #    機体(MAC)指定時は applied_by_mac に、単機は従来の "applied" に
            applied_entry = {"name": clean, "memo": str(profile.get("memo", "")),
                             "applied_at": time.time(), "crc": f"{crc:08x}",
                             "ff": ff_value, "est": est_value}
            if mac is not None:
                state = self._load_state()
                by_mac = state.get("applied_by_mac")
                if not isinstance(by_mac, dict):
                    by_mac = {}
                by_mac[mac] = {**applied_entry, "verified": True}
                state["applied_by_mac"] = by_mac
                self._save_state(state)
            else:
                self._save_state({
                    **{k: v for k, v in self._load_state().items()
                       if k == "applied_by_mac"},
                    "applied": applied_entry,
                    "verified": True,
                })
            forced = "(force適用)" if (diffs and force) else ""
            target = f" → {mac}" if mac else ""
            self._set_message(f"適用・CRC照合OK: {clean}{forced}"
                              f"(ff={ff_value}, est={est_value}){target}")
            return {"ok": True, "verified": True, "crc": f"{crc:08x}",
                    "name": clean, **self.status()}
        finally:
            self._end()

    # ---- モード / アンカー / 削除 ----

    def mode(self, ff: Any, est: Any, node_id: Optional[int] = None,
             mac: Optional[str] = None) -> dict[str, Any]:
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            # 排他スロットを取ってから状態を読む(read-modify-write が
            # 並行 apply() と競合しない — yaw側の no-TOCTOU 踏襲)
            state = self._load_state()
            if mac is not None:
                by_mac = state.get("applied_by_mac")
                applied = (by_mac.get(mac)
                           if isinstance(by_mac, dict)
                           and isinstance(by_mac.get(mac), dict) else None)
            else:
                applied = (state.get("applied")
                           if isinstance(state.get("applied"), dict) else None)
            # 状態ファイル由来の値も clamp 経由で無害化する(壊れた
            # ff_state.json の null/文字列で int() が例外を出さないように)
            ff_value = self._clamp_ff(
                ff, default=(self._clamp_ff(applied.get("ff"))
                             if applied else FF_MODE_DEFAULT))
            est_value = self._clamp_est(
                est, default=(self._clamp_est(applied.get("est"))
                              if applied else EST_MODE_DEFAULT))
            ok, detail = self._send_step(
                proto.MsgType.CMD_FF_MODE,
                proto.CmdFfMode(ff_mode=ff_value,
                                est_mode=est_value).to_payload(), node_id)
            if not ok:
                return self._fail(f"CMD_FF_MODE が失敗しました: {detail}")
            if applied is not None:
                applied["ff"] = ff_value
                applied["est"] = est_value
                if mac is not None:
                    state["applied_by_mac"][mac] = applied
                else:
                    state["applied"] = applied
                self._save_state(state)
            target = f" → {mac}" if mac else ""
            self._set_message(
                f"ffモードを変更しました(ff={ff_value}, est={est_value})"
                f"{target}")
            return {"ok": True, "ff": ff_value, "est": est_value,
                    **self.status()}
        finally:
            self._end()

    def anchor(self, node_id: Optional[int] = None) -> dict[str, Any]:
        busy = self._begin()
        if busy:
            return self._fail(busy)
        try:
            ok, detail = self._send_step(proto.MsgType.CMD_FF_ANCHOR, b"",
                                         node_id)
            if not ok:
                return self._fail(f"アンカー再取得に失敗しました: {detail}")
            self._set_message("アンカーを再取得しました")
            return {"ok": True, **self.status()}
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
                return self._fail(f"FFプロファイルが見つかりません: {clean}")
            path.unlink()
            self._set_message(f"削除しました: {clean}")
            return {"ok": True, **self.status()}
        finally:
            self._end()
