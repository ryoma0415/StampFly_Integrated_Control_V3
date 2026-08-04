"""pc_server エントリポイント(FastAPI)。

このファイルだけが Web フレームワークに依存する薄い層:
- static/ の配信(ビルド不要 vanilla JS UI)
- REST: /api/ports, /api/airframes, /api/config
- v2 REST(Experiment タブ): /api/sweep, /api/sequence, /api/cal3d,
  /api/accel6, /api/quickcal, /api/geomag, /api/calprofile, /api/ffprofile,
  /api/magbias, /api/flowcal, /api/flow, /api/yawref
  (GET=状態、POST {"action": ...}=操作。core 層の戻り値 dict をそのまま返す)
- WebSocket /ws: UI コマンド受付 + 20Hz 状態配信 + 即時 event/log 配信
  (v2 コマンド: set_mode "experiment" / experiment_activate / set_yaw_control /
   circle_start / circle_stop / shuttle_start / shuttle_stop /
   traj_sequence_start / traj_sequence_stop /
   motor_start / motor_set / motor_stop /
   exp_record_start / exp_record_stop、
   setpoint メッセージの yaw_deg、yaw メッセージ)

ブロッキングする SessionManager 呼び出しは asyncio.to_thread で executor に
逃がし、スレッド→async の橋渡しは queue.Queue を 20Hz でポーリングして行う
(コーディング規約: ブロッキング I/O を async ループに持ち込まない)。

起動(リポジトリ直下から):
  cd pc_server
  source .venv/bin/activate
  python -m uvicorn app:app --host 127.0.0.1 --port 8000
"""

from __future__ import annotations

import asyncio
import contextlib
import json
import logging
import queue
from pathlib import Path

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles

import core  # noqa: F401  (protocol/ と vendor/ の sys.path シム)
from core.session import SessionManager

PC_SERVER_DIR = Path(__file__).resolve().parent
STATIC_DIR = PC_SERVER_DIR / "static"

_log = logging.getLogger(__name__)

session = SessionManager()

# 1クライアントへの WebSocket 送信タイムアウト。受信ウィンドウが詰まった
# クライアント1つが全クライアント向け配信を止めるのを防ぐ(超過=切断扱い)
_WS_SEND_TIMEOUT_S: float = session.server_config["websocket"]["send_timeout_s"]

# 接続中の WebSocket クライアント(asyncio ループ内でのみ操作する)
_clients: set[WebSocket] = set()


def _list_serial_ports() -> list[dict]:
    """接続可能なシリアルポートの列挙(pyserial)。"""
    try:
        from serial.tools import list_ports  # 遅延 import
    except ImportError:
        return []
    return [{"device": p.device, "description": p.description}
            for p in list_ports.comports()]


def _encode_ws(message: dict) -> str | None:
    """WS 送信用 JSON 文字列化(全クライアント共通なので1回だけ)。

    非有限 float は session 側(_json_safe)で None 化済みのはずだが、
    取りこぼしがあると NaN トークン入りの不正 JSON になり、ブラウザ側は
    JSON.parse 失敗でフレームを黙って捨てる(UI が固まって見える)。
    allow_nan=False でサーバ側で止め、ログに残して1フレーム落とす。"""
    try:
        return json.dumps(message, allow_nan=False, separators=(",", ":"))
    except ValueError:
        _log.exception("WS メッセージに非有限 float が混入(フレーム破棄)")
        return None


async def _send_safely(websocket: WebSocket, text: str) -> bool:
    """1クライアントへの送信。失敗/タイムアウトで False(呼び出し元が除去)。"""
    try:
        await asyncio.wait_for(websocket.send_text(text),
                               timeout=_WS_SEND_TIMEOUT_S)
        return True
    except Exception:
        # タイムアウトも「死んだクライアント」として扱い、配信ループ全体を
        # 1接続の停滞に巻き込ませない
        return False


async def _broadcast(message: dict) -> None:
    text = _encode_ws(message)
    if text is None:
        return
    dead = [ws for ws in list(_clients)
            if not await _send_safely(ws, text)]
    for ws in dead:
        _clients.discard(ws)


async def _broadcaster_loop() -> None:
    """20Hz: 即時イベントの drain + 状態スナップショット配信。

    このタスクは session.events(無制限キュー)の唯一の消費者なので、
    1周期の例外で死なせてはならない(死ぬと UI 配信が全停止し、イベントが
    無限に溜まる)。例外はログして次周期へ進む。
    """
    period_s = 1.0 / session.server_config["rates"]["ws_state_hz"]
    while True:
        try:
            # 即時メッセージ(TLM_EVENT / LOG_TEXT / サーバ警告)を順に配信
            while True:
                try:
                    event = session.events.get_nowait()
                except queue.Empty:
                    break
                await _broadcast(event)
            if _clients:
                state = await asyncio.to_thread(session.get_state_snapshot)
                await _broadcast(state)
        except Exception:
            # CancelledError は BaseException なのでここを素通りし、
            # lifespan の task.cancel() による停止は妨げない
            _log.exception("broadcaster loop iteration failed")
        await asyncio.sleep(period_s)


@contextlib.asynccontextmanager
async def _lifespan(app: FastAPI):
    task = asyncio.create_task(_broadcaster_loop())
    try:
        yield
    finally:
        task.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await task
        await asyncio.to_thread(session.shutdown)


app = FastAPI(title="StampFly Integrated Control", lifespan=_lifespan)


# ----------------------------------------------------------------------
# REST
# ----------------------------------------------------------------------

@app.get("/api/ports")
async def api_ports() -> list[dict]:
    return await asyncio.to_thread(_list_serial_ports)


@app.get("/api/airframes")
async def api_airframes() -> dict:
    return {"airframes": session.airframes}


@app.put("/api/airframes")
async def api_airframes_update(body: dict) -> dict:
    """機体プロファイル一覧の更新(UI の編集画面から)。

    body: {"airframes":[...]}。検証・原子的保存・セッション反映は
    session.update_airframes が行う(core/ は fastapi 非依存のまま)。
    失敗時も HTTP 200 で {"ok":false,"error":...} を返し、UI が表示する。
    """
    ok, error = await asyncio.to_thread(
        session.update_airframes, body.get("airframes"))
    return {"ok": ok, "error": error, "airframes": session.airframes}


@app.get("/api/config")
async def api_config() -> dict:
    return {"server": session.server_config, "control": session.control_config}


@app.get("/api/mocap/bodies")
async def api_mocap_bodies() -> dict:
    """観測中の全リジッドボディ一覧(複数機タブの紐付け確認用)。

    NatNet 未接続なら接続を試みる。UI はこれをポーリングして ID と座標を
    ライブ表示し、機体プロファイルの rigid_body_id 割り当てを支援する。
    設定タブのライブプレビュー(変換後座標・正解ヨー)も同じ応答を使う。
    """
    return await asyncio.to_thread(session.mocap_bodies)


@app.get("/api/mocap/mapping")
async def api_mocap_mapping() -> dict:
    """適用中の MoCap マッピング(座標変換+姿勢)と適用可否。"""
    return await asyncio.to_thread(session.mocap_mapping)


@app.post("/api/mocap/primary")
async def api_mocap_primary(body: dict) -> dict:
    """単機(Position)対象リジッドボディ ID の変更(設定タブから)。

    body: {"rigid_body_id": N}。Motive の Streaming ID は作り直しで増える
    ため、観測中のボディを明示選択して control.json へ永続化する。
    検証・地上ガード・フィルタ再初期化は session 側。失敗も HTTP 200。
    """
    return await asyncio.to_thread(
        session.set_primary_rigid_body, body.get("rigid_body_id"))


@app.put("/api/mocap/mapping")
async def api_mocap_mapping_update(body: dict) -> dict:
    """MoCap マッピングの検証・適用・永続化(設定タブから)。

    body: {"coordinate_transform": {...}, "attitude_transform": {...}}。
    検証・地上ガード・原子的保存・アトミック差し替え・フィルタ再初期化は
    session.update_mocap_mapping が行う(core/ は fastapi 非依存のまま)。
    失敗時も HTTP 200 で {"ok":false,"message":...} を返し、UI が表示する。
    """
    return await asyncio.to_thread(session.update_mocap_mapping, body)


# ----------------------------------------------------------------------
# v2 REST(Experiment タブ)。すべてブロッキングし得るため to_thread。
# 失敗も HTTP 200 で {"ok": false, "message": ...} を返し UI が表示する。
# ----------------------------------------------------------------------

_UNKNOWN_ACTION = {"ok": False, "message": "不明な action です"}


@app.get("/api/sweep")
async def api_sweep_status() -> dict:
    return await asyncio.to_thread(session.experiment.sweep.status)


@app.post("/api/sweep")
async def api_sweep(body: dict) -> dict:
    action = body.get("action")
    if action == "start":
        return await asyncio.to_thread(
            session.sweep_start, body.get("mask"), body.get("pattern"),
            body.get("notes"))
    if action == "abort":
        return await asyncio.to_thread(session.experiment.sweep.abort)
    return _UNKNOWN_ACTION


@app.get("/api/sequence")
async def api_sequence_status() -> dict:
    return await asyncio.to_thread(session.experiment.sequence.status)


@app.post("/api/sequence")
async def api_sequence(body: dict) -> dict:
    action = body.get("action")
    if action == "start":
        return await asyncio.to_thread(
            session.sequence_start, body.get("masks"), body.get("pattern"),
            body.get("notes"), body.get("min_start_vbat"))
    if action == "abort":
        return await asyncio.to_thread(session.experiment.sequence.abort)
    if action == "resume":
        return await asyncio.to_thread(
            session.experiment.sequence.resume, bool(body.get("force")))
    return _UNKNOWN_ACTION


@app.get("/api/cal3d")
async def api_cal3d_status() -> dict:
    return await asyncio.to_thread(session.calibration.mag3d_status)


@app.post("/api/cal3d")
async def api_cal3d(body: dict) -> dict:
    action = body.get("action")
    handlers = {
        "start": session.calibration.mag3d_start,
        "stop": session.calibration.mag3d_stop,
        "fit": session.calibration.mag3d_fit,
        "apply": session.calibration.mag3d_apply,
        "clear": session.calibration.mag3d_clear,
    }
    handler = handlers.get(action)
    if handler is None:
        return _UNKNOWN_ACTION
    return await asyncio.to_thread(handler)


@app.get("/api/accel6")
async def api_accel6_status() -> dict:
    return await asyncio.to_thread(session.calibration.accel6_status)


@app.post("/api/accel6")
async def api_accel6(body: dict) -> dict:
    action = body.get("action")
    if action == "start":
        return await asyncio.to_thread(session.calibration.accel6_start)
    if action == "capture":
        return await asyncio.to_thread(session.calibration.accel6_capture,
                                       str(body.get("face", "")))
    if action == "apply":
        return await asyncio.to_thread(session.calibration.accel6_apply)
    if action == "clear":
        return await asyncio.to_thread(session.calibration.accel6_clear)
    return _UNKNOWN_ACTION


@app.post("/api/quickcal")
async def api_quickcal(body: dict) -> dict:
    """Attitude 0 / Yaw 0 / Yaw Clear(全モード対応)。

    任意キー "drone": <機体名> で複数機モードの対象機体を指定する
    (ffprofile の "drone" 方式に倣う。Multi モード中は必須、単機では無視)。
    モード/ノード解決と飛行中ガードは session.quickcal が行う。
    """
    drone = body.get("drone")
    return await asyncio.to_thread(
        session.quickcal, str(body.get("action", "")),
        None if drone is None else str(drone))


@app.get("/api/geomag")
async def api_geomag_status() -> dict:
    return await asyncio.to_thread(session.calibration.geomag_status)


@app.post("/api/geomag")
async def api_geomag(body: dict) -> dict:
    action = body.get("action")
    if action == "select":
        return await asyncio.to_thread(session.calibration.geomag_select,
                                       body.get("id"))
    if action == "apply":
        return await asyncio.to_thread(session.calibration.geomag_apply)
    return _UNKNOWN_ACTION


@app.get("/api/calprofile")
async def api_calprofile_status() -> dict:
    return await asyncio.to_thread(session.calibration.calprofile_status)


@app.post("/api/calprofile")
async def api_calprofile(body: dict) -> dict:
    action = body.get("action")
    if action == "save":
        return await asyncio.to_thread(session.calibration.calprofile_save,
                                       body.get("name"))
    if action == "apply":
        return await asyncio.to_thread(session.calibration.calprofile_apply,
                                       body.get("name"))
    if action == "delete":
        return await asyncio.to_thread(session.calibration.calprofile_delete,
                                       body.get("name"))
    return _UNKNOWN_ACTION


@app.get("/api/ffprofile")
async def api_ffprofile_status() -> dict:
    return await asyncio.to_thread(session.ffprofile.status)


@app.post("/api/ffprofile")
async def api_ffprofile(body: dict) -> dict:
    """FF プロファイル操作。apply / mode / anchor は "drone" を指定すると
    複数機モードの機体別操作(ノード宛+MAC 別適用状態)になる。"""
    action = body.get("action")
    drone = body.get("drone")
    if action == "extract":
        return await asyncio.to_thread(
            session.ffprofile.extract, body.get("folder"), body.get("stems"),
            body.get("name"), body.get("memo"))
    if action == "apply":
        if drone is not None:
            return await asyncio.to_thread(
                session.multi_ff_apply, str(drone), body.get("name"),
                body.get("ff"), body.get("est"), bool(body.get("force")))
        return await asyncio.to_thread(
            session.ffprofile.apply, body.get("name"), body.get("ff"),
            body.get("est"), bool(body.get("force")))
    if action == "mode":
        if drone is not None:
            return await asyncio.to_thread(
                session.multi_ff_mode, str(drone),
                body.get("ff"), body.get("est"))
        return await asyncio.to_thread(session.ffprofile.mode,
                                       body.get("ff"), body.get("est"))
    if action == "anchor":
        if drone is not None:
            return await asyncio.to_thread(session.multi_ff_anchor,
                                           str(drone))
        return await asyncio.to_thread(session.ffprofile.anchor)
    if action == "delete":
        return await asyncio.to_thread(session.ffprofile.delete,
                                       body.get("name"))
    return _UNKNOWN_ACTION


@app.get("/api/magbias")
async def api_magbias_status() -> dict:
    return await asyncio.to_thread(session.magbias.status)


@app.post("/api/magbias")
async def api_magbias(body: dict) -> dict:
    """magbias プロファイル操作(MAG_AUTOTUNE_DESIGN.md §3.4。単機のみ)。"""
    action = body.get("action")
    if action == "extract":
        return await asyncio.to_thread(
            session.magbias.extract, body.get("log"), body.get("name"))
    if action == "apply":
        return await asyncio.to_thread(
            session.magbias.apply, body.get("name"), bool(body.get("force")))
    if action == "clear":
        return await asyncio.to_thread(session.magbias.clear)
    if action == "delete":
        return await asyncio.to_thread(session.magbias.delete,
                                       body.get("name"))
    if action == "status":
        return await asyncio.to_thread(session.magbias.status)
    return _UNKNOWN_ACTION


@app.get("/api/flowcal")
async def api_flowcal_status() -> dict:
    return await asyncio.to_thread(session.flowcal.status)


@app.post("/api/flowcal")
async def api_flowcal(body: dict) -> dict:
    """フロー較正(純回転 2×2 フィット。FLIGHT_ANALYSIS_20260731.md §5)。

    start_record はモーター停止・非飛行時のみ受理(core/flowcal.py の
    ゲート)。apply は合格フィットのみ(不合格は force で強制)。
    合格フィットは data/flowcal_profiles/ へ自動保存され、apply {name} で
    保存済みプロファイルから選択適用・delete {name} で削除できる
    (magbias/ff プロファイルと同じ管理様式)。
    clear は PC 側の記録/フィット破棄(機体 NVS のクリアは /api/flow)。
    """
    action = body.get("action")
    if action == "start_record":
        return await asyncio.to_thread(session.flowcal.start_record)
    if action == "stop_and_fit":
        return await asyncio.to_thread(session.flowcal.stop_and_fit)
    if action == "apply":
        if body.get("name"):
            return await asyncio.to_thread(session.flowcal.apply_profile,
                                           body.get("name"))
        return await asyncio.to_thread(session.flowcal.apply,
                                       bool(body.get("force")))
    if action == "delete":
        return await asyncio.to_thread(session.flowcal.delete_profile,
                                       body.get("name"))
    if action == "clear":
        return await asyncio.to_thread(session.flowcal.clear)
    if action in ("status", "profiles"):
        return await asyncio.to_thread(session.flowcal.status)
    return _UNKNOWN_ACTION


@app.post("/api/flow")
async def api_flow(body: dict) -> dict:
    """flowcal 適用/クリアと flow probe のパススルー(契約 §3.5)。"""
    action = body.get("action")
    if action == "cal_set":
        return await asyncio.to_thread(
            session.magbias.flowcal_set, body.get("kx"), body.get("ky"))
    if action == "cal_clear":
        return await asyncio.to_thread(session.magbias.flowcal_clear)
    if action == "probe":
        return await asyncio.to_thread(
            session.magbias.flow_probe, body.get("n_cycles", 0))
    return _UNKNOWN_ACTION


@app.get("/api/yawref")
async def api_yawref_status() -> dict:
    """ヨー基準ソース(off|mocap|motion)と移動ベースヨーの現況。"""
    return await asyncio.to_thread(session.yaw_ref_status)


@app.post("/api/yawref")
async def api_yawref(body: dict) -> dict:
    """ヨー基準ソースの実行時切替(契約 §3.1)。body: {"source": ...}。"""
    return await asyncio.to_thread(
        session.set_yaw_ref_source, str(body.get("source", "")))


# ----------------------------------------------------------------------
# WebSocket
# ----------------------------------------------------------------------

async def _handle_command(message: dict) -> None:
    """{"type":"command", ...} の処理(ブロッキングし得るので to_thread)。"""
    action = message.get("action")
    if action == "connect":
        await asyncio.to_thread(session.connect, str(message.get("port", "")))
    elif action == "disconnect":
        await asyncio.to_thread(session.disconnect)
    elif action == "select_airframe":
        await asyncio.to_thread(session.select_airframe, str(message.get("name", "")))
    elif action == "set_mode":
        await asyncio.to_thread(session.set_mode, str(message.get("mode", "")))
    elif action == "start":
        # force: プリフライト・インターロック(P1-2)の明示的解除
        # (UI「強制離陸」ボタン。省略時は通常ゲート)
        await asyncio.to_thread(session.start, bool(message.get("force")))
    elif action == "stop":
        await asyncio.to_thread(session.stop)
    elif action == "reset":
        await asyncio.to_thread(session.reset)
    elif action == "set_logging":
        await asyncio.to_thread(session.set_logging, bool(message.get("enabled")))
    elif action == "experiment_activate":
        await asyncio.to_thread(session.activate_experiment)
    elif action == "set_yaw_control":
        await asyncio.to_thread(session.set_yaw_control,
                                bool(message.get("enabled")))
    elif action == "circle_start":
        await asyncio.to_thread(
            session.circle_start,
            float(message.get("center_x", 0.0)),
            float(message.get("center_y", 0.0)),
            float(message.get("radius_m", 0.0)),
            float(message.get("period_s", 0.0)),
            bool(message.get("clockwise", True)),
            float(message.get("alt_m", 0.0)),
            bool(message.get("face_tangent", False)))
    elif action == "circle_stop":
        await asyncio.to_thread(session.circle_stop)
    elif action == "shuttle_start":
        await asyncio.to_thread(
            session.shuttle_start,
            float(message.get("center_x", 0.0)),
            float(message.get("center_y", 0.0)),
            float(message.get("axis_deg", 0.0)),
            float(message.get("amplitude_m", 0.0)),
            float(message.get("period_s", 0.0)),
            int(message.get("cycles", 0)),
            float(message.get("alt_m", 0.0)))
    elif action == "shuttle_stop":
        await asyncio.to_thread(session.shuttle_stop)
    elif action == "traj_sequence_start":
        await asyncio.to_thread(
            session.traj_sequence_start,
            str(message.get("name", "")),
            float(message.get("alt_m", 0.0)),
            int(message.get("start_index", 0)))
    elif action == "traj_sequence_stop":
        await asyncio.to_thread(session.traj_sequence_stop)
    elif action == "motor_start":
        await asyncio.to_thread(session.motor_start,
                                float(message.get("duty", 0.0)),
                                int(message.get("mask", 0x0F)))
    elif action == "motor_set":
        await asyncio.to_thread(session.motor_apply,
                                float(message.get("duty", 0.0)))
    elif action == "motor_stop":
        await asyncio.to_thread(session.motor_stop)
    elif action == "exp_record_start":
        await asyncio.to_thread(session.exp_record_start)
    elif action == "exp_record_stop":
        await asyncio.to_thread(session.exp_record_stop)
    elif action == "multi_select":
        names = message.get("names")
        await asyncio.to_thread(
            session.multi_select,
            [str(n) for n in names] if isinstance(names, list) else [])
    elif action == "multi_start":
        await asyncio.to_thread(session.multi_start)
    elif action == "multi_yaw":
        yaw_deg = message.get("yaw_deg")
        await asyncio.to_thread(
            session.multi_yaw, str(message.get("name", "")),
            message.get("enabled"),
            None if yaw_deg is None else float(yaw_deg))
    else:
        session.warn(f"不明なコマンド: {action}")


# SPACE 緊急停止系のコマンド。受信ループが先行コマンド(select_airframe の
# ACK 待ち最大約4秒など)の完了を待たされないよう、順序キューを迂回して
# 即時に優先経路(session.emergency_stop / motor_stop: _command_lock 非経由)
# を実行する。正規の停止処理(STOP 再送監視など)は順序キュー側でも実行される。
_EMERGENCY_ACTIONS = frozenset({"stop", "motor_stop"})


def _is_emergency(message: dict) -> bool:
    return (message.get("type") == "command"
            and message.get("action") in _EMERGENCY_ACTIONS)


async def _handle_emergency(message: dict) -> None:
    if message.get("action") == "stop":
        await asyncio.to_thread(session.emergency_stop)
    else:   # motor_stop(_command_lock 非経由の即時停止)
        await asyncio.to_thread(session.motor_stop)


async def _handle_message(message: dict) -> None:
    msg_type = message.get("type")
    try:
        if msg_type == "command":
            await _handle_command(message)
        elif msg_type == "setpoint":
            # Posture モード: deg/m → session 層で rad へ変換(非ブロッキング)
            yaw_deg = message.get("yaw_deg")
            session.set_setpoint_deg(float(message["roll_deg"]),
                                     float(message["pitch_deg"]),
                                     float(message["alt_m"]),
                                     yaw_deg=(None if yaw_deg is None
                                              else float(yaw_deg)))
        elif msg_type == "yaw":
            # 共通ヨー角スライダ(±180°、両モードへ反映。非ブロッキング)
            session.set_yaw_setpoint_deg(float(message["yaw_deg"]))
        elif msg_type == "target":
            # Position モード: 制御座標系 m(非ブロッキング)
            session.set_target(float(message["x"]),
                               float(message["y"]),
                               float(message["z"]))
        elif msg_type == "multi_target":
            # 複数機モード: 機体別の目標位置。multi_target は _command_lock を
            # 取る(multi_start との直列化)ため必ず executor へ逃がす。
            # イベントループ上で同期的に呼ぶと、ロック保持中(connect/FF転送
            # など)に 20Hz 配信・全 WS 処理・SIGINT 処理まで停止する。
            await asyncio.to_thread(session.multi_target,
                                    str(message["name"]),
                                    float(message["x"]),
                                    float(message["y"]),
                                    float(message["z"]))
        else:
            session.warn(f"不明なメッセージ型: {msg_type}")
    except (KeyError, TypeError, ValueError) as exc:
        session.warn(f"不正なメッセージ ({msg_type}): {exc}")


async def _client_message_worker(pending: "asyncio.Queue[dict]") -> None:
    """1クライアントぶんの通常メッセージを受信順に処理するワーカー。

    受信ループ本体から処理を切り離すことで、低速コマンド(connect /
    select_airframe の ACK 待ちなど)の実行中も受信ループがブロックせず、
    後着の SPACE 緊急停止を即時に拾えるようにする(通常コマンドの
    クライアント内順序はこのキューが保存する)。
    """
    while True:
        message = await pending.get()
        try:
            await _handle_message(message)
        except Exception:
            # ワーカーを死なせると以降のコマンドが全て無視されるため継続
            _log.exception("client message handling failed")


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket) -> None:
    await websocket.accept()
    _clients.add(websocket)
    # 接続直後に現在状態を1回送る(UI 初期表示用)
    text = _encode_ws(await asyncio.to_thread(session.get_state_snapshot))
    if text is not None:
        await _send_safely(websocket, text)
    pending: asyncio.Queue = asyncio.Queue()
    worker = asyncio.create_task(_client_message_worker(pending))
    try:
        while True:
            message = await websocket.receive_json()
            if not isinstance(message, dict):
                continue
            if _is_emergency(message):
                # 緊急停止: 先行コマンドの完了を待たず優先経路を即時実行。
                # 正規の停止処理も順序キューへ積む(冪等)。
                await _handle_emergency(message)
            pending.put_nowait(message)
    except WebSocketDisconnect:
        pass
    finally:
        worker.cancel()
        with contextlib.suppress(asyncio.CancelledError):
            await worker
        _clients.discard(websocket)


# ----------------------------------------------------------------------
# 静的 UI(API ルートより後にマウントする)
# ----------------------------------------------------------------------

STATIC_DIR.mkdir(parents=True, exist_ok=True)
app.mount("/", StaticFiles(directory=str(STATIC_DIR), html=True), name="static")
