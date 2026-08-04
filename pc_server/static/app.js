"use strict";
/*
 * StampFly 統合管制 UI(v2: Posture / Position / Multi / Experiment の4タブ)
 *
 * 契約: docs/ARCHITECTURE.md「pc_server API(UI⇔サーバ契約)」に厳密に従う。
 *  - UI⇔WebSocket の単位は deg / m(rad変換はサーバ側 session 層の責務)
 *  - UI→サーバ:  {"type":"command", ...} / {"type":"setpoint", ...} /
 *                {"type":"target", ...} / {"type":"yaw", ...}
 *  - サーバ→UI:  {"type":"state","data":{drone, mocap, session}} (20Hz)
 *                {"type":"event", ...}(TLM_EVENT) / {"type":"log","origin","line"}
 *  - v2 REST(Experiment): /api/{sweep,sequence,cal3d,accel6,quickcal,geomag,
 *    calprofile,ffprofile}(GET=状態、POST {"action": ...}=操作)
 */

/* ===================== UI定数(マジックナンバー集約) ===================== */
/* ARCHITECTURE.md「安全クランプ」の UI層: roll/pitch ±5°(既定)、高度 0.1–1.0m */
const UI = {
  WS_RECONNECT_MS: 1000,        // WebSocket再接続バックオフ
  SEND_THROTTLE_MS: 100,        // setpoint/target/yaw 送信スロットル(10Hz)
  ROLL_PITCH_LIMIT_DEG: 5.0,    // UI層クランプ(既定±5°)
  ALT_MIN_M: 0.1,
  ALT_MAX_M: 1.0,
  YAW_LIMIT_DEG: 180.0,         // ヨー角スライダ範囲(契約 §3.2: ±180°)
  VOLT_WARN_V: 3.5,             // これ未満で警告(黄)
  VOLT_CRIT_V: 3.4,             // これ未満で危険(赤)
  CONSOLE_MAX_LINES: 200,       // コンソールのスクロールバック行数
  ECHO_SUPPRESS_MS: 1500,       // UI操作直後、20Hzのサーバecho上書きを抑制する猶予
  ATT_BAR_RANGE_DEG: 30,        // 姿勢バーのフルスケール(ファームクランプ±30°)
  YAW_BAR_RANGE_DEG: 180,       // ヨーバーのフルスケール
  ALT_BAR_MAX_M: 1.5,           // 高度バーのフルスケール(ファームクランプ上限)
  PLOT_RANGE_M: 2.0,            // XYプロットの表示半幅 [m]
  PLOT_GRID_M: 0.5,             // XYプロットのグリッド間隔 [m]
  TRAIL_MAX_POINTS: 600,        // 軌跡の保持点数(20Hz×30s)
  AF_DEFAULT_CHANNEL: 1,        // プロファイル編集「行追加」の既定チャネル
  AF_DEFAULT_ALT_M: 0.3,        // 同・既定初期高度 [m]
  DUTY_HIGH_MIN: 0.6,           // 高出力許可チェックが必要な duty 下限(契約 §3.6)
  DUTY_DEFAULT: 0.3,            // モーターテストの既定 duty
  CAL3D_TARGET_SAMPLES: 6000,   // 3D磁気収集の上限(進捗バー分母。サーバと同値)
  FF_POLL_MS: 5000,             // /api/ffprofile の定期ポーリング間隔
  EXP_FRESH_S: 0.5,             // TLM_EXP 表示の鮮度しきい値(UI表示用)
  RB_POLL_MS: 500,              // リジッドボディ確認(/api/mocap/bodies)のポーリング間隔
  MAP_POLL_MS: 500,             // 設定タブのマッピング・プレビューのポーリング間隔
  MULTI_YAW_LIMIT_DEG: 30,      // 複数機のヨー目標上限(server.json multi.max_yaw_ctrl_deg と同値。
                                //  XY 位置ループが制御座標系固定のため大ヨー保持は位置保持を劣化させる)
};

/* 軌道パラメータ(円・シャトル)の既定制限(/api/config 取得失敗時の
   フォールバック。正は control.json trajectory 節 — サーバ側が必ず再検証する) */
const TRAJ_FALLBACK = {
  radius_min_m: 0.05, radius_max_m: 1.5,
  period_min_s: 3.0, period_max_s: 120.0,
  center_abs_max_m: 2.0,
  shuttle_amplitude_min_m: 0.05, shuttle_amplitude_max_m: 0.5,
  speed_max_mps: 0.5,
  excursion_abs_max_m: 0.5,
};

/* MAC未設定プロファイルのプルダウン表示サフィックス */
const MAC_UNSET_SUFFIX = " ⚠ MAC未設定";

/* PROTOCOL.md の enum 定義(v2: 7=MOTOR_TEST, reason 11=mode_change) */
const FLIGHT_STATES = [
  { name: "INIT",        jp: "初期化" },
  { name: "CALIBRATION", jp: "キャリブレーション" },
  { name: "WAIT",        jp: "待機" },
  { name: "TAKEOFF",     jp: "離陸" },
  { name: "HOVER",       jp: "ホバリング" },
  { name: "LANDING",     jp: "着陸" },
  { name: "COMPLETE",    jp: "完了(要RESET)" },
  { name: "MOTOR_TEST",  jp: "モーターテスト" },
];
const STATE_COMPLETE = 6;
const REASONS = [
  "none", "start_cmd", "stop_cmd", "max_flight_time", "low_voltage",
  "start_rejected_low_voltage", "landed", "over_g", "link_loss", "reset_cmd",
  "start_rejected_not_ready", "mode_change",
];
/* TLM_STATE flags ビット定義 */
const FLAG_FLYING = 0x04; // bit2 = flying

/* TLM_STATE ekf2_status ビット(MAG_AUTOTUNE_DESIGN.md 契約 §1.1) */
const EKF2_STATUS_FUSED = 0x02;      // bit1: ヨー観測を直近0.5s内に受理
const EKF2_STATUS_RECAPTURE = 0x80;  // bit7: ソフト再捕捉(制限融合)中
const EKF2_FUSED_WIN_N = 100;        // fused率の窓(20Hz×100 ≈ 直近5s)

/* FF/推定モードの表示名(CMD_FF_MODE の enum に対応) */
const FF_MODE_NAMES = ["off", "A", "B"];

/* ===================== DOM参照 ===================== */
const $ = (id) => document.getElementById(id);
const els = {
  portSelect: $("portSelect"), btnRefreshPorts: $("btnRefreshPorts"), btnConnect: $("btnConnect"),
  airframeSelect: $("airframeSelect"), btnEditAirframes: $("btnEditAirframes"),
  afEditor: $("afEditor"), afKnownMacs: $("afKnownMacs"), afTbody: $("afTbody"),
  btnAfAddRow: $("btnAfAddRow"), afEditorMsg: $("afEditorMsg"),
  btnAfCancel: $("btnAfCancel"), btnAfSave: $("btnAfSave"),
  linkSerial: $("linkSerial"), linkRelay: $("linkRelay"), linkDrone: $("linkDrone"),
  voltage: $("voltage"),
  tabPosture: $("tabPosture"), tabPosition: $("tabPosition"), tabExperiment: $("tabExperiment"),
  tabMulti: $("tabMulti"),
  panelPosture: $("panelPosture"), panelPosition: $("panelPosition"), panelExperiment: $("panelExperiment"),
  panelMulti: $("panelMulti"),
  // 複数機タブ
  btnMultiStart: $("btnMultiStart"), btnMultiApply: $("btnMultiApply"),
  multiAirframeList: $("multiAirframeList"), multiSelectMsg: $("multiSelectMsg"),
  btnRbCheck: $("btnRbCheck"), rbList: $("rbList"),
  multiTargets: $("multiTargets"), multiCanvas: $("multiCanvas"),
  multiStatus: $("multiStatus"),
  mainEl: $("main"),
  rollSlider: $("rollSlider"), pitchSlider: $("pitchSlider"), altSlider: $("altSlider"),
  rollValue: $("rollValue"), pitchValue: $("pitchValue"), altValue: $("altValue"),
  btnCenter: $("btnCenter"), postureNote: $("postureNote"),
  targetX: $("targetX"), targetY: $("targetY"), targetZ: $("targetZ"),
  btnPresetHere: $("btnPresetHere"), btnPresetOrigin: $("btnPresetOrigin"),
  mocapStatus: $("mocapStatus"), mocapStatusText: $("mocapStatusText"), mocapCoords: $("mocapCoords"),
  xyCanvas: $("xyCanvas"), cmdRoll: $("cmdRoll"), cmdPitch: $("cmdPitch"),
  stateBadge: $("stateBadge"), phaseLabel: $("phaseLabel"), btnRearm: $("btnRearm"),
  attRollBar: $("attRollBar"), attPitchBar: $("attPitchBar"), attYawBar: $("attYawBar"),
  attRollNum: $("attRollNum"), attPitchNum: $("attPitchNum"), attYawNum: $("attYawNum"),
  attYawName: $("attYawName"),
  altCurBar: $("altCurBar"), altRefMarker: $("altRefMarker"),
  altCurNum: $("altCurNum"), altRefNum: $("altRefNum"),
  dutyBars: { fr: $("dutyFR"), fl: $("dutyFL"), rr: $("dutyRR"), rl: $("dutyRL") },
  dutyNums: { fr: $("dutyFRNum"), fl: $("dutyFLNum"), rr: $("dutyRRNum"), rl: $("dutyRLNum") },
  latency: $("latency"), relayStats: $("relayStats"),
  logToggle: $("logToggle"), logFile: $("logFile"),
  consoleEl: $("consoleEl"), overlay: $("overlay"), spaceHint: $("spaceHint"),
  // v2: 共通ヨーブロック(アクティブタブへ移設)
  yawBlock: $("yawBlock"), yawSlotPosture: $("yawSlotPosture"), yawSlotPosition: $("yawSlotPosition"),
  yawCtrlToggle: $("yawCtrlToggle"), yawSlider: $("yawSlider"), yawValue: $("yawValue"),
  btnYawCenter: $("btnYawCenter"), ffWarnBadge: $("ffWarnBadge"),
  ffQuickBlock: $("ffQuickBlock"), ffQuickSelect: $("ffQuickSelect"),
  btnFfQuickApply: $("btnFfQuickApply"), ffAppliedBanner: $("ffAppliedBanner"),
  // v2: ヨー推定モニタ
  ekfBadge: $("ekfBadge"), yawMadgwick: $("yawMadgwick"), yawEkf: $("yawEkf"),
  yawGyroInt: $("yawGyroInt"), yawMocapLabel: $("yawMocapLabel"), yawMocap: $("yawMocap"),
  yawRefMon: $("yawRefMon"), nisMon: $("nisMon"), ffgMon: $("ffgMon"),
  currentMon: $("currentMon"), ffModeMon: $("ffModeMon"),
  // v2: 円軌道
  trajSelect: $("trajSelect"), trajStatus: $("trajStatus"), circleParams: $("circleParams"),
  circleCx: $("circleCx"), circleCy: $("circleCy"), circleR: $("circleR"),
  circlePeriod: $("circlePeriod"), circleDir: $("circleDir"), circleAlt: $("circleAlt"),
  circleFaceTangent: $("circleFaceTangent"),
  btnCircleStart: $("btnCircleStart"), btnCircleStop: $("btnCircleStop"),
  shuttleParams: $("shuttleParams"),
  shuttleAxisMode: $("shuttleAxisMode"), shuttleAxisDeg: $("shuttleAxisDeg"),
  shuttleCx: $("shuttleCx"), shuttleCy: $("shuttleCy"), shuttleAmp: $("shuttleAmp"),
  shuttlePeriod: $("shuttlePeriod"), shuttleCycles: $("shuttleCycles"),
  shuttleAlt: $("shuttleAlt"),
  btnShuttleStart: $("btnShuttleStart"), btnShuttleStop: $("btnShuttleStop"),
  // v2: 評価シーケンス(スクリプト軌道)
  sequenceParams: $("sequenceParams"),
  seqPresetSelect: $("seqPresetSelect"), seqAlt: $("seqAlt"),
  seqStartIndex: $("seqStartIndex"), seqSegList: $("seqSegList"),
  seqTotal: $("seqTotal"), seqProgress: $("seqProgress"),
  btnTrajSeqStart: $("btnTrajSeqStart"), btnTrajSeqStop: $("btnTrajSeqStop"),
  // v2: Experiment タブ
  expActiveBadge: $("expActiveBadge"), btnExpActivate: $("btnExpActivate"),
  fixtureCheck: $("fixtureCheck"), dutyButtons: $("dutyButtons"),
  highDutyCheck: $("highDutyCheck"),
  btnMotorStart: $("btnMotorStart"), btnMotorApply: $("btnMotorApply"), btnMotorStop: $("btnMotorStop"),
  motorStatusText: $("motorStatusText"), expLive: $("expLive"),
  // 計測(EKF/FF性能ログ)パネル(T1-6: exp_record_start/stop)
  btnExpRecStart: $("btnExpRecStart"), btnExpRecStop: $("btnExpRecStop"),
  expRecStatus: $("expRecStatus"),
  // リアルタイムモニタ(ヨー/フロー速度/2D位置。canvas 自前描画)
  btnRtmonToggle: $("btnRtmonToggle"), rtmonBody: $("rtmonBody"),
  rtmonYawCanvas: $("rtmonYawCanvas"), rtmonVelCanvas: $("rtmonVelCanvas"),
  rtmonXyCanvas: $("rtmonXyCanvas"), btnRtmonReset: $("btnRtmonReset"),
  rtmonInfo: $("rtmonInfo"),
  sweepLocation: $("sweepLocation"), sweepOrientation: $("sweepOrientation"), sweepMemo: $("sweepMemo"),
  btnSweepStart: $("btnSweepStart"), btnSweepAbort: $("btnSweepAbort"),
  sweepStepTag: $("sweepStepTag"), sweepProgressFill: $("sweepProgressFill"),
  sweepPhase: $("sweepPhase"), sweepMessage: $("sweepMessage"), sweepResult: $("sweepResult"),
  btnSeqStart: $("btnSeqStart"), btnSeqResume: $("btnSeqResume"),
  btnSeqForce: $("btnSeqForce"), btnSeqAbort: $("btnSeqAbort"),
  seqProgress: $("seqProgress"), seqMessage: $("seqMessage"), seqMeta: $("seqMeta"),
  cal3dProgressFill: $("cal3dProgressFill"), cal3dStatusText: $("cal3dStatusText"),
  cal3dSamples: $("cal3dSamples"), cal3dFit: $("cal3dFit"), cal3dSaved: $("cal3dSaved"),
  accel6Captured: $("accel6Captured"), accel6Msg: $("accel6Msg"),
  accel6Accel: $("accel6Accel"), accel6Norm: $("accel6Norm"),
  quickcalMsg: $("quickcalMsg"),
  quickcalDroneRow: $("quickcalDroneRow"), quickcalDrone: $("quickcalDrone"),
  geomagSelect: $("geomagSelect"), btnGeomagApply: $("btnGeomagApply"),
  geomagInfo: $("geomagInfo"), geomagMsg: $("geomagMsg"),
  calprofName: $("calprofName"), btnCalprofSave: $("btnCalprofSave"),
  calprofSelect: $("calprofSelect"), btnCalprofApply: $("btnCalprofApply"),
  btnCalprofDelete: $("btnCalprofDelete"), calprofMsg: $("calprofMsg"),
  ffFolderSelect: $("ffFolderSelect"), ffExtractName: $("ffExtractName"),
  ffExtractMemo: $("ffExtractMemo"), btnFfExtract: $("btnFfExtract"),
  ffExtractResult: $("ffExtractResult"),
  ffProfileSelect: $("ffProfileSelect"), btnFfDelete: $("btnFfDelete"),
  ffModeSelect: $("ffModeSelect"), ffEstSelect: $("ffEstSelect"),
  btnFfApply: $("btnFfApply"), btnFfMode: $("btnFfMode"), btnFfAnchor: $("btnFfAnchor"),
  ffAppliedExp: $("ffAppliedExp"), ffApplyMsg: $("ffApplyMsg"),
  // 磁気オートチューン(EKF2)パネル
  yawRefSelect: $("yawRefSelect"), yawRefMotionInfo: $("yawRefMotionInfo"),
  ekf2YawInfo: $("ekf2YawInfo"), ekf2BmInfo: $("ekf2BmInfo"),
  ekf2StatusInfo: $("ekf2StatusInfo"), ekf2FusedInfo: $("ekf2FusedInfo"),
  // プリフライト・インターロック(P1-2: Posture/Position 両タブに同型)
  interlockBadges: document.querySelectorAll("[data-interlock-badge]"),
  interlockMsgs: document.querySelectorAll("[data-interlock-msg]"),
  forceStartBtns: document.querySelectorAll("[data-action=force-start]"),
  magbiasLogSelect: $("magbiasLogSelect"), btnMagbiasExtract: $("btnMagbiasExtract"),
  magbiasSelect: $("magbiasSelect"), btnMagbiasApply: $("btnMagbiasApply"),
  btnMagbiasClear: $("btnMagbiasClear"),
  magbiasApplied: $("magbiasApplied"), magbiasMsg: $("magbiasMsg"),
  // フロー較正(純回転フィット)パネル
  btnFlowcalStart: $("btnFlowcalStart"), btnFlowcalStop: $("btnFlowcalStop"),
  btnFlowcalApply: $("btnFlowcalApply"), btnFlowcalClear: $("btnFlowcalClear"),
  flowcalRecStatus: $("flowcalRecStatus"), flowcalBadge: $("flowcalBadge"),
  fcFillSamples: $("fcFillSamples"), fcValSamples: $("fcValSamples"),
  fcFillValid: $("fcFillValid"), fcValValid: $("fcValValid"),
  fcFillStdP: $("fcFillStdP"), fcValStdP: $("fcValStdP"),
  fcFillStdQ: $("fcFillStdQ"), fcValStdQ: $("fcValStdQ"),
  fcFillSqual: $("fcFillSqual"), fcValSqual: $("fcValSqual"),
  fcFillTof: $("fcFillTof"), fcValTof: $("fcValTof"),
  fcR2: $("fcR2"), fcScale: $("fcScale"), fcPhi: $("fcPhi"),
  fcUsed: $("fcUsed"), fcMatrix: $("fcMatrix"), fcDroneMatrix: $("fcDroneMatrix"),
  flowcalApplied: $("flowcalApplied"), flowcalMsg: $("flowcalMsg"),
  flowcalSelect: $("flowcalSelect"),
  btnFlowcalProfileApply: $("btnFlowcalProfileApply"),
  btnFlowcalProfileDelete: $("btnFlowcalProfileDelete"),
  // 設定タブ(UI専用: MoCap マッピング)
  tabSettings: $("tabSettings"), panelSettings: $("panelSettings"),
  mapAxisSel: { x: $("mapXAxis"), y: $("mapYAxis"), z: $("mapZAxis") },
  mapSignSel: { x: $("mapXSign"), y: $("mapYSign"), z: $("mapZSign") },
  mapFwdAxis: $("mapFwdAxis"), mapUpAxis: $("mapUpAxis"),
  mapYawSign: $("mapYawSign"), mapYawOffset: $("mapYawOffset"),
  btnYawZeroAlign: $("btnYawZeroAlign"), btnYawTlmAlign: $("btnYawTlmAlign"),
  mapFlipCorr: $("mapFlipCorr"), mapFlipGate: $("mapFlipGate"),
  btnMapPreview: $("btnMapPreview"), mapPreviewBox: $("mapPreviewBox"),
  btnMapApply: $("btnMapApply"), btnMapReload: $("btnMapReload"),
  mapMsg: $("mapMsg"),
};

/* ===================== 状態 ===================== */
let ws = null;
let wsOpen = false;
let uiMode = "posture";            // UI表示中のモード(サーバechoで同期)
let modeSentAt = -Infinity;        // set_mode送信時刻(echo抑制用)
let logToggleSentAt = -Infinity;   // set_logging送信時刻(echo抑制用)
let airframeSentAt = -Infinity;    // select_airframe送信時刻(echo抑制用)
let yawCtrlSentAt = -Infinity;     // set_yaw_control送信時刻(echo抑制用)
let trajTouchedAt = -Infinity;     // 軌道セレクタのユーザー操作時刻(echo抑制用)
let lastSession = null;            // 直近の session オブジェクト
let lastDrone = null;              // 直近の drone オブジェクト
let lastMocap = null;              // 直近の mocap オブジェクト
let airframes = [];                // /api/airframes の配列
const lastEventKeys = new Map();   // TLM_EVENT 2Hz再送のコンソール重複抑制(機体別)
const trail = [];                  // XYプロット軌跡 [{x,y}]

// 設定タブ(UI専用: サーバの session モードとは独立)
let settingsOpen = false;          // 設定タブ表示中(モードecho同期から除外)
let mapPollTimer = null;           // マッピング・プレビューのポーリングタイマ
let mapBodies = null;              // 直近の /api/mocap/bodies(ゼロ合わせ用)
let mapPrimaryRbId = null;         // 単機 Position の primary RB ID(GET mapping 由来)
let appliedMapping = null;         // サーバ適用済みマッピング(ゼロ合わせの前提検査用)

// v2: Experiment / FF 関連の REST 状態キャッシュ
let selectedDuty = UI.DUTY_DEFAULT;
let ffStatus = null;               // /api/ffprofile の状態
let magbiasStatus = null;          // /api/magbias の状態
let flowcalStatus = null;          // /api/flowcal の状態
let flowcalPollTimer = null;       // 記録中のライブメーターポーリング
let yawRefSentAt = -Infinity;      // ヨー基準ソース送信時刻(echo抑制用)
const ekf2FusedWin = [];           // fused ビット履歴(20Hz、直近 EKF2_FUSED_WIN_N)
let geomagStatus = null;           // /api/geomag の状態
let calprofStatus = null;          // /api/calprofile の状態
let accel6Status = null;           // /api/accel6 の状態
let cal3dStatus = null;            // /api/cal3d の状態(fit/saved を含む)
let trajLimits = { ...TRAJ_FALLBACK };  // /api/config の trajectory 節
let trajSequences = {};            // /api/config の trajectory.sequences(評価シーケンス)

const now = () => performance.now();
const clamp = (v, lo, hi) => Math.min(hi, Math.max(lo, v));

/* TLM_STATE角度フィールドは「同名でdeg換算」が契約だが、*_deg別名にも耐性を持たせる */
function pick(obj, ...names) {
  if (!obj) return null;
  for (const n of names) {
    if (obj[n] !== undefined && obj[n] !== null) return obj[n];
  }
  return null;
}

/* ===================== WebSocket ===================== */
function wsConnect() {
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  ws = new WebSocket(`${proto}//${location.host}/ws`);

  ws.onopen = () => {
    wsOpen = true;
    els.overlay.classList.remove("visible");
    appendConsole("ui", "サーバーに接続しました");
  };
  ws.onclose = () => {
    if (wsOpen) appendConsole("ui", "サーバーとの接続が切断されました。再接続します…");
    wsOpen = false;
    els.overlay.classList.add("visible");
    renderConnectivityLost();
    setTimeout(wsConnect, UI.WS_RECONNECT_MS); // 1秒バックオフで自動再接続
  };
  ws.onerror = () => { /* onclose が後続するためここでは何もしない */ };
  ws.onmessage = (ev) => {
    let msg;
    try { msg = JSON.parse(ev.data); } catch { return; }
    if (msg.type === "state") onState(msg.data || {});
    else if (msg.type === "event") onEvent(msg.data !== undefined ? msg.data : msg);
    else if (msg.type === "log") onLog(msg);
  };
}

function wsSend(obj) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify(obj));
    return true;
  }
  return false;
}

const sendCommand = (action, extra = {}) => wsSend({ type: "command", action, ...extra });

/* ===================== 送信スロットル(10Hz) ===================== */
function makeThrottledSender(sendFn) {
  let lastSent = -Infinity;
  let timer = null;
  return () => {
    const elapsed = now() - lastSent;
    if (elapsed >= UI.SEND_THROTTLE_MS) {
      lastSent = now();
      sendFn();
    } else if (timer === null) {
      // 末尾の値を確実に送るためトレーリング送信を予約
      timer = setTimeout(() => {
        timer = null;
        lastSent = now();
        sendFn();
      }, UI.SEND_THROTTLE_MS - elapsed);
    }
  };
}

function sendSetpointNow() {
  wsSend({
    type: "setpoint",
    roll_deg: clamp(parseFloat(els.rollSlider.value), -UI.ROLL_PITCH_LIMIT_DEG, UI.ROLL_PITCH_LIMIT_DEG),
    pitch_deg: clamp(parseFloat(els.pitchSlider.value), -UI.ROLL_PITCH_LIMIT_DEG, UI.ROLL_PITCH_LIMIT_DEG),
    alt_m: clamp(parseFloat(els.altSlider.value), UI.ALT_MIN_M, UI.ALT_MAX_M),
  });
}
function sendTargetNow() {
  wsSend({
    type: "target",
    x: parseFloat(els.targetX.value) || 0,
    y: parseFloat(els.targetY.value) || 0,
    z: clamp(parseFloat(els.targetZ.value) || UI.ALT_MIN_M, UI.ALT_MIN_M, UI.ALT_MAX_M),
  });
}
function sendYawNow() {
  // 共通ヨー角スライダ(両モードのコントローラへ反映される)
  wsSend({
    type: "yaw",
    yaw_deg: clamp(parseFloat(els.yawSlider.value) || 0,
                   -UI.YAW_LIMIT_DEG, UI.YAW_LIMIT_DEG),
  });
}
const sendSetpointThrottled = makeThrottledSender(sendSetpointNow);
const sendTargetThrottled = makeThrottledSender(sendTargetNow);
const sendYawThrottled = makeThrottledSender(sendYawNow);

/* ===================== REST 汎用ヘルパ ===================== */
async function apiGet(path, quiet = false) {
  try {
    const res = await fetch(path);
    return await res.json();
  } catch {
    // 定期ポーリング(quiet)はサーバ停止中のコンソール氾濫を避けて黙る
    if (!quiet) appendConsole("ui", `${path} の取得に失敗しました`);
    return null;
  }
}

async function apiPost(path, body) {
  try {
    const res = await fetch(path, {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    return await res.json();
  } catch {
    appendConsole("ui", `${path} との通信に失敗しました`);
    return null;
  }
}

async function apiPut(path, body) {
  try {
    const res = await fetch(path, {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(body || {}),
    });
    return await res.json();
  } catch {
    appendConsole("ui", `${path} との通信に失敗しました`);
    return null;
  }
}

/* ボタンを一時的に無効化して非同期操作を実行する(二重送信防止) */
async function withBusy(btn, fn) {
  if (btn) btn.disabled = true;
  try {
    await fn();
  } finally {
    if (btn) btn.disabled = false;
    updateExperimentControls();
  }
}

/* select を options で再構築する(可能なら現在の選択を維持) */
function rebuildSelect(sel, options, preferred) {
  const prev = sel.value;
  sel.innerHTML = "";
  for (const o of options) {
    const opt = document.createElement("option");
    opt.value = o.value;
    opt.textContent = o.label;
    if (o.title) opt.title = o.title;
    sel.appendChild(opt);
  }
  const want = (prev && options.some((o) => o.value === prev)) ? prev
    : (preferred && options.some((o) => o.value === preferred)) ? preferred : null;
  if (want !== null) sel.value = want;
}

/* ===================== REST ===================== */
async function fetchPorts() {
  try {
    const res = await fetch("/api/ports");
    const ports = await res.json(); // [{device, description}]
    const prev = els.portSelect.value;
    els.portSelect.innerHTML = "";
    for (const p of ports) {
      const opt = document.createElement("option");
      opt.value = p.device;
      opt.textContent = p.description ? `${p.device} — ${p.description}` : p.device;
      els.portSelect.appendChild(opt);
    }
    if (prev && ports.some((p) => p.device === prev)) els.portSelect.value = prev;
    if (ports.length === 0) {
      const opt = document.createElement("option");
      opt.value = "";
      opt.textContent = "(ポートなし)";
      els.portSelect.appendChild(opt);
    }
  } catch {
    appendConsole("ui", "ポート一覧の取得に失敗しました (/api/ports)");
  }
}

function macIsSet(mac) {
  return typeof mac === "string" && mac.trim() !== "";
}

/* プルダウンを airframes 配列から再構築する(MAC未設定は ⚠ サフィックス付き)。
   現在の選択(なければサーバ側選択)が新リストに残っていれば維持する。 */
function renderAirframeOptions() {
  const prev = els.airframeSelect.value ||
               (lastSession && lastSession.airframe) || "";
  els.airframeSelect.innerHTML = "";
  for (const a of airframes) {
    const opt = document.createElement("option");
    opt.value = a.name;
    opt.textContent = macIsSet(a.mac) ? a.name : a.name + MAC_UNSET_SUFFIX;
    opt.title = a.notes || "";
    els.airframeSelect.appendChild(opt);
  }
  if (prev && airframes.some((a) => a.name === prev)) {
    els.airframeSelect.value = prev;
  }
  renderMultiAirframeList();   // 複数機タブの選択候補も同じ一覧に追従させる
}

async function fetchAirframes() {
  try {
    const res = await fetch("/api/airframes");
    const body = await res.json(); // airframes.json の内容: {"airframes":[...]}
    airframes = Array.isArray(body) ? body : (body.airframes || []);
    renderAirframeOptions();
  } catch {
    appendConsole("ui", "機体プロファイルの取得に失敗しました (/api/airframes)");
  }
}

/* /api/config から軌道(円・シャトル)パラメータ制限を取り込み、入力欄の
   min/max に反映(正はサーバ側 control.json — ここは操作性のための表示制約のみ) */
async function fetchConfigLimits() {
  const body = await apiGet("/api/config");
  const traj = body && body.control && body.control.trajectory;
  if (!traj) return;
  trajLimits = { ...TRAJ_FALLBACK, ...traj };
  els.circleR.min = String(trajLimits.radius_min_m);
  els.circleR.max = String(trajLimits.radius_max_m);
  els.circlePeriod.min = String(trajLimits.period_min_s);
  els.circlePeriod.max = String(trajLimits.period_max_s);
  els.circleCx.min = String(-trajLimits.center_abs_max_m);
  els.circleCx.max = String(trajLimits.center_abs_max_m);
  els.circleCy.min = String(-trajLimits.center_abs_max_m);
  els.circleCy.max = String(trajLimits.center_abs_max_m);
  // シャトル: 振幅は専用キー、周期は circle と共通、中心は可動域(端点
  // center±A·e が ±excursion 内)の表示制約として ±excursion を使う
  els.shuttleAmp.min = String(trajLimits.shuttle_amplitude_min_m);
  els.shuttleAmp.max = String(trajLimits.shuttle_amplitude_max_m);
  els.shuttlePeriod.min = String(trajLimits.period_min_s);
  els.shuttlePeriod.max = String(trajLimits.period_max_s);
  els.shuttleCx.min = String(-trajLimits.excursion_abs_max_m);
  els.shuttleCx.max = String(trajLimits.excursion_abs_max_m);
  els.shuttleCy.min = String(-trajLimits.excursion_abs_max_m);
  els.shuttleCy.max = String(trajLimits.excursion_abs_max_m);
  // 評価シーケンスのプリセット(trajectory.sequences)→ 選択肢+一覧を構築
  trajSequences = (traj.sequences && typeof traj.sequences === "object")
    ? traj.sequences : {};
  renderSeqPresets();
}

/* ===================== 評価シーケンス(スクリプト軌道) ===================== */
/* プリセットは control.json trajectory.sequences(サーバが正)。ここでは
   一覧表示・時間見積り・開始インデックス選択のみを担う。 */

const SEQ_TYPE_JP = { hover: "ホバリング", circle: "円軌道", shuttle: "往復", yaw: "ヨー回頭" };

/* サーバ側 PositionController._segment_estimate_s と同じ静的見積り
   (shuttle は中心合流を仮定し端点停止まで +T/4、yaw は開始ヨー 0° を仮定) */
function seqSegmentEstimateS(seg) {
  if (seg.type === "hover") return seg.duration_s || 0;
  if (seg.type === "circle") return (seg.laps || 0) * (seg.period_s || 0);
  if (seg.type === "shuttle") return ((seg.cycles || 0) + 0.25) * (seg.period_s || 0);
  if (seg.type === "yaw") {
    let prev = 0;
    let total = 0;
    for (const tgt of (seg.targets_deg || [])) {
      const delta = ((tgt - prev + 180) % 360 + 360) % 360 - 180;  // 最短経路
      total += Math.abs(delta) / (seg.rate_dps || 1) + (seg.hold_s || 0);
      prev = tgt;
    }
    return total;
  }
  return 0;
}

/* サーバ側 _sequence_transit_estimates と同じセグメント間トランジット
   (合流点まで 0.25 m/s の等速直線)見積り。開始位置は開始時まで不明の
   ため目標既定 (0,0) を仮定する(「約」表示の前提。実行中の残り秒は
   サーバ側 snapshot が正)。 */
const SEQ_TRANSIT_EPS_M = 0.05;
const SEQ_TRANSIT_SPEED_MPS = 0.25;
function seqTransitEstimatesS(segs, startIdx) {
  let px = 0;
  let py = 0;
  const out = segs.map(() => 0);
  for (let i = startIdx; i < segs.length; i++) {
    const seg = segs[i];
    if (seg.type === "circle") {
      const cx = seg.center_x || 0;
      const cy = seg.center_y || 0;
      const r = seg.radius_m || 0;
      const norm = Math.hypot(px - cx, py - cy);
      const mx = norm > 0 ? cx + r * (px - cx) / norm : cx + r;  // 縮退: 位相0
      const my = norm > 0 ? cy + r * (py - cy) / norm : cy;
      const dist = Math.hypot(mx - px, my - py);
      if (dist > SEQ_TRANSIT_EPS_M) out[i] = dist / SEQ_TRANSIT_SPEED_MPS;
      px = mx; py = my;                  // laps·2π 後は合流点へ戻る
    } else if (seg.type === "shuttle") {
      const cx = seg.center_x || 0;
      const cy = seg.center_y || 0;
      const amp = seg.amplitude_m || 0;
      const th = (seg.axis_deg || 0) * Math.PI / 180;
      const ex = Math.cos(th);
      const ey = Math.sin(th);
      const s = Math.max(-amp, Math.min(amp, (px - cx) * ex + (py - cy) * ey));
      const mx = cx + s * ex;
      const my = cy + s * ey;
      const dist = Math.hypot(mx - px, my - py);
      if (dist > SEQ_TRANSIT_EPS_M) out[i] = dist / SEQ_TRANSIT_SPEED_MPS;
      // 終端は停止極値の端点(サーバ _shuttle_stop_phase と同じ規則)
      const base = Math.asin(amp > 0 ? s / amp : 0) + (seg.cycles || 0) * 2 * Math.PI;
      const k = Math.ceil((base - Math.PI / 2) / Math.PI - 1e-9);
      const sEnd = amp * Math.sin(Math.PI / 2 + k * Math.PI);
      px = cx + sEnd * ex; py = cy + sEnd * ey;
    }
    // hover/yaw: 現在点を保持(トランジット不要)
  }
  return out;
}

function seqSegmentLabel(seg) {
  if (seg.type === "hover") return `ホバリング ${seg.duration_s}s`;
  if (seg.type === "circle") {
    return `円軌道 R${seg.radius_m}m T${seg.period_s}s ${seg.laps}周` +
      (seg.clockwise ? " CW" : " CCW");
  }
  if (seg.type === "shuttle") {
    return `往復 軸${seg.axis_deg}° A${seg.amplitude_m}m ` +
      `T${seg.period_s}s ${seg.cycles}往復`;
  }
  if (seg.type === "yaw") {
    const targets = (seg.targets_deg || [])
      .map((v) => (v > 0 ? "+" : "") + v).join("→");
    return `ヨー回頭 ${targets}°(${seg.rate_dps}°/s・保持${seg.hold_s}s)`;
  }
  return String(seg.type || "?");
}

function seqSelectedSegments() {
  return trajSequences[els.seqPresetSelect.value] || [];
}

function renderSeqPresets() {
  const names = Object.keys(trajSequences);
  const prev = els.seqPresetSelect.value;
  els.seqPresetSelect.innerHTML = "";
  for (const name of names) {
    const opt = document.createElement("option");
    opt.value = name;
    opt.textContent = name;
    els.seqPresetSelect.appendChild(opt);
  }
  if (names.includes(prev)) els.seqPresetSelect.value = prev;
  renderSeqList();
}

/* セグメント一覧(タイプ・パラメータ・見積り秒)+開始インデックス+合計。
   バッテリ配慮のため「このセグメントから開始」より前は淡色(skipped)表示 */
function renderSeqList() {
  const segs = seqSelectedSegments();
  const prevIdx = parseInt(els.seqStartIndex.value, 10) || 0;
  els.seqStartIndex.innerHTML = "";
  segs.forEach((seg, i) => {
    const opt = document.createElement("option");
    opt.value = String(i);
    opt.textContent = `${i + 1}: ${SEQ_TYPE_JP[seg.type] || seg.type}`;
    els.seqStartIndex.appendChild(opt);
  });
  const startIdx = segs.length
    ? Math.min(Math.max(prevIdx, 0), segs.length - 1) : 0;
  if (segs.length) els.seqStartIndex.value = String(startIdx);
  els.seqSegList.innerHTML = "";
  let total = 0;
  // 合計にはセグメント間トランジット(合流点への等速移動)分も含める
  const transits = seqTransitEstimatesS(segs, startIdx);
  segs.forEach((seg, i) => {
    const est = seqSegmentEstimateS(seg);
    if (i >= startIdx) total += est + transits[i];
    const row = document.createElement("div");
    row.className = "seq-seg-row" + (i < startIdx ? " skipped" : "");
    const no = document.createElement("span");
    no.className = "seq-seg-no mono";
    no.textContent = String(i + 1);
    const label = document.createElement("span");
    label.className = "seq-seg-label";
    label.textContent = seqSegmentLabel(seg);
    const estEl = document.createElement("span");
    estEl.className = "seq-seg-est mono";
    estEl.textContent = `${est.toFixed(0)}s`;
    row.append(no, label, estEl);
    els.seqSegList.appendChild(row);
  });
  els.seqTotal.textContent = segs.length
    ? `合計見積り: 約${Math.round(total)}s(セグメント${startIdx + 1}〜${segs.length})`
    : "プリセットがありません(control.json trajectory.sequences)";
}

/* 実行中の進行表示(n/N・現セグメント種別・残り秒)+一覧ハイライト */
function updateSeqProgress(traj) {
  if (!traj) {
    els.seqProgress.classList.add("hidden");
    for (const row of els.seqSegList.children) row.classList.remove("running");
    return;
  }
  const segType = SEQ_TYPE_JP[traj.seg_type] || traj.seg_type || "--";
  // トランジット(合流点への等速移動)中は「移動中」を明示する
  const phaseLabel = traj.transit ? `${segType}へ移動中` : segType;
  els.seqProgress.textContent =
    `実行中: ${traj.name} ${traj.seg_index + 1}/${traj.seg_count}(${phaseLabel})` +
    ` 残り セグメント${Math.ceil(traj.seg_remaining_s)}s /` +
    ` 全体${Math.ceil(traj.remaining_s)}s`;
  els.seqProgress.classList.remove("hidden");
  // 表示中プリセットが実行中のものと同じときのみ行ハイライトを追従
  const highlight = els.seqPresetSelect.value === traj.name;
  Array.from(els.seqSegList.children).forEach((row, i) => {
    row.classList.toggle("running", highlight && i === traj.seg_index);
  });
  // yaw セグメントが自動操作するヨー目標をスライダ表示へ同期する
  // (input イベントは発火させない = サーバへ送り返さない)。シーケンス
  // 停止後はスライダが最終ヨー目標を指し、次回操作時のジャンプが消える
  if (typeof traj.target_yaw_rad === "number") {
    const deg = traj.target_yaw_rad * 180 / Math.PI;
    els.yawSlider.value = deg.toFixed(1);
    els.yawValue.textContent = fmtDeg(deg);
  }
}

/* ===================== 機体プロファイル編集 ===================== */
/* 契約: PUT /api/airframes {"airframes":[...]} → {"ok","error","airframes"}。
   検証(名前一意・MAC形式・チャネル/バイアス/高度範囲)はサーバが正。 */

let afRows = [];   // 編集中の行データ(airframes のディープコピー)

function afBlankRow() {
  return {
    name: "",
    mac: "",
    wifi_channel: UI.AF_DEFAULT_CHANNEL,
    roll_bias_deg: 0,
    pitch_bias_deg: 0,
    default_alt_m: UI.AF_DEFAULT_ALT_M,
    rigid_body_id: null,   // 複数機モード用(null=未設定)
    notes: "",
  };
}

function afMakeInput(row, field, type, opts = {}) {
  const input = document.createElement("input");
  input.type = type;
  if (opts.step !== undefined) input.step = String(opts.step);
  if (opts.placeholder) input.placeholder = opts.placeholder;
  if (opts.className) input.className = opts.className;
  input.value = row[field] === undefined || row[field] === null ? "" : String(row[field]);
  input.addEventListener("input", () => {
    if (type === "number") {
      const v = input.step && input.step.indexOf(".") < 0
        ? parseInt(input.value, 10) : parseFloat(input.value);
      row[field] = Number.isNaN(v) ? null : v;   // null はサーバ検証で弾かれる
    } else {
      row[field] = input.value;
    }
  });
  const td = document.createElement("td");
  td.appendChild(input);
  return td;
}

function renderAfEditorRows() {
  els.afTbody.innerHTML = "";
  for (const row of afRows) {
    const tr = document.createElement("tr");
    tr.appendChild(afMakeInput(row, "name", "text", { className: "af-name" }));
    tr.appendChild(afMakeInput(row, "mac", "text",
      { className: "af-mac mono", placeholder: "(未設定)" }));
    tr.appendChild(afMakeInput(row, "wifi_channel", "number",
      { step: 1, className: "af-ch" }));
    tr.appendChild(afMakeInput(row, "roll_bias_deg", "number",
      { step: 0.001, className: "af-num" }));
    tr.appendChild(afMakeInput(row, "pitch_bias_deg", "number",
      { step: 0.001, className: "af-num" }));
    tr.appendChild(afMakeInput(row, "default_alt_m", "number",
      { step: 0.05, className: "af-num" }));
    tr.appendChild(afMakeInput(row, "rigid_body_id", "number",
      { step: 1, className: "af-ch", placeholder: "-" }));
    tr.appendChild(afMakeInput(row, "notes", "text", { className: "af-notes" }));

    const tdDel = document.createElement("td");
    const btnDel = document.createElement("button");
    btnDel.type = "button";
    btnDel.className = "btn btn-small";
    btnDel.textContent = "行削除";
    btnDel.addEventListener("click", () => {
      afRows.splice(afRows.indexOf(row), 1);
      renderAfEditorRows();
    });
    tdDel.appendChild(btnDel);
    tr.appendChild(tdDel);
    els.afTbody.appendChild(tr);
  }
}

function setAfEditorMsg(text, isError) {
  els.afEditorMsg.textContent = text || "";
  els.afEditorMsg.classList.toggle("err", !!isError);
}

function openAirframeEditor() {
  afRows = airframes.map((a) => ({ ...a }));   // ディープコピー(1段で十分)
  // 既知の候補 MAC(設定済み MAC の一覧)をヒントに表示
  const known = [...new Set(airframes.map((a) => (a.mac || "").trim()).filter(Boolean))];
  els.afKnownMacs.textContent = known.length ? known.join(" / ") : "-";
  setAfEditorMsg("", false);
  renderAfEditorRows();
  els.afEditor.classList.add("visible");
}

function closeAirframeEditor() {
  els.afEditor.classList.remove("visible");
}

async function saveAirframes() {
  els.btnAfSave.disabled = true;
  setAfEditorMsg("保存中…", false);
  try {
    const res = await fetch("/api/airframes", {
      method: "PUT",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ airframes: afRows }),
    });
    const body = await res.json(); // {"ok","error","airframes"}
    if (body.ok) {
      airframes = body.airframes || [];
      renderAirframeOptions();
      appendConsole("ui", `機体プロファイルを保存しました(${airframes.length}件)`);
      closeAirframeEditor();
    } else {
      setAfEditorMsg(body.error || "保存に失敗しました", true);
    }
  } catch {
    setAfEditorMsg("サーバとの通信に失敗しました (/api/airframes)", true);
  } finally {
    els.btnAfSave.disabled = false;
  }
}

/* ===================== サーバ→UI: state(20Hz) ===================== */
function onState(data) {
  lastDrone = data.drone || null;
  lastMocap = data.mocap !== undefined ? data.mocap : null;
  lastSession = data.session || null;

  renderHeader();
  renderSession();
  renderDrone();
  renderYawMonitor();
  renderMocap();
  renderExperiment();
  renderMulti();
  rtmonOnState();   // リアルタイムモニタのサンプリング(描画は10Hz間引き)
  if (uiMode === "position") drawPlot();
  if (uiMode === "multi") drawMultiPlot();
}

function onEvent(ev) {
  /* TLM_EVENT: {state, prev_state, reason, flags, voltage}。2Hz再送は重複表示しない */
  const state = pick(ev, "state");
  const prev = pick(ev, "prev_state");
  const reason = pick(ev, "reason");
  const voltage = pick(ev, "voltage");
  // 複数機イベントは機体名つき。重複抑制キーも機体別に持つ
  // (別機体の同一遷移を取りこぼさない)
  const drone = pick(ev, "drone");
  const who = drone !== null && drone !== undefined ? String(drone) : "single";
  const key = `${prev}>${state}:${reason}`;
  if (lastEventKeys.get(who) === key) return;
  lastEventKeys.set(who, key);

  const sName = (i) => (FLIGHT_STATES[i] ? FLIGHT_STATES[i].name : `?${i}`);
  const rName = REASONS[reason] !== undefined ? REASONS[reason] : `?${reason}`;
  const vStr = typeof voltage === "number" ? ` ${voltage.toFixed(2)}V` : "";
  const tag = who === "single" ? "" : `[${who}] `;
  appendConsole("event",
    `${tag}${sName(prev)} → ${sName(state)} (${rName})${vStr}`);
}

function onLog(msg) {
  /* {"type":"log","origin":0|1,"line":...}; origin 0=relay, 1=drone(文字列にも耐性) */
  const o = msg.origin;
  const tag = (o === 1 || o === "drone") ? "drone" : (o === 0 || o === "relay") ? "relay" : "ui";
  appendConsole(tag, String(msg.line !== undefined ? msg.line : ""));
}

/* ===================== 描画: ヘッダ ===================== */
/* on=点灯(緑)。warn=true なら警告色(黄)で点灯 — 「生きているが要注意」状態 */
function setLinkInd(el, on, warn = false) {
  el.classList.toggle("on", !!on && !warn);
  el.classList.toggle("warn", !!on && !!warn);
}

function renderHeader() {
  const s = lastSession;
  const serialOn = !!(s && s.serial_connected);
  setLinkInd(els.linkSerial, serialOn);

  // リレー鮮度: サーバが RLY_STATS(1Hz)の受信時刻から判定した relay_fresh を使う。
  // リレー生存かつ ESP-NOW ターゲット未設定(relay_target_ok=false)は警告色で区別。
  const relayFresh = serialOn && !!(s && s.relay_fresh);
  const targetOk = !!(s && s.relay_target_ok);
  setLinkInd(els.linkRelay, relayFresh, !targetOk);
  els.linkRelay.title = (relayFresh && !targetOk)
    ? "リレー応答あり / ESP-NOWターゲット未設定(機体宛コマンドは転送されません)"
    : "";

  setLinkInd(els.linkDrone, !!(lastDrone && lastDrone.fresh));

  // 接続ボタンのトグル表示
  els.btnConnect.textContent = serialOn ? "切断" : "接続";
  els.btnConnect.classList.toggle("btn-danger", serialOn);
  els.btnConnect.classList.toggle("btn-primary", !serialOn);

  // 電圧(<3.5V 警告, <3.4V 危険)
  const v = lastDrone ? pick(lastDrone, "voltage") : null;
  if (typeof v === "number") {
    els.voltage.textContent = `${v.toFixed(2)} V`;
    els.voltage.className = "voltage " +
      (v < UI.VOLT_CRIT_V ? "v-crit" : v < UI.VOLT_WARN_V ? "v-warn" : "v-ok");
  } else {
    els.voltage.textContent = "--.- V";
    els.voltage.className = "voltage v-na";
  }
}

function renderConnectivityLost() {
  /* WS切断時はリンク表示を即時オフにし、操作系を安全側(無効)へ倒す */
  setLinkInd(els.linkSerial, false);
  setLinkInd(els.linkRelay, false);
  setLinkInd(els.linkDrone, false);
  for (const el of [els.rollSlider, els.pitchSlider, els.altSlider, els.btnCenter,
                    els.yawCtrlToggle, els.yawSlider, els.btnYawCenter,
                    els.btnMotorStart, els.btnMotorApply,
                    els.btnSweepStart, els.btnSweepAbort,
                    els.btnSeqStart, els.btnSeqAbort,
                    els.btnMultiStart, els.btnMultiApply]) {
    el.disabled = true;
  }
  els.postureNote.textContent = "スライダはシリアル接続後に操作できます";
  els.postureNote.classList.remove("hidden");
  stopRbCheck();                // WS 断で RB 確認ポーリングも停止
  stopMapPreview();             // 設定タブのマッピング・プレビューも停止
  updateExperimentControls();   // wsOpen=false で実験操作系も安全側へ
}

/* ===================== 描画: セッション/モニタ ===================== */
function isFlying() {
  if (lastSession && lastSession.phase === "flying") return true;
  if (lastDrone && typeof lastDrone.flags === "number" &&
      (lastDrone.flags & FLAG_FLYING)) return true;
  // 複数機モード: いずれかのスロットが開始/飛行中なら「飛行中」扱い
  const multi = lastSession && lastSession.multi;
  if (multi && (multi.drones || []).some((d) => d.phase !== "idle")) return true;
  return false;
}

function renderSession() {
  const s = lastSession;
  if (!s) return;

  // サーバ側モードへタブを同期(自分の set_mode 直後は抑制)
  if (s.mode && s.mode !== uiMode && now() - modeSentAt > UI.ECHO_SUPPRESS_MS) {
    applyMode(s.mode, false);
  }

  els.phaseLabel.textContent = `phase: ${s.phase || "-"} / mode: ${s.mode || "-"} / 機体: ${s.airframe || "-"}`;

  // 機体プロファイルのサーバ側選択をドロップダウンへ反映(ユーザー操作直後は抑制)
  if (s.airframe && els.airframeSelect.value !== s.airframe &&
      now() - airframeSentAt > UI.ECHO_SUPPRESS_MS &&
      airframes.some((a) => a.name === s.airframe)) {
    els.airframeSelect.value = s.airframe;
  }

  // レイテンシ
  els.latency.textContent = typeof s.latency_ms === "number" ? `${s.latency_ms.toFixed(1)} ms` : "-- ms";

  // リレー統計(コンパクト表示)
  if (s.relay_stats) {
    const r = s.relay_stats;
    const e = (r.crc_errors || 0) + (r.cobs_errors || 0) + (r.espnow_send_fail || 0) + (r.overflow_drops || 0);
    els.relayStats.textContent = `relay ↑${r.up_frames ?? "-"} ↓${r.down_frames ?? "-"} err:${e}`;
    els.relayStats.classList.toggle("err", e > 0);
  } else {
    els.relayStats.textContent = "";
  }

  // ログ保存トグル+ファイル名(サーバが正。ユーザー操作直後のみecho上書きを抑制)
  if (now() - logToggleSentAt > UI.ECHO_SUPPRESS_MS) {
    els.logToggle.checked = !!s.logging;
  }
  els.logFile.textContent = s.log_file || "-";

  // Position: 機体計算指令(機上XY制御 — 機体側 XY PID が計算した
  // roll_ref/pitch_ref の TLM_STATE エコー)を表示する
  els.cmdRoll.textContent = fmtDeg(pick(lastDrone, "roll_ref"));
  els.cmdPitch.textContent = fmtDeg(pick(lastDrone, "pitch_ref"));

  // ヨー角制御トグルのサーバecho同期(ユーザー操作直後は抑制)
  if (now() - yawCtrlSentAt > UI.ECHO_SUPPRESS_MS) {
    els.yawCtrlToggle.checked = !!s.yaw_ctrl_on;
  }

  // Posture スライダの有効/無効。
  // 高度: 接続中なら地上でも操作可(離陸目標高度を START 前に選べる)。
  // roll/pitch(+中央戻し): 安全のため従来どおり飛行中のみ操作可。
  const altEnable = wsOpen && uiMode === "posture" && !!s.serial_connected;
  const rpEnable = altEnable && isFlying();
  els.altSlider.disabled = !altEnable;
  for (const el of [els.rollSlider, els.pitchSlider, els.btnCenter]) {
    el.disabled = !rpEnable;
  }
  els.postureNote.textContent = altEnable
    ? "Roll/Pitch は飛行中のみ操作できます(高度は離陸前から変更可)"
    : "スライダはシリアル接続後に操作できます";
  els.postureNote.classList.toggle("hidden", rpEnable);

  // ヨー角制御(Posture/Position 共通ブロック)
  const yawTabActive = uiMode === "posture" || uiMode === "position";
  const yawToggleEnable = wsOpen && yawTabActive && !!s.serial_connected;
  const yawSliderEnable = yawToggleEnable && els.yawCtrlToggle.checked;
  els.yawCtrlToggle.disabled = !yawToggleEnable;
  els.yawSlider.disabled = !yawSliderEnable;
  els.btnYawCenter.disabled = !yawSliderEnable;
  els.ffQuickBlock.classList.toggle("hidden", !els.yawCtrlToggle.checked);

  // 軌道(円・シャトル・評価シーケンス)の状態表示+ボタン活性(サーバ側 trajectory が正)
  const traj = s.trajectory;
  const circleRunning = !!(traj && traj.mode === "circle");
  const shuttleRunning = !!(traj && traj.mode === "shuttle");
  const sequenceRunning = !!(traj && traj.mode === "sequence");
  const trajRunning = circleRunning || shuttleRunning || sequenceRunning;
  if (trajRunning) {
    const phaseDeg = typeof traj.phase_rad === "number"
      ? (traj.phase_rad * 180 / Math.PI).toFixed(0) : "--";
    if (circleRunning) {
      els.trajStatus.textContent = `円軌道 実行中 φ=${phaseDeg}°`;
    } else if (shuttleRunning) {
      const remaining = typeof traj.cycles_remaining === "number"
        ? ` 残り${traj.cycles_remaining.toFixed(1)}周` : "(連続)";
      els.trajStatus.textContent = `往復軌道 実行中 φ=${phaseDeg}°${remaining}`;
    } else {
      els.trajStatus.textContent =
        `シーケンス ${traj.seg_index + 1}/${traj.seg_count} ` +
        `残り${Math.ceil(traj.remaining_s)}s`;
    }
    els.trajStatus.classList.remove("hidden");
    // サーバ側で軌道実行中なら軌道セレクタを追従させる(直後の操作は抑制)
    const wantMode = circleRunning ? "circle"
      : shuttleRunning ? "shuttle" : "sequence";
    if (now() - trajTouchedAt > UI.ECHO_SUPPRESS_MS &&
        els.trajSelect.value !== wantMode) {
      els.trajSelect.value = wantMode;
      els.circleParams.classList.toggle("hidden", !circleRunning);
      els.shuttleParams.classList.toggle("hidden", !shuttleRunning);
      els.sequenceParams.classList.toggle("hidden", !sequenceRunning);
    }
  } else {
    els.trajStatus.classList.add("hidden");
  }
  // 開始ボタンは他方の軌道実行中も無効(軌道は同時に1つ)
  els.btnCircleStart.disabled = !(wsOpen && uiMode === "position" && !trajRunning);
  els.btnCircleStop.disabled = !circleRunning;
  els.btnShuttleStart.disabled = !(wsOpen && uiMode === "position" && !trajRunning);
  els.btnShuttleStop.disabled = !shuttleRunning;
  els.btnTrajSeqStart.disabled = !(wsOpen && uiMode === "position" && !trajRunning);
  els.btnTrajSeqStop.disabled = !sequenceRunning;
  updateSeqProgress(sequenceRunning ? traj : null);

  // ヨー基準ソース(磁気オートチューンパネル)。サーバが正、操作直後は抑制
  const yr = s.yaw_ref;
  if (yr) {
    if (now() - yawRefSentAt > UI.ECHO_SUPPRESS_MS &&
        els.yawRefSelect.value !== yr.source) {
      els.yawRefSelect.value = yr.source;
    }
    els.yawRefMotionInfo.textContent = (typeof yr.motion_yaw_deg === "number")
      ? `motion: ${yr.motion_yaw_deg.toFixed(1)}° ` +
        `J=${typeof yr.motion_J === "number" ? yr.motion_J.toFixed(1) : "--"}` +
        (yr.motion_valid ? "" : "(無効)")
      : `motion: --(励振不足)`;
  }

  // EKF2 状態表示(TLM_STATE 184B 拡張。未対応ファームでは undefined → "--")
  const d = lastDrone;
  if (d && typeof d.ekf2_yaw === "number") {
    els.ekf2YawInfo.textContent =
      `${d.ekf2_yaw.toFixed(1)}° / ${fmtNum(d.ekf2_yaw_innov, 2)}°`;
    els.ekf2BmInfo.textContent =
      `(${fmtNum(d.ekf2_bm_x_ut, 1)}, ${fmtNum(d.ekf2_bm_y_ut, 1)})`;
    const st = typeof d.ekf2_status === "number"
      ? `0x${d.ekf2_status.toString(16).padStart(2, "0")}` : "--";
    const gate = typeof d.ekf2_gate === "number"
      ? `0x${d.ekf2_gate.toString(16).padStart(2, "0")}` : "--";
    els.ekf2StatusInfo.textContent =
      `${st} / ${gate}${d.est_mode_ekf2 ? "(アクティブ)" : "(シャドー)"}`;
    // ヨー観測融合の明示(P1-2): fused ビット+再捕捉 bit7+直近 fused率
    const fused = (d.ekf2_status & EKF2_STATUS_FUSED) !== 0;
    const recap = (d.ekf2_status & EKF2_STATUS_RECAPTURE) !== 0;
    ekf2FusedWin.push(fused ? 1 : 0);
    if (ekf2FusedWin.length > EKF2_FUSED_WIN_N) ekf2FusedWin.shift();
    const fusedPct = 100 * ekf2FusedWin.reduce((a, b) => a + b, 0)
      / ekf2FusedWin.length;
    els.ekf2FusedInfo.textContent =
      `${fused ? "融合中" : "停止中"}${recap ? "(再捕捉中 bit7)" : ""}` +
      ` / fused率 ${fusedPct.toFixed(0)}%(直近${(ekf2FusedWin.length / 20).toFixed(0)}s)`;
  } else {
    els.ekf2YawInfo.textContent = "--";
    els.ekf2BmInfo.textContent = "--";
    els.ekf2StatusInfo.textContent = "--";
    els.ekf2FusedInfo.textContent = "--";
    ekf2FusedWin.length = 0;
  }

  // プリフライト・インターロック(P1-2)バッジ(Posture/Position 両タブ)
  renderInterlock(s.yaw_interlock);
}

/* プリフライト・インターロック(P1-2): 離陸ボタン付近のバッジ+強制離陸。
   サーバの session.yaw_interlock(state/blocking/message)が正 —
   ブロック判定は必ずサーバ側 start() ゲートが行う(UI は表示のみ)。 */
function renderInterlock(il) {
  let text = "整列 --";
  let cls = "badge b-dim";
  let msg = "";
  let showForce = false;
  if (il && il.source !== "off") {
    if (il.state === "ok") {
      text = "整列OK";
      cls = "badge b-ok";
    } else if (il.state === "waiting") {
      const hold = typeof il.hold_s === "number" ? il.hold_s.toFixed(1) : "-";
      text = `整列待ち ${hold}/${il.hold_required_s ?? 3}s`;
      cls = "badge b-warn";
    } else if (il.state === "no_fused") {
      text = il.recapture ? "再捕捉中" : "融合なし";
      cls = "badge b-warn";
    } else {   // no_telemetry
      text = "TLM未受信";
      cls = "badge b-dim";
    }
    if (il.blocking) {
      msg = `${il.message || ""}(EKF2制御: 離陸ブロック中)`;
      showForce = true;
    } else if (il.state === "no_telemetry") {
      msg = il.message || "";
    } else if (il.state !== "ok") {
      msg = `${il.message || ""}(シャドー: 警告のみ・離陸可)`;
    }
  }
  for (const el of els.interlockBadges) {
    el.textContent = text;
    el.className = cls;
  }
  for (const el of els.interlockMsgs) el.textContent = msg;
  for (const btn of els.forceStartBtns) btn.classList.toggle("hidden", !showForce);
}

function renderDrone() {
  const d = lastDrone;
  const state = d ? pick(d, "state") : null;

  // 飛行状態バッジ
  if (typeof state === "number" && FLIGHT_STATES[state]) {
    els.stateBadge.textContent = `${FLIGHT_STATES[state].name} — ${FLIGHT_STATES[state].jp}`;
    els.stateBadge.dataset.state = String(state);
  } else {
    els.stateBadge.textContent = "---";
    els.stateBadge.dataset.state = "-1";
  }

  // Re-arm(RESET)は COMPLETE のときのみ表示
  els.btnRearm.classList.toggle("hidden", state !== STATE_COMPLETE);

  // 姿勢(契約: TLM_STATE全フィールド・角度はdeg換算)
  const roll = pick(d, "roll", "roll_deg");
  const pitch = pick(d, "pitch", "pitch_deg");
  // Yaw は機体が制御に使うソースへ追従する: EKF 有効(est_mode=1)かつ健全なら
  // EKF ヨー(yaw_est)、それ以外は Madgwick(ファーム yaw_used と同じ選択規範。
  // 健全判定は EKF 健全性バッジと同一: anchor_valid && mag_fresh)
  const ekfYaw = pick(d, "yaw_est");
  const yawFromEkf = !!(d && d.est_mode_ekf && d.anchor_valid && d.mag_fresh
                        && typeof ekfYaw === "number");
  const yaw = yawFromEkf ? ekfYaw : pick(d, "yaw", "yaw_deg");
  setBipolarBar(els.attRollBar, roll, UI.ATT_BAR_RANGE_DEG);
  setBipolarBar(els.attPitchBar, pitch, UI.ATT_BAR_RANGE_DEG);
  setBipolarBar(els.attYawBar, yaw, UI.YAW_BAR_RANGE_DEG);
  els.attRollNum.textContent = fmtNum(roll, 1);
  els.attPitchNum.textContent = fmtNum(pitch, 1);
  els.attYawNum.textContent = fmtNum(yaw, 1);
  els.attYawName.textContent = yawFromEkf ? "Yaw(EKF)" : "Yaw";
  els.attYawName.title = yawFromEkf
    ? "EKF 有効・健全のため EKF ヨーを表示中(機体の制御ヨーと同じ選択)"
    : "Madgwick ヨーを表示中(EKF 無効または健全性低下時のフォールバック)";

  // 高度: 現在(カルマン推定) vs 目標
  const altEst = pick(d, "altitude_est");
  const altRef = pick(d, "alt_ref");
  els.altCurBar.style.width = `${pctOf(altEst, UI.ALT_BAR_MAX_M)}%`;
  els.altRefMarker.style.left = `${pctOf(altRef, UI.ALT_BAR_MAX_M)}%`;
  els.altRefMarker.style.display = typeof altRef === "number" ? "" : "none";
  els.altCurNum.textContent = fmtNum(altEst, 2);
  els.altRefNum.textContent = fmtNum(altRef, 2);

  // モータデューティ(0–1)
  for (const k of ["fr", "fl", "rr", "rl"]) {
    const duty = pick(d, `duty_${k}`);
    els.dutyBars[k].style.width = `${pctOf(duty, 1)}%`;
    els.dutyNums[k].textContent = typeof duty === "number" ? `${Math.round(duty * 100)}%` : "--%";
  }
}

/* v2: ヨー推定モニタ(Madgwick / EKF / ジャイロ積算 / MoCap)+EKF健全性バッジ */
function renderYawMonitor() {
  const d = lastDrone;
  els.yawMadgwick.textContent = fmtNum(pick(d, "yaw"), 1);
  els.yawEkf.textContent = fmtNum(pick(d, "yaw_est"), 1);
  // ジャイロ積算は無制限の連続角(ドリフト評価用)。表示は ±180° に折り返し、
  // 一周を超えたぶんは回転数として併記する(値そのものは失わない)
  const gyroInt = pick(d, "yaw_gyro_int");
  if (typeof gyroInt === "number") {
    const turns = Math.round((gyroInt - wrap180(gyroInt)) / 360);
    els.yawGyroInt.textContent =
      wrap180(gyroInt).toFixed(1) + (turns !== 0 ? `(${turns > 0 ? "+" : ""}${turns}周)` : "");
  } else {
    els.yawGyroInt.textContent = fmtNum(gyroInt, 1);
  }

  // MoCap ヨーは Position タブのみ表示(契約 §3.6)。
  // 正解Yaw(yaw_true_deg: 符号/オフセット/フリップ補正済み)を優先し、
  // 旧サーバ互換で yaw_deg(オイラー分解・Y-up では機首方位でない)へ
  // フォールバックする
  const mocapYaw = lastMocap
    ? ((typeof lastMocap.yaw_true_deg === "number") ? lastMocap.yaw_true_deg
       : (typeof lastMocap.yaw_deg === "number") ? lastMocap.yaw_deg : null)
    : null;
  const showMocap = uiMode === "position" && mocapYaw !== null;
  els.yawMocapLabel.classList.toggle("hidden", !showMocap);
  els.yawMocap.classList.toggle("hidden", !showMocap);
  if (showMocap) els.yawMocap.textContent = mocapYaw.toFixed(1);

  // 適用中ヨー目標(機体エコー)。ヨー制御 OFF 時は "--"
  const yawCtrlOn = !!(lastSession && lastSession.yaw_ctrl_on);
  const yawCtrlActive = !!(d && d.yaw_ctrl_active);
  els.yawRefMon.textContent = (yawCtrlOn || yawCtrlActive)
    ? fmtNum(pick(d, "yaw_ref"), 1) : "--.-";

  // EKF 診断値
  const nis = pick(d, "nis");
  els.nisMon.textContent = typeof nis === "number" ? nis.toFixed(2) : "--";
  const ffg = pick(d, "ffg");
  els.ffgMon.textContent = typeof ffg === "number"
    ? `0x${ffg.toString(16).padStart(2, "0")}` : "--";
  const cur = pick(d, "current_a");
  els.currentMon.textContent = typeof cur === "number" ? `${cur.toFixed(2)} A` : "-- A";

  if (d && typeof d.ff_status === "number") {
    const ffName = FF_MODE_NAMES[d.ff_mode] !== undefined
      ? FF_MODE_NAMES[d.ff_mode] : String(d.ff_mode);
    const flagsTxt = [
      d.anchor_valid ? "" : "⚠anchor",
      d.mag_fresh ? "" : "⚠mag",
      d.ffcal_loaded ? "" : "FF係数なし",
    ].filter(Boolean).join(" ");
    const estTxt = d.est_mode_ekf2 ? "EKF2"
      : d.est_mode_ekf ? "EKF" : "相補CF";
    els.ffModeMon.textContent =
      `ff=${ffName} / ${estTxt}${flagsTxt ? " " + flagsTxt : ""}`;
  } else {
    els.ffModeMon.textContent = "--";
  }

  // EKF 健全性バッジ(ffg/ff_status からの簡易判定)
  if (!d) {
    els.ekfBadge.classList.add("hidden");
  } else if (d.est_mode_ekf || d.est_mode_ekf2) {
    const name = d.est_mode_ekf2 ? "EKF2" : "EKF";
    const healthy = !!d.anchor_valid && !!d.mag_fresh;
    els.ekfBadge.textContent = healthy ? `${name} OK` : `${name}注意`;
    els.ekfBadge.className = `badge ${healthy ? "b-ok" : "b-warn"}`;
    els.ekfBadge.title = healthy ? ""
      : `EKF 健全性低下: ${d.anchor_valid ? "" : "アンカー無効 "}` +
        `${d.mag_fresh ? "" : "磁気更新停滞"}(機体はレートダンピングに縮退します)`;
  } else {
    els.ekfBadge.textContent = "相補CF";
    els.ekfBadge.className = "badge b-dim";
    els.ekfBadge.title = "";
  }

  // ff_mode=0 のままヨー角制御 ON の警告(契約 §3.2。飛行は可能)
  const warnFf = yawCtrlOn && d && d.ff_mode === 0;
  els.ffWarnBadge.classList.toggle("hidden", !warnFf);
}

function renderMocap() {
  const m = lastMocap;
  if (m) {
    // fresh = 生フレームの受信鮮度、valid = フィルタ/トラッキングの有効性。
    // 受信していてもデータ無効(トラッキング喪失・外れ値)は警告色で区別する
    // (無効中は位置表示が凍結し、XY 閉ループは水平固定になっている)
    const valid = m.valid !== false;
    setLinkInd(els.mocapStatus, !!m.fresh, !valid);
    els.mocapStatusText.textContent =
      m.fresh ? (valid ? "受信中" : "受信中(位置無効)") : "途絶";
    els.mocapStatus.title = (m.fresh && !valid)
      ? "MoCapフレームは届いていますが位置データが無効です(トラッキング喪失/外れ値)。位置表示は最後の有効値で凍結し、XY制御は水平固定になります"
      : "";
    const conf = typeof m.confidence === "number" ? ` conf: ${m.confidence.toFixed(2)}` : "";
    els.mocapCoords.textContent =
      `x: ${fmtNum(m.x, 2)} y: ${fmtNum(m.y, 2)} z: ${fmtNum(m.z, 2)} yaw: ${fmtNum(m.yaw_deg, 1)}°${conf}`;
    if (typeof m.x === "number" && typeof m.y === "number") {
      trail.push({ x: m.x, y: m.y });
      if (trail.length > UI.TRAIL_MAX_POINTS) trail.shift();
    }
  } else {
    setLinkInd(els.mocapStatus, false);
    els.mocapStatusText.textContent = "未受信";
    els.mocapCoords.textContent = "x: --.-- y: --.-- z: --.--";
  }
}

/* ===================== バー描画ヘルパ ===================== */
function pctOf(v, max) {
  return typeof v === "number" ? clamp(v / max, 0, 1) * 100 : 0;
}

/* 双極バー: 中央0で左右に伸びる */
function setBipolarBar(el, value, range) {
  if (typeof value !== "number") {
    el.style.left = "50%";
    el.style.width = "0%";
    return;
  }
  const half = clamp(value / range, -1, 1) * 50; // -50..+50 [%]
  if (half >= 0) {
    el.style.left = "50%";
    el.style.width = `${half}%`;
  } else {
    el.style.left = `${50 + half}%`;
    el.style.width = `${-half}%`;
  }
}

function fmtNum(v, digits) {
  return typeof v === "number" ? v.toFixed(digits) : "--." + "-".repeat(Math.max(digits, 1));
}
/* 連続角[deg]を (-180, 180] へ正規化(モジュロベース。無制限入力にも安全) */
function wrap180(deg) {
  const w = ((deg % 360) + 540) % 360 - 180;
  return w === -180 ? 180 : w;
}
function fmtDeg(v) {
  return typeof v === "number" ? `${v >= 0 ? "+" : ""}${v.toFixed(1)}°` : "--.-°";
}

/* ===================== XYプロット ===================== */
const plotCtx = els.xyCanvas.getContext("2d");
(function setupCanvasDpr() {
  const dpr = window.devicePixelRatio || 1;
  const w = els.xyCanvas.width, h = els.xyCanvas.height;
  els.xyCanvas.style.width = `${w}px`;
  els.xyCanvas.style.height = `${h}px`;
  els.xyCanvas.width = w * dpr;
  els.xyCanvas.height = h * dpr;
  plotCtx.scale(dpr, dpr);
})();

function plotToPx(x, y) {
  /* ワールド座標[m] → キャンバス座標。+X右 / +Y上 */
  const w = parseFloat(els.xyCanvas.style.width);
  const h = parseFloat(els.xyCanvas.style.height);
  const sx = w / (2 * UI.PLOT_RANGE_M);
  const sy = h / (2 * UI.PLOT_RANGE_M);
  return [w / 2 + x * sx, h / 2 - y * sy];
}

function drawPlot() {
  const w = parseFloat(els.xyCanvas.style.width);
  const h = parseFloat(els.xyCanvas.style.height);
  const css = getComputedStyle(document.documentElement);
  const cGrid = css.getPropertyValue("--plot-grid").trim() || "#2a3140";
  const cAxis = css.getPropertyValue("--plot-axis").trim() || "#3d4860";
  const cTrail = css.getPropertyValue("--plot-trail").trim() || "#3b82f6";
  const cCur = css.getPropertyValue("--plot-current").trim() || "#60a5fa";
  const cTarget = css.getPropertyValue("--plot-target").trim() || "#f59e0b";
  const cStale = css.getPropertyValue("--plot-stale").trim() || "#6b7280";
  const cCircle = css.getPropertyValue("--plot-circle").trim() || "#a78bfa";

  plotCtx.clearRect(0, 0, w, h);

  // グリッド
  plotCtx.lineWidth = 1;
  plotCtx.strokeStyle = cGrid;
  for (let g = -UI.PLOT_RANGE_M; g <= UI.PLOT_RANGE_M + 1e-9; g += UI.PLOT_GRID_M) {
    const [gx] = plotToPx(g, 0);
    const [, gy] = plotToPx(0, g);
    plotCtx.beginPath(); plotCtx.moveTo(gx, 0); plotCtx.lineTo(gx, h); plotCtx.stroke();
    plotCtx.beginPath(); plotCtx.moveTo(0, gy); plotCtx.lineTo(w, gy); plotCtx.stroke();
  }
  // 原点軸
  plotCtx.strokeStyle = cAxis;
  const [ox, oy] = plotToPx(0, 0);
  plotCtx.beginPath(); plotCtx.moveTo(ox, 0); plotCtx.lineTo(ox, h); plotCtx.stroke();
  plotCtx.beginPath(); plotCtx.moveTo(0, oy); plotCtx.lineTo(w, oy); plotCtx.stroke();

  // v2: 目標円軌道の重畳描画(契約 §3.6。サーバ側 trajectory が正)
  const traj = lastSession && lastSession.trajectory;
  if (traj && traj.mode === "circle" &&
      typeof traj.center_x === "number" && typeof traj.radius_m === "number") {
    const [ccx, ccy] = plotToPx(traj.center_x, traj.center_y);
    const rPx = traj.radius_m * (w / (2 * UI.PLOT_RANGE_M));
    plotCtx.strokeStyle = cCircle;
    plotCtx.lineWidth = 1.5;
    plotCtx.setLineDash([5, 4]);
    plotCtx.beginPath(); plotCtx.arc(ccx, ccy, rPx, 0, Math.PI * 2); plotCtx.stroke();
    plotCtx.setLineDash([]);
    // 中心マーカー
    plotCtx.beginPath(); plotCtx.moveTo(ccx - 4, ccy); plotCtx.lineTo(ccx + 4, ccy); plotCtx.stroke();
    plotCtx.beginPath(); plotCtx.moveTo(ccx, ccy - 4); plotCtx.lineTo(ccx, ccy + 4); plotCtx.stroke();
    // 現在位相の点
    if (typeof traj.phase_rad === "number") {
      const px = traj.center_x + traj.radius_m * Math.cos(traj.phase_rad);
      const py = traj.center_y + traj.radius_m * Math.sin(traj.phase_rad);
      const [ppx, ppy] = plotToPx(px, py);
      plotCtx.fillStyle = cCircle;
      plotCtx.beginPath(); plotCtx.arc(ppx, ppy, 3.5, 0, Math.PI * 2); plotCtx.fill();
    }
  }
  // v2: 目標シャトル軌道(直線往復)の重畳描画(円軌道と同パターン)
  if (traj && traj.mode === "shuttle" &&
      typeof traj.center_x === "number" && typeof traj.amplitude_m === "number" &&
      typeof traj.axis_deg === "number") {
    const th = traj.axis_deg * Math.PI / 180;
    const exA = Math.cos(th) * traj.amplitude_m;
    const eyA = Math.sin(th) * traj.amplitude_m;
    const [e1x, e1y] = plotToPx(traj.center_x + exA, traj.center_y + eyA);
    const [e2x, e2y] = plotToPx(traj.center_x - exA, traj.center_y - eyA);
    plotCtx.strokeStyle = cCircle;
    plotCtx.lineWidth = 1.5;
    plotCtx.setLineDash([5, 4]);
    plotCtx.beginPath(); plotCtx.moveTo(e1x, e1y); plotCtx.lineTo(e2x, e2y); plotCtx.stroke();
    plotCtx.setLineDash([]);
    // 中心マーカー
    const [scx, scy] = plotToPx(traj.center_x, traj.center_y);
    plotCtx.beginPath(); plotCtx.moveTo(scx - 4, scy); plotCtx.lineTo(scx + 4, scy); plotCtx.stroke();
    plotCtx.beginPath(); plotCtx.moveTo(scx, scy - 4); plotCtx.lineTo(scx, scy + 4); plotCtx.stroke();
    // 現在位相の点(target = center + A·sin(phase)·e)
    if (typeof traj.phase_rad === "number") {
      const sPh = Math.sin(traj.phase_rad);
      const [ppx, ppy] = plotToPx(traj.center_x + exA * sPh,
                                  traj.center_y + eyA * sPh);
      plotCtx.fillStyle = cCircle;
      plotCtx.beginPath(); plotCtx.arc(ppx, ppy, 3.5, 0, Math.PI * 2); plotCtx.fill();
    }
  }

  // 軌跡
  if (trail.length > 1) {
    plotCtx.strokeStyle = cTrail;
    plotCtx.globalAlpha = 0.45;
    plotCtx.lineWidth = 1.5;
    plotCtx.beginPath();
    trail.forEach((p, i) => {
      const [px, py] = plotToPx(p.x, p.y);
      if (i === 0) plotCtx.moveTo(px, py); else plotCtx.lineTo(px, py);
    });
    plotCtx.stroke();
    plotCtx.globalAlpha = 1;
  }

  // 目標位置(サーバ側で保持している target を正とする。円軌道中は移動目標)
  const t = lastSession && lastSession.target;
  if (t && typeof t.x === "number" && typeof t.y === "number") {
    const [tx, ty] = plotToPx(t.x, t.y);
    plotCtx.strokeStyle = cTarget;
    plotCtx.lineWidth = 1.5;
    const r = 7;
    plotCtx.beginPath(); plotCtx.moveTo(tx - r, ty); plotCtx.lineTo(tx + r, ty); plotCtx.stroke();
    plotCtx.beginPath(); plotCtx.moveTo(tx, ty - r); plotCtx.lineTo(tx, ty + r); plotCtx.stroke();
    plotCtx.beginPath(); plotCtx.arc(tx, ty, r - 2.5, 0, Math.PI * 2); plotCtx.stroke();
  }

  // 現在位置
  const m = lastMocap;
  if (m && typeof m.x === "number" && typeof m.y === "number") {
    const [cx, cy] = plotToPx(m.x, m.y);
    plotCtx.fillStyle = m.fresh ? cCur : cStale;
    plotCtx.beginPath(); plotCtx.arc(cx, cy, 5, 0, Math.PI * 2); plotCtx.fill();
  }
}

/* ===================== 複数機(Multi)タブ ===================== */
/* サーバ側 session.multi(20Hz WS)が正。node_id 順の色は CSS 変数
   --multi-c0..c3(fallback は下記配列)で機体タグ・プロット共通。 */
const MULTI_COLOR_FALLBACK = ["#3b82f6", "#10b981", "#f59e0b", "#ef4444"];
const multiTrails = new Map();     // name -> [{x,y}]
let multiTargetNames = [];         // 目標入力行を構築済みの機体名リスト
let rbPollTimer = null;            // リジッドボディ確認のポーリングタイマ
const multiYawWidgets = new Map(); // name -> {cb, input}(エコー同期用)
const multiYawSentAt = new Map();  // name -> ユーザー操作の送信時刻(エコー抑制)

const multiCtx = els.multiCanvas.getContext("2d");
(function setupMultiCanvasDpr() {
  const dpr = window.devicePixelRatio || 1;
  const w = els.multiCanvas.width, h = els.multiCanvas.height;
  els.multiCanvas.style.width = `${w}px`;
  els.multiCanvas.style.height = `${h}px`;
  els.multiCanvas.width = w * dpr;
  els.multiCanvas.height = h * dpr;
  multiCtx.scale(dpr, dpr);
})();

function multiColor(i) {
  const css = getComputedStyle(document.documentElement)
    .getPropertyValue(`--multi-c${i % 4}`).trim();
  return css || MULTI_COLOR_FALLBACK[i % MULTI_COLOR_FALLBACK.length];
}

function multiPlotToPx(x, y) {
  const w = parseFloat(els.multiCanvas.style.width);
  const h = parseFloat(els.multiCanvas.style.height);
  const s = w / (2 * UI.PLOT_RANGE_M);
  return [w / 2 + x * s, h / 2 - y * (h / (2 * UI.PLOT_RANGE_M))];
}

/* 機体選択チェックボックス一覧(MAC 設定済みプロファイルのみ) */
function renderMultiAirframeList() {
  const box = els.multiAirframeList;
  const checked = new Set(
    [...box.querySelectorAll("input:checked")].map((c) => c.value));
  box.innerHTML = "";
  for (const af of airframes) {
    if (!(af.mac || "").trim()) continue;
    const label = document.createElement("label");
    label.className = "multi-af";
    const cb = document.createElement("input");
    cb.type = "checkbox";
    cb.value = af.name;
    cb.checked = checked.has(af.name);
    const rb = af.rigid_body_id ? `RB${af.rigid_body_id}` : "RB未設定";
    const text = document.createElement("span");
    text.textContent = `${af.name}(ch${af.wifi_channel} / ${rb})`;
    text.classList.toggle("warn-text", !af.rigid_body_id);
    label.append(cb, text);
    box.appendChild(label);
  }
}

function sendMultiSelect() {
  const names = [...els.multiAirframeList.querySelectorAll("input:checked")]
    .map((c) => c.value);
  if (names.length < 2 || names.length > 4) {
    els.multiSelectMsg.textContent = "2〜4機を選択してください";
    return;
  }
  multiTrails.clear();
  els.multiSelectMsg.textContent = `選択を送信しました: ${names.join(", ")}`;
  sendCommand("multi_select", { names });
  appendConsole("ui", `複数機選択を送信: ${names.join(", ")}`);
}

/* 機体別目標入力行(選択機体が変わったときだけ再構築) */
function buildMultiTargetRows(drones) {
  const names = drones.map((d) => d.name);
  if (names.join("|") === multiTargetNames.join("|")) return;
  multiTargetNames = names;
  multiYawWidgets.clear();
  // 選択から外れた機体の軌跡を掃除する
  for (const key of [...multiTrails.keys()]) {
    if (!names.includes(key)) multiTrails.delete(key);
  }
  const box = els.multiTargets;
  box.innerHTML = "";
  if (!names.length) return;
  const head = document.createElement("div");
  head.className = "multi-head";
  const title = document.createElement("span");
  title.className = "mlabel";
  title.textContent = "機体別目標位置 [m]";
  head.appendChild(title);
  box.appendChild(head);

  drones.forEach((d, i) => {
    const row = document.createElement("div");
    row.className = "multi-target-row";
    const tag = document.createElement("span");
    tag.className = "multi-tag mono";
    tag.textContent = d.name;
    tag.style.borderColor = multiColor(i);
    tag.style.color = multiColor(i);
    row.appendChild(tag);

    const inputs = {};
    for (const [key, init] of [["x", "0.00"], ["y", "0.00"], ["z", "0.30"]]) {
      const label = document.createElement("label");
      label.textContent = key.toUpperCase();
      const input = document.createElement("input");
      input.type = "number";
      input.step = "0.05";
      input.value = init;
      inputs[key] = input;
      label.appendChild(input);
      row.appendChild(label);
    }
    const btn = document.createElement("button");
    btn.type = "button";
    btn.className = "btn btn-small";
    btn.textContent = "設定";
    btn.addEventListener("click", () => {
      const x = parseFloat(inputs.x.value);
      const y = parseFloat(inputs.y.value);
      const z = clamp(parseFloat(inputs.z.value) || UI.ALT_MIN_M,
                      UI.ALT_MIN_M, UI.ALT_MAX_M);
      if ([x, y].some(Number.isNaN)) return;
      wsSend({ type: "multi_target", name: d.name, x, y, z });
      appendConsole("ui",
        `目標設定(${d.name}): (${x.toFixed(2)}, ${y.toFixed(2)}, ${z.toFixed(2)})`);
    });
    row.appendChild(btn);
    box.appendChild(row);

    // 機体別サブ行: ヨー角制御 ON/OFF+目標、FF プロファイル適用
    const sub = document.createElement("div");
    sub.className = "multi-target-row multi-sub-row";

    const yawLabel = document.createElement("label");
    yawLabel.className = "switch-label";
    const yawCb = document.createElement("input");
    yawCb.type = "checkbox";
    yawCb.checked = !!d.yaw_ctrl_on;   // サーバ状態からシード(再読込対策)
    yawLabel.append(yawCb, document.createTextNode("ヨー制御"));
    const yawInputLabel = document.createElement("label");
    yawInputLabel.textContent = "ヨー°";
    const yawInput = document.createElement("input");
    yawInput.type = "number";
    yawInput.step = "1";
    yawInput.min = String(-UI.MULTI_YAW_LIMIT_DEG);
    yawInput.max = String(UI.MULTI_YAW_LIMIT_DEG);
    yawInput.value = typeof d.yaw_target_deg === "number"
      ? String(Math.round(d.yaw_target_deg)) : "0";
    yawInputLabel.appendChild(yawInput);
    const sendYaw = () => {
      const deg = clamp(parseFloat(yawInput.value) || 0,
                        -UI.MULTI_YAW_LIMIT_DEG, UI.MULTI_YAW_LIMIT_DEG);
      yawInput.value = String(deg);   // 実際に送る値を表示に反映
      multiYawSentAt.set(d.name, now());
      sendCommand("multi_yaw",
                  { name: d.name, enabled: yawCb.checked, yaw_deg: deg });
      appendConsole("ui", `ヨー設定(${d.name}): `
        + `${yawCb.checked ? "ON" : "OFF"} 目標 ${deg.toFixed(0)}°`);
    };
    yawCb.addEventListener("change", sendYaw);
    yawInput.addEventListener("change", () => {
      if (yawCb.checked) sendYaw();   // OFF 中は目標だけ書き換えても送らない
    });
    multiYawWidgets.set(d.name, { cb: yawCb, input: yawInput });
    sub.append(yawLabel, yawInputLabel);

    const ffSel = document.createElement("select");
    ffSel.className = "multi-ff-select";
    ffSel.dataset.drone = d.name;
    const ffBtn = document.createElement("button");
    ffBtn.type = "button";
    ffBtn.className = "btn btn-small";
    ffBtn.textContent = "FF適用";
    ffBtn.addEventListener("click", () => withBusy(ffBtn, () =>
      doFfApply(ffSel.value, undefined, undefined, d.name)));
    const ffStatusEl = document.createElement("span");
    ffStatusEl.className = "multi-ff-status mono";
    ffStatusEl.dataset.mac = d.mac;
    ffStatusEl.textContent = "FF: --";
    sub.append(ffSel, ffBtn, ffStatusEl);
    box.appendChild(sub);
  });
  updateMultiFfSelects();
}

/* 機体別 FF セレクタの選択肢を /api/ffprofile の一覧に追従させる。
   一覧が不変なら再構築しない(5秒ポーリングごとに開いているドロップ
   ダウンを壊さないため。新規行は optsSig 未設定なので必ず初回構築される) */
function updateMultiFfSelects() {
  const profiles = (ffStatus && ffStatus.profiles) || [];
  const opts = profiles.map((p) => ({
    value: p.name,
    label: p.error ? `${p.name}(読込不可)` : p.name,
    title: p.memo || "",
  }));
  const sig = JSON.stringify(opts);
  for (const sel of els.multiTargets.querySelectorAll(".multi-ff-select")) {
    if (sel.dataset.optsSig === sig) continue;
    sel.dataset.optsSig = sig;
    rebuildSelect(sel, opts);
  }
}

/* 機体別 FF 適用状態のテキスト(PC側記録 applied_by_mac + 機体側 TLM) */
function multiFfStatusText(d) {
  const byMac = (ffStatus && ffStatus.applied_by_mac) || {};
  const ap = byMac[d.mac];
  const t = d.tlm;
  const fw = t
    ? ` 機体:ff=${ffModeLabel(t.ff_mode)}/${t.est_mode_ekf ? "EKF" : "CF"}`
      + (t.ffcal_loaded ? "" : "(FF係数なし)")
      + (t.yaw_ctrl_active ? " ヨー制御中" : "")
    : "";
  return ap
    ? `FF: ${ap.name}(ff=${ffModeLabel(ap.ff)}, `
      + `est=${ap.est === 1 ? "EKF" : "CF"})${fw}`
    : `FF: 未適用${fw}`;
}

/* 機体別ステータスチップ+一斉スタート活性 */
function renderMulti() {
  const multi = lastSession ? lastSession.multi : null;
  const drones = (multi && multi.drones) || [];
  buildMultiTargetRows(drones);

  const box = els.multiStatus;
  box.innerHTML = "";
  drones.forEach((d, i) => {
    const chip = document.createElement("div");
    chip.className = "multi-chip";
    chip.style.borderLeftColor = multiColor(i);
    const t = d.tlm;
    const m = d.mocap;
    const phaseJp = { idle: "待機", armed: "開始", flying: "飛行中" }[d.phase] || d.phase;
    const volt = t && typeof t.voltage === "number" ? `${t.voltage.toFixed(2)}V` : "--V";
    const stateName = t ? t.state_name : "--";
    const link = t && t.fresh ? "TLM✓" : "TLM✗";
    // RB△ = 受信はあるが位置データ無効(トラッキング喪失/外れ値)。
    // 単機の「受信中(位置無効)」と同じ意味
    const mValid = m && m.valid !== false;
    const mocapTxt = m && m.fresh
      ? `${mValid ? "RB✓" : "RB△無効"} (${fmtNum(m.x, 2)}, ${fmtNum(m.y, 2)}, ${fmtNum(m.z, 2)})`
      : "RB✗";
    const lat = typeof d.latency_ms === "number"
      ? ` ${d.latency_ms.toFixed(0)}ms` : "";
    const yawTxt = d.yaw_ctrl_on
      ? `  ヨー${fmtNum(d.yaw_ref_deg, 0)}°` : "";
    chip.textContent =
      `[${d.node_id}] ${d.name}  ${phaseJp}/${stateName}  ${volt}  ${link}  ${mocapTxt}${lat}${yawTxt}`;
    chip.classList.toggle("chip-warn",
      !(t && t.fresh) || !(m && m.fresh) || !mValid);
    chip.classList.toggle("chip-flying", d.phase === "flying");
    box.appendChild(chip);

    // 軌跡の蓄積(MoCap 座標)
    if (m && typeof m.x === "number" && typeof m.y === "number") {
      let tr = multiTrails.get(d.name);
      if (!tr) { tr = []; multiTrails.set(d.name, tr); }
      tr.push({ x: m.x, y: m.y });
      if (tr.length > UI.TRAIL_MAX_POINTS) tr.shift();
    }
  });

  // 機体別 FF 適用状態(PC側記録+機体側 TLM)を持続行へ反映
  for (const el of els.multiTargets.querySelectorAll(".multi-ff-status")) {
    const d = drones.find((x) => x.mac === el.dataset.mac);
    if (d) el.textContent = multiFfStatusText(d);
  }

  // ヨー制御ウィジェットのサーバエコー同期(ユーザー操作直後は抑制。
  // 入力欄はフォーカス中を避けて生目標値 yaw_target_deg を反映)
  for (const d of drones) {
    const w = multiYawWidgets.get(d.name);
    if (!w) continue;
    if (now() - (multiYawSentAt.get(d.name) ?? -Infinity)
        <= UI.ECHO_SUPPRESS_MS) continue;
    w.cb.checked = !!d.yaw_ctrl_on;
    if (document.activeElement !== w.input
        && typeof d.yaw_target_deg === "number") {
      const v = String(Math.round(d.yaw_target_deg));
      if (w.input.value !== v) w.input.value = v;
    }
  }

  // 一斉スタート: 選択済み+全機 idle+WS 接続時のみ
  const anyActive = drones.some((d) => d.phase !== "idle");
  els.btnMultiStart.disabled =
    !(wsOpen && multi && multi.active && drones.length >= 2 && !anyActive);
  els.btnMultiApply.disabled =
    !(wsOpen && lastSession && lastSession.serial_connected && !anyActive);

  // クイック較正カードの対象機体セレクタ(Multi モード中のみ表示。
  // 選択肢は「選択適用」済みの機体 = サーバ側スロット)。
  // WS 20Hz で呼ばれるため、機体一覧が不変なら再構築しない
  // (開いているドロップダウンと選択値を壊さない。updateMultiFfSelects と同方式)
  els.quickcalDroneRow.classList.toggle("hidden", uiMode !== "multi");
  if (uiMode === "multi") {
    const opts = drones.map((d) => ({ value: d.name, label: d.name }));
    const sig = JSON.stringify(opts);
    if (els.quickcalDrone.dataset.optsSig !== sig) {
      els.quickcalDrone.dataset.optsSig = sig;
      rebuildSelect(els.quickcalDrone, opts);
    }
  }
}

function drawMultiPlot() {
  const w = parseFloat(els.multiCanvas.style.width);
  const h = parseFloat(els.multiCanvas.style.height);
  const css = getComputedStyle(document.documentElement);
  const cGrid = css.getPropertyValue("--plot-grid").trim() || "#2a3140";
  const cAxis = css.getPropertyValue("--plot-axis").trim() || "#3d4860";
  const cStale = css.getPropertyValue("--plot-stale").trim() || "#6b7280";

  multiCtx.clearRect(0, 0, w, h);
  multiCtx.lineWidth = 1;
  multiCtx.strokeStyle = cGrid;
  for (let g = -UI.PLOT_RANGE_M; g <= UI.PLOT_RANGE_M + 1e-9; g += UI.PLOT_GRID_M) {
    const [gx] = multiPlotToPx(g, 0);
    const [, gy] = multiPlotToPx(0, g);
    multiCtx.beginPath(); multiCtx.moveTo(gx, 0); multiCtx.lineTo(gx, h); multiCtx.stroke();
    multiCtx.beginPath(); multiCtx.moveTo(0, gy); multiCtx.lineTo(w, gy); multiCtx.stroke();
  }
  multiCtx.strokeStyle = cAxis;
  const [ox, oy] = multiPlotToPx(0, 0);
  multiCtx.beginPath(); multiCtx.moveTo(ox, 0); multiCtx.lineTo(ox, h); multiCtx.stroke();
  multiCtx.beginPath(); multiCtx.moveTo(0, oy); multiCtx.lineTo(w, oy); multiCtx.stroke();

  const multi = lastSession ? lastSession.multi : null;
  const drones = (multi && multi.drones) || [];
  drones.forEach((d, i) => {
    const color = multiColor(i);
    // 軌跡
    const tr = multiTrails.get(d.name) || [];
    if (tr.length > 1) {
      multiCtx.strokeStyle = color;
      multiCtx.globalAlpha = 0.4;
      multiCtx.lineWidth = 1.5;
      multiCtx.beginPath();
      tr.forEach((p, k) => {
        const [px, py] = multiPlotToPx(p.x, p.y);
        if (k === 0) multiCtx.moveTo(px, py); else multiCtx.lineTo(px, py);
      });
      multiCtx.stroke();
      multiCtx.globalAlpha = 1;
    }
    // 目標(◎十字)
    if (d.target && typeof d.target.x === "number") {
      const [tx, ty] = multiPlotToPx(d.target.x, d.target.y);
      multiCtx.strokeStyle = color;
      multiCtx.lineWidth = 1.5;
      const r = 7;
      multiCtx.beginPath(); multiCtx.moveTo(tx - r, ty); multiCtx.lineTo(tx + r, ty); multiCtx.stroke();
      multiCtx.beginPath(); multiCtx.moveTo(tx, ty - r); multiCtx.lineTo(tx, ty + r); multiCtx.stroke();
      multiCtx.beginPath(); multiCtx.arc(tx, ty, r - 2.5, 0, Math.PI * 2); multiCtx.stroke();
    }
    // 現在位置(●+ノード番号)
    const m = d.mocap;
    if (m && typeof m.x === "number" && typeof m.y === "number") {
      const [cx, cy] = multiPlotToPx(m.x, m.y);
      multiCtx.fillStyle = m.fresh ? color : cStale;
      multiCtx.beginPath(); multiCtx.arc(cx, cy, 5, 0, Math.PI * 2); multiCtx.fill();
      multiCtx.fillStyle = m.fresh ? color : cStale;
      multiCtx.font = "10px sans-serif";
      multiCtx.fillText(String(d.node_id), cx + 7, cy - 7);
    }
  });
}

/* リジッドボディ紐付け確認(500ms ポーリングのトグル) */
function renderRbList(result) {
  const box = els.rbList;
  box.innerHTML = "";
  if (!result || !result.connected) {
    box.textContent = "NatNet 未接続(Motive の配信設定を確認してください)";
    return;
  }
  const bodies = result.bodies || [];
  if (!bodies.length) {
    box.textContent = "リジッドボディ未検出(Motive 側で作成されているか確認)";
    return;
  }
  for (const b of bodies) {
    const line = document.createElement("div");
    line.className = "rb-line";
    const stale = typeof b.age_s === "number" && b.age_s > 1.0;
    line.classList.toggle("stale", stale);
    const assigned = airframes.find((a) => a.rigid_body_id === b.rigid_body_id);
    const tag = assigned ? ` ← ${assigned.name}` : "";
    line.textContent =
      `RB ${b.rigid_body_id}: x${fmtNum(b.x, 2)} y${fmtNum(b.y, 2)} ` +
      `z${fmtNum(b.z, 2)}${stale ? "(途絶)" : ""}${tag}`;
    box.appendChild(line);
  }
}

async function pollRbBodies() {
  renderRbList(await apiGet("/api/mocap/bodies", true));
}

function stopRbCheck() {
  if (rbPollTimer === null) return;
  clearInterval(rbPollTimer);
  rbPollTimer = null;
  els.btnRbCheck.textContent = "確認開始";
}

function toggleRbCheck() {
  if (rbPollTimer !== null) {
    stopRbCheck();
    return;
  }
  els.btnRbCheck.textContent = "確認停止";
  pollRbBodies();
  rbPollTimer = setInterval(pollRbBodies, UI.RB_POLL_MS);
}

/* ===================== 設定タブ: MoCap マッピング =====================
 * UI 専用タブ(サーバ session モードではない): set_mode を送らず、
 * renderSession のモードecho同期からも除外される(syncSettingsVisual)。
 * 編集フォームはサーバ echo に上書きされない(マッピングは本タブの
 * PUT でのみ変わる)ため、エコー抑制は不要。 */

const MAP_AXES = ["x", "y", "z"];
const RAD2DEG = 180 / Math.PI;

function setMapMsg(text, isErr) {
  els.mapMsg.textContent = text;
  els.mapMsg.classList.toggle("msg-err", !!isErr);
}

function mappingFormFill(mapping) {
  const ct = mapping.coordinate_transform || {};
  for (const axis of MAP_AXES) {
    const c = ct[axis] || {};
    if (c.axis) els.mapAxisSel[axis].value = c.axis;
    els.mapSignSel[axis].value = String((c.sign ?? 1) >= 0 ? 1 : -1);
  }
  const at = mapping.attitude_transform || {};
  if (at.forward_axis) els.mapFwdAxis.value = at.forward_axis;
  if (at.up_axis) els.mapUpAxis.value = at.up_axis;
  els.mapYawSign.value = String((at.yaw_sign ?? 1) >= 0 ? 1 : -1);
  els.mapYawOffset.value =
    (typeof at.yaw_offset_deg === "number" ? at.yaw_offset_deg : 0).toFixed(1);
  els.mapFlipCorr.checked = at.flip_correction !== false;
  els.mapFlipGate.value =
    (typeof at.flip_gate === "number" ? at.flip_gate : 0.5).toFixed(2);
}

function mappingFormPayload() {
  const ct = {};
  for (const axis of MAP_AXES) {
    ct[axis] = { axis: els.mapAxisSel[axis].value,
                 sign: parseInt(els.mapSignSel[axis].value, 10) };
  }
  return {
    coordinate_transform: ct,
    attitude_transform: {
      forward_axis: els.mapFwdAxis.value,
      up_axis: els.mapUpAxis.value,
      yaw_sign: parseInt(els.mapYawSign.value, 10),
      yaw_offset_deg: clamp(parseFloat(els.mapYawOffset.value) || 0, -360, 360),
      flip_correction: els.mapFlipCorr.checked,
      flip_gate: clamp(parseFloat(els.mapFlipGate.value) || 0.5, 0.05, 0.95),
    },
  };
}

async function loadMapping(quiet) {
  const res = await apiGet("/api/mocap/mapping", !!quiet);
  if (!res || !res.ok) {
    if (!quiet) setMapMsg("マッピングの取得に失敗しました", true);
    return;
  }
  mappingFormFill(res.mapping || {});
  appliedMapping = res.mapping || null;
  mapPrimaryRbId = res.primary_rigid_body_id ?? null;
  setMapMsg(mappingStatusText(res), !res.can_apply
    || res.machine_frame === "unsupported");
}

/* GET/PUT 応答 → ステータス行(適用可否+機上XY制御への変換状態) */
function mappingStatusText(res) {
  const frameNote = res.machine_frame === "mirrored_y"
    ? " / 機体への位置指令は自動変換されます(右手系→機体フレーム)"
    : res.machine_frame === "unsupported"
      ? " / ⚠ このマッピングは機上XY制御の対応外です(位置制御飛行は無効化されます)"
      : "";
  if (!res.can_apply) return (res.blocked_reason || "現在は適用できません") + frameNote;
  return "適用可能です(地上)" + frameNote;
}

async function applyMapping() {
  setMapMsg("適用中…", false);
  const res = await apiPut("/api/mocap/mapping", mappingFormPayload());
  if (!res) { setMapMsg("サーバとの通信に失敗しました", true); return; }
  if (!res.ok) { setMapMsg(res.message || "適用できませんでした", true); return; }
  mappingFormFill(res.mapping || {});
  appliedMapping = res.mapping || null;
  const frameNote = res.machine_frame === "mirrored_y"
    ? " 機体への位置指令は自動変換されます(右手系→機体フレーム)。"
    : res.machine_frame === "unsupported"
      ? " ⚠ このマッピングは機上XY制御の対応外のため位置制御飛行は無効化されます。"
      : "";
  setMapMsg("適用しました(control.json 保存済み・位置フィルタ再初期化・目標リセット)。"
    + frameNote, res.machine_frame === "unsupported");
  appendConsole("ui", "MoCap マッピングを適用しました");
}

/* 機体表示ヨー(renderDrone と同じ選択規範: EKF健全なら EKF、他は Madgwick) */
function currentDroneYawDeg() {
  const d = lastDrone;
  const ekfYaw = pick(d, "yaw_est");
  const fromEkf = !!(d && d.est_mode_ekf && d.anchor_valid && d.mag_fresh
                     && typeof ekfYaw === "number");
  const yaw = fromEkf ? ekfYaw : pick(d, "yaw", "yaw_deg");
  return (typeof yaw === "number") ? yaw : null;
}

function primaryPreviewBody() {
  const bodies = (mapBodies && mapBodies.bodies) || [];
  // primary RB(単機 Position の rigid_body_id)のみを対象にする。
  // 別ボディへの黙ったフォールバックはしない(隣の機体でゼロ合わせして
  // しまう事故を防ぐ)。primary 未設定時のみ唯一の観測ボディを許す。
  let primary = null;
  if (mapPrimaryRbId !== null) {
    primary = bodies.find((b) => b.rigid_body_id === mapPrimaryRbId) || null;
  } else if (bodies.length === 1) {
    primary = bodies[0];
  }
  if (!primary) return null;
  if (typeof primary.age_s === "number" && primary.age_s > 1.0) return null;
  return primary;
}

/* 編集フォームの「軸」設定(座標変換+前方軸)が適用済みマッピングと一致
 * しているか。heading はサーバが適用済みマッピングで計算するため、軸を
 * 編集したまま未適用だとゼロ合わせの前提が崩れる(符号・オフセットの編集は
 * 計算に折り込むので未適用でよい) */
function mapAxesMatchApplied() {
  if (!appliedMapping) return false;
  const ct = appliedMapping.coordinate_transform || {};
  for (const axis of MAP_AXES) {
    const c = ct[axis] || {};
    if (els.mapAxisSel[axis].value !== c.axis) return false;
    if (parseInt(els.mapSignSel[axis].value, 10) !== ((c.sign ?? 1) >= 0 ? 1 : -1)) {
      return false;
    }
  }
  const at = appliedMapping.attitude_transform || {};
  return els.mapFwdAxis.value === (at.forward_axis || "+x");
}

/* ゼロ合わせ: 現在の生 heading から「正解Yaw = targetDeg」となるオフセットを
 * フォームへ設定する(適用は別途「適用」ボタン)。yaw_true = sign*heading+offset */
async function alignYawOffset(targetDeg) {
  if (!mapAxesMatchApplied()) {
    setMapMsg("軸設定(座標変換・前方軸)が未適用です。先に「適用」してからゼロ合わせしてください", true);
    return;
  }
  // 鮮度を保証するため必ず取り直す(失敗時は古いキャッシュへ落とさない)
  mapBodies = await apiGet("/api/mocap/bodies", true);
  if (mapPollTimer !== null) renderMapPreview(mapBodies);
  const primary = primaryPreviewBody();
  if (!primary || typeof primary.heading_rad !== "number") {
    setMapMsg(mapPrimaryRbId !== null
      ? `単機リジッドボディ(RB ${mapPrimaryRbId})の方位が取得できません(Motive 配信と RB 検出を確認してください)`
      : "リジッドボディの方位が取得できません(Motive 配信と RB 検出を確認してください)", true);
    return;
  }
  const sign = parseInt(els.mapYawSign.value, 10) || 1;
  const offset = wrap180(targetDeg - sign * primary.heading_rad * RAD2DEG);
  els.mapYawOffset.value = offset.toFixed(1);
  setMapMsg(`ヨーオフセットを ${offset.toFixed(1)}° に設定しました(RB ${primary.rigid_body_id} 基準・未適用 — 「適用」で保存されます)`, false);
}

function flipFlagsText(flags) {
  if (!flags) return "なし";
  const parts = [];
  if (flags & 0x01) parts.push("上方軸反転補正");
  if (flags & 0x02) parts.push("ヨー180°補正");
  return parts.join("+");
}

function renderMapPreview(result) {
  const box = els.mapPreviewBox;
  box.innerHTML = "";
  if (!result || !result.connected) {
    box.textContent = "NatNet 未接続(Motive の配信設定を確認してください)";
    return;
  }
  const bodies = result.bodies || [];
  if (!bodies.length) {
    box.textContent = "リジッドボディ未検出(Motive 側で作成されているか確認)";
    return;
  }
  const tlmYaw = currentDroneYawDeg();
  for (const b of bodies) {
    const stale = typeof b.age_s === "number" && b.age_s > 1.0;
    const assigned = airframes.find((a) => a.rigid_body_id === b.rigid_body_id);
    const isPrimary = b.rigid_body_id === mapPrimaryRbId;

    const head = document.createElement("div");
    head.className = "rb-line";
    head.classList.toggle("stale", stale);
    head.textContent =
      `RB ${b.rigid_body_id}${isPrimary ? "(単機)" : ""}` +
      `${assigned ? ` ← ${assigned.name}` : ""}${stale ? "(途絶)" : ""}`;
    if (!isPrimary) {
      // Motive の Streaming ID は作り直しで増える(RB1 削除→再作成で RB2)。
      // 単機対象を観測中のボディへ明示的に付け替えるボタン(黙った
      // フォールバックはしない — 別機体への誤紐付け防止)
      const btn = document.createElement("button");
      btn.className = "btn btn-small rb-primary-btn";
      btn.textContent = "単機対象に設定";
      btn.title = "単機 Position モードの制御・ゼロ合わせの対象をこのリジッドボディに変更します(control.json へ保存。地上でのみ)";
      btn.addEventListener("click", () =>
        withBusy(btn, () => setPrimaryRigidBody(b.rigid_body_id)));
      head.appendChild(btn);
    }
    box.appendChild(head);

    const posLine = document.createElement("div");
    posLine.className = "rb-line rb-sub";
    posLine.classList.toggle("stale", stale);
    const mp = b.motive_pos;
    const motiveText = Array.isArray(mp)
      ? `Motive(${fmtNum(mp[0], 2)}, ${fmtNum(mp[1], 2)}, ${fmtNum(mp[2], 2)})`
      : "Motive(--)";
    posLine.textContent =
      `  ${motiveText} → 制御(x${fmtNum(b.x, 2)} y${fmtNum(b.y, 2)} z${fmtNum(b.z, 2)})`;
    box.appendChild(posLine);

    const yawLine = document.createElement("div");
    yawLine.className = "rb-line rb-sub";
    yawLine.classList.toggle("stale", stale);
    const yawTrue = (typeof b.yaw_true_rad === "number")
      ? (b.yaw_true_rad * RAD2DEG) : null;
    const heading = (typeof b.heading_rad === "number")
      ? (b.heading_rad * RAD2DEG) : null;
    let text = `  正解Yaw ${yawTrue === null ? "--" : yawTrue.toFixed(1)}°` +
      `(heading生値 ${heading === null ? "--" : heading.toFixed(1)}°、` +
      `フリップ補正: ${flipFlagsText(b.flip_flags)})`;
    if (isPrimary && tlmYaw !== null && yawTrue !== null) {
      text += ` / 機体ヨー ${tlmYaw.toFixed(1)}°(差 ${wrap180(yawTrue - tlmYaw).toFixed(1)}°)`;
    }
    yawLine.textContent = text;
    box.appendChild(yawLine);
  }
}

async function pollMapPreview() {
  mapBodies = await apiGet("/api/mocap/bodies", true);
  renderMapPreview(mapBodies);
}

/* 単機対象リジッドボディの付け替え(Motive の ID 増加への運用対応) */
async function setPrimaryRigidBody(rbId) {
  const res = await apiPost("/api/mocap/primary", { rigid_body_id: rbId });
  if (!res) { setMapMsg("サーバとの通信に失敗しました", true); return; }
  if (!res.ok) { setMapMsg(res.message || "変更できませんでした", true); return; }
  mapPrimaryRbId = res.primary_rigid_body_id ?? rbId;
  appliedMapping = res.mapping || appliedMapping;
  setMapMsg(`単機対象を RB ${mapPrimaryRbId} に変更しました(control.json 保存済み)`, false);
  appendConsole("ui", `単機対象リジッドボディ: RB ${mapPrimaryRbId}`);
  if (mapBodies) renderMapPreview(mapBodies);   // (単機)マーカー即時更新
}

function stopMapPreview() {
  if (mapPollTimer === null) return;
  clearInterval(mapPollTimer);
  mapPollTimer = null;
  els.btnMapPreview.textContent = "プレビュー開始";
}

function toggleMapPreview() {
  if (mapPollTimer !== null) {
    stopMapPreview();
    return;
  }
  els.btnMapPreview.textContent = "プレビュー停止";
  pollMapPreview();
  mapPollTimer = setInterval(pollMapPreview, UI.MAP_POLL_MS);
}

/* 設定タブの表示同期: モードecho(applyMode)が4タブを再活性化しても、
 * 設定タブ表示中はモードパネルを非表示に戻す */
function syncSettingsVisual() {
  els.tabSettings.classList.toggle("active", settingsOpen);
  els.panelSettings.classList.toggle("active", settingsOpen);
  if (settingsOpen) {
    for (const el of [els.tabPosture, els.tabPosition, els.tabMulti,
                      els.tabExperiment, els.panelPosture, els.panelPosition,
                      els.panelMulti, els.panelExperiment]) {
      el.classList.remove("active");
    }
  }
}

function openSettings() {
  if (settingsOpen) return;
  settingsOpen = true;
  syncSettingsVisual();
  loadMapping(false);
}

function closeSettings() {
  if (!settingsOpen) return;
  settingsOpen = false;
  stopMapPreview();
  syncSettingsVisual();
}

/* ===================== コンソール ===================== */
const CONSOLE_TAGS = {
  ui: "UI", relay: "RELAY", drone: "DRONE", event: "EVENT",
};

function appendConsole(tag, text) {
  const c = els.consoleEl;
  const nearBottom = c.scrollHeight - c.scrollTop - c.clientHeight < 30;

  const line = document.createElement("div");
  line.className = `line line-${tag}`;
  const t = new Date();
  const hh = String(t.getHours()).padStart(2, "0");
  const mm = String(t.getMinutes()).padStart(2, "0");
  const ss = String(t.getSeconds()).padStart(2, "0");

  const timeEl = document.createElement("span");
  timeEl.className = "time";
  timeEl.textContent = `${hh}:${mm}:${ss}`;
  const tagEl = document.createElement("span");
  tagEl.className = `tag tag-${tag}`;
  tagEl.textContent = CONSOLE_TAGS[tag] || tag;
  const msgEl = document.createElement("span");
  msgEl.className = "msg";
  msgEl.textContent = text;

  line.append(timeEl, tagEl, msgEl);
  c.appendChild(line);
  while (c.childElementCount > UI.CONSOLE_MAX_LINES) c.removeChild(c.firstElementChild);
  if (nearBottom) c.scrollTop = c.scrollHeight;
}

/* ===================== モード(タブ)切替 ===================== */
function applyMode(mode, sendToServer) {
  uiMode = mode;
  const tabs = { posture: els.tabPosture, position: els.tabPosition,
                 multi: els.tabMulti, experiment: els.tabExperiment };
  const panels = { posture: els.panelPosture, position: els.panelPosition,
                   multi: els.panelMulti, experiment: els.panelExperiment };
  for (const m of Object.keys(tabs)) {
    tabs[m].classList.toggle("active", m === mode);
    panels[m].classList.toggle("active", m === mode);
  }
  // Experiment はパネル数が多いため左カラムを広げる
  els.mainEl.classList.toggle("mode-experiment", mode === "experiment");
  // Experiment 表示中は飛行ログトグルを無効化(飛行ログは START〜着陸のみで
  // experiment では記録されない。計測は Experiment タブの計測パネルを使う)
  const logNa = mode === "experiment";
  els.logToggle.disabled = logNa;
  els.logToggle.parentElement.title = logNa
    ? "Experiment モードでは飛行ログは記録されません(計測は Experiment タブの「計測(EKF/FF性能ログ)」を使用)"
    : "";
  // 共通ヨーブロックをアクティブタブへ移設(単一実体・二重配線なし)
  if (mode === "position") {
    els.yawSlotPosition.appendChild(els.yawBlock);
  } else if (mode === "posture") {
    els.yawSlotPosture.appendChild(els.yawBlock);
  }
  if (sendToServer) {
    modeSentAt = now();
    sendCommand("set_mode", { mode });
  }
  // クイック較正カードの対象機体セレクタは Multi モード中のみ表示
  els.quickcalDroneRow.classList.toggle("hidden", mode !== "multi");
  if (mode === "position") drawPlot();
  if (mode === "multi") {
    renderMultiAirframeList();
    drawMultiPlot();
  } else {
    stopRbCheck();   // タブ離脱時に RB 確認ポーリングを止める
  }
  if (mode === "experiment") {
    refreshExperimentPanels();
    rtmonRequestDraw();   // リアルタイムモニタの初期描画(履歴は保持済み)
  }
  // 設定タブ表示中はモードechoでパネルを奪わない(UI専用タブ)
  syncSettingsVisual();
}

/* ===================== STOP(緊急停止) ===================== */
function doStop() {
  sendCommand("stop");
  if (uiMode === "experiment") {
    // 契約 §3.6: Experiment 中は CMD_MOTOR_STOP も送出する
    sendCommand("motor_stop");
    appendConsole("ui", "STOP+モーター停止 送信(緊急停止)");
  } else {
    appendConsole("ui", "STOP 送信(着陸要求)");
  }
  // 視覚フィードバック: フッタヒントを点滅
  els.spaceHint.classList.remove("flash");
  void els.spaceHint.offsetWidth; // reflowを挟んでアニメーションを再始動
  els.spaceHint.classList.add("flash");
}

/* ===================== v2: Experiment タブ ===================== */

function selectedSweepMask() {
  let mask = 0;
  for (const cb of document.querySelectorAll(".sweep-motor")) {
    if (cb.checked) mask |= 1 << Number(cb.dataset.bit);
  }
  return mask;
}

function selectedSweepPattern() {
  const checked = document.querySelector('input[name="sweepPattern"]:checked');
  return checked ? checked.value : "updown";
}

function sweepNotes() {
  return {
    location: els.sweepLocation.value || "",
    orientation: els.sweepOrientation.value || "",
    memo: els.sweepMemo.value || "",
  };
}

const SWEEP_PHASE_JP = {
  idle: "待機中", starting: "開始中", base: "基準測定(モーター停止)",
  settle: "整定中", measure: "計測中", gap: "OFF基準測定", gap_settle: "OFF整定",
  baseline: "OFF基準測定", done: "完了", aborted: "中断", error: "エラー",
};

/* 20Hzスナップショットの session.experiment から実験パネルを描画する */
function renderExperiment() {
  const exp = lastSession ? lastSession.experiment : null;

  // 有効化バッジ
  const active = !!(exp && exp.active);
  els.expActiveBadge.textContent = active ? "有効(MOTOR_TEST)" : "未有効";
  els.expActiveBadge.className = `badge ${active ? "b-ok" : "b-warn"}`;

  // モーターテスト状態
  const motor = exp && exp.motor;
  if (motor && motor.running) {
    els.motorStatusText.textContent =
      `回転中 duty=${Number(motor.duty).toFixed(2)} (${motor.motors || "-"})`;
    els.motorStatusText.classList.add("running");
  } else {
    els.motorStatusText.textContent = "停止";
    els.motorStatusText.classList.remove("running");
  }

  // 計測(EKF/FF性能ログ)の状態表示(サーバ側 experiment.recording が正)
  const rec = exp && exp.recording;
  if (rec && rec.active) {
    els.expRecStatus.textContent =
      `計測中: ${rec.file || "--"}(${rec.samples ?? 0}サンプル)`;
    els.expRecStatus.classList.add("running");
  } else {
    els.expRecStatus.textContent =
      rec && rec.file ? `停止中(直近: ${rec.file})` : "停止中";
    els.expRecStatus.classList.remove("running");
  }

  // TLM_EXP ライブ表示
  const sample = exp && exp.exp;
  const age = exp ? exp.exp_age_s : null;
  if (sample && typeof age === "number" && age <= UI.EXP_FRESH_S) {
    // 非有限値はサーバ側で null 化される(WS の JSON 保護)ため、
    // 各フィールドは null を "--" 表示に落とす(0.00 と誤認させない)
    const braw = Array.isArray(sample.b_raw)
      ? sample.b_raw.map((v) => fmtNum(v, 1)).join("/") : "--";
    const cur = sample.cv ? `${fmtNum(sample.current_a, 2)}A` : "--A";
    const vbat = sample.cv ? `${fmtNum(sample.vbat_v, 2)}V` : "--V";
    els.expLive.textContent =
      `TLM_EXP: I=${cur} V=${vbat} Braw=[${braw}]µT ` +
      `T=${fmtNum(sample.imu_temp_c, 1)}°C ` +
      `duty=${fmtNum(sample.duty_cmd, 2)}` +
      `${sample.motors_running ? " 回転中" : ""}`;
  } else {
    els.expLive.textContent = "TLM_EXP: なし(実験モード有効時に 25Hz 受信)";
  }

  // 加速度6面キャリブのライブ加速度(expLive と同じ TLM_EXP 鮮度ゲート)
  const accFresh = sample && typeof age === "number" && age <= UI.EXP_FRESH_S;
  const accOk = accFresh && [sample.ax, sample.ay, sample.az]
    .every((v) => typeof v === "number" && Number.isFinite(v));
  if (accOk) {
    els.accel6Accel.textContent =
      `${sample.ax.toFixed(3)} / ${sample.ay.toFixed(3)} / ${sample.az.toFixed(3)}`;
    els.accel6Norm.textContent =
      Math.hypot(sample.ax, sample.ay, sample.az).toFixed(3);
  } else {
    els.accel6Accel.textContent = "--";
    els.accel6Norm.textContent = "--";
  }

  // スイープ
  const sweep = exp && exp.sweep;
  if (sweep) {
    const phaseJp = SWEEP_PHASE_JP[sweep.phase] || sweep.phase || "--";
    const dutyTxt = sweep.running && typeof sweep.duty === "number" && sweep.duty > 0
      ? ` duty=${sweep.duty.toFixed(2)}` : "";
    els.sweepPhase.textContent = `${phaseJp}${dutyTxt}` +
      (sweep.running ? ` / ${sweep.motors || ""} ${(sweep.elapsed_s || 0).toFixed(0)}s` : "");
    els.sweepStepTag.textContent = (sweep.total_steps && sweep.step_index)
      ? `STEP ${sweep.step_index}/${sweep.total_steps}` : "";
    let frac = 0;
    if (sweep.phase === "done") frac = 1;
    else if (sweep.total_steps > 0) frac = clamp(sweep.step_index / sweep.total_steps, 0, 1);
    els.sweepProgressFill.style.width = `${frac * 100}%`;
    els.sweepProgressFill.classList.toggle("err", sweep.phase === "error");
    if (sweep.message) els.sweepMessage.textContent = sweep.message;
    const r = sweep.last_result;
    els.sweepResult.textContent = r
      ? `直近の保存結果: ${r.samples || "--"}(${r.sample_count ?? "--"}点` +
        `${r.aborted ? "・中断" : ""}` +
        `${r.baseline_flag_count ? `・基準ジャンプ${r.baseline_flag_count}件` : ""})`
      : "直近の保存結果: --";
  }

  // シーケンス
  const seq = exp && exp.sequence;
  if (seq) {
    els.seqProgress.textContent = seq.running
      ? `${Math.min((seq.index ?? 0) + 1, seq.total ?? 0)}/${seq.total ?? 0}本目`
      : (seq.phase === "done" ? "完了" : "");
    if (seq.message) els.seqMessage.textContent = seq.message;
    els.seqMeta.textContent = `保存セット: ${seq.last_meta || "--"}`;
    const waiting = seq.phase === "waiting_battery";
    els.btnSeqResume.classList.toggle("hidden", !waiting);
    els.btnSeqForce.classList.toggle("hidden", !waiting);
  }

  // 3D磁気の収集ライブ(サンプル数はスナップショットが最速)
  const cal3d = exp && exp.cal3d;
  if (cal3d) {
    els.cal3dSamples.textContent = String(cal3d.sample_count ?? 0);
    els.cal3dProgressFill.style.width =
      `${clamp((cal3d.sample_count || 0) / UI.CAL3D_TARGET_SAMPLES, 0, 1) * 100}%`;
    if (cal3d.collecting) els.cal3dStatusText.textContent = "収集中(機体を全方位に回す)";
  }

  updateExperimentControls();
}

/* Experiment 操作系の活性/非活性の一括更新 */
function updateExperimentControls() {
  const s = lastSession;
  const exp = s ? s.experiment : null;
  const serial = !!(s && s.serial_connected);
  const active = !!(exp && exp.active);
  const fixture = els.fixtureCheck.checked;
  const sweepRunning = !!(exp && exp.sweep && exp.sweep.running);
  const seqRunning = !!(exp && exp.sequence && exp.sequence.running);
  const busy = sweepRunning || seqRunning;
  const motorRunning = !!(exp && exp.motor && exp.motor.running);
  const recording = !!(exp && exp.recording && exp.recording.active);

  els.btnExpActivate.disabled = !(wsOpen && serial && uiMode === "experiment" && !active);

  // 高出力 duty ボタンの活性(0.6 以上は高出力許可チェック必須)
  const highOk = els.highDutyCheck.checked;
  for (const btn of els.dutyButtons.querySelectorAll(".duty-btn")) {
    const d = parseFloat(btn.dataset.duty);
    btn.disabled = d >= UI.DUTY_HIGH_MIN && !highOk;
    btn.classList.toggle("selected", Math.abs(d - selectedDuty) < 1e-9);
  }

  els.btnMotorStart.disabled = !(wsOpen && active && fixture && !busy);
  els.btnMotorApply.disabled = !(wsOpen && active && motorRunning && !busy);
  // Stop は安全経路のため常時活性(WS 切断時のみ無意味なので無効化)
  els.btnMotorStop.disabled = !wsOpen;

  // 計測(EKF/FF性能ログ): 開始はモーターテスト有効かつスイープ/シーケンス
  // 非実行時のみ。停止はサーバ側が常時受理するため計測中は常に押せる
  els.btnExpRecStart.disabled = !(wsOpen && active && !busy && !recording);
  els.btnExpRecStop.disabled = !(wsOpen && recording);

  // 計測中はスイープ/シーケンス開始を禁止(サーバ側拒否と同じ制限をUIにも)
  els.btnSweepStart.disabled =
    !(wsOpen && active && fixture && !busy && !recording && selectedSweepMask() !== 0);
  els.btnSweepAbort.disabled = !(wsOpen && sweepRunning);
  els.btnSeqStart.disabled = !(wsOpen && active && fixture && !busy && !recording);
  els.btnSeqAbort.disabled = !(wsOpen && seqRunning);

  // 実行中はスイープ条件の変更をロック。
  // 計測中はモーター選択を全モーター固定(CMD_MOTOR_RUN は 0xF のみ受理)
  for (const cb of document.querySelectorAll(".sweep-motor")) {
    if (recording) cb.checked = true;
    cb.disabled = busy || recording;
  }
  for (const rb of document.querySelectorAll('input[name="sweepPattern"]')) rb.disabled = busy;
  for (const inp of [els.sweepLocation, els.sweepOrientation, els.sweepMemo]) {
    inp.disabled = busy;
  }
}

/* ===================== リアルタイムモニタ(Experiment) ===================== */
/* ヨー3系統+MoCap / フロー速度 / フロー積算2D位置を canvas 自前描画する
   (外部ライブラリなし)。サンプリングは WS state(20Hz)ごとに常時行い、
   時刻は機体 elapsed_ms(TLM_STATE クロック)基準 — 同一フレーム再配信は
   dt=0 として捨て、逆行(機体再起動)でバッファをクリアする。
   再描画は requestAnimationFrame + 時刻間引きで 10Hz 上限。タブ非表示
   (document.hidden。rAF も停止する)・Experiment 以外のタブ・折りたたみ中は
   描画しない(サンプリング/積算は継続)。 */
const RTMON = {
  WINDOW_S: 60,            // 表示窓/軌跡保持 [s]
  CAPACITY: 1600,          // リングバッファ容量(60s × 25Hz + 余裕)
  REDRAW_MIN_MS: 100,      // 再描画間隔の下限(=10Hz 上限)
  DT_MAX_S: 0.25,          // 積算/差分 dt の上限(フレーム落ち・再接続保護)
  XY_HALF_MIN_M: 1.0,      // 2D位置プロットの表示半幅の下限(初期 ±1m)
  VEL_HALF_MIN: 0.25,      // 速度グラフ y 半幅の下限 [m/s]
  FLOW_VEL_VALID: 0x04,    // TlmState.FLOW_STATUS_VEL_VALID(契約 §1.1)
};

/* 固定容量リングバッファ(満杯時は最古を上書き) */
class RtRing {
  constructor(capacity) {
    this.cap = capacity;
    this.buf = new Array(capacity);
    this.start = 0;
    this.len = 0;
  }
  push(item) {
    if (this.len < this.cap) {
      this.buf[(this.start + this.len) % this.cap] = item;
      this.len++;
    } else {
      this.buf[this.start] = item;
      this.start = (this.start + 1) % this.cap;
    }
  }
  get(i) { return this.buf[(this.start + i) % this.cap]; }
  last() { return this.len ? this.get(this.len - 1) : null; }
  clear() { this.start = 0; this.len = 0; }
}

const rtmonRing = new RtRing(RTMON.CAPACITY);
let rtmonCollapsed = false;        // 折りたたみ(既定は表示)
let rtmonPos = { x: 0, y: 0 };     // フロー速度のデッドレコニング積算 [m]
let rtmonResetT = -Infinity;       // 積算リセット時刻(フロー軌跡はこれ以降のみ)
let rtmonLastDrawMs = 0;           // 直近描画時刻(rAF タイムスタンプ)
let rtmonRafId = null;

/* DPR 対応の 2D コンテキスト取得(xyCanvas の setupCanvasDpr と同じ手法) */
function rtmonSetupCanvas(canvas) {
  const dpr = window.devicePixelRatio || 1;
  const w = canvas.width, h = canvas.height;
  canvas.style.width = `${w}px`;
  canvas.style.height = `${h}px`;
  canvas.width = w * dpr;
  canvas.height = h * dpr;
  const ctx = canvas.getContext("2d");
  ctx.scale(dpr, dpr);
  return ctx;
}
const rtmonYawCtx = rtmonSetupCanvas(els.rtmonYawCanvas);
const rtmonVelCtx = rtmonSetupCanvas(els.rtmonVelCanvas);
const rtmonXyCtx = rtmonSetupCanvas(els.rtmonXyCanvas);

function rtmonColors() {
  const css = getComputedStyle(document.documentElement);
  const gv = (name, fb) => css.getPropertyValue(name).trim() || fb;
  return {
    grid: gv("--plot-grid", "#1d2330"),
    axis: gv("--plot-axis", "#384154"),
    text: gv("--text-dim", "#8b93a3"),
    mdg: gv("--rt-madgwick", "#ec4899"),
    ekf1: gv("--rt-ekf1", "#38bdf8"),
    ekf2: gv("--rt-ekf2", "#a78bfa"),
    mocap: gv("--rt-mocap", "#34d399"),
    vx: gv("--rt-vx", "#38bdf8"),
    vy: gv("--rt-vy", "#fbbf24"),
  };
}

/* WS state(20Hz)ごとの1サンプル取り込み+デッドレコニング積算 */
function rtmonOnState() {
  const d = lastDrone;
  if (!d || typeof d.elapsed_ms !== "number") return;
  const t = d.elapsed_ms / 1000;
  const prev = rtmonRing.last();
  if (prev) {
    if (t < prev.t - 1) {
      // 機体再起動(elapsed_ms リセット): 履歴と積算をやり直す
      rtmonRing.clear();
      rtmonPos = { x: 0, y: 0 };
      rtmonResetT = -Infinity;
    } else if (t <= prev.t) {
      return;   // 同一 TLM_STATE の再配信(WS 20Hz > TLM 25Hz の位相揺れ)
    }
  }
  const m = lastMocap;
  const mocapOk = !!(m && m.fresh && m.valid !== false &&
                     typeof m.x === "number" && typeof m.y === "number");
  const mocapYaw = m
    ? ((typeof m.yaw_true_deg === "number") ? m.yaw_true_deg
       : (typeof m.yaw_deg === "number") ? m.yaw_deg : null)
    : null;
  const vx = typeof d.flow_vx_mps === "number" ? d.flow_vx_mps : null;
  const vy = typeof d.flow_vy_mps === "number" ? d.flow_vy_mps : null;
  const velValid = typeof d.flow_status === "number" &&
                   (d.flow_status & RTMON.FLOW_VEL_VALID) !== 0;

  const last2 = rtmonRing.last();   // クリア後は null
  const dt = last2 ? t - last2.t : 0;
  let mvx = null, mvy = null;       // MoCap 速度の機体系換算(比較表示用)
  if (last2 && dt > 0 && dt <= RTMON.DT_MAX_S) {
    // デッドレコニング積算 p += R(ψ_active)·v_flow·dt(vel_valid 時のみ。
    // ψ_active はアクティブ推定器ヨー = drone.yaw_est)
    if (velValid && vx !== null && vy !== null &&
        typeof d.yaw_est === "number") {
      const psi = d.yaw_est * Math.PI / 180;
      rtmonPos.x += (Math.cos(psi) * vx - Math.sin(psi) * vy) * dt;
      rtmonPos.y += (Math.sin(psi) * vx + Math.cos(psi) * vy) * dt;
    }
    // MoCap 速度: 位置の前進差分(世界系)を MoCap ヨーで機体系へ回す
    // (フロー速度と同じフレームで重ねられるように)
    if (mocapOk && last2.mx !== null && last2.my !== null &&
        mocapYaw !== null) {
      const wx = (m.x - last2.mx) / dt;
      const wy = (m.y - last2.my) / dt;
      const ang = -mocapYaw * Math.PI / 180;
      const c = Math.cos(ang), s = Math.sin(ang);
      mvx = c * wx - s * wy;
      mvy = s * wx + c * wy;
    }
  }

  rtmonRing.push({
    t,
    yawMdg: typeof d.yaw === "number" ? d.yaw : null,
    yawEst: typeof d.yaw_est === "number" ? d.yaw_est : null,
    yawEkf2: typeof d.ekf2_yaw === "number" ? d.ekf2_yaw : null,
    yawMocap: mocapOk ? mocapYaw : null,
    vx, vy, velValid, mvx, mvy,
    squal: typeof d.flow_squal === "number" ? d.flow_squal : null,
    px: rtmonPos.x, py: rtmonPos.y,
    mx: mocapOk ? m.x : null, my: mocapOk ? m.y : null,
  });
  rtmonRequestDraw();
}

function rtmonVisible() {
  return uiMode === "experiment" && !rtmonCollapsed && !document.hidden;
}

/* rAF 1発予約 + 時刻間引き(<100ms なら描画せず次の WS state で再試行 →
   20Hz 流入時の実効再描画 ≈10Hz。タブ非表示時は rAF ごと停止) */
function rtmonRequestDraw() {
  if (!rtmonVisible() || rtmonRafId !== null) return;
  rtmonRafId = requestAnimationFrame((ts) => {
    rtmonRafId = null;
    if (ts - rtmonLastDrawMs < RTMON.REDRAW_MIN_MS) return;
    rtmonLastDrawMs = ts;
    rtmonDrawAll();
  });
}

/* 時系列1枠を描く。series: [{key, color, label, dash}](key はサンプルの
   フィールド名)。opts.wrap=true で ±180° ラップ跨ぎ(隣接差>180°)の線を
   切る。opts.fixed=[min,max] で固定レンジ、無指定は窓内データから自動。 */
function rtmonDrawTimeChart(canvas, ctx, series, opts) {
  const w = parseFloat(canvas.style.width);
  const h = parseFloat(canvas.style.height);
  const c = rtmonColors();
  ctx.clearRect(0, 0, w, h);
  ctx.font = '10px "SF Mono", ui-monospace, Menlo, monospace';
  const last = rtmonRing.last();
  if (!last) {
    ctx.fillStyle = c.text;
    ctx.fillText("データなし(テレメトリ受信待ち)", 10, h / 2);
    return;
  }
  const padL = 38, padR = 8, padT = 16, padB = 14;
  const t1 = last.t;
  const t0 = t1 - RTMON.WINDOW_S;

  // y レンジ(固定 or 窓内データから自動、最小半幅つき)
  let yMin, yMax;
  if (opts.fixed) {
    [yMin, yMax] = opts.fixed;
  } else {
    let lo = Infinity, hi = -Infinity;
    for (let i = 0; i < rtmonRing.len; i++) {
      const smp = rtmonRing.get(i);
      if (smp.t < t0) continue;
      for (const sr of series) {
        const v = smp[sr.key];
        if (typeof v === "number" && Number.isFinite(v)) {
          if (v < lo) lo = v;
          if (v > hi) hi = v;
        }
      }
    }
    const halfMin = opts.halfMin || 0.1;
    if (!Number.isFinite(lo)) { lo = -halfMin; hi = halfMin; }
    const mid = (hi + lo) / 2;
    const half = Math.max((hi - lo) / 2 * 1.15, halfMin);
    yMin = mid - half;
    yMax = mid + half;
  }
  const xOf = (t) => padL + ((t - t0) / RTMON.WINDOW_S) * (w - padL - padR);
  const yOf = (v) => padT + ((yMax - v) / (yMax - yMin)) * (h - padT - padB);

  // グリッド: 縦=10s 刻み、横=固定レンジは指定目盛 / 自動は4分割
  ctx.lineWidth = 1;
  ctx.strokeStyle = c.grid;
  ctx.fillStyle = c.text;
  for (let g = Math.ceil(t0 / 10) * 10; g <= t1 + 1e-9; g += 10) {
    const x = xOf(g);
    ctx.beginPath(); ctx.moveTo(x, padT); ctx.lineTo(x, h - padB); ctx.stroke();
  }
  const ticks = opts.ticks
    || [yMin, yMin + (yMax - yMin) * 0.25, yMin + (yMax - yMin) * 0.5,
        yMin + (yMax - yMin) * 0.75, yMax];
  for (const v of ticks) {
    if (v < yMin - 1e-9 || v > yMax + 1e-9) continue;
    const y = yOf(v);
    ctx.strokeStyle = Math.abs(v) < 1e-9 ? c.axis : c.grid;
    ctx.beginPath(); ctx.moveTo(padL, y); ctx.lineTo(w - padR, y); ctx.stroke();
    ctx.fillText(String(Math.round(v * 100) / 100), 2, y + 3);
  }

  // 各系列の折れ線(null/ラップ跨ぎで線を切る)
  for (const sr of series) {
    ctx.strokeStyle = sr.color;
    ctx.lineWidth = 1.4;
    if (sr.dash) ctx.setLineDash([4, 3]);
    ctx.beginPath();
    let pen = false;
    let prevV = null;
    for (let i = 0; i < rtmonRing.len; i++) {
      const smp = rtmonRing.get(i);
      if (smp.t < t0) continue;
      const raw = smp[sr.key];
      if (typeof raw !== "number" || !Number.isFinite(raw)) {
        pen = false;
        prevV = null;
        continue;
      }
      const v = opts.wrap ? wrap180(raw) : raw;
      if (opts.wrap && pen && prevV !== null && Math.abs(v - prevV) > 180) {
        pen = false;   // ±180° 跨ぎ: セグメント分割(縦線を描かない)
      }
      const x = xOf(smp.t), y = yOf(clamp(v, yMin, yMax));
      if (!pen) { ctx.moveTo(x, y); pen = true; } else { ctx.lineTo(x, y); }
      prevV = v;
    }
    ctx.stroke();
    ctx.setLineDash([]);
  }

  // 凡例(上端に色付きラベルを並べる)
  let lx = padL + 4;
  for (const sr of series) {
    ctx.fillStyle = sr.color;
    ctx.fillText(sr.label, lx, 11);
    lx += ctx.measureText(sr.label).width + 12;
  }
}

/* 2D位置(正方形): フロー積算軌跡(直近60s・リセット以降)+ MoCap 位置 */
function rtmonDrawXyChart() {
  const canvas = els.rtmonXyCanvas;
  const ctx = rtmonXyCtx;
  const w = parseFloat(canvas.style.width);
  const h = parseFloat(canvas.style.height);
  const c = rtmonColors();
  ctx.clearRect(0, 0, w, h);
  ctx.font = '10px "SF Mono", ui-monospace, Menlo, monospace';
  const last = rtmonRing.last();
  if (!last) {
    ctx.fillStyle = c.text;
    ctx.fillText("データなし", 10, h / 2);
    return;
  }
  const t0 = last.t - RTMON.WINDOW_S;

  // スケール自動: 窓内のフロー軌跡(リセット以降)と MoCap 位置の最大絶対値
  // (下限 ±1m)
  let half = RTMON.XY_HALF_MIN_M;
  for (let i = 0; i < rtmonRing.len; i++) {
    const smp = rtmonRing.get(i);
    if (smp.t < t0) continue;
    if (smp.t >= rtmonResetT) {
      half = Math.max(half, Math.abs(smp.px), Math.abs(smp.py));
    }
    if (smp.mx !== null) {
      half = Math.max(half, Math.abs(smp.mx), Math.abs(smp.my));
    }
  }
  half *= 1.1;
  const toPx = (x, y) => [w / 2 + (x / half) * (w / 2 - 6),
                          h / 2 - (y / half) * (h / 2 - 6)];

  // グリッド(0.5m / 1m / 2m 刻みをスケールに応じて選択)+ 原点軸
  const step = half <= 1.6 ? 0.5 : half <= 3.2 ? 1 : 2;
  ctx.lineWidth = 1;
  for (let g = -Math.floor(half / step) * step; g <= half + 1e-9; g += step) {
    ctx.strokeStyle = Math.abs(g) < 1e-9 ? c.axis : c.grid;
    const [gx] = toPx(g, 0);
    const [, gy] = toPx(0, g);
    ctx.beginPath(); ctx.moveTo(gx, 0); ctx.lineTo(gx, h); ctx.stroke();
    ctx.beginPath(); ctx.moveTo(0, gy); ctx.lineTo(w, gy); ctx.stroke();
  }
  ctx.fillStyle = c.text;
  ctx.fillText(`±${half.toFixed(1)}m`, 4, 11);

  // 軌跡2本: フロー積算(実線)と MoCap(接続時のみ)。null で線を切る
  const drawTrail = (getXY, color) => {
    ctx.strokeStyle = color;
    ctx.lineWidth = 1.4;
    ctx.globalAlpha = 0.6;
    ctx.beginPath();
    let pen = false;
    let lastPt = null;
    for (let i = 0; i < rtmonRing.len; i++) {
      const smp = rtmonRing.get(i);
      if (smp.t < t0) continue;
      const pt = getXY(smp);
      if (pt === null) { pen = false; continue; }
      const [x, y] = toPx(pt[0], pt[1]);
      if (!pen) { ctx.moveTo(x, y); pen = true; } else { ctx.lineTo(x, y); }
      lastPt = pt;
    }
    ctx.stroke();
    ctx.globalAlpha = 1;
    if (lastPt !== null) {   // 現在点
      const [x, y] = toPx(lastPt[0], lastPt[1]);
      ctx.fillStyle = color;
      ctx.beginPath(); ctx.arc(x, y, 4, 0, Math.PI * 2); ctx.fill();
    }
    return lastPt !== null;
  };
  drawTrail((smp) => (smp.t >= rtmonResetT ? [smp.px, smp.py] : null), c.vx);
  const hasMocap = drawTrail(
    (smp) => (smp.mx !== null ? [smp.mx, smp.my] : null), c.mocap);

  // 凡例
  ctx.fillStyle = c.vx;
  ctx.fillText("フロー積算", 4, h - 6);
  if (hasMocap) {
    ctx.fillStyle = c.mocap;
    ctx.fillText("MoCap", 78, h - 6);
  }
}

function rtmonDrawAll() {
  const c = rtmonColors();
  rtmonDrawTimeChart(els.rtmonYawCanvas, rtmonYawCtx, [
    { key: "yawMdg", color: c.mdg, label: "Madgwick" },
    { key: "yawEst", color: c.ekf1, label: "EKF" },
    { key: "yawEkf2", color: c.ekf2, label: "EKF2" },
    { key: "yawMocap", color: c.mocap, label: "MoCap" },
  ], { wrap: true, fixed: [-190, 190], ticks: [-180, -90, 0, 90, 180] });
  rtmonDrawTimeChart(els.rtmonVelCanvas, rtmonVelCtx, [
    { key: "vx", color: c.vx, label: "flow_vx" },
    { key: "vy", color: c.vy, label: "flow_vy" },
    { key: "mvx", color: c.vx, label: "MoCap vx", dash: true },
    { key: "mvy", color: c.vy, label: "MoCap vy", dash: true },
  ], { halfMin: RTMON.VEL_HALF_MIN });
  rtmonDrawXyChart();

  // 現在値の1行サマリ
  const smp = rtmonRing.last();
  if (smp) {
    const fv = (v, d2) => (typeof v === "number" ? v.toFixed(d2) : "--");
    els.rtmonInfo.textContent =
      `積算位置 x=${fv(smp.px, 2)} y=${fv(smp.py, 2)} m` +
      ` / flow vx=${fv(smp.vx, 2)} vy=${fv(smp.vy, 2)} m/s` +
      ` SQUAL=${smp.squal ?? "--"}` +
      `${smp.velValid ? "" : "(vel無効)"}` +
      (smp.mx !== null
        ? ` / MoCap x=${fv(smp.mx, 2)} y=${fv(smp.my, 2)} m` : "");
  }
}

function rtmonReset() {
  rtmonPos = { x: 0, y: 0 };
  const last = rtmonRing.last();
  rtmonResetT = last ? last.t : -Infinity;
  rtmonRequestDraw();
}

function rtmonToggle() {
  rtmonCollapsed = !rtmonCollapsed;
  els.rtmonBody.classList.toggle("hidden", rtmonCollapsed);
  els.btnRtmonToggle.textContent = rtmonCollapsed ? "表示" : "非表示";
  if (!rtmonCollapsed) rtmonRequestDraw();
}

/* ---- REST 状態の取得と描画(Experiment 各パネル) ---- */

function ffModeLabel(v) {
  return FF_MODE_NAMES[v] !== undefined ? FF_MODE_NAMES[v] : String(v);
}

function ffAppliedText(st) {
  const a = st && st.applied;
  if (!a) return "FF: 未適用";
  const when = a.applied_at
    ? new Date(a.applied_at * 1000).toLocaleString("ja-JP") : "-";
  const estName = a.est === 2 ? "EKF2" : a.est === 1 ? "EKF" : "CF";
  return `FF適用中: ${a.name}(ff=${ffModeLabel(a.ff)}, est=${estName}, ` +
         `crc=${a.crc || "-"}${a.verified ? "" : ", 未検証"})${when !== "-" ? " " + when : ""}`;
}

function renderFfStatus() {
  const st = ffStatus;
  if (!st) return;
  const profileOpts = (st.profiles || []).map((p) => ({
    value: p.name,
    label: p.warnings_count ? `${p.name}(警告${p.warnings_count})` : p.name,
    title: p.memo || "",
  }));
  const preferred = st.applied ? st.applied.name : null;
  rebuildSelect(els.ffQuickSelect, profileOpts, preferred);
  rebuildSelect(els.ffProfileSelect, profileOpts, preferred);
  if (profileOpts.length === 0) {
    for (const sel of [els.ffQuickSelect, els.ffProfileSelect]) {
      const opt = document.createElement("option");
      opt.value = "";
      opt.textContent = "(プロファイルなし)";
      sel.appendChild(opt);
    }
  }
  const folderOpts = (st.folders || []).map((f) => ({ value: f, label: f }));
  rebuildSelect(els.ffFolderSelect, folderOpts);
  if (folderOpts.length === 0) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = "(sweep_results にフォルダなし)";
    els.ffFolderSelect.appendChild(opt);
  }
  const banner = ffAppliedText(st) + (st.busy ? "(操作中…)" : "");
  els.ffAppliedBanner.textContent = banner;
  els.ffAppliedExp.textContent = banner;
  const appliedOk = !!st.applied;
  els.ffAppliedBanner.classList.toggle("applied", appliedOk);
  els.ffAppliedExp.classList.toggle("applied", appliedOk);
  if (st.message) els.ffApplyMsg.textContent = st.message;
}

async function fetchFfStatus() {
  const body = await apiGet("/api/ffprofile", true);
  if (body) {
    ffStatus = body;
    renderFfStatus();
    updateMultiFfSelects();   // 複数機タブの機体別 FF セレクタも追従
  }
}

function setFfStatus(resp) {
  // POST 応答は status() のフィールドを含む(ok/message 等の余剰キーは無害)
  if (resp && Array.isArray(resp.profiles)) {
    ffStatus = resp;
    renderFfStatus();
    updateMultiFfSelects();
  }
}

/* ---- magbias(磁気オートチューン)パネル ---- */

function renderMagbias() {
  const st = magbiasStatus;
  if (!st) return;
  const profileOpts = (st.profiles || []).map((p) => ({
    value: p.name,
    label: p.delta_b_ut ? p.name : `${p.name}(Δbなし=適用不可)`,
    title: p.delta_b_ut
      ? `Δb=(${p.delta_b_ut.map((v) => v.toFixed(2)).join(", ")})µT`
      : "ヨー励振不足(hover_residual のみ)",
  }));
  const preferred = st.applied ? st.applied.name : null;
  rebuildSelect(els.magbiasSelect, profileOpts, preferred);
  if (profileOpts.length === 0) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = "(プロファイルなし)";
    els.magbiasSelect.appendChild(opt);
  }
  const logOpts = (st.logs || []).map((f) => ({ value: f, label: f }));
  rebuildSelect(els.magbiasLogSelect, logOpts);
  if (logOpts.length === 0) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = "(flight_logs に CSV なし)";
    els.magbiasLogSelect.appendChild(opt);
  }
  const a = st.applied;
  const banner = a
    ? `magbias適用中: ${a.name} Δb=(${(a.delta_b_ut || [])
        .map((v) => v.toFixed(2)).join(", ")})µT` +
      `${a.verified ? "" : "(未検証)"}`
    : "magbias: 未適用";
  els.magbiasApplied.textContent = banner + (st.busy ? "(操作中…)" : "");
  els.magbiasApplied.classList.toggle("applied", !!a);
  if (st.message) els.magbiasMsg.textContent = st.message;
}

async function fetchMagbias() {
  const body = await apiGet("/api/magbias", true);
  if (body) {
    magbiasStatus = body;
    renderMagbias();
  }
}

function setMagbiasStatus(resp) {
  if (resp && Array.isArray(resp.profiles)) {
    magbiasStatus = resp;
    renderMagbias();
  }
}

/* ---- フロー較正(純回転 2×2 フィット)パネル ---- */

/* メータースケール/合格ライン(core/flowcal.py の受入基準と同値。
   合格ラインの位置は index.html の .fc-meter-line と対で保守する) */
const FLOWCAL_UI = {
  SAMPLES_SCALE: 600,   // n メーターのフルスケール(合格 200 = 33.3%)
  SAMPLES_PASS: 200,
  VALID_PASS: 0.8,      // 有効サンプル率の目安ライン(80%)
  STD_SCALE: 1.0,       // 励振 std のフルスケール [rad/s](合格 0.30 = 30%)
  STD_PASS: 0.3,
  SQUAL_SCALE: 128,     // SQUAL フルスケール(ゲート 25 = 19.5%)
  SQUAL_PASS: 25,
  TOF_SCALE_M: 1.0,     // ToF フルスケール [m](推奨帯 0.15〜0.5 = 15〜50%)
  TOF_MIN_M: 0.15,
  TOF_MAX_M: 0.5,
  POLL_MS: 500,         // 記録中ライブメーターのポーリング間隔
};

function setFcMeter(fill, valEl, frac, pass, text) {
  fill.style.width = `${clamp(frac * 100, 0, 100)}%`;
  fill.classList.toggle("pass", !!pass);
  valEl.textContent = text;
}

function fcMatrixLabel(m) {
  if (!Array.isArray(m) || m.length !== 4) return "--";
  // ワイヤ順は m00,m01,m10,m11(行優先)
  return `[${m[0].toFixed(1)} ${m[1].toFixed(1)}; ${m[2].toFixed(1)} ${m[3].toFixed(1)}]`;
}

function fcImpliedLabel(m) {
  // 含意パラメータ: kx=|K·e1|(第1列), ky=|K·e2|(第2列), φ0(diag·R 形の平均)
  if (!Array.isArray(m) || m.length !== 4) return "";
  const kx = Math.hypot(m[0], m[2]);
  const ky = Math.hypot(m[1], m[3]);
  const phi = 0.5 * (Math.atan2(-m[1], m[0]) + Math.atan2(m[2], m[3]))
    * 180 / Math.PI;
  return ` kx=${kx.toFixed(0)} ky=${ky.toFixed(0)} φ0=${phi >= 0 ? "+" : ""}${phi.toFixed(1)}°`;
}

function renderFlowcal() {
  const st = flowcalStatus;
  if (!st) return;
  // 記録状態+ライブメーター
  els.flowcalRecStatus.textContent = st.collecting
    ? `記録中 ${st.duration_s != null ? st.duration_s.toFixed(0) : "-"}s`
    : (st.busy ? "適用中…" : "待機中");
  const live = st.live;
  const nValid = live ? live.n_valid : 0;
  setFcMeter(els.fcFillSamples, els.fcValSamples,
    nValid / FLOWCAL_UI.SAMPLES_SCALE, nValid >= FLOWCAL_UI.SAMPLES_PASS,
    live ? `${live.n_valid}/${live.n_total}` : "--");
  setFcMeter(els.fcFillValid, els.fcValValid,
    live ? live.valid_ratio : 0,
    live && live.valid_ratio >= FLOWCAL_UI.VALID_PASS,
    live ? `${(live.valid_ratio * 100).toFixed(0)}%` : "--");
  setFcMeter(els.fcFillStdP, els.fcValStdP,
    live ? live.std_p_rad_s / FLOWCAL_UI.STD_SCALE : 0,
    live && live.std_p_rad_s >= FLOWCAL_UI.STD_PASS,
    live ? `${live.std_p_rad_s.toFixed(2)} rad/s` : "--");
  setFcMeter(els.fcFillStdQ, els.fcValStdQ,
    live ? live.std_q_rad_s / FLOWCAL_UI.STD_SCALE : 0,
    live && live.std_q_rad_s >= FLOWCAL_UI.STD_PASS,
    live ? `${live.std_q_rad_s.toFixed(2)} rad/s` : "--");
  setFcMeter(els.fcFillSqual, els.fcValSqual,
    live ? live.squal / FLOWCAL_UI.SQUAL_SCALE : 0,
    live && live.squal >= FLOWCAL_UI.SQUAL_PASS,
    live ? String(live.squal) : "--");
  setFcMeter(els.fcFillTof, els.fcValTof,
    live ? live.tof_m / FLOWCAL_UI.TOF_SCALE_M : 0,
    live && live.tof_m >= FLOWCAL_UI.TOF_MIN_M
      && live.tof_m <= FLOWCAL_UI.TOF_MAX_M,
    live ? `${live.tof_m.toFixed(2)} m` : "--");

  // フィット結果+合否バッジ
  const fit = st.fit;
  if (!fit) {
    els.flowcalBadge.textContent = "フィット未実施";
    els.flowcalBadge.className = "badge b-dim";
  } else if (fit.ok) {
    els.flowcalBadge.textContent = "合格(適用可)";
    els.flowcalBadge.className = "badge b-ok";
  } else {
    const nw = (fit.warnings || []).length;
    els.flowcalBadge.textContent = `不合格(警告${nw}件)`;
    els.flowcalBadge.className = "badge b-warn";
  }
  const hasMatrix = !!(fit && Array.isArray(fit.matrix));
  els.fcR2.textContent = hasMatrix
    ? `${fit.r2x.toFixed(3)} / ${fit.r2y.toFixed(3)}` : "--";
  els.fcScale.textContent = hasMatrix
    ? `${fit.kx.toFixed(1)} / ${fit.ky.toFixed(1)}` : "--";
  els.fcPhi.textContent = hasMatrix
    ? `${fit.phi0_deg >= 0 ? "+" : ""}${fit.phi0_deg.toFixed(1)}° / ×${fit.ratio.toFixed(2)}` : "--";
  els.fcUsed.textContent = hasMatrix
    ? `${fit.n_used}(棄却 ${fit.n_rejected} / 有効 ${fit.n_valid} / 総 ${fit.n_total})`
    : (fit ? `有効 ${fit.n_valid} / 総 ${fit.n_total}` : "--");
  els.fcMatrix.textContent = hasMatrix
    ? fcMatrixLabel(fit.matrix) : "--";
  const drone = st.drone;
  els.fcDroneMatrix.textContent = !drone
    ? "--(CAL_GET 未受信)"
    : (drone.valid
      ? fcMatrixLabel(drone.matrix) + fcImpliedLabel(drone.matrix)
      : "未設定(既定 diag(450,450))");

  // 保存済みプロファイル一覧(magbias パネルと同様式。適用中を優先選択)
  const profileOpts = (st.profiles || []).map((p) => ({
    value: p.name,
    label: p.error ? `${p.name}(読込不可)` : p.name,
    title: p.kx != null && p.ky != null
      ? `kx=${p.kx.toFixed(0)} ky=${p.ky.toFixed(0)}` +
        (p.phi0_deg != null
          ? ` φ0=${p.phi0_deg >= 0 ? "+" : ""}${p.phi0_deg.toFixed(1)}°` : "") +
        (p.r2x != null && p.r2y != null
          ? ` r²=${p.r2x.toFixed(2)}/${p.r2y.toFixed(2)}` : "") +
        (p.memo ? ` — ${p.memo}` : "")
      : (p.memo || ""),
  }));
  const preferredProfile = st.applied && st.applied.name
    ? st.applied.name : null;
  rebuildSelect(els.flowcalSelect, profileOpts, preferredProfile);
  if (profileOpts.length === 0) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = "(プロファイルなし — フィット合格時に自動保存)";
    els.flowcalSelect.appendChild(opt);
  }

  // 適用状態バナー+メッセージ
  const a = st.applied;
  els.flowcalApplied.textContent = a
    ? `flowcal適用中: ${a.name ? `${a.name} ` : ""}` +
      `K=${fcMatrixLabel(a.matrix)} ` +
      `φ0=${a.phi0_deg >= 0 ? "+" : ""}${(a.phi0_deg ?? 0).toFixed(1)}°` +
      `${a.forced ? "(force適用)" : ""}${a.verified ? "" : "(未検証)"}`
    : "flowcal: 未適用(既定 diag(450,450))";
  els.flowcalApplied.classList.toggle("applied", !!a);
  // P2a: 適用記録と機体行列の照合警告(applied_unverified/不一致)を優先表示
  if (st.verify_warning) {
    els.flowcalMsg.textContent =
      (st.message ? `${st.message} / ` : "") + `⚠ ${st.verify_warning}`;
  } else if (st.message) {
    els.flowcalMsg.textContent = st.message;
  }

  // ボタン活性(適用は「フィット結果あり」が条件。合否はサーバが判定し、
  // 不合格は confirm 経由の force で強制適用できる)
  els.btnFlowcalStart.disabled = !!(st.collecting || st.busy);
  els.btnFlowcalStop.disabled = !st.collecting;
  els.btnFlowcalApply.disabled = !!(st.collecting || st.busy || !hasMatrix);
  els.btnFlowcalClear.disabled = !!st.busy;
  const profileName = els.flowcalSelect.value;
  els.btnFlowcalProfileApply.disabled =
    !!(st.collecting || st.busy || !profileName);
  els.btnFlowcalProfileDelete.disabled = !!(st.busy || !profileName);
}

function flowcalEnsurePolling() {
  const active = !!(flowcalStatus && flowcalStatus.collecting);
  if (active && flowcalPollTimer === null) {
    flowcalPollTimer = setInterval(fetchFlowcal, FLOWCAL_UI.POLL_MS);
  } else if (!active && flowcalPollTimer !== null) {
    clearInterval(flowcalPollTimer);
    flowcalPollTimer = null;
  }
}

async function fetchFlowcal() {
  const body = await apiGet("/api/flowcal", true);
  if (body && typeof body.collecting === "boolean") {
    flowcalStatus = body;
    renderFlowcal();
    flowcalEnsurePolling();
  }
}

function setFlowcalStatus(resp) {
  // POST 応答は status() のフィールドを含む(ok/message 等の余剰キーは無害)
  if (resp && typeof resp.collecting === "boolean") {
    flowcalStatus = resp;
    renderFlowcal();
    flowcalEnsurePolling();
  }
}

function renderGeomag() {
  const st = geomagStatus;
  if (!st) return;
  const opts = (st.profiles || []).map((p) => ({ value: p.id, label: p.label }));
  const cfg = st.config || {};
  rebuildSelect(els.geomagSelect, opts, cfg.selected || null);
  if (cfg.error) {
    els.geomagInfo.textContent = "--";
    els.geomagMsg.textContent = String(cfg.error);
    return;
  }
  const p = cfg.profile;
  if (p) {
    els.geomagInfo.textContent =
      `${p.label}: 偏角${p.declination_east_deg >= 0 ? "東" : "西"}` +
      `${Math.abs(p.declination_east_deg).toFixed(2)}° 伏角${p.inclination_deg.toFixed(1)}° ` +
      `H=${p.horizontal_uT.toFixed(1)} F=${p.total_uT.toFixed(1)}µT`;
  }
}

async function fetchGeomag() {
  const body = await apiGet("/api/geomag");
  if (body) {
    geomagStatus = body;
    renderGeomag();
  }
}

function renderCalprof() {
  const st = calprofStatus;
  if (!st) return;
  const opts = (st.profiles || []).map((p) => {
    const valid = p.valid
      ? Object.keys(p.valid).filter((k) => p.valid[k]).join(",") : "";
    return {
      value: p.name,
      label: p.error ? `${p.name}(読込不可)` : p.name,
      title: valid ? `有効: ${valid}` : "",
    };
  });
  rebuildSelect(els.calprofSelect, opts);
  if (opts.length === 0) {
    const opt = document.createElement("option");
    opt.value = "";
    opt.textContent = "(保存済みプロファイルなし)";
    els.calprofSelect.appendChild(opt);
  }
  if (st.message) els.calprofMsg.textContent = st.message;
}

async function fetchCalprof() {
  const body = await apiGet("/api/calprofile");
  if (body) {
    calprofStatus = body;
    renderCalprof();
  }
}

function renderAccel6() {
  const st = accel6Status;
  if (!st) return;
  const captured = st.captured || [];
  els.accel6Captured.textContent = captured.length
    ? `${captured.join(", ")}(${captured.length}/6)${st.ready ? " — Apply 可" : ""}`
    : "なし(0/6)";
  for (const btn of document.querySelectorAll(".accel6-face")) {
    btn.classList.toggle("done", captured.includes(btn.dataset.face));
  }
}

async function fetchAccel6() {
  const body = await apiGet("/api/accel6");
  if (body) {
    accel6Status = body;
    renderAccel6();
  }
}

function renderCal3d() {
  const st = cal3dStatus;
  if (!st) return;
  const fit = st.fit;
  els.cal3dStatusText.textContent = st.error
    ? String(st.error)
    : (st.collecting ? "収集中(機体を全方位に回す)" : (fit ? "Fit 済み(Apply 可)" : "待機中"));
  if (typeof st.sample_count === "number") {
    els.cal3dSamples.textContent = String(st.sample_count);
  }
  els.cal3dFit.textContent = fit && typeof fit.relative_rms_error === "number"
    ? `${(fit.relative_rms_error * 100).toFixed(2)}%(${fit.sample_count}点)` : "--";
  const saved = st.saved;
  if (saved && !saved.error) {
    const rms = typeof saved.relative_rms_error === "number"
      ? `${(saved.relative_rms_error * 100).toFixed(2)}%` : "-";
    els.cal3dSaved.textContent =
      `RMS ${rms} / ${saved.sample_count ?? "-"}点${saved.applied_at ? " / 適用済" : ""}`;
  } else {
    els.cal3dSaved.textContent = saved && saved.error ? "読込不可" : "なし";
  }
}

async function fetchCal3d() {
  const body = await apiGet("/api/cal3d");
  if (body) {
    cal3dStatus = body;
    renderCal3d();
  }
}

function refreshExperimentPanels() {
  fetchFfStatus();
  fetchGeomag();
  fetchCalprof();
  fetchAccel6();
  fetchCal3d();
  fetchMagbias();
  fetchFlowcal();
}

/* ---- FF 適用(共通: mag3d 不一致時は confirm で force 再適用) ---- */
async function doFfApply(name, ff, est, drone) {
  if (!name) {
    appendConsole("ui", "FFプロファイルが選択されていません");
    return;
  }
  const body = { action: "apply", name };
  if (ff !== undefined) body.ff = ff;
  if (est !== undefined) body.est = est;
  if (drone) body.drone = drone;   // 複数機モード: 機体別適用(ノード宛)
  let resp = await apiPost("/api/ffprofile", body);
  if (resp && !resp.ok && resp.mag3d_mismatch && Array.isArray(resp.diffs)) {
    const detail = resp.diffs.slice(0, 4).join("\n");
    if (window.confirm(`機体の mag3d がプロファイル取得時と一致しません:\n${detail}\n\n強制適用しますか?(ヨー推定精度が劣化する可能性があります)`)) {
      resp = await apiPost("/api/ffprofile", { ...body, force: true });
    }
  }
  if (resp) {
    const target = drone ? `(${drone})` : "";
    appendConsole("ui", resp.ok
      ? `FFプロファイルを適用しました: ${name}${target}`
      : `FF適用失敗${target}: ${resp.message || "不明なエラー"}`);
    setFfStatus(resp);
  }
}

/* ===================== イベント配線 ===================== */
function wireEvents() {
  // ヘッダ
  els.btnRefreshPorts.addEventListener("click", fetchPorts);
  els.btnConnect.addEventListener("click", () => {
    if (lastSession && lastSession.serial_connected) {
      sendCommand("disconnect");
    } else {
      const port = els.portSelect.value;
      if (!port) {
        appendConsole("ui", "シリアルポートを選択してください");
        return;
      }
      sendCommand("connect", { port });
    }
  });
  // 機体プロファイル編集モーダル
  els.btnEditAirframes.addEventListener("click", openAirframeEditor);
  els.btnAfAddRow.addEventListener("click", () => {
    afRows.push(afBlankRow());
    renderAfEditorRows();
  });
  els.btnAfCancel.addEventListener("click", closeAirframeEditor);
  els.btnAfSave.addEventListener("click", saveAirframes);

  els.airframeSelect.addEventListener("change", () => {
    const name = els.airframeSelect.value;
    airframeSentAt = now();
    sendCommand("select_airframe", { name });
    // 選択機体の既定高度をスライダ初期値へ反映(飛行中は触らない)
    const af = airframes.find((a) => a.name === name);
    if (af && typeof af.default_alt_m === "number" && !isFlying()) {
      const v = clamp(af.default_alt_m, UI.ALT_MIN_M, UI.ALT_MAX_M);
      els.altSlider.value = String(v);
      els.altValue.textContent = `${v.toFixed(2)} m`;
      els.targetZ.value = v.toFixed(2);
    }
  });

  // タブ(4モードタブ: posture / position / multi / experiment)
  for (const tab of [els.tabPosture, els.tabPosition, els.tabMulti,
                     els.tabExperiment]) {
    tab.addEventListener("click", () => {
      // 設定タブ → 現行モードタブへの復帰は純 UI 操作(set_mode を送らない)
      // のため飛行中でも許可する(設定タブ表示中に TLM 由来の飛行昇格が
      // 起きた場合に操作パネルへ戻れなくなるのを防ぐ)
      if (settingsOpen && tab.dataset.mode === uiMode) {
        closeSettings();
        applyMode(uiMode, false);
        return;
      }
      if (isFlying()) {
        appendConsole("ui", "飛行中はモードを切り替えできません");
        return;
      }
      const wasSettings = settingsOpen;
      closeSettings();
      if (tab.dataset.mode !== uiMode) {
        applyMode(tab.dataset.mode, true);
      } else if (wasSettings) {
        applyMode(uiMode, false);   // 設定タブから同一モードタブへ戻る場合の表示復元
      }
    });
  }

  // 設定タブ(UI専用: set_mode を送らない)
  els.tabSettings.addEventListener("click", () => {
    if (isFlying()) {
      appendConsole("ui", "飛行中はモードを切り替えできません");
      return;
    }
    openSettings();
  });
  els.btnMapApply.addEventListener("click", () =>
    withBusy(els.btnMapApply, applyMapping));
  els.btnMapReload.addEventListener("click", () =>
    withBusy(els.btnMapReload, () => loadMapping(false)));
  els.btnMapPreview.addEventListener("click", toggleMapPreview);
  els.btnYawZeroAlign.addEventListener("click", () =>
    withBusy(els.btnYawZeroAlign, () => alignYawOffset(0)));
  els.btnYawTlmAlign.addEventListener("click", () =>
    withBusy(els.btnYawTlmAlign, async () => {
      const yaw = currentDroneYawDeg();
      if (yaw === null) {
        setMapMsg("機体のヨー推定が取得できません(テレメトリ未受信)", true);
        return;
      }
      await alignYawOffset(yaw);
    }));

  // 複数機タブ: 選択適用 / 一斉スタート / リジッドボディ確認
  els.btnMultiApply.addEventListener("click", sendMultiSelect);
  els.btnMultiStart.addEventListener("click", () => {
    const multi = lastSession && lastSession.multi;
    const names = ((multi && multi.drones) || []).map((d) => d.name);
    if (window.confirm(
        `複数機モードで一斉離陸します(${names.join(", ")})。よろしいですか?`)) {
      sendCommand("multi_start");
      appendConsole("ui", "一斉スタート送信");
    }
  });
  els.btnRbCheck.addEventListener("click", toggleRbCheck);

  // START / STOP / RESET(data-action で配線)
  for (const btn of document.querySelectorAll("[data-action=start]")) {
    btn.addEventListener("click", () => {
      const label = uiMode === "posture" ? "Posture(姿勢制御)" : "Position(位置制御)";
      if (window.confirm(`${label} モードで離陸を開始します。よろしいですか?`)) {
        sendCommand("start");
        appendConsole("ui", "START 送信");
      }
    });
  }
  for (const btn of document.querySelectorAll("[data-action=stop]")) {
    btn.addEventListener("click", doStop);
  }
  // プリフライト・インターロックの強制離陸(P1-2: 明示的 override)
  for (const btn of els.forceStartBtns) {
    btn.addEventListener("click", () => {
      if (window.confirm(
          "インターロック未成立のまま強制離陸します(EKF2 ヨー基準の融合が"
          + "未確認 — ヨー観測なしのコースト飛行になる可能性)。よろしいですか?")) {
        sendCommand("start", { force: true });
        appendConsole("ui", "START 送信(force: インターロック解除)");
      }
    });
  }
  els.btnRearm.addEventListener("click", () => {
    if (window.confirm("Re-arm(RESET)します。機体が静止し高度0.15m未満であることを確認してください。")) {
      sendCommand("reset");
      appendConsole("ui", "RESET 送信");
    }
  });

  // Posture スライダ(10Hzスロットルで setpoint 送信)
  const onSliderInput = () => {
    els.rollValue.textContent = fmtDeg(parseFloat(els.rollSlider.value));
    els.pitchValue.textContent = fmtDeg(parseFloat(els.pitchSlider.value));
    els.altValue.textContent = `${parseFloat(els.altSlider.value).toFixed(2)} m`;
    sendSetpointThrottled();
  };
  for (const el of [els.rollSlider, els.pitchSlider, els.altSlider]) {
    el.addEventListener("input", onSliderInput);
  }
  els.btnCenter.addEventListener("click", () => {
    els.rollSlider.value = "0";
    els.pitchSlider.value = "0";
    onSliderInput();
  });

  // v2: ヨー角スライダ+ヨー角制御トグル(Posture/Position 共通)
  const onYawInput = () => {
    els.yawValue.textContent = fmtDeg(parseFloat(els.yawSlider.value));
    sendYawThrottled();
  };
  els.yawSlider.addEventListener("input", onYawInput);
  els.btnYawCenter.addEventListener("click", () => {
    els.yawSlider.value = "0";
    onYawInput();
  });
  els.yawCtrlToggle.addEventListener("change", () => {
    yawCtrlSentAt = now();
    const enabled = els.yawCtrlToggle.checked;
    sendCommand("set_yaw_control", { enabled });
    els.ffQuickBlock.classList.toggle("hidden", !enabled);
    els.yawSlider.disabled = !enabled;
    els.btnYawCenter.disabled = !enabled;
    if (enabled) {
      // ON にした瞬間の目標をサーバへ送っておく(スライダ据え置きでも一致させる)
      sendYawNow();
      fetchFfStatus();
    }
  });
  els.btnFfQuickApply.addEventListener("click", () =>
    withBusy(els.btnFfQuickApply, () => doFfApply(els.ffQuickSelect.value)));

  // Position 目標入力+プリセット
  const onTargetChanged = () => sendTargetThrottled();
  for (const el of [els.targetX, els.targetY, els.targetZ]) {
    el.addEventListener("change", onTargetChanged);
  }
  els.btnPresetHere.addEventListener("click", () => {
    // この場で: 現在のMoCap位置XYを目標に(Zは現在の入力値を維持)
    if (!lastMocap || typeof lastMocap.x !== "number") {
      appendConsole("ui", "MoCap位置が未受信のため「この場で」を設定できません");
      return;
    }
    if (lastMocap.valid === false) {
      // 無効中の表示位置は凍結値のため、目標にすると実位置とずれる
      appendConsole("ui", "MoCap位置データが無効のため「この場で」を設定できません(トラッキング復帰を待ってください)");
      return;
    }
    els.targetX.value = lastMocap.x.toFixed(2);
    els.targetY.value = lastMocap.y.toFixed(2);
    onTargetChanged();
  });
  els.btnPresetOrigin.addEventListener("click", () => {
    els.targetX.value = "0.00";
    els.targetY.value = "0.00";
    onTargetChanged();
  });

  // v2: 軌道セレクタ+円/シャトル/評価シーケンス開始/停止
  els.trajSelect.addEventListener("change", () => {
    trajTouchedAt = now();
    els.circleParams.classList.toggle("hidden", els.trajSelect.value !== "circle");
    els.shuttleParams.classList.toggle("hidden", els.trajSelect.value !== "shuttle");
    els.sequenceParams.classList.toggle("hidden", els.trajSelect.value !== "sequence");
  });
  els.btnCircleStart.addEventListener("click", () => {
    const radius = clamp(parseFloat(els.circleR.value) || 0,
                         trajLimits.radius_min_m, trajLimits.radius_max_m);
    const period = clamp(parseFloat(els.circlePeriod.value) || 0,
                         trajLimits.period_min_s, trajLimits.period_max_s);
    const cx = clamp(parseFloat(els.circleCx.value) || 0,
                     -trajLimits.center_abs_max_m, trajLimits.center_abs_max_m);
    const cy = clamp(parseFloat(els.circleCy.value) || 0,
                     -trajLimits.center_abs_max_m, trajLimits.center_abs_max_m);
    const alt = clamp(parseFloat(els.circleAlt.value) || UI.ALT_MIN_M,
                      UI.ALT_MIN_M, UI.ALT_MAX_M);
    sendCommand("circle_start", {
      center_x: cx, center_y: cy, radius_m: radius, period_s: period,
      clockwise: els.circleDir.value === "cw", alt_m: alt,
      face_tangent: els.circleFaceTangent.checked,
    });
    appendConsole("ui",
      `円軌道開始要求: 中心(${cx.toFixed(2)}, ${cy.toFixed(2)}) r=${radius.toFixed(2)}m ` +
      `周期${period.toFixed(0)}s ${els.circleDir.value.toUpperCase()} 高度${alt.toFixed(2)}m`);
  });
  els.btnCircleStop.addEventListener("click", () => {
    sendCommand("circle_stop");
    appendConsole("ui", "円軌道停止要求(現在目標でホバ復帰)");
  });
  // シャトル: 方向は X軸/Y軸/角度指定(角度入力は角度指定時のみ有効)
  els.shuttleAxisMode.addEventListener("change", () => {
    const mode = els.shuttleAxisMode.value;
    els.shuttleAxisDeg.disabled = mode !== "custom";
    if (mode === "x") els.shuttleAxisDeg.value = "0";
    else if (mode === "y") els.shuttleAxisDeg.value = "90";
  });
  els.btnShuttleStart.addEventListener("click", () => {
    const axisDeg = els.shuttleAxisMode.value === "x" ? 0
      : els.shuttleAxisMode.value === "y" ? 90
      : (parseFloat(els.shuttleAxisDeg.value) || 0);
    const amp = clamp(parseFloat(els.shuttleAmp.value) || 0,
                      trajLimits.shuttle_amplitude_min_m,
                      trajLimits.shuttle_amplitude_max_m);
    const period = clamp(parseFloat(els.shuttlePeriod.value) || 0,
                         trajLimits.period_min_s, trajLimits.period_max_s);
    const cx = clamp(parseFloat(els.shuttleCx.value) || 0,
                     -trajLimits.excursion_abs_max_m,
                     trajLimits.excursion_abs_max_m);
    const cy = clamp(parseFloat(els.shuttleCy.value) || 0,
                     -trajLimits.excursion_abs_max_m,
                     trajLimits.excursion_abs_max_m);
    const cycles = Math.max(0, Math.round(parseFloat(els.shuttleCycles.value) || 0));
    const alt = clamp(parseFloat(els.shuttleAlt.value) || UI.ALT_MIN_M,
                      UI.ALT_MIN_M, UI.ALT_MAX_M);
    sendCommand("shuttle_start", {
      center_x: cx, center_y: cy, axis_deg: axisDeg, amplitude_m: amp,
      period_s: period, cycles: cycles, alt_m: alt,
    });
    appendConsole("ui",
      `往復軌道開始要求: 中心(${cx.toFixed(2)}, ${cy.toFixed(2)}) 軸${axisDeg.toFixed(0)}° ` +
      `A=${amp.toFixed(2)}m 周期${period.toFixed(0)}s ` +
      `${cycles === 0 ? "連続" : cycles + "サイクル"} 高度${alt.toFixed(2)}m`);
  });
  els.btnShuttleStop.addEventListener("click", () => {
    sendCommand("shuttle_stop");
    appendConsole("ui", "往復軌道停止要求(現在目標でホバ復帰)");
  });
  // 評価シーケンス: プリセット選択+開始インデックス+開始/停止
  els.seqPresetSelect.addEventListener("change", renderSeqList);
  els.seqStartIndex.addEventListener("change", renderSeqList);
  els.btnTrajSeqStart.addEventListener("click", () => {
    const name = els.seqPresetSelect.value;
    if (!name) return;
    const alt = clamp(parseFloat(els.seqAlt.value) || UI.ALT_MIN_M,
                      UI.ALT_MIN_M, UI.ALT_MAX_M);
    const startIndex = Math.max(0, parseInt(els.seqStartIndex.value, 10) || 0);
    sendCommand("traj_sequence_start",
                { name: name, alt_m: alt, start_index: startIndex });
    appendConsole("ui",
      `評価シーケンス開始要求: ${name} セグメント${startIndex + 1}から ` +
      `高度${alt.toFixed(2)}m`);
  });
  els.btnTrajSeqStop.addEventListener("click", () => {
    sendCommand("traj_sequence_stop");
    appendConsole("ui", "評価シーケンス停止要求(現在目標でホバ復帰)");
  });

  // ログ保存トグル
  els.logToggle.addEventListener("change", () => {
    logToggleSentAt = now();
    sendCommand("set_logging", { enabled: els.logToggle.checked });
  });

  // ---- v2: Experiment タブ ----
  els.btnExpActivate.addEventListener("click", () => {
    sendCommand("experiment_activate");
    appendConsole("ui", "実験モード有効化を要求しました(CMD_MODE)");
  });
  els.fixtureCheck.addEventListener("change", updateExperimentControls);
  els.highDutyCheck.addEventListener("change", () => {
    if (!els.highDutyCheck.checked && selectedDuty >= UI.DUTY_HIGH_MIN) {
      selectedDuty = UI.DUTY_DEFAULT;   // 高出力許可を外したら安全側に戻す
    }
    updateExperimentControls();
  });
  els.dutyButtons.addEventListener("click", (ev) => {
    const btn = ev.target.closest(".duty-btn");
    if (!btn || btn.disabled) return;
    selectedDuty = parseFloat(btn.dataset.duty);
    updateExperimentControls();
  });
  els.btnMotorStart.addEventListener("click", () => {
    sendCommand("motor_start", { duty: selectedDuty, mask: 0x0F });
    appendConsole("ui", `モーター開始要求: duty=${selectedDuty.toFixed(1)}(全モーター)`);
  });
  els.btnMotorApply.addEventListener("click", () => {
    sendCommand("motor_set", { duty: selectedDuty });
    appendConsole("ui", `duty 変更要求: ${selectedDuty.toFixed(1)}`);
  });
  els.btnMotorStop.addEventListener("click", () => {
    sendCommand("motor_stop");
    appendConsole("ui", "モーター停止要求");
  });

  // 計測(EKF/FF性能ログ)開始/停止(結果はサーバの info/警告ログで通知される)
  els.btnExpRecStart.addEventListener("click", () => {
    sendCommand("exp_record_start");
    appendConsole("ui", "計測開始を要求しました(EKF/FF性能ログ)");
  });
  els.btnExpRecStop.addEventListener("click", () => {
    sendCommand("exp_record_stop");
    appendConsole("ui", "計測停止を要求しました");
  });

  // リアルタイムモニタ: 折りたたみ / 積算リセット / タブ復帰時の再描画
  // (非表示中は rAF が発火しないため、可視化された瞬間に1回描き直す)
  els.btnRtmonToggle.addEventListener("click", rtmonToggle);
  els.btnRtmonReset.addEventListener("click", rtmonReset);
  document.addEventListener("visibilitychange", () => {
    if (!document.hidden) rtmonRequestDraw();
  });

  // スイープ
  els.btnSweepStart.addEventListener("click", () => withBusy(els.btnSweepStart, async () => {
    const resp = await apiPost("/api/sweep", {
      action: "start", mask: selectedSweepMask(),
      pattern: selectedSweepPattern(), notes: sweepNotes(),
    });
    if (resp && resp.message) els.sweepMessage.textContent = resp.message;
    if (resp && !resp.ok) appendConsole("ui", `スイープ開始不可: ${resp.message || ""}`);
  }));
  els.btnSweepAbort.addEventListener("click", () => withBusy(els.btnSweepAbort, async () => {
    await apiPost("/api/sweep", { action: "abort" });
    appendConsole("ui", "スイープ中断要求");
  }));

  // 加算性シーケンス
  els.btnSeqStart.addEventListener("click", () => withBusy(els.btnSeqStart, async () => {
    const resp = await apiPost("/api/sequence", {
      action: "start", pattern: selectedSweepPattern(), notes: sweepNotes(),
    });
    if (resp && resp.message) els.seqMessage.textContent = resp.message;
    if (resp && !resp.ok) appendConsole("ui", `シーケンス開始不可: ${resp.message || ""}`);
  }));
  els.btnSeqResume.addEventListener("click", () => withBusy(els.btnSeqResume, async () => {
    const resp = await apiPost("/api/sequence", { action: "resume" });
    if (resp && resp.message) els.seqMessage.textContent = resp.message;
  }));
  els.btnSeqForce.addEventListener("click", () => withBusy(els.btnSeqForce, async () => {
    if (!window.confirm("電圧がしきい値未満でも次のスイープを開始します。よろしいですか?")) return;
    const resp = await apiPost("/api/sequence", { action: "resume", force: true });
    if (resp && resp.message) els.seqMessage.textContent = resp.message;
  }));
  els.btnSeqAbort.addEventListener("click", () => withBusy(els.btnSeqAbort, async () => {
    await apiPost("/api/sequence", { action: "abort" });
    appendConsole("ui", "シーケンス中断要求");
  }));

  // 3D磁気キャリブレーション
  for (const btn of document.querySelectorAll("[data-cal3d-action]")) {
    btn.addEventListener("click", () => withBusy(btn, async () => {
      const action = btn.dataset.cal3dAction;
      if (action === "apply" &&
          !window.confirm("3D磁気キャリブレーションを機体へ適用します。\n機体側で FF は自動無効化されます(再適用が必要)。よろしいですか?")) {
        return;
      }
      if (action === "clear" &&
          !window.confirm("機体の 3D磁気キャリブレーションをクリアします。よろしいですか?")) {
        return;
      }
      const resp = await apiPost("/api/cal3d", { action });
      if (resp) {
        cal3dStatus = resp;
        renderCal3d();
        if (resp.error) appendConsole("ui", `3D磁気: ${resp.error}`);
        else if (action === "apply" && resp.ok) fetchFfStatus(); // FF自動無効の反映
      }
    }));
  }

  // 加速度6面
  for (const btn of document.querySelectorAll("[data-accel6-action]")) {
    btn.addEventListener("click", () => withBusy(btn, async () => {
      const action = btn.dataset.accel6Action;
      const body = { action };
      if (action === "capture") body.face = btn.dataset.face;
      const resp = await apiPost("/api/accel6", body);
      if (resp) {
        accel6Status = resp;
        renderAccel6();
        if (resp.message) els.accel6Msg.textContent = resp.message;
      }
    }));
  }

  // クイック較正(Attitude 0 / Yaw 0 / Clear。全モード共通カード)
  // ヨーゼロは FF 停止→設定→FF 復元→アンカー再取得の多段シーケンスで数秒かかるため、
  // 実行中は 4 ボタンまとめて無効化する(同時操作・二度押し防止)。
  // Multi モード中は対象機体セレクタの機体名を "drone" として送る(サーバ側で必須検証)
  const quickcalBtns = document.querySelectorAll("[data-quickcal-action]");
  for (const btn of quickcalBtns) {
    btn.addEventListener("click", async () => {
      const body = { action: btn.dataset.quickcalAction };
      if (uiMode === "multi") {
        const drone = els.quickcalDrone.value;
        if (!drone) {
          els.quickcalMsg.textContent =
            "対象機体がありません(複数機タブで「選択適用」してから実行してください)";
          return;
        }
        body.drone = drone;
      }
      for (const b of quickcalBtns) b.disabled = true;
      try {
        const resp = await apiPost("/api/quickcal", body);
        if (resp && resp.message) {
          els.quickcalMsg.textContent = resp.message;
          appendConsole("ui", resp.message);
        }
      } finally {
        for (const b of quickcalBtns) b.disabled = false;
        updateExperimentControls();
      }
    });
  }

  // 地磁気(都道府県)
  els.geomagSelect.addEventListener("change", () => withBusy(els.btnGeomagApply, async () => {
    const resp = await apiPost("/api/geomag", { action: "select", id: els.geomagSelect.value });
    if (resp) {
      geomagStatus = resp;
      renderGeomag();
      if (resp.message) els.geomagMsg.textContent = resp.message;
    }
  }));
  els.btnGeomagApply.addEventListener("click", () => withBusy(els.btnGeomagApply, async () => {
    const resp = await apiPost("/api/geomag", { action: "apply" });
    if (resp) {
      geomagStatus = resp;
      renderGeomag();
      if (resp.message) els.geomagMsg.textContent = resp.message;
    }
  }));

  // キャリブレーション・プロファイル
  els.btnCalprofSave.addEventListener("click", () => withBusy(els.btnCalprofSave, async () => {
    const resp = await apiPost("/api/calprofile",
                               { action: "save", name: els.calprofName.value });
    if (resp) {
      calprofStatus = resp;
      renderCalprof();
    }
  }));
  els.btnCalprofApply.addEventListener("click", () => withBusy(els.btnCalprofApply, async () => {
    const name = els.calprofSelect.value;
    if (!name) return;
    if (!window.confirm(`プロファイル「${name}」を機体へ適用します(NVS書込+読み戻し照合)。よろしいですか?`)) return;
    const resp = await apiPost("/api/calprofile", { action: "apply", name });
    if (resp) {
      calprofStatus = resp;
      renderCalprof();
      if (Array.isArray(resp.mismatches) && resp.mismatches.length) {
        appendConsole("ui", `照合不一致: ${resp.mismatches.slice(0, 6).join(", ")}`);
      }
    }
  }));
  els.btnCalprofDelete.addEventListener("click", () => withBusy(els.btnCalprofDelete, async () => {
    const name = els.calprofSelect.value;
    if (!name) return;
    if (!window.confirm(`プロファイル「${name}」を削除します。よろしいですか?`)) return;
    const resp = await apiPost("/api/calprofile", { action: "delete", name });
    if (resp) {
      calprofStatus = resp;
      renderCalprof();
    }
  }));

  // FF プロファイル(抽出・適用・モード・アンカー・削除)
  els.btnFfExtract.addEventListener("click", () => withBusy(els.btnFfExtract, async () => {
    const folder = els.ffFolderSelect.value;
    if (!folder) {
      els.ffExtractResult.textContent = "抽出元フォルダを選択してください";
      return;
    }
    els.ffExtractResult.textContent = "抽出中…(最大2分)";
    const resp = await apiPost("/api/ffprofile", {
      action: "extract", folder,
      name: els.ffExtractName.value || null,
      memo: els.ffExtractMemo.value || null,
    });
    if (resp) {
      els.ffExtractResult.textContent = resp.ok
        ? `抽出完了: ${resp.name}` +
          (resp.warnings && resp.warnings.length ? `(警告${resp.warnings.length}件)` : "")
        : `抽出失敗: ${resp.message || "不明なエラー"}`;
      if (resp.warnings && resp.warnings.length) {
        appendConsole("ui", `FF抽出警告: ${resp.warnings.slice(0, 3).join(" / ")}`);
      }
      setFfStatus(resp);
    }
  }));
  els.btnFfApply.addEventListener("click", () => withBusy(els.btnFfApply, () =>
    doFfApply(els.ffProfileSelect.value,
              parseInt(els.ffModeSelect.value, 10),
              parseInt(els.ffEstSelect.value, 10))));
  els.btnFfMode.addEventListener("click", () => withBusy(els.btnFfMode, async () => {
    const resp = await apiPost("/api/ffprofile", {
      action: "mode",
      ff: parseInt(els.ffModeSelect.value, 10),
      est: parseInt(els.ffEstSelect.value, 10),
    });
    if (resp) {
      if (resp.message) els.ffApplyMsg.textContent = resp.message;
      setFfStatus(resp);
    }
  }));
  els.btnFfAnchor.addEventListener("click", () => withBusy(els.btnFfAnchor, async () => {
    const resp = await apiPost("/api/ffprofile", { action: "anchor" });
    if (resp && resp.message) els.ffApplyMsg.textContent = resp.message;
  }));
  els.btnFfDelete.addEventListener("click", () => withBusy(els.btnFfDelete, async () => {
    const name = els.ffProfileSelect.value;
    if (!name) return;
    if (!window.confirm(`FFプロファイル「${name}」を削除します。よろしいですか?`)) return;
    const resp = await apiPost("/api/ffprofile", { action: "delete", name });
    if (resp) setFfStatus(resp);
  }));

  // 磁気オートチューン(EKF2)パネル
  els.yawRefSelect.addEventListener("change", async () => {
    yawRefSentAt = now();
    const source = els.yawRefSelect.value;
    const resp = await apiPost("/api/yawref", { source });
    appendConsole("ui", resp && resp.ok
      ? `ヨー基準ソース: ${source}`
      : `ヨー基準ソース切替失敗: ${(resp && resp.message) || "不明なエラー"}`);
  });
  els.btnMagbiasExtract.addEventListener("click", () =>
    withBusy(els.btnMagbiasExtract, async () => {
      const log = els.magbiasLogSelect.value;
      if (!log) {
        els.magbiasMsg.textContent = "抽出元ログを選択してください";
        return;
      }
      els.magbiasMsg.textContent = "抽出中…(最大2分)";
      const resp = await apiPost("/api/magbias", { action: "extract", log });
      if (resp) {
        els.magbiasMsg.textContent = resp.ok
          ? `抽出完了: ${resp.name}` +
            (resp.warnings && resp.warnings.length
              ? `(警告: ${resp.warnings.join(" / ")})` : "")
          : `抽出失敗: ${resp.message || "不明なエラー"}`;
        setMagbiasStatus(resp);
      }
    }));
  els.btnMagbiasApply.addEventListener("click", () =>
    withBusy(els.btnMagbiasApply, async () => {
      const name = els.magbiasSelect.value;
      if (!name) {
        appendConsole("ui", "magbias プロファイルが選択されていません");
        return;
      }
      let resp = await apiPost("/api/magbias", { action: "apply", name });
      if (resp && !resp.ok && resp.binding_mismatch
          && Array.isArray(resp.diffs)) {
        const detail = resp.diffs.slice(0, 4).join("\n");
        if (window.confirm(`機体の ffcal がプロファイル取得時と一致しません:\n${detail}\n\n強制適用しますか?(Δb の意味が変わっている可能性があります)`)) {
          resp = await apiPost("/api/magbias",
                               { action: "apply", name, force: true });
        }
      }
      if (resp) {
        appendConsole("ui", resp.ok
          ? `magbias を適用しました: ${name}`
          : `magbias 適用失敗: ${resp.message || "不明なエラー"}`);
        setMagbiasStatus(resp);
      }
    }));
  els.btnMagbiasClear.addEventListener("click", () =>
    withBusy(els.btnMagbiasClear, async () => {
      const resp = await apiPost("/api/magbias", { action: "clear" });
      if (resp) {
        appendConsole("ui", resp.ok
          ? "magbias をクリアしました"
          : `magbias クリア失敗: ${resp.message || "不明なエラー"}`);
        setMagbiasStatus(resp);
      }
    }));

  // フロー較正(純回転フィット)。withBusy は使わない — ボタン活性は
  // renderFlowcal が collecting/busy/fit から一元的に決める(finally の
  // 一律再有効化が「停止ボタンは記録中のみ」等の状態則を壊すため)
  els.btnFlowcalStart.addEventListener("click", async () => {
    els.btnFlowcalStart.disabled = true;
    const resp = await apiPost("/api/flowcal", { action: "start_record" });
    if (resp) {
      appendConsole("ui", resp.ok
        ? "フロー較正の記録を開始しました(20〜30秒ゆらしてください)"
        : `フロー較正 記録開始失敗: ${resp.message || "不明なエラー"}`);
      setFlowcalStatus(resp);
    }
    renderFlowcal();
    if (!flowcalStatus) els.btnFlowcalStart.disabled = false;
  });
  els.btnFlowcalStop.addEventListener("click", async () => {
    els.btnFlowcalStop.disabled = true;
    const resp = await apiPost("/api/flowcal", { action: "stop_and_fit" });
    if (resp) {
      appendConsole("ui", resp.ok
        ? "フロー較正フィット: 合格(適用できます)"
        : `フロー較正フィット: ${resp.message || "不明なエラー"}`);
      setFlowcalStatus(resp);
    }
    renderFlowcal();
    if (!flowcalStatus) els.btnFlowcalStop.disabled = false;
  });
  els.btnFlowcalApply.addEventListener("click", async () => {
    els.btnFlowcalApply.disabled = true;
    let resp = await apiPost("/api/flowcal", { action: "apply" });
    if (resp && !resp.ok && resp.quality_warning
        && Array.isArray(resp.warnings)) {
      const detail = resp.warnings.slice(0, 5).join("\n");
      if (window.confirm(`フィットが受入基準を満たしていません:\n${detail}\n\n強制適用しますか?(推奨: 収集をやり直す)`)) {
        resp = await apiPost("/api/flowcal", { action: "apply", force: true });
      }
    }
    if (resp) {
      appendConsole("ui", resp.ok
        ? "flowcal を適用しました(CAL_GET 読み戻し照合OK)"
        : `flowcal 適用失敗: ${resp.message || "不明なエラー"}`);
      setFlowcalStatus(resp);
    }
    renderFlowcal();
    if (!flowcalStatus) els.btnFlowcalApply.disabled = false;
  });
  els.btnFlowcalClear.addEventListener("click", async () => {
    els.btnFlowcalClear.disabled = true;
    const resp = await apiPost("/api/flowcal", { action: "clear" });
    if (resp) {
      appendConsole("ui", resp.ok
        ? "フロー較正の記録・フィット結果をクリアしました(機体 NVS は不変)"
        : `フロー較正 クリア失敗: ${resp.message || "不明なエラー"}`);
      setFlowcalStatus(resp);
    }
    els.btnFlowcalClear.disabled = false;
    renderFlowcal();
  });
  // 保存済みプロファイルの選択適用/削除(magbias パネルと同様式)
  els.flowcalSelect.addEventListener("change", renderFlowcal);
  els.btnFlowcalProfileApply.addEventListener("click", async () => {
    const name = els.flowcalSelect.value;
    if (!name) {
      appendConsole("ui", "flowcal プロファイルが選択されていません");
      return;
    }
    els.btnFlowcalProfileApply.disabled = true;
    const resp = await apiPost("/api/flowcal", { action: "apply", name });
    if (resp) {
      appendConsole("ui", resp.ok
        ? `flowcal プロファイルを適用しました: ${name}(読み戻し照合OK)`
        : `flowcal プロファイル適用失敗: ${resp.message || "不明なエラー"}`);
      setFlowcalStatus(resp);
    }
    renderFlowcal();
    if (!flowcalStatus) els.btnFlowcalProfileApply.disabled = false;
  });
  els.btnFlowcalProfileDelete.addEventListener("click", async () => {
    const name = els.flowcalSelect.value;
    if (!name) {
      appendConsole("ui", "flowcal プロファイルが選択されていません");
      return;
    }
    if (!window.confirm(
      `flowcal プロファイル「${name}」を削除しますか?\n` +
      "(機体 NVS と適用状態は変更されません)")) return;
    els.btnFlowcalProfileDelete.disabled = true;
    const resp = await apiPost("/api/flowcal", { action: "delete", name });
    if (resp) {
      appendConsole("ui", resp.ok
        ? `flowcal プロファイルを削除しました: ${name}`
        : `flowcal プロファイル削除失敗: ${resp.message || "不明なエラー"}`);
      setFlowcalStatus(resp);
    }
    renderFlowcal();
    if (!flowcalStatus) els.btnFlowcalProfileDelete.disabled = false;
  });

  // SPACE = どこからでも緊急STOP(Experiment 中はモーター停止も送出)
  // 例外: テキスト入力(プロファイル名・メモ等)へのフォーカス中のみ通常入力を
  // 許す(空白を打てるようにする)。数値入力・スライダ・ボタンでは従来どおり
  // 即STOP。プロファイル編集モーダル内の入力も従来どおり除外。
  document.addEventListener("keydown", (ev) => {
    if (ev.code === "Space" && !ev.repeat) {
      if (els.afEditor.classList.contains("visible") &&
          ev.target instanceof HTMLInputElement) {
        return;
      }
      if (ev.target instanceof HTMLInputElement && ev.target.type === "text") {
        return;
      }
      if (ev.target instanceof HTMLTextAreaElement) {
        return;
      }
      ev.preventDefault(); // ボタンのSpace押下/スクロールを抑止し必ずSTOPにする
      doStop();
    }
  });
}

/* ===================== 起動 ===================== */
function init() {
  wireEvents();
  fetchPorts();
  fetchAirframes();
  renderSeqPresets();   // /api/config 取得前・失敗時も空状態メッセージを表示
  fetchConfigLimits();
  fetchFfStatus();
  // FF 適用状態(適用中バナー)は低頻度ポーリングで同期する
  setInterval(fetchFfStatus, UI.FF_POLL_MS);
  applyMode("posture", false);
  els.overlay.classList.add("visible"); // 接続成功までオーバーレイ表示
  wsConnect();
  drawPlot();
  updateExperimentControls();
}

init();
