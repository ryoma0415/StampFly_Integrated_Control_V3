# StampFly Integrated Control — アーキテクチャ仕様 v2

本書は各コンポーネントの責務・境界・API・コーディング規約を定める契約文書。
通信のワイヤ仕様は `PROTOCOL.md` が正。FF 較正パイプラインの詳細は
`FF_PIPELINE.md`、ログ列は `LOG_STRUCTURE.md`。

## 中心原則: ファームウェアは1本、モードはPC側のみ

機体は常に「roll/pitch角度 + 目標高度(+ v2: ヨー角)のセットポイント追従機」
として動作する。
- **Postureモード** = UIのスライダ値 → そのまま CMD_SETPOINT
- **Positionモード** = NatNet位置 → フィルタ → 位置誤差 → CMD_POS_ERR
  (**機上XY制御**: 機体側が自身のヨー推定で誤差をヨー回転補償し XY PID で
  roll/pitch 指令を計算する。PC は角度指令を送らない。
  v2: 円軌道モードは 50Hz 送信ループ内で目標 XY を時間更新する)
- **Experimentモード**(v2) = ベンチ実験専用。機体を CMD_MODE で MOTOR_TEST
  状態に遷移させ、モーターテスト/スイープ/キャリブレーションを行う
  (飛行制御 PID・ミキサは動かない。§実験モードの状態遷移)
- **Multiモード**(v2) = 2〜4機の同時位置制御。リレーの多重化拡張
  (RLY_SET_PEERS + RLY_MUX_UP/DOWN)で1本のリレーを共有し、機体ごとの
  PositionController が 50Hz CMD_POS_ERR を送る(§複数機モード)

モード切替にファーム書き換え・再起動は不要。機体差(角度バイアス)は**PC側の機体
プロファイルで指令に加算**し、ファームに機体固有定数を置かない。
機体固有の較正(3D磁気・加速度6面・FF 係数等)は機体 NVS に永続化し、
PC 側にはそのスナップショット(プロファイル JSON)を置く(§NVS 永続化一覧)。

v2 で追加された機能ブロック:
- **ヨー推定**(`firmware_stampfly/src/yaw_estimation/`): BMM150 磁気 +
  INA3221 電流 + モーター電流FF補正(ΔB̂)+ 4状態EKF / 補正CF。
  yaw側プロジェクト(旧 `Yaw_Estimation_Project/Yaw_Calibration_and_Estimation`。
  2026-07-27 に本ツリーから削除、同内容は `../StampFly_Telemetry/Telemetry/` に現存)
  からの移植で、数式・符号・定数値は不変(定数は `yaw_config.hpp` に集約)。
- **ヨー角制御**: psi_pid(角度誤差 wrapPi → ヨーレート目標)。
  CMD_SETPOINT flags bit1 が無効なら V1 と同一のレートダンピングのみ。
- **機上XY制御**(CMD_POS_ERR 0x24): PC は位置誤差ストリームのみを送り、
  機体側がヨー回転補償+XY PID(`config.hpp` の `pos_*` ゲイン)で
  roll/pitch 指令を計算する。v2.2 で唯一の XY 経路になった(旧 PC 側
  XY PID 経路=`xy_command_mode:"pc"` と `pc_server/core/pid.py` は削除)。
- **制御診断テレメトリ TLM_CTRL**(0x35、v2.2): 姿勢制御 2 ループ
  (角度・角速度)の PID 成分 18 個+指令角速度 3 個+flags を 25Hz で
  常時送出し、CSV ログ v4 に記録する(PROTOCOL.md / LOG_STRUCTURE.md)。
- **実験機能**(`pc_server/core/experiment.py` ほか): モーターテスト・
  電流×磁場スイープ・加算性シーケンス・各種キャリブレーション・FF
  プロファイル抽出/適用。クイック較正(Attitude 0 / Yaw 0)は v2.2 で
  全モード共通機能になった(UI 共通モニタ列のカード+`/api/quickcal`)。
- **flight_log_viewer**: 飛行後の CSV ログ(v4・109列)可視化・ヨー比較・
  EKF 診断・PID 成分/角速度追従・同期アニメーション。
- **複数機同時制御**(`pc_server/core/multi.py` + firmware_relay 多重化拡張):
  2〜4機の MoCap 位置制御を1本のリレーで同時に行う(機体ファーム無改修。
  §複数機モード)。

## フォルダ構成

```
StampFly_Integrated_Control_V2/
├── docs/                  PROTOCOL.md, ARCHITECTURE.md, OPERATION_GUIDE.md,
│                          LOG_STRUCTURE.md, FF_PIPELINE.md, README.md(索引)
├── protocol/              プロトコル単一真実の実装(PROTOCOL_VERSION=0x02)
│   ├── stampfly_protocol.hpp   ヘッダオンリーC++。Arduino非依存(純粋C++17)。
│   │                           COBS, CRC16, フレームpack/parse, 全enum/struct
│   ├── stampfly_protocol.py    同等のPython実装
│   ├── test_vectors.json       共有バイトベクタ(v2 全メッセージ型を収録)
│   └── tests/                  pytest(Python往復+ベクタ) と host_test.cpp(g++でコンパイル
│                               しベクタを検証、pytestから subprocess で実行)
├── firmware_stampfly/     機体ファーム(PlatformIO, board: esp32-s3-devkitc-1)
│   ├── platformio.ini     release(-O2, CORE_DEBUG_LEVEL=0) / debug(-Og -g3) の2env。
│   │                      build_flags に -I../protocol
│   ├── src/main.cpp       製品版同様 ~40行: setup()→init_copter(), loop()→loop_400Hz()
│   ├── src/config.hpp     FlightConfig: 全マジックナンバーをここに集約(ゲイン, クランプ,
│   │                      タイムアウト, チャネル, レート分周比。v2: yaw_angle ゲイン,
│   │                      yaw_rate_limit_rad_s, モーターテスト定数)
│   ├── src/comm.{hpp,cpp} ESP-NOW送受信。受信cb=検証+メールボックス格納のみ。
│   │                      TXはキュー+送信関数。リレーMAC学習。チャネルピン留め
│   ├── src/flight_control.{hpp,cpp}  状態機械+カスケードPID+ミキサ(OptiTrack版を基に
│   │                      整理)。v2: psi_pid(ヨー角制御)・MOTOR_TEST 状態と
│   │                      モーターテストサービス・0x14–0x23 コマンド処理+TLM_ACK。
│   │                      v2.2: 機上XY制御(CMD_POS_ERR: ヨー回転補償+XY PID)、
│   │                      TLM_CTRL 用の PID 成分転記(update_control_diag)
│   ├── src/telemetry.{hpp,cpp}  TLM_STATE 25Hz(400Hzの16分周)+TLM_EVENT生成。
│   │                      v2: TLM_EXP 25Hz(MOTOR_TEST のみ・8tick位相ずらし)+TLM_CAL_DATA。
│   │                      v2.2: TLM_CTRL 25Hz(常時・TLM_STATE から 4tick 位相ずらし)
│   ├── src/indicators.{hpp,cpp} LED状態表示+非ブロッキングブザー
│   ├── src/{sensor,imu,tof,alt_kalman,pid}.{hpp,cpp}  OptiTrack版から流用(飛行実績層)。
│   │                      sensor.cpp に v2 のセンサースケジューリング統合
│   │                      (§I2C スケジューリング)と Yaw_gyro_integral(400Hz積算)
│   ├── src/yaw_estimation/   ヨー推定モジュール(yaw側から移植。UDP/JSON依存を除去)
│   │   ├── yaw_config.hpp        FF/EKF/MAG 系定数の集約(数値は yaw側と完全同一)
│   │   ├── bmm150_driver.{hpp,cpp}   BMM150 磁気センサ(RHALL補償・軸変換)
│   │   ├── current_sensor.{hpp,cpp}  INA3221(CH2のみ・140µs変換・AVG16・20Hz)
│   │   ├── mag_calibration.{hpp,cpp} 3D磁気較正(offset+matrix)の保持・適用
│   │   ├── accel_calibration.{hpp,cpp} 加速度6面較正
│   │   ├── ff_calibration.{hpp,cpp}  FF係数(LUT/モーター係数)保持・ステージング・ΔB̂計算
│   │   ├── yaw_estimator.{hpp,cpp}   相補フィルタ(リファレンスCF/補正CF)
│   │   ├── yaw_estimator_kf.{hpp,cpp} 4状態EKF(ψ, b_g, b_mx, b_my)
│   │   ├── sensor_hub_ff.{hpp,cpp}   FF補正挿入点+アイドルアンカー+NVS復元
│   │   ├── angle_utils.hpp           wrapPi / levelMagVector の共通ヘッダ(一本化)
│   │   └── persistence.{hpp,cpp}     NVS 永続化(§NVS 永続化一覧)
│   └── lib/               bmi270, vl53l3c, MdgwickAHRS を OptiTrack版からコピー
├── firmware_relay/        リレーファーム(PlatformIO, board: esp32dev)。v2 でマルチ
│   │                      機体対応(RLY_SET_PEERS / RLY_MUX 多重化、UART 460800 化)
│   ├── platformio.ini     -I../protocol。release / release-115200(460800 非対応
│   │                      ブリッジ用フォールバック)/ debug の 3env
│   └── src/
│       ├── main.cpp       初期化+ループ(小さく)
│       ├── uart_link.{hpp,cpp}   RX: 0x00区切り蓄積→protocolヘッダのデコーダ呼び出し。
│       │                  TX: FreeRTOSキュー+専用書き出しタスク(唯一のSerialライタ)
│       ├── espnow_link.{hpp,cpp} ピア管理(単機 SET_TARGET / マルチ SET_PEERS 最大4機
│       │                  の排他2モード)、受信cb=送信元MAC→node_id帰属+キュー投入のみ
│       └── router.{hpp,cpp}      型レンジで転送、RLY_*処理、統計、LOG_TEXT発行。
│                          v2: RLY_MUX_UP 展開(内側検証→ピア送信)/下り RLY_MUX_DOWN
│                          包装、node別 TLM_STATE 間引き、マルチ中の非エンベロープ上り拒否
├── pc_server/
│   ├── app.py             FastAPI: 静的UI配信 + WebSocket + REST。uvicornで起動
│   ├── core/
│   │   ├── serial_link.py 読み取りスレッド、COBSデコード、型別ディスパッチ、書き込みロック、
│   │   │                  RLY_TARGET_ACK待ち合わせ、統計。v2: send_with_ack
│   │   │                  (TLM_ACK 待ち 1.0s×最大2回再送)。multi: send_to/
│   │   │                  send_setpoint_to/send_pos_err_to(RLY_MUX_UP 包装)、register_node_handler
│   │   │                  (MUX_DOWN 内側フレームの node 付きディスパッチ)、
│   │   │                  set_relay_peers、node別レイテンシ(node_latency_ms)
│   │   ├── session.py     SessionManager: 接続/モード(posture/position/experiment/
│   │   │                  multi)/Start/Stop/Reset/ログON-OFF/機体プロファイル適用を
│   │   │                  一元管理。v2: ヨー指令・円軌道・実験モード遷移・
│   │   │                  SPACE緊急停止優先経路
│   │   ├── posture.py     PostureController: UI setpoint→スルーレート制限→50Hz送信。
│   │   │                  SetpointShaper(v2: shape_yaw = 最短経路wrap+45°/s)を定義
│   │   ├── position.py    PositionController: mocap→フィルタ→位置誤差→CMD_POS_ERR
│   │   │                  50Hz送信(機上XY制御。XY PID は機体側 — roll/pitch は
│   │   │                  この層では生成しない)。
│   │   │                  v2: 円軌道(位相合流・MoCap途絶中の位相凍結・接線ヨー)。
│   │   │                  START時の reset_filter / data_valid ゲート /
│   │   │                  データ無効持続監視の基準(data_invalid_age_s)
│   │   ├── multi.py       v2: MultiControlManager+DroneSlot(2〜4機の同時位置制御。
│   │   │                  §複数機モード)
│   │   ├── mocap.py       NatNet接続(vendor/のSDK)、座標変換、PositionFilter。
│   │   │                  multi: 全リジッドボディ配信(subscribe(rigid_body_id, cb)/
│   │   │                  bodies_snapshot() / パッシブ start)
│   │   ├── filter.py      既存position_filter.pyを移植・整理。2026-07:
│   │   │                  tracking_valid=0 は欠測扱い(アンカー非更新)+
│   │   │                  連続外れ値/予測経過での強制再シード(ロックアウト対策。
│   │   │                  再シード後 N フレームは検疫 = probation で閉ループ抑止)
│   │   ├── experiment.py  v2: ExperimentHub(TLM_EXP配信・モーターテスト・排他スロット)
│   │   │                  + SweepRunner(電流×磁場スイープ)+ SequenceRunner(加算性)
│   │   ├── calibration.py v2: CalibrationManager(3D磁気fit・加速度6面・Attitude0/Yaw0/
│   │   │                  地磁気47都道府県・キャリブプロファイル保存/適用/照合)。
│   │   │                  v2.2: Attitude0/Yaw0 は姿勢ソースを TLM_STATE 化して
│   │   │                  全モード対応+node_id 指定で Multi の機体宛に対応
│   │   ├── ffprofile.py   v2: FfProfileManager(FF抽出サブプロセス・CMD_FF_* 分割転送・
│   │   │                  CRC照合・ff_state.json)
│   │   └── logger.py      CSVログ(START〜飛行終了)。logs/flight_logs/
│   │                      <日時>_<mode>.csv(Multi は機体ごと <日時>_multi_<機体名>.csv)。
│   │                      50Hz制御行+最新テレメトリ(TLM_STATE+TLM_CTRL)/
│   │                      mocapスナップショット結合。
│   │                      列定義は docs/LOG_STRUCTURE.md と1対1(v4: 109列)
│   ├── config/
│   │   ├── server.json    ポート既定値, serial(baudrate 既定 460800), レート,
│   │   │                  クランプ(v2: max_yaw_deg, yaw_slew_rate_deg_per_s),
│   │   │                  フェイルセーフ閾値, experiment 節, multi 節(§複数機モード)
│   │   ├── airframes.json 機体プロファイル配列: {name, mac, wifi_channel,
│   │   │                  roll_bias_deg, pitch_bias_deg, default_alt_m,
│   │   │                  rigid_body_id(任意。複数機モードで必須), notes}
│   │   ├── control.json   フィルタ設定, 座標変換, natnet 節, control 節
│   │   │                  (pos_err_clamp_m ほか), trajectory 節(v2: 円軌道の制限値)。
│   │   │                  旧 "pid" 節と "xy_command_mode" は v2.2 で削除
│   │   │                  (XY ゲインはファーム config.hpp の pos_* が実体)
│   │   ├── geomagnetic_profiles.json  v2: 47都道府県の地磁気プロファイル+selected
│   │   └── mag3d_calibration.json     v2: 3D磁気較正のPC側スナップショット(生成物。
│   │                      スイープCSVの b*_cor 計算に使用)
│   ├── data/              v2: 生成データの集約先(契約 §6)
│   │   ├── sweep_results/         スイープ CSV+meta(サブフォルダで8本1セット管理可)
│   │   ├── ff_profiles/           FFプロファイル JSON(stampfly_ff_profile v1)
│   │   ├── calibration_profiles/  キャリブプロファイル JSON(stampfly_calibration_profile v1)
│   │   ├── yaw_eval_results/      旧形式 yawlog 置き場(レガシー。専用解析スクリプトは削除済み)
│   │   └── ff_state.json          FF適用状態(適用中プロファイル名・crc・ff/est)
│   ├── static/            index.html, app.js, style.css(ビルド不要のvanilla JS。CDN禁止)
│   ├── vendor/            NatNetClient.py, MoCapData.py, DataDescriptions.py(既存流用)
│   ├── tools/             make_geomag_profiles.py(geomagnetic_profiles.json 生成)
│   ├── tests/             pytest(フェイク serial/NatNet でロジック検証。v2: 実験/
│   │                      キャリブ/FF/円軌道/ヨー整形/ログ列のテストを含む)
│   └── requirements.txt   fastapi, uvicorn[standard], pyserial, numpy, pytest(バージョン固定)
├── data_analysis/         v2: FF係数抽出・スイープ/実験ログ解析(独立 venv)
│   ├── ff_params/core.py  純粋関数: スイープ8ラン → FFプロファイル dict(忠実移植。
│   │                      sequence meta → 単機ラン展開も担当)
│   ├── make_ff_profile.py 抽出CLI+対話モード(既定入力 ../pc_server/data/sweep_results、
│   │                      既定出力 ../pc_server/data/ff_profiles)
│   ├── plot_sweep.py      スイープ校正解析+加算性シーケンス検証のグラフ化(対話式/CLI)
│   ├── plot_explog.py     Experiment 計測ログ(pc_server/data/exp_logs)のグラフ化(対話式/CLI)
│   ├── graphs/            グラフ出力先(生成物)
│   ├── tests/test_ff_extraction.py    受入テスト(6/12照合+付録A再現+sequence展開)
│   │        fixtures/results.json     受入テストの真値
│   └── requirements.txt   numpy, matplotlib
├── flight_log_viewer/     v2: 飛行ログ(logs/flight_logs/*.csv v4・109列)の可視化(独立 venv)
│   ├── visualize.py       対話式/バッチ CLI
│   ├── viewer/            constants(109列契約)/loader(旧ログ v1〜v3 互換)/
│   │                      plots/yaw_analysis/animation/report/style/jp_font
│   ├── tests/make_dummy_log.py  109列ダミーログ生成(単機+Multi グループ)
│   ├── output/            生成物(.gitignore対象)
│   └── requirements.txt   numpy, pandas, matplotlib(opencv はオプション)
└── logs/                  実行時CSV出力先(.gitignore対象)
```

## pc_server API(UI⇔サーバ契約)

単位規約: **UIとWebSocket JSONは deg / m、プロトコルとcore内部は rad / m**。
変換はsession層で行う。

### REST
- `GET /api/ports` → `[{device, description}]`(シリアルポート列挙)
- `GET /api/airframes` → airframes.json の内容
- `PUT /api/airframes` body `{"airframes":[...]}` →
  `{"ok":bool,"error":str|null,"airframes":[...]}`(UI の機体プロファイル編集。
  session.update_airframes が検証 → airframes.json へ原子的保存(temp+os.replace)
  → セッション反映。`mac` は空文字列 = 未設定を許容(未設定プロファイルは
  select_airframe 不可。MAC は `AA:BB:CC:DD:EE:FF` 形式=2桁16進オクテット
  x6 のみ受理)。飛行中は選択中プロファイルの変更/削除を拒否。
  選択中プロファイルのバイアス/default_alt 変更は非飛行時に即再適用、
  MAC/チャネル変更は次の select_airframe/connect で反映(自動再送しない)。
  選択中プロファイルが削除/MAC 未設定化されたら選択を解除し
  relay_target_ok=false(機体未選択のままの start は拒否)。
  検証の制限値は server.json の `clamps.max_roll_pitch_deg` と `airframe_limits`
  (件数上限 `max_profiles`、文字数上限 `name_max_chars`/`notes_max_chars` を含む)。
  `rigid_body_id` は任意キー(1 以上の整数 または null=未設定。複数機モードの
  select で必須))
- `GET /api/config` → server.json+control.json の実効値
- `GET /api/mocap/bodies` → `{"connected":bool, "bodies":[{rigid_body_id, x, y, z,
  yaw_rad, tracking_valid, quality, marker_count, "age_s", ...}]}`
  (観測中の全リジッドボディ一覧、ID 昇順。Multi タブの「リジッドボディ確認」が
  500ms ポーリングして rigid_body_id の紐付けを支援。NatNet 未接続なら接続を
  試みる — パッシブ起動)

### WebSocket `/ws`
- サーバ→UI(20Hz): `{"type":"state","data":{
    "drone": {TLM_STATEの全フィールド(角度はdeg換算), "fresh": bool} | null(TLM_STATE未受信),
    "mocap": {"x","y","z","yaw_deg","confidence","fresh"} | null,
    "session": {"mode":"posture"|"position"|"experiment"|"multi", "phase":"idle"|"connected"|"flying"|...,
      "serial_connected":bool, "airframe":name, "logging":bool, "log_file":str|null,
      "target":{"x","y","z"}|null, "setpoint":{"roll_deg","pitch_deg","alt_m"},
      "latency_ms":float|null, "relay_stats":{...}|null,
      "relay_fresh":bool(RLY_STATS受信時刻ベースの鮮度), "relay_target_ok":bool,
      "experiment": {"active":bool, "motor":{...}, "sweep":{...}, "sequence":{...},
        "cal3d":{...}, "exp_age_s":float|null,
        "exp": {TLM_EXP由来: "current_a","vbat_v","cv","b_raw","b_cal","imu_temp_c",
          "ax","ay","az"[g](フィルタ後・較正前。非有限はnull。6面キャリブのライブ表示),
          "roll_deg","pitch_deg","yaw_deg","duty_cmd","motors_mask",
          "mag_fresh","motors_running"} | null(TLM_EXP未受信)}
        | null(Experimentモード以外),
      "multi": {"active":bool, "drones":[{"node_id","name","mac","rigid_body_id",
        "phase":"idle"|"armed"|"flying", "target":{"x","y","z"}|null(未設定),
        "tlm":{"state","state_name","flying","low_voltage","voltage",
          "altitude_est","yaw"(deg),"fresh"}|null(TLM_STATE未受信),
        "mocap":{"x","y","z","yaw_deg","confidence","fresh"}|null,
        "latency_ms":float|null, "stop_pending":bool}]}
        | null(Multiモード以外)}}}`
- サーバ→UI(即時): `{"type":"event", ...}`(TLM_EVENT)、`{"type":"log","origin","line"}`
- UI→サーバ:
  - `{"type":"command","action":"connect","port":...}` / `"disconnect"`
  - `{"type":"command","action":"select_airframe","name":...}`(接続時にRLY_SET_TARGET)
  - `{"type":"command","action":"set_mode","mode":"posture"|"position"|"experiment"|"multi"}`(v2)
  - `{"type":"command","action":"start"}` / `"stop"` / `"reset"`
  - `{"type":"setpoint","roll_deg":..,"pitch_deg":..,"alt_m":..,"yaw_deg":..}`(Posture時。v2: yaw_deg)
  - `{"type":"yaw","yaw_deg":..}`(v2: 共通ヨースライダ)
  - `{"type":"target","x":..,"y":..,"z":..}`(Position時)
  - Multi時(v2): `{"type":"command","action":"multi_select","names":[...]}`
    (2〜4機の選択適用 → RLY_SET_PEERS)/
    `{"type":"multi_target","name":..,"x":..,"y":..,"z":..}`(機体別目標)/
    `{"type":"command","action":"multi_start"}`(一斉離陸)/
    `{"type":"command","action":"multi_yaw","name":..,"enabled":bool,
    "yaw_deg":..}`(機体別ヨー角制御 ON/OFF+目標 ±180°)。
    `stop` は全機一斉(§複数機モード)。
    `/api/ffprofile` の `apply`/`mode`/`anchor` は `"drone":name` 指定で
    機体別(ノード宛+MAC 別適用状態 `applied_by_mac`)になる
  - `{"type":"command","action":"set_logging","enabled":bool}`
  - v2 追加コマンド: `"experiment_activate"` / `"set_yaw_control"` /
    `"circle_start"`(center_x/center_y/radius_m/period_s/clockwise/alt_m/face_tangent)/
    `"circle_stop"` / `"motor_start"`(duty/mask)/ `"motor_set"`(duty)/ `"motor_stop"`
  - v2 REST(Experiment タブ): `/api/sweep` `/api/sequence` `/api/cal3d` `/api/accel6`
    `/api/geomag` `/api/calprofile` `/api/ffprofile`
  - `/api/quickcal`(v2.2: 全モード共通): `{"action": attitude_zero|attitude_clear|
    yaw_zero|yaw_clear, "drone"?: 機体名}`。Multi モード中は `"drone"` 必須
    (選択適用済み機体のノード宛に実行)、単機モードでは無視。単機は phase が
    armed/flying、Multi は対象スロットが armed/flying なら拒否(飛行中ガード)。
    既存ガード(モーター回転中・スイープ/シーケンス/計測中・磁気鮮度)は
    CalibrationManager 側で維持(experiment 外では自然に no-op)
  - 緊急停止の優先経路(v2): `stop` / `motor_stop` は受信ループの順序キューを迂回して
    即時実行される(先行する低速コマンドの ACK 待ちに巻き込まれない)。

### UI構成(static/)
ヘッダ: ポート選択+接続、機体プロファイル選択(MAC未設定は「⚠ MAC未設定」表示)
+「編集」ボタン(プロファイル編集モーダル: 行追加/行削除/保存=PUT/キャンセル)、
リンク状態(serial/relay/drone)、電圧。
タブ(v2 で 4 タブ): **Posture**(Start/Stop、roll/pitchスライダ±5°(設定で±10°まで)
=飛行中のみ操作可、高度スライダ0.1–1.0m=接続中なら離陸前から操作可(CMD_SETPOINT
flags bit0 による離陸目標高度の事前設定)、ヨー角スライダ±180°+ヨー角制御トグル+
FF プロファイル欄) / **Position**(Start/Stop、目標XYZ入力+プリセット、XY平面
プロット(現在/目標+目標軌道円の重畳)、「機体計算指令」表示(機上 XY PID が
計算した roll/pitch の TLM_STATE エコー)、軌道セレクタ
(ホバリング/円軌道)+円軌道パラメータ+開始/停止、ヨー系は Posture と共通+
「進行方向を向く」オプション) / **Experiment**(v2: モーターテストバー(0.6 以上は
高出力許可チェック)、スイープ、加算性シーケンス、3D磁気/Accel6/地磁気/
キャリブプロファイル/FF 抽出・適用の各パネル。機体は CMD_MODE で MOTOR_TEST 状態)/
**Multi**(v2: 機体選択 2〜4機チェックボックス+「選択適用」、リジッドボディ確認
(/api/mocap/bodies を 500ms ポーリング)、機体別目標入力行、機体別色の共有 XY
プロット、機体別ステータスチップ、一斉スタート(確認ダイアログ)。
STOP / SPACE は全機一斉)。
共通モニタ: 姿勢数値+バー(Yaw は EKF 有効かつ健全 = est_mode_ekf &&
anchor_valid && mag_fresh のとき EKF ヨー表示・ラベル「Yaw(EKF)」、それ以外は
Madgwick — ファーム yaw_used と同じ選択規範)、高度(現在vs目標)、モータデューティ、
飛行状態インジケータ、EKF 健全性(ffg/ff_status)警告バッジ、レイテンシ、
**クイック較正カード**(Attitude 0 / Attitude Clear / Yaw 0 / Yaw Clear。v2.2 で
Experiment パネルから移動し全タブで見える。Multi モード中は対象機体
ドロップダウンを表示)、イベント/ログコンソール、ログ保存トグル+ファイル名。
COMPLETE状態のときのみRe-arm(RESET)ボタン表示。
**SPACE 緊急停止は全タブで有効**(stop に加え、Experiment 中は CMD_MOTOR_STOP も送出。
Multi 中は**全機**へ一斉 CMD_STOP)。

## 実験モードの状態遷移(v2)

PC 側 session のモード(posture / position / experiment)と機体側
FlightState(PROTOCOL.md の enum)の対応。実装は
`pc_server/core/session.py`(set_mode / activate_experiment /
_enter_experiment / _exit_experiment)と
`firmware_stampfly/src/flight_control.cpp`(CMD_MODE 処理)。

```
 PC session mode          機体 FlightState
 ─────────────────        ──────────────────────────────
 posture / position  ←→   WAIT / TAKEOFF / HOVER / LANDING / COMPLETE
       │  set_mode("experiment")(armed/flying 中は拒否)
       │  = 50Hz 送信停止 → CMD_MODE(1) 送信 → TLM_ACK(ok) 確認
       ▼                          (機体: WAIT → MOTOR_TEST, reason=11 mode_change)
 experiment(active)  ←→   MOTOR_TEST(=7)
       │  set_mode("posture"/"position")
       │  = スイープ/シーケンス中断+モーター停止 → CMD_MODE(0)+ACK
       ▼                          (機体: MOTOR_TEST → WAIT, reason=11)
 posture / position(50Hz 送信再開)
```

- **experiment 開始**: 飛行中(phase=armed/flying)は拒否 → 50Hz セットポイント
  送信を停止 → CMD_MODE(mode=1) 送信+ACK 確認。ACK が ok 以外
  (bad_state = 機体が WAIT 以外)なら元のモードへ戻し 50Hz を再開する。
  機体側は WAIT でのみ mode=1 を受理(同一状態への再送は冪等に ok)。
- **experiment 中**: START は PC・機体の両方で拒否(機体側 reason=10)。
  CMD_MOTOR_RUN / CMD_MOTOR_STOP・キャリブ/FF コマンド(0x14–0x23)を受理し、
  TLM_EXP を 25Hz 送出。飛行制御 PID・ミキサは動かず、PWM のライターは
  モーターテストサービスのみ。CMD_MOTOR_RUN は 1.5s 途絶で自動停止。
- **CMD_STOP(SPACE 緊急停止)**: 機体は MOTOR_TEST→WAIT に遷移する。PC 側は
  スイープ/シーケンス中断+モーター停止+experiment_active 解除を行うため、
  実験を続けるには UI の「実験モードを有効化」(activate_experiment →
  CMD_MODE(1) 再送)が必要。
- **シリアル切断**: CMD_MODE は送らない(送れない)。機体は CMD_MOTOR_RUN の
  1.5s 途絶フェイルセーフでモーターを自動停止する(MOTOR_TEST 状態には残る)。
- **緊急停止の優先経路**: `SessionManager.emergency_stop()` は
  `_command_lock` を経由せず、キープアライブ停止+ CMD_MOTOR_STOP + CMD_STOP を
  先行送出する(低速コマンドの ACK 待ち最大約4秒に巻き込まれない)。

## 複数機モード(MODE_MULTI、v2)

2〜4機を1本のリレーで同時に MoCap 位置制御する。ワイヤ仕様
(RLY_SET_PEERS 0x55 / RLY_PEERS_ACK 0x56 / RLY_MUX_UP 0x57 / RLY_MUX_DOWN 0x58)
は PROTOCOL.md が正。**機体ファームは無改修** — ESP-NOW 区間のバイト列は
単機時と同一で、多重化エンベロープはシリアル区間(PC⇔リレー)にのみ存在する。

- **実装**: `pc_server/core/multi.py` の `MultiControlManager`(選択・機体別目標・
  一斉開始/停止・20Hz supervise)+ `DroneSlot`(機体1機ぶんの実行状態)。
  SerialLink / MocapSource は1つを共有し、機体ごとに独立の PositionController
  (自前の 50Hz 送信スレッド。CMD_POS_ERR がハートビート兼用)を持つ。
  MoCap は機体プロファイルの `rigid_body_id` で `MocapSource.subscribe()` する。
  node_id = 選択順 index = RLY_SET_PEERS のエントリ index。
- **宛先分け**: 上りは serial_link の `send_to` / `send_pos_err_to` が
  RLY_MUX_UP で包み(内側・外側で seq を共有)、下りは RLY_MUX_DOWN の
  内側フレーム(TLM_STATE / TLM_CTRL / TLM_ACK 等)を
  `register_node_handler` 経由で node_id 付きディスパッチする。
  リレーは単機(RLY_SET_TARGET)/マルチ(RLY_SET_PEERS)の排他2モードで、
  マルチ中は非エンベロープの上り 0x10–0x2F をレート制限つき LOG_TEXT で拒否する。
- **開始条件(start_all)**: 全機の目標設定済み・全機の MoCap 新鮮**かつ
  位置データ有効(data_valid)**・目標同士の
  XY 距離 ≥ `multi.min_target_separation_m`(既定 0.5m)・全機が地上
  (|z| ≤ `multi.start_ground_z_max_m`、既定 0.3m — RB ID 取り違え対策)。
  1機でも不合格なら全機開始しない。開始受理時に各スロットのフィルタを
  `reset_filter` で飛行単位に初期化する(単機 START と同じ)。目標 XY は ±`multi.target_xy_abs_max_m`
  (既定 2.0m)以内のみ受理し、いずれかのスロットが armed/flying の間の
  目標変更でも他機目標との最小間隔を再検証する。
- **フェイルセーフ**: 単機セッションと同じ規範をスロットごとに適用する:
  STOP 再送(600ms×3、LANDING/WAIT イベントで解除)/ MoCap 途絶
  (>300ms 水平固定は PositionController 内蔵、>2s で**当該機のみ** CMD_STOP)/
  MoCap データ無効の持続(受信はあるが data_valid=0 継続。
  > `failsafe.data_invalid_warn_s` 警告、> `failsafe.data_invalid_stop_s` で
  **当該機のみ** CMD_STOP)/ START 猶予。加えて armed/flying スロットのテレメトリ途絶
  (> `multi.tlm_timeout_s`、既定 3s — リレー再起動・機体電源断の検出)で
  当該スロットを idle へ解放し警告する。飛行中の閉ループで XY 位置誤差
  > `multi.divergence_error_m`(既定 1.0m)が `multi.divergence_hold_s`
  (既定 1.0s)継続した機体は発散とみなし**当該機のみ** CMD_STOP
  (rigid_body_id 取り違えによる交差結合の最終防衛線。2026-07 から単機
  セッションにも同規範を適用: `failsafe.divergence_error_m` /
  `divergence_hold_s`)。SPACE / stop は**全機一斉**
  (`emergency_stop_all` はロックを迂回して CMD_STOP を先行送出)。
  機体側 200ms/500ms のリンク喪失フェイルセーフが機体ごとの最終防衛線。
- **飛行ガード**: マルチ選択中は `select_airframe` を拒否(RLY_SET_TARGET が
  リレーのピア表をクリアし全機への上り経路を切断するため)。armed/flying
  スロットのプロファイルは `update_airframes` でも変更・削除不可。
- **server.json `multi` 節**: `min_drones`(2)/ `max_drones`(4)/
  `tlm_state_div`(1 = 間引きなし。リレーが node 別に TLM_STATE のみ 1/n 間引き)/
  `target_xy_abs_max_m`(2.0)/ `min_target_separation_m`(0.5)/
  `tlm_timeout_s`(3.0)/ `start_ground_z_max_m`(0.3)/
  `divergence_error_m`(1.0)/ `divergence_hold_s`(1.0)/
  `max_yaw_ctrl_deg`(30.0 — UI 側 `UI.MULTI_YAW_LIMIT_DEG` と同値に保つ)。
- **UART 帯域**: シリアルは既定 **460800**(リレー `RELAY_UART_BAUD` と
  server.json `serial.baudrate` を必ず一致させる)。下りは機体ごとに
  TLM_STATE ≈146B(ワイヤ)×25Hz ≈3.7KB/s+TLM_CTRL ≈100B×25Hz ≈2.5KB/s
  = **≈6.2KB/s/機**(マルチ時は +MUX 包装 ≈10B/フレーム)。
  115200(≈11.5KB/s)では単機で既に下り ≈53% で、2機以上は収容不能のため
  460800(≈46KB/s。4機下り ≈58%、上り 4×50Hz CMD_POS_ERR ≈18%)へ移行した
  (数値の正典は PROTOCOL.md「帯域予算」)。460800 非対応の USB シリアル
  ブリッジは firmware_relay の `release-115200` env+`serial.baudrate`=115200
  で運用するが、リレーの間引き(`multi.tlm_state_div`)は TLM_STATE のみで
  TLM_CTRL は対象外のため**単機のみを推奨**する。
- **機体別ヨー角制御(v2)**: `multi_yaw` で機体ごとに ON/OFF+目標を選択
  できる。目標は ±`multi.max_yaw_ctrl_deg`(既定 30°)に制限する —
  機上XY制御(CMD_POS_ERR)は機体側でヨー回転補償を行うため原理上は
  不要な制約だが、**機上ヨー回転補償の飛行未検証のため暫定維持**している
  (飛行検証完了後に撤廃を検討)。目標は各スロットの
  SetpointShaper が最短経路 wrap+45°/s 制限で整形(単機のヨースライダと
  同一経路)。OFF の機体は flags bit1=0(レートダンピング)。MoCap 途絶中の
  機体へのヨー変更は拒否。est_mode=1 で EKF 不健全時は機体側が自動で
  レートダンピングに縮退する(単機 §13 と同一のファーム側フェイルセーフ)。
- **機体別 FF プロファイル(v2)**: `/api/ffprofile` の `apply`/`mode`/`anchor`
  に `"drone":name` を指定するとノード宛(RLY_MUX_UP + ノード別 TLM_ACK/
  TLM_CAL_DATA 照合)で実行される。適用状態は `ff_state.json` の
  `applied_by_mac[mac]` に機体別記録(単機の `applied` とは独立)。
  飛行中(armed/flying)の機体への FF 操作は拒否される。
- **スコープ**: 静的目標のみ(円軌道なし)。CSV ログは機体ごとに
  `<日時>_multi_<機体名>.csv` で記録される(LOG_STRUCTURE.md)。

## 安全クランプ(多層)

| 層 | roll/pitch | alt | yaw(v2) |
|---|---|---|---|
| UI | ±5°(既定) | 0.1–1.0m | スライダ ±180°(0.1° 刻み) |
| pc_server | ±10° + スルーレート30°/s | 0.1–1.2m + 0.3m/s | ±180°(`max_yaw_deg`)+最短経路 wrap+スルーレート 45°/s(`yaw_slew_rate_deg_per_s`) |
| ファーム | ±30°(継承) | 0.05–1.5m | 角度誤差を wrapPi(±π)して psi_pid → 出力ヨーレートを ±0.5 rad/s にクランプ(`yaw_rate_limit_rad_s`。2026-07-17 に duty 天井対策で 1.0→0.5) |

ヨーの値は `pc_server/config/server.json` の clamps 節と
`firmware_stampfly/src/config.hpp`(yaw_angle / yaw_rate_limit_rad_s)が実体。
EKF 不健全時はファームが角度制御をレートダンピングに縮退する
(PROTOCOL.md フェイルセーフ表)。

## NVS 永続化一覧(v2)

実装は `firmware_stampfly/src/yaw_estimation/persistence.{hpp,cpp}`。
書込みは **WAIT / COMPLETE / MOTOR_TEST のコマンド処理内のみ**
(飛行中の NVS 書込み禁止)。ブート時復元は `sensor_hub_ff.cpp` の
`sensorHubFfInit()` で以下の順に行う(yaw側 app.cpp の順序踏襲):

**mag3d → accel6 → attmount → geomag → (リファレンスCF reset) → yawzero → ffcal**

(yawzero はリファレンスCFのリセット後に復元する。リセットが復元済み磁気
オフセットを消すため。)

| namespace | 内容 | 書込コマンド | 特記 |
|---|---|---|---|
| `mag3d` | 3D磁気較正 offset[3] + matrix[9] | CMD_MAG3D_SET | 適用/クリア時: FF 自動無効(ff_mode=0 を NVS 保存)+アンカー破棄+ヨー推定器再シード |
| `accel6` | 加速度6面 offset[3] + scale[3] | CMD_ACCEL6_SET | 適用時に姿勢参照リセット(ahrs_reset 含む) |
| `attmount` | マウントオフセット roll/pitch [rad] | CMD_ATTMOUNT_SET | 磁気レベル化入力にのみ適用 |
| `geomag` | 地磁気リファレンス5値(偏角・伏角・H・V・F) | CMD_GEOMAG_SET | |
| `yawzero` | mag_yaw_offset [rad] | CMD_YAWZERO_SET | 復元専用 API(PC 側が offset を逆算して送る。PROTOCOL.md 0x1B) |
| `ffcal` | schema(u32=1) / valid / nlut / crc(u32) / blob(float32列) / ff / est | CMD_FF_COMMIT(係数+crc)、CMD_FF_MODE(ff/est のみ更新) | ロード時 schema 照合 → blob の CRC 再計算照合 → 不一致は自己修復破棄。ffcal 無効なら ff_mode=0 に落とす |

## I2C スケジューリング(v2: 位相スタガ・電圧20Hz化)

実装は `firmware_stampfly/src/sensor.cpp`(400Hz ループ)と
`yaw_estimation/sensor_hub_ff.cpp`(`sensorHubFfUpdate`)。

- **20Hz スロット**(`YAW_SLOW_SLOT_PERIOD_MS` = 50ms): 同一 tick 内で
  **updateCurrent() → updateMagnetometer() の順**(順序契約。FF 補正が磁気
  サンプルと同時刻の電流を見るため)。
- **位相スタガ**: ToF 読みが発生した tick では磁気/電流読みをスキップし
  次 tick へ繰延べる(重量級 I2C 読みの同 tick 集中による 2.5ms 予算超過回避)。
- **電圧の 20Hz 化**: 電圧は INA3221(CH2)の 20Hz 電流読みの bus_voltage に
  一本化(旧: 毎 tick getVoltage は廃止)。INA3221 は CH2 のみ・140µs 変換・
  AVG16(実効平均窓 ≈4.5ms。HW 平均が旧 voltage_filter を代替)。
  低電圧判定「<3.34V が 0.25 秒継続」は **20Hz×5 サンプル連続**に置換
  (意味を保存。config.hpp `low_voltage_count`)。計測不能な 20Hz スロットも
  低電圧と同様にカウントする(電圧が測れない機体は飛ばさない)。
- **EKF**: predict は毎 tick(400Hz、dt 実測)、update は fresh 磁気サンプル
  のみ(BMM150 実効 ~10Hz)。predict はチルト運動学
  `ψ̇=((ω_z−b_g)·cosθ−p·sinθ)/cosφ`(|roll|>60° は従来式 `ω_z−b_g` に
  フォールバック)。NIS 棄却が 5s 続くとソフト再捕捉(ffg bit7、
  Δψ≤3°/更新・バイアス補正ゼロの制限付き更新)で飛行中も自動復帰する
  (FF_PIPELINE.md §5.5)。
- **Z軸ジャイロ積算**: sensor.cpp の Yaw_rate 確定直後に
  `Yaw_gyro_integral += Yaw_rate * Interval_time`(400Hz)。ahrs_reset() で
  ゼロクリア(TLM_STATE yaw_gyro_int_rad)。
- **テレメトリ位相**: TLM_STATE は 16 分周(25Hz)、TLM_EXP は同じ 25Hz を
  8 tick 位相をずらして送出(`telemetry_exp_phase_ticks`)。
- **アイドルアンカー**: モーター完全停止中(PWM 実出力ゼロ)に b_cal と
  I_total を 20Hz×2s 窓(40 サンプル)で蓄積。モーター始動遷移(離陸 START・
  モーターテスト開始)で凍結し EKF を reanchor(ψ0 は健全な EKF の現在ψを優先、
  不健全時はリファレンスCF — FF_PIPELINE.md §5.5 改修C)。CMD_FF_ANCHOR で
  手動再取得(回転中/窓未充足は status=busy)。EKF が観測を 10s 超受理できて
  いないときは停止中に自動再アンカー(30s クールダウン、LOG_TEXT 通知)。

## コーディング規約

- 識別子は英語、説明コメントは日本語可。既存コードのtypo(Rall_*, RNAGE0FLAG等)を
  新規コードに持ち込まない。
- マジックナンバー禁止: ファームは config.hpp / FlightConfig、PCは config/*.json。
- C++: グローバルvolatile間通信を新規に増やさない。タスク間は既存のportMUX
  メールボックス/FreeRTOSキューのパターンに従う。ISR/受信コールバック内での
  Serial出力・ブロッキング禁止。
- Python: スレッド共有状態はlockで保護。time.monotonic()のみ使用(time.time()禁止)。
  ブロッキングI/Oをasyncループに持ち込まない(スレッド+queueで橋渡し)。
- テスト: protocol往復・パーサ破損系・コントローラのフェイルセーフはハードなしで
  pytest可能に保つ(既存プロジェクトのフェイクserial/NatNetパターンを踏襲)。

## 検証コマンド

```
# プロトコル
cd protocol && python3 -m pytest tests/ -q
# ファーム(コンパイル)
~/.platformio/penv/bin/pio run -d firmware_stampfly -e release
~/.platformio/penv/bin/pio run -d firmware_relay -e release
# サーバ
cd pc_server && .venv/bin/python -m pytest tests/ -q
# FF 抽出の受入テスト(pytest 不要、直接実行)
cd data_analysis && .venv/bin/python tests/test_ff_extraction.py
# flight_log_viewer の動作確認(ダミーログ生成 → 全出力)
cd flight_log_viewer && .venv/bin/python tests/make_dummy_log.py \
  && .venv/bin/python visualize.py ../logs/flight_logs/dummy_position.csv --all
# サーバ起動
cd pc_server && .venv/bin/python -m uvicorn app:app --host 127.0.0.1 --port 8000
```

## 既存資産の参照元(コピー/移植元のパス)

以下は**移植元の歴史的記録であり、V2 の動作(ビルド・実行・テスト)には
不要**(参照先が無くても V2 は完結する)。

> **注記(2026-07-27)**: 移植元のうち `Previous_Version/`(OptiTrack版・
> Drone_Log_Viewer・PySerial版)と `Yaw_Estimation_Project/` は
> **本ツリーから削除済み**(別途アーカイブに保管)。以下に挙げるパスは
> **由来の記録であって、この作業ツリー内で解決できるパスではない**。
> 現存する参照先は `../../../Original_Projects/`(リポジトリルートからの相対)と、
> ヨー実験系の同期先 `../StampFly_Telemetry/` のみ。

- 機体ファーム流用層:
  `Previous_Version/StampFly_OptiTrack_PID_Control_System/M5StampFly/src/`
  (sensor, imu, tof, alt_kalman, pid, flight_state.hpp の構造化パターン,
  esp_now_callback_compat.hpp)と同 `lib/`
  (原典: github.com/ryoma0415/StampFly_OptiTrack_PID_Control_System)
- LED/ブザー: `../../../Original_Projects/M5StampFly-main/src/led.*`(流用),
  `buzzer.*`(非ブロッキング化+ピン修正のうえ移植)**← 現存**
- PC側:
  `Previous_Version/StampFly_OptiTrack_PID_Control_System/NatNet_PID_Controller/`
  の pid_controller.py, position_filter.py, NatNetClient.py ほか、config.json の
  ゲイン値(NatNet SDK 本体は `../../../Original_Projects/NatNetSDK/` **← 現存**)
- v2 ヨー推定・実験機能: `Yaw_Estimation_Project/Yaw_Calibration_and_Estimation/`
  (firmware/src の bmm150_driver, mag_calibration, current_sensor, ff_calibration,
  yaw_estimator, yaw_estimator_kf, persistence と sensor_hub.cpp の FF 挿入点/
  アンカー、pc_server/server.py の SweepRunner/SequenceRunner/fit_ellipsoid/
  calprofile/geomag/FfProfileManager、data_analysis/ 一式)。数式・符号・定数値は
  無変更(ω_z=−gyro_z−offset、R_z 標準CCW、levelMagVector 非教科書符号、
  Q は dt スケール)。同内容の実装は `../StampFly_Telemetry/Telemetry/`
  (firmware/src/yaw_estimation/, pc_server/)に現存する
- v2 円軌道: `Previous_Version/StampFly_OptiTrack_PID_Control_System/
  NatNet_PID_Controller/circling_controller.py`(軌道生成の参考のみ。ゲイン・
  符号は V2 の座標変換規約に従い、旧 config の値は流用しない)
- v2 flight_log_viewer: `Previous_Version/Drone_Log_Viewer/`
  (For_Research / For_Presentation のグラフ・同期アニメーション構成を参考)
- 既知の流用禁止物: 旧esp32_relay(両方)、HoveringControllerの__getattr__委譲、
  -O0のplatformio.ini、全マーカー重心フォールバック
