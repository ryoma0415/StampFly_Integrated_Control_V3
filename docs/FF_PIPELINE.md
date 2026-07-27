# 電流FF較正パイプライン 実装仕様書(インターフェース契約)— V2

スイープ実験データ → パラメーター抽出 → FFプロファイルJSON → UI選択 → ファームウェア配信 →
補正Yaw飛行、の全区間を定義する。方式の数式は yaw側プロジェクト(旧
`Yaw_Estimation_Project/Yaw_Calibration_and_Estimation`)の
`yaw_estimation_ff_two_methods.md`(方式A/B)と `yaw_estimation_method_v2.md`(§5)に従う。
本書は**実装間のインターフェース**を確定する契約書。

本書は yaw側 `docs/ff_pipeline_design.md`(2026-07-02)を V2 に移設したもの。
**数式・係数定義・受入基準は原文のまま**であり、V2 で変わったのは輸送路
(UDP `ffcal_*` JSON → シリアル `CMD_FF_*` バイナリ。ワイヤ仕様は
[PROTOCOL.md](PROTOCOL.md) が正)とファイル配置(`pc_server/data/` 配下)、
スイープのサンプル源(UDP JSON テレメトリ → TLM_EXP)である。
なお上記 yaw側プロジェクトへの参照は**移植元の歴史的出所**であり、
V2 の動作(ビルド・実行・テスト)には不要(数式の規範を示すための参照)。
**yaw側フォルダは 2026-07-27 に本ツリーから削除済み**(別途アーカイブに保管)。
以降、実装上の正典は本書と V2 実コード
(`firmware_stampfly/src/yaw_estimation/ff_calibration.*` / `yaw_estimator_kf.*`、
`pc_server/core/ffprofile.py`、`data_analysis/ff_params`)とする。
方式A/Bの補間式は §5.1、抽出計算は §3.2、4状態EKF の予測・更新・ゲートは §5.5 に
すべて明記してあり、**本書だけで実装・検証が完結する**(削除済み文書の参照は不要)。

---

## 0. 全体データフロー

```
【取得】pc_server UI(Experiment タブ: スイープ/加算性シーケンス)
    機体は MOTOR_TEST 状態。モーター駆動 = CMD_MOTOR_RUN(0.4s キープアライブ)、
    サンプル源 = TLM_EXP(25Hz)
    └→ pc_server/data/sweep_results/…/sweep_*_{samples.csv, meta.json}
        ※ユーザーは sweep_results/ 内にサブフォルダを作り、同一機体・同時期の
          8本(16ファイル)をひとまとまりで移動して管理することがある

【① 抽出】data_analysis/ff_params(ライブラリ)+ make_ff_profile.py(CLI)
    入力: フォルダ指定(主) or 8ファイル指定(従)
    └→ pc_server/data/ff_profiles/<name>.json(stampfly_ff_profile v1)

【② 適用】pc_server /api/ffprofile apply(core/ffprofile.py)
    mag3dバインディング照合 → CMD_FF_BEGIN/LUT/MOT/AUX/COMMIT 分割転送(各 TLM_ACK)
    → CRC照合 → 機体 NVS 永続化
    └→ pc_server/data/ff_state.json に適用状態を記録

【③ 実験】ファームウェアがFF補正+推定(CF/EKF)をオンデバイス実行
    TLM_STATE 末尾拡張に補正後Yaw・ジャイロ積算・ΔB̂・b_m・NIS・ffg 等を載せる
    └→ 飛行ログ(logs/flight_logs/*.csv v4・109列、LOG_STRUCTURE.md)に 50Hz で記録
        └→ flight_log_viewer でヨー4系統比較・EKF診断・サマリレポート
```

(yaw側にあった `/api/yawlog` による専用 yaw ログは V2 では廃止し、飛行ログ+
flight_log_viewer に統合した。`pc_server/data/yaw_eval_results/` は旧形式
yawlog の置き場としてのみ残る。旧形式向け解析スクリプト
(analyze_yaw_eval.py / replay_yaw_ff.py)は削除済み。)

## 1. ディレクトリ・ファイル構成(V2)

```
StampFly_Integrated_Control_V2/
  docs/FF_PIPELINE.md                 # 本書
  data_analysis/
    ff_params/
      __init__.py
      core.py                         # 純粋関数: 8ラン → プロファイルdict
                                      # (+ sequence meta → 単機ラン展開)
    make_ff_profile.py                # 抽出CLI+対話モード(8本 or 全機4+sequence)
    plot_sweep.py                     # スイープ校正解析+加算性検証グラフ(対話式)
    plot_explog.py                    # Experiment計測ログ(exp_logs)グラフ化(対話式)
    graphs/                           # グラフ出力先(生成物)
    tests/
      test_ff_extraction.py           # 受入テスト(6/12照合 + 付録A再現)
      fixtures/results.json           # 受入テストの真値(6/12 feasibility 結果)
  pc_server/
    core/ffprofile.py                 # 抽出サブプロセス起動・適用・状態管理
    data/
      sweep_results/                  # スイープCSV+meta(取得元)
      ff_profiles/                    # プロファイルJSON置き場
      ff_state.json                   # 適用状態(サーバーが管理)
      yaw_eval_results/               # 旧形式yawログ置き場(レガシー)
    tests/test_ffprofile.py           # フェイクserialでの適用/CRC/リトライ試験
  firmware_stampfly/src/yaw_estimation/
    ff_calibration.hpp/.cpp           # 係数保持・LUT/差動計算・ステージング
    yaw_estimator_kf.hpp/.cpp         # 4状態EKF
    sensor_hub_ff.hpp/.cpp            # FF補正挿入点・アイドルアンカー
    persistence.hpp/.cpp              # NVS(namespace "ffcal")
    yaw_config.hpp                    # FF_*/FF_EKF_* 定数(yaw側と同値)
```

## 2. FFプロファイルJSON(schema `stampfly_ff_profile` version 1)

ファイル名 = `<name>.json`。`name` はフォルダ指定時はフォルダ名、それ以外は指定またはデフォルト
`ff_<最初の全機ランの日付YYYYMMDD>`。数値はすべて float(JSONのnumber)。座標系は body・b_cal
空間(mag3d適用後)、単位は µT / A / duty(0-1)。

```jsonc
{
  "schema": "stampfly_ff_profile",
  "version": 1,
  "name": "DroneTest_20260629",
  "memo": "Drone-test リビング 2026-06-29〜30 満充電 8本",   // ★1行メモ(UI表示用・ユーザー編集可)
  "created_at": "2026-07-02T12:00:00+09:00",
  "provenance": {
    "tool": "make_ff_profile.py",
    "tool_version": "1.0",
    "source_dir": "sweep_results/DroneTest_20260629",        // 相対 or 絶対(情報のみ)
    "all_motor_runs": [                                       // 全機×4姿勢(必ず4本)
      {"stem": "sweep_20260629_141809", "orientation": "Yaw=0°", "location": "リビング_ドローンtest",
       "vbat_start_v": 4.26, "created_at": "..."}
      // ×4
    ],
    "single_motor_runs": [                                    // 単機×4(必ず4本)
      {"stem": "sweep_20260630_163738", "motor": "FL", "vbat_start_v": 4.25, "created_at": "..."}
      // ×4 (FL,FR,RL,RR 各1)
    ],
    "acquired_span": ["2026-06-29", "2026-06-30"]
  },
  "binding": {                                                // ★有効性バインディング
    "mag3d": {"offset": [x,y,z], "matrix": [[...],[...],[...]]},  // 取得時metaのスナップショット
    "mag3d_hash": "sha256:<hex>",                             // 正準文字列(%.9g join ',')のSHA256
    "consistent_across_runs": true                             // 8本のmag3dが一致していたか
  },
  "method_a": {
    "lut": {
      "i_idle_a": 0.162,                                       // ベンチ参考値(実行時はアンカー実測が優先)
      "points": [ {"i_a": 0.162, "db": [0,0,0]}, ... ]         // (I_idle,0)起点・電流昇順・~20点
    },
    "affine_ref": {"a": [ax,ay,az], "b": [bx,by,bz]}           // 参考(全域・アンカー込みpooled)
  },
  "method_b": {
    "a_m":      {"FL": [3], "FR": [3], "RL": [3], "RR": [3]},  // I_active空間・原点拘束 [µT/A]
    "a_bar":    [3],                                            // ā = mean(a_m)
    "a_tilde":  {"FL": [3], "FR": [3], "RL": [3], "RR": [3]},  // ã_m = a_m − ā
    "duty_to_current": {                                        // Î=c2·d²+c1·d+c0 [A](I_active)
      "FL": {"c2":..,"c1":..,"c0":..,"rms_a":..}, ...
    },
    "pair_diff_xy_uT_per_A": 30.4                               // (FL+RR−FR−RL)/2 の水平ノルム
  },
  "stats": {                                                    // KF設計・診断用(実行には未使用)
    "noise_floor_std_uT": [3],
    "hysteresis_max_uT": [3],
    "additivity_closure": [3],                                  // Σa_m/4 ÷ 全機slope
    "orientation_slope_std": [3]
  },
  "quality": {
    "baseline_flag_count": 0,
    "affine_fit_rms_uT": [3],
    "warnings": ["..."]                                         // 抽出時の警告(mag3d不一致等)
  }
}
```

## 3. 抽出仕様(data_analysis/ff_params + make_ff_profile.py)

### 3.1 入力

- **フォルダ指定(主)**: `--folder <path>`。フォルダ内の `sweep_*_meta.json` +
  `sweep_*_samples.csv` ペアを列挙。**ちょうど8ペア**であること(過不足はエラー)。
  パスが存在しない場合は `--results-dir` 直下のサブフォルダ名として解決する。
- **ファイル指定(従)**: `--stems sweep_A sweep_B ...`(8個)+ `--results-dir <dir>`
  (既定 `../pc_server/data/sweep_results`)。
- 自動分類: meta の `motors` が `FL+FR+RL+RR` のもの=全機ラン(4本必須、`notes.orientation`
  が4種の別姿勢であること。表記ゆれ `Yaw=+-180°`/`Yaw=±180°` 許容)、`FL`/`FR`/`RL`/`RR`
  単独=単機ラン(各1本必須)。構造が合わなければ内訳を示してエラー。
  **meta.aborted=true の中断ランは stem 列挙付きでエラー拒否**(部分データからの
  静かな誤係数抽出を防ぐ)。出力JSONは非有限値(NaN/Inf)禁止
  (`allow_nan=False`、欠損する provenance 値は null)。
- `--name`, `--memo`, `-o/--out`(既定 `../pc_server/data/ff_profiles/<name>.json`),
  `--plots`(検証図PNG、既定off)。memo 既定値は
  `"<notes.locationの多数派> <acquired_span> 取得8本"` を自動生成。

### 3.2 計算(旧 analyze_feasibility_20260612.py から忠実に移植・同スクリプトは削除済み)

集計・フィットは feasibility スクリプトの関数を**そのまま**ライブラリ化する
(`aggregate`: phase=='measure'、(duty,leg)別、n≥3 / `fit_affine`(アンカー込み) /
`fit_prop`(原点拘束) / LUT生成(4姿勢平均+(idle_mean,0)起点・電流昇順) /
duty→電流2次fit)。idle は meta `idle_current_a` 優先。

- `method_a.lut` : 全機4本から。`affine_ref`: 全域pooled・アンカー込み。
- `method_b.a_m` : 単機4本、I_active空間・原点拘束。`duty_to_current`: 同、2次fit。
- mag3d一致チェック: 8本のmeta.mag3dを比較、offset/matrix の相対差>1e-6 なら
  `consistent_across_runs=false` + warning(binding には全機ラン第1本の値を採用)。

### 3.3 受入テスト(tests/test_ff_extraction.py — venvのpythonで直接実行)

```sh
cd data_analysis && .venv/bin/python tests/test_ff_extraction.py
```

1. **6/12照合(厳密)**: 6/12の8 stem で抽出し、
   `tests/fixtures/results.json`(旧 `graphs/feasibility_20260612/results.json`。
   `graphs/` は生成物置き場のため git 管理外)の `pooled_model.a/b`・
   `additivity.per_motor_a_prop`・`duty_to_current_quadfit`・`lut_breakpoints.points`
   と相対誤差 < 1e-6 で一致すること(同一コードパスの移植確認)。
2. **付録A再現(新機体)**: 6/29の4本+6/30の4本で抽出し、
   `yaw_estimation_ff_two_methods.md` 付録Aと比較:
   affine a=[24.41,28.67,8.40], b=[−6.50,−3.49,−1.56](許容 2% or 0.15 abs)、
   a_m 4本(許容 2% or 0.5 µT/A abs)、ā=[23.02,28.66,5.89]、
   duty→電流 c2/c1/c0(許容 5%)、pair_diff_xy=30.4(許容 2%)。
   ※付録Aの生成コードは消失済みのため、系統的な不一致が出た場合は集計選択肢
   (飛行帯/アンカー等)を調査しテストにコメントを残す。

## 4. 配信プロトコル(CMD_FF_* 系。V2: UDP `ffcal_*` JSON を置換)

シリアル/ESP-NOW のバイナリメッセージ(ワイヤ仕様の正典は
[PROTOCOL.md](PROTOCOL.md))。すべて **TLM_ACK(0x32、6B:
`u8 acked_type, u32 acked_seq, u8 status`)で応答**する。status:
0=ok, 1=bad_state, 2=invalid_arg, 3=crc_mismatch, 4=busy, 5=incomplete。
受理は WAIT / COMPLETE / MOTOR_TEST 状態のみ(飛行中の NVS 書込み禁止)。

| type | 名前 | payload | 動作(yaw側コマンドとの対応) |
|---|---|---|---|
| 0x1D | CMD_FF_BEGIN | `u8 nlut(4-24)` =1B | ステージング領域クリア、点数宣言(= ffcal_begin) |
| 0x1E | CMD_FF_LUT | `u8 idx, f32 i_a, f32 db_x, db_y, db_z` =17B | LUT点k(電流昇順)(= ffcal_lut) |
| 0x1F | CMD_FF_MOT | `u8 idx(0=FL,1=FR,2=RL,3=RR), f32 a_tilde[3], f32 c2, c1, c0` =25B | a_tilde=**ã_m**、c2..c0=duty→I_active 2次fit(= ffcal_mot) |
| 0x20 | CMD_FF_AUX | `f32 iid_a` =4B | ベンチ参考アイドル電流(= ffcal_aux) |
| 0x21 | CMD_FF_COMMIT | `u32 crc32` =4B | 完全性チェック(全idx受領+昇順)→CRC照合→ランタイム反映+NVS保存。不一致は status=crc_mismatch、未完は incomplete。**冪等**: ステージング無しでも確定済みCRCと一致すれば ok(ACKロスト後の再送対策)(= ffcal_commit) |
| 0x22 | CMD_FF_MODE | `u8 ff_mode(0=off,1=A,2=B), u8 est_mode(0=相補,1=EKF)` =2B | 実行時切替+NVS保存。切替時は補正系推定器をリファレンスから再シード(= ffmode) |
| 0x23 | CMD_FF_ANCHOR | 0B | アンカー即時再取得(モーター停止時のみ。回転中/窓未充足は status=busy)(= ffanchor) |

クリアに相当する専用コマンドはない(mag3d 変更時にファームが ff_mode=0 に
自動で落とす。§5.3)。読み出しは CMD_CAL_GET(0x17)→ TLM_CAL_DATA(0x34)の
末尾 `ff_nlut / ff_crc32 / ff_mode / est_mode` と valid_flags bit5(ffcal)で行う
(= ffcal_get)。

**CRC定義(原文維持)**: CRC-32(IEEE, zlib.crc32互換)。対象バイト列は下記の
float32 little-endian 連結:

```
for k in 0..nlut-1: ia_k, dx_k, dy_k, dz_k
for m in 0..3(FL,FR,RL,RR): ax,ay,az,c2,c1,c0
iid
```

PC側: `zlib.crc32(b"".join(struct.pack('<f', v)))`(`core/ffprofile.py` の
`ff_crc32`。送信前に各値を `ff_float32` で float32 に丸め、CRC と送信値を機体側と
一致させる)。CMD_FF_COMMIT には u32 LE で載せ、`ff_state.json` には hex8 文字列で
記録する。firmware側: 受信した float 値のメモリ表現から同一手順で計算。
yaw側は float を `%.9g` テキスト化して UDP 送信していたが、V2 はバイナリ float32 を
そのまま送るため往復誤差自体が存在しない(丸め規約は同一)。

## 5. ファームウェア設計(firmware_stampfly/src/yaw_estimation/)

### 5.1 ff_calibration モジュール

- ステージング(CMD_FF_BEGIN〜COMMIT)と確定領域の二段構え。commit時に完全性+CRC検証。
- NVS namespace `"ffcal"`: `schema`(u32=1), `valid`(bool), `nlut`(u32), `crc`(u32),
  `blob`(bytes: §4のCRC対象と同一順のfloat32列), `ff`(u32), `est`(u32)。
  ブート時ロード: schema照合→blob CRC再計算照合→不一致は自己修復破棄(既存mag3dの前例に従う)。
  ffcal が無効なら ff モードは 0(off)に落とす。
- 計算API:
  - `ffComputeDeltaB(i_total, duty[4], mask, motors_running)`
    - 方式A: 区分線形LUT補間。範囲外は端区間の傾きで外挿。
    - 方式B: + 差動項。`Î_m=q_m(d_m)`(mask外は d=0)、`I_active=I_total−I_idle`
      (I_idle はアンカー実測、未取得時は iid)、`s=I_active/ΣÎ_m`(ΣÎ<0.05A なら差動項0)、
      `δI_m=s·Î_m−I_active/4`、`ΔB̂_diff=Σ ã_m·δI_m`。
  - σ_ff/σ_slew/σ_diff の自己申告値も返す(EKFの適応R用。ff_two_methods §2.†)。
    |δI| は max_m|δI_m| を採用。定数は `yaw_config.hpp`
    (κ_ff=0.03, τ_resid=0.05s, σ_diff係数=0.3×30 µT/A)。
- duty の配線順は **FL,FR,RL,RR**(CMD_FF_MOT の idx と同順)。飛行中はミキサ
  出力4値を並べ替えて渡し(ベースの変数は FR,FL,RR,RL 順)、MOTOR_TEST 中は
  テスト duty×mask を渡す(`sensor.cpp`)。

### 5.2 挿入点と同期(sensor_hub_ff.cpp)

1. 20Hz スロット内で **`updateCurrent()` を `updateMagnetometer()` より先に呼ぶ**
   (同 tick 内順序契約。ToF 読み tick ではスロット自体をスキップして次 tick へ
   繰延べ — ARCHITECTURE.md「I2C スケジューリング」)。
2. `updateMagnetometer()` 内、mag3d適用(`b_cal`)直後・EMAフィルタ前に:
   `b_corr = b_cal − ΔB̂`(ffモード≠off かつ ffcal valid **かつ 電流サンプル有効** のとき。
   INA3221不調・読取失敗時は非補正縮退)。
   同tickで取得済みの電流・duty(実効値)を使用。
3. **補正系と非補正系で EMAフィルタ状態を分離**(2インスタンス)。
   非補正系(従来パス)は従来どおり動き続ける(TLM_EXP の bx_raw/bx_cal、
   スイープ収集に影響なし)。
4. INA3221 設定: 電流計測チャネル(CH2)のみ・変換時間140µs+AVG16 で実効平均窓
   ≈4.5ms(`current_sensor.cpp`)。電圧(bus_voltage)も同一変換から 20Hz で取得し、
   機体の電圧系はこの読みに一本化されている。

### 5.3 アンカー

- モーター停止中(モーターテスト停止 **かつ PWM実出力ゼロ** — duty→0のランプダウン
  完了まで窓を開始しない)、`b_cal` と `I_total` の直近2s移動平均を常時更新
  (20Hzリングバッファ、40サンプル。`FF_ANCHOR_PERIOD_MS`/`FF_ANCHOR_WINDOW_SAMPLES`)。
- **モーター始動遷移(停止→回転)で凍結**: `B0`(3軸), `I_idle`, `B0_horiz=levelMag(B0,roll,pitch)_xy`,
  `ψ0=健全なEKFの現在ψ(不健全時はリファレンスCFの現在yaw。§5.5 改修C)`。
  b_m←0、EKF P←P0、補正系EMA・norm_ref再初期化。
  離陸(CMD_START)とモーターテスト開始の両方が「始動遷移」。
- CMD_FF_ANCHOR で手動再取得(停止時のみ)。ブート後は窓が満ちた時点で自動初回取得。
- **地上自動再アンカー(2026-07改修B-2)**: モーターOFF・窓full で EKF が 10s 超
  観測を受理できていないとき自動で再取得(30s クールダウン。§5.5 参照)。
- アンカー有効フラグはテレメトリ ff_status bit3(anchor_valid)で公開。
- **CMD_MAG3D_SET(適用/クリア)時**: FF係数は旧 b_cal 空間で無効になるため、アンカー無効化+
  窓リセット+補正系再シード+**ff_mode=0(NVS保存)** を行う。係数blobはNVSに残す
  (CMD_FF_MODE で再有効化可能だが、原則は再抽出→再適用)。

### 5.4 推定器構成

常時3系統(ffモードoff時は①のみ):
1. **リファレンスCF**(既存YawEstimatorそのまま、非補正mag)。
   既存の yaw_zero/NVSシード等の意味は一切変えない。ffモードoff時はアクティブ出力もこれ。
2. **補正CF**(YawEstimator第2インスタンス、補正mag)。est_mode=0のときアクティブ。
   アンカー/モード切替時に①の内部状態(yaw・offset)をコピーして再シード。
3. **EKF**(yaw_estimator_kf、補正mag)。est_mode=1のときアクティブ。

アクティブ推定器の出力は TLM_STATE `yaw_est_rad`(オフセット97)。

### 5.5 4状態EKF(yaw_estimator_kf、ff_two_methods §4 準拠)

- 状態 `x=[ψ, b_g, b_mx, b_my]`、内部単位 rad / rad/s / µT。
- 予測: sensorHub tick毎(400Hz, dt実測)。**チルト運動学予測(2026-07改修A)**:
  本ファームの姿勢規約(sensor.cpp の Madgwick 軸入替)でのオイラー角レートは
  `ψ̇=((ω_z−b_g)·cosθ−p·sinθ)/cosφ`(φ/θ=マウントオフセット適用後の Madgwick
  roll/pitch=レベル化と同一値、p=ロールレート未フィルタ・起動offset減算後)。
  `cosφ<cos60°`(`FF_EKF_TILT_KIN_COS_MIN`、ロール特異点 cosφ=0 のガード)では
  従来式 `ψ̇=ω_z−b_g` にフォールバックし、ψ̇ は ±720°/s にクランプ
  (`FF_EKF_PSI_DOT_CLAMP_RAD_S`)。ヤコビアンは `F[0][1]=−cosθ/cosφ·dt`
  (フォールバック時 `−dt`)。b_g は従来どおり ω_z のバイアスとしてのみ
  モデル化(p のバイアスは非モデル化)。導出・実データ検証(7/10 実験ログとの
  per-step 回帰)は `yaw_estimator_kf.cpp` の predict コメント参照。
  `ψ⁻=wrap(ψ+ψ̇·dt)`; `b_m⁻=(1−dt/τ_bm)·b_m`; `P⁻=F·P·Fᵀ+Q·dt`。
  ω_z は既存規約(−gyro_z−起動offset)。
- 更新: freshな磁気サンプルのみ(実効10Hz)。観測 `z=(ℓx,ℓy)`(補正EMA後→レベル化)。
  `h(x)=R_z(ψ−ψ0)·B0_horiz+b_m`(R_z は標準CCW: atan2がψとともに増える向き)。
  `H=[R_z'(ψ−ψ0)·B0_horiz, 0, I₂]`。NIS=yᵀS⁻¹y。
- 適応R: `R_eff=R_base+σ_ff²+σ_slew²+σ_diff²+(sinθ_tilt·σ_rz)²`。
- ゲート(すべて実装、状態bitをテレメトリ `ffg` = TLM_STATE オフセット133へ):
  bit0: NIS>5.99→R×(NIS/5.99)膨張(適用中)、bit1: NIS>13.8→棄却、
  bit2: |‖b_corr,filt‖−‖B0‖|>20µT→棄却(8-20µTはR膨張=bit0扱い)、
  bit3: |b_corr,filt.z−B0.z|>12µT→棄却、bit4: tilt>25°→スキップ、
  bit5: ‖b_m‖>20µT→磁気更新凍結(要再アンカー)、bit6: |db_m/dt|>0.3µT/s 10s継続警告、
  bit7: ソフト再捕捉(制限付き更新)適用中(下記 B-1)。
  連続棄却>3s: P のψ・b_m対角を1.02/s で緩膨張(それぞれ P0 の10倍上限)。
- **NISロック脱出(2026-07改修B、2段)**:
  - **B-1 ソフト再捕捉(ffg bit7、飛行中も有効)**: 最終「通常受理」
    (NIS≤13.8 での採用)から `FF_EKF_RECAPTURE_AFTER_S`(5s)を超えて
    NIS>13.8 の棄却が続き、かつ観測が norm/z/tilt/凍結ゲートを通過している
    (=NIS だけで弾かれている)とき、棄却の代わりに制限付き更新を行う:
    (1) R を (NIS/13.8) 倍に膨張してゲイン減衰、(2) b_g/b_mx/b_my のゲインは
    ゼロ(大乖離をバイアスに吸収させない。状態・共分散とも同じ制限ゲイン)、
    (3) Δψ を `FF_EKF_RECAPTURE_MAX_STEP_RAD`(3°)/更新にクランプ
    (磁気更新は実効10Hz→最大30°/s の引き込み)。time_since_accept は通常受理
    でのみリセットされるため、引き込みが進んで NIS が下がれば自然に通常経路へ
    復帰する。報告される nis 値は膨張前の基本 R での値。
  - **B-2 地上自動再アンカー**: `ffAnchorService` 内、モーターOFF・B0 窓 full・
    FF補正有効・アンカー有効で、EKF の time_since_accept が
    `FF_EKF_AUTO_REANCHOR_AFTER_S`(10s)を超えていたら
    `ffFreezeAnchorFromWindow()` を自動発動(NIS棄却・b_m凍結の継続に対する
    最終救済)。`FF_EKF_AUTO_REANCHOR_COOLDOWN_S`(30s)のクールダウンで連発
    防止。発動時は LOG_TEXT「EKF自動再アンカー(NIS棄却継続のため)」を送出。
- **再アンカー ψ0 の選択(2026-07改修C)**: `ffFreezeAnchorFromWindow()` は
  EKF が健全(FF補正有効 && anchor_valid && time_since_accept <
  `FF_EKF_ANCHOR_PSI0_FRESH_S`(1s) && nis<5.99 && 非凍結)なら anchor_psi0 に
  **EKF の現在 ψ**、そうでなければ従来どおり**リファレンスCF の yaw** を使う。
  停止直後のリファレンスCF はリキャプチャ上限 2°/更新の引き込み遅れで過渡誤差
  (7/10 実測 −15° 前後)を持ち得るため、健全時は EKF 継続でスナップ≈0 とし、
  EKF が壊れているときだけ CF で矯正する。ブート初回は EKF 未アンカーで必ず CF。
  B-2 発動時は time_since_accept>10s のため必ず CF 側になる。
- 定数(`yaw_config.hpp`、rad系に換算して定義): q_ψ=5e-4 deg²/s, q_bg=1e-8 (°/s)²/s,
  q_bm=0.02 µT²/s, τ_bm=120s, P0=diag((10°)²,(0.5°/s)²,(4µT)²,(4µT)²),
  R_base=4.0µT², σ_rz=3.5µT, κ_ff=0.03, τ_resid=0.05s, σ_diff係数=0.3×30µT/A。

### 5.6 テレメトリ(V2: TLM_STATE 末尾拡張。yaw側の短縮キーとの対応)

既存フィールドは不変(末尾追加のみ)。正典は PROTOCOL.md の TLM_STATE 135B 表。

| yaw側キー | V2 TLM_STATE フィールド(オフセット) | 内容 |
|---|---|---|
| `y` | `yaw_est_rad`(97) | アクティブ推定器の融合yaw(off時=リファレンスCFと同値) |
| `yu` | —(V2では非搭載。比較は `yaw`=Madgwick と `yaw_gyro_int_rad`(101)で行う) | リファレンス系 |
| `fdx,fdy` | `db_hat_x_ut`(113), `db_hat_y_ut`(117) | 適用中の ΔB̂ x/y(off時0) |
| `fbx,fby` | `bm_x_ut`(121), `bm_y_ut`(125) | EKF b_m(CF時0) |
| `fns` | `nis`(129) | 直近更新のNIS |
| `ffg` | `ffg`(133) | ゲート状態bit(§5.5) |
| `ffm,fes,fcv,ffa` | `ff_status`(134): bit0-1 ff_mode / bit2 est_mode / bit4 ffcal_loaded / bit3 anchor_valid(+bit5 yaw_ctrl_active, bit6 mag_fresh) | モード・有効フラグ |
| `cur` | `current_a`(109) | 総電流(20Hz更新) |

### 5.7 コマンド・NVS

- 0x14–0x23 のコマンドは専用リングバッファに積み、400Hz ループが順に処理して
  TLM_ACK を返す(`flight_control.cpp`。ISR/受信コールバックでは処理しない)。
- persistence に ffcal namespace の save/load/clear(§5.1 のキー構成)。
  ブートロード順: mag3d → accel6 → attmount → geomag → yawzero → **ffcal**
  (ARCHITECTURE.md「NVS 永続化一覧」)。

## 6. PCサーバー API(/api/ffprofile、pc_server/app.py + core/ffprofile.py)

GET(ポーリング用) →

```jsonc
{
  "profiles": [{"name":..,"memo":..,"created_at":..,"warnings_count":0}],
  "folders":  ["DroneTest_20260629", ...],      // data/sweep_results/ 直下のサブフォルダ名
  "loose_stems": ["sweep_20260611_171306", ...], // 直下の未分類sweep(ファイル選択モード用)
  "applied": {"name":..,"memo":..,"applied_at":..,"verified":true,"crc":"<hex8>",
              "ff":2,"est":1} | null,
  "busy": false, "message": "..."
}
```

POST actions:
- `extract` `{action, folder}` または `{action, stems:[8], name?, memo?}`:
  `data_analysis/.venv/bin/python`(無ければ `python3`)で `make_ff_profile.py` を
  サブプロセス実行(タイムアウト120s)。成功で `{ok, name, quality, warnings}`。
  出力は適用前に `ff_profile_wire_values` で健全性検証する。
- `apply` `{action, name, ff?:0-2(既定2=方式B), est?:0-1(既定1=EKF), force?:bool}`:
  1. CMD_CAL_GET→TLM_CAL_DATA で現在mag3dを取得し、profile.binding.mag3d と全要素
     絶対誤差 2e-3 以内で照合。不一致は `force` なしなら拒否(差分を返す)。
  2. CMD_FF_BEGIN→CMD_FF_LUT×N→CMD_FF_MOT×4→CMD_FF_AUX→CMD_FF_COMMIT(crc32) を
     ACK待ち(タイムアウト1.0s、リトライ2回 — `serial_link.send_with_ack`、
     server.json の `command_ack_timeout_s` / `command_ack_max_retries`)で逐次送信。
     commit の ACK ロスト時は CAL_GET 読み戻しでCRC一致を確認できれば成功として続行。
  3. CAL_GET 読み戻しで valid_flags bit5 + ff_crc32 照合 → CMD_FF_MODE 送信。
  4. `pc_server/data/ff_state.json` に
     `{applied:{name,memo,applied_at,crc(hex8),ff,est},verified}` を原子的保存。
- `mode` `{action, ff, est}` / `anchor` `{action}` / `delete` `{action, name}` / `status`。
- 排他: スイープ/シーケンスと同じスロット(`calprofile_begin/end` +
  `start_gate`)を共有。スイープ実行中は FF 操作を拒否、FF 操作中はスイープ開始を
  拒否(no-TOCTOU)。

## 7. UI(pc_server/static/)

1. **Experiment タブ「FFプロファイル(電流FF較正)」パネル**: フォルダ `<select>`
   (GET folders)+ name/memo 入力 →「抽出実行」→結果+品質+警告表示。
   プロファイル `<select>` + ff(off/方式A/方式B)・est(相補CF/EKF)ドロップ
   ダウン+「適用」/「モードのみ変更」/「アンカー再取得」/「削除」。
   適用中バナー「FF適用中: <name>(ff=…, est=…)」。
2. **Posture/Position タブの FF クイック欄**: ヨー角制御トグルを ON にすると
   FF プロファイル選択+「適用」+適用中バナーを表示。ff_mode=0 のままヨー角制御
   ON の場合は「FF未適用」警告バッジ(飛行は可能)。
3. ヨー推定モニタ(共通): Madgwick / EKF / ジャイロ積算 /(Positionでは MoCap)の
   数値表示+NIS・ffg・電流・FF状態+EKF健全性バッジ。

## 8. テスト(V2: フェイク serial)

yaw側の fake_drone e2e に代わり、`pc_server/tests/test_ffprofile.py` が
`tests/fakes.py` の FakeDroneResponder(0x14–0x23 に TLM_ACK を返し、
CMD_FF_BEGIN/COMMIT/MODE を TLM_CAL_DATA 状態へ反映して読み戻し検証を模擬。
ACK ドロップ・status 上書きを注入可能)に対して、分割適用・CRC照合・
ACKリトライ・commit ACK ロスト回復(読み戻し救済)・binding 照合(force 含む)・
ff_state.json 永続化を検証する。実機の CRC 再計算照合はファーム側
(ff_calibration)の責務で、テストベクタは protocol/tests が担う。
実行: `cd pc_server && .venv/bin/python -m pytest tests/ -q`。

## 9. 受入基準まとめ

| # | 項目 | 基準 |
|---|---|---|
| 1 | 抽出6/12照合 | results.json と <1e-6 一致 |
| 2 | 抽出付録A再現 | §3.3の許容内 |
| 3 | リプレイ | 6/29スイープ再生でEKF発散なし(静置なのでψ変動<15°)、補正後残差がv2 §2.2オーダー |
| 4 | ファーム | `~/.platformio/penv/bin/pio run -d firmware_stampfly -e release` 成功。プロトコル v2 契約(PROTOCOL.md)維持 |
| 5 | サーバ | `pc_server` の pytest 全パス(フェイク serial での CMD_FF_* 往復・commit ACKロスト回復含む) |
| 6 | UI | サーバー起動で Experiment タブの FF パネル表示・抽出/適用が動作 |
