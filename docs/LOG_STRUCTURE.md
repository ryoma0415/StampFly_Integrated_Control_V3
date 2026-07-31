# StampFly Integrated Control — フライトログ形式(v6・136列)

本書は `pc_server/core/logger.py` が生成する CSV の列構成を定義する。
列定義の実体は `logger.py` の `COLUMNS` であり、本書と1対1で対応させること。
flight_log_viewer も同一の 136 列契約を正として読み込む
(旧ログ v1〜v5 との互換は「旧版からの変更履歴」参照)。

v6 は v5 の 115 列を**不変のまま**、末尾に磁気オートチューン/フロー拡張
21 列を追加した(§15-16。MAG_AUTOTUNE_DESIGN.md §1.1 / §3)。
`*.meta.json` の `log_columns_version` は 6 になり、ログ開始時点の
ヨー基準ソース(`yaw_ref_source`)も記録する。

v5 は v4 の 109 列を**不変のまま**、末尾に MoCap 正解ヨー/生クォータニオン
6 列を追加した(§14)。合わせてログファイルと同名の `*.meta.json`
サイドカーに、記録時に適用されていた MoCap マッピング
(`coordinate_transform` / `attitude_transform`)を保存する — マッピングが
実行時変更可能になったため、「このログがどの座標フレームで記録されたか」を
列契約と独立に恒久記録する。

v4 は従来の「末尾追加のみ」方式をやめた**全面再編**である:
v3(100列)から 14 列を削除し、TLM_CTRL(制御ループ診断テレメトリ、
PROTOCOL.md 0x35)由来の 23 列を追加、全列を論理グループ順に並び替えた。
旧 PC 側 XY PID の廃止(機上XY制御 CMD_POS_ERR への一本化)に伴う削除を含む。

## 出力先・行レート

- 出力先: リポジトリ直下 `logs/flight_logs/YYYYMMDD_HHMMSS_<mode>.csv`
  (`<mode>` = `posture` | `position`)。複数機モードでは機体ごとに
  `YYYYMMDD_HHMMSS_multi_<機体名>.csv`(全機同一タイムスタンプ。機体名の
  英数と `-`/`_` 以外は `-` に置換)。
- ログの寿命は **START(CMD_START 受理)〜飛行終了(着陸/START 猶予切れ/
  切断)**。トグル ON は「次の飛行からの予約」で、飛行終了時にファイルを
  閉じてトグルは自動 OFF になる。飛行中(armed/flying)の ON は即記録開始、
  OFF は即閉じ。experiment モードでは飛行ログは記録されない
  (計測は別系統の explog、README §5.5)。
- 記録中は 50Hz 送信(Posture: CMD_SETPOINT / Position・Multi: CMD_POS_ERR。
  複数機は各機それぞれ)ごとに1行を出力する。
- 各行には「その送信時点での最新テレメトリ/mocap スナップショット」を結合する。
  テレメトリは TLM_STATE(25Hz)と TLM_CTRL(25Hz、TLM_STATE と位相ずらし)の
  2 系統で、いずれも行レート(50Hz)より遅い。**連続する複数行が同じ
  スナップショットを共有し得る**(`tlm_age_ms` / `tlm_ctrl_age_ms` で判別する)。

## 書式と書き込み挙動(`logger.py` 実装)

- 列数は 136(1行目がヘッダ)。順序は `COLUMNS` の宣言順(下の列リファレンスの
  掲載順と同一)。
- float は小数 6 桁の固定表記(`FLOAT_DECIMALS = 6`)。bool は `"1"`/`"0"`、
  `None`(未取得)は空文字で出力される。
- `timestamp` / `elapsed_time` は `log_row()` が自動付与する(呼び出し側が
  明示指定した場合はその値が優先)。
- ディスクへのフラッシュは 50 行ごと(`server.json` の
  `logging.flush_every_rows`。50Hz なら約1秒ごと)。異常終了時は最大で
  直近1秒ぶんの行が失われ得る。
- ファイルのライフサイクル: ログトグル ON(予約)状態で START(CMD_START
  受理)されたときに開かれ、飛行終了(着陸イベント / START 猶予切れ /
  再武装解除)・トグル OFF・切断・サーバ終了で閉じる。飛行終了で閉じた
  ときはトグルも自動 OFF になる(1ファイル = 1飛行)。
- **プリロール(P2b、2026-07-31 追加)**: ログ予約 ON の間はファイル閉鎖中も
  50Hz 行を組み立ててリングバッファに保持し(直近 `logging.pre_roll_s`、
  既定 3.0s)、START でファイル先頭へフラッシュする — **アーム前区間
  (地上アンカー・I_idle・Δb_z 復元に必須)が必ずログに入る**。
  `elapsed_time` の基点(0)はプリロール先頭行で、START トリガー時点は
  `*.meta.json` の `pre_roll_s`(トリガーまでの秒数。プリロール行なしなら
  0)から求める(単機のみ。複数機は従来どおり開始時点から)。
- 行が記録されるのは 50Hz 送信スレッドの動作中(=シリアル接続中)のみ。
  送信失敗時も行は出力され、`send_success=0`・`command_sequence` 空となる。
- 複数機モードでは機体ごとの FlightLogger に `mode="multi"` で記録される。
  全機が着陸(非飛行 phase)になった時点で全ファイルを閉じて自動 OFF。
  stop_all / 緊急停止後も着陸検知までは記録が続く。

## 座標系と単位

- 位置(`pos_*`, `raw_pos_*`, `target_*`)は Motive 座標を制御座標系へ変換した後の値 [m]。
- **送信指令の `cmd_err_x_m` / `cmd_err_y_m` は「実際に機体へ送ったワイヤ値」**
  (機体ワイヤフレーム)。右手系マッピング適用中は `cmd_err_y_m = -error_y`
  となる(機上XY制御の符号規約がレガシーフレーム前提のため、送信境界で
  自動変換する — 2026-07 導入)。`error_x`/`error_y` 列は従来どおり
  制御座標系。適用中マッピングは同名 `*.meta.json` を参照。
  変換は `config/control.json` の `coordinate_transform`(既定: 制御x←Motive z、
  制御y←−Motive x、制御z←Motive y)。旧システムの y 軸負ゲイン規約は廃止し、
  符号はこの座標変換に移した(そのため y 座標の符号は旧ログと反転している)。
- `_rad` はラジアン、`_deg` は度。`_rad_s` は rad/s。
- 時間系(`*_ms`)はミリ秒。`elapsed_time` は秒(time.monotonic 基準)。
- 0/1 フラグは文字列 `"0"`/`"1"`。値が未取得の列は空文字。

## 列リファレンス(v6・136列。ファイル上の並び順)

各グループ見出しの「出所」は値の生成元:
**PC** = pc_server(session / multi の送信・ログ組み立て)、
**meta** = PositionController の診断辞書(Position/Multi のみ。Posture では空)、
**TLM_STATE** = 最新 TLM_STATE スナップショット(25Hz。未受信時は空)、
**TLM_CTRL** = 最新 TLM_CTRL スナップショット(25Hz。未受信時は空。
旧ファームでは常に空)。

### 1. セッション / タイミング(7列、出所: PC)

| 列 | 型 | 説明 |
| --- | --- | --- |
| `timestamp` | ISO8601文字列 | 行の記録時刻(ローカル壁時計、ミリ秒精度)。 |
| `elapsed_time` | float (s) | ログ開始からの経過秒(monotonic)。 |
| `mode` | string | `posture` / `position` / `multi`。 |
| `phase` | string | セッションフェーズ(`idle`/`connected`/`armed`/`flying`。Multi は当該スロットの phase)。 |
| `command_sequence` | int | この行で送信した CMD_SETPOINT / CMD_POS_ERR の seq(送信失敗時は空)。 |
| `send_success` | 0/1 | シリアル書き込み成功フラグ。 |
| `feedback_latency_ms` | float (ms) | TLM_STATE の `seq_echo` から計測した直近の往復遅延(Multi はノード別)。未計測時は空。 |

### 2. 目標と位置(11列、出所: meta。制御座標系 m。Posture では空)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `target_x`, `target_y`, `target_z` | float | m | 目標位置(z は目標高度の指令元。円軌道中は時間更新された軌道目標)。 |
| `pos_x`, `pos_y`, `pos_z` | float | m | 制御に使用したフィルタ後位置。 |
| `raw_pos_x`, `raw_pos_y`, `raw_pos_z` | float | m | フィルタへ入力したリジッドボディ生位置。 |
| `error_x`, `error_y` | float | m | 目標位置に対する XY 誤差(クランプ前)。 |

### 3. 送信指令(PC→機体)(10列、出所: PC)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `cmd_err_x_m`, `cmd_err_y_m` | float | m | 送信した XY 位置誤差(CMD_POS_ERR。制御座標系、±`control.pos_err_clamp_m` クランプ後)。Position/Multi のみ。bit2=0(無効)中も実誤差を送る。 |
| `cmd_xy_valid` | 0/1 | - | flags bit2(FLAG_XY_ERR_VALID)を立てて送信したか(閉ループ有効+データ有効+MoCap 非途絶)。Position/Multi のみ。 |
| `roll_ref_rad`, `pitch_ref_rad` | float | rad | 送信した角度指令(**機体バイアス加算後**)。Posture のみ。Position/Multi は角度指令を送らないため 0(機体側計算値は `tlm_roll_ref_rad`/`tlm_pitch_ref_rad` に現れる)。 |
| `alt_ref_m` | float | m | 送信した目標高度。 |
| `cmd_yaw_ref_rad` | float | rad | 送信したヨー角目標(整形済み)。ヨー制御 OFF 時は 0。 |
| `yaw_ctrl_on` | 0/1 | - | flags bit1(FLAG_YAW_REF_VALID)を立てて送信したか(=ヨー角制御 ON)。 |
| `roll_bias_deg`, `pitch_bias_deg` | float | deg | 適用中の機体プロファイルバイアス。**Posture の角度指令にのみ加算**される(Position/Multi では角度指令自体を送らないため非加算。参考値として記録)。 |

### 4. 機体実測: 姿勢・角速度・加速度(10列、出所: TLM_STATE)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_age_ms` | float | ms | TLM_STATE 受信からこの行までの経過(鮮度判定用)。 |
| `tlm_roll_rad`, `tlm_pitch_rad`, `tlm_yaw_rad` | float | rad | 実測姿勢(Madgwick AHRS)。yaw は ±π・リセット時 0(旧ファームのログは [-2π, 0]。比較時は wrap で正規化する)。 |
| `tlm_p_rad_s`, `tlm_q_rad_s`, `tlm_r_rad_s` | float | rad/s | 実測角速度(角速度ループの制御量。`tlm_*_rate_ref_rad_s` との比較で内ループ追従が見える)。 |
| `tlm_ax_g`, `tlm_ay_g`, `tlm_az_g` | float | g | フィルタ後加速度。 |

### 5. 機体計算指令(8列、出所: TLM_STATE + TLM_CTRL)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_roll_ref_rad`, `tlm_pitch_ref_rad` | float | rad | 機体側で適用中の角度指令(TLM_STATE エコー)。Posture では送信指令の往復確認、Position/Multi では**機上 XY PID の出力**(UI「機体計算指令」の表示元)。 |
| `tlm_yaw_ref_rad` | float | rad | 機体側で適用中のヨー目標(途絶ラッチ後含む。ヨー制御 off 時 0)。 |
| `tlm_ctrl_age_ms` | float | ms | TLM_CTRL 受信からこの行までの経過(`tlm_age_ms` と同方式)。TLM_CTRL は 25Hz のため連続 2 行が同じスナップショットを共有し得る。 |
| `tlm_ctrl_flags` | u8 | - | TLM_CTRL flags。**bit0 = xy_onboard_active**(CMD_POS_ERR 経路で機上XY指令生成中)、**bit1 = yaw_ctrl_active**(ヨー角制御アクティブ)、**bit2 = flying**。PID 成分の有効区間判別に使う(下記)。 |
| `tlm_roll_rate_ref_rad_s`, `tlm_pitch_rate_ref_rad_s` | float | rad/s | 角度ループ出力=roll/pitch 指令角速度(角速度ループの目標)。 |
| `tlm_yaw_rate_ref_rad_s` | float | rad/s | psi_pid 出力(±`yaw_rate_limit_rad_s` クランプ**後**)のヨー指令角速度。 |

### 6. 姿勢PID成分(18列、出所: TLM_CTRL)

角度ループ(phi/theta/psi_pid)9 列+角速度ループ(p/q/r_pid)9 列。
成分の定義はファーム `PID::update()` の合成式
`m_kp*(err + m_integral + m_differential)` の3分解、すなわち
**P = kp·err、I = kp·integral、D = kp·differential(P+I+D = そのPIDの出力)**。

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_pid_roll_ang_{p,i,d}` | float | rad/s 相当 | 角度ループ roll(phi_pid)の P/I/D 成分。合計=roll 指令角速度。 |
| `tlm_pid_pitch_ang_{p,i,d}` | float | rad/s 相当 | 同 pitch(theta_pid)。 |
| `tlm_pid_yaw_ang_{p,i,d}` | float | rad/s 相当 | 同 yaw(psi_pid)。**クランプ前**の値(`tlm_yaw_rate_ref_rad_s` はクランプ後 → 差でクランプ発動が分かる)。 |
| `tlm_pid_roll_rate_{p,i,d}` | float | duty 相当 | 角速度ループ roll(p_pid)の P/I/D 成分。合計=ミキサへのロールモーメント指令。 |
| `tlm_pid_pitch_rate_{p,i,d}` | float | duty 相当 | 同 pitch(q_pid)。 |
| `tlm_pid_yaw_rate_{p,i,d}` | float | duty 相当 | 同 yaw(r_pid)。 |

**PID リセット中は成分が 0 になる**(非飛行時の全 PID リセット、ヨー角制御
OFF 時の psi_pid 毎 tick リセット等)。有効区間は `tlm_ctrl_flags`
(bit2=flying、bit1=yaw_ctrl_active)で判別する設計である。

### 7. 高度系(5列、出所: TLM_STATE)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_alt_ref_m` | float | m | 機体側で適用中の目標高度。 |
| `tlm_altitude_tof_m` | float | m | ToF 距離(チルト補正済み: slant·cosφ·cosθ。チルト>0.70rad は直前値保持)。 |
| `tlm_altitude_est_m` | float | m | カルマン推定高度。 |
| `tlm_alt_velocity_m_s` | float | m/s | 高度速度。 |
| `tlm_z_dot_ref_m_s` | float | m/s | 高度速度指令。 |

### 8. ヨー推定・FF診断(10列、出所: TLM_STATE)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_yaw_est_rad` | float | rad | アクティブ推定器ヨー(est_mode=1 なら EKF ψ、0 なら補正CF。ff/est 未設定時はリファレンスCF)。 |
| `tlm_yaw_gyro_int_rad` | float | rad | Z軸角速度の単純積算(400Hz、ahrs_reset でゼロクリア)。 |
| `tlm_current_a` | float | A | 総電流(INA3221 CH2、20Hz 更新)。 |
| `tlm_db_hat_x_ut`, `tlm_db_hat_y_ut` | float | µT | 適用中の FF 補正ベクトル ΔB̂ の x/y。 |
| `tlm_bm_x_ut`, `tlm_bm_y_ut` | float | µT | EKF 磁気バイアス状態 b_m の x/y。 |
| `tlm_nis` | float | - | 直近 EKF 更新の NIS。 |
| `tlm_ffg` | u8 | - | EKF ゲート/健全性ビット(PROTOCOL.md の ffg 定義参照)。 |
| `tlm_ff_status` | u8 | - | bit0-1 ff_mode, bit2 est_mode(EKF), bit3 anchor_valid, bit4 ffcal_loaded, bit5 yaw_ctrl_active, bit6 mag_fresh。 |

### 9. モータ・電源(5列、出所: TLM_STATE)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_duty_fr`, `tlm_duty_fl`, `tlm_duty_rr`, `tlm_duty_rl` | float | 0–1 | モータデューティ。 |
| `tlm_voltage_v` | float | V | バッテリ電圧(INA3221 の 20Hz 読み)。 |

### 10. 機体状態・システム(6列、出所: TLM_STATE)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_state` | u8 | - | FlightState 数値(0=INIT … 6=COMPLETE, 7=MOTOR_TEST)。名前表記列(v3 の `tlm_state_name`)は v4 で廃止(名前は PROTOCOL.md の enum 表を参照)。 |
| `tlm_flags` | u8 | - | bit0 low_voltage, bit1 setpoint_fresh, bit2 flying。 |
| `tlm_reason` | u8 | - | 直近の遷移理由(Reason 数値。名前表記列は v4 で廃止)。 |
| `tlm_seq_echo` | u32 | - | 機体が最後に適用したセットポイント系コマンドの seq。 |
| `tlm_elapsed_ms` | u32 | ms | 機体起動からの経過(TLM_CTRL の elapsed_ms と同一クロック。行突合用)。 |
| `tlm_loop_dt_us` | u16 | µs | 機体の直近制御周期実測値。 |

### 11. MoCap 実測ヨー・軌道(4列、出所: meta。Posture では空)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `mocap_yaw_deg` | float | deg | MoCap リジッドボディのヨー真値(mocap.py の yaw_rad を deg 換算。比較用)。 |
| `mocap_heading_deg` | float | deg | MoCap 実測の制御座標系ヨー(リジッドボディ前方軸の方位)。CMD_POS_ERR の mocap_yaw 欄(機上ヨー回転補償)に使った値で、機体推定ヨー(`tlm_yaw_est_rad`)とのフレーム整合検証に使う。heading 未取得時は空欄。 |
| `traj_mode` | int | - | 軌道モード: hover=0 / circle=1。 |
| `traj_phase_rad` | float | rad | 円軌道の現在位相(±π)。**hover 時は空欄**。MoCap 途絶中は位相凍結のため直近値のまま。 |

### 12. フィルタ状態(10列、出所: meta。Posture では空)

| 列 | 型 | 説明 |
| --- | --- | --- |
| `data_valid` | 0/1 | 閉ループ更新に有効と判断されたか(信頼度・外れ値・トラッキング・フレーム間隔・再シード検疫を考慮)。 |
| `control_active` | 0/1 | XY 閉ループが有効か(Start 受理後のみ 1)。 |
| `mocap_dropout` | 0/1 | MoCap 途絶(>300ms)により XY 無効(CMD_POS_ERR bit2=0)中か。 |
| `is_outlier` | 0/1 | フィルタが外れ値と判定したか。tracking_valid=0 のフレームは距離判定をせず欠測扱い(0 のまま予測を出力)。 |
| `used_prediction` | 0/1 | 生データではなく予測位置を使用したか(外れ値時・トラッキング喪失時。外挿は max_prediction_s で頭打ち)。 |
| `confidence` | float (0-1) | フィルタの信頼度スコア。再シード後の検疫中(reseed_probation_frames)は受理行でも劣化値(≤0.35)になる。 |
| `consecutive_outliers` | int | 連続外れ値フレーム数。強制再シード(2026-07 のロックアウト対策)で 0 に戻るため、**`is_outlier`=1 かつ本列=0 の行は再シードフレーム**を意味する。 |
| `data_source` | string | `"rigid_body"` または `"none"`(本システムはリジッドボディのみ。マーカー重心フォールバックは廃止)。 |
| `filter_threshold` | float (m) | 外れ値判定に使った動的距離しきい値。 |
| `tracking_valid` | 0/1 | Motive の `tracking_valid`。 |

### 13. リジッドボディ / フレーム診断(5列、出所: meta。Posture では空)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `rb_error` | float | - | NatNet が報告するリジッドボディ解法エラー。 |
| `rb_marker_count` | int | count | リジッドボディ解法に寄与したマーカー数(v3 まで重複していた `marker_count` は v4 で本列に一本化)。 |
| `frame_number` | int | - | 最新の NatNet フレーム番号。 |
| `frame_dt_ms` | float | ms | 連続する NatNet フレーム間の時間差。 |
| `mocap_age_ms` | float | ms | 最後の有効 pose からこの行までの経過。 |

### 14. MoCap 正解ヨー・生クォータニオン(6列、出所: meta。v5 追加。Posture では空)

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `mocap_yaw_true_deg` | float | deg | **正解ヨー**。前方軸の制御座標方位に `attitude_transform` の符号・オフセットを適用し (-180, 180] にラップし、**ヨー連続性フィルタ**(2026-07-31 追加、`pc_server/core/yaw_continuity.py`)を通した値。棄却中(bit1 参照)はジャイロ伝播予測でコーストする。ビューアの「MoCap 正解Yaw」系列・ヨー統計の基準はこの列で、**CMD_POS_ERR のヨー基準ワイヤ経路と単一情報源**(旧 `mocap_yaw_deg` は Z-up 前提オイラー分解で Motive が Y-up の場合は機首方位ではない — 2026-07-27 実測確定。診断用に残置)。 |
| `mocap_flip` | int | bit | フリップ補正フラグ。bit0 = 上方軸反転補正(マーカー対称性による前方軸まわり180°別解)、bit1 = **ヨー連続性フィルタが棄却中**(出力はジャイロ伝播コースト)。0 = 補正/棄却なし。**【改訂 2026-07-31】意味拡張**: 旧実装の bit1 は「継続性ガードの180°トグル補正中」だったが、約90°別解グリッチ実測(18:20 飛行、`FLIGHT_ANALYSIS_20260731_1820.md`)を受け、180°フリップも90°グリッチも同一機構で棄却する連続性フィルタ(ジャイロ予測 ±30° ゲート、棄却 >5s で計測へ再整列)へ統合し、bit1 の意味を**連続性棄却フラグ**に拡張(ビット位置・列名は互換維持)。旧ログ(7/31以前)の bit1 は180°補正のみで、18:20飛行では90°グリッチへの誤180°補正を含む点に注意。 |
| `mocap_qx` | float | - | Motive 生クォータニオン x(NatNet unpack 順)。 |
| `mocap_qy` | float | - | 同 y。 |
| `mocap_qz` | float | - | 同 z。 |
| `mocap_qw` | float | - | 同 w。**生クォータニオンがあれば任意のマッピングを事後再計算できる**(軸ずれ調査の再発防止)。 |

### 15. 磁気オートチューン/フロー拡張(16列、出所: TLM_STATE。v6 追加)

TLM_STATE 135B→184B 拡張(MAG_AUTOTUNE_DESIGN.md §1.1 / PROTOCOL.md)の
全フィールド。旧ファーム(135B TLM_STATE)との組み合わせでは受信自体が
拒否されるため、これらの列は空にならない(v6 ログ = 184B ファーム)。

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `tlm_mag_cal_x_ut`, `tlm_mag_cal_y_ut`, `tlm_mag_cal_z_ut` | float | µT | b_cal(mag3d 適用後・FF 補正**前**)機体系磁場。リプレイ・バッチ較正の入力。 |
| `tlm_mag_lev_x_ut`, `tlm_mag_lev_y_ut` | float | µT | FF補正+magbias+EMA 後のレベル化水平磁場(= EKF 観測 zx, zy)。 |
| `tlm_ekf2_yaw_rad` | float | rad | EKF2 の ψ(est_mode≠2 のシャドー実行中も常時)。 |
| `tlm_ekf2_bm_x_ut`, `tlm_ekf2_bm_y_ut` | float | µT | EKF2 の磁気バイアス状態 b_m。 |
| `tlm_ekf2_yaw_innov_rad` | float | rad | 直近ヨー観測イノベーション(未受信時 0)。 |
| `tlm_ekf2_status` | u8 | bit | bit0 yaw_obs_fresh / bit1 yaw_obs_fused / bit2 flight_anchor_done / bit3 tau_rw_mode / bit4 bm_frozen / bit5 healthy2 / bit6 yaw_obs_low_trust。 |
| `tlm_ekf2_gate` | u8 | bit | EKF2 のゲートビット(`tlm_ffg` と同一ビット定義)。 |
| `tlm_flow_vx_mps`, `tlm_flow_vy_mps` | float | m/s | フロー機体系速度(無効時 0。ride-along — 制御未接続)。 |
| `tlm_flow_squal` | u8 | - | PMW3901 SQUAL 生値。 |
| `tlm_flow_status` | u8 | bit | bit0 sensor_ok / bit1 burst_ok / bit2 vel_valid / bit3 range_valid / bit4 squal_ok / bit5 init_retry。 |
| `tlm_flow_dt_ms` | u8 | ms | フロー読み実測 dt(0-255 クランプ)。 |

### 16. ヨー基準ソース・移動ベースヨー(5列、出所: PC。v6 追加。Position のみ)

CMD_POS_ERR の `mocap_yaw`(外部ヨー基準)欄の供給状態と、PC 側で常時計算
している移動ベースヨー(`core/motion_yaw.py`。MAG_AUTOTUNE_DESIGN.md §3.1-2)。

| 列 | 型 | 単位 | 説明 |
| --- | --- | --- | --- |
| `yaw_ref_source` | string | - | ヨー基準ソース(`off` / `mocap` / `motion`。行単位 — 飛行中の切替も追える)。 |
| `yaw_ref_sent_rad` | float | rad | CMD_POS_ERR の `mocap_yaw` 欄に実際に送った値(ワイヤ規約)。bit3=0 のときは 0。**【改訂 2026-07-31 P0-1】** mocap ソースの送信値は `heading×wire_sign`(較正が乗らない旧経路 — 7/31 の基準鏡像化事故の原因)から、**連続性フィルタ後の正解ヨー(`mocap_yaw_true_deg` と同一情報源)+整列方式**へ変更: `align=arm`(既定)は `wrap(yaw_true + anchor)`(anchor はアーム遷移時に機体ヨー `tlm_yaw_est_rad` へ整列 — 値は meta.json の `yaw_ref.anchor` に記録)、`align=absolute` は `yaw_true` そのまま。連続性棄却中・MoCap 途絶中・アンカー未ラッチ時は bit3=0。 |
| `yaw_ref_valid` | 0/1 | - | flags bit3(FLAG_MOCAP_YAW_VALID)を立てて送信したか。 |
| `motion_yaw_rad` | float | rad | 移動ベースヨー推定(ワイヤ規約変換済み — `yaw_ref_sent_rad` と直接比較可能)。ソースが mocap の間も常時記録する(切替前の妥当性検証用)。推定 invalid(励振不足)時は空。 |
| `motion_yaw_J` | float | (m/s²)²·samples | Fisher情報 Σ\|u_B\|²(8s 窓)。ゲート閾値は `control.json` の `motion_yaw.j_min`。 |

## TLM_CTRL スナップショットの注意(25Hz)

- TLM_CTRL は 25Hz(400Hz ループの 16 分周、TLM_STATE から 4 tick 位相ずらし)
  で全飛行状態で常時送出される。行レート(50Hz)より遅いため、**連続する
  2 行が同じ TLM_CTRL スナップショットを共有し得る**。重複除去や微分をする
  解析では `tlm_ctrl_age_ms`(または `tlm_elapsed_ms` との突合)で
  スナップショット更新を判別すること。
- `tlm_ctrl_flags` のビット定義: bit0 = xy_onboard_active、
  bit1 = yaw_ctrl_active、bit2 = flying。**PID 成分はリセット中 0** になるため、
  非飛行区間・ヨー制御 OFF 区間の成分値はこのフラグでマスクして扱う
  (flight_log_viewer の PID 成分図は自動で網掛け表示する)。
- 旧ファーム(TLM_CTRL 非対応)と組み合わせた場合、TLM_CTRL 由来の 23 列は
  すべて空欄のままになる(行自体は記録される)。

## 旧版からの変更履歴

- **v1(77列)**: 初版。`NatNet_PID_Controller/LOG_STRUCTURE.md`(57列)の語彙を
  継承し、TLM_STATE 全フィールドの `tlm_*` スナップショット列で拡張した。
- **v2(94列)**: 末尾に 17 列追加 — ヨー指令 3 列(`cmd_yaw_ref_rad` /
  `cmd_yaw_ref_deg` / `yaw_ctrl_on`)、TLM_STATE 末尾拡張(ヨー推定/FF 診断)
  11 列、MoCap ヨー・軌道 3 列。
- **v3(100列)**: 末尾に 6 列追加 — 機上XY制御(CMD_POS_ERR)診断
  (`xy_cmd_mode` / `cmd_err_x_m` / `cmd_err_y_m` / `cmd_xy_valid` /
  `cmd_mocap_yaw_deg` / `mocap_heading_deg`)。
- **v4(109列)**: 全面再編。
  - **削除(14列)**: deg 重複列 `roll_ref_deg` / `pitch_ref_deg` /
    `cmd_yaw_ref_deg`(rad 列から導出可能)、`marker_count`
    (`rb_marker_count` と重複)、`tlm_state_name` / `tlm_reason_name`
    (数値列+PROTOCOL.md の enum 表で足りる)、
    `cmd_mocap_yaw_deg`(`mocap_heading_deg` と重複)、
    PC 側 XY PID 成分 6 列(`pid_x_p/i/d`, `pid_y_p/i/d` — 旧 PC 側 XY PID
    経路の削除に伴う)、`xy_cmd_mode`(機上XY制御へ一本化されモード自体が消滅)。
  - **追加(23列)**: `tlm_ctrl_age_ms`, `tlm_ctrl_flags`,
    `tlm_{roll,pitch,yaw}_rate_ref_rad_s`(3)、
    `tlm_pid_{roll,pitch,yaw}_ang_{p,i,d}`(9)、
    `tlm_pid_{roll,pitch,yaw}_rate_{p,i,d}`(9)— いずれも TLM_CTRL 由来。
  - **並び替え**: 追記順を廃し、セッション → 目標/位置 → 送信指令 → 機体実測 →
    機体計算指令 → PID 成分 → 高度 → ヨー推定 → モータ/電源 → 状態 →
    MoCap/軌道 → フィルタ → RB 診断 の論理順にした。
- **v5(115列)**: 末尾追加のみ — MoCap 正解ヨー・生クォータニオン
  6 列(`mocap_yaw_true_deg` / `mocap_flip` / `mocap_qx/qy/qz/qw`、§14)。
  同時にログと同名の `*.meta.json` サイドカー(適用中 MoCap マッピングの
  記録)を導入。v4 の 109 列は順序・意味とも不変のため、旧ツールは先頭
  109 列をそのまま読める。
- **v6(136列、本版)**: 末尾追加のみ — 磁気オートチューン/フロー拡張
  21 列(TLM_STATE 184B 拡張の `tlm_*` 16 列 §15+PC 側ヨー基準/
  移動ベースヨー 5 列 §16。MAG_AUTOTUNE_DESIGN.md §1.1 / §3)。
  `*.meta.json` の `log_columns_version` は 6、ログ開始時点の
  `yaw_ref_source` も記録する。v5 の 115 列は順序・意味とも不変のため、
  旧ツールは先頭 115 列をそのまま読める。
- **旧ログ互換(flight_log_viewer)**: `viewer/loader.py` は列の過不足を警告のみで
  続行する(必須列は `elapsed_time` のみ)。加えて v4 で廃止された
  `roll_ref_deg` / `pitch_ref_deg` / `cmd_yaw_ref_deg` は「CSV に列があれば
  それを使用、無ければ `*_rad` 列から派生生成」するため、v1〜v3 のログは
  そのまま読める。`marker_count` 参照図は `rb_marker_count` を優先し、
  無ければ旧 `marker_count` にフォールバックする。

## 旧形式(V1 以前のシステム)からの主な変更点

- `feedback_roll_rad`/`feedback_pitch_rad` 等の `feedback_*` 列は、TLM_STATE 全体を
  `tlm_*` 列として記録する形式に置き換えた(`tlm_roll_ref_rad`/`tlm_pitch_ref_rad` が
  旧 feedback 列に対応)。
- `feedback_sequence` → `tlm_seq_echo`、`feedback_age_ms` → `tlm_age_ms`。
- `loop_time_ms` は廃止(行レートは送信スレッドの 50Hz 固定。機体側の実周期は
  `tlm_loop_dt_us` を参照)。
- `rb_pos_*`, `rb_q*`, `rb_roll_deg` 等のリジッドボディ姿勢列は、姿勢が TLM(AHRS)で
  得られるため `rb_error`/`rb_marker_count` のみ残した(mocap yaw は比較・
  フレーム整合検証用)。
- Posture モードでも同一スキーマを使用し、位置系の列は空となる。
