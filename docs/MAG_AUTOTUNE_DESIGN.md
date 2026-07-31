# 飛行中自動磁気チューニング & 電流FF残差オンライン学習 実装契約書(v1)

作成日: 2026-07-27
位置づけ: **実装契約書**(ワイヤ仕様・NVSスキーマ・推定器仕様の正典。実装完了後、
ワイヤ仕様はPROTOCOL.mdへ、ログ仕様はLOG_STRUCTURE.mdへ反映して本書は設計根拠を残す)。
背景・根拠: `FLIGHT_ANALYSIS_20260727.md`(7/27診断)、`YAW_FUSION_EKF_DESIGN.md`、
`YAW_ESTIMATION_POSITION_ONLY_STUDY.md`。

## 0. 決定事項(ユーザー承認済み 2026-07-27)

| 論点 | 決定 |
|---|---|
| EKF改修方式 | **新推定器 est_mode=2(EKF2)として追加**。現行4状態EKF(est_mode=1)は数式・定数とも無改変で並走(V2契約維持)。EKF2は常時シャドー実行 |
| ヨー基準ソース | **段階切替**: PC側スイッチで MoCap実測ヨー → MoCap位置から計算する移動ベースヨー。ファームはCMD_POS_ERRの基準ヨーを消費するだけでソース非依存 |
| FF残差学習 | **ハイブリッド**: 機上はb_m可観測化(=残差の連続計測)まで。学習回帰はPC側バッチ→着陸後に確認付きコミット |
| フロー | **移植GO(ride-alongのみ)**: StampFly_Telemetry の flow/ 一式を移植。**burst読み成立時のみ有効化**(単発6ms読みは400Hz tickに入らないため飛行中は使わない)。制御・EKF観測への使用は次フェーズ |
| 学習パラメータ永続化 | 機体NVS `magbias`(mag3d/ffcalバインディング非破壊の加算適用)+PC側JSONプロファイル(ローカル実験で再利用) |

## 1. ワイヤ契約(protocol/stampfly_protocol.py が正典。C++/Python両実装で一致必須)

### 1.1 TLM_STATE 拡張: 135B → **184B**(末尾追加のみ、既存オフセット不変)

| offset | 型 | フィールド | 内容 |
|---|---|---|---|
| 135 | f32 | mag_cal_x_ut | b_cal(mag3d適用後・FF補正**前**)機体系X [µT] |
| 139 | f32 | mag_cal_y_ut | 同Y |
| 143 | f32 | mag_cal_z_ut | 同Z |
| 147 | f32 | mag_lev_x_ut | FF補正+magbias+EMA後レベル化水平X(=EKF観測zx)[µT] |
| 151 | f32 | mag_lev_y_ut | 同Y |
| 155 | f32 | ekf2_yaw_rad | EKF2のψ(シャドー中も常時) |
| 159 | f32 | ekf2_bm_x_ut | EKF2のb_mx |
| 163 | f32 | ekf2_bm_y_ut | EKF2のb_my |
| 167 | f32 | ekf2_yaw_innov_rad | 直近ヨー観測イノベーション(未受信時0) |
| 171 | u8 | ekf2_status | bit0 yaw_obs_fresh(<1s) / bit1 yaw_obs_fused(直近0.5s内受理) / bit2 flight_anchor_done / bit3 tau_rw_mode / bit4 bm_frozen / bit5 healthy2 / bit6 yaw_obs_low_trust / bit7 ~~予約~~ yaw_recapture=ヨー観測再捕捉中・制限融合モード(**改訂 2026-07-31**: §2.1 改訂注記参照。細部は実装が正) |
| 172 | u8 | ekf2_gate | EKF2のゲートビット(ffgと同一ビット定義) |
| 173 | f32 | flow_vx_mps | フロー機体系X速度(無効時0) |
| 177 | f32 | flow_vy_mps | 同Y |
| 181 | u8 | flow_squal | SQUAL生値 |
| 182 | u8 | flow_status | bit0 sensor_ok / bit1 burst_ok / bit2 vel_valid / bit3 range_valid / bit4 squal_ok / bit5 init_retry(現実装は恒久無効方針=runtimeリトライなしのため常に0。ビット定義は予約として維持)/ bit6-7 予約 |
| 183 | u8 | flow_dt_ms | フロー読み実測dt [ms](0-255クランプ) |

`assert struct.calcsize == 184`。ff_status **bit7 = est_mode==2**(bit2=EKF1は既存のまま。
bit2/bit7とも0なら相補CF)。

### 1.2 CMD_POS_ERR(0x24, 21B — サイズ・既存フィールド不変)

- flags **bit4 = FLAG_YAW_REF_LOW_TRUST** 新設: 基準ヨーの信頼度低(移動ベースヨー時に
  PCが立てる)→ファームはR_ψを低信頼プリセットに切替。
- `mocap_yaw` フィールドの意味を「MoCap実測ヨー」から「**外部ヨー基準**(ソースはPC側設定:
  実測 or 移動ベース)」へ一般化(名称は互換のため維持)。bit3=基準有効は既存どおり。

### 1.3 CMD_FF_MODE(0x22)

- est_mode 受理範囲 0..1 → **0..2**(2=EKF2)。NVS `ffcal/est` に2を保存可。
- 切替時の再シード規約は既存と同じ(EKF2はアクティブ化時、直近のアクティブ推定器
  ヨーで `reseedYaw`)。

### 1.4 新コマンド(すべてTLM_ACK応答、WAIT/COMPLETE/MOTOR_TESTのみ受理=飛行中NVS書込み禁止)

> **改訂(2026-07-27 実装時)**: 当初案の type 0x24-0x26 は既存の
> CMD_POS_ERR(0x24)/CMD_LED_MODE(0x25)と衝突するため、空き番号へ同順シフトして
> **0x26/0x27/0x28** に確定(payload・動作仕様は不変。全実装は enum 参照で整合済み)。

| type | 名前 | payload | 動作 |
|---|---|---|---|
| 0x26 | CMD_MAGBIAS_SET | `u8 mode(0=clear,1=set), f32 dx,dy,dz` =13B | 学習ハードアイアン残差Δb [µT, b_cal空間] をNVS `magbias` へ保存+即適用。**ff_modeは降格しない**(FF係数はb_cal空間のまま有効)。適用時: アンカー無効化+窓リセット+補正系再シード(mag3d変更時と同パターン、ff_mode降格だけしない) |
| 0x27 | CMD_FLOWCAL_SET | `u8 mode(0=clear,1=set), f32 m00,m01,m10,m11` =**17B** | フロー較正行列 K [counts/rad] をNVS `flowcal`(schema v2)へ保存。K は機体系レート→センサーカウントレートの 2×2 写像(純スケールなら K=diag(kx,ky)、取付回転込みなら K=diag(kx,ky)·R(φ0))。ファーム適用は rate = K⁻¹·counts_rate(ジャイロ補償より前) |

> **改訂(2026-07-31 flowcal 2×2 化)**: CMD_FLOWCAL_SET は当初の 9B
> (`mode, kx, ky`)から 17B の 2×2 行列へ変更。7/31 初飛行の実測
> (FLIGHT_ANALYSIS_20260731.md §5)でスケール誤差 x1.11/y1.26 に加えて
> 取付回転 φ0=-9.4° を検出し、kx/ky の 2 定数では回転を表現できないため。
> 受理条件: 対角 m00/m11 ∈ [100, 2000]、|m01|,|m10| ≤ 2000、
> det(K) ≥ 100²(det≈0 ガード。不成立は invalid_arg)。
| 0x28 | CMD_FLOW_PROBE | `u8 n_cycles(0=既定200)` =1B | PMW3901 burstプローブ実行(モーター停止時のみ、busyで拒否)。結果はLOG_TEXT複数行(agree_ratio等の要約)+ probe成功でburst_okラッチ更新 |

### 1.5 TLM_CAL_DATA(0x34)拡張: 112B → **140B**(末尾追加)

| offset | 型 | フィールド |
|---|---|---|
| 112 | f32×3 | magbias dx,dy,dz [µT] |
| 124 | f32 | flowcal_m00 [counts/rad](旧 kx。K 行列対角) |
| 128 | f32 | flowcal_m11 [counts/rad](旧 ky。同) |
| 132 | f32 | flowcal_m01 [counts/rad](2026-07-31 追加。非対角) |
| 136 | f32 | flowcal_m10 [counts/rad](同) |

valid_flags: **bit6=magbias有効、bit7=flowcal有効** 新設(bit5=ffcalは既存)。

> **改訂(2026-07-31 flowcal 2×2 化)**: 当初の 132B(flowcal kx,ky@124)から
> 140B へ末尾拡張。既存オフセット 124/128 の kx/ky は m00/m11 に改名
> (K 行列の対角成分として意味維持)、m01@132・m10@136 を末尾追加。
> ワイヤ順は **m00, m11, m01, m10**(オフセット互換維持のため行優先ではない)。

## 2. ファームウェア仕様

### 2.1 EKF2(`yaw_estimation/yaw_estimator_kf2.{hpp,cpp}` 新設)

現行 `YawEstimatorKf` のコピーを基に以下を変更(コピー元は無改変):

1. **ヨー擬似観測** `updateYawObs(float psi_meas_rad, bool low_trust, float dt_s)`:
   - H=[1,0,0,0]、y=wrapPi(psi_meas−ψ)。スカラー逐次更新(4状態全てに効かせる —
     b_g/b_mはPの相関経由で可観測化される。これが本体)。
   - R_ψ = (2°)²(通常)/(6°)²(low_trust)。`FF_EKF2_R_PSI_RAD2` / `_LOW_TRUST` を
     yaw_config.hppへ。
   - ゲート: |y|>30°は棄却+カウンタ(連続棄却N≥25でイノベーション値をテレメトリに
     残しつつ融合停止フラグ)。Δψクランプ ±3°/更新(既存 FF_EKF_RECAPTURE_MAX_STEP_RAD 流用)。
     **クランプ発動時は制限付き更新**(2026-07-27レビュー反映): バイアス行K=0+
     ψ行は実効ゲイン Δψ_clamp/y で状態・P更新を整合(磁気recaptureと同流儀。
     stale相関経由のb_g/b_m直撃とP00過収縮を防ぐ)。
   - 呼び出し: 新しいCMD_POS_ERR受信を消費した400Hz tick(実効50Hz)、bit3有効+age<100ms時。

> **改訂(2026-07-31 18:20飛行解析反映 — ヨー観測ソフト再捕捉)**: 当初仕様の
> 融合停止ラッチは解除経路が地上経路(reanchor/reseedYaw)のみで、飛行状態
> 再アンカーの発動条件が yaw_obs_fused を要求するため、**離陸前にラッチが
> 発動すると飛行中に永久復帰不能**(18:20飛行で t=1.0s 発動・融合0%を実証。
> `FLIGHT_ANALYSIS_20260731_1820.md` §2)。本コミットで**飛行中の解除経路
> (ソフト再捕捉)を追加**: ラッチ中もゲート内観測の安定成立を監視し、成立で
> 制限融合モード経由でラッチ解除・融合再開する(磁気ソフト再捕捉と同じ設計思想)。
> 再捕捉(制限融合)モード中は **ekf2_status bit7(旧・予約)= yaw_recapture** で
> 示す。閾値・解除条件・bit7 の正確なセマンティクスは**本コミットの実装
> (yaw_estimator_kf2.cpp / yaw_config.hpp の FF_EKF2_YAW_RECAPTURE_*)が正**。

2. **τ_bm適応**: EKF2はGauss-Markovゼロ回帰を**廃止**(a=1固定)。
   - yaw_obs健全(最終受理<1.0s): q_bm 現行値(ランダムウォーク=b_mは自由に追従)
   - yaw_obs喪失(≥1.0s): q_bm×0.1(準凍結ホールド=学習済みb_mでコースト)
   - ekf2_status bit3にモードを反映。
3. **飛行状態再アンカー**: 1フライト1回。条件: flying遷移後>5s ∧ |alt_est−alt_ref|<0.1m
   が2s継続 ∧ 電流>1.0A ∧ yaw_obs_fused。実行: B0f=飛行中b_corr_filt の2sリング平均
   (sensor_hub_ffに飛行中リング新設)、ψ0←EKF2現在ψ、b_m←0、**P_ψは維持**
   (P0スナップ窓を開かない)、P_bm←(2µT)²。以後norm/zゲート基準はB0f。
   不成立時は地上アンカーのまま(EKF1と同等=悪化しない)。
4. その他のゲート(norm/z/tilt/NIS/ソフト再捕捉)・磁気観測数式・定数は現行EKFと同一。

### 2.2 sensor_hub_ff

- EKF2を常時インスタンス化・**シャドー実行**(FFパイプライン健全時、EKF1と同条件で
  predict/update+ヨー観測)。est_mode==2のときのみ出力がyaw_activeへ。
- healthy2 = 既存 `sensorHubFfEkfHealthy()` 同等(FF有効∧アンカー有効∧非凍結)。
  **yaw_obs鮮度はhealthyに含めない**(MoCap途絶でヨー角制御を落とさない — コースト)。
- magbias適用: `b_corr = b_cal − ΔB̂ − Δb_magbias`(補正系のみ、EMA前)。
- テレメトリ用に b_cal(3軸)とレベル化観測z(2軸)をsensor_stateへ公開。

### 2.3 flight_control

- CMD_POS_ERR: bit4読取→low_trust、mocap_yaw+受信tickをsensor hub入力へ配線
  (consume-onceフラグ)。est_mode=2受理。CMD_MAGBIAS/FLOWCAL/FLOW_PROBEハンドラ
  (リングバッファ経由、既存0x14-0x23と同パターン)。

### 2.4 フロー移植(`src/flow/` — StampFly_Telemetry/Telemetry/firmware/src/flow より)

- pmw3901_driver / flow_hub / flow_config / flow_persistence / pmw3901_burst_probe を移植。
- **読みはburst(ハードCS 13B≈52µs)のみ**。起動時セルフテスト(burst 1回+単発照合、
  モーター停止時)で burst_ok を判定。burst_ok=false ならフロー恒久無効
  (flow_status bit1=0、飛行挙動は完全に従来どおり)。単発6ms読みは飛行中は使わない。
- 読みスロット: 既存20Hz低速スロットと同居しない独立20ms周期(50Hz)。ToF/磁気の
  I2Cとは別バス(SPI)なのでtick内共存可(52µs)。
- 出力はテレメトリ+ログのみ(ride-along)。制御・EKF観測には未接続。
- **【改訂 2026-07-31】counts→レート変換は 2×2 行列適用**: 軸マッピング・符号
  (既定固定)適用後のカウントレートに rate = K⁻¹·counts_rate(ジャイロ補償より
  前)。K⁻¹ は NVS ロード/CMD_FLOWCAL_SET 適用時に一度計算してキャッシュ
  (det≈0 は flowcalMatrixValid が事前拒否+computeInverse の二重ガード)。
- **【2026-07-31 修正】軸マッピング既定は 機体x=−dy / 機体y=+dx**(Telemetry
  drone3 純回転較正で実機検証済みの搭載向き)。移植時に恒等へ誤固定されており、
  フロー較正パネル初回実行が φ0≈−92°(欠落90°+実ひねり)として検出した。
  90°級の入替はマッピング定数が持ち、K 行列は微小回転・スケールのみを担う。
  ※修正前のログ(7/31飛行含む)の tlm_flow_vx/vy は旧軸(90°回り)である点に注意。
- **PC側較正フィット(core/flowcal.py)の距離復元**: ファームの v=trans·d は
  ToF **生スラント距離**を使うが、TLM_STATE の altitude_tof はチルト補正+LPF後。
  フィットでは d_slant = altitude_tof / max(cosφ·cosθ, 0.2) で復元して整合させる
  (未補正だと±25°ロッキング較正で非対角+7.7%・φ0−0.6°の系統誤差 — 回帰テストあり)。

### 2.5 NVS(persistence)

| namespace | キー | 内容 |
|---|---|---|
| `magbias` | schema(u32=1), valid(u8), blob(f32×3), crc(u32) | Δb [µT, b_cal空間]。ブート時CRC照合、破損は自己修復破棄 |
| `flowcal` | schema(u32=**2**), valid(u8), m00,m01,m10,m11(f32×4), crc(u32) | フロー較正行列 K [counts/rad](**改訂 2026-07-31**: v1 の kx,ky 2 定数 → 2×2 行列)。crc は {m00,m01,m10,m11} の f32 列。未設定時は既定 diag(450,450)。**v1 → v2 移行**: ブートロードで schema=1 を検出したら v1 CRC({kx,ky})・値域照合の上 diag(kx,ky) へ移行して v2 で保存し直す(旧キー kx/ky は削除。照合不成立は自己修復破棄) |

ブートロード順: mag3d → accel6 → attmount → geomag → yawzero → ffcal → **magbias → flowcal**。
mag3d変更時: magbiasは旧b_cal空間で無効になるため**連動クリア**(ffcalのff_mode降格と同時)。

## 3. PCサーバー仕様

1. **ヨー基準ソース切替** `yaw_ref_source: off|mocap|motion`(server設定+ランタイムAPI+UI
   トグル)。mocap: 実測ヨー(bit4=0)。motion: 移動ベースヨー(bit4=1)。
   invalid時はbit3を落とす(ファームはコースト)。

> **改訂(2026-07-31 18:20飛行解析反映 — ワイヤ較正適用・アーム相対基準・
> 連続性ゲート)**: 当初実装は CMD_POS_ERR の `mocap_yaw` 欄へ生方位
> (heading×wire_sign)を送っており、**attitude_transform(yaw_sign/yaw_offset
> 較正)が表示・ログ列にしか適用されないバグ**があった(7/31両フライトで
> 真値−88.6°/−90.4°の定数オフセット基準が機体へ届き融合0%。
> `FLIGHT_ANALYSIS_20260731_1820.md` §1)。本コミットで以下に改修:
> - **(a) 較正適用+アーム相対基準**: ワイヤの基準は較正適用済み yaw_true 系列
>   から生成し、アーム検知時の yaw_true を ψ_arm として保持して
>   `wrap(yaw_true − ψ_arm)` を送る。機上ヨーはアームで0リセットされるため
>   フレーム原点が構造的に一致し、当日の絶対オフセット較正への依存を排除する。
> - **(b) 連続性ゲート**: MoCapヨーのフレーム間ジャンプがジャイロ整合閾値を
>   超える観測(実測: 約90°別解グリッチはジャンプ中央値94.9°/行 vs 正常
>   p99=5.4°/行)を棄却する一般ゲートへ、従来の cont_flip(180°専用補正)を
>   拡張。棄却中は bit3 を落とす(ファームはコースト)。棄却フラグはログ
>   `mocap_flip` 列へ(LOG_STRUCTURE.md §14 注記)。
>
> 送信値の定義・閾値・復帰条件の最終仕様は**本コミットの実装
> (pc_server/core/session.py / mocap.py)が正**。
2. **移動ベースヨー計算** `core/motion_yaw.py`: 8s滑動窓LS
   ψ̂=arg Σ A·conj(u_B)。A=因果LPF(2Hz biquad)+2階差分のMoCap位置加速度、
   u_B=テレメトリroll/pitch(遅延90ms補償)。Fisher情報 J=Σ|u_B|² が閾値未満なら
   invalid。50Hz出力。係数・符号規約は `docs/analysis_20260727/scripts/poc_yaw_from_motion.py`
   のPoC実証値に従う。
3. **ロガー**: log_columns_version **5→6**。TLM新列(prefix `tlm_`)+PC側列
   `yaw_ref_source, yaw_ref_sent_rad, yaw_ref_valid, motion_yaw_rad, motion_yaw_J` を追加。
4. **magbiasプロファイル管理** `/api/magbias`(ffprofile APIと同型):
   - `extract {log}`: data_analysis/magbias_learn.py をサブプロセス実行 →
     `pc_server/data/magbias_profiles/<name>.json`
   - `apply {name}`: CMD_MAGBIAS_SET → CAL_GET読み戻し照合 → `magbias_state.json` 記録
   - `clear` / `status`。UI: Experimentタブに小パネル。
5. flowcal適用・flow probe実行のパススルーAPI(同型・小規模)。

### magbiasプロファイルJSON(schema `stampfly_magbias_profile` v1)

```jsonc
{
  "schema": "stampfly_magbias_profile", "version": 1,
  "name": "DroneX_20260727", "created_at": "...",
  "source_logs": ["20260727_164200_position.csv"],
  "delta_b_ut": [dx, dy, dz],            // ハードアイアン残差(b_cal空間)。ヨー励振不足時null
  "hover_residual_ut": {"leveled_xy": [x, y], "z": z, "current_a": 3.2},  // ホバ点残差
  "ff_supplement": {                      // FF電圧/duty回帰(診断・将来のFFモデル拡張用)
    "voltage_coeff_ut_per_v": [..], "duty_coeff": [..], "r2": ..
  },
  "binding": {"mag3d_hash": "sha256:..", "ffcal_crc": "hex8"},  // 依存パラメータ束縛
  "quality": {"yaw_excursion_deg": .., "fit_rms_ut": .., "warnings": [..]}
}
```

## 4. data_analysis

- `magbias_learn.py`: v6ログ→上記プロファイルJSON。ハードアイアンはヨー励振区間の
  ベクトルフィット(励振<45°なら delta_b=null で hover_residual のみ)、FF電圧/duty項は
  b_m時系列×(V, duty, I)回帰。図出力付き。
- `replay/ekf2_replay.py`: EKF2のPythonリプレイ(mag_cal+FFプロファイル+magbiasから
  b_corr再構成→EMA→レベル化→EKF2)。ゲート挙動・ヨー軌跡のA/B検証用。
  姿勢はテレメトリ25Hz補間の近似である旨を明記。

## 5. 段階的ロールアウト(運用)

1. **Phase 0**: 本実装+地上検証(ビルド・テスト・ベンチ)。burstプローブ実機実行。
2. **Phase 1(シャドー)**: est_mode=1のまま飛行。EKF2はログのみ。mocapヨー基準で
   イノベーション・b_m挙動を確認。ekf2 vs ekf1 vs mocap比較(viewer新パネル)。
3. **Phase 2(有効化)**: est_mode=2でヨー角制御。MoCap途絶注入テスト(PC側でbit3を
   意図的に10s落とす)でコースト品質確認。
4. **Phase 3(学習)**: 離陸後±90°〜360°緩回頭マニューバ→magbias_learn→着陸後apply。
   ヨー基準をmotionへ切替えて位置のみ運用を実証。
5. 効果判定指標: EKF2ヨーRMS(vs MoCap)<2°(mocap基準時)/<5°(motion基準時)、
   |b_m|最終値<5µT、magbias適用後の初期b_m立ち上がり縮小。

## 6. 受入基準

| # | 項目 | 基準 |
|---|---|---|
| 1 | ファーム | `pio run -d firmware_stampfly -e release` 成功。既存TLM/CMDオフセット不変 |
| 2 | protocol | 全structサイズassert・round-tripテストパス(184/140B含む。140B は 2026-07-31 flowcal 2×2 化) |
| 3 | pc_server | pytest全パス(magbias apply・yaw_refソース切替・v6ロガー含む) |
| 4 | viewer | v5/v6両ログを読めて、v6でEKF2比較パネルが出る |
| 5 | 後方互換 | est_mode=0/1の飛行挙動・現行EKF数式/定数は完全不変。フロー無効時(burst不成立)の飛行挙動不変 |
| 6 | リプレイ | ekf2_replayが7/27ログ(v5、ヨー観測なし相当)でEKF1相当挙動を再現 |
