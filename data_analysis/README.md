# data_analysis — FF 係数抽出・計測データ解析ツール

pc_server が取得したスイープ結果 / Experiment 計測ログ / 飛行ログをオフラインで
解析する独立 venv のツール群。plot_sweep / make_ff_profile / plot_explog は
**引数なしで起動すると番号選択の対話モード**、引数を渡すと CLI として動く
(`--help` あり、`q` で中止)。

| スクリプト | 役割 | 入力 | 出力 |
|---|---|---|---|
| `plot_sweep.py` | スイープ 1 本の校正解析(図12枚)+加算性シーケンス検証 | `sweep_*_samples.csv` / `sequence_*_meta.json` | `graphs/sweep_<stamp>/`・`graphs/additivity_<stem>/` |
| `make_ff_profile.py` | スイープ結果 → FF プロファイル JSON 抽出 | スイープ 8 本(または全機 4 本+sequence meta) | `../pc_server/data/ff_profiles/<name>.json` |
| `plot_explog.py` | Experiment 計測ログのグラフ化(図6枚+summary.txt)+アニメ MP4(スマホ動画同期可)+全期間俯瞰ボード PNG | `../pc_server/data/exp_logs/explog_*.csv`(+`exp_logs/videos/*.mp4`) | `graphs/explog_<stamp>/` |
| `magbias_learn.py` | 飛行ログ → magbias プロファイル JSON(ハードアイアン残差 Δb / ホバ点残差 / FF電圧・duty回帰。`docs/MAG_AUTOTUNE_DESIGN.md` §3-§4) | `../logs/flight_logs/*.csv`(v6推奨、v5は縮退) | `../pc_server/data/magbias_profiles/<name>.json`(+`--plots` で `graphs/magbias_<name>/`) |
| `replay/ekf2_replay.py` | EKF2(est_mode=2)の Python リプレイ — ヨー観測あり/なしの A/B・ゲート挙動検証(契約 §2.1/§4) | 飛行ログ CSV(v6=フル再構成 / v5=制限モード) | `graphs/ekf2_replay_<stem>/`(ψ軌跡CSV+比較図) |

`graphs/` は出力専用ディレクトリ(生成物)。数値ロジック本体は `ff_params/core.py`
(純粋関数ライブラリ)にあり、`tests/test_ff_extraction.py` /
`tests/test_magbias_learn.py` / `tests/test_ekf2_replay.py` が受入テスト。

## セットアップ

```sh
cd data_analysis
python3 -m venv .venv
.venv/bin/pip install -r requirements.txt   # numpy, matplotlib, opencv-python
.venv/bin/python tests/test_ff_extraction.py  # 受入テスト(pytest 不要、約1秒)
```

opencv-python は `plot_explog.py` のアニメ動画同期時のみ使う(それ以外は
numpy+matplotlib で足りる)。アニメの MP4 出力には別途 ffmpeg が必要
(macOS: `brew install ffmpeg`)。

## ① plot_sweep.py — スイープのグラフ化・加算性検証

```sh
# 対話モード: [1] スイープ1本 / [2] 加算性シーケンス → 一覧から番号選択(Enter=最新)
.venv/bin/python plot_sweep.py

# CLI: 拡張子/名前で自動判別(.csv → スイープ校正解析、sequence_*_meta.json → 加算性)
.venv/bin/python plot_sweep.py ../pc_server/data/sweep_results/sweep_20260612_202215_samples.csv
.venv/bin/python plot_sweep.py ../pc_server/data/sweep_results/sequence_XXXX_meta.json
# シーケンスが単機のみの場合、比較対象の全機スイープを明示指定(複数可、対話質問なし)
.venv/bin/python plot_sweep.py ../pc_server/data/sweep_results/sequence_XXXX_meta.json \
    --target sweep_YYYY --target sweep_ZZZZ
# オプション: -o <出力dir> / --results-dir <探索dir>(既定 ../pc_server/data/sweep_results/)
#            --target <stem or samples.csv>(加算性の比較対象。複数指定可)
```

- スイープ: ΔB 3D散布・ΔB vs 電流(全域/飛行帯 0.5–0.8 フィット)・|ΔB|・std・
  基準ドリフト・品質チェック・ヒステリシス・サンプル回帰・温度など PNG 12 枚。
- 加算性: Σ単機 vs 全機の比較図+判定サマリ PNG。しきい値は
  `max(4σ·RMS(noise), 0.5 µT)`。判定は stdout にも出る。
- **シーケンスが単機4本のみの場合**(Experiment タブの加算性シーケンスは単機のみを
  記録する): 同じフォルダから別取得の全機/対角ペアスイープ
  (`sweep_*_meta.json`, aborted=false)を候補一覧から対話選択して比較する。
  Enter で「単機と同姿勢(orientation)で時刻が最も近い1本」を自動選択、
  カンマ区切りで複数選択可、q で中止。`--target` 指定時は対話質問しない。
  候補も無い場合は「全機同時スイープ(FL+FR+RL+RR)を1本取得してください」と
  案内して終了する(空の判定サマリは出さない)。
  図/表の条件フッターには比較対象の由来(シーケンス内 or 外部選択)と姿勢を明記し、
  姿勢が異なる対象との比較には「※姿勢が異なる比較(参考)」の注記が付く。

## ② make_ff_profile.py — FF プロファイル抽出

pc_server の UI「FF 抽出」からサブプロセスとして呼ばれるのと同じ CLI。
手動でも使える。**入力は「展開後にちょうど 8 本」**:

- **8 本指定(従来)**: 全機 FL+FR+RL+RR × 4 姿勢(Yaw=0°/90°/±180°/-90°)+
  単機 FL/FR/RL/RR 各 1 本の sweep stem 8 個。
- **5 ファイル指定(シーケンス)**: 全機 4 姿勢の sweep stem 4 個+
  `sequence_*_meta.json` 1 個。sequence meta 内の単機 4 ラン
  (phase=done・非 aborted のみ)が自動展開され、従来の 8 本と同じ入力集合になる。

```sh
# 対話モード: sweep_results のサブフォルダ一覧 or stems 手動選択 → name/memo 入力
.venv/bin/python make_ff_profile.py

# フォルダ指定(8ペア、または 4ペア+sequence meta 入り)
.venv/bin/python make_ff_profile.py --folder ../pc_server/data/sweep_results/<セット名>

# stems 指定(8個、または全機4個+sequence meta で計5個)
.venv/bin/python make_ff_profile.py --stems sweep_A sweep_B ... --results-dir <dir>

# 共通オプション: --name <プロファイル名> --memo <1行メモ> -o <出力先> --plots(検証図PNG)
```

既定出力は `../pc_server/data/ff_profiles/<name>.json`(stampfly_ff_profile v1)。
中断ラン(meta.aborted=true)は stem 列挙付きでエラー拒否。詳細仕様は
`docs/FF_PIPELINE.md`。

## ③ plot_explog.py — Experiment 計測ログのグラフ化・アニメーション

```sh
# 対話モード: [1]静止画グラフ一式 / [2]アニメ(スマホ動画同期) / [3]アニメ(動画なし)
#   → explog CSV を新しい順に番号選択(Enter=最新)
#   → [2] のときは exp_logs/videos/ の動画を番号選択(パス直接入力も可)
#   → [2][3] ではスクロール窓幅(秒)を入力(Enter=5, 5〜60)
.venv/bin/python plot_explog.py

# CLI(静止画・従来)
.venv/bin/python plot_explog.py ../pc_server/data/exp_logs/explog_20260710_153000.csv [-o dir]

# CLI(アニメ)
.venv/bin/python plot_explog.py explog_xxx.csv --animation                 # 動画なし, 20fps
.venv/bin/python plot_explog.py explog_xxx.csv --video 動画.mp4            # 動画同期(fps=動画fps)
.venv/bin/python plot_explog.py explog_xxx.csv --video 動画.mp4 --start 10 --end 60 --fps 30
.venv/bin/python plot_explog.py explog_xxx.csv --animation --window 20     # スクロール窓幅 20 秒
```

図 6 枚+`summary.txt`(ドリフト率・NIS 統計・ゲート発火数など):

1. `01_yaw_comparison.png` — ヨー3系統(ジャイロ積分/推定/Madgwick)重畳+duty・モーターON帯
2. `02_yaw_error.png` — 開始基準ドリフトとジャイロ積分基準差(RMS/最大付き)
3. `03_current_duty.png` — 電流・バッテリ電圧・duty
4. `04_mag_ff.png` — Δb_cal と FF 予測 db_hat の重畳+残差(x/y/z)
5. `05_ekf_diagnostics.png` — NIS(閾値 2.0/5.99/13.8)+ffg 8ゲートのラスタ(bit7=ソフト再捕捉「再捕捉中」を含む)
6. `06_ff_status.png` — ff_mode(off/A/B)ステップ+ステータスフラグのラスタ

併設の `explog_<ts>_meta.json` があればプロファイル名・ff/est モードを
タイトルとサマリに表示(無くても動作)。必要列が欠けた図は理由付きでスキップ。

### アニメーション(7枠・スマホ動画同期)

出力は `graphs/explog_<stamp>/explog_<stamp>_animation[_with_video].mp4`
(1920x1080, libx264/yuv420p)。7枠 = ①スマホ動画(左2×2) ②ヨー(±180°ラップ)
③duty+電流 ④磁場x(校正済みとFF補正後) ⑤磁場y(同) ⑥b_m ⑦IMU温度。
時系列6枠は**スクロール窓表示+現在値表示**方式:

- x軸は常に**現在時刻 t が中央**の [t−窓幅/2, t+窓幅/2]。線は時刻≤t の
  履歴のみ描画し、**右半分は未来(空白)**。開始直後は左半分が負時間の空白で
  ドットだけが中央にあり、時間とともに履歴が左へ流れる(負の時間帯は目盛りが
  負値になる)。窓幅は既定 5 秒、`--window W` または対話メニュー
  ([2][3] の CSV/動画選択後の質問)で 5〜60 秒に変更可。
- y軸レンジはログ全期間のデータから初期化時に一度だけ固定
  (スクロール中にフレーム毎へ暴れない)。
- 全トレースに**現在位置ドット**(現在時刻の補間値の位置。線と同色・黒フチの
  大きな点。常に x軸中央)を重畳。値が NaN の区間(ヨーのラップ跨ぎ等)は非表示。
  overview.png にはドットは付かない。

アニメ生成時(メニュー[2][3]・`--animation`/`--video` の全経路)は、同一の
3×4 レイアウトで**全期間を俯瞰する静止画ボード**
`explog_<stamp>_overview.png` も同時出力する。動画枠は先頭フレームの
サムネイル固定(動画なし経路では「動画なし」表示)、時系列6枠は
`--start`/`--end` に関わらず常にログ全期間(カーソル・現在値ボックスなし)。

- **動画置き場**: `../pc_server/data/exp_logs/videos/`(対話メニュー[2]で番号選択)。
- **同期規約**: 「機体 LED がマゼンタに変わった瞬間 = 計測開始(t_s=0)」。
  スマホ動画は LED マゼンタ点灯の瞬間を先頭にカットしてから渡すこと
  (計測中は PC 側が CMD_LED_MODE でマゼンタ常灯を維持する)。
- 動画は正方形前提(非正方形は中央クロップ)。出力 fps は動画 fps(既定30)、
  ログ(≈23.5Hz)は各フレーム時刻へ線形補間。長さ = min(動画, ログ, `--end`)。
- 動画なし(`--animation` / メニュー[3])は 20fps・①枠は「動画なし」表示。
- 依存: ffmpeg(必須)、opencv-python(動画同期時のみ)。

## ④ magbias_learn.py — 飛行中磁気チューニング学習(契約 §3-§4)

pc_server の `/api/magbias extract` からサブプロセスとして呼ばれるのと同じ CLI。

```sh
.venv/bin/python magbias_learn.py ../logs/flight_logs/<log>.csv \
    [-o out.json] [--name NAME] [--plots] [--yaw-source auto|mocap|ekf2]
```

- **ハードアイアン残差 Δb**: ヨー励振区間のベクトルフィット
  `z_k = R_z(ψ_k)·B0h' + Δb`(mag_lev + mocap真値 or ekf2_yaw、磁気EMA遅れ
  τ≈0.456s をヨーの時間シフトで補償)。励振 <45° なら `delta_b=null`。
- **hover_residual**: EKF2 b_m の安定ホバ窓中央値。**ff_supplement**: b_m ×
  (電圧, duty4本, 電流)の線形回帰+R²。
- v5 ログ(7/27 以前、mag_lev/ekf2 列なし)では EKF1 tlm_bm 代用の縮退動作
  (warnings 明記)。出力スキーマは `stampfly_magbias_profile` v1(契約 §3)。

## ⑤ replay/ekf2_replay.py — EKF2 リプレイ(契約 §2.1/§4)

```sh
.venv/bin/python replay/ekf2_replay.py ../logs/flight_logs/<log>.csv \
    [--ff-profile ff.json] [--magbias magbias.json] \
    [--yaw-obs none|mocap|<列名>] [--low-trust] [--ekf1-mode] [-o dir]
```

- v6 ログ: b_cal + FFプロファイル + magbias から `b_corr→EMA→レベル化→EKF2`
  をフル再構成。ヨー擬似観測・τ_bm適応・飛行状態再アンカー(契約 §2.1)込み。
- v5 ログ: 制限モード(EKF1 ログ状態からの合成観測+受理マスク)。
  実観測イノベーション履歴が無いため ψ/b_m 配分は再現対象外(軌跡クラスと
  ゲート挙動のみ)。`--ekf1-mode` で現行 EKF1 の τ_bm 意味論に戻して比較可。
- 姿勢・レートはテレメトリ 25Hz 補間の近似(ファームは 400Hz)。

## テスト

```sh
.venv/bin/python tests/test_ff_extraction.py   # EXIT=0 で全合格
.venv/bin/python tests/test_magbias_learn.py   # 既知Δb注入の回復精度ほか
.venv/bin/python tests/test_ekf2_replay.py     # EKF2数式・パイプライン検証
```

test_ff_extraction は 6/12 実測 8 本と `tests/fixtures/results.json`(真値)の
照合、付録 A 再現、sequence meta 展開の検証を含む。数値挙動を変える変更を
したら必ず実行すること。
