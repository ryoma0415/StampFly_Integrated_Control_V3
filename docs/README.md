# StampFly Integrated Control — ドキュメント索引

セットアップ+運用手順はリポジトリ直下の [../README.md](../README.md) に
一本化した(本フォルダと二重管理しない)。

| 文書 | 内容 |
|---|---|
| [../README.md](../README.md) | システム概要、必要機材、セットアップ(venv ×3 / pio / Motive)、pc_server 起動と UI の使い方(4タブ・ヨー制御・円軌道・Experiment・Multi・クイック較正)、検証コマンド、トラブルシューティング |
| [OPERATION_GUIDE.md](OPERATION_GUIDE.md) | 段階的な安全飛行手順(機体電源 OFF での通信確認 → 短時間ホバー → 本飛行)、緊急時エスカレーション、フェイルセーフ一覧、機体プロファイル較正、機上XY制御の初飛行前チェック §8.2.1、v2 運用手順(モーター実験モード §11 / ヨー較正・FF パイプライン §12 / ヨー角制御飛行 §13 / 軌道モード(円・往復・評価シーケンス)§14 / 飛行後解析 §15 / 複数機モード §16) |
| [PROTOCOL.md](PROTOCOL.md) | 通信ワイヤ仕様の正典 v2(COBS+CRC16 フレーム、メッセージ型 0x10–0x58、TLM_STATE 135B / TLM_CTRL 89B / TLM_EXP / TLM_CAL_DATA、フェイルセーフ規範、レート・帯域予算) |
| [ARCHITECTURE.md](ARCHITECTURE.md) | モジュール構成・API 契約・コーディング規約の正典 v2(実験モードの状態遷移、複数機モード、安全クランプ多層表、NVS 永続化一覧、I2C スケジューリングを含む) |
| [LOG_STRUCTURE.md](LOG_STRUCTURE.md) | CSV フライトログの列定義 v4・109列(`pc_server/core/logger.py` の `COLUMNS` と1対1。TLM_CTRL 由来の姿勢 PID 成分を含む) |
| [FF_PIPELINE.md](FF_PIPELINE.md) | 電流FF較正パイプラインの契約(スイープ → `data_analysis` 抽出 → FFプロファイル JSON → CMD_FF_* 分割転送 → EKF 実行。数式・受入基準は yaw側設計書の原文維持) |
| [OPTICAL_FLOW_STUDY.md](OPTICAL_FLOW_STUDY.md) | オプティカルフロー(PMW3901)活用の調査・検討書(将来検討。実装仕様ではない) |
| [RESEARCH_THEMES_2026.md](RESEARCH_THEMES_2026.md) | 研究テーマ検討書(2023-2026 動向調査+本プラットフォームでの実機検証適性。参考文献50本) |
| [../flight_log_viewer/README.md](../flight_log_viewer/README.md) | 飛行ログビューア(静止画グラフ・ヨー4系統比較・EKF 診断・同期アニメーション)の使い方 |
