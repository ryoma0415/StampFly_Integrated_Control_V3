#ifndef STAMPFLY_FLOW_PMW3901_DRIVER_HPP
#define STAMPFLY_FLOW_PMW3901_DRIVER_HPP

#include <Arduino.h>

// PMW3901MB オプティカルフローセンサーの SPI ドライバ。
// StampFly_Telemetry/Telemetry/firmware/src/flow/pmw3901_driver.hpp からの移植
// (MAG_AUTOTUNE_DESIGN.md §2.4)。
//
// - BMI270 と SPI2_HOST を共有する(バス初期化は imu_init() → spi_init() が
//   済ませている前提。本ドライバは spi_bus_add_device で第2デバイスを足す)。
// - CS(G12)は手動 GPIO 制御: レジスタ読みはアドレス送信後 t_SRAD を
//   NCS Low のまま待つ必要があり、ESP-IDF の自動 CS では
//   トランザクション境界で CS が上がってしまうため。
// - タイミング規定(データシート Table 5): t_SWW/t_SWR=45µs、t_SRR/t_SRW=20µs、
//   t_SRAD=35µs を各操作に織り込み済み。
// - 初期化列は Bitcraze crazyflie-firmware(pmw3901.c, Flow deck 飛行実績)の
//   シーケンスを採用。飛行中の再初期化は不可(復帰はフル再初期化 ≈50ms+delay 込み)。
//
// 【V2変更】運用の読みは pmw3901ReadMotionBurstHardCs()(ハードCS 一時登録の
// Motion_Burst 13B、≈52µs)のみ(契約 §2.4)。移植元の手動CS burst
// (実機で固定パターン 0x93/0xFE を返す既知問題あり)は持ち込まない。
// 単発読み pmw3901ReadMotion()(保守タイミング ≈6ms)は起動時セルフテストと
// CMD_FLOW_PROBE の照合専用で、飛行中は使わない。

// Motion_Burst(0x16) 12 バイトのデコード結果。
struct Pmw3901Burst {
    // 読み出し経路のカナリア: 同一サイクル内で Product_ID(0x00) を読んだ値。
    // 0x49 なら「読み経路は健全 → Motion/Delta の値はレジスタの実内容」と確定
    // できる (単発読み方式のみ。バーストでは 0)。
    uint8_t product_probe = 0;
    uint8_t motion = 0;       // bit7=MOT: 新規モーション検出
    uint8_t observation = 0;
    int16_t delta_x = 0;      // フレーム間移動量 [counts] (16bit 符号付き)
    int16_t delta_y = 0;
    uint8_t squal = 0;        // 表面品質 (特徴量の多さ)
    uint8_t rawdata_sum = 0;
    uint8_t rawdata_max = 0;
    uint8_t rawdata_min = 0;
    uint8_t shutter_upper = 0;  // 0x1F で露光飽和 (照度不足)
    uint8_t shutter_lower = 0;
};

// フル初期化(電源投入シーケンス+性能最適化レジスタ列)。成功で true。
// 失敗時は再度呼んでよい(内部でリセットからやり直す)。数十 ms ブロックする
// ため、呼ぶのは起動時と CMD_FLOW_PROBE(モーター停止時)に限ること。
bool pmw3901Init();

// 初期化済みで使用可能か。
bool pmw3901Ready();

// 直近 init で読んだ Product_ID(期待値 0x49)/ Revision_ID。
uint8_t pmw3901ProductId();
uint8_t pmw3901Revision();

// 初期化の自己診断値:
// Check47 = 書き込み検証ハンドシェイク (期待 0x08。他値 = 書き込み不達の直接証拠)、
// Check15 = 観測チェック (期待 0xBF)。
uint8_t pmw3901Check47();
uint8_t pmw3901Check15();

// 【V2追加】ハードCS Motion_Burst 13B 一括読み(契約 §2.4 の運用読み)。
// stampfly_ecosystem 実証構成(spics_io_num=G12 + spi_device_polling_transmit
// で tx[0]=0x16 の 13B 一括転送。BMI270 同一バス共有・100Hz 常用の飛行実績)を
// 毎回の呼び出しで一時再現する: spi_bus_add_device(ハードCS)→ 転送 →
// spi_bus_remove_device + pinMode で G12 を手動 GPIO CS に復元
// (pmw3901_burst_probe と同一パターン。手動CS 運用と GPIO12 の所有が
// 呼び出し内で排他になり、同一 400Hz タスクの BMI270 読みとは順次実行で
// 衝突しない)。転送 52µs+登録/解除オーバーヘッドで 1 回 ≈0.15ms。
// raw12 が非 nullptr なら生ペイロード 12B(コマンドエコー除去後)も返す
// (起動時セルフテストの固定パターン判定用)。false は SPI エラー。
bool pmw3901ReadMotionBurstHardCs(Pmw3901Burst& out, uint8_t* raw12 = nullptr);

// 単発レジスタ読みでのモーションデータ取得 (Bitcraze Arduino ドライバ
// readMotionCount と同方式)。Motion(0x02) を読んで Delta をフリーズ →
// DX_L/H, DY_L/H, SQUAL, Shutter_Upper を順に読む (計 7 レジスタ。保守
// タイミング採用のため ≈6ms/回 — セルフテスト/プローブ照合専用)。
// RawData 統計と Shutter_Lower は診断値なので PMW3901_DIAG_DECIMATION 回に
// 1 回だけ読む (それ以外は前回値を保持)。false は SPI エラー。
bool pmw3901ReadMotion(Pmw3901Burst& out);

#endif
