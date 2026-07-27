// ===========================================================================
// pmw3901_driver.cpp — PMW3901 SPI ドライバ実装
//
// StampFly_Telemetry/Telemetry/firmware/src/flow/pmw3901_driver.cpp からの移植
// (MAG_AUTOTUNE_DESIGN.md §2.4)。移植差分:
//   - 手動CS Motion_Burst(既知問題あり・調査用)を削除し、ハードCS 一時登録の
//     pmw3901ReadMotionBurstHardCs() を追加(運用の読みは burst のみ)
//   - USBSerial 出力は起動時初期化の診断のみ(V2規約: 定常状態では出さない。
//     init/プローブはモーター停止時に限られるため許容)
// ===========================================================================
#include "pmw3901_driver.hpp"

#include <Arduino.h>

#include "driver/spi_master.h"
#include "flow_config.hpp"

namespace {

spi_device_handle_t s_dev = nullptr;
bool s_ready = false;
uint8_t s_product_id = 0;
uint8_t s_revision = 0;
// 初期化の自己診断値。
// s_check47: 書き込み検証ハンドシェイク (0x43←0x10 後の 0x47 読み。期待 0x08。
//            それ以外 = 「書き込みが効いていない」の直接証拠)
// s_check15: 観測チェック (シーケンス後の 0x15 読み。期待 0xBF)
uint8_t s_check47 = 0;
uint8_t s_check15 = 0;
// 書き込み方式の A/B: true = アドレス/データを 50µs 空けた 2 回転送 (Bitcraze 形)、
// false = 連続 2 バイト 1 転送 (データシート Figure 9 の素直な形)。
// 0x47 ハンドシェイクが通らないまま init に失敗するたびに切り替えて、
// どちらの方式ならこの個体に書き込めるかを実機上で自動判別する。
bool s_split_writes = true;

inline void csLow() {
    digitalWrite(PMW3901_CS_PIN, LOW);
}

inline void csHigh() {
    digitalWrite(PMW3901_CS_PIN, HIGH);
}

bool spiTransfer(const uint8_t* tx, uint8_t* rx, size_t len) {
    spi_transaction_t trans = {};
    trans.length = len * 8;
    trans.tx_buffer = tx;
    trans.rx_buffer = rx;
    return spi_device_polling_transmit(s_dev, &trans) == ESP_OK;
}

// タイミングは Bitcraze crazyflie-firmware (pmw3901.c) の保守値に全面的に
// 合わせる。データシート最小値 (t_SRAD=35µs、書き込みは連続 2B、t_SRR=20µs)
// での実装は、移植元実機 (2026-07-12 ログ flowlog_20260712_203325) で SQUAL 等の
// 単純レジスタは読めるのに Motion/Delta (0x02-0x06 — 読み出しで Delta を
// ラッチする特殊レジスタ) だけが 0xFF/0x00 固定になる症状を出した。
// 最小値ぎりぎりは不足とみて、実績値 (500µs/50µs/200µs) を使う。
static const uint32_t PMW3901_T_CS_SETUP_US = 50;   // CS Low → 最初のクロック
static const uint32_t PMW3901_T_WRITE_GAP_US = 50;  // 書き込みのアドレス→データ間
static const uint32_t PMW3901_T_SRAD_US = 500;      // 読みのアドレス→データ待ち
static const uint32_t PMW3901_T_CS_HOLD_US = 50;    // 最終クロック → CS High
static const uint32_t PMW3901_T_GAP_US = 200;       // トランザクション間

// レジスタ書き込み。方式は s_split_writes で切替:
//  - split: アドレス(MSB=1) とデータを 50µs 空けた 2 回の 1B 転送 (Bitcraze 形)
//  - continuous: 連続 2B の 1 転送 (データシート Figure 9 の素直な形)
void regWrite(uint8_t reg, uint8_t value) {
    const uint8_t addr = static_cast<uint8_t>(reg | 0x80u);
    csLow();
    delayMicroseconds(PMW3901_T_CS_SETUP_US);
    if (s_split_writes) {
        spiTransfer(&addr, nullptr, 1);
        delayMicroseconds(PMW3901_T_WRITE_GAP_US);
        spiTransfer(&value, nullptr, 1);
    } else {
        const uint8_t tx[2] = {addr, value};
        spiTransfer(tx, nullptr, 2);
    }
    delayMicroseconds(PMW3901_T_CS_HOLD_US);
    csHigh();
    delayMicroseconds(PMW3901_T_GAP_US);
}

// レジスタ読み込み: アドレス送信 → t_SRAD 待機 (CS Low のまま) → 1B 受信。
uint8_t regRead(uint8_t reg) {
    const uint8_t addr = static_cast<uint8_t>(reg & 0x7Fu);
    uint8_t rx = 0;
    const uint8_t dummy = 0;
    csLow();
    delayMicroseconds(PMW3901_T_CS_SETUP_US);
    spiTransfer(&addr, nullptr, 1);
    delayMicroseconds(PMW3901_T_SRAD_US);
    spiTransfer(&dummy, &rx, 1);
    delayMicroseconds(PMW3901_T_CS_HOLD_US);
    csHigh();
    delayMicroseconds(PMW3901_T_GAP_US);
    return rx;
}

void initPerformanceRegistersMain();

// 性能最適化レジスタ列。骨格は Bitcraze crazyflie-firmware (pmw3901.c) の
// InitRegisters() と同一だが、Bitcraze が省略しているデータシートの
// 条件付き初期化ブロック (pmw3901_doc_ja.md §3.2 / PX4 実装にも存在) を追加:
//  (1) 0x47 ハンドシェイク: 0x43←0x10 を書いて 0x47==0x08 を確認 (≤3 回試行)。
//      これは「書き込みが本当に効いているか」の読み返し検証を兼ねる。
//      通らなければ init 失敗 (データシート: 電源再投入が必要)。
//  (2) 0x67 bit7 に応じた 0x48 の分岐、0x73==0x00 のとき C1/C2 調整
//      (0x73 条件の向きは doc 記載と逆だが、PX4/pimoroni/simondlevy の
//       参照実装群に合わせて ==0x00 とする)。
//  (3) シーケンス後の観測チェック: 15ms 待って 0x15==0xBF (警告のみ)。
// PixArt 独自項目のためレジスタ個別の意味は非公開。
bool initPerformanceRegisters() {
    // ---- (1)(2) 条件付き初期化ブロック ----
    regWrite(0x7F, 0x00);
    regWrite(0x55, 0x01);
    regWrite(0x50, 0x07);
    regWrite(0x7F, 0x0E);
    s_check47 = 0;
    for (uint8_t attempt = 0; attempt < 3; attempt++) {
        regWrite(0x43, 0x10);
        s_check47 = regRead(0x47);
        if (s_check47 == 0x08) {
            break;
        }
    }
    if (s_check47 != 0x08) {
        USBSerial.printf(
            "PMW3901: write-verify handshake failed (0x47=0x%02X, expect 0x08, writes=%s)\n"
            "  -> 書き込みがセンサーに効いていません。バッテリーを外した完全な電源再投入を試してください\n",
            s_check47,
            s_split_writes ? "split" : "continuous"
        );
        return false;
    }

    const uint8_t v67 = regRead(0x67);
    regWrite(0x48, (v67 & 0x80u) ? 0x04 : 0x02);
    regWrite(0x7F, 0x00);
    regWrite(0x51, 0x7B);
    regWrite(0x50, 0x00);
    regWrite(0x55, 0x00);
    regWrite(0x7F, 0x0E);
    if (regRead(0x73) == 0x00) {
        uint8_t c1 = regRead(0x70);
        c1 = static_cast<uint8_t>(c1 + (c1 <= 0x1C ? 0x0E : 0x0B));
        if (c1 > 0x3F) {
            c1 = 0x3F;
        }
        const uint8_t c2 = static_cast<uint8_t>((regRead(0x71) * 45u) / 100u);
        regWrite(0x7F, 0x00);
        regWrite(0x61, 0xAD);
        regWrite(0x51, 0x70);
        regWrite(0x7F, 0x0E);
        regWrite(0x70, c1);
        regWrite(0x71, c2);
        USBSerial.printf("PMW3901: C1/C2 tuning applied (C1=0x%02X C2=0x%02X)\n", c1, c2);
    }

    initPerformanceRegistersMain();
    return true;
}

// Bitcraze InitRegisters() と同一の主シーケンス。
void initPerformanceRegistersMain() {
    static const uint8_t seq_a[][2] = {
        {0x7F, 0x00}, {0x61, 0xAD}, {0x7F, 0x03}, {0x40, 0x00},
        {0x7F, 0x05}, {0x41, 0xB3}, {0x43, 0xF1}, {0x45, 0x14},
        {0x5B, 0x32}, {0x5F, 0x34}, {0x7B, 0x08}, {0x7F, 0x06},
        {0x44, 0x1B}, {0x40, 0xBF}, {0x4E, 0x3F}, {0x7F, 0x08},
        {0x65, 0x20}, {0x6A, 0x18}, {0x7F, 0x09}, {0x4F, 0xAF},
        {0x5F, 0x40}, {0x48, 0x80}, {0x49, 0x80}, {0x57, 0x77},
        {0x60, 0x78}, {0x61, 0x78}, {0x62, 0x08}, {0x63, 0x50},
        {0x7F, 0x0A}, {0x45, 0x60}, {0x7F, 0x00}, {0x4D, 0x11},
        {0x55, 0x80}, {0x74, 0x1F}, {0x75, 0x1F}, {0x4A, 0x78},
        {0x4B, 0x78}, {0x44, 0x08}, {0x45, 0x50}, {0x64, 0xFF},
        {0x65, 0x1F}, {0x7F, 0x14}, {0x65, 0x67}, {0x66, 0x08},
        {0x63, 0x70}, {0x7F, 0x15}, {0x48, 0x48}, {0x7F, 0x07},
        {0x41, 0x0D}, {0x43, 0x14}, {0x4B, 0x0E}, {0x45, 0x0F},
        {0x44, 0x42}, {0x4C, 0x80}, {0x7F, 0x10}, {0x5B, 0x02},
        {0x7F, 0x07}, {0x40, 0x41}, {0x70, 0x00},
    };
    static const uint8_t seq_b[][2] = {
        {0x32, 0x44}, {0x7F, 0x07}, {0x40, 0x40}, {0x7F, 0x06},
        {0x62, 0xF0}, {0x63, 0x00}, {0x7F, 0x0D}, {0x48, 0xC0},
        {0x6F, 0xD5}, {0x7F, 0x00}, {0x5B, 0xA0}, {0x4E, 0xA8},
        {0x5A, 0x50}, {0x40, 0x80}, {0x7F, 0x00}, {0x5A, 0x10},
        {0x54, 0x00},
    };
    for (const auto& wv : seq_a) {
        regWrite(wv[0], wv[1]);
    }
    // 観測チェック (doc §3.2): {0x7F,0x07},{0x70,0x00} の後 15ms 待って
    // 0x15 を読むと 0xBF になるはず。書き込みが効いているかの第 2 の検証点
    // (不一致は警告のみ — Bitcraze はこのチェックなしでも飛んでいるため)。
    delay(15);
    s_check15 = regRead(0x15);
    if (s_check15 != 0xBF) {
        USBSerial.printf(
            "PMW3901: observation check mismatch (0x15=0x%02X, expect 0xBF)\n", s_check15);
    }
    for (const auto& wv : seq_b) {
        regWrite(wv[0], wv[1]);
    }
}

// ---- 【V2追加】ハードCS burst 用ヘルパー(pmw3901_burst_probe と同一パターン) ----

// eco 構成のハードCSデバイスを登録する。登録で GPIO12 の出力ソースが
// GPIO マトリクス経由で SPI CS 信号に切り替わる (以後 digitalWrite は無効)。
bool burstDeviceAttach(spi_device_handle_t& dev) {
    spi_device_interface_config_t devcfg = {};
    devcfg.mode = 3;
    devcfg.clock_speed_hz = PMW3901_SPI_CLOCK_HZ;
    devcfg.spics_io_num = PMW3901_CS_PIN;  // ハードCS (eco 実証構成)
    devcfg.queue_size = 7;                 // eco pmw3901.c と同値 (polling では未使用)
    return spi_bus_add_device(SPI2_HOST, &devcfg, &dev) == ESP_OK;
}

// デバイスを外し、G12 を手動 GPIO CS (既定運用) に戻す。remove は GPIO を
// 復元しないので pinMode で明示的にマトリクスを GPIO 出力へ戻す。
void burstDeviceDetach(spi_device_handle_t dev) {
    spi_bus_remove_device(dev);
    pinMode(PMW3901_CS_PIN, OUTPUT);
    digitalWrite(PMW3901_CS_PIN, HIGH);
}

}  // namespace

bool pmw3901Init() {
    s_ready = false;

    pinMode(PMW3901_CS_PIN, OUTPUT);
    csHigh();

    if (s_dev == nullptr) {
        spi_device_interface_config_t devcfg = {};
        devcfg.mode = 3;
        devcfg.clock_speed_hz = PMW3901_SPI_CLOCK_HZ;
        devcfg.spics_io_num = -1;  // CS は G12 を手動制御 (t_SRAD 中 Low 保持のため)
        devcfg.queue_size = 4;
        const esp_err_t err = spi_bus_add_device(SPI2_HOST, &devcfg, &s_dev);
        if (err != ESP_OK) {
            USBSerial.printf("PMW3901: spi_bus_add_device failed (%d)\n", static_cast<int>(err));
            s_dev = nullptr;
            return false;
        }
    }

    // 初期化列の途中に BMI270 読みが割り込まないようバスを占有する
    // (同一タスク実行だが、多段トランザクション+delay を含むため保険)。
    spi_device_acquire_bus(s_dev, portMAX_DELAY);

    // バス取得直後の初回トランザクション・グリッチ対策のダミー読み
    // (pmw3901ReadMotion と同じ理由。最初の実操作が 0x3A リセット書き込みの
    //  ため、化けると以降が全て無意味になる)。
    (void)regRead(0x00);

    // SPI ポートリセット (NCS トグル) + Power_Up_Reset。電源投入からは
    // ブートシーケンス(sensor_init までの各初期化)で 40ms 要件を大きく超えている。
    csHigh();
    delay(2);
    csLow();
    delay(2);
    csHigh();
    delay(2);
    regWrite(0x3A, 0x5A);
    delay(5);

    s_product_id = regRead(0x00);
    const uint8_t inverse_id = regRead(0x5F);
    s_revision = regRead(0x01);
    if (s_product_id != 0x49 || inverse_id != 0xB6) {
        spi_device_release_bus(s_dev);
        USBSerial.printf(
            "PMW3901: product id mismatch (id=0x%02X inv=0x%02X)\n",
            s_product_id,
            inverse_id
        );
        return false;
    }

    // モーション関連レジスタ (0x02-0x06) を一度読み捨てて Delta を解放する。
    for (uint8_t reg = 0x02; reg <= 0x06; reg++) {
        (void)regRead(reg);
    }
    delay(1);

    // Frame Capture 離脱シーケンス (データシート p.27-28 step 8)。
    // Frame Capture は航法を無効化し、復旧には本来ハードウェアリセットが必要
    // (§7.3)。本モジュールの NRESET は RC 電源投入リセットのみで MCU から
    // 叩けないため、過去に航法無効状態へ落ちた個体への best-effort 回復として
    // 毎 init で離脱列を流しておく (正常時に流しても無害な値: 初期化列と同値)。
    static const uint8_t frame_capture_exit[][2] = {
        {0x7F, 0x00}, {0x4D, 0x11}, {0x40, 0x80}, {0x55, 0x80},
        {0x7F, 0x08}, {0x6A, 0x18}, {0x7F, 0x07}, {0x41, 0x0D},
        {0x4C, 0x80}, {0x7F, 0x00},
    };
    for (const auto& wv : frame_capture_exit) {
        regWrite(wv[0], wv[1]);
    }

    if (!initPerformanceRegisters()) {
        spi_device_release_bus(s_dev);
        // 書き込み方式を切り替えて次回の init リトライで A/B 判定する。
        s_split_writes = !s_split_writes;
        USBSerial.printf(
            "PMW3901: init failed; next retry uses %s writes\n",
            s_split_writes ? "split" : "continuous"
        );
        return false;
    }

    spi_device_release_bus(s_dev);
    s_ready = true;
    USBSerial.printf(
        "PMW3901 init ok: id=0x%02X rev=0x%02X check47=0x%02X check15=0x%02X writes=%s\n",
        s_product_id,
        s_revision,
        s_check47,
        s_check15,
        s_split_writes ? "split" : "continuous"
    );
    return true;
}

bool pmw3901Ready() {
    return s_ready;
}

uint8_t pmw3901ProductId() {
    return s_product_id;
}

uint8_t pmw3901Revision() {
    return s_revision;
}

uint8_t pmw3901Check47() {
    return s_check47;
}

uint8_t pmw3901Check15() {
    return s_check15;
}

bool pmw3901ReadMotionBurstHardCs(Pmw3901Burst& out, uint8_t* raw12) {
    if (!s_ready) {
        return false;
    }
    // eco pmw3901_read_motion_burst() と同一の転送: tx[0]=0x16、計 13B を
    // 1 トランザクション (CS はハードウェアが転送全体で Low 保持)。
    // バッファは DMA の語境界パディング書き込みに備えて 16B 確保で渡す。
    spi_device_handle_t dev = nullptr;
    if (!burstDeviceAttach(dev)) {
        return false;
    }
    alignas(4) uint8_t tx[16] = {0};
    alignas(4) uint8_t rx[16] = {0};
    tx[0] = 0x16;
    spi_transaction_t trans = {};
    trans.length = 13 * 8;
    trans.tx_buffer = tx;
    trans.rx_buffer = rx;
    const bool ok = spi_device_polling_transmit(dev, &trans) == ESP_OK;
    delayMicroseconds(50);  // eco の読み後ディレイ (PMW3901_SPI_READ_DELAY_US)
    burstDeviceDetach(dev);

    if (!ok) {
        return false;
    }
    // rx[0] はコマンドエコー。ペイロードは rx[1..12] (eco と同じ並び)。
    const uint8_t* p = &rx[1];
    if (raw12 != nullptr) {
        for (uint8_t k = 0; k < 12; k++) {
            raw12[k] = p[k];
        }
    }
    out.product_probe = 0;  // バーストではカナリア読みなし
    out.motion = p[0];
    out.observation = p[1];
    out.delta_x = static_cast<int16_t>((static_cast<uint16_t>(p[3]) << 8) | p[2]);
    out.delta_y = static_cast<int16_t>((static_cast<uint16_t>(p[5]) << 8) | p[4]);
    out.squal = p[6];
    out.rawdata_sum = p[7];
    out.rawdata_max = p[8];
    out.rawdata_min = p[9];
    out.shutter_upper = p[10];
    out.shutter_lower = p[11];
    return true;
}

// 診断レジスタ (RawData 統計・Shutter_Lower) の読み出し間引き。
// セルフテスト/プローブ用途では表示専用の値なので毎回読む必要はない。
static const uint8_t PMW3901_DIAG_DECIMATION = 25;

bool pmw3901ReadMotion(Pmw3901Burst& out) {
    if (!s_ready || s_dev == nullptr) {
        return false;
    }
    static uint8_t diag_countdown = 0;
    static uint8_t last_rawdata_sum = 0;
    static uint8_t last_rawdata_max = 0;
    static uint8_t last_rawdata_min = 0;
    static uint8_t last_shutter_lower = 0;

    spi_device_acquire_bus(s_dev, portMAX_DELAY);
    // バス取得直後の最初のトランザクションは化ける (移植元実機で先頭読みの 0xFF を
    // 恒常観測 — BMI270(mode0)→PMW3901(mode3) 切替の初回グリッチとみられる)。
    // ダミー読みを 1 回捨てて吸収する (regRead 内の CS トグルでセンサー側の
    // SPI フレーミングも復位する)。
    (void)regRead(0x00);
    // 読み出し経路カナリア: 同一サイクル・同一条件で既知値レジスタを読み、
    // 0x49 が返るなら読み経路は健全で、続く Motion/Delta はレジスタの実内容。
    out.product_probe = regRead(0x00);
    // Motion(0x02) の読みで Delta_X/Y がフリーズし、読み出しで解放される
    // (データシート §4.1)。Bitcraze readMotionCount と同じく MOT7 の値に
    // よらず毎回 Delta を読む (MOT7=0 なら 0 が返るだけ)。
    out.motion = regRead(0x02);
    out.observation = 0;  // 単発読みでは取得しない (バースト専用フィールド)
    const uint8_t dxl = regRead(0x03);
    const uint8_t dxh = regRead(0x04);
    const uint8_t dyl = regRead(0x05);
    const uint8_t dyh = regRead(0x06);
    out.delta_x = static_cast<int16_t>((static_cast<uint16_t>(dxh) << 8) | dxl);
    out.delta_y = static_cast<int16_t>((static_cast<uint16_t>(dyh) << 8) | dyl);
    out.squal = regRead(0x07);
    out.shutter_upper = regRead(0x0C);  // 品質ゲート対象 (==0x1F で露光飽和)
    if (diag_countdown == 0) {
        last_rawdata_sum = regRead(0x08);
        last_rawdata_max = regRead(0x09);
        last_rawdata_min = regRead(0x0A);
        last_shutter_lower = regRead(0x0B);
        diag_countdown = PMW3901_DIAG_DECIMATION;
    }
    diag_countdown--;
    spi_device_release_bus(s_dev);

    out.rawdata_sum = last_rawdata_sum;
    out.rawdata_max = last_rawdata_max;
    out.rawdata_min = last_rawdata_min;
    out.shutter_lower = last_shutter_lower;
    return true;
}
