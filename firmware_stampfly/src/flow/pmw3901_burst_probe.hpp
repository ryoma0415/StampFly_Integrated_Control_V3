#ifndef STAMPFLY_FLOW_PMW3901_BURST_PROBE_HPP
#define STAMPFLY_FLOW_PMW3901_BURST_PROBE_HPP

#include <Arduino.h>

// ハードウェアCS Motion_Burst のベンチプローブ (CMD_FLOW_PROBE)。
// StampFly_Telemetry/Telemetry/firmware/src/flow/pmw3901_burst_probe.hpp からの
// 移植 (MAG_AUTOTUNE_DESIGN.md §1.4/§2.4)。
//
// 背景: 移植元の手動 GPIO CS 構成では Motion_Burst が固定パターン (0x93/0xFE)
// を返して不成立だった (2026-07-12 実機ログ。ただし後に個体不良と判明した
// drone test 上の観測の可能性がある)。一方 stampfly_ecosystem は
// 「ハードウェア CS (spics_io_num=G12) + spi_device_polling_transmit で
// tx[0]=0x16 の 13B 一括転送」で 100Hz 常用・BMI270 同一バス共有の飛行実績が
// ある。本プローブはその eco 構成を一時的に再現し、burst 読みが本機の
// SPI2_HOST 共有構成でも成立するかを実機集計する。
//
// 方式: サイクルごとに第2の spi_device (ハードCS G12, 2MHz mode3) を
// spi_bus_add_device で登録して burst 13B を 1 転送し、spi_bus_remove_device
// + pinMode で G12 を手動 GPIO CS に戻してから単発読み (pmw3901ReadMotion)
// を行う — 手動CS運用と GPIO12 の所有はサイクル内で排他になる。
// 読みのみで書き込み系レジスタには一切触れない (Frame Capture 事故防止)。
//
// 【V2変更】結果要約は呼び出し側 (flow_hub) が LOG_TEXT でも送出する。
// 本モジュールの USBSerial 出力はベンチ診断用にそのまま残す (モーター停止時
// のみ実行されるため V2 の「定常状態では USB シリアルへ出力しない」規約に
// 抵触しない)。

// 集計結果。判定基準は移植元 docs/pmw3901_burst_probe.md §4 を参照。
struct Pmw3901BurstProbeResult {
    uint16_t cycles = 0;      // 集計サイクル数 (ウォームアップ 1 サイクル除く)
    uint16_t burst_ok = 0;    // burst SPI 転送成功
    uint16_t burst_err = 0;
    uint16_t single_ok = 0;   // 単発読み成功
    uint16_t single_err = 0;

    // ---- (a) burst ペイロード妥当性 ----
    uint16_t fixed_pattern = 0;   // 12B 中 10B 以上が 0x93/0xFE (既知の固定パターン)
    uint16_t all_ff = 0;          // 12B 全て 0xFF (mode 切替グリッチ/無応答)
    uint16_t all_zero = 0;        // 12B 全て 0x00 (デッドリード疑い)
    uint16_t mot_93fe = 0;        // motion バイトが 0x93/0xFE
    uint16_t mot_set = 0;         // MOT(bit7) が立ったサイクル (参考値)
    uint16_t delta_in_range = 0;  // |Δx|,|Δy| <= 240 (PX4 妥当性ゲートと同値)

    // ---- (b) burst Δ と単発 Δ の整合 ----
    // burst が本当に Motion 読みとして処理されたなら Δ はクリアされ、直後の
    // 単発読みの残差は ≈0 になる (残差 <= FLOW_BURST_PROBE_RESIDUAL_TOL)。
    uint16_t agree = 0;
    // SQUAL は read-clear でない緩変化量なので burst/単発で一致するはず
    // (差 <= FLOW_BURST_PROBE_SQUAL_TOL)。全ゼロ系デッドリードの検出用。
    uint16_t squal_match = 0;
    int32_t burst_dx_sum = 0;   // 移動テストで Σburst ≈ 実移動、Σ単発 ≈ 0 を確認する
    int32_t burst_dy_sum = 0;
    int32_t single_dx_sum = 0;
    int32_t single_dy_sum = 0;

    // ---- (c) 所要時間 [µs] (burst は polling_transmit のみ、単発は 7 レジスタ 1 式) ----
    uint32_t burst_us_sum = 0;
    uint32_t burst_us_min = 0;
    uint32_t burst_us_max = 0;
    uint32_t single_us_sum = 0;
    uint32_t single_us_min = 0;
    uint32_t single_us_max = 0;

    // ---- (d) SQUAL 分布 (burst 読み値。bin = squal >> 5 の 8 分割) ----
    uint16_t squal_hist[8] = {0};
    uint32_t burst_squal_sum = 0;
    uint32_t single_squal_sum = 0;

    uint8_t first_rx[12] = {0};  // 最初の集計サイクルの burst 生ペイロード (目視用)
};

// N サイクルの交互実行プローブ。数秒ブロックするため呼ぶのは CMD_FLOW_PROBE
// (モーター停止時のみ — 実行中はテレメトリ・failsafe も止まる)に限ること。
// pmw3901Ready() が前提。false は spi_bus_add_device 失敗 (集計なし)。
bool pmw3901BurstProbeRun(uint16_t cycles, Pmw3901BurstProbeResult& out);

#endif
