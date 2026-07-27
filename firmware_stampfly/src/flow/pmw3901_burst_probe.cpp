// ===========================================================================
// pmw3901_burst_probe.cpp — ハードCS Motion_Burst ベンチプローブ実装
//
// StampFly_Telemetry/Telemetry/firmware/src/flow/pmw3901_burst_probe.cpp からの
// 移植 (MAG_AUTOTUNE_DESIGN.md §1.4/§2.4)。ロジックは移植元と同一
// (attach/detach ヘルパーはドライバ側の運用 burst と独立に保持し、所要時間の
// 計測点を移植元 docs/pmw3901_burst_probe.md §2(c) の定義どおり「転送のみ」に
// 保つ)。USBSerial 出力はベンチ診断用に残す (LOG_TEXT 要約は flow_hub 側)。
// ===========================================================================
#include "pmw3901_burst_probe.hpp"

#include <Arduino.h>
#include <cstring>

#include "driver/spi_master.h"
#include "flow_config.hpp"
#include "pmw3901_driver.hpp"

namespace {

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

// eco pmw3901_read_motion_burst() と同一の転送: tx[0]=0x16、計 13B を
// 1 トランザクション (CS はハードウェアが転送全体で Low 保持)。
// バッファは DMA の語境界パディング書き込みに備えて 16B 確保で渡すこと。
bool burstReadHardCs(spi_device_handle_t dev, uint8_t* rx16) {
    alignas(4) uint8_t tx[16] = {0};
    tx[0] = 0x16;
    spi_transaction_t trans = {};
    trans.length = 13 * 8;
    trans.tx_buffer = tx;
    trans.rx_buffer = rx16;
    return spi_device_polling_transmit(dev, &trans) == ESP_OK;
}

void accumulateTiming(uint32_t us, uint32_t& sum, uint32_t& min_v, uint32_t& max_v, bool first) {
    sum += us;
    if (first || us < min_v) {
        min_v = us;
    }
    if (first || us > max_v) {
        max_v = us;
    }
}

}  // namespace

bool pmw3901BurstProbeRun(uint16_t cycles, Pmw3901BurstProbeResult& out) {
    out = Pmw3901BurstProbeResult{};
    if (!pmw3901Ready() || cycles == 0) {
        return false;
    }
    out.cycles = cycles;

    // ウォームアップ: 直前の BMI270 (mode0) 通信による初回トランザクション・
    // グリッチと、前回読みからの Δ 蓄積を捨てる (単発読み 1 回 + burst 1 回を
    // 集計せず流す。pmw3901ReadMotion は内部の初回ダミー読みも実施する)。
    Pmw3901Burst warm;
    (void)pmw3901ReadMotion(warm);
    {
        spi_device_handle_t dev = nullptr;
        if (!burstDeviceAttach(dev)) {
            USBSerial.println("PMW3901 burst probe: spi_bus_add_device failed");
            return false;
        }
        alignas(4) uint8_t rx[16] = {0};
        (void)burstReadHardCs(dev, rx);
        burstDeviceDetach(dev);
    }

    bool first_rx_saved = false;
    for (uint16_t i = 0; i < cycles; i++) {
        // ---- burst (ハードCS)。add/remove をサイクル毎に行い、単発読みの
        // 手動 CS と GPIO12 の所有が重ならないようにする。計測は転送のみ。----
        spi_device_handle_t dev = nullptr;
        if (!burstDeviceAttach(dev)) {
            out.burst_err++;
            continue;
        }
        alignas(4) uint8_t rx[16] = {0};
        const uint32_t t0 = micros();
        const bool burst_ok = burstReadHardCs(dev, rx);
        const uint32_t burst_us = micros() - t0;
        delayMicroseconds(50);  // eco の読み後ディレイ (PMW3901_SPI_READ_DELAY_US)
        burstDeviceDetach(dev);

        // ---- 直後の単発読み (手動CS・保守タイミング、実績のある読み方式) ----
        Pmw3901Burst single;
        const uint32_t t2 = micros();
        const bool single_ok = pmw3901ReadMotion(single);
        const uint32_t single_us = micros() - t2;

        if (single_ok) {
            out.single_ok++;
            out.single_squal_sum += single.squal;
            accumulateTiming(single_us, out.single_us_sum, out.single_us_min,
                             out.single_us_max, out.single_ok == 1);
        } else {
            out.single_err++;
        }

        if (!burst_ok) {
            out.burst_err++;
            delay(FLOW_BURST_PROBE_CYCLE_GAP_MS);
            continue;
        }
        out.burst_ok++;
        accumulateTiming(burst_us, out.burst_us_sum, out.burst_us_min,
                         out.burst_us_max, out.burst_ok == 1);

        // rx[0] はコマンドエコー。ペイロードは rx[1..12] (eco と同じ並び)。
        const uint8_t* p = &rx[1];
        if (!first_rx_saved) {
            memcpy(out.first_rx, p, 12);
            first_rx_saved = true;
        }

        // ---- (a) ペイロード妥当性 ----
        uint8_t n_93fe = 0;
        uint8_t n_ff = 0;
        uint8_t n_zero = 0;
        for (uint8_t k = 0; k < 12; k++) {
            if (p[k] == 0x93 || p[k] == 0xFE) {
                n_93fe++;
            }
            if (p[k] == 0xFF) {
                n_ff++;
            }
            if (p[k] == 0x00) {
                n_zero++;
            }
        }
        if (n_93fe >= 10) {
            out.fixed_pattern++;
        }
        if (n_ff == 12) {
            out.all_ff++;
        }
        if (n_zero == 12) {
            out.all_zero++;
        }
        const uint8_t motion = p[0];
        if (motion == 0x93 || motion == 0xFE) {
            out.mot_93fe++;
        }
        if (motion & 0x80u) {
            out.mot_set++;
        }
        const int16_t bdx = static_cast<int16_t>((static_cast<uint16_t>(p[3]) << 8) | p[2]);
        const int16_t bdy = static_cast<int16_t>((static_cast<uint16_t>(p[5]) << 8) | p[4]);
        const uint8_t bsq = p[6];
        if (bdx >= -240 && bdx <= 240 && bdy >= -240 && bdy <= 240) {
            out.delta_in_range++;
        }
        out.burst_dx_sum += bdx;
        out.burst_dy_sum += bdy;
        out.burst_squal_sum += bsq;
        out.squal_hist[bsq >> 5]++;

        // ---- (b) 整合: burst が Δ をクリアしたなら単発残差 ≈ 0 ----
        if (single_ok) {
            out.single_dx_sum += single.delta_x;
            out.single_dy_sum += single.delta_y;
            if (abs(single.delta_x) <= FLOW_BURST_PROBE_RESIDUAL_TOL &&
                abs(single.delta_y) <= FLOW_BURST_PROBE_RESIDUAL_TOL) {
                out.agree++;
            }
            const int16_t sq_diff = static_cast<int16_t>(bsq) - static_cast<int16_t>(single.squal);
            if (abs(sq_diff) <= FLOW_BURST_PROBE_SQUAL_TOL) {
                out.squal_match++;
            }
        }

        delay(FLOW_BURST_PROBE_CYCLE_GAP_MS);
    }

    // ---- USB シリアルへの要約 (LOG_TEXT 要約と同内容。ベンチ診断用) ----
    const float bok = out.burst_ok > 0 ? static_cast<float>(out.burst_ok) : 1.0f;
    const float sok = out.single_ok > 0 ? static_cast<float>(out.single_ok) : 1.0f;
    USBSerial.printf(
        "PMW3901 burst probe: n=%u burst ok/err=%u/%u single ok/err=%u/%u\n",
        out.cycles, out.burst_ok, out.burst_err, out.single_ok, out.single_err);
    USBSerial.printf(
        "PMW3901 burst probe: fixed93FE=%u allFF=%u allZERO=%u mot93FE=%u dRange=%u\n",
        out.fixed_pattern, out.all_ff, out.all_zero, out.mot_93fe, out.delta_in_range);
    USBSerial.printf(
        "PMW3901 burst probe: agree=%u (%.1f%%) squalMatch=%u "
        "sum burst dx/dy=%ld/%ld single dx/dy=%ld/%ld\n",
        out.agree, 100.0f * out.agree / bok, out.squal_match,
        static_cast<long>(out.burst_dx_sum), static_cast<long>(out.burst_dy_sum),
        static_cast<long>(out.single_dx_sum), static_cast<long>(out.single_dy_sum));
    USBSerial.printf(
        "PMW3901 burst probe: burst us avg/min/max=%.0f/%lu/%lu single us avg/min/max=%.0f/%lu/%lu\n",
        out.burst_us_sum / bok,
        static_cast<unsigned long>(out.burst_us_min),
        static_cast<unsigned long>(out.burst_us_max),
        out.single_us_sum / sok,
        static_cast<unsigned long>(out.single_us_min),
        static_cast<unsigned long>(out.single_us_max));
    USBSerial.printf(
        "PMW3901 burst probe: squal avg burst/single=%.1f/%.1f hist=[%u %u %u %u %u %u %u %u]\n",
        out.burst_squal_sum / bok, out.single_squal_sum / sok,
        out.squal_hist[0], out.squal_hist[1], out.squal_hist[2], out.squal_hist[3],
        out.squal_hist[4], out.squal_hist[5], out.squal_hist[6], out.squal_hist[7]);
    USBSerial.printf(
        "PMW3901 burst probe: first rx = "
        "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n",
        out.first_rx[0], out.first_rx[1], out.first_rx[2], out.first_rx[3],
        out.first_rx[4], out.first_rx[5], out.first_rx[6], out.first_rx[7],
        out.first_rx[8], out.first_rx[9], out.first_rx[10], out.first_rx[11]);
    return true;
}
