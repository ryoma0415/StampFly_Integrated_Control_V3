// ===========================================================================
// flow_hub.cpp — オプティカルフローハブ実装
//
// StampFly_Telemetry/Telemetry/firmware/src/flow/flow_hub.cpp からの移植
// (MAG_AUTOTUNE_DESIGN.md §2.4)。移植差分は flow_hub.hpp の【V2変更】を参照
// (burst 読みのみ・起動時セルフテスト・恒久無効方針・アキュムレータ非移植・
//  スケールのみ較正・CMD_FLOW_PROBE の LOG_TEXT 要約)。
// ===========================================================================
#include "flow_hub.hpp"

#include <math.h>

#include "../comm.hpp"                // CMD_FLOW_PROBE 結果の LOG_TEXT 送出
#include "flow_persistence.hpp"
#include "pmw3901_burst_probe.hpp"
#include "pmw3901_driver.hpp"
#include "stampfly_protocol.hpp"      // flow_status のビット定義(TlmState::FLOW_STATUS_*)

namespace {

FlowState g_flow;
FlowCalibration g_flow_cal;

// K⁻¹ を計算してキャッシュする(2026-07-31 改訂: 2×2 行列較正)。
// det≈0(FLOW_SCALE_DET_MIN_COUNTS2 未満)は数値不安定として拒否 —
// flowcalMatrixValid 通過後は成立しないが、防御として二重に張る。
bool computeInverse(FlowCalibration& cal) {
    const float det = cal.m00 * cal.m11 - cal.m01 * cal.m10;
    if (!isfinite(det) || fabsf(det) < FLOW_SCALE_DET_MIN_COUNTS2) {
        return false;
    }
    const float inv_det = 1.0f / det;
    cal.inv00 = cal.m11 * inv_det;
    cal.inv01 = -cal.m01 * inv_det;
    cal.inv10 = -cal.m10 * inv_det;
    cal.inv11 = cal.m00 * inv_det;
    return true;
}

// 健全率の移動窓 (直近 N 読みの sample_healthy を 1bit ずつ保持する代わりに
// 単純なリングカウンタで近似: 窓内 valid 数 / 窓長)。
uint8_t g_health_window[FLOW_HEALTH_WINDOW_READS] = {0};
uint16_t g_health_head = 0;
uint16_t g_health_filled = 0;
uint16_t g_health_sum = 0;

void healthWindowPush(bool healthy) {
    const uint8_t v = healthy ? 1 : 0;
    if (g_health_filled >= FLOW_HEALTH_WINDOW_READS) {
        g_health_sum -= g_health_window[g_health_head];
    } else {
        g_health_filled++;
    }
    g_health_window[g_health_head] = v;
    g_health_sum += v;
    g_health_head = static_cast<uint16_t>((g_health_head + 1) % FLOW_HEALTH_WINDOW_READS);
    g_flow.health_ratio =
        g_health_filled > 0 ? static_cast<float>(g_health_sum) / g_health_filled : 0.0f;
}

// 【V2追加】起動時セルフテスト(契約 §2.4): burst 1 回+単発照合で burst_ok を
// 判定する。プローブ(pmw3901_burst_probe)の 1 サイクル判定と同じ基準:
//  (a) burst 転送成功 かつ ペイロードが固定パターン(0x93/0xFE ≥10B)・
//      全 0xFF・全 0x00 でなく、|Δ| ≤ 240(PX4 妥当性ゲート)
//  (b) 直後の単発読みで Δ 残差 ≤ FLOW_BURST_PROBE_RESIDUAL_TOL(burst が
//      本当に Motion 読みとして処理され Δ をクリアした証拠)かつ SQUAL 一致
// 先頭にウォームアップ 1 サイクル(集計外)を流し、BMI270(mode0)→
// PMW3901(mode3) 切替の初回トランザクション・グリッチを吸収する。
// モーター停止時(ブート)にのみ呼ぶこと(単発読み ≈6ms×3 ブロック)。
bool burstStartupSelftest() {
    // ウォームアップ(集計外): 単発 1 回 + burst 1 回
    Pmw3901Burst warm;
    (void)pmw3901ReadMotion(warm);
    Pmw3901Burst warm_burst;
    (void)pmw3901ReadMotionBurstHardCs(warm_burst);
    delay(FLOW_BURST_PROBE_CYCLE_GAP_MS);

    // ---- 判定サイクル: burst → 単発照合 ----
    Pmw3901Burst burst;
    uint8_t raw[12] = {0};
    if (!pmw3901ReadMotionBurstHardCs(burst, raw)) {
        USBSerial.println("PMW3901 selftest: burst SPI error");
        return false;
    }
    uint8_t n_93fe = 0;
    uint8_t n_ff = 0;
    uint8_t n_zero = 0;
    for (uint8_t k = 0; k < 12; k++) {
        if (raw[k] == 0x93 || raw[k] == 0xFE) n_93fe++;
        if (raw[k] == 0xFF) n_ff++;
        if (raw[k] == 0x00) n_zero++;
    }
    const bool payload_sane =
        n_93fe < 10 && n_ff < 12 && n_zero < 12 &&
        burst.delta_x >= -240 && burst.delta_x <= 240 &&
        burst.delta_y >= -240 && burst.delta_y <= 240;

    Pmw3901Burst single;
    if (!pmw3901ReadMotion(single)) {
        USBSerial.println("PMW3901 selftest: single-read SPI error");
        return false;
    }
    const bool residual_ok =
        abs(single.delta_x) <= FLOW_BURST_PROBE_RESIDUAL_TOL &&
        abs(single.delta_y) <= FLOW_BURST_PROBE_RESIDUAL_TOL;
    const int16_t sq_diff =
        static_cast<int16_t>(burst.squal) - static_cast<int16_t>(single.squal);
    const bool squal_ok = abs(sq_diff) <= FLOW_BURST_PROBE_SQUAL_TOL;

    const bool pass = payload_sane && residual_ok && squal_ok;
    USBSerial.printf(
        "PMW3901 selftest: %s (payload=%d residual=%d squal=%d) "
        "burst dx/dy/sq=%d/%d/%u single dx/dy/sq=%d/%d/%u\n",
        pass ? "PASS" : "FAIL",
        payload_sane ? 1 : 0, residual_ok ? 1 : 0, squal_ok ? 1 : 0,
        static_cast<int>(burst.delta_x), static_cast<int>(burst.delta_y), burst.squal,
        static_cast<int>(single.delta_x), static_cast<int>(single.delta_y), single.squal);
    return pass;
}

// プローブ集計の合格判定(移植元 docs/pmw3901_burst_probe.md §4 の基準)。
bool probeResultPass(const Pmw3901BurstProbeResult& r) {
    if (r.cycles == 0 || r.burst_ok == 0) return false;
    const float n = static_cast<float>(r.cycles);
    const float bok = static_cast<float>(r.burst_ok);
    return (bok / n) > FLOW_PROBE_PASS_BURST_OK_RATIO &&
           r.fixed_pattern == 0 && r.all_ff == 0 &&
           (static_cast<float>(r.all_zero) / bok) < FLOW_PROBE_PASS_ALLZERO_RATIO &&
           r.delta_in_range == r.burst_ok &&
           (static_cast<float>(r.agree) / bok) > FLOW_PROBE_PASS_AGREE_RATIO &&
           (static_cast<float>(r.squal_match) / bok) > FLOW_PROBE_PASS_SQUAL_MATCH_RATIO;
}

}  // namespace

void flowHubInit() {
    // NVS "flowcal" ロード(ブートロード順: … → magbias → flowcal。契約 §2.5)。
    // 未設定・破損時は既定 diag(450,450) のまま(loadFlowcal が既定値を返す。
    // v1 スキーマは loadFlowcal 内で diag(kx,ky) へ移行済み)。
    float m00 = FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;
    float m01 = 0.0f;
    float m10 = 0.0f;
    float m11 = FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;
    bool cal_valid = false;
    loadFlowcal(cal_valid, m00, m01, m10, m11);
    g_flow_cal.valid = cal_valid;
    g_flow_cal.m00 = m00;
    g_flow_cal.m01 = m01;
    g_flow_cal.m10 = m10;
    g_flow_cal.m11 = m11;
    // K⁻¹ をロード時に一度計算してキャッシュ(det≈0 は loadFlowcal の
    // flowcalMatrixValid 照合で除外済み。防御失敗時は既定へ戻す)。
    if (!computeInverse(g_flow_cal)) {
        g_flow_cal = FlowCalibration{};  // 既定 diag(450,450)+既定 K⁻¹
        USBSerial.println("flowcal inverse failed; using default scale");
    }

    g_flow.sensor_ok = pmw3901Init();
    g_flow.product_id = pmw3901ProductId();
    if (!g_flow.sensor_ok) {
        // 【V2変更】初期化リトライはしない(恒久無効。契約 §2.4)。
        // 復帰手段はベンチでの CMD_FLOW_PROBE(モーター停止時に再 init を試みる)。
        USBSerial.println("PMW3901 init failed; flow disabled (recover via flow probe)");
        return;
    }
    delay(50);
    // 起動時セルフテストで burst_ok を判定。不成立ならフロー恒久無効
    // (読みスロットは走らず、飛行挙動は完全に従来どおり)。
    g_flow.burst_ok = burstStartupSelftest();
    if (!g_flow.burst_ok) {
        USBSerial.println("PMW3901 burst selftest failed; flow disabled (recover via flow probe)");
    }
}

void flowHubUpdate(
    float roll_rate_rad_s,
    float pitch_rate_rad_s,
    float distance_m,
    bool distance_valid,
    float yaw_rad
) {
    static uint32_t last_read_us = 0;

    // 【V2変更】burst 不成立/未初期化なら恒久無効(リトライなし。契約 §2.4)。
    if (!g_flow.sensor_ok || !g_flow.burst_ok) {
        return;
    }

    // 50Hz(20ms)自己スロットル(契約 §2.4 の独立スロット)。
    const uint32_t now_us = micros();
    if (last_read_us != 0 && now_us - last_read_us < FLOW_READ_PERIOD_US) {
        return;
    }
    const float dt_s = last_read_us == 0
        ? FLOW_READ_PERIOD_US * 1.0e-6f
        : static_cast<float>(now_us - last_read_us) * 1.0e-6f;
    last_read_us = now_us;

    // 【V2変更】ハードCS burst 読みのみ(≈52µs 転送+デバイス登録/解除)。
    // 単発 6ms 読みは 400Hz tick 予算(2500µs)に入らないため飛行中は使わない。
    Pmw3901Burst burst;
    const bool read_ok = pmw3901ReadMotionBurstHardCs(burst);
    g_flow.read_count++;
    if (!read_ok) {
        g_flow.read_error_count++;
    }

    uint8_t bits = FLOW_HEALTH_SENSOR_OK;
    if (read_ok) {
        bits |= FLOW_HEALTH_READ_OK;
        g_flow.dx = burst.delta_x;
        g_flow.dy = burst.delta_y;
        g_flow.squal = burst.squal;
        g_flow.shutter_upper = burst.shutter_upper;
        g_flow.shutter_lower = burst.shutter_lower;
        g_flow.rawdata_sum = burst.rawdata_sum;
        g_flow.rawdata_max = burst.rawdata_max;
        g_flow.rawdata_min = burst.rawdata_min;
        g_flow.motion = burst.motion;
    }
    g_flow.read_dt_s = dt_s;

    // 品質ゲート (データシート基準 + Bitcraze の外れ値上限)。
    const bool shutter_sat = g_flow.shutter_upper == FLOW_SHUTTER_SAT_UPPER;
    const bool squal_low = g_flow.squal < FLOW_SQUAL_MIN;
    // データシートの破棄基準は「SQUAL低 かつ 露光飽和」の同時成立だが、
    // ゲートビットは個別に立てて原因を可視化する。
    if (read_ok && !squal_low) {
        bits |= FLOW_HEALTH_SQUAL_OK;
    }
    if (read_ok && !shutter_sat) {
        bits |= FLOW_HEALTH_SHUTTER_OK;
    }
    // 外れ値ゲートはレート基準 (Bitcraze の 100 counts/10ms を counts/s に換算)。
    // カウントはフレーム間積算で dt に比例して育つため、読み周期が伸びた tick を
    // 固定カウント閾値で誤棄却しないようにする。
    const float delta_limit_counts = FLOW_DELTA_OUTLIER_CPS * dt_s;
    const bool delta_ok = read_ok && dt_s > 1.0e-4f &&
                          fabsf(static_cast<float>(g_flow.dx)) <= delta_limit_counts &&
                          fabsf(static_cast<float>(g_flow.dy)) <= delta_limit_counts;
    if (delta_ok) {
        bits |= FLOW_HEALTH_DELTA_OK;
    }
    const bool range_ok = distance_valid && distance_m >= FLOW_MIN_HEIGHT_M &&
                          distance_m <= FLOW_MAX_HEIGHT_M;
    if (range_ok) {
        bits |= FLOW_HEALTH_RANGE_OK;
    }
    if (g_flow_cal.valid) {
        bits |= FLOW_HEALTH_CAL_VALID;
    }

    // 読みとして健全か(SQUAL/露光/外れ値。高度・較正有無は不問)。
    g_flow.sample_healthy = read_ok && !squal_low && !shutter_sat && delta_ok;

    // ---- 生フローレート [counts/s] ----
    if (read_ok && dt_s > 1.0e-4f) {
        g_flow.fx_raw_cps = static_cast<float>(g_flow.dx) / dt_s;
        g_flow.fy_raw_cps = static_cast<float>(g_flow.dy) / dt_s;
    } else {
        g_flow.fx_raw_cps = 0.0f;
        g_flow.fy_raw_cps = 0.0f;
    }

    // ---- 較正適用: 機体軸フローレート [rad/s] ----
    // 【2026-07-31 改訂】rate = K⁻¹·counts_rate(2×2 行列適用。ジャイロ補償
    // より前)。K⁻¹ はロード/適用時に計算済みのキャッシュ。軸マッピング・
    // 符号(既定固定)を先に適用してから行列に掛ける。
    const float raw_x = (g_flow_cal.x_src == 0 ? g_flow.fx_raw_cps : g_flow.fy_raw_cps);
    const float raw_y = (g_flow_cal.y_src == 0 ? g_flow.fx_raw_cps : g_flow.fy_raw_cps);
    const float cx = raw_x * g_flow_cal.x_sign;   // [counts/s]
    const float cy = raw_y * g_flow_cal.y_sign;
    g_flow.flow_x_rate = g_flow_cal.inv00 * cx + g_flow_cal.inv01 * cy;
    g_flow.flow_y_rate = g_flow_cal.inv10 * cx + g_flow_cal.inv11 * cy;

    const bool rate_ok = fabsf(g_flow.flow_x_rate) < FLOW_MAX_RATE_RAD_S &&
                         fabsf(g_flow.flow_y_rate) < FLOW_MAX_RATE_RAD_S;
    if (rate_ok) {
        bits |= FLOW_HEALTH_RATE_OK;
    }

    g_flow.health_bits = bits;
    // 【V2変更】vel_valid は CAL_VALID を除く全ビット成立(未較正でも既定
    // スケール 450 で換算する — ride-along テレメトリ用途。契約 §2.5)。
    g_flow.vel_valid = (bits | FLOW_HEALTH_CAL_VALID) == 0xFF;

    // ---- 速度換算 (回転補償 + 高度スケール) ----
    // FRD 系の測定モデル: flow_x = vx/d + q、flow_y = vy/d − p
    // (Bitcraze mm_flow.c は FLU 系の (vx·R22/z)−ωy_FLU。ωy_FLU=−q_FRD の変換で
    //  この形になる — 前進と機首上げは同符号で画像流に混入する)。
    // 逆算: vx = (flow_x − q)·d, vy = (flow_y + p)·d。d は ToF 生スラント距離。
    // 純回転中は flow_x=+q, flow_y=−p なので v≈0 になる (較正の規約と整合)。
    if (g_flow.vel_valid) {
        const float d = distance_m;
        g_flow.vx = (g_flow.flow_x_rate - pitch_rate_rad_s) * d;
        g_flow.vy = (g_flow.flow_y_rate + roll_rate_rad_s) * d;

        // ---- デッドレコニング (ワールド平面, アクティブヨーで回転) ----
        const float cy = cosf(yaw_rad);
        const float sy = sinf(yaw_rad);
        float px = g_flow.px + (g_flow.vx * cy - g_flow.vy * sy) * dt_s;
        float py = g_flow.py + (g_flow.vx * sy + g_flow.vy * cy) * dt_s;
        if (FLOW_POS_LEAK_TAU_S > 0.0f) {
            const float leak = 1.0f - dt_s / FLOW_POS_LEAK_TAU_S;
            px *= leak;
            py *= leak;
        }
        g_flow.px = px;
        g_flow.py = py;
    } else {
        g_flow.vx = 0.0f;
        g_flow.vy = 0.0f;
        // 位置は保持 (積分停止)。
    }

    // 健全率は「読みとして健全か」(sample_healthy: SQUAL/露光/外れ値) で数える。
    // vel_valid 基準だと接地中 (RANGE) が定義上ずっと 0% になり、
    // 床・照度の良し悪しが読めないため。
    healthWindowPush(g_flow.sample_healthy);
}

bool flowHubApplyCalibration(float m00, float m01, float m10, float m11) {
    // K⁻¹ を先にテンポラリで計算し、成立したときだけ適用する
    // (det≈0 は flight_control の flowcalMatrixValid で拒否済み — 防御)。
    FlowCalibration next = g_flow_cal;
    next.valid = true;
    next.m00 = m00;
    next.m01 = m01;
    next.m10 = m10;
    next.m11 = m11;
    if (!computeInverse(next)) {
        return false;
    }
    g_flow_cal = next;
    saveFlowcal(true, m00, m01, m10, m11);
    return true;
}

void flowHubClearCalibration() {
    g_flow_cal = FlowCalibration{};  // 既定 diag(450,450)+既定 K⁻¹ へ戻す
    clearFlowcal();
}

bool flowHubRunBurstProbe(uint8_t n_cycles) {
    // 未初期化(起動時 init 失敗)なら再初期化を 1 回試みる(モーター停止時の
    // ベンチ操作なので数十 ms のブロックは許容。成功すれば復帰経路になる)。
    if (!pmw3901Ready()) {
        g_flow.sensor_ok = pmw3901Init();
        g_flow.product_id = pmw3901ProductId();
        if (!g_flow.sensor_ok) {
            comm_send_log("flow probe: PMW3901 init failed (sensor unavailable)");
            return false;
        }
    }

    uint16_t cycles = n_cycles == 0 ? FLOW_BURST_PROBE_DEFAULT_CYCLES
                                    : static_cast<uint16_t>(n_cycles);
    if (cycles < FLOW_BURST_PROBE_MIN_CYCLES) cycles = FLOW_BURST_PROBE_MIN_CYCLES;
    if (cycles > FLOW_BURST_PROBE_MAX_CYCLES) cycles = FLOW_BURST_PROBE_MAX_CYCLES;

    Pmw3901BurstProbeResult r;
    if (!pmw3901BurstProbeRun(cycles, r)) {
        comm_send_log("flow probe: run failed (spi_bus_add_device)");
        return false;
    }

    // 合否判定 → burst_ok ラッチ更新(契約 §1.4: probe 成功で更新。不合格の
    // 実測も確定情報なのでラッチは合否どおり上書きする)。
    const bool pass = probeResultPass(r);
    g_flow.burst_ok = pass;

    // ---- LOG_TEXT 要約(複数行。1 行 ≤ MAX_LOG_TEXT_SIZE=180B)。
    //      連続送信で ESP-NOW TX バッファを溢れさせないよう行間に 5ms 置く
    //      (プローブ自体が数秒ブロックするベンチ操作なので許容)。----
    const float bok = r.burst_ok > 0 ? static_cast<float>(r.burst_ok) : 1.0f;
    const float sok = r.single_ok > 0 ? static_cast<float>(r.single_ok) : 1.0f;
    char line[stampfly::MAX_LOG_TEXT_SIZE + 1];
    snprintf(line, sizeof(line),
             "flow probe: n=%u burst ok/err=%u/%u single ok/err=%u/%u",
             r.cycles, r.burst_ok, r.burst_err, r.single_ok, r.single_err);
    comm_send_log(line);
    delay(5);
    snprintf(line, sizeof(line),
             "flow probe: fixed=%u allFF=%u allZERO=%u dRange=%u agree=%u (%.1f%%) sqMatch=%u",
             r.fixed_pattern, r.all_ff, r.all_zero, r.delta_in_range,
             r.agree, 100.0f * r.agree / bok, r.squal_match);
    comm_send_log(line);
    delay(5);
    snprintf(line, sizeof(line),
             "flow probe: burst us avg/min/max=%.0f/%lu/%lu single us avg=%.0f "
             "squal avg b/s=%.1f/%.1f",
             r.burst_us_sum / bok,
             static_cast<unsigned long>(r.burst_us_min),
             static_cast<unsigned long>(r.burst_us_max),
             r.single_us_sum / sok,
             r.burst_squal_sum / bok, r.single_squal_sum / sok);
    comm_send_log(line);
    delay(5);
    snprintf(line, sizeof(line),
             "flow probe: sum burst dx/dy=%ld/%ld single dx/dy=%ld/%ld",
             static_cast<long>(r.burst_dx_sum), static_cast<long>(r.burst_dy_sum),
             static_cast<long>(r.single_dx_sum), static_cast<long>(r.single_dy_sum));
    comm_send_log(line);
    delay(5);
    snprintf(line, sizeof(line),
             "flow probe: %s -> burst_ok=%d",
             pass ? "PASS" : "FAIL", pass ? 1 : 0);
    comm_send_log(line);
    return true;
}

uint8_t flowHubStatusByte() {
    using stampfly::TlmState;
    uint8_t status = 0;
    if (g_flow.sensor_ok) status |= TlmState::FLOW_STATUS_SENSOR_OK;
    if (g_flow.burst_ok) status |= TlmState::FLOW_STATUS_BURST_OK;
    if (g_flow.vel_valid) status |= TlmState::FLOW_STATUS_VEL_VALID;
    if (g_flow.health_bits & FLOW_HEALTH_RANGE_OK) status |= TlmState::FLOW_STATUS_RANGE_VALID;
    if (g_flow.health_bits & FLOW_HEALTH_SQUAL_OK) status |= TlmState::FLOW_STATUS_SQUAL_OK;
    // FLOW_STATUS_INIT_RETRY(bit5)は V2 では常に 0(恒久無効方針のため
    // 初期化リトライは存在しない。契約 §2.4)。bit6-7 予約 = 0。
    return status;
}

FlowState& flowHubState() {
    return g_flow;
}

FlowCalibration& flowHubCalibration() {
    return g_flow_cal;
}
