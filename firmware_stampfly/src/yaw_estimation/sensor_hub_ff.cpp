// ===========================================================================
// sensor_hub_ff.cpp — ヨー推定・電流FF補正の統合フック実装
//
// yaw側 sensor_hub.cpp から FF補正挿入点・アイドルアンカー・3系統推定器の
// 駆動ロジックを抽出移植したもの。IMU/AHRS/ToF はベース(sensor.cpp)側の
// 実装を使うため持ち込まない。数式・符号・定数値は yaw側と同一。
// 差分は入力の取り方のみ:
//   - motors_running / duty[4](FL,FR,RL,RR)/ mask を呼び出し側から受け取る
//     (yaw側は motorTestRunning()/motorTestDuty()×mask だった。飛行中は
//      ミキサ出力4値を FL,FR,RL,RR の順で渡す — ベース変数は FR,FL,RR,RL 順
//      なので呼び出し側で並べ替えること)
//   - ToF 読み tick では磁気/電流の 20Hz スロットをスキップ(位相スタガ)
// ===========================================================================
#include "sensor_hub_ff.hpp"

#include <math.h>

#include "../comm.hpp"  // 地上自動再アンカー(V2改修B-2)の LOG_TEXT 通知
#include "angle_utils.hpp"
#include "persistence.hpp"
#include "stampfly_protocol.hpp"  // ekf2_status のビット定義(TlmState::EKF2_STATUS_*)
#include "yaw_config.hpp"

YawEstimationState g_yaw_est;

namespace {

void noteMagError(uint32_t& counter, const char* what) {
    counter++;
    if (counter % 100 == 1) {
        USBSerial.printf("BMM150 %s: errors=%lu\n", what, static_cast<unsigned long>(counter));
    }
}

// ---- 電流FF補正: アイドルアンカー (yaw側 ff_pipeline_design.md §5.3) ----

// モーター停止中の b_cal / I_total を 20Hz で積む 2s リングバッファ。
struct FfAnchorWindow {
    MagVector b[FF_ANCHOR_WINDOW_SAMPLES];
    float i[FF_ANCHOR_WINDOW_SAMPLES];
    uint8_t head = 0;
    uint8_t count = 0;

    void reset() {
        head = 0;
        count = 0;
    }
    void push(const MagVector& b_cal, float i_total) {
        b[head] = b_cal;
        i[head] = i_total;
        head = static_cast<uint8_t>((head + 1) % FF_ANCHOR_WINDOW_SAMPLES);
        if (count < FF_ANCHOR_WINDOW_SAMPLES) {
            count++;
        }
    }
    bool full() const { return count >= FF_ANCHOR_WINDOW_SAMPLES; }
    void mean(MagVector& b_mean, float& i_mean) const {
        float sx = 0.0f, sy = 0.0f, sz = 0.0f, si = 0.0f;
        for (uint8_t k = 0; k < count; k++) {
            sx += b[k].x;
            sy += b[k].y;
            sz += b[k].z;
            si += i[k];
        }
        const float inv = count > 0 ? 1.0f / static_cast<float>(count) : 0.0f;
        b_mean = MagVector{sx * inv, sy * inv, sz * inv};
        i_mean = si * inv;
    }
};

FfAnchorWindow g_ff_anchor_window;
TwoWire* g_wire = nullptr;          // sensorHubFfInit で渡された I2C バス
bool g_in_flight = false;           // 今tickの飛行状態スナップショット(入力から転記)
bool g_mag_cal_seen = false;        // fresh な b_cal を一度でも観測したか
bool g_ff_boot_anchor_done = false; // ブート後の自動初回アンカー取得済みか
// 直近 update で使ったレベル化入力姿勢(アンカー凍結時の B0 水平化に使う)
float g_last_roll_rad = 0.0f;
float g_last_pitch_rad = 0.0f;

// ---- EKF2 ヨー擬似観測のステージ(consume-once。契約 §2.1-1 / §2.3) ----
// flight_control(400Hzループ内のコマンド適用)が sensorHubFfStageYawObs で書き、
// 次 tick の sensorHubFfUpdate(同じ400Hzループ)が取り出す。同一タスク内の
// 受け渡しなので排他は不要。
bool g_yaw_obs_pending = false;         // 未消費のヨー観測があるか
float g_yaw_obs_rad = 0.0f;             // 外部ヨー基準 [rad]
bool g_yaw_obs_low_trust = false;       // CMD_POS_ERR flags bit4
uint32_t g_yaw_obs_rx_ms = 0;           // 受信時刻(age 判定用)
uint32_t g_yaw_obs_last_rx_ms = 0;      // 直近受信時刻(ekf2_status bit0 の根拠)
bool g_yaw_obs_last_low_trust = false;  // 直近受信の low_trust(ekf2_status bit6)

// ---- EKF2 飛行状態再アンカー(契約 §2.1-3。1フライト1回) ----
FfAnchorWindow g_ekf2_flight_window;      // 飛行中 b_corr_filt の 2s リング(20Hz)
bool g_ekf2_flight_anchor_done = false;   // 実施済み(ekf2_status bit2)
bool g_ekf2_prev_in_flight = false;       // 飛行遷移検出用
uint32_t g_ekf2_flight_start_ms = 0;      // flying 遷移時刻
float g_ekf2_alt_hold_s = 0.0f;           // |alt_est−alt_ref|<0.1m の継続時間

// 補正系推定器の再シード (§5.4): 補正CFへリファレンスCFの内部状態
// (yaw・磁気オフセット・ヨーゼロ)をコピーし、補正系EMAとノルム基準を
// 再初期化。EKF / EKF2 の ψ もリファレンス yaw に合わせる。
void ffReseedCorrectedEstimators() {
    g_yaw_est.yaw_estimator_corr = g_yaw_est.yaw_estimator;
    g_yaw_est.yaw_estimator_corr.resetMagNormReference();
    g_yaw_est.mag_filter_corr.reset();
    g_yaw_est.yaw_kf.reseedYaw(g_yaw_est.yaw_estimator.yaw());
    g_yaw_est.yaw_kf2.reseedYaw(g_yaw_est.yaw_estimator.yaw());
}

// 停止窓が満ちていればアンカーを凍結する: B0 / I_idle / B0_horiz / ψ0 を
// 確定し、b_m←0・P←P0 (reanchor)・補正EMA・補正系norm_ref を再初期化。
bool ffFreezeAnchorFromWindow() {
    if (!g_ff_anchor_window.full()) {
        return false;
    }
    // V2改修C: 再アンカー ψ0 の健全化。EKF が直近まで観測を通常受理できている
    // (=健全) なら ψ0 に EKF の現在 ψ を使う。停止直後のリファレンスCF は
    // リキャプチャ上限 2°/更新の引き込み遅れで過渡誤差 (7/10 実測 −15° 前後) を
    // 持ち得るため、健全時は EKF 継続でスナップ≈0 とし、EKF が壊れている
    // (NIS棄却継続・凍結・未アンカー・FF停止) ときだけ従来どおり CF で矯正する。
    // ブート直後の初回は anchorValid()=false なので従来どおり CF になる。
    // 注意: ffReseedCorrectedEstimators() が yaw_kf.reseedYaw() で ψ/nis/
    // time_since_accept を消すため、判定と ψ の読み出しは必ずこの位置で行う。
    const YawEstimatorKf& kf = g_yaw_est.yaw_kf;
    const bool ekf_psi0_ok = sensorHubFfCorrectionActive() && kf.anchorValid() &&
                             kf.timeSinceAccept() < FF_EKF_ANCHOR_PSI0_FRESH_S &&
                             kf.nis() < FF_EKF_NIS_INFLATE && !kf.magFrozen();
    const float psi0 = ekf_psi0_ok ? kf.yaw() : g_yaw_est.yaw_estimator.yaw();
    // EKF2 も同じ健全判定を独立に適用する(健全な EKF2 は自分の ψ で継続 =
    // スナップ≈0、壊れているときだけ CF で矯正。契約 §2.2 のシャドー並走)。
    const YawEstimatorKf2& kf2 = g_yaw_est.yaw_kf2;
    const bool ekf2_psi0_ok = sensorHubFfCorrectionActive() && kf2.anchorValid() &&
                              kf2.timeSinceAccept() < FF_EKF_ANCHOR_PSI0_FRESH_S &&
                              kf2.nis() < FF_EKF_NIS_INFLATE && !kf2.magFrozen();
    const float psi0_2 = ekf2_psi0_ok ? kf2.yaw() : g_yaw_est.yaw_estimator.yaw();

    MagVector b0;
    float i_idle = 0.0f;
    g_ff_anchor_window.mean(b0, i_idle);
    // magbias 適用中は B0 も観測空間(b_corr = b_cal − ΔB̂ − Δb_magbias)へ
    // 合わせる(契約 §2.2)。窓は b_cal を積むため凍結時に Δb を引く。
    // これを怠ると norm/z ゲート基準と b_m の初期値が Δb 分ずれ、magbias の
    // 目的(初期 b_m 立ち上がり縮小)が失われる。アイドル中は ΔB̂≈0。
    if (g_yaw_est.magbias_valid) {
        b0 = MagVector{
            b0.x - g_yaw_est.magbias.x,
            b0.y - g_yaw_est.magbias.y,
            b0.z - g_yaw_est.magbias.z,
        };
    }
    const MagVector b0h = levelMagVectorBody(g_last_roll_rad, g_last_pitch_rad, b0);
    g_yaw_est.ff.anchor_b0 = b0;
    g_yaw_est.ff.anchor_b0h_x = b0h.x;
    g_yaw_est.ff.anchor_b0h_y = b0h.y;
    g_yaw_est.ff.anchor_psi0 = psi0;
    g_yaw_est.ff.anchor_i_idle = i_idle;
    g_yaw_est.ff.anchor_valid = true;
    ffReseedCorrectedEstimators();
    g_yaw_est.yaw_kf.reanchor(psi0, b0h.x, b0h.y, b0);
    g_yaw_est.yaw_kf2.reanchor(psi0_2, b0h.x, b0h.y, b0);
    // 地上アンカーで基準場を取り直したため、EKF2 の飛行状態再アンカー実施済み
    // フラグ(ekf2_status bit2)も落とす(次フライトで再武装される)。
    g_ekf2_flight_anchor_done = false;
    return true;
}

// 毎tick呼ぶアンカーサービス: 始動遷移でのアンカー凍結と、停止中20Hzの
// 窓更新、ブート後の自動初回取得を行う。
// 「回転中」判定は呼び出し側の motors_running(PWM実出力ありを含む)に従う:
// ランプダウン中サンプルが停止窓に混入して B0/I_idle を汚染するのを防ぐため、
// 呼び出し側は実出力 duty>0 の間も true を渡すこと(yaw側 C2 と同基準)。
void ffAnchorService(bool motors_running) {
    static uint32_t last_push_ms = 0;
    static bool prev_running = false;
    static uint32_t last_auto_reanchor_ms = 0;  // V2改修B-2 のクールダウン起点
    const bool running = motors_running;

    if (running && !prev_running) {
        // 停止→回転の始動遷移: 直前の停止窓でアンカー凍結。
        // 窓が満ちていない場合は旧アンカーを維持する。
        ffFreezeAnchorFromWindow();
        g_ff_anchor_window.reset();
    } else if (!running && prev_running) {
        // 回転→停止: OFF直後の過渡を含まないよう窓を貯め直す。
        g_ff_anchor_window.reset();
    }
    prev_running = running;

    if (running) {
        return;
    }
    const uint32_t now_ms = millis();
    if (now_ms - last_push_ms < FF_ANCHOR_PERIOD_MS) {
        return;
    }
    last_push_ms = now_ms;
    if (!g_mag_cal_seen || !g_yaw_est.current_ready || !g_yaw_est.current_sample.valid) {
        return;
    }
    g_ff_anchor_window.push(g_yaw_est.frame.mag_cal_body, g_yaw_est.current_sample.current_a);
    if (!g_ff_boot_anchor_done && g_ff_anchor_window.full()) {
        g_ff_boot_anchor_done = ffFreezeAnchorFromWindow();
    }

    // V2改修B-2: 地上自動再アンカー。モーターOFF・B0 窓 full で、EKF が最終受理
    // から FF_EKF_AUTO_REANCHOR_AFTER_S(10s) を超えて観測を受理できていない
    // (NIS棄却や b_m 凍結が続いている) 場合、アンカーを自動で取り直して
    // B0/ψ0 を現在の磁場環境に合わせ直す (ソフト再捕捉 B-1 でも戻れない
    // 大乖離・環境変化の最終救済)。FF_EKF_AUTO_REANCHOR_COOLDOWN_S(30s) の
    // クールダウンで連発を防ぐ。EKF が実際に駆動されている (FF補正有効かつ
    // アンカー有効 = time_since_accept が進む条件) ときのみ発動する。
    if (g_ff_anchor_window.full() && g_yaw_est.ff.anchor_valid &&
        sensorHubFfCorrectionActive() &&
        g_yaw_est.yaw_kf.timeSinceAccept() > FF_EKF_AUTO_REANCHOR_AFTER_S &&
        now_ms - last_auto_reanchor_ms >=
            static_cast<uint32_t>(FF_EKF_AUTO_REANCHOR_COOLDOWN_S * 1000.0f)) {
        if (ffFreezeAnchorFromWindow()) {
            last_auto_reanchor_ms = now_ms;
            comm_send_log("EKF自動再アンカー(NIS棄却継続のため)");
        }
    }
}

// 20Hz 電流スロット。fresh な読みが起きた tick は current_slot_fired=true。
void updateCurrent() {
    static uint32_t last_current_ms = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_current_ms < YAW_SLOW_SLOT_PERIOD_MS) {
        return;
    }
    last_current_ms = now_ms;
    // スロット発火はデバイス不調でも報告する(ベース側の低電圧判定が
    // 「計測不能な20Hzスロット」を連続カウントして安全側に倒せるように)。
    g_yaw_est.current_slot_fired = true;
    if (!g_yaw_est.current_ready) {
        return;
    }
    g_yaw_est.current_sample = currentSensorRead();
}

// 20Hz 磁気スロット + FF補正パス(yaw側 updateMagnetometer と同一ロジック)。
void updateMagnetometer() {
    static uint32_t last_mag_sample_ms = 0;
    static uint32_t mag_errors = 0;
    const uint32_t now_ms = millis();
    if (now_ms - last_mag_sample_ms < YAW_SLOW_SLOT_PERIOD_MS) {
        return;
    }
    last_mag_sample_ms = now_ms;

    if (!g_yaw_est.bmm_ready) {
        // 再初期化リトライは非飛行時のみ: Bmm150Driver::begin() は delay(5)/
        // delay(10) を含み得るため、TAKEOFF/HOVER/LANDING の 400Hz 制御ループを
        // 5〜15ms ブロックして dt を乱す(半死チップは電源書込みに ACK した後
        // chip_id/トリム読出しで失敗し delay まで到達する)。飛行中は保留し、
        // WAIT/COMPLETE/MOTOR_TEST/CALIBRATION に戻ってから 1s 周期で再試行する。
        static uint32_t last_retry_ms = 0;
        if (!g_in_flight && g_wire != nullptr && now_ms - last_retry_ms > 1000) {
            last_retry_ms = now_ms;
            g_yaw_est.bmm_ready = g_yaw_est.bmm150.begin(*g_wire, BMM150_I2C_ADDRESS);
        }
        return;
    }

    Bmm150RawSample sample;
    if (!g_yaw_est.bmm150.readRaw(sample)) {
        noteMagError(mag_errors, "read failed");
        return;
    }

    if (!sample.data_ready) {
        return;
    }

    if (!sample.compensated_valid) {
        noteMagError(mag_errors, "compensation failed");
        return;
    }

    YawFfMagFrame& frame = g_yaw_est.frame;
    frame.mag_dt_s = frame.last_mag_fresh_ms == 0
        ? 0.1f
        : static_cast<float>(sample.timestamp_ms - frame.last_mag_fresh_ms) * 1.0e-3f;
    frame.last_mag_fresh_ms = sample.timestamp_ms;

    const MagVector mag_raw_body = transformMagToBody(sample.compensated_sensor);
    const MagVector mag_cal_body = g_yaw_est.mag3d_calibration.valid()
        ? g_yaw_est.mag3d_calibration.apply(mag_raw_body)
        : mag_raw_body;
    frame.mag_raw_body = mag_raw_body;
    frame.mag_cal_body = mag_cal_body;
    g_mag_cal_seen = true;
    // 非補正パス(従来どおり): リファレンスCF・アンカー窓・実験テレメトリ用。
    frame.mag_filtered_body = g_yaw_est.mag_filter.update(mag_cal_body);
    frame.yaw_mag_level_rad =
        wrapPi(atan2f(frame.mag_filtered_body.y, frame.mag_filtered_body.x));
    frame.mag_sample_fresh = true;

    // ---- 電流FF補正パス (ff_pipeline_design.md §5.2) ----
    // b_cal 直後・EMA 前で b_corr = b_cal − ΔB̂。電流は同tickで取得済み
    // (updateCurrent が先に呼ばれる)、duty は呼び出し側が渡した実効値×マスク
    // (飛行中はミキサ出力 FL,FR,RL,RR、モーターテスト中はテストduty×mask)。
    // 電流サンプルが無効 (INA3221 死亡/読み取り失敗) の tick は else 分岐の
    // 非補正縮退 (ΔB̂=0) に落とす — i_total=0 での誤った外挿を避ける。
    const bool ff_active = sensorHubFfCorrectionActive();
    if (ff_active) {
        const FfCorrection corr = g_yaw_est.ff_calibration.compute(
            g_yaw_est.current_sample.current_a,
            g_yaw_est.duty,
            g_yaw_est.motor_mask,
            g_yaw_est.motors_running,
            g_yaw_est.ff.ff_mode,
            g_yaw_est.ff.anchor_i_idle,
            g_yaw_est.ff.anchor_valid,
            now_ms
        );
        g_yaw_est.ff.delta_b = corr.delta_b;
        g_yaw_est.ff.sigma_ff_uT = corr.sigma_ff_uT;
        g_yaw_est.ff.sigma_slew_uT = corr.sigma_slew_uT;
        g_yaw_est.ff.sigma_diff_uT = corr.sigma_diff_uT;
        // magbias 適用 (契約 §2.2): b_corr = b_cal − ΔB̂ − Δb_magbias。
        // 補正系のみ・EMA 前。無効時 magbias は0ベクトル(適用/ロード側で保証)。
        const MagVector& dbm = g_yaw_est.magbias;
        const MagVector b_corr{
            mag_cal_body.x - corr.delta_b.x - dbm.x,
            mag_cal_body.y - corr.delta_b.y - dbm.y,
            mag_cal_body.z - corr.delta_b.z - dbm.z,
        };
        frame.mag_corr_filtered_body = g_yaw_est.mag_filter_corr.update(b_corr);
    } else {
        g_yaw_est.ff.delta_b = MagVector{};
        g_yaw_est.ff.sigma_ff_uT = 0.0f;
        g_yaw_est.ff.sigma_slew_uT = 0.0f;
        g_yaw_est.ff.sigma_diff_uT = 0.0f;
        frame.mag_corr_filtered_body = frame.mag_filtered_body;
    }
}

// EKF2 飛行状態再アンカーサービス(契約 §2.1-3。毎tick呼ぶ)。
// 飛行中の b_corr_filt を 20Hz で 2s リングに積み、条件成立で EKF2 の基準場を
// 飛行電流状態の B0f へ差し替える(1フライト1回)。条件:
//   flying 遷移後 >5s ∧ |alt_est−alt_ref|<0.1m が 2s 継続 ∧ 電流 >1.0A ∧
//   yaw_obs_fused(直近 0.5s 内に受理)∧ リング full。
// 不成立時は地上アンカーのまま(EKF1 と同等 = 悪化しない)。EKF1・FF 補正の
// アンカー(ff.anchor_*)には触れない — EKF2 のゲート基準のみ変わる。
void ekf2FlightAnchorService(
    const SensorHubFfInputs& in, float roll_rad, float pitch_rad, float dt_s
) {
    static uint32_t last_push_ms = 0;
    const uint32_t now_ms = millis();

    if (in.in_flight && !g_ekf2_prev_in_flight) {
        // 飛行開始: 1フライト1回の再武装+リング/整定タイマのリセット
        g_ekf2_flight_anchor_done = false;
        g_ekf2_flight_window.reset();
        g_ekf2_alt_hold_s = 0.0f;
        g_ekf2_flight_start_ms = now_ms;
    }
    g_ekf2_prev_in_flight = in.in_flight;
    if (!in.in_flight) {
        return;
    }

    // 飛行中 b_corr_filt の 2s リング(20Hz。アイドルアンカー窓と同じ周期・容量)
    if (now_ms - last_push_ms >= FF_ANCHOR_PERIOD_MS) {
        last_push_ms = now_ms;
        if (sensorHubFfCorrectionActive()) {
            g_ekf2_flight_window.push(
                g_yaw_est.frame.mag_corr_filtered_body,
                g_yaw_est.current_sample.current_a
            );
        } else {
            // 補正非稼働 (電流サンプル無効等) で push が中断した場合はリングを
            // 破棄 — リングは時刻を持たないため、残すと「直近2s」でない古い
            // b_corr が B0f 平均へ混入する。full() = 連続2s分の有効サンプル保証。
            g_ekf2_flight_window.reset();
        }
    }

    if (g_ekf2_flight_anchor_done) {
        return;
    }

    // |alt_est − alt_ref| < 0.1m の継続時間
    if (fabsf(in.alt_est_m - in.alt_ref_m) < FF_EKF2_FLIGHT_ANCHOR_ALT_TOL_M) {
        g_ekf2_alt_hold_s += dt_s;
    } else {
        g_ekf2_alt_hold_s = 0.0f;
    }

    const bool yaw_obs_fused =
        g_yaw_est.yaw_kf2.timeSinceYawAccept() < FF_EKF2_YAW_FUSED_WINDOW_S;
    if ((now_ms - g_ekf2_flight_start_ms) >
            static_cast<uint32_t>(FF_EKF2_FLIGHT_ANCHOR_MIN_FLY_S * 1000.0f) &&
        g_ekf2_alt_hold_s >= FF_EKF2_FLIGHT_ANCHOR_ALT_HOLD_S &&
        g_yaw_est.current_sample.valid &&
        g_yaw_est.current_sample.current_a > FF_EKF2_FLIGHT_ANCHOR_MIN_CURRENT_A &&
        yaw_obs_fused && g_ekf2_flight_window.full() &&
        sensorHubFfCorrectionActive() && g_yaw_est.ff.anchor_valid) {
        MagVector b0f;
        float i_mean = 0.0f;
        g_ekf2_flight_window.mean(b0f, i_mean);
        (void)i_mean;  // 飛行アンカーでは電流平均は使わない(FF の I_idle は不変)
        const MagVector b0h = levelMagVectorBody(roll_rad, pitch_rad, b0f);
        g_yaw_est.yaw_kf2.flightReanchor(b0h.x, b0h.y, b0f);
        g_ekf2_flight_anchor_done = true;
        comm_send_log("EKF2飛行状態再アンカー(B0を飛行電流状態で再取得)");
    }
}

}  // namespace

void sensorHubFfInit(TwoWire& wire) {
    g_wire = &wire;
    g_yaw_est.bmm_ready = g_yaw_est.bmm150.begin(wire, BMM150_I2C_ADDRESS);
    if (g_yaw_est.bmm_ready) {
        USBSerial.printf(
            "BMM150 init ok: chip_id=0x%02X trim=%s\n",
            g_yaw_est.bmm150.chipId(),
            g_yaw_est.bmm150.trimValid() ? "ok" : "invalid"
        );
    } else {
        USBSerial.printf("BMM150 init failed: chip_id=0x%02X\n", g_yaw_est.bmm150.chipId());
    }

    g_yaw_est.current_ready = currentSensorInit(wire);
    if (g_yaw_est.current_ready) {
        USBSerial.println("INA3221 init ok: battery current/voltage on CH2 (20Hz)");
    } else {
        USBSerial.println("INA3221 init failed");
    }

    // NVS 復元(契約 §2.5 のブートロード順): mag3d → accel6 → attmount →
    // geomag → yawzero → ffcal → magbias(→ flowcal はフロー移植 §2.4 側が追加)。
    // yawzero はリファレンスCFのリセット後に復元する
    // (リセットが復元済み磁気オフセットを消すため — yaw側 app.cpp 踏襲)。
    loadMag3DCalibration(g_yaw_est.mag3d_calibration);
    loadAccelCalibration(g_yaw_est.accel_calibration);
    loadAttitudeMountZero(
        g_yaw_est.attitude_mount_valid,
        g_yaw_est.roll_mount_offset_rad,
        g_yaw_est.pitch_mount_offset_rad
    );
    loadGeomagneticReference(g_yaw_est.yaw_estimator);
    g_yaw_est.yaw_estimator.reset(0.0f);
    loadYawZero(g_yaw_est.yaw_estimator);
    loadFfCalibration(g_yaw_est.ff_calibration, g_yaw_est.ff.ff_mode, g_yaw_est.ff.est_mode);
    loadMagbias(g_yaw_est.magbias_valid, g_yaw_est.magbias);
    // 補正CFはリファレンスと同じ状態から開始する。
    g_yaw_est.yaw_estimator_corr = g_yaw_est.yaw_estimator;
}

void sensorHubFfUpdate(const SensorHubFfInputs& in) {
    // 入力スナップショット(20Hzスロット内のFF計算・手動アンカーが参照)
    g_yaw_est.motors_running = in.motors_running;
    for (uint8_t m = 0; m < 4; m++) {
        g_yaw_est.duty[m] = in.duty[m];
    }
    g_yaw_est.motor_mask = in.motor_mask;
    g_in_flight = in.in_flight;
    g_yaw_est.current_slot_fired = false;

    YawFfMagFrame& frame = g_yaw_est.frame;
    frame.mag_sample_fresh = false;
    frame.mag_dt_s = 0.0f;

    // レベル化に使う姿勢: Madgwick ロール/ピッチにマウントオフセットを適用
    // (yaw側の roll/pitch_tilt_comp_rad 相当。適用先は磁気レベル化のみで、
    //  飛行制御の姿勢には影響しない)。
    const float roll_rad = g_yaw_est.attitude_mount_valid
        ? applyAttitudeOffset(in.roll_rad, g_yaw_est.roll_mount_offset_rad)
        : in.roll_rad;
    const float pitch_rad = g_yaw_est.attitude_mount_valid
        ? applyAttitudeOffset(in.pitch_rad, g_yaw_est.pitch_mount_offset_rad)
        : in.pitch_rad;
    g_last_roll_rad = roll_rad;
    g_last_pitch_rad = pitch_rad;

    float dt_s = in.dt_s;
    if (dt_s <= 0.0f || dt_s > 0.2f) {
        dt_s = SENSOR_PERIOD_US * 1.0e-6f;
    }

    // 20Hz スロット: updateCurrent → updateMagnetometer の順(同tick内順序契約)。
    // ToF 読みが発生した tick ではスキップし次 tick へ繰延べる(位相スタガ:
    // ToF の重量級I2C読みと磁気/電流読みの同tick集中による 2.5ms 予算超過回避)。
    if (!in.tof_read_this_tick) {
        updateCurrent();
        updateMagnetometer();
    }

    // テレメトリ用: 補正系EMAのレベル化水平成分(=EKF観測 zx/zy)を fresh 磁気
    // tick でその時の姿勢により更新(契約 §2.2 の sensor_state 公開の源泉。
    // 更新間は直近の EKF 観測値を保持する)。
    if (frame.mag_sample_fresh) {
        const MagVector lev =
            levelMagVectorBody(roll_rad, pitch_rad, frame.mag_corr_filtered_body);
        frame.mag_lev_x_ut = lev.x;
        frame.mag_lev_y_ut = lev.y;
    }

    // ステージ済みヨー観測の取り出し(consume-once。契約 §2.1-1 / §2.3)。
    // age 検査を通った分のみ EKF2 へ融合する(FF 非稼働時も消費して破棄)。
    bool yaw_obs_new = false;
    float yaw_obs_rad = 0.0f;
    bool yaw_obs_low_trust = false;
    if (g_yaw_obs_pending) {
        g_yaw_obs_pending = false;
        if (millis() - g_yaw_obs_rx_ms < FF_EKF2_YAW_OBS_MAX_AGE_MS) {
            yaw_obs_new = true;
            yaw_obs_rad = g_yaw_obs_rad;
            yaw_obs_low_trust = g_yaw_obs_low_trust;
        }
    }

    // リファレンスCF (既存・非補正mag)。ffモードoff時のアクティブ出力でもある。
    g_yaw_est.yaw_estimator.update(
        in.yaw_rate_rad_s,
        roll_rad,
        pitch_rad,
        frame.mag_filtered_body,
        dt_s,
        frame.mag_sample_fresh,
        frame.mag_dt_s
    );

    // 補正系推定器 (ffモードoff/ffcal無効/電流無効時はリファレンスのみ稼働)。
    const bool ff_active = sensorHubFfCorrectionActive();
    if (ff_active) {
        // 補正CF: 補正済みEMA磁場で第2インスタンスを回す。
        g_yaw_est.yaw_estimator_corr.update(
            in.yaw_rate_rad_s,
            roll_rad,
            pitch_rad,
            frame.mag_corr_filtered_body,
            dt_s,
            frame.mag_sample_fresh,
            frame.mag_dt_s
        );
        // EKF: 予測は毎tick(dt実測)、更新は fresh 磁気サンプルのみ
        // (hold値での二重実行を mag_sample_fresh で防ぐ)。
        // V2改修A: チルト運動学予測用に p(ロールレート)とレベル化と同じ
        // マウントオフセット適用後の roll/pitch を渡す。
        g_yaw_est.yaw_kf.predict(in.yaw_rate_rad_s, in.roll_rate_rad_s, roll_rad, pitch_rad, dt_s);
        if (frame.mag_sample_fresh && g_yaw_est.ff.anchor_valid) {
            g_yaw_est.yaw_kf.update(
                frame.mag_corr_filtered_body,
                roll_rad,
                pitch_rad,
                g_yaw_est.ff.sigma_ff_uT,
                g_yaw_est.ff.sigma_slew_uT,
                g_yaw_est.ff.sigma_diff_uT,
                frame.mag_dt_s
            );
        }
        // EKF2: 常時シャドー実行(契約 §2.2)。EKF1 と同一の磁気観測入力・
        // 同一条件の predict/update に加え、ヨー擬似観測(§2.1-1)を融合する。
        g_yaw_est.yaw_kf2.predict(
            in.yaw_rate_rad_s, in.roll_rate_rad_s, roll_rad, pitch_rad, dt_s);
        if (frame.mag_sample_fresh && g_yaw_est.ff.anchor_valid) {
            g_yaw_est.yaw_kf2.update(
                frame.mag_corr_filtered_body,
                roll_rad,
                pitch_rad,
                g_yaw_est.ff.sigma_ff_uT,
                g_yaw_est.ff.sigma_slew_uT,
                g_yaw_est.ff.sigma_diff_uT,
                frame.mag_dt_s
            );
        }
        if (yaw_obs_new) {
            g_yaw_est.yaw_kf2.updateYawObs(yaw_obs_rad, yaw_obs_low_trust, dt_s);
        }
        // est_mode==2 のときのみ EKF2 の出力が yaw_active へ(契約 §2.2)
        g_yaw_est.ff.yaw_active_rad = g_yaw_est.ff.est_mode == 2
            ? g_yaw_est.yaw_kf2.yaw()
            : (g_yaw_est.ff.est_mode == 1
                   ? g_yaw_est.yaw_kf.yaw()
                   : g_yaw_est.yaw_estimator_corr.yaw());
    } else {
        g_yaw_est.ff.yaw_active_rad = g_yaw_est.yaw_estimator.yaw();
    }

    // アイドルアンカー窓の更新と始動遷移での凍結 (§5.3)。
    ffAnchorService(in.motors_running);

    // EKF2 飛行状態再アンカー(契約 §2.1-3)。
    ekf2FlightAnchorService(in, roll_rad, pitch_rad, dt_s);
}

bool sensorHubFfAnchorNow() {
    // 回転中(ランプダウン中の実出力を含む)はまだ磁気汚染があり得るので、
    // 実出力ゼロまで手動アンカーも拒否する (ffAnchorService と同基準)。
    if (g_yaw_est.motors_running) {
        return false;
    }
    return ffFreezeAnchorFromWindow();
}

void sensorHubFfReseed() {
    ffReseedCorrectedEstimators();
}

bool sensorHubFfCorrectionActive() {
    return g_yaw_est.ff.ff_mode != 0 && g_yaw_est.ff_calibration.valid() &&
           g_yaw_est.current_ready && g_yaw_est.current_sample.valid;
}

void sensorHubFfOnMag3dChange() {
    // mag3d較正の変更で b_cal 空間が変わるため、旧空間で凍結した B0 アンカーを
    // 破棄し、窓を貯め直し、ブート時自動アンカーを再武装、補正系推定器を再シード。
    // 係数blobはNVSに残す (ffmode で再有効化可能、ただし旧空間係数の妥当性は
    // ユーザー責任) が、補正モード自体は安全側の off に落として永続化する。
    g_yaw_est.ff.anchor_valid = false;
    g_ff_anchor_window.reset();
    g_ff_boot_anchor_done = false;
    g_ekf2_flight_anchor_done = false;
    // magbias は旧 b_cal 空間で無効になるため連動クリアする(契約 §2.5。
    // ffcal の ff_mode 降格と同時)。
    if (g_yaw_est.magbias_valid) {
        g_yaw_est.magbias_valid = false;
        g_yaw_est.magbias = MagVector{};
        clearMagbias();
    }
    ffReseedCorrectedEstimators();
    if (g_yaw_est.ff.ff_mode != 0) {
        g_yaw_est.ff.ff_mode = 0;
        saveFfModes(g_yaw_est.ff.ff_mode, g_yaw_est.ff.est_mode);
    }
}

bool sensorHubFfEkfHealthy() {
    // アンカー判定は EKF 磁気更新ゲート(sensorHubFfUpdate 内の
    // mag_sample_fresh && ff.anchor_valid)と同じ ff.anchor_valid を使う。
    // yaw_kf.anchorValid() は reanchor() 後にクリアされる経路が無く、
    // sensorHubFfOnMag3dChange() が ff.anchor_valid だけを落とした後に
    // true のまま残る(→ predict-only の EKF を「健全」と誤判定し、契約
    // §2.3 のレートダンピング縮退が働かない)。ff.anchor_valid=true の
    // とき yaw_kf.anchorValid() は必ず true(凍結時に必ず reanchor する)
    // なので、この置換で判定は狭くなる方向にのみ変わる。
    return sensorHubFfCorrectionActive() && g_yaw_est.ff.anchor_valid &&
           (g_yaw_est.yaw_kf.gateBits() & FF_EKF_GATE_BM_FROZEN) == 0;
}

bool sensorHubFfMagFresh(uint32_t now_ms) {
    const uint32_t last = g_yaw_est.frame.last_mag_fresh_ms;
    return last != 0 && (now_ms - last) < YAW_MAG_FRESH_TIMEOUT_MS;
}

void sensorHubFfStageYawObs(float psi_meas_rad, bool low_trust, uint32_t rx_ms) {
    if (!isfinite(psi_meas_rad)) {
        return;
    }
    g_yaw_obs_rad = wrapPi(psi_meas_rad);
    g_yaw_obs_low_trust = low_trust;
    g_yaw_obs_rx_ms = rx_ms;
    g_yaw_obs_pending = true;
    g_yaw_obs_last_rx_ms = rx_ms;          // ekf2_status bit0(受信鮮度)の根拠
    g_yaw_obs_last_low_trust = low_trust;  // ekf2_status bit6
}

void sensorHubFfApplyMagbias(bool valid, const MagVector& delta_b_ut) {
    // 契約 §1.4 CMD_MAGBIAS_SET: b_corr 空間が Δb 分変わるため、mag3d 変更時と
    // 同パターンでアンカー無効化+窓リセット+ブート時自動アンカー再武装+
    // 補正系再シードを行う。ff_mode は降格しない(そこだけ mag3d と異なる —
    // FF 係数は b_cal 空間のまま有効)。NVS 保存(saveMagbias)は呼び出し側。
    g_yaw_est.magbias_valid = valid;
    g_yaw_est.magbias = valid ? delta_b_ut : MagVector{};
    g_yaw_est.ff.anchor_valid = false;
    g_ff_anchor_window.reset();
    g_ff_boot_anchor_done = false;
    g_ekf2_flight_anchor_done = false;
    ffReseedCorrectedEstimators();
}

bool sensorHubFfEkf2Healthy() {
    // healthy2(契約 §2.2): sensorHubFfEkfHealthy() と同等の判定を EKF2 に適用。
    // yaw_obs 鮮度は含めない(MoCap 途絶ではコーストし、ヨー角制御を落とさない)。
    return sensorHubFfCorrectionActive() && g_yaw_est.ff.anchor_valid &&
           (g_yaw_est.yaw_kf2.gateBits() & FF_EKF_GATE_BM_FROZEN) == 0;
}

uint8_t sensorHubFfEkf2Status(uint32_t now_ms) {
    // TLM_STATE ekf2_status(契約 §1.1): bit0 yaw_obs_fresh(受信<1s) /
    // bit1 yaw_obs_fused(直近0.5s内受理) / bit2 flight_anchor_done /
    // bit3 tau_rw_mode / bit4 bm_frozen / bit5 healthy2 /
    // bit6 yaw_obs_low_trust / bit7 予約。
    using stampfly::TlmState;
    const YawEstimatorKf2& kf2 = g_yaw_est.yaw_kf2;
    uint8_t status = 0;
    if (g_yaw_obs_last_rx_ms != 0 &&
        (now_ms - g_yaw_obs_last_rx_ms) <
            static_cast<uint32_t>(FF_EKF2_YAW_FRESH_WINDOW_S * 1000.0f)) {
        status |= TlmState::EKF2_STATUS_YAW_OBS_FRESH;
    }
    if (kf2.timeSinceYawAccept() < FF_EKF2_YAW_FUSED_WINDOW_S) {
        status |= TlmState::EKF2_STATUS_YAW_OBS_FUSED;
    }
    if (g_ekf2_flight_anchor_done) {
        status |= TlmState::EKF2_STATUS_FLIGHT_ANCHOR_DONE;
    }
    if (kf2.tauRwMode()) {
        status |= TlmState::EKF2_STATUS_TAU_RW_MODE;
    }
    if (kf2.magFrozen()) {
        status |= TlmState::EKF2_STATUS_BM_FROZEN;
    }
    if (sensorHubFfEkf2Healthy()) {
        status |= TlmState::EKF2_STATUS_HEALTHY2;
    }
    if (g_yaw_obs_last_low_trust) {
        status |= TlmState::EKF2_STATUS_YAW_OBS_LOW_TRUST;
    }
    return status;
}
