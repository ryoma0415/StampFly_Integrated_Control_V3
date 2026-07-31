#ifndef STAMPFLY_FLOW_HUB_HPP
#define STAMPFLY_FLOW_HUB_HPP

#include <Arduino.h>

#include "flow_config.hpp"

// オプティカルフロー(PMW3901)の読み出し・速度換算・健全性ゲート・
// デッドレコニングを担うハブ。
// StampFly_Telemetry/Telemetry/firmware/src/flow/flow_hub.hpp からの移植
// (MAG_AUTOTUNE_DESIGN.md §2.4)。ベース sensor.cpp の 400Hz tick から呼ばれ、
// 内部で 50Hz(20ms)に自己スロットルする。telemetry / flight_control が
// 状態を読む。出力はテレメトリ+ログのみ(ride-along。制御・EKF観測は未接続)。
//
// 【V2変更(契約 §2.4)】
//  - 読みは burst(ハードCS 13B ≈52µs)のみ。起動時セルフテスト(burst 1回+
//    単発照合、モーター停止時)で burst_ok を判定し、不成立ならフロー恒久無効
//    (初期化リトライも行わない=飛行挙動完全不変)。復帰は CMD_FLOW_PROBE
//    (プローブ合格で burst_ok ラッチ更新)のみ。
//  - テレメトリ窓アキュムレータ(移植元 acc_*)は V2 のワイヤに対応枠が
//    ないため非移植。
//  - 較正は 2×2 行列 K のみワイヤ設定可(CMD_FLOWCAL_SET。契約 §1.4/§2.5。
//    【2026-07-31 改訂】kx/ky 2 定数 → K 行列化: スケール誤差に加えて取付
//    回転 φ0 を表現する。FLIGHT_ANALYSIS_20260731.md §5)。
//    軸マッピング・符号は既定値(x=dx/+, y=dy/+)に固定。
//
// 速度換算 (FRD 系。Bitcraze mm_flow.c の FLU 測定モデルを座標変換して逆算。
// FLU の ωy は FRD の −q になる点に注意。導出は移植元 docs/optical_flow_design.md):
//   測定モデル:      flow_x_rate = vx/d + q     (前進と機首上げは同符号で混入)
//                    flow_y_rate = vy/d − p
//   純回転中 (v=0):  flow_x_rate = +q,  flow_y_rate = −p   … 較正の規約
//   逆算:            vx = (flow_x_rate − q) · d
//                    vy = (flow_y_rate + p) · d
//   d = ToF 生スラント距離 [m](チルト補正 cosφcosθ は掛けない —
//   フロー式の分母 z/R22 ≈ d で相殺するため)。

// counts→rad 較正行列と軸マッピングの較正値 (行列のみ NVS "flowcal" に永続化)。
// K [counts/rad] は「機体系レート [rad/s] → センサーカウントレート [counts/s]」
// の 2×2 写像(行優先)。純スケールなら K=diag(kx,ky)、取付回転 φ0 込みなら
// K=diag(kx,ky)·R(φ0)。適用は rate = K⁻¹·counts_rate(ジャイロ補償より前)。
// K⁻¹ はロード/適用時に一度計算してキャッシュする(det≈0 は
// flowcalMatrixValid が事前に拒否)。
struct FlowCalibration {
    bool valid = false;    // NVS 較正済みか(false でも既定行列で換算は行う)
    // 軸マッピング(V2: 既定固定。ワイヤ設定なし)。StampFly 実装の搭載向きは
    // 機体x = −dy / 機体y = +dx(Telemetry drone3 純回転較正で実機検証済み:
    // flowcal_20260713_111537.json xsrc=1/xsig=-1/ysrc=0/ysig=+1)。
    // 【2026-07-31 修正】移植時に恒等 (x=dx/+, y=dy/+) へ誤って固定されており、
    // フロー較正パネル初回実行で φ0≈−92° として検出された(欠落90°+実ひねり)。
    // 90°級の入替はここで持ち、K 行列は微小回転・スケールのみを担う。
    uint8_t x_src = 1;     // 0=dx, 1=dy
    uint8_t y_src = 0;
    float x_sign = -1.0f;  // ±1
    float y_sign = 1.0f;
    // K 行列 [counts/rad](既定 diag(450,450))
    float m00 = FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;
    float m01 = 0.0f;
    float m10 = 0.0f;
    float m11 = FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;
    // K⁻¹ キャッシュ [rad/counts](flowHubInit / flowHubApplyCalibration が更新)
    float inv00 = 1.0f / FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;
    float inv01 = 0.0f;
    float inv10 = 0.0f;
    float inv11 = 1.0f / FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;
};

// 健全性ビット(内部集計用。TLM の flow_status とは別 — そちらは
// flowHubStatusByte() が stampfly::TlmState::FLOW_STATUS_* で合成する)。
enum FlowHealthBits : uint8_t {
    FLOW_HEALTH_SENSOR_OK = 1u << 0,   // 初期化成功 (Product ID 0x49 確認済み)
    FLOW_HEALTH_READ_OK = 1u << 1,     // 直近バースト読み成功
    FLOW_HEALTH_SQUAL_OK = 1u << 2,    // SQUAL >= 0x19
    FLOW_HEALTH_SHUTTER_OK = 1u << 3,  // 露光非飽和 (照度十分)
    FLOW_HEALTH_DELTA_OK = 1u << 4,    // |Δ| レート換算の外れ値でない
    FLOW_HEALTH_RANGE_OK = 1u << 5,    // ToF 0.08-2.0m かつ有効
    FLOW_HEALTH_RATE_OK = 1u << 6,     // |フローレート| < 7.4 rad/s
    FLOW_HEALTH_CAL_VALID = 1u << 7,   // NVS 較正済み(vel_valid の条件には含めない)
};

struct FlowState {
    bool sensor_ok = false;   // pmw3901Init 成功
    bool burst_ok = false;    // burst 読み成立ラッチ(起動時セルフテスト/プローブ)
    uint8_t product_id = 0;

    // ---- 直近読み (50Hz burst) ----
    int16_t dx = 0;
    int16_t dy = 0;
    uint8_t squal = 0;
    uint8_t shutter_upper = 0;
    uint8_t shutter_lower = 0;
    uint8_t rawdata_sum = 0;
    uint8_t rawdata_max = 0;
    uint8_t rawdata_min = 0;
    uint8_t motion = 0;
    float read_dt_s = 0.0f;
    uint8_t health_bits = 0;
    // squal/shutter/delta/read が全て OK(照度・面品質の実況。高度は不問)
    bool sample_healthy = false;

    // ---- 換算値 ----
    float fx_raw_cps = 0.0f;  // 生フローレート [counts/s] (センサー軸)
    float fy_raw_cps = 0.0f;
    float flow_x_rate = 0.0f;  // 較正後フローレート [rad/s] (機体軸, 回転補償前)
    float flow_y_rate = 0.0f;
    float vx = 0.0f;  // 機体軸速度 [m/s] (回転補償+高度スケール後。無効時 0)
    float vy = 0.0f;
    bool vel_valid = false;  // 【V2変更】CAL_VALID を除く health_bits 全成立
                             // (未較正でも既定スケール 450 で換算する — ride-along)

    // ---- デッドレコニング位置 [m] (ヨー推定でワールド平面へ回した積分) ----
    // 診断用。速度が無効な区間は保持 (積分停止)。
    float px = 0.0f;
    float py = 0.0f;

    // ---- 統計 ----
    float health_ratio = 0.0f;  // 直近 FLOW_HEALTH_WINDOW_READS の sample_healthy 率
    uint32_t read_count = 0;
    uint32_t read_error_count = 0;
};

// PMW3901 初期化 + 較正の NVS ロード + 起動時セルフテスト(burst_ok 判定)。
// imu_init()(SPI バス初期化)の後、モーター停止が保証されるブート時に呼ぶ
// (sensor_init 内。ブートロード順の最後 = magbias の後。契約 §2.5)。
void flowHubInit();

// 400Hz tick から毎回呼ぶ(内部 20ms 自己スロットル。契約 §2.4 の独立スロット)。
// sensor_ok && burst_ok でないときは何もしない(恒久無効=飛行挙動不変)。
// distance_m は ToF 生スラント距離 [m]、yaw_rad はデッドレコニングの
// ワールド回転に使う(アクティブヨー推定)。
void flowHubUpdate(
    float roll_rate_rad_s,
    float pitch_rate_rad_s,
    float distance_m,
    bool distance_valid,
    float yaw_rad
);

// 較正行列 K の適用+NVS 保存 / クリア(CMD_FLOWCAL_SET。契約 §1.4)。
// 値は flowcalMatrixValid で妥当性チェック済みであること(flight_control 層で
// チェック)。K⁻¹ 計算不成立(det≈0。検査通過後は起こり得ない防御)なら
// 何も変えずに false を返す。
bool flowHubApplyCalibration(float m00, float m01, float m10, float m11);
void flowHubClearCalibration();

// CMD_FLOW_PROBE の実処理(契約 §1.4): burst プローブを実行し、結果要約を
// LOG_TEXT 複数行で送出、合否で burst_ok ラッチを更新する。数秒ブロックする
// ため呼ぶのはモーター停止時のみ。n_cycles=0 は既定 200(受付範囲へクランプ)。
// 未初期化(起動時 init 失敗)なら再初期化を 1 回試みる。
// 戻り値: プローブを実行できたか(false = init 不成立 / SPI デバイス登録失敗)。
bool flowHubRunBurstProbe(uint8_t n_cycles);

// TLM_STATE flow_status バイトの合成(stampfly::TlmState::FLOW_STATUS_* を
// 単一ソースとして参照。契約 §1.1)。bit5 init_retry は V2 では常に 0
// (恒久無効方針のため初期化リトライは存在しない)。
uint8_t flowHubStatusByte();

FlowState& flowHubState();
FlowCalibration& flowHubCalibration();

#endif
