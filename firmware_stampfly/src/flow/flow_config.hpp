// ===========================================================================
// flow_config.hpp — オプティカルフロー(PMW3901)モジュールの定数集約
//
// StampFly_Telemetry/Telemetry/firmware/src/flow/flow_config.hpp からの移植
// (MAG_AUTOTUNE_DESIGN.md §2.4)。ハード仕様の出典: pmw3901_doc_ja.md・
// M5StamFly_spec_ja.md §4.2(SPI 共有ピン)。ゲート値の出典: PMW3901 データ
// シート V1.10 と Bitcraze/PX4 実装の定石(移植元 optical_flow_design.md 参照)。
//
// 【V2変更】読み方式は burst(ハードCS)のみ(契約 §2.4)。読み周期は独立
// 20ms(50Hz)。初期化リトライ定数(FLOW_INIT_RETRY_PERIOD_MS)は削除
// (burst_ok 不成立時は恒久無効 = リトライしない。復帰は CMD_FLOW_PROBE)。
// ===========================================================================
#pragma once

#include <Arduino.h>

// ---------------------------------------------------------------------------
// PMW3901 SPI 接続 (BMI270 と SPI2_HOST を共有)
// ---------------------------------------------------------------------------

// チップセレクト。M5StamFly_spec_ja.md §4.2: G12 = CS2 (PMW3901)。
// imu_init() が G12 を HIGH に待避済みだが、ドライバ側でも独立に設定する。
// 注意: 単発レジスタ読みはアドレス送信後 t_SRAD を NCS Low のまま待つ必要が
// あるため、ESP-IDF spi_master の自動 CS は使えない(spics_io_num=-1 で追加し
// G12 を手動 GPIO 制御する)。burst 読みは逆にハードCS デバイスを一時登録して
// 13B 一括転送する(pmw3901_burst_probe と同じ eco 実証構成)。
static const uint8_t PMW3901_CS_PIN = 12;

// SPI クロック上限 2MHz(データシート)。モードは 3。
static const uint32_t PMW3901_SPI_CLOCK_HZ = 2000000UL;

// 読み出しスロット周期 [µs](50Hz)。
// 【V2変更】契約 §2.4: 既存 20Hz 低速スロット(磁気/電流)と同居しない独立
// 20ms 周期。burst 読み 1 回 ≈52µs(+デバイス登録/解除)で 400Hz tick 予算
// (2500µs)内に収まる。Δカウントはセンサー内部で読み出し間も積算されるので
// 50Hz で情報は失われない(レート換算は実測 dt を使う)。
static const uint32_t FLOW_READ_PERIOD_US = 20000;

// ---------------------------------------------------------------------------
// 品質ゲート
// ---------------------------------------------------------------------------

// データシートの唯一の定量品質基準: SQUAL < 0x19 かつ Shutter_Upper == 0x1F
// (露光飽和=照度不足)でデータ破棄。
static const uint8_t FLOW_SQUAL_MIN = 0x19;
static const uint8_t FLOW_SHUTTER_SAT_UPPER = 0x1F;

// 外れ値上限 [counts/s]。Bitcraze OULIER_LIMIT=100 counts/10ms のレート換算。
// 読み周期変動に対して不変になるよう counts/s で持ち、tick ごとに dt を掛けて
// カウント閾値に直して比較する。
static const float FLOW_DELTA_OUTLIER_CPS = 10000.0f;

// 速度換算が有効な対地距離範囲 [m]。下限はデータシートの動作保証 80mm、
// 上限は VL53L3CX のスケーリング実用上限。
static const float FLOW_MIN_HEIGHT_M = 0.08f;
static const float FLOW_MAX_HEIGHT_M = 2.0f;

// データシートの最大検出角レート [rad/s]。換算後フローレートがこれを超えたら
// 追跡不能域として無効化する。
static const float FLOW_MAX_RATE_RAD_S = 7.4f;

// ---------------------------------------------------------------------------
// スケール較正
// ---------------------------------------------------------------------------

// counts→rad 換算の既定値 [counts/rad](NVS `flowcal` 未設定時の既定。
// 契約 §2.5)。文献実装は 385(PX4)〜794(ArduPilot)、FOV42°/35px×10 の
// 第一原理で ≈477。移植元での実機較正例は ≈472(drone 3)。未較正の速度は
// 最大 2 倍程度のスケール誤差を持ち得る(ride-along テレメトリ用途)。
static const float FLOW_DEFAULT_SCALE_COUNTS_PER_RAD = 450.0f;

// CMD_FLOWCAL_SET で受け付けるスケールの妥当範囲 [counts/rad]。
static const float FLOW_SCALE_MIN_COUNTS_PER_RAD = 100.0f;
static const float FLOW_SCALE_MAX_COUNTS_PER_RAD = 2000.0f;

// 【2026-07-31 改訂】較正は 2×2 行列 K [counts/rad](「機体系レート [rad/s]
// → センサーカウントレート [counts/s]」の写像。契約 §1.4/§2.5)。純スケール
// なら K=diag(kx,ky)、取付回転 φ0 込みなら K=diag(kx,ky)·R(φ0)。適用は
// rate = K⁻¹·counts_rate(ジャイロ補償より前)。対角 m00/m11 は上記スケール
// 範囲、非対角 m01/m10(取付回転成分)は |m| ≤ FLOW_SCALE_MAX まで許容。
// det(K) が下限未満(det≈0 含む)は K⁻¹ が数値不安定になるため拒否する。
// 下限は最小スケール対角行列の det = MIN²(K=diag(kx,ky)·R(φ0) 形なら
// det = kx·ky ≥ MIN² なので実用領域を狭めない)。
static const float FLOW_SCALE_DET_MIN_COUNTS2 =
    FLOW_SCALE_MIN_COUNTS_PER_RAD * FLOW_SCALE_MIN_COUNTS_PER_RAD;

// K 行列の妥当性検査(CMD_FLOWCAL_SET 受理と NVS ロードで共用)。
static inline bool flowcalMatrixValid(float m00, float m01, float m10, float m11) {
    if (!(isfinite(m00) && isfinite(m01) && isfinite(m10) && isfinite(m11))) {
        return false;
    }
    if (m00 < FLOW_SCALE_MIN_COUNTS_PER_RAD || m00 > FLOW_SCALE_MAX_COUNTS_PER_RAD ||
        m11 < FLOW_SCALE_MIN_COUNTS_PER_RAD || m11 > FLOW_SCALE_MAX_COUNTS_PER_RAD) {
        return false;
    }
    if (fabsf(m01) > FLOW_SCALE_MAX_COUNTS_PER_RAD ||
        fabsf(m10) > FLOW_SCALE_MAX_COUNTS_PER_RAD) {
        return false;
    }
    const float det = m00 * m11 - m01 * m10;
    return det >= FLOW_SCALE_DET_MIN_COUNTS2;
}

// ---------------------------------------------------------------------------
// ハードCS Motion_Burst プローブ (CMD_FLOW_PROBE) / 起動時セルフテスト
// ---------------------------------------------------------------------------

// 交互実行 (burst→単発) のサイクル数の既定値と受付範囲。1 サイクル ≈8ms
// (単発読み ~6ms + burst ~0.1ms + 間隔) なので既定 200 で ≈2 秒ブロックする。
// 【V2変更】CMD_FLOW_PROBE の n_cycles は u8 のため上限は 255(移植元の
// 2000 上限はワイヤ上表現できない)。
static const uint16_t FLOW_BURST_PROBE_DEFAULT_CYCLES = 200;
static const uint16_t FLOW_BURST_PROBE_MIN_CYCLES = 20;
static const uint16_t FLOW_BURST_PROBE_MAX_CYCLES = 255;

// burst 直後の単発Δ残差の許容 [counts]。burst が Δ をクリアしていれば残差は
// 蓄積 ~3ms 分 (静止でほぼ 0、手動きでも 1 count 程度) に収まる。
static const int16_t FLOW_BURST_PROBE_RESIDUAL_TOL = 4;

// burst/単発の SQUAL 一致許容。SQUAL は read-clear でない緩変化量なので、
// 同一サイクル内なら大きくは離れない (全ゼロ系デッドリードの検出が目的)。
static const uint8_t FLOW_BURST_PROBE_SQUAL_TOL = 16;

// サイクル間の待ち [ms] (Δ 蓄積窓を確保しつつ全体を数秒に収める)。
static const uint32_t FLOW_BURST_PROBE_CYCLE_GAP_MS = 2;

// プローブ合格判定の閾値(移植元 docs/pmw3901_burst_probe.md §4)。
// burst 転送成功率 >99% / fixed=allFF=0 / allZero<5% / Δ妥当範囲=全数 /
// Δ整合率 >0.95 / SQUAL一致率 >0.90。合格で burst_ok ラッチを立てる。
static const float FLOW_PROBE_PASS_BURST_OK_RATIO = 0.99f;
static const float FLOW_PROBE_PASS_ALLZERO_RATIO = 0.05f;
static const float FLOW_PROBE_PASS_AGREE_RATIO = 0.95f;
static const float FLOW_PROBE_PASS_SQUAL_MATCH_RATIO = 0.90f;

// ---------------------------------------------------------------------------
// 統計・表示
// ---------------------------------------------------------------------------

// 健全率(直近窓)の窓長 [読み回数]。100 読み = 2s @50Hz。
static const uint16_t FLOW_HEALTH_WINDOW_READS = 100;

// デッドレコニング位置のリーク時定数 [s]。0 で純積分。
static const float FLOW_POS_LEAK_TAU_S = 0.0f;
