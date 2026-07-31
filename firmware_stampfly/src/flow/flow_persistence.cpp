// ===========================================================================
// flow_persistence.cpp — フロー較正行列の NVS 永続化 実装
//
// StampFly_Telemetry/Telemetry/firmware/src/flow/flow_persistence.cpp を基に、
// スキーマを MAG_AUTOTUNE_DESIGN.md §2.5 へ差し替えたもの。CRC 照合・自己修復
// 破棄の流儀は yaw_estimation/persistence.cpp の magbias に倣う。
// 【2026-07-31 改訂】schema v2(2×2 行列 m00,m01,m10,m11)。v1(kx/ky)は
// ブートロードで diag(kx,ky) へ移行して v2 保存し直す。
// ===========================================================================
#include "flow_persistence.hpp"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "../yaw_estimation/ff_calibration.hpp"  // crc32Of (ffcal/magbias と共用)
#include "flow_config.hpp"

namespace {
const uint32_t FLOWCAL_SCHEMA_V1 = 1;  // 旧: kx/ky の 2 定数(移行元)
const uint32_t FLOWCAL_SCHEMA = 2;     // MAG_AUTOTUNE_DESIGN.md §2.5(2026-07-31 改訂)

Preferences preferences;

// v1 スケール値域(移行時の照合。v2 は flowcalMatrixValid で行列ごと検査)。
bool scaleInRange(float scale) {
    return isfinite(scale) && scale >= FLOW_SCALE_MIN_COUNTS_PER_RAD &&
           scale <= FLOW_SCALE_MAX_COUNTS_PER_RAD;
}
}  // namespace

void saveFlowcal(bool valid, float m00, float m01, float m10, float m11) {
    preferences.begin("flowcal", false);
    preferences.putUInt("schema", FLOWCAL_SCHEMA);
    preferences.putUChar("valid", valid ? 1 : 0);
    if (valid) {
        preferences.putFloat("m00", m00);
        preferences.putFloat("m01", m01);
        preferences.putFloat("m10", m10);
        preferences.putFloat("m11", m11);
        const float blob[4] = {m00, m01, m10, m11};
        preferences.putUInt("crc", FfCalibration::crc32Of(blob, 4));
    }
    // v1 の残骸キーは常に掃除する(schema 更新後に読む経路はない)。
    preferences.remove("kx");
    preferences.remove("ky");
    preferences.end();
}

void clearFlowcal() {
    saveFlowcal(false, 0.0f, 0.0f, 0.0f, 0.0f);
}

void loadFlowcal(bool& valid, float& m00, float& m01, float& m10, float& m11) {
    valid = false;
    m00 = FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;
    m01 = 0.0f;
    m10 = 0.0f;
    m11 = FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;

    preferences.begin("flowcal", true);
    const uint32_t schema = preferences.getUInt("schema", 0);
    const uint8_t stored_valid = preferences.getUChar("valid", 0);
    const float stored_m00 = preferences.getFloat("m00", 0.0f);
    const float stored_m01 = preferences.getFloat("m01", 0.0f);
    const float stored_m10 = preferences.getFloat("m10", 0.0f);
    const float stored_m11 = preferences.getFloat("m11", 0.0f);
    const float stored_kx = preferences.getFloat("kx", 0.0f);   // v1 移行用
    const float stored_ky = preferences.getFloat("ky", 0.0f);
    const uint32_t crc = preferences.getUInt("crc", 0);
    preferences.end();

    bool corrupt = false;
    if (stored_valid != 0 && schema == FLOWCAL_SCHEMA) {
        const float blob[4] = {stored_m00, stored_m01, stored_m10, stored_m11};
        if (FfCalibration::crc32Of(blob, 4) == crc &&
            flowcalMatrixValid(stored_m00, stored_m01, stored_m10, stored_m11)) {
            valid = true;
            m00 = stored_m00;
            m01 = stored_m01;
            m10 = stored_m10;
            m11 = stored_m11;
            USBSerial.printf(
                "Loaded flowcal: K=[%.1f %.1f; %.1f %.1f] counts/rad\n",
                static_cast<double>(m00), static_cast<double>(m01),
                static_cast<double>(m10), static_cast<double>(m11)
            );
        } else {
            corrupt = true;
        }
    } else if (stored_valid != 0 && schema == FLOWCAL_SCHEMA_V1) {
        // v1(kx/ky)検出: CRC は {kx, ky} の 2 要素列で照合し、成立なら
        // diag(kx,ky) へ移行して v2 で保存し直す(契約 §2.5 改訂)。
        const float blob[2] = {stored_kx, stored_ky};
        if (FfCalibration::crc32Of(blob, 2) == crc &&
            scaleInRange(stored_kx) && scaleInRange(stored_ky)) {
            valid = true;
            m00 = stored_kx;
            m01 = 0.0f;
            m10 = 0.0f;
            m11 = stored_ky;
            saveFlowcal(true, m00, m01, m10, m11);
            USBSerial.printf(
                "Migrated flowcal v1->v2: diag(%.1f, %.1f) counts/rad\n",
                static_cast<double>(m00), static_cast<double>(m11)
            );
        } else {
            corrupt = true;
        }
    } else if (stored_valid != 0) {
        corrupt = true;  // 未知 schema も破棄対象
    }

    if (corrupt) {
        // CRC/スキーマ/値域不一致は自己修復破棄 (mag3d / ffcal / magbias の前例に従う)。
        clearFlowcal();
        USBSerial.println("Ignored corrupt flowcal; re-apply via flowcal command");
    }
}
