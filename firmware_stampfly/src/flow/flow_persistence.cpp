// ===========================================================================
// flow_persistence.cpp — フロースケール較正の NVS 永続化 実装
//
// StampFly_Telemetry/Telemetry/firmware/src/flow/flow_persistence.cpp を基に、
// スキーマを MAG_AUTOTUNE_DESIGN.md §2.5 (schema/valid/kx/ky/crc) へ差し替えた
// もの。CRC 照合・自己修復破棄の流儀は yaw_estimation/persistence.cpp の
// magbias に倣う。
// ===========================================================================
#include "flow_persistence.hpp"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "../yaw_estimation/ff_calibration.hpp"  // crc32Of (ffcal/magbias と共用)
#include "flow_config.hpp"

namespace {
const uint32_t FLOWCAL_SCHEMA = 1;  // MAG_AUTOTUNE_DESIGN.md §2.5

Preferences preferences;

bool scaleInRange(float scale) {
    return isfinite(scale) && scale >= FLOW_SCALE_MIN_COUNTS_PER_RAD &&
           scale <= FLOW_SCALE_MAX_COUNTS_PER_RAD;
}
}  // namespace

void saveFlowcal(bool valid, float kx, float ky) {
    preferences.begin("flowcal", false);
    preferences.putUInt("schema", FLOWCAL_SCHEMA);
    preferences.putUChar("valid", valid ? 1 : 0);
    if (valid) {
        preferences.putFloat("kx", kx);
        preferences.putFloat("ky", ky);
        const float blob[2] = {kx, ky};
        preferences.putUInt("crc", FfCalibration::crc32Of(blob, 2));
    }
    preferences.end();
}

void clearFlowcal() {
    saveFlowcal(false, 0.0f, 0.0f);
}

void loadFlowcal(bool& valid, float& kx, float& ky) {
    valid = false;
    kx = FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;
    ky = FLOW_DEFAULT_SCALE_COUNTS_PER_RAD;

    preferences.begin("flowcal", true);
    const uint32_t schema = preferences.getUInt("schema", 0);
    const uint8_t stored_valid = preferences.getUChar("valid", 0);
    const float stored_kx = preferences.getFloat("kx", 0.0f);
    const float stored_ky = preferences.getFloat("ky", 0.0f);
    const uint32_t crc = preferences.getUInt("crc", 0);
    preferences.end();

    bool corrupt = false;
    if (stored_valid != 0 && schema == FLOWCAL_SCHEMA) {
        const float blob[2] = {stored_kx, stored_ky};
        if (FfCalibration::crc32Of(blob, 2) == crc &&
            scaleInRange(stored_kx) && scaleInRange(stored_ky)) {
            valid = true;
            kx = stored_kx;
            ky = stored_ky;
            USBSerial.printf(
                "Loaded flowcal: kx=%.1f ky=%.1f counts/rad\n",
                static_cast<double>(kx),
                static_cast<double>(ky)
            );
        } else {
            corrupt = true;
        }
    } else if (stored_valid != 0) {
        corrupt = true;  // schema 不一致も破棄対象
    }

    if (corrupt) {
        // CRC/スキーマ/値域不一致は自己修復破棄 (mag3d / ffcal / magbias の前例に従う)。
        clearFlowcal();
        USBSerial.println("Ignored corrupt flowcal; re-apply via flowcal command");
    }
}
