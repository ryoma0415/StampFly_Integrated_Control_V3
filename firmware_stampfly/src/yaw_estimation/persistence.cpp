#include "persistence.hpp"

#include <Arduino.h>
#include <Preferences.h>
#include <math.h>

#include "yaw_config.hpp"

namespace {
const uint32_t MAG3D_CALIBRATION_SCHEMA = 2;
const uint32_t GEOMAGNETIC_SCHEMA = 1;
const uint32_t FFCAL_SCHEMA = 1;
const uint32_t MAGBIAS_SCHEMA = 1;  // MAG_AUTOTUNE_DESIGN.md §2.5

Preferences preferences;

// Persist/restore the nine 3D soft-iron matrix elements as "m0".."m8" in NVS.
// Both helpers assume the caller has already opened the "mag3d" namespace.
void putMag3DMatrix(const float* matrix) {
    for (uint8_t i = 0; i < 9; i++) {
        char key[5];
        snprintf(key, sizeof(key), "m%u", i);
        preferences.putFloat(key, matrix[i]);
    }
}

void getMag3DMatrix(float* matrix) {
    for (uint8_t i = 0; i < 9; i++) {
        char key[5];
        snprintf(key, sizeof(key), "m%u", i);
        matrix[i] = preferences.getFloat(key, (i == 0 || i == 4 || i == 8) ? 1.0f : 0.0f);
    }
}
}  // namespace

void saveMag3DCalibration(const MagSoftIronCalibration& calibration) {
    preferences.begin("mag3d", false);
    preferences.putUInt("schema", MAG3D_CALIBRATION_SCHEMA);
    preferences.putBool("valid", calibration.valid());
    if (calibration.valid()) {
        const MagVector off = calibration.offset();
        const float* matrix = calibration.matrix();
        preferences.putFloat("ox", off.x);
        preferences.putFloat("oy", off.y);
        preferences.putFloat("oz", off.z);
        putMag3DMatrix(matrix);
    }
    preferences.end();
}

void loadMag3DCalibration(MagSoftIronCalibration& calibration) {
    preferences.begin("mag3d", true);
    const bool valid = preferences.getBool("valid", false);
    const uint32_t schema = preferences.getUInt("schema", 0);
    const bool schema_matches = schema == MAG3D_CALIBRATION_SCHEMA;
    if (valid && schema_matches) {
        MagVector offset{
            preferences.getFloat("ox", 0.0f),
            preferences.getFloat("oy", 0.0f),
            preferences.getFloat("oz", 0.0f),
        };
        float matrix[9];
        getMag3DMatrix(matrix);
        if (calibration.set(offset, matrix)) {
            USBSerial.println("Loaded 3D magnetometer calibration");
        }
    }
    preferences.end();

    if (valid && !schema_matches) {
        calibration.reset();
        saveMag3DCalibration(calibration);
        USBSerial.println("Ignored old 3D magnetometer calibration; recalibrate after BMM150 trim compensation");
    }
}

void saveGeomagneticReference(const GeomagneticReference& reference) {
    preferences.begin("geomag", false);
    preferences.putUInt("schema", GEOMAGNETIC_SCHEMA);
    preferences.putBool("valid", reference.valid);
    preferences.putBool("reject", reference.use_for_rejection);
    preferences.putFloat("decl", reference.declination_east_rad);
    preferences.putFloat("incl", reference.inclination_rad);
    preferences.putFloat("h", reference.horizontal_uT);
    preferences.putFloat("z", reference.vertical_uT);
    preferences.putFloat("f", reference.total_uT);
    preferences.putFloat("ftol", reference.total_tolerance_ratio);
    preferences.putFloat("htol", reference.horizontal_tolerance_ratio);
    preferences.putFloat("itol", reference.inclination_tolerance_rad);
    preferences.putFloat("izsgn", reference.inclination_z_sign);
    preferences.end();
}

void loadGeomagneticReference(YawEstimator& yaw_estimator) {
    preferences.begin("geomag", true);
    const bool valid = preferences.getBool("valid", false);
    const uint32_t schema = preferences.getUInt("schema", 0);
    if (valid && schema == GEOMAGNETIC_SCHEMA) {
        GeomagneticReference reference;
        reference.valid = true;
        reference.use_for_rejection = preferences.getBool("reject", true);
        reference.declination_east_rad = preferences.getFloat("decl", 0.0f);
        reference.inclination_rad = preferences.getFloat("incl", 0.0f);
        reference.horizontal_uT = preferences.getFloat("h", 0.0f);
        reference.vertical_uT = preferences.getFloat("z", 0.0f);
        reference.total_uT = preferences.getFloat("f", 0.0f);
        reference.total_tolerance_ratio = preferences.getFloat("ftol", 0.35f);
        reference.horizontal_tolerance_ratio = preferences.getFloat("htol", 0.45f);
        reference.inclination_tolerance_rad = preferences.getFloat("itol", 20.0f * DEG_TO_RAD);
        reference.inclination_z_sign = preferences.getFloat("izsgn", GEOMAG_INCLINATION_Z_SIGN);
        if (yaw_estimator.setGeomagneticReference(reference)) {
            USBSerial.println("Loaded geomagnetic reference");
        }
    }
    preferences.end();
}

void saveAccelCalibration(const AccelSixFaceCalibration& calibration) {
    preferences.begin("accel6", false);
    preferences.putBool("valid", calibration.valid());
    if (calibration.valid()) {
        const AccelVector off = calibration.offset();
        const AccelVector sc = calibration.scale();
        preferences.putFloat("ox", off.x);
        preferences.putFloat("oy", off.y);
        preferences.putFloat("oz", off.z);
        preferences.putFloat("sx", sc.x);
        preferences.putFloat("sy", sc.y);
        preferences.putFloat("sz", sc.z);
    }
    preferences.end();
}

void loadAccelCalibration(AccelSixFaceCalibration& calibration) {
    preferences.begin("accel6", true);
    const bool valid = preferences.getBool("valid", false);
    if (valid) {
        const AccelVector offset{
            preferences.getFloat("ox", 0.0f),
            preferences.getFloat("oy", 0.0f),
            preferences.getFloat("oz", 0.0f),
        };
        const AccelVector scale{
            preferences.getFloat("sx", 1.0f),
            preferences.getFloat("sy", 1.0f),
            preferences.getFloat("sz", 1.0f),
        };
        if (calibration.setCalibration(offset, scale)) {
            USBSerial.println("Loaded 6-face accelerometer calibration");
        }
    }
    preferences.end();
}

void saveAttitudeMountZero(bool valid, float roll_rad, float pitch_rad) {
    preferences.begin("attmount", false);
    preferences.putBool("valid", valid);
    preferences.putFloat("roll", roll_rad);
    preferences.putFloat("pitch", pitch_rad);
    preferences.end();
}

void loadAttitudeMountZero(bool& valid, float& roll_rad, float& pitch_rad) {
    preferences.begin("attmount", true);
    valid = preferences.getBool("valid", false);
    roll_rad = preferences.getFloat("roll", 0.0f);
    pitch_rad = preferences.getFloat("pitch", 0.0f);
    preferences.end();
    // Range check doubles as self-healing: an out-of-range offset persisted by
    // an older firmware would stall the attitude wrap on every boot.
    if (!valid || !isfinite(roll_rad) || !isfinite(pitch_rad) ||
        fabsf(roll_rad) > PI || fabsf(pitch_rad) > PI) {
        valid = false;
        roll_rad = 0.0f;
        pitch_rad = 0.0f;
    } else {
        USBSerial.println("Loaded mounting attitude zero");
    }
}

void saveYawZero(bool valid, float mag_yaw_offset_rad) {
    preferences.begin("yawzero", false);
    preferences.putBool("valid", valid);
    preferences.putFloat("off", mag_yaw_offset_rad);
    preferences.end();
}

void loadYawZero(YawEstimator& yaw_estimator) {
    preferences.begin("yawzero", true);
    const bool valid = preferences.getBool("valid", false);
    const float offset = preferences.getFloat("off", 0.0f);
    preferences.end();
    if (valid && isfinite(offset) && fabsf(offset) <= PI) {
        yaw_estimator.restoreYawZero(offset);
        USBSerial.println("Loaded yaw zero reference");
    }
}

void saveFfCalibration(const FfCalibration& calibration, uint8_t ff_mode, uint8_t est_mode) {
    preferences.begin("ffcal", false);
    preferences.putUInt("schema", FFCAL_SCHEMA);
    preferences.putBool("valid", calibration.valid());
    if (calibration.valid()) {
        float values[FF_LUT_MAX_POINTS * 4 + 25];
        calibration.serialize(values);
        const uint16_t count = calibration.blobFloatCount();
        preferences.putUInt("nlut", calibration.nlut());
        preferences.putUInt("crc", calibration.crc());
        preferences.putBytes("blob", values, count * sizeof(float));
    }
    preferences.putUInt("ff", ff_mode);
    preferences.putUInt("est", est_mode);
    preferences.end();
}

void saveFfModes(uint8_t ff_mode, uint8_t est_mode) {
    preferences.begin("ffcal", false);
    preferences.putUInt("ff", ff_mode);
    preferences.putUInt("est", est_mode);
    preferences.end();
}

void clearFfCalibration() {
    preferences.begin("ffcal", false);
    preferences.putUInt("schema", FFCAL_SCHEMA);
    preferences.putBool("valid", false);
    preferences.putUInt("ff", 0);  // 較正が無ければ補正モードは off
    preferences.end();
}

void loadFfCalibration(FfCalibration& calibration, uint8_t& ff_mode, uint8_t& est_mode) {
    ff_mode = 0;
    est_mode = 0;

    preferences.begin("ffcal", true);
    const uint32_t schema = preferences.getUInt("schema", 0);
    const bool valid = preferences.getBool("valid", false);
    const uint32_t nlut = preferences.getUInt("nlut", 0);
    const uint32_t crc = preferences.getUInt("crc", 0);
    const uint32_t ff_stored = preferences.getUInt("ff", 0);
    const uint32_t est_stored = preferences.getUInt("est", 0);

    bool restored = false;
    bool corrupt = false;
    if (valid && schema == FFCAL_SCHEMA &&
        nlut >= FF_LUT_MIN_POINTS && nlut <= FF_LUT_MAX_POINTS) {
        float values[FF_LUT_MAX_POINTS * 4 + 25];
        const size_t expected_bytes =
            FfCalibration::blobFloatCountFor(static_cast<uint8_t>(nlut)) * sizeof(float);
        const size_t read_bytes = preferences.getBytes("blob", values, sizeof(values));
        if (read_bytes == expected_bytes &&
            FfCalibration::crc32Of(values, FfCalibration::blobFloatCountFor(static_cast<uint8_t>(nlut))) == crc &&
            calibration.restoreFromBlob(values, static_cast<uint8_t>(nlut), crc)) {
            restored = true;
        } else {
            corrupt = true;
        }
    } else if (valid) {
        corrupt = true;  // schema 不一致や nlut 範囲外も破棄対象
    }
    preferences.end();

    if (restored) {
        ff_mode = ff_stored <= 2 ? static_cast<uint8_t>(ff_stored) : 0;
        // est の受理範囲は 0..2 (2=EKF2。MAG_AUTOTUNE_DESIGN.md §1.3)
        est_mode = est_stored <= 2 ? static_cast<uint8_t>(est_stored) : 0;
        USBSerial.printf(
            "Loaded FF calibration: nlut=%lu crc=%08lx ff=%u est=%u\n",
            static_cast<unsigned long>(nlut),
            static_cast<unsigned long>(crc),
            ff_mode,
            est_mode
        );
    } else if (corrupt) {
        // CRC/スキーマ不一致は自己修復破棄 (mag3d の前例に従う)。
        calibration.clear();
        clearFfCalibration();
        USBSerial.println("Ignored corrupt FF calibration; re-send via ffcal_* commands");
    }
}

// ---- 学習ハードアイアン残差 Δb (MAG_AUTOTUNE_DESIGN.md §2.5) ----
// NVS namespace "magbias": schema(u32=1) / valid(u8) / blob(f32×3) / crc(u32)。
// CRC は ffcal と同じ CRC-32 (IEEE, float32 列) を FfCalibration::crc32Of で共用。

void saveMagbias(bool valid, const MagVector& delta_b_ut) {
    preferences.begin("magbias", false);
    preferences.putUInt("schema", MAGBIAS_SCHEMA);
    preferences.putUChar("valid", valid ? 1 : 0);
    if (valid) {
        float blob[3] = {delta_b_ut.x, delta_b_ut.y, delta_b_ut.z};
        preferences.putBytes("blob", blob, sizeof(blob));
        preferences.putUInt("crc", FfCalibration::crc32Of(blob, 3));
    }
    preferences.end();
}

void clearMagbias() {
    saveMagbias(false, MagVector{});
}

void loadMagbias(bool& valid, MagVector& delta_b_ut) {
    valid = false;
    delta_b_ut = MagVector{};

    preferences.begin("magbias", true);
    const uint32_t schema = preferences.getUInt("schema", 0);
    const uint8_t stored_valid = preferences.getUChar("valid", 0);
    float blob[3] = {0.0f, 0.0f, 0.0f};
    const size_t read_bytes = preferences.getBytes("blob", blob, sizeof(blob));
    const uint32_t crc = preferences.getUInt("crc", 0);
    preferences.end();

    bool corrupt = false;
    if (stored_valid != 0 && schema == MAGBIAS_SCHEMA) {
        if (read_bytes == sizeof(blob) &&
            FfCalibration::crc32Of(blob, 3) == crc &&
            isfinite(blob[0]) && isfinite(blob[1]) && isfinite(blob[2])) {
            valid = true;
            delta_b_ut = MagVector{blob[0], blob[1], blob[2]};
            USBSerial.printf(
                "Loaded magbias: db=(%.2f, %.2f, %.2f) uT\n",
                static_cast<double>(blob[0]),
                static_cast<double>(blob[1]),
                static_cast<double>(blob[2])
            );
        } else {
            corrupt = true;
        }
    } else if (stored_valid != 0) {
        corrupt = true;  // schema 不一致も破棄対象
    }

    if (corrupt) {
        // CRC/スキーマ不一致は自己修復破棄 (mag3d / ffcal の前例に従う)。
        clearMagbias();
        USBSerial.println("Ignored corrupt magbias; re-apply via magbias command");
    }
}
