#ifndef STAMPFLY_YAW_ESTIMATION_PERSISTENCE_HPP
#define STAMPFLY_YAW_ESTIMATION_PERSISTENCE_HPP

#include "accel_calibration.hpp"
#include "ff_calibration.hpp"
#include "mag_calibration.hpp"
#include "yaw_estimator.hpp"

// Non-volatile storage (NVS / Preferences) of the calibration and reference
// state. State is passed by reference rather than accessed through the shared
// g_app, so this module stays decoupled from the application state.

void saveMag3DCalibration(const MagSoftIronCalibration& calibration);
void loadMag3DCalibration(MagSoftIronCalibration& calibration);

void saveGeomagneticReference(const GeomagneticReference& reference);
void loadGeomagneticReference(YawEstimator& yaw_estimator);

void saveAccelCalibration(const AccelSixFaceCalibration& calibration);
void loadAccelCalibration(AccelSixFaceCalibration& calibration);

void saveAttitudeMountZero(bool valid, float roll_rad, float pitch_rad);
void loadAttitudeMountZero(bool& valid, float& roll_rad, float& pitch_rad);

// User yaw zero (the magnetic offset captured when "Yaw 0" was pressed), so
// the same heading reference survives a reboot. Cleared by yaw_clear.
void saveYawZero(bool valid, float mag_yaw_offset_rad);
void loadYawZero(YawEstimator& yaw_estimator);

// 電流FF較正 (ff_pipeline_design.md §5.1/§5.7)。NVS namespace "ffcal":
// schema(u32=1) / valid(bool) / nlut(u32) / crc(u32) /
// blob(bytes: §4のCRC対象と同一順の float32 列) / ff(u32) / est(u32)。
// ロード時は schema 照合 → blob の CRC 再計算照合 → 不一致なら自己修復破棄
// (mag3d の前例に従う)。ffcal が無効なら ff モードは 0 (off) に落とす。
void saveFfCalibration(const FfCalibration& calibration, uint8_t ff_mode, uint8_t est_mode);
// ffmode コマンド用: blob はそのままに ff/est キーだけ更新する。
void saveFfModes(uint8_t ff_mode, uint8_t est_mode);
// ffcal_clear 用: 無効化して ff モードを off に戻す。
void clearFfCalibration();
void loadFfCalibration(FfCalibration& calibration, uint8_t& ff_mode, uint8_t& est_mode);

// 学習ハードアイアン残差 Δb (MAG_AUTOTUNE_DESIGN.md §2.5)。NVS namespace
// "magbias": schema(u32=1) / valid(u8) / blob(f32×3) / crc(u32)。
// Δb は b_cal 空間 [µT]。ブート時 CRC 照合、破損は自己修復破棄 (ffcal に倣う)。
// mag3d 変更時は旧 b_cal 空間で無効になるため連動クリアする
// (sensorHubFfOnMag3dChange が clearMagbias を呼ぶ)。
void saveMagbias(bool valid, const MagVector& delta_b_ut);
void clearMagbias();
void loadMagbias(bool& valid, MagVector& delta_b_ut);

#endif
