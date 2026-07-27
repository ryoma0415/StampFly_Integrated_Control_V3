#ifndef STAMPFLY_FLOW_PERSISTENCE_HPP
#define STAMPFLY_FLOW_PERSISTENCE_HPP

// フロースケール較正の NVS 永続化 (MAG_AUTOTUNE_DESIGN.md §2.5)。
// NVS namespace "flowcal":
//   schema(u32=1) / valid(u8) / kx(f32) / ky(f32) / crc(u32)
// crc は {kx, ky} の float32 列に対する CRC-32 (ffcal/magbias と同じ
// FfCalibration::crc32Of を共用)。ロード時は schema/CRC/値域
// (FLOW_SCALE_MIN/MAX_COUNTS_PER_RAD) を照合し、破損は自己修復破棄する。
// 未設定時の適用スケールは既定 450.0 (FLOW_DEFAULT_SCALE_COUNTS_PER_RAD)。
// 書き込みは CMD_FLOWCAL_SET (非飛行状態のみ受理) からのみ行う。
//
// 【V2変更】移植元 (StampFly_Telemetry flow_persistence) は軸マッピング
// (xsrc/ysrc/xsig/ysig) も永続化していたが、契約 §2.5 のスキーマは kx/ky のみ。
// 軸マッピング・符号は flow_hub の既定値 (x=dx/+, y=dy/+) に固定する。

void saveFlowcal(bool valid, float kx, float ky);
void clearFlowcal();
// 成功時 valid=true と kx/ky を返す。未設定・破損時は valid=false のまま。
void loadFlowcal(bool& valid, float& kx, float& ky);

#endif
