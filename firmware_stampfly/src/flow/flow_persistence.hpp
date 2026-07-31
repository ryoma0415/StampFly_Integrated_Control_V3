#ifndef STAMPFLY_FLOW_PERSISTENCE_HPP
#define STAMPFLY_FLOW_PERSISTENCE_HPP

// フロー較正行列の NVS 永続化 (MAG_AUTOTUNE_DESIGN.md §2.5)。
// NVS namespace "flowcal" schema v2(2026-07-31 改訂: 2×2 行列化):
//   schema(u32=2) / valid(u8) / m00,m01,m10,m11(f32) / crc(u32)
// K 行列 [counts/rad] は「機体系レート [rad/s] → センサーカウントレート
// [counts/s]」の写像(行優先)。crc は {m00, m01, m10, m11} の float32 列に
// 対する CRC-32 (ffcal/magbias と同じ FfCalibration::crc32Of を共用)。
// ロード時は schema/CRC/値域+det (flowcalMatrixValid) を照合し、破損は
// 自己修復破棄する。未設定時の適用行列は既定 diag(450,450)
// (FLOW_DEFAULT_SCALE_COUNTS_PER_RAD)。
// 書き込みは CMD_FLOWCAL_SET (非飛行状態のみ受理) からのみ行う。
//
// 【v1 → v2 移行】旧 schema v1 (schema=1 / valid / kx,ky / crc{kx,ky}) を
// ブートロードで検出したら diag(kx,ky) へ移行して v2 で保存し直す
// (旧キー kx/ky は削除)。
//
// 【V2変更】移植元 (StampFly_Telemetry flow_persistence) は軸マッピング
// (xsrc/ysrc/xsig/ysig) も永続化していたが、契約 §2.5 のスキーマは行列のみ。
// 軸マッピング・符号は flow_hub の既定値 (x=dx/+, y=dy/+) に固定する。

void saveFlowcal(bool valid, float m00, float m01, float m10, float m11);
void clearFlowcal();
// 成功時 valid=true と K 行列を返す。未設定・破損時は valid=false のまま
// 既定 diag(450,450) を返す。v1 検出時は diag(kx,ky) へ移行して v2 保存。
void loadFlowcal(bool& valid, float& m00, float& m01, float& m10, float& m11);

#endif
