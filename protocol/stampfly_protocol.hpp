// ===========================================================================
// stampfly_protocol.hpp — StampFly Integrated Control 通信プロトコル(C++実装)
//
// docs/PROTOCOL.md v2 が唯一の正(single source of truth)。
// Python実装 stampfly_protocol.py とのバイト互換は test_vectors.json と
// tests/(pytest + host_test.cpp)により強制される。
//
// 設計上の制約:
// - ヘッダオンリー / 純粋C++17。Arduino / ESP-IDF に依存しない
//   (ホスト g++ テストとファームウェアの両方から同一ソースを使う)。
// - 動的確保なし。すべて呼び出し元提供バッファ+上限長で厳格に境界検査する。
// - シリアライズはフィールドごとの明示的リトルエンディアン書き込み
//   (packed構造体の memcpy / パディングに依存しない)。
// - ISR / 受信コールバックから呼べるよう、ブロッキング・出力は一切行わない。
// ===========================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstring>

namespace stampfly {

static_assert(sizeof(float) == 4, "IEEE-754 binary32 float required");

// ---------------------------------------------------------------------------
// 定数(PROTOCOL.md 「論理フレーム」「トランスポート」)
// ---------------------------------------------------------------------------

constexpr uint8_t PROTOCOL_VERSION = 0x02;

constexpr size_t FRAME_HEADER_SIZE = 7;  // ver(1) + type(1) + seq(4) + len(1)
constexpr size_t FRAME_CRC_SIZE = 2;
constexpr size_t FRAME_OVERHEAD = FRAME_HEADER_SIZE + FRAME_CRC_SIZE;  // = 9
constexpr size_t MAX_PAYLOAD_SIZE = 200;
constexpr size_t MAX_FRAME_SIZE = FRAME_OVERHEAD + MAX_PAYLOAD_SIZE;  // = 209

constexpr uint8_t COBS_DELIMITER = 0x00;
constexpr size_t SERIAL_RX_BUFFER_CAP = 256;  // 受信蓄積バッファ上限(超過 → 次の0x00まで読み捨て)

constexpr size_t MAX_LOG_TEXT_SIZE = 180;  // LOG_TEXT の UTF-8 テキスト上限

// --- マルチ機体(リレー多重化)関連 ---
constexpr size_t RLY_MAX_PEERS = 4;        // RLY_SET_PEERS の最大登録数(=同時制御機体数上限)
constexpr size_t RLY_PEER_ENTRY_SIZE = 7;  // mac(6) + tlm_state_div(1)
constexpr size_t MUX_HEADER_SIZE = 1;      // RLY_MUX_UP/DOWN の node_id(1B)
// エンベロープに収まる内側フレームの最大ペイロード長(= 200 - 1 - 9 = 190)。
// 既存の全メッセージ(最大 TLM_STATE 184B)が収まる。
constexpr size_t MAX_MUX_INNER_PAYLOAD =
    MAX_PAYLOAD_SIZE - MUX_HEADER_SIZE - FRAME_OVERHEAD;

// COBSエンコード後の最大長(本実装は満杯ブロック直後にも終端コード0x01を出す方式)
constexpr size_t cobs_max_encoded_size(size_t n) { return n + n / 254 + 2; }
// シリアルワイヤ上の1フレーム最大長(COBS + デリミタ1バイト)
constexpr size_t MAX_WIRE_SIZE = cobs_max_encoded_size(MAX_FRAME_SIZE) + 1;

// ---------------------------------------------------------------------------
// メッセージ型 / enum(PROTOCOL.md 「メッセージ型」「enum定義」)
// ---------------------------------------------------------------------------

enum class MsgType : uint8_t {
  // 上り(PC -> ドローン): 0x10–0x2F
  CMD_START = 0x10,        // 離陸開始(payload 0B)
  CMD_STOP = 0x11,         // 即時着陸(payload 0B、全飛行状態で受理)
  CMD_SETPOINT = 0x12,     // 姿勢+高度+ヨー目標(17B、ハートビート兼用 50Hz)
  CMD_RESET = 0x13,        // COMPLETE からの復帰(payload 0B)
  CMD_MODE = 0x14,         // WAIT<->MOTOR_TEST 切替(1B)
  CMD_MOTOR_RUN = 0x15,    // モーターテスト駆動(5B、0.4s 周期キープアライブ)
  CMD_MOTOR_STOP = 0x16,   // モーター即停止(payload 0B、MOTOR_TEST 内)
  CMD_CAL_GET = 0x17,      // キャリブ一括取得要求(payload 0B → TLM_CAL_DATA)
  CMD_MAG3D_SET = 0x18,    // 3D磁気キャリブ設定(49B)
  CMD_ACCEL6_SET = 0x19,   // 加速度6面キャリブ設定(25B)
  CMD_ATTMOUNT_SET = 0x1A, // 姿勢マウントオフセット設定(9B)
  CMD_YAWZERO_SET = 0x1B,  // ヨーゼロオフセット設定(5B)
  CMD_GEOMAG_SET = 0x1C,   // 地磁気プロファイル設定(20B)
  CMD_FF_BEGIN = 0x1D,     // FF 係数ステージング開始(1B)
  CMD_FF_LUT = 0x1E,       // FF LUT 点(17B)
  CMD_FF_MOT = 0x1F,       // FF モーター係数(25B)
  CMD_FF_AUX = 0x20,       // FF ベンチ参考アイドル電流(4B)
  CMD_FF_COMMIT = 0x21,    // FF 係数 CRC照合+NVS 永続化(4B)
  CMD_FF_MODE = 0x22,      // ff_mode / est_mode 実行時切替(2B)
  CMD_FF_ANCHOR = 0x23,    // アンカー再取得要求(payload 0B)
  CMD_POS_ERR = 0x24,      // XY位置誤差+高度+ヨー目標(21B、機上XY制御。ハートビート兼用 50Hz)
  CMD_LED_MODE = 0x25,     // LED 表示モード切替(1B、計測中インジケータ)
  // 磁気オートチューン/フロー拡張(MAG_AUTOTUNE_DESIGN.md §1.4。追加のみ・
  // ver=0x02 のまま。契約書記載の 0x24-0x26 は既存 CMD_POS_ERR / CMD_LED_MODE
  // と衝突するため、空きの 0x26-0x28 を同順で割当)
  CMD_MAGBIAS_SET = 0x26,  // 学習ハードアイアン残差 Δb 適用/クリア(13B)
  CMD_FLOWCAL_SET = 0x27,  // フロースケール適用/クリア(9B)
  CMD_FLOW_PROBE = 0x28,   // PMW3901 burst プローブ実行(1B)
  // 下り(ドローン -> PC): 0x30–0x4F
  TLM_STATE = 0x30,        // フル状態テレメトリ(184B、25Hz)
  TLM_EVENT = 0x31,        // 状態遷移イベント(8B、即時+2Hz)
  TLM_ACK = 0x32,          // 0x14–0x23, 0x25–0x28 コマンドへの応答(6B)
  TLM_EXP = 0x33,          // 実験テレメトリ(86B、MOTOR_TEST 中のみ 25Hz)
  TLM_CAL_DATA = 0x34,     // キャリブ一括データ(132B、CMD_CAL_GET 応答)
  TLM_CTRL = 0x35,         // 制御ループ診断テレメトリ(89B、25Hz 常時)
  // ログ(リレー/ドローン -> PC)
  LOG_TEXT = 0x40,      // 人間向けテキスト(1〜181B)
  // リレー宛/発: 0x50–0x5F
  RLY_SET_TARGET = 0x50,  // ESP-NOW ピア設定(7B)
  RLY_TARGET_ACK = 0x51,  // SET_TARGET 応答(8B)
  RLY_STATS = 0x52,       // リレー統計(24B、1Hz)
  RLY_PING = 0x53,        // 疎通確認(payload 0B)
  RLY_PONG = 0x54,        // PING 応答(4B)
  // マルチ機体拡張(追加のみ・ver=0x02 のまま。単機経路 0x50/0x51 とは排他)
  RLY_SET_PEERS = 0x55,   // 複数ピア設定(可変長 2+7×N、N=0..4)
  RLY_PEERS_ACK = 0x56,   // SET_PEERS 応答(4B)
  RLY_MUX_UP = 0x57,      // PC→リレー: node_id + 内側フレーム(機体宛)
  RLY_MUX_DOWN = 0x58,    // リレー→PC: node_id + 内側フレーム(機体発)
};

// 型レンジルーティング(リレーは中身を解釈せず型で転送先を決める)
constexpr bool is_uplink_type(uint8_t type) { return type >= 0x10 && type <= 0x2F; }    // ドローン行き
constexpr bool is_downlink_type(uint8_t type) { return type >= 0x30 && type <= 0x4F; }  // PC行き
constexpr bool is_relay_type(uint8_t type) { return type >= 0x50 && type <= 0x5F; }     // リレー宛/発

enum class FlightState : uint8_t {
  INIT = 0,
  CALIBRATION = 1,
  WAIT = 2,
  TAKEOFF = 3,
  HOVER = 4,
  LANDING = 5,
  COMPLETE = 6,
  MOTOR_TEST = 7,  // v2: モーターテストモード(AUTO_MOTOR_TEST)
};

enum class Reason : uint8_t {
  NONE = 0,
  START_CMD = 1,
  STOP_CMD = 2,
  MAX_FLIGHT_TIME = 3,
  LOW_VOLTAGE = 4,
  START_REJECTED_LOW_VOLTAGE = 5,
  LANDED = 6,
  OVER_G = 7,
  LINK_LOSS = 8,
  RESET_CMD = 9,
  START_REJECTED_NOT_READY = 10,
  MODE_CHANGE = 11,  // v2: CMD_MODE による WAIT<->MOTOR_TEST 遷移
};

// ---------------------------------------------------------------------------
// CRC16-CCITT-FALSE(poly 0x1021, init 0xFFFF, 非反転, xorout なし)
// 検証ベクタ: ASCII "123456789" -> 0x29B1
// ---------------------------------------------------------------------------

inline uint16_t crc16_ccitt_false(const uint8_t* data, size_t len, uint16_t crc = 0xFFFFu) {
  for (size_t i = 0; i < len; ++i) {
    crc = static_cast<uint16_t>(crc ^ (static_cast<uint16_t>(data[i]) << 8));
    for (int bit = 0; bit < 8; ++bit) {
      if ((crc & 0x8000u) != 0) {
        crc = static_cast<uint16_t>((crc << 1) ^ 0x1021u);
      } else {
        crc = static_cast<uint16_t>(crc << 1);
      }
    }
  }
  return crc;
}

// ---------------------------------------------------------------------------
// 明示的リトルエンディアン読み書き(ホストのエンディアンに依存しない)
// ---------------------------------------------------------------------------

inline void wr_u8(uint8_t* p, uint8_t v) { p[0] = v; }

inline void wr_u16(uint8_t* p, uint16_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
}

inline void wr_u32(uint8_t* p, uint32_t v) {
  p[0] = static_cast<uint8_t>(v & 0xFFu);
  p[1] = static_cast<uint8_t>((v >> 8) & 0xFFu);
  p[2] = static_cast<uint8_t>((v >> 16) & 0xFFu);
  p[3] = static_cast<uint8_t>((v >> 24) & 0xFFu);
}

inline void wr_f32(uint8_t* p, float v) {
  // floatのビットパターンを取り出してLEで書く(フィールド単位のmemcpyは可搬)
  uint32_t bits = 0;
  std::memcpy(&bits, &v, sizeof(bits));
  wr_u32(p, bits);
}

inline uint8_t rd_u8(const uint8_t* p) { return p[0]; }

inline uint16_t rd_u16(const uint8_t* p) {
  return static_cast<uint16_t>(static_cast<uint16_t>(p[0]) |
                               (static_cast<uint16_t>(p[1]) << 8));
}

inline uint32_t rd_u32(const uint8_t* p) {
  return static_cast<uint32_t>(p[0]) | (static_cast<uint32_t>(p[1]) << 8) |
         (static_cast<uint32_t>(p[2]) << 16) | (static_cast<uint32_t>(p[3]) << 24);
}

inline float rd_f32(const uint8_t* p) {
  const uint32_t bits = rd_u32(p);
  float v = 0.0f;
  std::memcpy(&v, &bits, sizeof(v));
  return v;
}

// ---------------------------------------------------------------------------
// COBS(Consistent Overhead Byte Stuffing)
// ---------------------------------------------------------------------------

// COBSエンコード。出力にデリミタ 0x00 は含まない(送信時は呼び出し元が付加)。
// 「入力末尾が満杯(254B)ブロックで終わる場合にも終端コード0x01を出力する」方式で、
// Python実装と完全に同一のアルゴリズム構造(出力はバイト単位で一致する)。
// 戻り値: 成功なら true、*out_len にエンコード後長。out_cap 不足なら false。
inline bool cobs_encode(const uint8_t* in, size_t in_len,
                        uint8_t* out, size_t out_cap, size_t* out_len) {
  if (out == nullptr || out_len == nullptr) return false;
  if (in == nullptr && in_len > 0) return false;
  size_t oi = 0;
  size_t code_idx = 0;
  uint8_t code = 1;
  if (oi >= out_cap) return false;
  out[oi++] = 0;  // 先頭コードバイトのプレースホルダ
  for (size_t i = 0; i < in_len; ++i) {
    const uint8_t b = in[i];
    if (b == 0) {
      out[code_idx] = code;
      code = 1;
      code_idx = oi;
      if (oi >= out_cap) return false;
      out[oi++] = 0;
    } else {
      if (oi >= out_cap) return false;
      out[oi++] = b;
      ++code;
      if (code == 0xFF) {
        out[code_idx] = code;
        code = 1;
        code_idx = oi;
        if (oi >= out_cap) return false;
        out[oi++] = 0;
      }
    }
  }
  out[code_idx] = code;
  *out_len = oi;
  return true;
}

// COBSデコード。入力にデリミタ 0x00 を含めないこと。
// 0x00 混入・ブロック長不足・空入力・out_cap 超過は false(厳格拒否)。
inline bool cobs_decode(const uint8_t* in, size_t in_len,
                        uint8_t* out, size_t out_cap, size_t* out_len) {
  if (in == nullptr || out == nullptr || out_len == nullptr) return false;
  if (in_len == 0) return false;
  size_t i = 0;
  size_t oi = 0;
  while (i < in_len) {
    const uint8_t code = in[i];
    if (code == 0) return false;  // データ内デリミタは不正
    ++i;
    const size_t block = static_cast<size_t>(code) - 1;
    if (i + block > in_len) return false;  // ブロック切れ
    for (size_t k = 0; k < block; ++k) {
      const uint8_t b = in[i + k];
      if (b == 0) return false;
      if (oi >= out_cap) return false;
      out[oi++] = b;
    }
    i += block;
    if (code != 0xFF && i < in_len) {
      if (oi >= out_cap) return false;
      out[oi++] = 0;
    }
  }
  *out_len = oi;
  return true;
}

// ---------------------------------------------------------------------------
// 論理フレーム pack / parse
// ---------------------------------------------------------------------------

enum class ParseStatus : uint8_t {
  ok = 0,
  bad_ver = 1,
  bad_crc = 2,
  bad_len = 3,
  overflow = 4,
};

// 検証済み論理フレームのビュー。payload は呼び出し元バッファ内を指す(非コピー)。
struct FrameView {
  uint8_t ver = 0;
  uint8_t type = 0;
  uint32_t seq = 0;
  uint8_t len = 0;
  const uint8_t* payload = nullptr;
};

// 論理フレーム(ver..crc16)を out に構築する。
// 戻り値: ok / bad_len(payload_len > 200)/ overflow(out_cap 不足)。
inline ParseStatus pack_frame(uint8_t type, uint32_t seq,
                              const uint8_t* payload, size_t payload_len,
                              uint8_t* out, size_t out_cap, size_t* out_len) {
  if (payload_len > MAX_PAYLOAD_SIZE) return ParseStatus::bad_len;
  if (payload == nullptr && payload_len > 0) return ParseStatus::bad_len;
  const size_t total = FRAME_OVERHEAD + payload_len;
  if (out == nullptr || out_len == nullptr || out_cap < total) return ParseStatus::overflow;
  out[0] = PROTOCOL_VERSION;
  out[1] = type;
  wr_u32(out + 2, seq);
  out[6] = static_cast<uint8_t>(payload_len);
  if (payload_len > 0) std::memcpy(out + FRAME_HEADER_SIZE, payload, payload_len);
  const uint16_t crc = crc16_ccitt_false(out, FRAME_HEADER_SIZE + payload_len);
  wr_u16(out + FRAME_HEADER_SIZE + payload_len, crc);
  *out_len = total;
  return ParseStatus::ok;
}

inline ParseStatus pack_frame(MsgType type, uint32_t seq,
                              const uint8_t* payload, size_t payload_len,
                              uint8_t* out, size_t out_cap, size_t* out_len) {
  return pack_frame(static_cast<uint8_t>(type), seq, payload, payload_len,
                    out, out_cap, out_len);
}

// 論理フレームを検証・分解する。
// 判定順: 構造(bad_len)→ CRC(bad_crc)→ バージョン(bad_ver)。
// CRC を ver 判定より先に行うのは、CRC不一致フレームの ver バイト自体が
// 信用できないため(CRC を通過して ver!=2 のものだけが本当の別バージョン。
// v1 機器の混在は ver_errors として可視化される)。
inline ParseStatus parse_frame(const uint8_t* data, size_t len, FrameView* out) {
  if (data == nullptr || out == nullptr) return ParseStatus::bad_len;
  if (len < FRAME_OVERHEAD) return ParseStatus::bad_len;
  const uint8_t payload_len = data[6];
  if (payload_len > MAX_PAYLOAD_SIZE) return ParseStatus::bad_len;
  if (len != FRAME_OVERHEAD + static_cast<size_t>(payload_len)) return ParseStatus::bad_len;
  const size_t crc_off = FRAME_HEADER_SIZE + payload_len;
  const uint16_t crc_calc = crc16_ccitt_false(data, crc_off);
  const uint16_t crc_rx = rd_u16(data + crc_off);
  if (crc_calc != crc_rx) return ParseStatus::bad_crc;
  if (data[0] != PROTOCOL_VERSION) return ParseStatus::bad_ver;
  out->ver = data[0];
  out->type = data[1];
  out->seq = rd_u32(data + 2);
  out->len = payload_len;
  out->payload = data + FRAME_HEADER_SIZE;
  return ParseStatus::ok;
}

// シリアル区間ワイヤ形式を一括生成: COBS(論理フレーム) + 0x00 デリミタ。
// (タスクコンテキスト用。MAX_FRAME_SIZE のスタックバッファを使う)
inline ParseStatus encode_wire_frame(uint8_t type, uint32_t seq,
                                     const uint8_t* payload, size_t payload_len,
                                     uint8_t* out, size_t out_cap, size_t* out_len) {
  uint8_t logical[MAX_FRAME_SIZE];
  size_t logical_len = 0;
  const ParseStatus st = pack_frame(type, seq, payload, payload_len,
                                    logical, sizeof(logical), &logical_len);
  if (st != ParseStatus::ok) return st;
  if (out == nullptr || out_len == nullptr || out_cap == 0) return ParseStatus::overflow;
  size_t enc_len = 0;
  if (!cobs_encode(logical, logical_len, out, out_cap - 1, &enc_len)) {
    return ParseStatus::overflow;
  }
  out[enc_len] = COBS_DELIMITER;
  *out_len = enc_len + 1;
  return ParseStatus::ok;
}

inline ParseStatus encode_wire_frame(MsgType type, uint32_t seq,
                                     const uint8_t* payload, size_t payload_len,
                                     uint8_t* out, size_t out_cap, size_t* out_len) {
  return encode_wire_frame(static_cast<uint8_t>(type), seq, payload, payload_len,
                           out, out_cap, out_len);
}

// ---------------------------------------------------------------------------
// ペイロード (de)serializer
//
// すべてフィールド単位の明示的LE読み書き。serialize は cap、deserialize は
// len を厳格検査し、不一致は false(バッファ外アクセスは決して行わない)。
// ---------------------------------------------------------------------------

// 0x12 CMD_SETPOINT(17B)— 姿勢+高度+ヨー目標。ハートビートを兼ねる(50Hz)。
// v2: yaw_ref(±π、機体ヨー角目標)と flags bit1(yaw_ref 有効)を追加。
struct CmdSetpoint {
  static constexpr MsgType TYPE = MsgType::CMD_SETPOINT;
  static constexpr size_t PAYLOAD_SIZE = 4 + 4 + 4 + 4 + 1;
  static constexpr uint8_t FLAG_ALT_REF_VALID = 0x01;  // bit0: alt_ref 有効
  static constexpr uint8_t FLAG_YAW_REF_VALID = 0x02;  // bit1: yaw_ref 有効(=ヨー角制御ON)

  float roll_ref = 0.0f;   // rad
  float pitch_ref = 0.0f;  // rad
  float alt_ref = 0.0f;    // m
  float yaw_ref = 0.0f;    // rad(±π、機体ヨー角目標)
  uint8_t flags = 0;
};
static_assert(CmdSetpoint::PAYLOAD_SIZE == 17, "PROTOCOL.md: CMD_SETPOINT payload = 17B");

inline bool serialize(const CmdSetpoint& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdSetpoint::PAYLOAD_SIZE) return false;
  wr_f32(out + 0, m.roll_ref);
  wr_f32(out + 4, m.pitch_ref);
  wr_f32(out + 8, m.alt_ref);
  wr_f32(out + 12, m.yaw_ref);
  wr_u8(out + 16, m.flags);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdSetpoint* out) {
  if (in == nullptr || out == nullptr || len != CmdSetpoint::PAYLOAD_SIZE) return false;
  out->roll_ref = rd_f32(in + 0);
  out->pitch_ref = rd_f32(in + 4);
  out->alt_ref = rd_f32(in + 8);
  out->yaw_ref = rd_f32(in + 12);
  out->flags = rd_u8(in + 16);
  return true;
}

// 0x24 CMD_POS_ERR(21B)— XY 位置誤差+高度+ヨー目標(機上XY制御モード)。
// CMD_SETPOINT の代替ストリーム(50Hz、ハートビート兼用)。PC は roll/pitch
// 角度指令の代わりに制御座標系の位置誤差(目標 − フィルタ済み現在位置)を送り、
// 機体側が自身のヨー推定で誤差を機体座標系へ回転(ヨー回転補償)してから
// XY PID を回す。alt_ref / yaw_ref / flags bit0-1 の意味は CMD_SETPOINT と同一。
// mocap_yaw は外部ヨー基準(制御座標系ヨー。ソースは PC 側設定: MoCap 実測 or
// 移動ベースヨー。名称は互換のため維持)。ファームはソース非依存に消費し、
// EKF2(est_mode=2)のヨー擬似観測・フレーム整合検証・ログに用いる(bit3 有効時のみ)。
// bit4 は基準ヨー低信頼 → R_ψ 低信頼プリセット切替(MAG_AUTOTUNE_DESIGN.md §1.2)。
struct CmdPosErr {
  static constexpr MsgType TYPE = MsgType::CMD_POS_ERR;
  static constexpr size_t PAYLOAD_SIZE = 4 + 4 + 4 + 4 + 4 + 1;
  static constexpr uint8_t FLAG_ALT_REF_VALID = 0x01;      // bit0(CMD_SETPOINT と同義)
  static constexpr uint8_t FLAG_YAW_REF_VALID = 0x02;      // bit1(同上)
  static constexpr uint8_t FLAG_XY_ERR_VALID = 0x04;       // bit2: err_x/err_y 有効
  static constexpr uint8_t FLAG_MOCAP_YAW_VALID = 0x08;    // bit3: mocap_yaw(外部ヨー基準)有効
  static constexpr uint8_t FLAG_YAW_REF_LOW_TRUST = 0x10;  // bit4: 基準ヨー低信頼

  float err_x = 0.0f;      // m(制御座標系。target - filtered)
  float err_y = 0.0f;      // m
  float alt_ref = 0.0f;    // m
  float yaw_ref = 0.0f;    // rad(±π、機体ヨー角目標)
  float mocap_yaw = 0.0f;  // rad(±π、外部ヨー基準。bit3 有効時のみ)
  uint8_t flags = 0;
};
static_assert(CmdPosErr::PAYLOAD_SIZE == 21, "PROTOCOL.md: CMD_POS_ERR payload = 21B");

inline bool serialize(const CmdPosErr& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdPosErr::PAYLOAD_SIZE) return false;
  wr_f32(out + 0, m.err_x);
  wr_f32(out + 4, m.err_y);
  wr_f32(out + 8, m.alt_ref);
  wr_f32(out + 12, m.yaw_ref);
  wr_f32(out + 16, m.mocap_yaw);
  wr_u8(out + 20, m.flags);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdPosErr* out) {
  if (in == nullptr || out == nullptr || len != CmdPosErr::PAYLOAD_SIZE) return false;
  out->err_x = rd_f32(in + 0);
  out->err_y = rd_f32(in + 4);
  out->alt_ref = rd_f32(in + 8);
  out->yaw_ref = rd_f32(in + 12);
  out->mocap_yaw = rd_f32(in + 16);
  out->flags = rd_u8(in + 20);
  return true;
}

// 0x14 CMD_MODE(1B)— WAIT->MOTOR_TEST(mode=1)/ MOTOR_TEST->WAIT(mode=0)。
// 他状態では TLM_ACK status=bad_state。
struct CmdMode {
  static constexpr MsgType TYPE = MsgType::CMD_MODE;
  static constexpr size_t PAYLOAD_SIZE = 1;
  static constexpr uint8_t MODE_FLIGHT = 0;
  static constexpr uint8_t MODE_MOTOR_TEST = 1;

  uint8_t mode = 0;
};
static_assert(CmdMode::PAYLOAD_SIZE == 1, "PROTOCOL.md: CMD_MODE payload = 1B");

inline bool serialize(const CmdMode& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdMode::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.mode);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdMode* out) {
  if (in == nullptr || out == nullptr || len != CmdMode::PAYLOAD_SIZE) return false;
  out->mode = rd_u8(in + 0);
  return true;
}

// 0x15 CMD_MOTOR_RUN(5B)— MOTOR_TEST 状態のみ。PC は 0.4s 周期で再送
// (キープアライブ)。機体は 1.5s 途絶で自動停止。ソフトスタート 2.0duty/s。
struct CmdMotorRun {
  static constexpr MsgType TYPE = MsgType::CMD_MOTOR_RUN;
  static constexpr size_t PAYLOAD_SIZE = 4 + 1;
  static constexpr uint8_t MASK_FL = 0x01;  // bit0
  static constexpr uint8_t MASK_FR = 0x02;  // bit1
  static constexpr uint8_t MASK_RL = 0x04;  // bit2
  static constexpr uint8_t MASK_RR = 0x08;  // bit3

  float duty = 0.0f;  // 0–1
  uint8_t mask = 0;   // 駆動対象モーター
};
static_assert(CmdMotorRun::PAYLOAD_SIZE == 5, "PROTOCOL.md: CMD_MOTOR_RUN payload = 5B");

inline bool serialize(const CmdMotorRun& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdMotorRun::PAYLOAD_SIZE) return false;
  wr_f32(out + 0, m.duty);
  wr_u8(out + 4, m.mask);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdMotorRun* out) {
  if (in == nullptr || out == nullptr || len != CmdMotorRun::PAYLOAD_SIZE) return false;
  out->duty = rd_f32(in + 0);
  out->mask = rd_u8(in + 4);
  return true;
}

// 0x18 CMD_MAG3D_SET(49B)— 3D磁気キャリブ設定。valid=0 でクリア。
// 適用時: NVS 永続化+FF 自動無効(ff_mode=0)+アンカー破棄+ヨー推定器再シード。
struct CmdMag3dSet {
  static constexpr MsgType TYPE = MsgType::CMD_MAG3D_SET;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4 * 3 + 4 * 9;

  uint8_t valid = 0;
  float offset[3] = {0.0f, 0.0f, 0.0f};  // µT
  float matrix[9] = {0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f,
                     0.0f, 0.0f, 0.0f};  // 行優先
};
static_assert(CmdMag3dSet::PAYLOAD_SIZE == 49, "PROTOCOL.md: CMD_MAG3D_SET payload = 49B");

inline bool serialize(const CmdMag3dSet& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdMag3dSet::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.valid);
  for (size_t i = 0; i < 3; ++i) wr_f32(out + 1 + 4 * i, m.offset[i]);
  for (size_t i = 0; i < 9; ++i) wr_f32(out + 13 + 4 * i, m.matrix[i]);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdMag3dSet* out) {
  if (in == nullptr || out == nullptr || len != CmdMag3dSet::PAYLOAD_SIZE) return false;
  out->valid = rd_u8(in + 0);
  for (size_t i = 0; i < 3; ++i) out->offset[i] = rd_f32(in + 1 + 4 * i);
  for (size_t i = 0; i < 9; ++i) out->matrix[i] = rd_f32(in + 13 + 4 * i);
  return true;
}

// 0x19 CMD_ACCEL6_SET(25B)— 加速度6面キャリブ設定。適用時に姿勢参照リセット。
struct CmdAccel6Set {
  static constexpr MsgType TYPE = MsgType::CMD_ACCEL6_SET;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4 * 3 + 4 * 3;

  uint8_t valid = 0;
  float offset[3] = {0.0f, 0.0f, 0.0f};  // g
  float scale[3] = {0.0f, 0.0f, 0.0f};
};
static_assert(CmdAccel6Set::PAYLOAD_SIZE == 25, "PROTOCOL.md: CMD_ACCEL6_SET payload = 25B");

inline bool serialize(const CmdAccel6Set& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdAccel6Set::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.valid);
  for (size_t i = 0; i < 3; ++i) wr_f32(out + 1 + 4 * i, m.offset[i]);
  for (size_t i = 0; i < 3; ++i) wr_f32(out + 13 + 4 * i, m.scale[i]);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdAccel6Set* out) {
  if (in == nullptr || out == nullptr || len != CmdAccel6Set::PAYLOAD_SIZE) return false;
  out->valid = rd_u8(in + 0);
  for (size_t i = 0; i < 3; ++i) out->offset[i] = rd_f32(in + 1 + 4 * i);
  for (size_t i = 0; i < 3; ++i) out->scale[i] = rd_f32(in + 13 + 4 * i);
  return true;
}

// 0x1A CMD_ATTMOUNT_SET(9B)— 姿勢マウントオフセット設定。
struct CmdAttmountSet {
  static constexpr MsgType TYPE = MsgType::CMD_ATTMOUNT_SET;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4 + 4;

  uint8_t valid = 0;
  float roll_rad = 0.0f;
  float pitch_rad = 0.0f;
};
static_assert(CmdAttmountSet::PAYLOAD_SIZE == 9, "PROTOCOL.md: CMD_ATTMOUNT_SET payload = 9B");

inline bool serialize(const CmdAttmountSet& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdAttmountSet::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.valid);
  wr_f32(out + 1, m.roll_rad);
  wr_f32(out + 5, m.pitch_rad);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdAttmountSet* out) {
  if (in == nullptr || out == nullptr || len != CmdAttmountSet::PAYLOAD_SIZE) return false;
  out->valid = rd_u8(in + 0);
  out->roll_rad = rd_f32(in + 1);
  out->pitch_rad = rd_f32(in + 5);
  return true;
}

// 0x1B CMD_YAWZERO_SET(5B)— ヨーゼロオフセット設定。valid=0 でクリア。
struct CmdYawzeroSet {
  static constexpr MsgType TYPE = MsgType::CMD_YAWZERO_SET;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4;

  uint8_t valid = 0;
  float offset_rad = 0.0f;
};
static_assert(CmdYawzeroSet::PAYLOAD_SIZE == 5, "PROTOCOL.md: CMD_YAWZERO_SET payload = 5B");

inline bool serialize(const CmdYawzeroSet& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdYawzeroSet::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.valid);
  wr_f32(out + 1, m.offset_rad);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdYawzeroSet* out) {
  if (in == nullptr || out == nullptr || len != CmdYawzeroSet::PAYLOAD_SIZE) return false;
  out->valid = rd_u8(in + 0);
  out->offset_rad = rd_f32(in + 1);
  return true;
}

// 0x1C CMD_GEOMAG_SET(20B)— 地磁気プロファイル設定(NVS 永続化)。
struct CmdGeomagSet {
  static constexpr MsgType TYPE = MsgType::CMD_GEOMAG_SET;
  static constexpr size_t PAYLOAD_SIZE = 4 * 5;

  float declination_east_deg = 0.0f;  // 偏角(東向き正)[deg]
  float inclination_deg = 0.0f;       // 伏角 [deg]
  float horizontal_ut = 0.0f;         // 水平分力 [µT]
  float vertical_ut = 0.0f;           // 鉛直分力 [µT]
  float total_ut = 0.0f;              // 全磁力 [µT]
};
static_assert(CmdGeomagSet::PAYLOAD_SIZE == 20, "PROTOCOL.md: CMD_GEOMAG_SET payload = 20B");

inline bool serialize(const CmdGeomagSet& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdGeomagSet::PAYLOAD_SIZE) return false;
  wr_f32(out + 0, m.declination_east_deg);
  wr_f32(out + 4, m.inclination_deg);
  wr_f32(out + 8, m.horizontal_ut);
  wr_f32(out + 12, m.vertical_ut);
  wr_f32(out + 16, m.total_ut);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdGeomagSet* out) {
  if (in == nullptr || out == nullptr || len != CmdGeomagSet::PAYLOAD_SIZE) return false;
  out->declination_east_deg = rd_f32(in + 0);
  out->inclination_deg = rd_f32(in + 4);
  out->horizontal_ut = rd_f32(in + 8);
  out->vertical_ut = rd_f32(in + 12);
  out->total_ut = rd_f32(in + 16);
  return true;
}

// 0x1D CMD_FF_BEGIN(1B)— FF 係数ステージング開始(nlut は 4–24)。
struct CmdFfBegin {
  static constexpr MsgType TYPE = MsgType::CMD_FF_BEGIN;
  static constexpr size_t PAYLOAD_SIZE = 1;
  static constexpr uint8_t NLUT_MIN = 4;
  static constexpr uint8_t NLUT_MAX = 24;

  uint8_t nlut = 0;
};
static_assert(CmdFfBegin::PAYLOAD_SIZE == 1, "PROTOCOL.md: CMD_FF_BEGIN payload = 1B");

inline bool serialize(const CmdFfBegin& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdFfBegin::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.nlut);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdFfBegin* out) {
  if (in == nullptr || out == nullptr || len != CmdFfBegin::PAYLOAD_SIZE) return false;
  out->nlut = rd_u8(in + 0);
  return true;
}

// 0x1E CMD_FF_LUT(17B)— FF LUT 点(電流 → 磁気補正ベクトル)。
struct CmdFfLut {
  static constexpr MsgType TYPE = MsgType::CMD_FF_LUT;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4 * 4;

  uint8_t idx = 0;     // LUT インデックス(0 <= idx < nlut)
  float i_a = 0.0f;    // 電流 [A]
  float db_x = 0.0f;   // 磁気補正 x [µT]
  float db_y = 0.0f;   // 同 y
  float db_z = 0.0f;   // 同 z
};
static_assert(CmdFfLut::PAYLOAD_SIZE == 17, "PROTOCOL.md: CMD_FF_LUT payload = 17B");

inline bool serialize(const CmdFfLut& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdFfLut::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.idx);
  wr_f32(out + 1, m.i_a);
  wr_f32(out + 5, m.db_x);
  wr_f32(out + 9, m.db_y);
  wr_f32(out + 13, m.db_z);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdFfLut* out) {
  if (in == nullptr || out == nullptr || len != CmdFfLut::PAYLOAD_SIZE) return false;
  out->idx = rd_u8(in + 0);
  out->i_a = rd_f32(in + 1);
  out->db_x = rd_f32(in + 5);
  out->db_y = rd_f32(in + 9);
  out->db_z = rd_f32(in + 13);
  return true;
}

// 0x1F CMD_FF_MOT(25B)— FF モーター係数(idx: 0=FL, 1=FR, 2=RL, 3=RR)。
struct CmdFfMot {
  static constexpr MsgType TYPE = MsgType::CMD_FF_MOT;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4 * 3 + 4 * 3;
  static constexpr uint8_t MOTOR_FL = 0;
  static constexpr uint8_t MOTOR_FR = 1;
  static constexpr uint8_t MOTOR_RL = 2;
  static constexpr uint8_t MOTOR_RR = 3;

  uint8_t idx = 0;
  float a_tilde[3] = {0.0f, 0.0f, 0.0f};  // 単位差分磁気ベクトル
  float c2 = 0.0f;  // duty->電流 2次係数
  float c1 = 0.0f;
  float c0 = 0.0f;
};
static_assert(CmdFfMot::PAYLOAD_SIZE == 25, "PROTOCOL.md: CMD_FF_MOT payload = 25B");

inline bool serialize(const CmdFfMot& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdFfMot::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.idx);
  for (size_t i = 0; i < 3; ++i) wr_f32(out + 1 + 4 * i, m.a_tilde[i]);
  wr_f32(out + 13, m.c2);
  wr_f32(out + 17, m.c1);
  wr_f32(out + 21, m.c0);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdFfMot* out) {
  if (in == nullptr || out == nullptr || len != CmdFfMot::PAYLOAD_SIZE) return false;
  out->idx = rd_u8(in + 0);
  for (size_t i = 0; i < 3; ++i) out->a_tilde[i] = rd_f32(in + 1 + 4 * i);
  out->c2 = rd_f32(in + 13);
  out->c1 = rd_f32(in + 17);
  out->c0 = rd_f32(in + 21);
  return true;
}

// 0x20 CMD_FF_AUX(4B)— ベンチ参考アイドル電流。
struct CmdFfAux {
  static constexpr MsgType TYPE = MsgType::CMD_FF_AUX;
  static constexpr size_t PAYLOAD_SIZE = 4;

  float iid_a = 0.0f;  // [A]
};
static_assert(CmdFfAux::PAYLOAD_SIZE == 4, "PROTOCOL.md: CMD_FF_AUX payload = 4B");

inline bool serialize(const CmdFfAux& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdFfAux::PAYLOAD_SIZE) return false;
  wr_f32(out + 0, m.iid_a);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdFfAux* out) {
  if (in == nullptr || out == nullptr || len != CmdFfAux::PAYLOAD_SIZE) return false;
  out->iid_a = rd_f32(in + 0);
  return true;
}

// 0x21 CMD_FF_COMMIT(4B)— ステージング済み係数の CRC-32(IEEE, zlib 互換,
// float32 LE 連結)照合 → NVS 永続化。冪等。
struct CmdFfCommit {
  static constexpr MsgType TYPE = MsgType::CMD_FF_COMMIT;
  static constexpr size_t PAYLOAD_SIZE = 4;

  uint32_t crc32 = 0;
};
static_assert(CmdFfCommit::PAYLOAD_SIZE == 4, "PROTOCOL.md: CMD_FF_COMMIT payload = 4B");

inline bool serialize(const CmdFfCommit& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdFfCommit::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.crc32);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdFfCommit* out) {
  if (in == nullptr || out == nullptr || len != CmdFfCommit::PAYLOAD_SIZE) return false;
  out->crc32 = rd_u32(in + 0);
  return true;
}

// 0x22 CMD_FF_MODE(2B)— ff_mode / est_mode の実行時切替(NVS 永続化)。
// est_mode の受理範囲は 0..2(2=EKF2、MAG_AUTOTUNE_DESIGN.md §1.3)。
struct CmdFfMode {
  static constexpr MsgType TYPE = MsgType::CMD_FF_MODE;
  static constexpr size_t PAYLOAD_SIZE = 1 + 1;
  static constexpr uint8_t FF_MODE_OFF = 0;
  static constexpr uint8_t FF_MODE_A = 1;
  static constexpr uint8_t FF_MODE_B = 2;
  static constexpr uint8_t EST_MODE_COMPLEMENTARY = 0;  // 補正相補フィルタ
  static constexpr uint8_t EST_MODE_EKF = 1;
  static constexpr uint8_t EST_MODE_EKF2 = 2;           // ヨー擬似観測付き EKF2

  uint8_t ff_mode = 0;
  uint8_t est_mode = 0;
};
static_assert(CmdFfMode::PAYLOAD_SIZE == 2, "PROTOCOL.md: CMD_FF_MODE payload = 2B");

inline bool serialize(const CmdFfMode& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdFfMode::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.ff_mode);
  wr_u8(out + 1, m.est_mode);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdFfMode* out) {
  if (in == nullptr || out == nullptr || len != CmdFfMode::PAYLOAD_SIZE) return false;
  out->ff_mode = rd_u8(in + 0);
  out->est_mode = rd_u8(in + 1);
  return true;
}

// 0x25 CMD_LED_MODE(1B)— LED 表示モード切替(計測中インジケータ)。
// mode=1(RECORDING)で MOTOR_TEST 中のみマゼンタ常灯(動画カット点検出用)。
// フェイルセーフ: 最後の mode=1 受信から 3 秒で AUTO へ自動復帰(PC は約 1 秒
// 間隔で再送)。MOTOR_TEST 離脱でも即 AUTO。受理状態はキャリブ系と同じ。
struct CmdLedMode {
  static constexpr MsgType TYPE = MsgType::CMD_LED_MODE;
  static constexpr size_t PAYLOAD_SIZE = 1;
  static constexpr uint8_t MODE_AUTO = 0;       // 通常の状態表示
  static constexpr uint8_t MODE_RECORDING = 1;  // 計測中(マゼンタ常灯)

  uint8_t mode = 0;
};
static_assert(CmdLedMode::PAYLOAD_SIZE == 1, "PROTOCOL.md: CMD_LED_MODE payload = 1B");

inline bool serialize(const CmdLedMode& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdLedMode::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.mode);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdLedMode* out) {
  if (in == nullptr || out == nullptr || len != CmdLedMode::PAYLOAD_SIZE) return false;
  out->mode = rd_u8(in + 0);
  return true;
}

// 0x26 CMD_MAGBIAS_SET(13B)— 学習ハードアイアン残差 Δb の適用/クリア。
// Δb [µT, b_cal 空間] を NVS `magbias` へ保存し即適用(mode=0 でクリア)。
// ff_mode は降格しない(FF 係数は b_cal 空間のまま有効)。適用時: アンカー
// 無効化+窓リセット+補正系再シード(mag3d 変更時と同パターン、ff_mode 降格
// だけしない)。WAIT / COMPLETE / MOTOR_TEST のみ受理(MAG_AUTOTUNE_DESIGN.md §1.4)。
struct CmdMagbiasSet {
  static constexpr MsgType TYPE = MsgType::CMD_MAGBIAS_SET;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4 * 3;
  static constexpr uint8_t MODE_CLEAR = 0;
  static constexpr uint8_t MODE_SET = 1;

  uint8_t mode = 0;  // 0=clear, 1=set
  float dx = 0.0f;   // µT(b_cal 空間の Δb x)
  float dy = 0.0f;   // µT(同 y)
  float dz = 0.0f;   // µT(同 z)
};
static_assert(CmdMagbiasSet::PAYLOAD_SIZE == 13, "PROTOCOL.md: CMD_MAGBIAS_SET payload = 13B");

inline bool serialize(const CmdMagbiasSet& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdMagbiasSet::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.mode);
  wr_f32(out + 1, m.dx);
  wr_f32(out + 5, m.dy);
  wr_f32(out + 9, m.dz);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdMagbiasSet* out) {
  if (in == nullptr || out == nullptr || len != CmdMagbiasSet::PAYLOAD_SIZE) return false;
  out->mode = rd_u8(in + 0);
  out->dx = rd_f32(in + 1);
  out->dy = rd_f32(in + 5);
  out->dz = rd_f32(in + 9);
  return true;
}

// 0x27 CMD_FLOWCAL_SET(9B)— フロースケール [counts/rad] の適用/クリア。
// NVS `flowcal` へ保存(mode=0 でクリア=既定 450.0 に戻る)。
// WAIT / COMPLETE / MOTOR_TEST のみ受理(MAG_AUTOTUNE_DESIGN.md §1.4)。
struct CmdFlowcalSet {
  static constexpr MsgType TYPE = MsgType::CMD_FLOWCAL_SET;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4 + 4;
  static constexpr uint8_t MODE_CLEAR = 0;
  static constexpr uint8_t MODE_SET = 1;

  uint8_t mode = 0;  // 0=clear, 1=set
  float kx = 0.0f;   // counts/rad
  float ky = 0.0f;   // counts/rad
};
static_assert(CmdFlowcalSet::PAYLOAD_SIZE == 9, "PROTOCOL.md: CMD_FLOWCAL_SET payload = 9B");

inline bool serialize(const CmdFlowcalSet& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdFlowcalSet::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.mode);
  wr_f32(out + 1, m.kx);
  wr_f32(out + 5, m.ky);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdFlowcalSet* out) {
  if (in == nullptr || out == nullptr || len != CmdFlowcalSet::PAYLOAD_SIZE) return false;
  out->mode = rd_u8(in + 0);
  out->kx = rd_f32(in + 1);
  out->ky = rd_f32(in + 5);
  return true;
}

// 0x28 CMD_FLOW_PROBE(1B)— PMW3901 burst プローブ実行。
// モーター停止時のみ(回転中は status=busy で拒否)。結果は LOG_TEXT 複数行
// (agree_ratio 等の要約)+ probe 成功で burst_ok ラッチ更新。n_cycles=0 は
// 既定 200 サイクル。WAIT / COMPLETE / MOTOR_TEST のみ受理。
struct CmdFlowProbe {
  static constexpr MsgType TYPE = MsgType::CMD_FLOW_PROBE;
  static constexpr size_t PAYLOAD_SIZE = 1;
  static constexpr uint8_t DEFAULT_CYCLES = 200;

  uint8_t n_cycles = 0;  // プローブサイクル数(0=既定 200)
};
static_assert(CmdFlowProbe::PAYLOAD_SIZE == 1, "PROTOCOL.md: CMD_FLOW_PROBE payload = 1B");

inline bool serialize(const CmdFlowProbe& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < CmdFlowProbe::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.n_cycles);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, CmdFlowProbe* out) {
  if (in == nullptr || out == nullptr || len != CmdFlowProbe::PAYLOAD_SIZE) return false;
  out->n_cycles = rd_u8(in + 0);
  return true;
}

// 0x30 TLM_STATE(184B)— フル状態テレメトリ(25Hz、400Hzループの16分周)。
// オフセットは PROTOCOL.md の表と1対1対応(宣言順に隙間なくパック)。
// v2: 末尾追加のみ(既存オフセット 0–96 は v1 と不変。serial_link.py が
// seq_echo を先頭オフセット直読みするため末尾追加限定)。
// 磁気オートチューン/フロー拡張(MAG_AUTOTUNE_DESIGN.md §1.1): オフセット
// 135 以降を末尾追加して 135B → 184B(旧 135B フレームは受理しない)。
struct TlmState {
  static constexpr MsgType TYPE = MsgType::TLM_STATE;
  static constexpr size_t PAYLOAD_SIZE =
      4 + 4 + 1 + 1 + 1 +  // seq_echo, elapsed_ms, state, flags, reason
      4 * 3 +              // roll, pitch, yaw
      4 * 3 +              // p, q, r
      4 * 2 +              // roll_ref, pitch_ref
      4 +                  // alt_ref
      4 * 2 +              // altitude_tof, altitude_est
      4 +                  // alt_velocity
      4 +                  // z_dot_ref
      4 +                  // voltage
      4 * 4 +              // duty_fr, duty_fl, duty_rr, duty_rl
      4 * 3 +              // ax, ay, az
      2 +                  // loop_dt_us
      4 * 3 +              // yaw_est_rad, yaw_gyro_int_rad, yaw_ref_rad(v2)
      4 +                  // current_a(v2)
      4 * 2 +              // db_hat_x_ut, db_hat_y_ut(v2)
      4 * 2 +              // bm_x_ut, bm_y_ut(v2)
      4 +                  // nis(v2)
      1 + 1 +              // ffg, ff_status(v2)
      4 * 5 +              // mag_cal_x/y/z_ut, mag_lev_x/y_ut(磁気オートチューン拡張)
      4 * 4 +              // ekf2_yaw_rad, ekf2_bm_x/y_ut, ekf2_yaw_innov_rad(同)
      1 + 1 +              // ekf2_status, ekf2_gate(同)
      4 * 2 +              // flow_vx_mps, flow_vy_mps(同)
      1 + 1 + 1;           // flow_squal, flow_status, flow_dt_ms(同)
  static constexpr uint8_t FLAG_LOW_VOLTAGE = 0x01;     // bit0
  static constexpr uint8_t FLAG_SETPOINT_FRESH = 0x02;  // bit1 (<200ms)
  static constexpr uint8_t FLAG_FLYING = 0x04;          // bit2
  // ff_status ビット定義(v2)
  static constexpr uint8_t FF_STATUS_FF_MODE_MASK = 0x03;    // bit0-1: ff_mode(0-2)
  static constexpr uint8_t FF_STATUS_EST_EKF = 0x04;         // bit2: est_mode==1(EKF)
  static constexpr uint8_t FF_STATUS_ANCHOR_VALID = 0x08;    // bit3
  static constexpr uint8_t FF_STATUS_FFCAL_LOADED = 0x10;    // bit4
  static constexpr uint8_t FF_STATUS_YAW_CTRL_ACTIVE = 0x20; // bit5
  static constexpr uint8_t FF_STATUS_MAG_FRESH = 0x40;       // bit6
  static constexpr uint8_t FF_STATUS_EST_EKF2 = 0x80;        // bit7: est_mode==2(EKF2。
                                                             // bit2/bit7 とも 0 なら相補CF)
  // ekf2_status ビット定義(MAG_AUTOTUNE_DESIGN.md §1.1)
  static constexpr uint8_t EKF2_STATUS_YAW_OBS_FRESH = 0x01;       // bit0: ヨー観測受信 <1s
  static constexpr uint8_t EKF2_STATUS_YAW_OBS_FUSED = 0x02;       // bit1: 直近 0.5s 内に受理
  static constexpr uint8_t EKF2_STATUS_FLIGHT_ANCHOR_DONE = 0x04;  // bit2
  static constexpr uint8_t EKF2_STATUS_TAU_RW_MODE = 0x08;         // bit3: q_bm RWモード
  static constexpr uint8_t EKF2_STATUS_BM_FROZEN = 0x10;           // bit4
  static constexpr uint8_t EKF2_STATUS_HEALTHY2 = 0x20;            // bit5
  static constexpr uint8_t EKF2_STATUS_YAW_OBS_LOW_TRUST = 0x40;   // bit6(bit7 予約)
  // flow_status ビット定義(MAG_AUTOTUNE_DESIGN.md §1.1)
  static constexpr uint8_t FLOW_STATUS_SENSOR_OK = 0x01;    // bit0
  static constexpr uint8_t FLOW_STATUS_BURST_OK = 0x02;     // bit1
  static constexpr uint8_t FLOW_STATUS_VEL_VALID = 0x04;    // bit2
  static constexpr uint8_t FLOW_STATUS_RANGE_VALID = 0x08;  // bit3
  static constexpr uint8_t FLOW_STATUS_SQUAL_OK = 0x10;     // bit4
  static constexpr uint8_t FLOW_STATUS_INIT_RETRY = 0x20;   // bit5(bit6-7 予約)

  uint32_t seq_echo = 0;      // 最後に適用した CMD_SETPOINT / CMD_POS_ERR の seq(未受信なら0)
  uint32_t elapsed_ms = 0;    // 起動からの経過 [ms]
  uint8_t state = 0;          // FlightState
  uint8_t flags = 0;
  uint8_t reason = 0;         // Reason(直近の遷移理由)
  float roll = 0.0f;          // rad(実測姿勢, AHRS)
  float pitch = 0.0f;         // rad
  float yaw = 0.0f;           // rad
  float p = 0.0f;             // rad/s(実測角速度)
  float q = 0.0f;             // rad/s
  float r = 0.0f;             // rad/s
  float roll_ref = 0.0f;      // rad(適用中の指令)
  float pitch_ref = 0.0f;     // rad
  float alt_ref = 0.0f;       // m(適用中の目標高度)
  float altitude_tof = 0.0f;  // m(ToF生値)
  float altitude_est = 0.0f;  // m(カルマン推定)
  float alt_velocity = 0.0f;  // m/s
  float z_dot_ref = 0.0f;     // m/s
  float voltage = 0.0f;       // V
  float duty_fr = 0.0f;       // 0–1
  float duty_fl = 0.0f;
  float duty_rr = 0.0f;
  float duty_rl = 0.0f;
  float ax = 0.0f;            // g(フィルタ後加速度)
  float ay = 0.0f;
  float az = 0.0f;
  uint16_t loop_dt_us = 0;    // µs(直近の実測制御周期)
  // --- v2 追加(オフセット 97 以降) ---
  float yaw_est_rad = 0.0f;       // rad(アクティブ推定器ヨー。est_mode=1 なら EKF ψ)
  float yaw_gyro_int_rad = 0.0f;  // rad(Z軸角速度の単純積算 400Hz、ahrs_reset でゼロ)
  float yaw_ref_rad = 0.0f;       // rad(適用中ヨー目標。ラッチ後含む。制御 off 時 0)
  float current_a = 0.0f;         // A(総電流、20Hz 更新)
  float db_hat_x_ut = 0.0f;       // µT(FF 補正ベクトル x)
  float db_hat_y_ut = 0.0f;       // µT(同 y)
  float bm_x_ut = 0.0f;           // µT(EKF 磁気バイアス状態 x)
  float bm_y_ut = 0.0f;           // µT(同 y)
  float nis = 0.0f;               // 直近 EKF 更新の NIS
  uint8_t ffg = 0;                // EKF ゲート/健全性ビット(yaw側 ffg 定義踏襲)
  uint8_t ff_status = 0;          // FF_STATUS_* ビット
  // --- 磁気オートチューン/フロー拡張(オフセット 135 以降。契約 §1.1) ---
  float mag_cal_x_ut = 0.0f;      // µT(b_cal = mag3d 適用後・FF 補正前、機体系 x)
  float mag_cal_y_ut = 0.0f;      // µT(同 y)
  float mag_cal_z_ut = 0.0f;      // µT(同 z)
  float mag_lev_x_ut = 0.0f;      // µT(FF補正+magbias+EMA後レベル化水平 x = EKF観測 zx)
  float mag_lev_y_ut = 0.0f;      // µT(同 y = zy)
  float ekf2_yaw_rad = 0.0f;      // rad(EKF2 の ψ。シャドー中も常時)
  float ekf2_bm_x_ut = 0.0f;      // µT(EKF2 の b_mx)
  float ekf2_bm_y_ut = 0.0f;      // µT(同 b_my)
  float ekf2_yaw_innov_rad = 0.0f;  // rad(直近ヨー観測イノベーション。未受信時 0)
  uint8_t ekf2_status = 0;        // EKF2_STATUS_* ビット
  uint8_t ekf2_gate = 0;          // EKF2 のゲートビット(ffg と同一ビット定義)
  float flow_vx_mps = 0.0f;       // m/s(フロー機体系 x 速度。無効時 0)
  float flow_vy_mps = 0.0f;       // m/s(同 y)
  uint8_t flow_squal = 0;         // SQUAL 生値
  uint8_t flow_status = 0;        // FLOW_STATUS_* ビット
  uint8_t flow_dt_ms = 0;         // ms(フロー読み実測 dt。0-255 クランプ)
};
static_assert(TlmState::PAYLOAD_SIZE == 184, "PROTOCOL.md: TLM_STATE payload = 184B");

inline bool serialize(const TlmState& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < TlmState::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.seq_echo);
  wr_u32(out + 4, m.elapsed_ms);
  wr_u8(out + 8, m.state);
  wr_u8(out + 9, m.flags);
  wr_u8(out + 10, m.reason);
  wr_f32(out + 11, m.roll);
  wr_f32(out + 15, m.pitch);
  wr_f32(out + 19, m.yaw);
  wr_f32(out + 23, m.p);
  wr_f32(out + 27, m.q);
  wr_f32(out + 31, m.r);
  wr_f32(out + 35, m.roll_ref);
  wr_f32(out + 39, m.pitch_ref);
  wr_f32(out + 43, m.alt_ref);
  wr_f32(out + 47, m.altitude_tof);
  wr_f32(out + 51, m.altitude_est);
  wr_f32(out + 55, m.alt_velocity);
  wr_f32(out + 59, m.z_dot_ref);
  wr_f32(out + 63, m.voltage);
  wr_f32(out + 67, m.duty_fr);
  wr_f32(out + 71, m.duty_fl);
  wr_f32(out + 75, m.duty_rr);
  wr_f32(out + 79, m.duty_rl);
  wr_f32(out + 83, m.ax);
  wr_f32(out + 87, m.ay);
  wr_f32(out + 91, m.az);
  wr_u16(out + 95, m.loop_dt_us);
  wr_f32(out + 97, m.yaw_est_rad);
  wr_f32(out + 101, m.yaw_gyro_int_rad);
  wr_f32(out + 105, m.yaw_ref_rad);
  wr_f32(out + 109, m.current_a);
  wr_f32(out + 113, m.db_hat_x_ut);
  wr_f32(out + 117, m.db_hat_y_ut);
  wr_f32(out + 121, m.bm_x_ut);
  wr_f32(out + 125, m.bm_y_ut);
  wr_f32(out + 129, m.nis);
  wr_u8(out + 133, m.ffg);
  wr_u8(out + 134, m.ff_status);
  wr_f32(out + 135, m.mag_cal_x_ut);
  wr_f32(out + 139, m.mag_cal_y_ut);
  wr_f32(out + 143, m.mag_cal_z_ut);
  wr_f32(out + 147, m.mag_lev_x_ut);
  wr_f32(out + 151, m.mag_lev_y_ut);
  wr_f32(out + 155, m.ekf2_yaw_rad);
  wr_f32(out + 159, m.ekf2_bm_x_ut);
  wr_f32(out + 163, m.ekf2_bm_y_ut);
  wr_f32(out + 167, m.ekf2_yaw_innov_rad);
  wr_u8(out + 171, m.ekf2_status);
  wr_u8(out + 172, m.ekf2_gate);
  wr_f32(out + 173, m.flow_vx_mps);
  wr_f32(out + 177, m.flow_vy_mps);
  wr_u8(out + 181, m.flow_squal);
  wr_u8(out + 182, m.flow_status);
  wr_u8(out + 183, m.flow_dt_ms);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, TlmState* out) {
  if (in == nullptr || out == nullptr || len != TlmState::PAYLOAD_SIZE) return false;
  out->seq_echo = rd_u32(in + 0);
  out->elapsed_ms = rd_u32(in + 4);
  out->state = rd_u8(in + 8);
  out->flags = rd_u8(in + 9);
  out->reason = rd_u8(in + 10);
  out->roll = rd_f32(in + 11);
  out->pitch = rd_f32(in + 15);
  out->yaw = rd_f32(in + 19);
  out->p = rd_f32(in + 23);
  out->q = rd_f32(in + 27);
  out->r = rd_f32(in + 31);
  out->roll_ref = rd_f32(in + 35);
  out->pitch_ref = rd_f32(in + 39);
  out->alt_ref = rd_f32(in + 43);
  out->altitude_tof = rd_f32(in + 47);
  out->altitude_est = rd_f32(in + 51);
  out->alt_velocity = rd_f32(in + 55);
  out->z_dot_ref = rd_f32(in + 59);
  out->voltage = rd_f32(in + 63);
  out->duty_fr = rd_f32(in + 67);
  out->duty_fl = rd_f32(in + 71);
  out->duty_rr = rd_f32(in + 75);
  out->duty_rl = rd_f32(in + 79);
  out->ax = rd_f32(in + 83);
  out->ay = rd_f32(in + 87);
  out->az = rd_f32(in + 91);
  out->loop_dt_us = rd_u16(in + 95);
  out->yaw_est_rad = rd_f32(in + 97);
  out->yaw_gyro_int_rad = rd_f32(in + 101);
  out->yaw_ref_rad = rd_f32(in + 105);
  out->current_a = rd_f32(in + 109);
  out->db_hat_x_ut = rd_f32(in + 113);
  out->db_hat_y_ut = rd_f32(in + 117);
  out->bm_x_ut = rd_f32(in + 121);
  out->bm_y_ut = rd_f32(in + 125);
  out->nis = rd_f32(in + 129);
  out->ffg = rd_u8(in + 133);
  out->ff_status = rd_u8(in + 134);
  out->mag_cal_x_ut = rd_f32(in + 135);
  out->mag_cal_y_ut = rd_f32(in + 139);
  out->mag_cal_z_ut = rd_f32(in + 143);
  out->mag_lev_x_ut = rd_f32(in + 147);
  out->mag_lev_y_ut = rd_f32(in + 151);
  out->ekf2_yaw_rad = rd_f32(in + 155);
  out->ekf2_bm_x_ut = rd_f32(in + 159);
  out->ekf2_bm_y_ut = rd_f32(in + 163);
  out->ekf2_yaw_innov_rad = rd_f32(in + 167);
  out->ekf2_status = rd_u8(in + 171);
  out->ekf2_gate = rd_u8(in + 172);
  out->flow_vx_mps = rd_f32(in + 173);
  out->flow_vy_mps = rd_f32(in + 177);
  out->flow_squal = rd_u8(in + 181);
  out->flow_status = rd_u8(in + 182);
  out->flow_dt_ms = rd_u8(in + 183);
  return true;
}

// 0x31 TLM_EVENT(8B)— 状態遷移時に即時送信+2Hzで定期再送。
struct TlmEvent {
  static constexpr MsgType TYPE = MsgType::TLM_EVENT;
  static constexpr size_t PAYLOAD_SIZE = 1 + 1 + 1 + 1 + 4;

  uint8_t state = 0;       // FlightState
  uint8_t prev_state = 0;  // FlightState
  uint8_t reason = 0;      // Reason
  uint8_t flags = 0;
  float voltage = 0.0f;    // V
};
static_assert(TlmEvent::PAYLOAD_SIZE == 8, "PROTOCOL.md: TLM_EVENT payload = 8B");

inline bool serialize(const TlmEvent& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < TlmEvent::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.state);
  wr_u8(out + 1, m.prev_state);
  wr_u8(out + 2, m.reason);
  wr_u8(out + 3, m.flags);
  wr_f32(out + 4, m.voltage);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, TlmEvent* out) {
  if (in == nullptr || out == nullptr || len != TlmEvent::PAYLOAD_SIZE) return false;
  out->state = rd_u8(in + 0);
  out->prev_state = rd_u8(in + 1);
  out->reason = rd_u8(in + 2);
  out->flags = rd_u8(in + 3);
  out->voltage = rd_f32(in + 4);
  return true;
}

// 0x32 TLM_ACK(6B)— 0x14–0x23, 0x25–0x28 コマンドへの応答。
struct TlmAck {
  static constexpr MsgType TYPE = MsgType::TLM_ACK;
  static constexpr size_t PAYLOAD_SIZE = 1 + 4 + 1;
  static constexpr uint8_t STATUS_OK = 0;
  static constexpr uint8_t STATUS_BAD_STATE = 1;
  static constexpr uint8_t STATUS_INVALID_ARG = 2;
  static constexpr uint8_t STATUS_CRC_MISMATCH = 3;
  static constexpr uint8_t STATUS_BUSY = 4;
  static constexpr uint8_t STATUS_INCOMPLETE = 5;

  uint8_t acked_type = 0;  // 応答対象のメッセージ型
  uint32_t acked_seq = 0;  // 応答対象フレームの seq
  uint8_t status = 0;      // STATUS_*
};
static_assert(TlmAck::PAYLOAD_SIZE == 6, "PROTOCOL.md: TLM_ACK payload = 6B");

inline bool serialize(const TlmAck& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < TlmAck::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.acked_type);
  wr_u32(out + 1, m.acked_seq);
  wr_u8(out + 5, m.status);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, TlmAck* out) {
  if (in == nullptr || out == nullptr || len != TlmAck::PAYLOAD_SIZE) return false;
  out->acked_type = rd_u8(in + 0);
  out->acked_seq = rd_u32(in + 1);
  out->status = rd_u8(in + 5);
  return true;
}

// 0x33 TLM_EXP(86B)— 実験テレメトリ。MOTOR_TEST 状態でのみ 25Hz 送出
// (TLM_STATE と 8tick 位相をずらす)。隙間なくパック。
struct TlmExp {
  static constexpr MsgType TYPE = MsgType::TLM_EXP;
  static constexpr size_t PAYLOAD_SIZE =
      4 +          // elapsed_ms
      4 * 3 +      // current_a, vbat_v, shunt_uv
      4 * 3 +      // bx_raw, by_raw, bz_raw
      4 * 3 +      // bx_cal, by_cal, bz_cal
      4 +          // imu_temp_c
      4 * 3 +      // roll, pitch, yaw
      4 * 3 +      // p, q, r
      4 * 3 +      // ax, ay, az
      4 +          // duty_cmd
      1 + 1;       // motors_mask, flags
  static constexpr uint8_t FLAG_CURRENT_VALID = 0x01;   // bit0
  static constexpr uint8_t FLAG_MAG_FRESH = 0x02;       // bit1
  static constexpr uint8_t FLAG_MOTORS_RUNNING = 0x04;  // bit2

  uint32_t elapsed_ms = 0;   // 起動からの経過 [ms]
  float current_a = 0.0f;    // A(INA3221 CH2 総電流)
  float vbat_v = 0.0f;       // V
  float shunt_uv = 0.0f;     // µV
  float bx_raw = 0.0f;       // µT(RHALL補償+軸変換後・mag3D 前)
  float by_raw = 0.0f;
  float bz_raw = 0.0f;
  float bx_cal = 0.0f;       // µT(mag3D 後)
  float by_cal = 0.0f;
  float bz_cal = 0.0f;
  float imu_temp_c = 0.0f;   // ℃
  float roll = 0.0f;         // rad(Madgwick)
  float pitch = 0.0f;
  float yaw = 0.0f;
  float p = 0.0f;            // rad/s
  float q = 0.0f;
  float r = 0.0f;
  float ax = 0.0f;           // g(フィルタ後)
  float ay = 0.0f;
  float az = 0.0f;
  float duty_cmd = 0.0f;     // モーターテスト指令 duty(0–1)
  uint8_t motors_mask = 0;   // CmdMotorRun::MASK_* と同ビット割り
  uint8_t flags = 0;         // FLAG_*
};
static_assert(TlmExp::PAYLOAD_SIZE == 86, "PROTOCOL.md: TLM_EXP payload = 86B");

inline bool serialize(const TlmExp& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < TlmExp::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.elapsed_ms);
  wr_f32(out + 4, m.current_a);
  wr_f32(out + 8, m.vbat_v);
  wr_f32(out + 12, m.shunt_uv);
  wr_f32(out + 16, m.bx_raw);
  wr_f32(out + 20, m.by_raw);
  wr_f32(out + 24, m.bz_raw);
  wr_f32(out + 28, m.bx_cal);
  wr_f32(out + 32, m.by_cal);
  wr_f32(out + 36, m.bz_cal);
  wr_f32(out + 40, m.imu_temp_c);
  wr_f32(out + 44, m.roll);
  wr_f32(out + 48, m.pitch);
  wr_f32(out + 52, m.yaw);
  wr_f32(out + 56, m.p);
  wr_f32(out + 60, m.q);
  wr_f32(out + 64, m.r);
  wr_f32(out + 68, m.ax);
  wr_f32(out + 72, m.ay);
  wr_f32(out + 76, m.az);
  wr_f32(out + 80, m.duty_cmd);
  wr_u8(out + 84, m.motors_mask);
  wr_u8(out + 85, m.flags);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, TlmExp* out) {
  if (in == nullptr || out == nullptr || len != TlmExp::PAYLOAD_SIZE) return false;
  out->elapsed_ms = rd_u32(in + 0);
  out->current_a = rd_f32(in + 4);
  out->vbat_v = rd_f32(in + 8);
  out->shunt_uv = rd_f32(in + 12);
  out->bx_raw = rd_f32(in + 16);
  out->by_raw = rd_f32(in + 20);
  out->bz_raw = rd_f32(in + 24);
  out->bx_cal = rd_f32(in + 28);
  out->by_cal = rd_f32(in + 32);
  out->bz_cal = rd_f32(in + 36);
  out->imu_temp_c = rd_f32(in + 40);
  out->roll = rd_f32(in + 44);
  out->pitch = rd_f32(in + 48);
  out->yaw = rd_f32(in + 52);
  out->p = rd_f32(in + 56);
  out->q = rd_f32(in + 60);
  out->r = rd_f32(in + 64);
  out->ax = rd_f32(in + 68);
  out->ay = rd_f32(in + 72);
  out->az = rd_f32(in + 76);
  out->duty_cmd = rd_f32(in + 80);
  out->motors_mask = rd_u8(in + 84);
  out->flags = rd_u8(in + 85);
  return true;
}

// 0x34 TLM_CAL_DATA(132B)— CMD_CAL_GET への応答(キャリブ一括データ)。
// 磁気オートチューン/フロー拡張(MAG_AUTOTUNE_DESIGN.md §1.5): magbias と
// flowcal を末尾追加して 112B → 132B。
struct TlmCalData {
  static constexpr MsgType TYPE = MsgType::TLM_CAL_DATA;
  static constexpr size_t PAYLOAD_SIZE =
      1 +          // valid_flags
      4 * 3 +      // mag3d_offset
      4 * 9 +      // mag3d_matrix
      4 * 3 +      // accel6_offset
      4 * 3 +      // accel6_scale
      4 + 4 +      // attmount_roll_rad, attmount_pitch_rad
      4 +          // yawzero_offset_rad
      4 * 5 +      // geomag(decl_east_deg, incl_deg, H_uT, V_uT, F_uT)
      1 +          // ff_nlut
      4 +          // ff_crc32
      1 + 1 +      // ff_mode, est_mode
      4 * 3 +      // magbias(磁気オートチューン拡張)
      4 * 2;       // flowcal(同)
  static constexpr uint8_t VALID_MAG3D = 0x01;    // bit0
  static constexpr uint8_t VALID_ACCEL6 = 0x02;   // bit1
  static constexpr uint8_t VALID_ATTMOUNT = 0x04; // bit2
  static constexpr uint8_t VALID_YAWZERO = 0x08;  // bit3
  static constexpr uint8_t VALID_GEOMAG = 0x10;   // bit4
  static constexpr uint8_t VALID_FFCAL = 0x20;    // bit5
  static constexpr uint8_t VALID_MAGBIAS = 0x40;  // bit6: magbias 有効(契約 §1.5)
  static constexpr uint8_t VALID_FLOWCAL = 0x80;  // bit7: flowcal 有効(同上)

  uint8_t valid_flags = 0;
  float mag3d_offset[3] = {0.0f, 0.0f, 0.0f};
  float mag3d_matrix[9] = {0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f,
                           0.0f, 0.0f, 0.0f};  // 行優先
  float accel6_offset[3] = {0.0f, 0.0f, 0.0f};
  float accel6_scale[3] = {0.0f, 0.0f, 0.0f};
  float attmount_roll_rad = 0.0f;
  float attmount_pitch_rad = 0.0f;
  float yawzero_offset_rad = 0.0f;
  float geomag[5] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};  // decl_east_deg, incl_deg, H, V, F
  uint8_t ff_nlut = 0;
  uint32_t ff_crc32 = 0;
  uint8_t ff_mode = 0;
  uint8_t est_mode = 0;
  float magbias[3] = {0.0f, 0.0f, 0.0f};  // Δb dx, dy, dz [µT, b_cal 空間]
  float flowcal[2] = {0.0f, 0.0f};        // kx, ky [counts/rad]
};
static_assert(TlmCalData::PAYLOAD_SIZE == 132, "PROTOCOL.md: TLM_CAL_DATA payload = 132B");

inline bool serialize(const TlmCalData& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < TlmCalData::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.valid_flags);
  for (size_t i = 0; i < 3; ++i) wr_f32(out + 1 + 4 * i, m.mag3d_offset[i]);
  for (size_t i = 0; i < 9; ++i) wr_f32(out + 13 + 4 * i, m.mag3d_matrix[i]);
  for (size_t i = 0; i < 3; ++i) wr_f32(out + 49 + 4 * i, m.accel6_offset[i]);
  for (size_t i = 0; i < 3; ++i) wr_f32(out + 61 + 4 * i, m.accel6_scale[i]);
  wr_f32(out + 73, m.attmount_roll_rad);
  wr_f32(out + 77, m.attmount_pitch_rad);
  wr_f32(out + 81, m.yawzero_offset_rad);
  for (size_t i = 0; i < 5; ++i) wr_f32(out + 85 + 4 * i, m.geomag[i]);
  wr_u8(out + 105, m.ff_nlut);
  wr_u32(out + 106, m.ff_crc32);
  wr_u8(out + 110, m.ff_mode);
  wr_u8(out + 111, m.est_mode);
  for (size_t i = 0; i < 3; ++i) wr_f32(out + 112 + 4 * i, m.magbias[i]);
  for (size_t i = 0; i < 2; ++i) wr_f32(out + 124 + 4 * i, m.flowcal[i]);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, TlmCalData* out) {
  if (in == nullptr || out == nullptr || len != TlmCalData::PAYLOAD_SIZE) return false;
  out->valid_flags = rd_u8(in + 0);
  for (size_t i = 0; i < 3; ++i) out->mag3d_offset[i] = rd_f32(in + 1 + 4 * i);
  for (size_t i = 0; i < 9; ++i) out->mag3d_matrix[i] = rd_f32(in + 13 + 4 * i);
  for (size_t i = 0; i < 3; ++i) out->accel6_offset[i] = rd_f32(in + 49 + 4 * i);
  for (size_t i = 0; i < 3; ++i) out->accel6_scale[i] = rd_f32(in + 61 + 4 * i);
  out->attmount_roll_rad = rd_f32(in + 73);
  out->attmount_pitch_rad = rd_f32(in + 77);
  out->yawzero_offset_rad = rd_f32(in + 81);
  for (size_t i = 0; i < 5; ++i) out->geomag[i] = rd_f32(in + 85 + 4 * i);
  out->ff_nlut = rd_u8(in + 105);
  out->ff_crc32 = rd_u32(in + 106);
  out->ff_mode = rd_u8(in + 110);
  out->est_mode = rd_u8(in + 111);
  for (size_t i = 0; i < 3; ++i) out->magbias[i] = rd_f32(in + 112 + 4 * i);
  for (size_t i = 0; i < 2; ++i) out->flowcal[i] = rd_f32(in + 124 + 4 * i);
  return true;
}

// 0x35 TLM_CTRL(89B)— 制御ループ診断テレメトリ。25Hz(400Hzループの16分周、
// TLM_STATE と 4tick 位相をずらす)で全飛行状態(MOTOR_TEST 中も含む)常時送出。
// PID 成分は PID::update() の合成式 m_kp*(err + m_integral + m_differential) の
// 3分解(P=kp*err、I=kp*m_integral、D=kp*m_differential。P+I+D=そのPIDの出力)。
// yaw の角度ループ成分はクランプ前の値を記録する(yaw_rate_ref はクランプ後
// → 差でクランプ発動が分かる)。PID リセット中(非飛行時や、ヨー制御OFF時の
// psi_pid 毎tickリセット等)は成分が 0 になるため、flags で有効区間を判別する。
struct TlmCtrl {
  static constexpr MsgType TYPE = MsgType::TLM_CTRL;
  static constexpr size_t PAYLOAD_SIZE =
      4 +      // elapsed_ms
      4 * 3 +  // roll_rate_ref, pitch_rate_ref, yaw_rate_ref
      4 * 9 +  // pid_ang[9]
      4 * 9 +  // pid_rate[9]
      1;       // flags
  static constexpr uint8_t FLAG_XY_ONBOARD_ACTIVE = 0x01;  // bit0: CMD_POS_ERR 経路で機上XY指令生成中
  static constexpr uint8_t FLAG_YAW_CTRL_ACTIVE = 0x02;    // bit1
  static constexpr uint8_t FLAG_FLYING = 0x04;             // bit2

  uint32_t elapsed_ms = 0;      // 起動からの経過 [ms](TLM_STATE と同一クロック)
  float roll_rate_ref = 0.0f;   // rad/s(角度ループ出力=ロール指令角速度)
  float pitch_rate_ref = 0.0f;  // rad/s(同ピッチ)
  float yaw_rate_ref = 0.0f;    // rad/s(psi_pid 出力。±yaw_rate_limit_rad_s クランプ後)
  float pid_ang[9] = {0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f,
                      0.0f, 0.0f, 0.0f};   // 角度ループPID成分(roll_p,i,d, pitch_p,i,d, yaw_p,i,d)
  float pid_rate[9] = {0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 0.0f};  // 角速度ループPID成分(同順。roll=p_pid, pitch=q_pid, yaw=r_pid)
  uint8_t flags = 0;            // FLAG_*
};
static_assert(TlmCtrl::PAYLOAD_SIZE == 89, "PROTOCOL.md: TLM_CTRL payload = 89B");

inline bool serialize(const TlmCtrl& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < TlmCtrl::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.elapsed_ms);
  wr_f32(out + 4, m.roll_rate_ref);
  wr_f32(out + 8, m.pitch_rate_ref);
  wr_f32(out + 12, m.yaw_rate_ref);
  for (size_t i = 0; i < 9; ++i) wr_f32(out + 16 + 4 * i, m.pid_ang[i]);
  for (size_t i = 0; i < 9; ++i) wr_f32(out + 52 + 4 * i, m.pid_rate[i]);
  wr_u8(out + 88, m.flags);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, TlmCtrl* out) {
  if (in == nullptr || out == nullptr || len != TlmCtrl::PAYLOAD_SIZE) return false;
  out->elapsed_ms = rd_u32(in + 0);
  out->roll_rate_ref = rd_f32(in + 4);
  out->pitch_rate_ref = rd_f32(in + 8);
  out->yaw_rate_ref = rd_f32(in + 12);
  for (size_t i = 0; i < 9; ++i) out->pid_ang[i] = rd_f32(in + 16 + 4 * i);
  for (size_t i = 0; i < 9; ++i) out->pid_rate[i] = rd_f32(in + 52 + 4 * i);
  out->flags = rd_u8(in + 88);
  return true;
}

// 0x40 LOG_TEXT(1〜181B)— 人間向けテキスト。データUARTへの生テキスト直書きの代替。
// text はビュー(deserialize 時は入力バッファ内を指す。コピーしない)。
struct LogText {
  static constexpr MsgType TYPE = MsgType::LOG_TEXT;
  static constexpr uint8_t ORIGIN_RELAY = 0;
  static constexpr uint8_t ORIGIN_DRONE = 1;

  uint8_t origin = 0;            // 0=relay, 1=drone
  const uint8_t* text = nullptr; // UTF-8(NUL終端は不要・含めない)
  size_t text_len = 0;           // <= MAX_LOG_TEXT_SIZE
};

// 可変長のため out_len 付き。text_len > 180 は拒否する。
inline bool serialize(const LogText& m, uint8_t* out, size_t cap, size_t* out_len) {
  if (out == nullptr || out_len == nullptr) return false;
  if (m.text_len > MAX_LOG_TEXT_SIZE) return false;
  if (m.text == nullptr && m.text_len > 0) return false;
  const size_t total = 1 + m.text_len;
  if (cap < total) return false;
  wr_u8(out + 0, m.origin);
  if (m.text_len > 0) std::memcpy(out + 1, m.text, m.text_len);
  *out_len = total;
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, LogText* out) {
  if (in == nullptr || out == nullptr) return false;
  if (len < 1 || len > 1 + MAX_LOG_TEXT_SIZE) return false;
  out->origin = rd_u8(in + 0);
  out->text = in + 1;
  out->text_len = len - 1;
  return true;
}

// UTF-8 文字境界を保ったまま text を max_len バイト以下に切り詰めた長さを返す
// (PROTOCOL.md LOG_TEXT: 多バイト文字を分断しない)。
// 切り詰め位置が多バイト文字の途中になる場合は、その文字の先頭(リード)バイトまで
// 遡って文字ごと落とす。末尾がすでに不正な UTF-8 の場合は新たな分断を作らないこと
// のみ保証し、入力をそのまま通す(受信側 PC は U+FFFD 置換で受理する)。
inline size_t utf8_truncate_len(const uint8_t* text, size_t len, size_t max_len) {
  if (text == nullptr) return 0;
  const size_t cut = (len <= max_len) ? len : max_len;
  if (cut == 0) return 0;
  // 末尾文字のリードバイトを探す(0b10xxxxxx = 継続バイトを遡る)
  size_t lead = cut - 1;
  while (lead > 0 && (text[lead] & 0xC0) == 0x80) --lead;
  const uint8_t b = text[lead];
  size_t expect;  // リードバイトが宣言するシーケンス長
  if (b < 0x80) {
    expect = 1;                            // ASCII
  } else if ((b & 0xE0) == 0xC0) {
    expect = 2;                            // 110xxxxx
  } else if ((b & 0xF0) == 0xE0) {
    expect = 3;                            // 1110xxxx
  } else if ((b & 0xF8) == 0xF0) {
    expect = 4;                            // 11110xxx
  } else {
    return cut;  // 孤立継続バイト等の不正列: 分断回避の対象外
  }
  return (lead + expect <= cut) ? cut : lead;
}

// 0x50 RLY_SET_TARGET(7B)— ESP-NOW ピア設定。
struct RlySetTarget {
  static constexpr MsgType TYPE = MsgType::RLY_SET_TARGET;
  static constexpr size_t PAYLOAD_SIZE = 6 + 1;

  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  uint8_t wifi_channel = 0;  // 1-13
};
static_assert(RlySetTarget::PAYLOAD_SIZE == 7, "PROTOCOL.md: RLY_SET_TARGET payload = 7B");

inline bool serialize(const RlySetTarget& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlySetTarget::PAYLOAD_SIZE) return false;
  for (size_t i = 0; i < 6; ++i) wr_u8(out + i, m.mac[i]);  // MACは配列順(エンディアン無関係)
  wr_u8(out + 6, m.wifi_channel);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlySetTarget* out) {
  if (in == nullptr || out == nullptr || len != RlySetTarget::PAYLOAD_SIZE) return false;
  for (size_t i = 0; i < 6; ++i) out->mac[i] = rd_u8(in + i);
  out->wifi_channel = rd_u8(in + 6);
  return true;
}

// 0x51 RLY_TARGET_ACK(8B)— SET_TARGET への応答。
struct RlyTargetAck {
  static constexpr MsgType TYPE = MsgType::RLY_TARGET_ACK;
  static constexpr size_t PAYLOAD_SIZE = 1 + 6 + 1;
  static constexpr uint8_t STATUS_OK = 0;
  static constexpr uint8_t STATUS_INVALID_MAC = 1;
  static constexpr uint8_t STATUS_PEER_FAILED = 2;

  uint8_t status = 0;
  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  uint8_t channel = 0;
};
static_assert(RlyTargetAck::PAYLOAD_SIZE == 8, "PROTOCOL.md: RLY_TARGET_ACK payload = 8B");

inline bool serialize(const RlyTargetAck& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlyTargetAck::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.status);
  for (size_t i = 0; i < 6; ++i) wr_u8(out + 1 + i, m.mac[i]);
  wr_u8(out + 7, m.channel);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlyTargetAck* out) {
  if (in == nullptr || out == nullptr || len != RlyTargetAck::PAYLOAD_SIZE) return false;
  out->status = rd_u8(in + 0);
  for (size_t i = 0; i < 6; ++i) out->mac[i] = rd_u8(in + 1 + i);
  out->channel = rd_u8(in + 7);
  return true;
}

// 0x52 RLY_STATS(24B)— リレー統計(1Hz自動送信)。
struct RlyStats {
  static constexpr MsgType TYPE = MsgType::RLY_STATS;
  static constexpr size_t PAYLOAD_SIZE = 4 * 6;

  uint32_t up_frames = 0;
  uint32_t down_frames = 0;
  uint32_t crc_errors = 0;
  uint32_t cobs_errors = 0;
  uint32_t espnow_send_fail = 0;
  uint32_t overflow_drops = 0;
};
static_assert(RlyStats::PAYLOAD_SIZE == 24, "PROTOCOL.md: RLY_STATS payload = 24B");

inline bool serialize(const RlyStats& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlyStats::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.up_frames);
  wr_u32(out + 4, m.down_frames);
  wr_u32(out + 8, m.crc_errors);
  wr_u32(out + 12, m.cobs_errors);
  wr_u32(out + 16, m.espnow_send_fail);
  wr_u32(out + 20, m.overflow_drops);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlyStats* out) {
  if (in == nullptr || out == nullptr || len != RlyStats::PAYLOAD_SIZE) return false;
  out->up_frames = rd_u32(in + 0);
  out->down_frames = rd_u32(in + 4);
  out->crc_errors = rd_u32(in + 8);
  out->cobs_errors = rd_u32(in + 12);
  out->espnow_send_fail = rd_u32(in + 16);
  out->overflow_drops = rd_u32(in + 20);
  return true;
}

// 0x54 RLY_PONG(4B)— RLY_PING 応答(PING の seq をエコー)。
struct RlyPong {
  static constexpr MsgType TYPE = MsgType::RLY_PONG;
  static constexpr size_t PAYLOAD_SIZE = 4;

  uint32_t echo_seq = 0;
};
static_assert(RlyPong::PAYLOAD_SIZE == 4, "PROTOCOL.md: RLY_PONG payload = 4B");

inline bool serialize(const RlyPong& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlyPong::PAYLOAD_SIZE) return false;
  wr_u32(out + 0, m.echo_seq);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlyPong* out) {
  if (in == nullptr || out == nullptr || len != RlyPong::PAYLOAD_SIZE) return false;
  out->echo_seq = rd_u32(in + 0);
  return true;
}

// 0x55 RLY_SET_PEERS(可変長 2+7×N B、N=0..4)— マルチ機体ピア設定。
// count=0 でマルチモード解除。全ピアで wifi_channel を共有する(無線は1チャネル)。
struct RlyPeerEntry {
  uint8_t mac[6] = {0, 0, 0, 0, 0, 0};
  uint8_t tlm_state_div = 1;  // TLM_STATE 間引き(1=全転送, n=1/n 転送。0 は 1 扱い)
};

struct RlySetPeers {
  static constexpr MsgType TYPE = MsgType::RLY_SET_PEERS;
  static constexpr size_t MIN_PAYLOAD_SIZE = 2;
  static constexpr size_t MAX_PAYLOAD_SIZE_BYTES =
      MIN_PAYLOAD_SIZE + RLY_PEER_ENTRY_SIZE * RLY_MAX_PEERS;  // = 30

  uint8_t count = 0;         // 0..RLY_MAX_PEERS(index がそのまま node_id)
  uint8_t wifi_channel = 0;  // 1-13(count=0 のときは 0 を許容)
  RlyPeerEntry peers[RLY_MAX_PEERS] = {};
};
static_assert(RlySetPeers::MAX_PAYLOAD_SIZE_BYTES == 30,
              "PROTOCOL.md: RLY_SET_PEERS payload = 2+7*N (max 30B)");

inline bool serialize(const RlySetPeers& m, uint8_t* out, size_t cap, size_t* out_len) {
  if (out == nullptr || out_len == nullptr) return false;
  if (m.count > RLY_MAX_PEERS) return false;
  const size_t need = RlySetPeers::MIN_PAYLOAD_SIZE + RLY_PEER_ENTRY_SIZE * m.count;
  if (cap < need) return false;
  wr_u8(out + 0, m.count);
  wr_u8(out + 1, m.wifi_channel);
  for (size_t i = 0; i < m.count; ++i) {
    const size_t off = RlySetPeers::MIN_PAYLOAD_SIZE + RLY_PEER_ENTRY_SIZE * i;
    for (size_t j = 0; j < 6; ++j) wr_u8(out + off + j, m.peers[i].mac[j]);
    wr_u8(out + off + 6, m.peers[i].tlm_state_div);
  }
  *out_len = need;
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlySetPeers* out) {
  if (in == nullptr || out == nullptr || len < RlySetPeers::MIN_PAYLOAD_SIZE) return false;
  const uint8_t count = rd_u8(in + 0);
  if (count > RLY_MAX_PEERS) return false;
  if (len != RlySetPeers::MIN_PAYLOAD_SIZE + RLY_PEER_ENTRY_SIZE * count) return false;
  out->count = count;
  out->wifi_channel = rd_u8(in + 1);
  for (size_t i = 0; i < count; ++i) {
    const size_t off = RlySetPeers::MIN_PAYLOAD_SIZE + RLY_PEER_ENTRY_SIZE * i;
    for (size_t j = 0; j < 6; ++j) out->peers[i].mac[j] = rd_u8(in + off + j);
    out->peers[i].tlm_state_div = rd_u8(in + off + 6);
  }
  for (size_t i = count; i < RLY_MAX_PEERS; ++i) out->peers[i] = RlyPeerEntry{};
  return true;
}

// 0x56 RLY_PEERS_ACK(4B)— SET_PEERS への応答。
struct RlyPeersAck {
  static constexpr MsgType TYPE = MsgType::RLY_PEERS_ACK;
  static constexpr size_t PAYLOAD_SIZE = 4;
  static constexpr uint8_t STATUS_OK = 0;
  static constexpr uint8_t STATUS_INVALID_MAC = 1;
  static constexpr uint8_t STATUS_PEER_FAILED = 2;
  static constexpr uint8_t STATUS_BAD_COUNT = 3;
  static constexpr uint8_t STATUS_BAD_CHANNEL = 4;
  static constexpr uint8_t FAILED_NONE = 0xFF;

  uint8_t status = 0;
  uint8_t count = 0;          // 受理したピア数のエコー
  uint8_t wifi_channel = 0;
  uint8_t failed_index = FAILED_NONE;  // 失敗エントリの index(なければ 0xFF)
};
static_assert(RlyPeersAck::PAYLOAD_SIZE == 4, "PROTOCOL.md: RLY_PEERS_ACK payload = 4B");

inline bool serialize(const RlyPeersAck& m, uint8_t* out, size_t cap) {
  if (out == nullptr || cap < RlyPeersAck::PAYLOAD_SIZE) return false;
  wr_u8(out + 0, m.status);
  wr_u8(out + 1, m.count);
  wr_u8(out + 2, m.wifi_channel);
  wr_u8(out + 3, m.failed_index);
  return true;
}

inline bool deserialize(const uint8_t* in, size_t len, RlyPeersAck* out) {
  if (in == nullptr || out == nullptr || len != RlyPeersAck::PAYLOAD_SIZE) return false;
  out->status = rd_u8(in + 0);
  out->count = rd_u8(in + 1);
  out->wifi_channel = rd_u8(in + 2);
  out->failed_index = rd_u8(in + 3);
  return true;
}

// 0x57/0x58 RLY_MUX_UP / RLY_MUX_DOWN(1+内側フレーム長)— シリアル区間の
// 機体多重化エンベロープ。inner は完全な内側論理フレーム(ver..crc16)への
// ゼロコピー参照(元ペイロードバッファが生きている間のみ有効)。
struct RlyMuxView {
  uint8_t node_id = 0;
  const uint8_t* inner = nullptr;
  size_t inner_len = 0;
};

// MUX ペイロードを (node_id, 内側フレーム参照) に分解する。
// 内側フレームの CRC/構造検証は行わない(受信側が parse_frame で検証)。
inline bool mux_unwrap(const uint8_t* payload, size_t len, RlyMuxView* out) {
  if (payload == nullptr || out == nullptr) return false;
  if (len < MUX_HEADER_SIZE + FRAME_OVERHEAD) return false;
  const uint8_t node_id = rd_u8(payload);
  if (node_id >= RLY_MAX_PEERS) return false;
  out->node_id = node_id;
  out->inner = payload + MUX_HEADER_SIZE;
  out->inner_len = len - MUX_HEADER_SIZE;
  return true;
}

// MUX ペイロード(node_id + 内側フレーム)を構築する。戻り値は書き込んだ
// バイト数(失敗時 0)。
inline size_t mux_wrap(uint8_t node_id, const uint8_t* inner, size_t inner_len,
                       uint8_t* out, size_t cap) {
  if (inner == nullptr || out == nullptr) return 0;
  if (node_id >= RLY_MAX_PEERS) return 0;
  if (inner_len < FRAME_OVERHEAD) return 0;
  if (MUX_HEADER_SIZE + inner_len > MAX_PAYLOAD_SIZE) return 0;
  if (cap < MUX_HEADER_SIZE + inner_len) return 0;
  wr_u8(out, node_id);
  for (size_t i = 0; i < inner_len; ++i) out[MUX_HEADER_SIZE + i] = inner[i];
  return MUX_HEADER_SIZE + inner_len;
}

// 既知型の期待ペイロード長。固定長は値、可変長(LOG_TEXT)は -1、未知型は -2。
// ドローン側受理規則「len==期待値」の判定に使用する。
inline int expected_payload_size(uint8_t type) {
  switch (static_cast<MsgType>(type)) {
    case MsgType::CMD_START:
    case MsgType::CMD_STOP:
    case MsgType::CMD_RESET:
    case MsgType::CMD_MOTOR_STOP:
    case MsgType::CMD_CAL_GET:
    case MsgType::CMD_FF_ANCHOR:
    case MsgType::RLY_PING:
      return 0;
    case MsgType::CMD_SETPOINT:
      return static_cast<int>(CmdSetpoint::PAYLOAD_SIZE);
    case MsgType::CMD_POS_ERR:
      return static_cast<int>(CmdPosErr::PAYLOAD_SIZE);
    case MsgType::CMD_MODE:
      return static_cast<int>(CmdMode::PAYLOAD_SIZE);
    case MsgType::CMD_MOTOR_RUN:
      return static_cast<int>(CmdMotorRun::PAYLOAD_SIZE);
    case MsgType::CMD_MAG3D_SET:
      return static_cast<int>(CmdMag3dSet::PAYLOAD_SIZE);
    case MsgType::CMD_ACCEL6_SET:
      return static_cast<int>(CmdAccel6Set::PAYLOAD_SIZE);
    case MsgType::CMD_ATTMOUNT_SET:
      return static_cast<int>(CmdAttmountSet::PAYLOAD_SIZE);
    case MsgType::CMD_YAWZERO_SET:
      return static_cast<int>(CmdYawzeroSet::PAYLOAD_SIZE);
    case MsgType::CMD_GEOMAG_SET:
      return static_cast<int>(CmdGeomagSet::PAYLOAD_SIZE);
    case MsgType::CMD_FF_BEGIN:
      return static_cast<int>(CmdFfBegin::PAYLOAD_SIZE);
    case MsgType::CMD_FF_LUT:
      return static_cast<int>(CmdFfLut::PAYLOAD_SIZE);
    case MsgType::CMD_FF_MOT:
      return static_cast<int>(CmdFfMot::PAYLOAD_SIZE);
    case MsgType::CMD_FF_AUX:
      return static_cast<int>(CmdFfAux::PAYLOAD_SIZE);
    case MsgType::CMD_FF_COMMIT:
      return static_cast<int>(CmdFfCommit::PAYLOAD_SIZE);
    case MsgType::CMD_FF_MODE:
      return static_cast<int>(CmdFfMode::PAYLOAD_SIZE);
    case MsgType::CMD_LED_MODE:
      return static_cast<int>(CmdLedMode::PAYLOAD_SIZE);
    case MsgType::CMD_MAGBIAS_SET:
      return static_cast<int>(CmdMagbiasSet::PAYLOAD_SIZE);
    case MsgType::CMD_FLOWCAL_SET:
      return static_cast<int>(CmdFlowcalSet::PAYLOAD_SIZE);
    case MsgType::CMD_FLOW_PROBE:
      return static_cast<int>(CmdFlowProbe::PAYLOAD_SIZE);
    case MsgType::TLM_STATE:
      return static_cast<int>(TlmState::PAYLOAD_SIZE);
    case MsgType::TLM_EVENT:
      return static_cast<int>(TlmEvent::PAYLOAD_SIZE);
    case MsgType::TLM_ACK:
      return static_cast<int>(TlmAck::PAYLOAD_SIZE);
    case MsgType::TLM_EXP:
      return static_cast<int>(TlmExp::PAYLOAD_SIZE);
    case MsgType::TLM_CAL_DATA:
      return static_cast<int>(TlmCalData::PAYLOAD_SIZE);
    case MsgType::TLM_CTRL:
      return static_cast<int>(TlmCtrl::PAYLOAD_SIZE);
    case MsgType::LOG_TEXT:
      return -1;
    case MsgType::RLY_SET_TARGET:
      return static_cast<int>(RlySetTarget::PAYLOAD_SIZE);
    case MsgType::RLY_TARGET_ACK:
      return static_cast<int>(RlyTargetAck::PAYLOAD_SIZE);
    case MsgType::RLY_STATS:
      return static_cast<int>(RlyStats::PAYLOAD_SIZE);
    case MsgType::RLY_PONG:
      return static_cast<int>(RlyPong::PAYLOAD_SIZE);
    case MsgType::RLY_SET_PEERS:  // 可変長(2+7×N)
    case MsgType::RLY_MUX_UP:     // 可変長(1+内側フレーム)
    case MsgType::RLY_MUX_DOWN:   // 可変長(1+内側フレーム)
      return -1;
    case MsgType::RLY_PEERS_ACK:
      return static_cast<int>(RlyPeersAck::PAYLOAD_SIZE);
    default:
      return -2;
  }
}

// ---------------------------------------------------------------------------
// 逐次シリアルレシーバ
//
// 0x00 区切りのシリアルバイト列から論理フレームを取り出す。リレー(uart_link)
// と PC 側ホストテストの両方で使用する。動的確保・ブロッキング・出力なし。
//
// 回復則(PROTOCOL.md「トランスポート」):
// - COBSデコード失敗 / CRC不一致 / ver不一致 / len不整合はフレームごと破棄
//   (カウンタ加算のみ。部分回復しない)。
// - 蓄積が 256B を超えたら次の 0x00 まで読み捨て(overflow_drops 加算)。
//
// スレッド安全ではない。単一の受信コンテキストから使用すること。
// ---------------------------------------------------------------------------

class SerialFrameReceiver {
 public:
  // 受信統計。フィールド名は Python 側 ReceiverCounters と同一。
  struct Counters {
    uint32_t frames_ok = 0;
    uint32_t cobs_errors = 0;
    uint32_t crc_errors = 0;
    uint32_t ver_errors = 0;
    uint32_t len_errors = 0;
    uint32_t overflow_drops = 0;
  };

  // 1バイト供給(ポーリング型)。完全な有効フレームが復号できたとき true を
  // 返し、frame() が有効になる。frame() の payload ポインタは次に有効フレーム
  // が復号されるまで有効。
  bool feed(uint8_t byte) {
    if (byte == COBS_DELIMITER) {
      if (dropping_) {
        // 読み捨てモード終了(オーバーフロー分のカウントは突入時に済んでいる)
        dropping_ = false;
        rx_len_ = 0;
        return false;
      }
      if (rx_len_ == 0) return false;  // 連続デリミタ / アイドルは無視
      const size_t n = rx_len_;
      rx_len_ = 0;
      size_t decoded_len = 0;
      if (!cobs_decode(rx_, n, decoded_, sizeof(decoded_), &decoded_len)) {
        ++counters_.cobs_errors;
        return false;
      }
      switch (parse_frame(decoded_, decoded_len, &frame_)) {
        case ParseStatus::ok:
          ++counters_.frames_ok;
          return true;
        case ParseStatus::bad_crc:
          ++counters_.crc_errors;
          return false;
        case ParseStatus::bad_ver:
          ++counters_.ver_errors;
          return false;
        default:  // bad_len(parse_frame は overflow を返さない)
          ++counters_.len_errors;
          return false;
      }
    }
    if (dropping_) return false;
    if (rx_len_ >= SERIAL_RX_BUFFER_CAP) {
      // 上限超過 → 次の 0x00 まで読み捨て
      dropping_ = true;
      rx_len_ = 0;
      ++counters_.overflow_drops;
      return false;
    }
    rx_[rx_len_++] = byte;
    return false;
  }

  // バッファ供給+コールバック型。完成フレームごとに on_frame(const FrameView&)
  // を呼ぶ。std::function を使わないテンプレートなのでヒープ確保なし。
  template <typename OnFrame>
  void feed(const uint8_t* data, size_t len, OnFrame&& on_frame) {
    for (size_t i = 0; i < len; ++i) {
      if (feed(data[i])) on_frame(static_cast<const FrameView&>(frame_));
    }
  }

  const FrameView& frame() const { return frame_; }
  const Counters& counters() const { return counters_; }

  // 蓄積バッファと読み捨て状態をクリアする(カウンタは維持)。
  void reset() {
    rx_len_ = 0;
    dropping_ = false;
  }

  void reset_counters() { counters_ = Counters{}; }

 private:
  uint8_t rx_[SERIAL_RX_BUFFER_CAP] = {};
  uint8_t decoded_[SERIAL_RX_BUFFER_CAP] = {};
  size_t rx_len_ = 0;
  bool dropping_ = false;
  FrameView frame_;
  Counters counters_;
};

}  // namespace stampfly
