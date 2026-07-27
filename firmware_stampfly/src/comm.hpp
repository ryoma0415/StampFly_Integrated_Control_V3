// ===========================================================================
// comm.hpp — ESP-NOW 送受信(機体側)
//
// PROTOCOL.md「ドローン側の受理規則」の実装:
// - 受信フレームは len==期待値 / CRC一致 / ver==PROTOCOL_VERSION のもののみ受理
// - ブート後最初の有効上りフレームの送信元MACをリレーピアとして学習(以後不変)
// - 受信コールバック(WiFiタスク)は検証+portMUXメールボックス格納のみ。
//   出力・ブロッキングは行わない。400Hzループが comm_consume_commands() で消費。
// - WiFiチャネルは FLIGHT_CONFIG.wifi_channel に esp_wifi_set_channel でピン留め
// - v2: 上り 0x14-0x23, 0x25-0x28(CMD_MODE〜CMD_FF_ANCHOR, CMD_LED_MODE,
//   CMD_MAGBIAS_SET〜CMD_FLOW_PROBE)はリングバッファに積み、400Hzループ側が
//   型別に deserialize して処理・TLM_ACK 応答する
//   (0x24 は CMD_POS_ERR ストリームで別扱い)
// ===========================================================================
#pragma once

#include <stddef.h>
#include <stdint.h>

#include "stampfly_protocol.hpp"

// v2 コマンド(0x14-0x23, 0x25-0x28)1件分。payload は parse_frame と型別期待長の
// 検証を通過した生バイト列(消費側の flight_control が型別に deserialize する)。
struct V2Command {
    uint8_t type = 0;    // stampfly::MsgType の数値(0x14-0x23, 0x25-0x28)
    uint32_t seq = 0;    // 論理フレーム seq(TLM_ACK の acked_seq に使う)
    uint8_t len = 0;     // payload 長
    uint8_t payload[stampfly::CmdMag3dSet::PAYLOAD_SIZE] = {0};  // 49B = v2 コマンドの最大
};

// v2 コマンドリングの容量。キャリブ/FF系はPC側がACK待ち(1s+リトライ2回)で
// 直列送信するため、実際の同時滞留は CMD_MOTOR_RUN キープアライブとの
// 2〜3件程度。あふれた場合は新着を落とす(PC側リトライで回復する)。
constexpr size_t V2_COMMAND_QUEUE_CAPACITY = 8;

// 400Hzループが1tickごとに取り出すコマンドのスナップショット。
// 優先度 STOP > START > RESET > SETPOINT は消費側(flight_control)が
// stop_pending を最優先で処理することで成立する。
struct CommandSnapshot {
    bool stop_pending = false;
    bool start_pending = false;
    bool reset_pending = false;
    bool setpoint_pending = false;          // 新しい CMD_SETPOINT が届いたか
    stampfly::CmdSetpoint setpoint{};       // 最新の setpoint(pending時のみ有効)
    uint32_t setpoint_seq = 0;              // その論理フレーム seq
    uint32_t setpoint_rx_ms = 0;            // 受信時刻 millis()
    // --- CMD_POS_ERR(機上XY制御モード。setpoint と同じ「最新値上書き」規律) ---
    bool pos_err_pending = false;           // 新しい CMD_POS_ERR が届いたか
    stampfly::CmdPosErr pos_err{};          // 最新の pos_err(pending時のみ有効)
    uint32_t pos_err_seq = 0;               // その論理フレーム seq
    uint32_t pos_err_rx_ms = 0;             // 受信時刻 millis()
    V2Command v2[V2_COMMAND_QUEUE_CAPACITY]{};  // v2 コマンド(受信順)
    size_t v2_count = 0;                    // 取り出した v2 コマンド件数
};

// WiFi(STA)+チャネルピン留め+ESP-NOW初期化+受信コールバック登録。
// setup段階で1回呼ぶ(ブート時の数行のシリアル出力を含む)。
void comm_init(void);

// メールボックスの内容を取り出してクリアする(400Hzループ専用)。
// 何かしらのコマンドが入っていたら true。
bool comm_consume_commands(CommandSnapshot* out);

// リレーピアを学習済みか(=下りフレームを送れる状態か)
bool comm_relay_ready(void);

// 論理フレーム(ver/type/seq/len/payload/crc)を組み立ててESP-NOW送信する。
// seq は送信ごとに単調増加(1始まり)。リレー未学習なら false。
// 400Hzループ(単一送信コンテキスト)からのみ呼ぶこと。
bool comm_send_payload(stampfly::MsgType type, const uint8_t* payload, size_t payload_len);

// LOG_TEXT(origin=drone)を送る。データ経路に生テキストは流さない(PROTOCOL.md)。
bool comm_send_log(const char* text);
