// ===========================================================================
// comm.cpp — ESP-NOW 送受信(機体側)実装
// ===========================================================================
#include "comm.hpp"

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

#include <cstring>

#include "config.hpp"
#include "esp_now_callback_compat.hpp"

namespace {

constexpr size_t MAC_ADDRESS_LENGTH = 6;

// --- コマンドメールボックス(受信コールバックが書き、400Hzループが読む) ---
// portMUXクリティカルセクションで保護する(ARCHITECTURE.md の規定パターン)。
portMUX_TYPE command_mux = portMUX_INITIALIZER_UNLOCKED;

struct CommandMailbox {
    bool stop_pending = false;
    bool start_pending = false;
    bool reset_pending = false;
    bool setpoint_pending = false;
    stampfly::CmdSetpoint setpoint{};
    uint32_t setpoint_seq = 0;
    uint32_t setpoint_rx_ms = 0;
    // CMD_POS_ERR(機上XY制御モード。setpoint と同じ「最新値上書き」規律)
    bool pos_err_pending = false;
    stampfly::CmdPosErr pos_err{};
    uint32_t pos_err_seq = 0;
    uint32_t pos_err_rx_ms = 0;
    // v2 コマンド(0x14-0x23, 0x25-0x28)のリングバッファ。満杯時は新着を落とす
    // (PC側の ACK タイムアウト+リトライで回復する)。
    V2Command v2_ring[V2_COMMAND_QUEUE_CAPACITY]{};
    size_t v2_head = 0;   // 次に取り出す位置
    size_t v2_count = 0;  // 滞留件数
};
CommandMailbox mailbox;

// V2Command::payload は v2 コマンドの最大 payload(CMD_MAG3D_SET 49B)で確保
// している。他の v2 上り型がそれを超えないことをコンパイル時に固定する。
static_assert(stampfly::CmdMode::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdMotorRun::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdAccel6Set::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdAttmountSet::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdYawzeroSet::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdGeomagSet::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdFfBegin::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdFfLut::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdFfMot::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdFfAux::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdFfCommit::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdFfMode::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdLedMode::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdMagbiasSet::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdFlowcalSet::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");
static_assert(stampfly::CmdFlowProbe::PAYLOAD_SIZE <= sizeof(V2Command::payload), "v2 payload buf");

// v2 コマンド型(0x14-0x23, 0x25-0x28。0x24 は CMD_POS_ERR ストリームで別扱い)か。
// 型レンジは stampfly_protocol.hpp の割り当てに一致。
constexpr bool is_v2_command_type(uint8_t type) {
    return (type >= static_cast<uint8_t>(stampfly::MsgType::CMD_MODE) &&
            type <= static_cast<uint8_t>(stampfly::MsgType::CMD_FF_ANCHOR)) ||
           (type >= static_cast<uint8_t>(stampfly::MsgType::CMD_LED_MODE) &&
            type <= static_cast<uint8_t>(stampfly::MsgType::CMD_FLOW_PROBE));
}

// --- リレーピア(最初の有効上りフレームの送信元MACを学習、以後不変) ---
portMUX_TYPE relay_mux = portMUX_INITIALIZER_UNLOCKED;
uint8_t relay_mac[MAC_ADDRESS_LENGTH] = {0};
bool relay_known = false;

// 下り送信用 seq(送信者ごとの単調増加カウンタ、1始まり)。
// 400Hzループ(単一コンテキスト)からのみ更新する。
uint32_t tx_seq = 0;

// リレーピアを学習する(受信コールバック=WiFiタスクコンテキストから呼ばれる。
// esp_now_add_peer はWiFiタスクから呼んで問題ない実績パターン。出力は行わない)。
void learn_relay_peer(const uint8_t* mac) {
    if (mac == nullptr) return;

    bool known;
    portENTER_CRITICAL(&relay_mux);
    known = relay_known;
    portEXIT_CRITICAL(&relay_mux);
    if (known) return;  // 以後不変(PROTOCOL.md)

    esp_now_peer_info_t peer = {};
    std::memcpy(peer.peer_addr, mac, MAC_ADDRESS_LENGTH);
    peer.channel = 0;  // 現在ピン留め中のチャネルを使用
    peer.ifidx = WIFI_IF_STA;
    peer.encrypt = false;
    if (esp_now_is_peer_exist(mac)) {
        esp_now_del_peer(mac);
    }
    if (esp_now_add_peer(&peer) != ESP_OK) return;

    portENTER_CRITICAL(&relay_mux);
    std::memcpy(relay_mac, mac, MAC_ADDRESS_LENGTH);
    relay_known = true;
    portEXIT_CRITICAL(&relay_mux);
}

bool copy_relay_mac(uint8_t* out) {
    bool ok;
    portENTER_CRITICAL(&relay_mux);
    ok = relay_known;
    if (ok) std::memcpy(out, relay_mac, MAC_ADDRESS_LENGTH);
    portEXIT_CRITICAL(&relay_mux);
    return ok;
}

// ESP-NOW受信コールバック(WiFiタスク)。検証+メールボックス格納のみを行う。
void on_esp_now_recv(const EspNowRecvInfo* sender_info, const uint8_t* data, int len) {
    if (data == nullptr || len <= 0) return;

    // 受理規則: 構造・CRC・ver を parse_frame で、型ごとの期待長を別途検証
    stampfly::FrameView frame;
    if (stampfly::parse_frame(data, static_cast<size_t>(len), &frame) != stampfly::ParseStatus::ok) {
        return;
    }
    if (!stampfly::is_uplink_type(frame.type)) return;  // ドローン行き(0x10-0x2F)のみ
    const int expected = stampfly::expected_payload_size(frame.type);
    if (expected < 0 || frame.len != static_cast<uint8_t>(expected)) return;

    learn_relay_peer(espNowRecvSourceAddress(sender_info));

    switch (static_cast<stampfly::MsgType>(frame.type)) {
        case stampfly::MsgType::CMD_STOP:
            portENTER_CRITICAL(&command_mux);
            mailbox.stop_pending = true;
            portEXIT_CRITICAL(&command_mux);
            break;
        case stampfly::MsgType::CMD_START:
            portENTER_CRITICAL(&command_mux);
            mailbox.start_pending = true;
            portEXIT_CRITICAL(&command_mux);
            break;
        case stampfly::MsgType::CMD_RESET:
            portENTER_CRITICAL(&command_mux);
            mailbox.reset_pending = true;
            portEXIT_CRITICAL(&command_mux);
            break;
        case stampfly::MsgType::CMD_SETPOINT: {
            stampfly::CmdSetpoint sp;
            if (!stampfly::deserialize(frame.payload, frame.len, &sp)) return;
            const uint32_t now_ms = millis();
            portENTER_CRITICAL(&command_mux);
            mailbox.setpoint = sp;          // 最新値で上書き(50Hzストリーム)
            mailbox.setpoint_seq = frame.seq;
            mailbox.setpoint_rx_ms = now_ms;
            mailbox.setpoint_pending = true;
            portEXIT_CRITICAL(&command_mux);
            break;
        }
        case stampfly::MsgType::CMD_POS_ERR: {
            stampfly::CmdPosErr pe;
            if (!stampfly::deserialize(frame.payload, frame.len, &pe)) return;
            const uint32_t now_ms = millis();
            portENTER_CRITICAL(&command_mux);
            mailbox.pos_err = pe;           // 最新値で上書き(50Hzストリーム)
            mailbox.pos_err_seq = frame.seq;
            mailbox.pos_err_rx_ms = now_ms;
            mailbox.pos_err_pending = true;
            portEXIT_CRITICAL(&command_mux);
            break;
        }
        default:
            // v2 コマンド(0x14-0x23, 0x25)はリングへ積む(期待長は検証済み)。
            // 満杯なら新着を落とす(PC側リトライで回復)。
            if (is_v2_command_type(frame.type)) {
                portENTER_CRITICAL(&command_mux);
                if (mailbox.v2_count < V2_COMMAND_QUEUE_CAPACITY) {
                    const size_t slot =
                        (mailbox.v2_head + mailbox.v2_count) % V2_COMMAND_QUEUE_CAPACITY;
                    V2Command& c = mailbox.v2_ring[slot];
                    c.type = frame.type;
                    c.seq = frame.seq;
                    c.len = frame.len;
                    if (frame.len > 0) std::memcpy(c.payload, frame.payload, frame.len);
                    mailbox.v2_count++;
                }
                portEXIT_CRITICAL(&command_mux);
            }
            break;  // 上記以外の未対応上り型は黙って破棄
    }
}

}  // namespace

void comm_init(void) {
    WiFi.mode(WIFI_STA);
    // チャネルピン留め(PC側機体プロファイルのチャネルと一致させる)
    esp_wifi_set_channel(FLIGHT_CONFIG.wifi_channel, WIFI_SECOND_CHAN_NONE);

    if (esp_now_init() != ESP_OK) {
        // 通信なしではSTARTも届かないため飛行は始まらない(安全側)。起動は継続。
        USBSerial.printf("ESP-NOW init failed\r\n");
        return;
    }
    esp_now_register_recv_cb(on_esp_now_recv);

    // ブート時の情報表示(定常状態ではUSBへ出力しない)
    USBSerial.printf("ESP-NOW ready: MAC=%s ch=%u\r\n",
                     WiFi.macAddress().c_str(),
                     static_cast<unsigned>(FLIGHT_CONFIG.wifi_channel));
}

bool comm_consume_commands(CommandSnapshot* out) {
    if (out == nullptr) return false;
    portENTER_CRITICAL(&command_mux);
    out->stop_pending = mailbox.stop_pending;
    out->start_pending = mailbox.start_pending;
    out->reset_pending = mailbox.reset_pending;
    out->setpoint_pending = mailbox.setpoint_pending;
    out->setpoint = mailbox.setpoint;
    out->setpoint_seq = mailbox.setpoint_seq;
    out->setpoint_rx_ms = mailbox.setpoint_rx_ms;
    out->pos_err_pending = mailbox.pos_err_pending;
    out->pos_err = mailbox.pos_err;
    out->pos_err_seq = mailbox.pos_err_seq;
    out->pos_err_rx_ms = mailbox.pos_err_rx_ms;
    // v2 コマンドリングを受信順に全件取り出す
    out->v2_count = mailbox.v2_count;
    for (size_t i = 0; i < mailbox.v2_count; i++) {
        out->v2[i] = mailbox.v2_ring[(mailbox.v2_head + i) % V2_COMMAND_QUEUE_CAPACITY];
    }
    mailbox.v2_head = 0;
    mailbox.v2_count = 0;
    mailbox.stop_pending = false;
    mailbox.start_pending = false;
    mailbox.reset_pending = false;
    mailbox.setpoint_pending = false;
    mailbox.pos_err_pending = false;
    portEXIT_CRITICAL(&command_mux);
    return out->stop_pending || out->start_pending || out->reset_pending ||
           out->setpoint_pending || out->pos_err_pending || out->v2_count > 0;
}

bool comm_relay_ready(void) {
    uint8_t mac[MAC_ADDRESS_LENGTH];
    return copy_relay_mac(mac);
}

bool comm_send_payload(stampfly::MsgType type, const uint8_t* payload, size_t payload_len) {
    uint8_t mac[MAC_ADDRESS_LENGTH];
    if (!copy_relay_mac(mac)) return false;

    // ESP-NOW区間はフレーム境界が保存されるため、論理フレームをそのまま送る(COBSなし)
    uint8_t frame[stampfly::MAX_FRAME_SIZE];
    size_t frame_len = 0;
    // seq は1始まり。0 は TLM_STATE.seq_echo の「未受信」番兵に予約されているため、
    // u32 ラップ時は 0 を飛ばして 1 へ戻す(PROTOCOL.md 論理フレーム)。
    if (++tx_seq == 0) tx_seq = 1;
    if (stampfly::pack_frame(type, tx_seq, payload, payload_len,
                             frame, sizeof(frame), &frame_len) != stampfly::ParseStatus::ok) {
        return false;
    }
    return esp_now_send(mac, frame, frame_len) == ESP_OK;
}

bool comm_send_log(const char* text) {
    if (text == nullptr) return false;
    // 上限超過は UTF-8 文字境界で切り詰める(多バイト文字を分断しない — PROTOCOL.md LOG_TEXT)
    const size_t text_len = stampfly::utf8_truncate_len(
        reinterpret_cast<const uint8_t*>(text), strlen(text), stampfly::MAX_LOG_TEXT_SIZE);

    stampfly::LogText msg;
    msg.origin = stampfly::LogText::ORIGIN_DRONE;
    msg.text = reinterpret_cast<const uint8_t*>(text);
    msg.text_len = text_len;

    uint8_t payload[1 + stampfly::MAX_LOG_TEXT_SIZE];
    size_t payload_len = 0;
    if (!stampfly::serialize(msg, payload, sizeof(payload), &payload_len)) return false;
    return comm_send_payload(stampfly::MsgType::LOG_TEXT, payload, payload_len);
}
