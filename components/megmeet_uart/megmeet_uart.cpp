#include "megmeet_uart.h"

namespace esphome {
namespace megmeet_uart {

static const char *const TAG = "megmeet_uart";

#define MEGMEET_DEBUG 1

// -----------------------------------------------------------------------------
// RX Ring Buffer
// -----------------------------------------------------------------------------

static constexpr size_t RX_BUFFER_SIZE = 512;

static uint8_t rx_buffer[RX_BUFFER_SIZE];
static size_t rx_head = 0;
static size_t rx_tail = 0;

// -----------------------------------------------------------------------------
// Helper Functions
// -----------------------------------------------------------------------------
static uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0;

    for (size_t i = 0; i < len; i++)
        crc += data[i];

    return crc;
}

static inline void dump_byte(uint8_t byte)
{
#if MEGMEET_DEBUG
    ESP_LOGI(TAG, "%02X", byte);
#endif
}

static inline bool buffer_full()
{
    return ((rx_head + 1) % RX_BUFFER_SIZE) == rx_tail;
}

static inline bool buffer_empty()
{
    return rx_head == rx_tail;
}

static inline void buffer_push(uint8_t b)
{
    if (buffer_full()) {
        ESP_LOGW(TAG, "RX buffer overflow, dropping oldest byte");
        rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;
    }

    rx_buffer[rx_head] = b;
    rx_head = (rx_head + 1) % RX_BUFFER_SIZE;
}

static inline bool buffer_pop(uint8_t &b)
{
    if (buffer_empty())
        return false;

    b = rx_buffer[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUFFER_SIZE;

    return true;
}

// -----------------------------------------------------------------------------
// Component
// -----------------------------------------------------------------------------

void MegmeetUART::setup()
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " Megmeet UART Protocol Analyzer");
    ESP_LOGI(TAG, "========================================");
}

static void event_marker(const char *name)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " EVENT : %s", name);
    ESP_LOGI(TAG, "========================================");
}

void MegmeetUART::dump_config()
{
    ESP_LOGCONFIG(TAG, "Megmeet UART");
}



// Loop

void MegmeetUART::loop()
{
    static std::vector<uint8_t> rx;

    while (available()) {
        uint8_t b;
        if (!read_byte(&b))
            break;

        rx.push_back(b);
    }

    while (rx.size() >= 2) {

        // Find header
        if (!(rx[0] == 0x55 && rx[1] == 0x35)) {
            rx.erase(rx.begin());
            continue;
        }

        // Wait until a full 10-byte frame exists
        if (rx.size() < 10)
            return;

        Frame frame;

        frame.header1 = rx[0];
        frame.header2 = rx[1];
        frame.proto1  = rx[2];
        frame.proto2  = rx[3];
        frame.type    = rx[4];
        frame.data1   = rx[5];
        frame.data2   = rx[6];
        frame.data3   = rx[7];
        frame.crc1    = rx[8];
        frame.crc2    = rx[9];

        process_frame(frame);

        rx.erase(rx.begin(), rx.begin() + 10);
    }
}

// Loop End


void MegmeetUART::process_frame(const Frame &frame)
{
    compare_frame(frame);
}

void MegmeetUART::dump_frame(const char *title, const Frame &f)
{
    ESP_LOGI(TAG,
             "%-10s %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
             title,
             f.header1,
             f.header2,
             f.proto1,
             f.proto2,
             f.type,
             f.data1,
             f.data2,
             f.data3,
             f.crc1,
             f.crc2);
}

void MegmeetUART::compare_frame(const Frame &frame)
{
    PacketState &old = packet_db_[frame.type];

    if (!old.valid) {
        old.valid = true;
        old.frame = frame;

        dump_frame("NEW", frame);
        return;
    }

    bool changed = false;

    if (frame.proto1 != old.frame.proto1) changed = true;
    if (frame.proto2 != old.frame.proto2) changed = true;
    if (frame.data1  != old.frame.data1) changed = true;
    if (frame.data2  != old.frame.data2) changed = true;
    if (frame.data3  != old.frame.data3) changed = true;
    if (frame.crc1   != old.frame.crc1) changed = true;
    if (frame.crc2   != old.frame.crc2) changed = true;

    if (!changed)
        return;

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "====================================");
    ESP_LOGI(TAG, "TYPE %02X CHANGED", frame.type);
    ESP_LOGI(TAG, "====================================");

    if (frame.proto1 != old.frame.proto1)
        ESP_LOGI(TAG, "proto1 : %02X -> %02X", old.frame.proto1, frame.proto1);

    if (frame.proto2 != old.frame.proto2)
        ESP_LOGI(TAG, "proto2 : %02X -> %02X", old.frame.proto2, frame.proto2);

    if (frame.data1 != old.frame.data1)
        ESP_LOGI(TAG, "data1  : %02X -> %02X", old.frame.data1, frame.data1);

    if (frame.data2 != old.frame.data2)
        ESP_LOGI(TAG, "data2  : %02X -> %02X", old.frame.data2, frame.data2);

    if (frame.data3 != old.frame.data3)
        ESP_LOGI(TAG, "data3  : %02X -> %02X", old.frame.data3, frame.data3);

    if (frame.crc1 != old.frame.crc1)
        ESP_LOGI(TAG, "byte8  : %02X -> %02X", old.frame.crc1, frame.crc1);

    if (frame.crc2 != old.frame.crc2)
        ESP_LOGI(TAG, "byte9  : %02X -> %02X", old.frame.crc2, frame.crc2);

    dump_frame("OLD", old.frame);
    dump_frame("NEW", frame);

    old.frame = frame;
}


// Helpers


// Helpers End

// Handlers //

void MegmeetUART::process_unknown(const Frame &f)
{
    ESP_LOGI(TAG,
        "UNKNOWN : "
        "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
        f.header1,
        f.header2,
        f.proto1,
        f.proto2,
        f.type,
        f.data1,
        f.data2,
        f.data3,
        f.crc1,
        f.crc2);
}

// Handlers End


}  // namespace megmeet_uart
}  // namespace esphome
