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

void MegmeetUART::dump_config()
{
    ESP_LOGCONFIG(TAG, "Megmeet UART");
}



// Loop

void MegmeetUART::loop()
{
    static uint8_t frame[10];
    static uint8_t pos = 0;

    static uint32_t packet_counter = 0;
    static uint32_t last_packet = 0;

    while (available()) {

        uint8_t b;
        if (!read_byte(&b))
            return;

        switch (pos) {

        // Wait for 0x55
        case 0:
            if (b == 0x55) {
                frame[0] = b;
                pos = 1;
            }
            break;

        // Wait for 0x35
        case 1:
            if (b == 0x35) {
                frame[1] = b;
                pos = 2;
            } else if (b == 0x55) {
                frame[0] = 0x55;
                pos = 1;
            } else {
                pos = 0;
            }
            break;

        default:
            frame[pos++] = b;

            if (pos == sizeof(frame)) {

                uint32_t now = millis();

                ESP_LOGI(TAG, "");
                ESP_LOGI(TAG,
                         "========== Packet %lu (+%lu ms) ==========",
                         (unsigned long) ++packet_counter,
                         (unsigned long) (now - last_packet));

                last_packet = now;

                char line[64];
                int len = 0;

                for (int i = 0; i < 10; i++) {
                    len += sprintf(line + len, "%02X ", frame[i]);
                }

                ESP_LOGI(TAG, "%s", line);

                pos = 0;
            }

            break;
        }
    }
}

// Loop End


void MegmeetUART::process_frame(const Frame &frame)
{
    switch (frame.type) {

        case 0x09:
            process_status(frame);
            break;

        case 0x2F:
            process_control(frame);
            break;

        case 0x4C:
            process_heartbeat(frame);
            break;

        case 0x61:
            process_sensor(frame);
            break;

        case 0x98:
            process_query(frame);
            break;

        default:
            process_unknown(frame);
            break;
    }
}



// Helpers

void MegmeetUART::dump_frame(const char *name, const Frame &f)
{
  ESP_LOGI(
      TAG,
      "%-10s %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
      name,
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
// Helpers End

// Handlers //

void MegmeetUART::process_control(const Frame &f)
{
  dump_frame("CONTROL", f);
}

void MegmeetUART::process_status(const Frame &f)
{
  dump_frame("STATUS", f);
}

void MegmeetUART::process_query(const Frame &f)
{
  dump_frame("QUERY", f);
}

void MegmeetUART::process_heartbeat(const Frame &f)
{
  dump_frame("HEARTBEAT", f);
}

void MegmeetUART::process_sensor(const Frame &f)
{
  dump_frame("SENSOR", f);
}

void MegmeetUART::process_unknown(const Frame &f)
{
  dump_frame("UNKNOWN", f);
}

// Handlers End


}  // namespace megmeet_uart
}  // namespace esphome
