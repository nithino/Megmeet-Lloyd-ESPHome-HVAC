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


void MegmeetUART::loop() {
    static constexpr uint8_t HEADER1 = 0x55;
    static constexpr uint8_t HEADER2 = 0x35;
    static constexpr size_t FRAME_SIZE = 10;

    static uint8_t frame[FRAME_SIZE];
    static size_t index = 0;
    static uint32_t last_frame = 0;

    while (available()) {

        uint8_t byte;

        if (!read_byte(&byte))
            break;

        switch (index) {

        case 0:
            if (byte == HEADER1) {
                frame[index++] = byte;
            }
            break;

        case 1:
            if (byte == HEADER2) {
                frame[index++] = byte;
            } else if (byte == HEADER1) {
                frame[0] = HEADER1;
                index = 1;
            } else {
                index = 0;
            }
            break;

        default:
            frame[index++] = byte;

            if (index == FRAME_SIZE) {

                uint32_t now = millis();

                ESP_LOGI(
                    TAG,
                    "[+%4lu ms] CMD=%02X DATA=%02X %02X %02X CHK=%02X %02X",
                    (unsigned long)(now - last_frame),
                    frame[4],
                    frame[5],
                    frame[6],
                    frame[7],
                    frame[8],
                    frame[9]);

                last_frame = now;
                index = 0;
            }
            break;
        }
    }
}



// Loop End



}  // namespace megmeet_uart
}  // namespace esphome
