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
    LOG_UART_DEVICE(this);
}

void MegmeetUART::loop()
{
    static uint32_t last_byte_time = 0;

    while (available()) {

        uint8_t byte;

        if (!read_byte(&byte))
            break;

        uint32_t now = millis();

#if MEGMEET_DEBUG
        if (last_byte_time != 0) {
            ESP_LOGD(TAG,
                     "[+%4lu ms] %02X",
                     (unsigned long)(now - last_byte_time),
                     byte);
        } else {
            ESP_LOGD(TAG,
                     "[startup] %02X",
                     byte);
        }
#endif

        last_byte_time = now;

        buffer_push(byte);

        dump_byte(byte);
    }
}

}  // namespace megmeet_uart
}  // namespace esphome
