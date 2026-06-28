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
  static uint8_t frame[10];
  static uint8_t index = 0;

  while (available()) {
    uint8_t byte;

    if (!read_byte(&byte))
      break;

    switch (index) {

      // Waiting for first sync byte
      case 0:
        if (byte == 0x55) {
          frame[0] = byte;
          index = 1;
        }
        break;

      // Waiting for second sync byte
      case 1:
        if (byte == 0x35) {
          frame[1] = byte;
          index = 2;
        } else if (byte == 0x55) {
          // Possible new frame starting
          frame[0] = 0x55;
          index = 1;
        } else {
          index = 0;
        }
        break;

      default:
        frame[index++] = byte;

        if (index == sizeof(frame)) {

          ESP_LOGI(
              TAG,
              "RX: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X",
              frame[0],
              frame[1],
              frame[2],
              frame[3],
              frame[4],
              frame[5],
              frame[6],
              frame[7],
              frame[8],
              frame[9]);

          index = 0;
        }
        break;
    }
  }
}

// Loop End



}  // namespace megmeet_uart
}  // namespace esphome
