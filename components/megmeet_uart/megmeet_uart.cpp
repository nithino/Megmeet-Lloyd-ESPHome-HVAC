#include "megmeet_uart.h"

namespace esphome {
namespace megmeet_uart {

static const char *const TAG = "megmeet_uart";

void MegmeetUART::setup() {
  ESP_LOGI(TAG, "Megmeet UART initialized");
}

void MegmeetUART::dump_config() {
  ESP_LOGCONFIG(TAG, "Megmeet UART");
}

void MegmeetUART::loop() {

  while (available()) {

    uint8_t c;

    if (!read_byte(&c))
      return;

    ESP_LOGI(TAG, "%02X", c);
  }

}

}  // namespace megmeet_uart
}  // namespace esphome
