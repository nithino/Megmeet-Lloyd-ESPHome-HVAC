#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace megmeet_uart {

class MegmeetUART : public Component,
                    public uart::UARTDevice {
 public:
  void setup() override;

  void loop() override;

  void dump_config() override;
};

}  // namespace megmeet_uart
}  // namespace esphome
