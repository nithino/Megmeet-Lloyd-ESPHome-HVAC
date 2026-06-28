#pragma once

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"

namespace esphome {
namespace megmeet_uart {

class MegmeetUART : public Component, public uart::UARTDevice {

 public:
  MegmeetUART(uart::UARTComponent *parent) : UARTDevice(parent) {}

  void setup() override;
  void loop() override;
  void dump_config() override;

 protected:
  struct Frame {
    uint8_t header1;
    uint8_t header2;
    uint8_t proto1;
    uint8_t proto2;
    uint8_t type;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t crc1;
    uint8_t crc2;
  };

  void process_frame(const Frame &frame);

  void process_status(const Frame &frame);
  void process_control(const Frame &frame);
  void process_query(const Frame &frame);
  void process_heartbeat(const Frame &frame);
  void process_sensor(const Frame &frame);
  void process_unknown(const Frame &frame);
};

}  // namespace megmeet_uart
}  // namespace esphome
