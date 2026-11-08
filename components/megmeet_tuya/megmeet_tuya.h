#pragma once

// Reader (and future writer) for the Lloyd / Megmeet AC mainboard UART on the Wi-Fi module header.
//
// Confirmed on the wire (9600 8N1, parity NONE): Tuya-style frames
//   55 AA | VER | CMD | LEN_HI LEN_LO | DATA... | CHECKSUM (sum of all previous bytes & 0xFF)
// The AC streams VER=00 CMD=07 (DP status report) frames on its own every ~1 s, one 4-byte integer
// DP per frame, without any handshake. It has never answered a heartbeat, query or DP write
// (see NOTES.md / REPORT.md, 2026-11-08). TX helpers are provided for when the TX path is fixed.

#include <map>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

namespace esphome {
namespace megmeet_tuya {

enum DpType : uint8_t { DP_RAW = 0, DP_BOOL = 1, DP_VALUE = 2, DP_STRING = 3, DP_ENUM = 4, DP_BITMAP = 5 };

class MegmeetTuya : public Component, public uart::UARTDevice {
 public:
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_heartbeat_interval(uint32_t ms) { heartbeat_interval_ = ms; }
  void set_link_timeout(uint32_t ms) { link_timeout_ = ms; }
  void set_link_sensor(binary_sensor::BinarySensor *s) { link_sensor_ = s; }
  void register_sensor(uint8_t dp, sensor::Sensor *s, float multiply, float offset);

  // --- TX helpers (usable from lambdas) ---
  // Version byte defaults to 0x00 (module->MCU in the public Tuya spec).
  void send_command(uint8_t cmd, const std::vector<uint8_t> &payload, uint8_t version = 0x00);
  void set_dp_bool(uint8_t dp, bool value, uint8_t version = 0x00);
  void set_dp_value(uint8_t dp, int32_t value, uint8_t version = 0x00);
  void set_dp_enum(uint8_t dp, uint8_t value, uint8_t version = 0x00);
  void send_raw(const std::vector<uint8_t> &bytes);
  // Diagnostic: invert the TX signal in the GPIO matrix at runtime (ESP32-C3 only).
  void set_tx_inverted(uint8_t tx_gpio, bool inverted);

  // Last raw value seen for a DP; false if never seen.
  bool get_dp(uint8_t dp, int32_t *out) const;

 protected:
  void handle_frame_(uint8_t version, uint8_t cmd, const uint8_t *data, uint16_t len);
  void handle_dps_(const uint8_t *data, uint16_t len);
  void publish_dp_(uint8_t dp, int32_t value);

  struct SensorEntry {
    uint8_t dp;
    sensor::Sensor *sensor;
    float multiply;
    float offset;
  };
  std::vector<SensorEntry> sensors_;
  std::map<uint8_t, int32_t> dp_values_;
  std::vector<uint8_t> rx_;

  binary_sensor::BinarySensor *link_sensor_{nullptr};
  uint32_t heartbeat_interval_{0};
  uint32_t link_timeout_{30000};
  uint32_t last_heartbeat_{0};
  uint32_t last_valid_frame_{0};
  bool ever_valid_{false};
  uint32_t frames_ok_{0};
  uint32_t frames_bad_{0};
  uint32_t last_stats_{0};
};

}  // namespace megmeet_tuya
}  // namespace esphome
