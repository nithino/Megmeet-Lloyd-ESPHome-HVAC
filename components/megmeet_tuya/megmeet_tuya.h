#pragma once

// Lloyd / Megmeet AC mainboard UART on the Wi-Fi module header (product "Meg_Lloyd_AC_Gen").
//
// 9600 8N1, parity NONE, Tuya MCU protocol:
//   55 AA | VER | CMD | LEN_HI LEN_LO | DATA... | CHECKSUM (sum of all previous bytes & 0xFF)
// The AC streams VER=00 CMD=07 status reports on its own (sensor DPs, one per frame). It answers
// heartbeats (0x00), product query (0x01), working mode (0x02), DP query (0x08) and DP writes (0x06)
// (TX path confirmed 2026-11-23). With handshake enabled this component sends heartbeats every
// heartbeat_interval, and on boot and after an AC restart (heartbeat reply 0x00) it re-runs:
// product query -> working mode -> network status (0x03 = 0x04 "cloud connected") -> DP query.

#include <functional>
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
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_heartbeat_interval(uint32_t ms) { heartbeat_interval_ = ms; }
  void set_link_timeout(uint32_t ms) { link_timeout_ = ms; }
  void set_handshake(bool h) { handshake_ = h; }
  void set_link_sensor(binary_sensor::BinarySensor *s) { link_sensor_ = s; }
  void register_sensor(uint8_t dp, sensor::Sensor *s, float multiply, float offset);
  // Called with the raw value every time the AC reports this DP (reports, query answers, write echoes).
  void add_dp_listener(uint8_t dp, std::function<void(int32_t)> &&f) { listeners_.push_back({dp, std::move(f)}); }

  // --- TX helpers (usable from lambdas). Frames are queued and sent at least 150 ms apart. ---
  void send_command(uint8_t cmd, const std::vector<uint8_t> &payload, uint8_t version = 0x00);
  void set_dp_bool(uint8_t dp, bool value, uint8_t version = 0x00);
  void set_dp_value(uint8_t dp, int32_t value, uint8_t version = 0x00);
  void set_dp_enum(uint8_t dp, uint8_t value, uint8_t version = 0x00);
  void query_dps() { send_command(0x08, {}); }
  // Write a DP using the type the AC last reported for it (fallback_type if never reported).
  void write_dp(uint8_t dp, int32_t value, uint8_t fallback_type);
  void send_raw(const std::vector<uint8_t> &bytes);
  // Diagnostic: invert the TX signal in the GPIO matrix at runtime (ESP32-C3 only).
  void set_tx_inverted(uint8_t tx_gpio, bool inverted);

  // Last raw value seen for a DP; false if never seen.
  bool get_dp(uint8_t dp, int32_t *out) const;
  bool is_link_up() const;
  // Log every known DP (number, type, value) at INFO.
  void dump_dps();

 protected:
  void handle_frame_(uint8_t version, uint8_t cmd, const uint8_t *data, uint16_t len);
  void handle_dps_(const uint8_t *data, uint16_t len);
  void publish_dp_(uint8_t dp, uint8_t type, int32_t value);
  void start_init_(const char *why);

  struct SensorEntry {
    uint8_t dp;
    sensor::Sensor *sensor;
    float multiply;
    float offset;
  };
  struct Listener {
    uint8_t dp;
    std::function<void(int32_t)> f;
  };
  std::vector<SensorEntry> sensors_;
  std::vector<Listener> listeners_;
  std::map<uint8_t, int32_t> dp_values_;
  std::map<uint8_t, uint8_t> dp_types_;
  std::vector<uint8_t> rx_;
  std::vector<std::vector<uint8_t>> tx_queue_;

  binary_sensor::BinarySensor *link_sensor_{nullptr};
  bool handshake_{true};
  uint32_t heartbeat_interval_{15000};
  uint32_t link_timeout_{30000};
  uint32_t last_heartbeat_{0};
  uint32_t last_valid_frame_{0};
  uint32_t last_tx_{0};
  uint8_t init_step_{0};  // 0 = idle, 1..4 = next init command to send
  uint32_t init_next_{0};
  bool ever_valid_{false};
  uint32_t frames_ok_{0};
  uint32_t frames_bad_{0};
  uint32_t last_stats_{0};
  uint32_t mcu_restarts_{0};
};

}  // namespace megmeet_tuya
}  // namespace esphome
