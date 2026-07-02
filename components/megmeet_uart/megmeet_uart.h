#pragma once

#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"

namespace esphome {
namespace megmeet_uart {

// -----------------------------------------------------------------------
// Frame layout (confirmed against captured traffic):
//
//   55 35 | 07 | 08 | TYPE | D1 D2 D3 | T1 T2
//   hdr1/2  LEN  sub   type   payload   tail
//
// - LEN (byte 2) is the count of bytes that follow it (always 7 so far,
//   i.e. every frame observed is a fixed 10 bytes).
// - sub-id (byte 3) is normally 0x08; two captured frames used 0x05
//   instead, with different TYPE/payload semantics -- likely a separate
//   handshake/negotiation sub-protocol, not yet decoded.
// - TYPE identifies the message: 0x4C=HEARTBEAT, 0x09=STATUS,
//   0x2F=CONTROL, 0x98=QUERY. Others are still unclassified.
// - T1/T2 (the last two bytes) are NOT a checksum of the preceding 8
//   bytes -- identical 9-byte payloads have been observed on the wire
//   with different T1/T2 values (verified against every common 8-bit
//   sum/XOR/CRC-8 scheme). T1 looks like a real, independently-varying
//   value per frame type; T2 tracks it closely enough to still look
//   checksum-like but the formula isn't confirmed.
//
// EVERYTHING below that transmits (replay_last_control_frame,
// adjust_control_setpoint) is an EXPERIMENT, not a confirmed working
// write path. All frames captured so far are things the mainboard SENT;
// we have zero confirmed examples of a frame the mainboard ACCEPTS as a
// command. Sending it a byte-for-byte copy of its own broadcast is a
// guess, not a known-good protocol. Treat every send as a probe: watch
// the AC directly after pressing, and don't be surprised if nothing
// happens -- that's expected until proven otherwise, not a bug.
// -----------------------------------------------------------------------

class MegmeetUART : public Component, public uart::UARTDevice {
 public:
  MegmeetUART() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_status_raw_sensor(sensor::Sensor *s) { status_raw_sensor_ = s; }
  void set_control_setpoint_raw_sensor(sensor::Sensor *s) { control_setpoint_raw_sensor_ = s; }
  void set_heartbeat_tick_sensor(sensor::Sensor *s) { heartbeat_tick_sensor_ = s; }

  void set_last_status_frame_sensor(text_sensor::TextSensor *s) { last_status_frame_sensor_ = s; }
  void set_last_control_frame_sensor(text_sensor::TextSensor *s) { last_control_frame_sensor_ = s; }
  void set_last_heartbeat_frame_sensor(text_sensor::TextSensor *s) { last_heartbeat_frame_sensor_ = s; }
  void set_last_unknown_frame_sensor(text_sensor::TextSensor *s) { last_unknown_frame_sensor_ = s; }

  // --- Experimental TX, called from the button platform ---

  // Re-send the most recently captured CONTROL frame, byte-for-byte,
  // unmodified. Null-hypothesis test: does the mainboard react to
  // receiving a copy of its own last broadcast at all?
  void replay_last_control_frame();

  // Re-send the most recently captured CONTROL frame with tail1 shifted
  // by delta_steps * 2 (2 is the step size observed between real
  // captured setpoint values). tail2 is copied unchanged from the
  // captured frame -- we don't have a formula to recompute it, so this
  // is deliberately testing whether the mainboard cares about tail2
  // matching tail1, or ignores/recomputes it on its own.
  void adjust_control_setpoint(int delta_steps);

 protected:
  struct Frame {
    uint8_t header1;
    uint8_t header2;
    uint8_t len;
    uint8_t sub_id;
    uint8_t type;
    uint8_t data1;
    uint8_t data2;
    uint8_t data3;
    uint8_t tail1;
    uint8_t tail2;
  };

  struct PacketState {
    bool valid = false;
    Frame frame{};
  };

  static constexpr uint8_t TYPE_HEARTBEAT = 0x4C;
  static constexpr uint8_t TYPE_STATUS = 0x09;
  static constexpr uint8_t TYPE_CONTROL = 0x2F;
  static constexpr uint8_t TYPE_QUERY = 0x98;

  std::vector<uint8_t> rx_buffer_;
  PacketState packet_db_[256];

  uint32_t frame_count_{0};
  uint32_t unknown_count_{0};

  Frame last_control_frame_{};
  bool control_frame_valid_{false};

  void process_frame(const Frame &frame);

  void handle_heartbeat_(const Frame &frame);
  void handle_status_(const Frame &frame);
  void handle_control_(const Frame &frame);
  void handle_query_(const Frame &frame);
  void handle_unknown_(const Frame &frame);

  void compare_frame_(const Frame &frame);

  std::string frame_to_hex_(const Frame &frame);
  void publish_text_(text_sensor::TextSensor *sensor, const std::string &value);
  void publish_val_(sensor::Sensor *sensor, float value);
  void transmit_frame_(const Frame &frame);

  sensor::Sensor *status_raw_sensor_{nullptr};
  sensor::Sensor *control_setpoint_raw_sensor_{nullptr};
  sensor::Sensor *heartbeat_tick_sensor_{nullptr};

  text_sensor::TextSensor *last_status_frame_sensor_{nullptr};
  text_sensor::TextSensor *last_control_frame_sensor_{nullptr};
  text_sensor::TextSensor *last_heartbeat_frame_sensor_{nullptr};
  text_sensor::TextSensor *last_unknown_frame_sensor_{nullptr};
};

}  // namespace megmeet_uart
}  // namespace esphome
