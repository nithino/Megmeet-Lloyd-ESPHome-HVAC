#pragma once

#include <string>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/uart/uart.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

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
//
// CONFIRMED via a controlled test (2 "temp down" + 3 "temp up" presses,
// 30s apart, AC otherwise idle): the genuine user setpoint is NOT in
// CONTROL (0x2F) at all. It's a previously-"UNKNOWN" frame type, 0x20,
// which the mainboard sends once, 4-22ms after the IR receiver decodes
// a temperature button -- not as part of the periodic broadcast cycle.
// Its tail1 byte is the setpoint in degrees C directly: no scaling, no
// offset. All 5 presses in the test matched exactly (24 -> 23 -> 22 ->
// 23 -> 24 -> 25). See handle_setpoint_event_().
//
// CONTROL (0x2F) tail1 is NOT the setpoint -- that was an earlier,
// wrong hypothesis based on a too-short capture window. Over a longer
// (18 min) idle session it was shown to drift continuously in small
// +/-2 steps with zero button presses, which a real setpoint can't do.
// It's most likely a live feedback value (e.g. coil/regulation target)
// unrelated to what the user dialed in. It's still logged via
// last_control_frame_sensor_ (raw hex) for future analysis, but no
// numeric sensor is derived from it anymore -- don't resurrect
// control_setpoint_raw/control_ramp_raw without new evidence.
//
// POWER: type=0x20's sibling discovery. type=0x40, sub_id=0x05, fires
// once, instantly (same event-driven signature as setpoint), the moment
// the mainboard receives a power button press. data3: 0x0E = off
// requested, 0x0F = on requested. CONFIRMED, simple, high confidence --
// see handle_power_event_().
//
// IMPORTANT: 0x40=0x0E means the OFF command was received, not that the
// AC has actually stopped. This unit runs a ~60s shutdown countdown
// (the physical flap visibly closes at the end of it) before it's
// really off. Controlled test (power off, wait 90s+ uninterrupted)
// showed STATUS and CONTROL tail1 both drop to 0x00 and then stop
// updating entirely, ~59-64s after the off command -- that's the real
// "AC has stopped" signal. See ac_running_sensor_ / handle_status_().
// 0x00 was never observed on STATUS during normal operation in any
// capture so far, but this is one data point, not an exhaustive proof.
//
// FAN SPEED: sub_id=0x05, event-driven like the above, but the mapping
// is NOT fully nailed down -- deliberately exposed as a raw hex text
// sensor (last_fan_event_frame_sensor_) rather than a named speed enum.
// Controlled test (low -> low-medium -> medium -> high -> auto -> low)
// showed TWO different frame types: type=0x2B specifically and
// consistently (reproduced twice) for the low<->low-medium transition,
// and type=0x41 for the other three transitions, with data3 = 0x40
// (medium), 0xA0 (high), 0x20 (auto), 0x15 (low). Only one sample each
// for those four, and the values aren't an obvious linear encoding, so
// don't build a speed-name lookup table on this without more data.
//
// MODE: sub_id=0x05, type=0x10, data3 = 0xA0 (Dry), 0x10 (Fan), 0x14
// (Auto) in one controlled test -- same one-sample-per-value caveat as
// fan speed, exposed the same way (last_mode_event_frame_sensor_, raw
// hex only). What IS confirmed and useful: a rejected mode change
// (this unit doesn't support Heat -- no beep, no AC reaction) produced
// NO type=0x10 frame at all. Presence/absence of this frame is a
// reliable accept/reject signal even without trusting the exact byte
// mapping. Mode changes also have real side effects worth knowing about
// if you see them elsewhere in the traffic: each mode remembers its own
// last setpoint and fan speed, so switching modes can trigger a
// TYPE_SETPOINT_EVENT and/or a fan-family frame as those get restored
// -- that's the AC's actual behavior, not a decoding error.
//
// Frames with sub_id=0x05 are NOT guaranteed to have globally unique
// TYPE bytes vs the main sub_id=0x08 broadcast channel -- type=0x2E was
// observed under both sub_ids with different payload shapes during the
// mode test, still undeciphered. Every sub_id=0x05 handler below
// double-checks sub_id itself and falls back to handle_unknown_() (with
// a warning) on a mismatch, rather than assuming the type byte alone is
// enough.
// -----------------------------------------------------------------------

class MegmeetUART : public Component, public uart::UARTDevice {
 public:
  MegmeetUART() = default;

  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_status_raw_sensor(sensor::Sensor *s) { status_raw_sensor_ = s; }
  void set_setpoint_celsius_sensor(sensor::Sensor *s) { setpoint_celsius_sensor_ = s; }
  void set_heartbeat_tick_sensor(sensor::Sensor *s) { heartbeat_tick_sensor_ = s; }

  void set_power_requested_sensor(binary_sensor::BinarySensor *s) { power_requested_sensor_ = s; }
  void set_ac_running_sensor(binary_sensor::BinarySensor *s) { ac_running_sensor_ = s; }

  void set_last_status_frame_sensor(text_sensor::TextSensor *s) { last_status_frame_sensor_ = s; }
  void set_last_control_frame_sensor(text_sensor::TextSensor *s) { last_control_frame_sensor_ = s; }
  void set_last_heartbeat_frame_sensor(text_sensor::TextSensor *s) { last_heartbeat_frame_sensor_ = s; }
  void set_last_unknown_frame_sensor(text_sensor::TextSensor *s) { last_unknown_frame_sensor_ = s; }
  void set_last_fan_event_frame_sensor(text_sensor::TextSensor *s) { last_fan_event_frame_sensor_ = s; }
  void set_last_mode_event_frame_sensor(text_sensor::TextSensor *s) { last_mode_event_frame_sensor_ = s; }

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
  // Event-driven, not periodic: sent once by the mainboard 4-22ms after
  // the IR receiver decodes a temperature button press. tail1 = setpoint
  // in degrees C, direct (confirmed by controlled test, see block above).
  static constexpr uint8_t TYPE_SETPOINT_EVENT = 0x20;
  // Event-driven. data3: 0x0E = power off requested, 0x0F = power on
  // requested. Fires instantly on button press -- does NOT mean the AC
  // has actually finished its ~60s shutdown countdown, see block above.
  static constexpr uint8_t TYPE_POWER_EVENT = 0x40;
  // Event-driven, fan speed related. Two distinct type bytes observed:
  // 0x2B specifically for the low<->low-medium transition, 0x41 for
  // every other transition. See block above for what is/isn't confirmed.
  static constexpr uint8_t TYPE_FAN_EVENT_LOW = 0x2B;
  static constexpr uint8_t TYPE_FAN_EVENT_OTHER = 0x41;
  // Event-driven mode change. Absence of this frame after a mode button
  // press is a reliable signal the mainboard rejected the mode (e.g.
  // unsupported Heat on a cooling-only unit). See block above.
  static constexpr uint8_t TYPE_MODE_EVENT = 0x10;

  // All the event-driven frames above (setpoint/power/fan/mode) have
  // only ever been observed with this sub_id. The main periodic
  // broadcast frames (HEARTBEAT/STATUS/CONTROL/QUERY) use 0x08.
  static constexpr uint8_t SUB_ID_EVENT = 0x05;

  // STATUS/CONTROL tail1 sitting at 0x00 (and no longer updating) is
  // the confirmed signal that the AC has genuinely finished its
  // shutdown countdown -- see block above. Tracked here so
  // handle_status_() can drive ac_running_sensor_.
  static constexpr uint8_t STATUS_STOPPED_VALUE = 0x00;

  std::vector<uint8_t> rx_buffer_;
  PacketState packet_db_[256];

  uint32_t frame_count_{0};
  uint32_t unknown_count_{0};

  // Last CONTROL frame seen, used as the (unconfirmed/experimental)
  // basis for replay_last_control_frame()/adjust_control_setpoint().
  // CONTROL is known NOT to carry the real setpoint (see block above),
  // so these TX experiments are testing blind until a write path is
  // found for the real setpoint mechanism (TYPE_SETPOINT_EVENT).
  Frame last_control_frame_{};
  bool control_frame_valid_{false};

  void process_frame(const Frame &frame);

  void handle_heartbeat_(const Frame &frame);
  void handle_status_(const Frame &frame);
  void handle_control_(const Frame &frame);
  void handle_query_(const Frame &frame);
  void handle_setpoint_event_(const Frame &frame);
  void handle_power_event_(const Frame &frame);
  void handle_fan_event_(const Frame &frame);
  void handle_mode_event_(const Frame &frame);
  void handle_unknown_(const Frame &frame);

  void compare_frame_(const Frame &frame);

  std::string frame_to_hex_(const Frame &frame);
  void publish_text_(text_sensor::TextSensor *sensor, const std::string &value);
  void publish_val_(sensor::Sensor *sensor, float value);
  void publish_bin_(binary_sensor::BinarySensor *sensor, bool value);
  void transmit_frame_(const Frame &frame);

  sensor::Sensor *status_raw_sensor_{nullptr};
  sensor::Sensor *setpoint_celsius_sensor_{nullptr};
  sensor::Sensor *heartbeat_tick_sensor_{nullptr};

  binary_sensor::BinarySensor *power_requested_sensor_{nullptr};
  binary_sensor::BinarySensor *ac_running_sensor_{nullptr};

  text_sensor::TextSensor *last_status_frame_sensor_{nullptr};
  text_sensor::TextSensor *last_control_frame_sensor_{nullptr};
  text_sensor::TextSensor *last_heartbeat_frame_sensor_{nullptr};
  text_sensor::TextSensor *last_unknown_frame_sensor_{nullptr};
  text_sensor::TextSensor *last_fan_event_frame_sensor_{nullptr};
  text_sensor::TextSensor *last_mode_event_frame_sensor_{nullptr};
};

}  // namespace megmeet_uart
}  // namespace esphome
