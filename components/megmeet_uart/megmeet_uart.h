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
// IMPORTANT, corrected after a bug was caught in send_short_setpoint_probe():
// SETPOINT_EVENT (0x20) uses sub_id=0x08 -- the SAME sub_id as the main
// HEARTBEAT/STATUS/CONTROL/QUERY broadcast channel -- NOT sub_id=0x05
// like POWER/FAN/MODE below. This was wrongly assumed to match the
// others (see the old SUB_ID_EVENT comment, now corrected) and wasn't
// actually verified until a probe frame was sent with the wrong sub_id
// and caught by comparing it against real captured bytes. Real captured
// example: 55 35 07 08 20 00 10 00 1B CC -- sub_id is byte 4, 0x08.
// Worth remembering as a real, unexplained asymmetry: setpoint changes
// are event-driven like power/fan/mode, but ride the main channel while
// those three use the separate one. Don't assume TYPE alone tells you
// the sub_id for any frame type on this bus -- check real captured
// bytes every time, as this incident demonstrates.
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
// really off.
//
// AC RUNNING (revised -- the first version of this was wrong, see
// below): the reliable signal is NOT a specific STATUS value, it's
// whether STATUS frames are being sent AT ALL. Two controlled tests:
//   1. Power off, wait 90s+ uninterrupted: STATUS stopped updating
//      right as the AC's flap physically closed (originally read as
//      "value freezes at 0x00", see caveat below).
//   2. Full ESP32 power-cycle while the AC was already off (steady
//      state, not mid-countdown): during 86 CONFIRMED-off seconds
//      before the next power-on press, ZERO STATUS frames arrived --
//      HEARTBEAT and CONTROL kept flowing the whole time regardless.
//      The very first STATUS frame arrived 231ms after the power-ON
//      press, with a non-zero tail1 (0x1C). That directly disproves
//      "tail1 == 0x00" as the off signal -- it happened to be 0x00 at
//      the exact moment test 1 stopped, but that's not what off looks
//      like in general; ABSENCE of STATUS is what off looks like.
// Implemented as a staleness check: ac_running_sensor_ goes false if no
// STATUS frame has arrived for AC_RUNNING_TIMEOUT_MS, true the instant
// one arrives. The timeout is chosen well above the longest STATUS gap
// seen during confirmed-running traffic in any capture so far (53s, in
// a fan-speed test), with margin -- see AC_RUNNING_TIMEOUT_MS. STATUS
// itself keeps behaving as a raw diagnostic value otherwise; the OLD
// STATUS_STOPPED_VALUE/tail1-based logic has been removed.
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

  // The above two operate on CONTROL (0x2F), which is CONFIRMED NOT to
  // be the real setpoint (see the block above) -- they were always
  // testing the wrong frame. These operate on TYPE_SETPOINT_EVENT
  // (0x20) instead, the frame we've actually confirmed carries the
  // real setpoint. Same experimental status otherwise: nothing here is
  // a confirmed write path, only a probe. Start with
  // replay_last_setpoint_event() -- it's byte-for-byte identical to
  // real traffic the mainboard already accepts as valid framing, so
  // it's the lowest-risk way to test whether inbound frames get
  // processed at all before trying one with a changed value.
  void replay_last_setpoint_event();

  // Re-send the most recently captured SETPOINT_EVENT frame with tail1
  // shifted by delta_degrees directly (no doubling -- tail1 IS the
  // degrees C value here, confirmed 1:1, unlike CONTROL's guessed *2
  // step). tail2 is copied unchanged, same open question as above.
  void adjust_setpoint_event(int delta_degrees);

  // SEPARATE, more speculative probe: tests whether the mainboard's
  // UART parser secretly understands Tuya's PUBLIC standard MCU serial
  // protocol (55 AA framing, DP-based commands) for inbound commands,
  // even though everything it broadcasts uses 55 35 framing. This is a
  // real product built on Tuya's platform (confirmed), but the specific
  // protocol we've reverse-engineered is confirmed NOT to be Tuya's
  // published spec (different header, different checksum algorithm) --
  // so this is a low-probability, low-cost longshot, not a confirmed
  // lead. Sends a standard Tuya "set DP1 (power) to off/on" command
  // frame, built and checksummed per Tuya's public spec. DP1-for-power
  // is a common convention for their AC product category, NOT
  // confirmed for this specific product -- a negative result here
  // rules out that one convention, not "any possible Tuya framing."
  void send_tuya_power_probe(bool on);

  // SEPARATE probe, different hypothesis: tests whether the mainboard's
  // parser is length-driven (the `len` byte in every real frame we've
  // captured exactly equals the byte count that follows it -- 0x07,
  // matching sub_id+type+data1+data2+data3+tail1+tail2 -- suggesting
  // it's a genuine length field, not a constant) and whether a SHORTER
  // frame, dropping the padding bytes that are always 00 10 00 in every
  // real SETPOINT_EVENT broadcast, might be what a genuine command
  // looks like. Sends: 55 35 04 08 20 [current setpoint] [tail2], 7
  // bytes total, len correctly set to 4. Null-hypothesis test like
  // replay_last_setpoint_event() -- uses the AC's own current setpoint
  // value, not a changed one, so a reaction can only mean "this shape
  // got processed," not "and the value took." tail2 is a GUESS (copied
  // from the last real captured frame) -- we have no checksum formula
  // for this shortened shape, so a negative result here is ambiguous
  // (wrong shape vs. rejected checksum), unlike the byte-for-byte replay
  // above. This is intentionally more speculative than the probes above.
  void send_short_setpoint_probe();

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

  // The main periodic broadcast channel: HEARTBEAT/STATUS/CONTROL/QUERY,
  // AND (despite being event-driven, not periodic) SETPOINT_EVENT too --
  // see the corrected note at the top of this file. Named explicitly
  // after send_short_setpoint_probe() was caught using SUB_ID_EVENT
  // (wrong) instead of this for a SETPOINT_EVENT-shaped frame.
  static constexpr uint8_t SUB_ID_MAIN = 0x08;

  // POWER/FAN/MODE's separate channel -- confirmed distinct from
  // SUB_ID_MAIN for those three specifically. Do NOT assume this also
  // covers SETPOINT_EVENT; it doesn't (see above).
  static constexpr uint8_t SUB_ID_EVENT = 0x05;

  // ac_running_sensor_ goes false if no STATUS frame arrives for this
  // long. Set well above the longest STATUS-to-STATUS gap observed
  // during confirmed-running traffic across every capture so far (53s,
  // in a fan-speed test) -- see the block above for the full reasoning.
  // This is a heuristic margin, not a proven bound; if a longer running
  // gap ever surfaces, this needs to grow accordingly.
  static constexpr uint32_t AC_RUNNING_TIMEOUT_MS = 90000;

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

  // Last genuine SETPOINT_EVENT (0x20) frame seen -- basis for
  // replay_last_setpoint_event()/adjust_setpoint_event(). Unlike
  // last_control_frame_, this one IS the confirmed real setpoint field.
  Frame last_setpoint_event_frame_{};
  bool setpoint_event_valid_{false};

  // Timestamp (millis()) of the last STATUS frame received, used by the
  // loop()-driven staleness check that drives ac_running_sensor_. 0
  // means "never seen one" -- see check_ac_running_() and the block
  // above for why absence, not value, is the running/off signal.
  uint32_t last_status_millis_{0};
  bool status_ever_seen_{false};
  // Set once in setup(). If STATUS is never seen at all (AC already off
  // when this booted), check_ac_running_() falls back to timing out
  // against this instead, so the sensor still converges to "false"
  // within AC_RUNNING_TIMEOUT_MS rather than sitting at unknown forever.
  uint32_t boot_millis_{0};

  void process_frame(const Frame &frame);
  void check_ac_running_();

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
