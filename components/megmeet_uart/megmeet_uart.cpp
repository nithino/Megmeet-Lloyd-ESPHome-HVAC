#include "megmeet_uart.h"
#include <cinttypes>
#include <cstdio>

namespace esphome {
namespace megmeet_uart {

static const char *const TAG = "megmeet_uart";

void MegmeetUART::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Megmeet UART...");
  boot_millis_ = millis();
}

void MegmeetUART::dump_config() {
  ESP_LOGCONFIG(TAG, "Megmeet UART:");
  LOG_SENSOR("  ", "Status Raw", this->status_raw_sensor_);
  LOG_SENSOR("  ", "Setpoint Celsius", this->setpoint_celsius_sensor_);
  LOG_SENSOR("  ", "Heartbeat Tick", this->heartbeat_tick_sensor_);
  LOG_BINARY_SENSOR("  ", "Power Requested", this->power_requested_sensor_);
  LOG_BINARY_SENSOR("  ", "AC Running", this->ac_running_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Status Frame", this->last_status_frame_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Control Frame", this->last_control_frame_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Heartbeat Frame", this->last_heartbeat_frame_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Unknown Frame", this->last_unknown_frame_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Fan Event Frame", this->last_fan_event_frame_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Mode Event Frame", this->last_mode_event_frame_sensor_);
}

// -----------------------------------------------------------------------
// Byte stream -> frames
// -----------------------------------------------------------------------

void MegmeetUART::loop() {
  // Runs every call regardless of what's on the wire right now -- this
  // is what lets ac_running_sensor_ notice STATUS has gone quiet even
  // when no new bytes ever arrive to trigger the parsing below.
  check_ac_running_();

  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b))
      break;
    rx_buffer_.push_back(b);
  }

  while (rx_buffer_.size() >= 2) {
    // Resync on the 55 35 header, discarding stray bytes.
    if (!(rx_buffer_[0] == 0x55 && rx_buffer_[1] == 0x35)) {
      rx_buffer_.erase(rx_buffer_.begin());
      continue;
    }

    // Every frame observed so far is a fixed 10 bytes (len byte is
    // always 0x07). Wait for a full frame before parsing.
    if (rx_buffer_.size() < 10)
      return;

    Frame frame{};
    frame.header1 = rx_buffer_[0];
    frame.header2 = rx_buffer_[1];
    frame.len = rx_buffer_[2];
    frame.sub_id = rx_buffer_[3];
    frame.type = rx_buffer_[4];
    frame.data1 = rx_buffer_[5];
    frame.data2 = rx_buffer_[6];
    frame.data3 = rx_buffer_[7];
    frame.tail1 = rx_buffer_[8];
    frame.tail2 = rx_buffer_[9];

    process_frame(frame);

    rx_buffer_.erase(rx_buffer_.begin(), rx_buffer_.begin() + 10);
  }
}

void MegmeetUART::process_frame(const Frame &frame) {
  frame_count_++;

  switch (frame.type) {
    case TYPE_HEARTBEAT:
      handle_heartbeat_(frame);
      break;
    case TYPE_STATUS:
      handle_status_(frame);
      break;
    case TYPE_CONTROL:
      handle_control_(frame);
      break;
    case TYPE_QUERY:
      handle_query_(frame);
      break;
    case TYPE_SETPOINT_EVENT:
      handle_setpoint_event_(frame);
      break;
    case TYPE_POWER_EVENT:
      handle_power_event_(frame);
      break;
    case TYPE_FAN_EVENT_LOW:
    case TYPE_FAN_EVENT_OTHER:
      handle_fan_event_(frame);
      break;
    case TYPE_MODE_EVENT:
      handle_mode_event_(frame);
      break;
    default:
      unknown_count_++;
      handle_unknown_(frame);
      break;
  }

  // Background diff-logger: prints whenever a given TYPE's payload
  // changes from what we last saw. This is the tool that found the
  // CONTROL setpoint stepping pattern -- leave it running at DEBUG level
  // for continued reverse engineering.
  compare_frame_(frame);
}

void MegmeetUART::check_ac_running_() {
  if (!status_ever_seen_) {
    // AC may already have been off when we booted -- STATUS won't ever
    // arrive to seed last_status_millis_ in that case. Fall back to
    // timing out against boot instead, so this still converges to
    // "false" within AC_RUNNING_TIMEOUT_MS rather than sitting at
    // unknown indefinitely.
    if (millis() - boot_millis_ > AC_RUNNING_TIMEOUT_MS) {
      publish_bin_(ac_running_sensor_, false);
      if (climate_ != nullptr)
        climate_->update_power_state(false);
    }
    return;
  }
  if (millis() - last_status_millis_ > AC_RUNNING_TIMEOUT_MS) {
    publish_bin_(ac_running_sensor_, false);
    if (climate_ != nullptr)
      climate_->update_power_state(false);
  }
}

std::string MegmeetUART::frame_to_hex_(const Frame &f) {
  char buf[32];
  snprintf(buf, sizeof(buf), "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", f.header1, f.header2, f.len,
           f.sub_id, f.type, f.data1, f.data2, f.data3, f.tail1, f.tail2);
  return std::string(buf);
}

void MegmeetUART::publish_text_(text_sensor::TextSensor *sensor, const std::string &value) {
  if (sensor == nullptr)
    return;
  if (sensor->has_state() && sensor->state == value)
    return;
  sensor->publish_state(value);
}

void MegmeetUART::publish_val_(sensor::Sensor *sensor, float value) {
  if (sensor == nullptr)
    return;
  if (sensor->has_state() && sensor->state == value)
    return;
  sensor->publish_state(value);
}

void MegmeetUART::publish_bin_(binary_sensor::BinarySensor *sensor, bool value) {
  if (sensor == nullptr)
    return;
  if (sensor->has_state() && sensor->state == value)
    return;
  sensor->publish_state(value);
}

// -----------------------------------------------------------------------
// Frame type handlers
//
// CAVEAT: tail1/tail2 do not behave like a checksum of the other 8
// bytes -- identical 8-byte payloads have been observed with different
// tail bytes, ruling out every common 8-bit sum/XOR/CRC-8 scheme. Until
// that's nailed down, treat status_raw / heartbeat_tick as RAW
// diagnostic values, not calibrated readings. setpoint_celsius is the
// one exception -- it's confirmed directly against a controlled button
// test, see handle_setpoint_event_() below.
// -----------------------------------------------------------------------

void MegmeetUART::handle_heartbeat_(const Frame &frame) {
  ESP_LOGD(TAG, "HEARTBEAT  %s", frame_to_hex_(frame).c_str());
  publish_text_(last_heartbeat_frame_sensor_, frame_to_hex_(frame));
  publish_val_(heartbeat_tick_sensor_, frame.tail2);
}

void MegmeetUART::handle_status_(const Frame &frame) {
  ESP_LOGD(TAG, "STATUS     %s", frame_to_hex_(frame).c_str());
  publish_text_(last_status_frame_sensor_, frame_to_hex_(frame));
  publish_val_(status_raw_sensor_, frame.tail1);

  // Presence of STATUS traffic at all -- not any particular value -- is
  // the confirmed running signal (see the block at the top of the .h
  // file; a value-based version of this was tried and disproven). Any
  // STATUS frame arriving means the AC is running, immediately.
  status_ever_seen_ = true;
  last_status_millis_ = millis();
  publish_bin_(ac_running_sensor_, true);
  if (climate_ != nullptr)
    climate_->update_power_state(true);
}

void MegmeetUART::handle_control_(const Frame &frame) {
  ESP_LOGD(TAG, "CONTROL    %s", frame_to_hex_(frame).c_str());
  publish_text_(last_control_frame_sensor_, frame_to_hex_(frame));

  // No numeric sensor is derived from tail1 here -- it's confirmed NOT
  // to be the setpoint (see the block at the top of the .h file), and
  // what it actually represents isn't nailed down yet. Kept around only
  // as the (experimental, unconfirmed) basis for replay_last_control_frame()/
  // adjust_control_setpoint() below.
  last_control_frame_ = frame;
  control_frame_valid_ = true;
}

void MegmeetUART::handle_setpoint_event_(const Frame &frame) {
  if (frame.sub_id != SUB_ID_MAIN) {
    ESP_LOGW(TAG, "type 0x%02X seen with unexpected sub_id 0x%02X (expected 0x%02X) -- treating as unknown: %s",
             frame.type, frame.sub_id, SUB_ID_MAIN, frame_to_hex_(frame).c_str());
    unknown_count_++;
    handle_unknown_(frame);
    return;
  }

  ESP_LOGI(TAG, "SETPOINT   %s  (tail1 = %d C)", frame_to_hex_(frame).c_str(), frame.tail1);
  publish_val_(setpoint_celsius_sensor_, frame.tail1);
  if (climate_ != nullptr)
    climate_->update_setpoint(frame.tail1);

  last_setpoint_event_frame_ = frame;
  setpoint_event_valid_ = true;
}

void MegmeetUART::handle_power_event_(const Frame &frame) {
  if (frame.sub_id != SUB_ID_EVENT) {
    ESP_LOGW(TAG, "type 0x%02X seen with unexpected sub_id 0x%02X (expected 0x%02X) -- treating as unknown: %s",
             frame.type, frame.sub_id, SUB_ID_EVENT, frame_to_hex_(frame).c_str());
    unknown_count_++;
    handle_unknown_(frame);
    return;
  }
  bool on_requested = frame.data3 == 0x0F;
  ESP_LOGI(TAG, "POWER      %s  (%s requested)", frame_to_hex_(frame).c_str(), on_requested ? "ON" : "OFF");
  publish_bin_(power_requested_sensor_, on_requested);
}

void MegmeetUART::handle_fan_event_(const Frame &frame) {
  if (frame.sub_id != SUB_ID_EVENT) {
    ESP_LOGW(TAG, "type 0x%02X seen with unexpected sub_id 0x%02X (expected 0x%02X) -- treating as unknown: %s",
             frame.type, frame.sub_id, SUB_ID_EVENT, frame_to_hex_(frame).c_str());
    unknown_count_++;
    handle_unknown_(frame);
    return;
  }
  // Raw hex only -- the speed-name mapping isn't confirmed yet, see the
  // block at the top of the .h file. Don't infer a speed here.
  ESP_LOGI(TAG, "FAN EVENT  %s", frame_to_hex_(frame).c_str());
  publish_text_(last_fan_event_frame_sensor_, frame_to_hex_(frame));

  if (climate_ != nullptr) {
    if (frame.type == TYPE_FAN_EVENT_LOW) {
      climate_->update_fan_toggle();
    } else {
      climate_->update_fan_raw(frame.data3);
    }
  }
}

void MegmeetUART::handle_mode_event_(const Frame &frame) {
  if (frame.sub_id != SUB_ID_EVENT) {
    ESP_LOGW(TAG, "type 0x%02X seen with unexpected sub_id 0x%02X (expected 0x%02X) -- treating as unknown: %s",
             frame.type, frame.sub_id, SUB_ID_EVENT, frame_to_hex_(frame).c_str());
    unknown_count_++;
    handle_unknown_(frame);
    return;
  }
  // Raw hex only -- the mode-name mapping isn't confirmed yet, see the
  // block at the top of the .h file. Presence of this frame at all is
  // the reliable part (mode change accepted); absence after a mode
  // button press means the mainboard rejected it.
  ESP_LOGI(TAG, "MODE EVENT %s", frame_to_hex_(frame).c_str());
  publish_text_(last_mode_event_frame_sensor_, frame_to_hex_(frame));
  if (climate_ != nullptr)
    climate_->update_mode_raw(frame.data3);
}

void MegmeetUART::handle_query_(const Frame &frame) { ESP_LOGD(TAG, "QUERY      %s", frame_to_hex_(frame).c_str()); }

void MegmeetUART::handle_unknown_(const Frame &frame) {
  ESP_LOGI(TAG, "UNKNOWN    %s  (type=0x%02X, %" PRIu32 " unknown frames seen)", frame_to_hex_(frame).c_str(),
           frame.type, unknown_count_);
  publish_text_(last_unknown_frame_sensor_, frame_to_hex_(frame));
}

// -----------------------------------------------------------------------
// Change-diff logger
// -----------------------------------------------------------------------

void MegmeetUART::compare_frame_(const Frame &frame) {
  PacketState &old = packet_db_[frame.type];

  if (!old.valid) {
    old.valid = true;
    old.frame = frame;
    return;
  }

  bool changed = frame.sub_id != old.frame.sub_id || frame.data1 != old.frame.data1 ||
                 frame.data2 != old.frame.data2 || frame.data3 != old.frame.data3 || frame.tail1 != old.frame.tail1;

  if (!changed)
    return;

  ESP_LOGD(TAG, "type 0x%02X payload changed: %s -> %s", frame.type, frame_to_hex_(old.frame).c_str(),
           frame_to_hex_(frame).c_str());

  old.frame = frame;
}

// -----------------------------------------------------------------------
// Experimental TX (see the big caveat block at the top of the .h file)
// -----------------------------------------------------------------------

void MegmeetUART::transmit_frame_(const Frame &f) {
  uint8_t buf[10] = {f.header1, f.header2, f.len, f.sub_id, f.type, f.data1, f.data2, f.data3, f.tail1, f.tail2};
  ESP_LOGW(TAG, "TX (experimental, unconfirmed protocol): %s", frame_to_hex_(f).c_str());
  this->write_array(buf, sizeof(buf));
}

void MegmeetUART::replay_last_control_frame() {
  if (!control_frame_valid_) {
    ESP_LOGW(TAG, "No CONTROL frame captured yet -- nothing to replay. Wait for the unit to send one first.");
    return;
  }
  transmit_frame_(last_control_frame_);
}

void MegmeetUART::adjust_control_setpoint(int delta_steps) {
  if (!control_frame_valid_) {
    ESP_LOGW(TAG, "No CONTROL frame captured yet -- nothing to base the change on.");
    return;
  }

  Frame f = last_control_frame_;
  int new_tail1 = static_cast<int>(f.tail1) + delta_steps * 2;

  if (new_tail1 < 0 || new_tail1 > 255) {
    ESP_LOGW(TAG, "Setpoint step out of byte range (%d), ignoring", new_tail1);
    return;
  }

  f.tail1 = static_cast<uint8_t>(new_tail1);
  // tail2 intentionally left as whatever was captured alongside the
  // OLD tail1 -- we have no formula to recompute it for the new value.
  // This is the actual experiment: if the AC reacts anyway, tail2
  // either isn't checked, or isn't a function of tail1 the way we
  // assumed. If nothing happens, that's evidence tail2 does matter.
  transmit_frame_(f);
}

void MegmeetUART::replay_last_setpoint_event() {
  if (!setpoint_event_valid_) {
    ESP_LOGW(TAG, "No SETPOINT_EVENT frame captured yet -- nothing to replay. Change the temperature "
                  "once from the remote first so there's a real frame to copy.");
    return;
  }
  transmit_frame_(last_setpoint_event_frame_);
}

void MegmeetUART::adjust_setpoint_event(int delta_degrees) {
  if (!setpoint_event_valid_) {
    ESP_LOGW(TAG, "No SETPOINT_EVENT frame captured yet -- nothing to base the change on.");
    return;
  }

  Frame f = last_setpoint_event_frame_;
  int new_tail1 = static_cast<int>(f.tail1) + delta_degrees;

  if (new_tail1 < 0 || new_tail1 > 255) {
    ESP_LOGW(TAG, "Setpoint step out of byte range (%d), ignoring", new_tail1);
    return;
  }

  f.tail1 = static_cast<uint8_t>(new_tail1);
  // Same open question on tail2 as adjust_control_setpoint() above.
  transmit_frame_(f);
}

void MegmeetUART::send_tuya_power_probe(bool on) {
  // Built directly from Tuya's public MCU serial protocol spec:
  //   55 AA | VER | CMD=0x06 (send command) | LEN(2, big-endian) |
  //   [DPID | TYPE=0x01 (bool) | LEN=0x0001 | VALUE] | CHECKSUM
  // Checksum = sum of every preceding byte, mod 256 (Tuya's algorithm,
  // NOT the same as anything used elsewhere in this component -- our
  // real frames were already confirmed not to use simple additive
  // checksums, so this is deliberately a different, Tuya-native calc).
  // DP1 = power switch is the conventional assignment for Tuya's AC
  // product category, NOT confirmed for this specific unit. See the
  // caveat block at the top of the .h file before drawing conclusions
  // from the result either way.
  uint8_t value = on ? 0x01 : 0x00;
  uint8_t buf[12] = {0x55, 0xAA, 0x00, 0x06, 0x00, 0x05, 0x01, 0x01, 0x00, 0x01, value, 0x00};

  uint16_t sum = 0;
  for (int i = 0; i < 11; i++)
    sum += buf[i];
  buf[11] = static_cast<uint8_t>(sum & 0xFF);

  char hex[40];
  snprintf(hex, sizeof(hex), "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", buf[0], buf[1], buf[2],
           buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11]);
  ESP_LOGW(TAG, "TX (Tuya protocol probe, speculative -- see .h file caveat): %s", hex);
  this->write_array(buf, sizeof(buf));
}

void MegmeetUART::send_short_setpoint_probe() {
  if (!setpoint_event_valid_) {
    ESP_LOGW(TAG, "No SETPOINT_EVENT frame captured yet -- nothing to base this on.");
    return;
  }

  // 55 35 04 08 20 [current setpoint] [tail2] -- len correctly set to 4
  // for this shorter shape (sub_id+type+tail1+tail2, no data1/data2/data3
  // padding). tail1/tail2 copied from the last real captured frame --
  // null-hypothesis value, tail2 is a guess, see the caveat in the .h
  // file declaration.
  uint8_t buf[7] = {0x55,
                     0x35,
                     0x04,
                     SUB_ID_MAIN,
                     TYPE_SETPOINT_EVENT,
                     last_setpoint_event_frame_.tail1,
                     last_setpoint_event_frame_.tail2};

  char hex[24];
  snprintf(hex, sizeof(hex), "%02X %02X %02X %02X %02X %02X %02X", buf[0], buf[1], buf[2], buf[3], buf[4], buf[5],
           buf[6]);
  ESP_LOGW(TAG, "TX (short-frame probe, speculative -- see .h file caveat): %s", hex);
  this->write_array(buf, sizeof(buf));
}

void MegmeetUART::send_tuya_temp_probe() {
  if (!setpoint_event_valid_) {
    ESP_LOGW(TAG, "No SETPOINT_EVENT frame captured yet -- nothing to base this on.");
    return;
  }

  // Tuya "kt" category schema, confirmed live from Tuya's own dev docs:
  //   DP2 = temp_set, type Integer, scale=10 -> wire value = degrees C * 10,
  //   4 bytes big-endian.
  //   55 AA | VER=00 | CMD=0x06 | LEN(2) |
  //   [DPID=02 | TYPE=0x02 (integer) | LEN=0004 | VALUE(4, big-endian)] |
  //   CHECKSUM
  uint32_t value = static_cast<uint32_t>(last_setpoint_event_frame_.tail1) * 10;
  uint8_t buf[15] = {0x55,
                      0xAA,
                      0x00,
                      0x06,
                      0x00,
                      0x08,
                      0x02,
                      0x02,
                      0x00,
                      0x04,
                      static_cast<uint8_t>((value >> 24) & 0xFF),
                      static_cast<uint8_t>((value >> 16) & 0xFF),
                      static_cast<uint8_t>((value >> 8) & 0xFF),
                      static_cast<uint8_t>(value & 0xFF),
                      0x00};

  uint16_t sum = 0;
  for (int i = 0; i < 14; i++)
    sum += buf[i];
  buf[14] = static_cast<uint8_t>(sum & 0xFF);

  char hex[52];
  snprintf(hex, sizeof(hex), "%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X", buf[0],
           buf[1], buf[2], buf[3], buf[4], buf[5], buf[6], buf[7], buf[8], buf[9], buf[10], buf[11], buf[12],
           buf[13], buf[14]);
  ESP_LOGW(TAG, "TX (Tuya DP2 temp_set probe, speculative -- see .h file caveat): %s", hex);
  this->write_array(buf, sizeof(buf));
}

void MegmeetUART::send_direction_flag_probe() {
  if (!setpoint_event_valid_) {
    ESP_LOGW(TAG, "No SETPOINT_EVENT frame captured yet -- nothing to base this on.");
    return;
  }

  // Byte-for-byte copy of the real captured frame, EXCEPT sub_id has its
  // high bit set (0x08 -> 0x88). Single-variable test of the "high bit =
  // module origin" hypothesis borrowed from AUX_HVAC_Protocol -- see the
  // caveat in the .h file declaration.
  Frame f = last_setpoint_event_frame_;
  f.sub_id |= 0x80;
  transmit_frame_(f);
}

}  // namespace megmeet_uart
}  // namespace esphome
