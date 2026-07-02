#include "megmeet_uart.h"
#include <cinttypes>
#include <cstdio>

namespace esphome {
namespace megmeet_uart {

static const char *const TAG = "megmeet_uart";

void MegmeetUART::setup() { ESP_LOGCONFIG(TAG, "Setting up Megmeet UART..."); }

void MegmeetUART::dump_config() {
  ESP_LOGCONFIG(TAG, "Megmeet UART:");
  LOG_SENSOR("  ", "Status Raw", this->status_raw_sensor_);
  LOG_SENSOR("  ", "Control Setpoint Raw", this->control_setpoint_raw_sensor_);
  LOG_SENSOR("  ", "Heartbeat Tick", this->heartbeat_tick_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Status Frame", this->last_status_frame_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Control Frame", this->last_control_frame_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Heartbeat Frame", this->last_heartbeat_frame_sensor_);
  LOG_TEXT_SENSOR("  ", "Last Unknown Frame", this->last_unknown_frame_sensor_);
}

// -----------------------------------------------------------------------
// Byte stream -> frames
// -----------------------------------------------------------------------

void MegmeetUART::loop() {
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

// -----------------------------------------------------------------------
// Frame type handlers
//
// CAVEAT: tail1/tail2 do not behave like a checksum of the other 8
// bytes -- identical 8-byte payloads have been observed with different
// tail bytes, ruling out every common 8-bit sum/XOR/CRC-8 scheme. Until
// that's nailed down, treat status_raw / control_setpoint_raw /
// heartbeat_tick as RAW diagnostic values, not calibrated readings.
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
}

void MegmeetUART::handle_control_(const Frame &frame) {
  ESP_LOGD(TAG, "CONTROL    %s", frame_to_hex_(frame).c_str());
  publish_text_(last_control_frame_sensor_, frame_to_hex_(frame));
  publish_val_(control_setpoint_raw_sensor_, frame.tail1);

  // Keep a raw copy around for the button platform to replay/modify.
  // Only overwritten by frames we actually received from the mainboard,
  // never by anything we transmitted ourselves.
  last_control_frame_ = frame;
  control_frame_valid_ = true;
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

}  // namespace megmeet_uart
}  // namespace esphome
