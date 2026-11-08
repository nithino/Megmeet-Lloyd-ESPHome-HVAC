#include "megmeet_tuya.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace megmeet_tuya {

static const char *const TAG = "megmeet_tuya";
static const uint16_t MAX_DATA_LEN = 256;

void MegmeetTuya::dump_config() {
  ESP_LOGCONFIG(TAG, "Megmeet Tuya-variant UART:");
  ESP_LOGCONFIG(TAG, "  Heartbeat interval: %u ms (0 = off)", (unsigned) heartbeat_interval_);
  ESP_LOGCONFIG(TAG, "  Link timeout: %u ms", (unsigned) link_timeout_);
  for (auto &e : sensors_)
    ESP_LOGCONFIG(TAG, "  DP %u -> '%s' (x%.3f %+.2f)", e.dp, e.sensor->get_name().c_str(), e.multiply, e.offset);
  LOG_BINARY_SENSOR("  ", "Link", link_sensor_);
}

void MegmeetTuya::register_sensor(uint8_t dp, sensor::Sensor *s, float multiply, float offset) {
  sensors_.push_back({dp, s, multiply, offset});
}

void MegmeetTuya::loop() {
  const uint32_t now = millis();

  while (this->available()) {
    uint8_t b;
    if (!this->read_byte(&b))
      break;
    rx_.push_back(b);
  }

  while (true) {
    // Drop bytes until the buffer starts with 55 AA (keep a lone trailing 0x55).
    size_t i = 0;
    while (i < rx_.size()) {
      if (rx_[i] == 0x55 && (i + 1 == rx_.size() || rx_[i + 1] == 0xAA))
        break;
      i++;
    }
    if (i > 0)
      rx_.erase(rx_.begin(), rx_.begin() + i);
    if (rx_.size() < 7)
      break;
    uint16_t len = (uint16_t(rx_[4]) << 8) | rx_[5];
    if (len > MAX_DATA_LEN) {
      frames_bad_++;
      rx_.erase(rx_.begin(), rx_.begin() + 2);
      continue;
    }
    size_t total = 7u + len;
    if (rx_.size() < total)
      break;
    uint8_t sum = 0;
    for (size_t k = 0; k < total - 1; k++)
      sum += rx_[k];
    if (sum != rx_[total - 1]) {
      frames_bad_++;
      ESP_LOGV(TAG, "checksum mismatch (got %02X want %02X)", rx_[total - 1], sum);
      rx_.erase(rx_.begin(), rx_.begin() + 2);
      continue;
    }
    frames_ok_++;
    last_valid_frame_ = now;
    ever_valid_ = true;
    handle_frame_(rx_[2], rx_[3], rx_.data() + 6, len);
    rx_.erase(rx_.begin(), rx_.begin() + total);
  }
  if (rx_.size() > 1024)
    rx_.clear();

  if (link_sensor_ != nullptr) {
    bool up = ever_valid_ && (now - last_valid_frame_ < link_timeout_);
    if (!link_sensor_->has_state() || link_sensor_->state != up)
      link_sensor_->publish_state(up);
  }

  if (heartbeat_interval_ > 0 && now - last_heartbeat_ >= heartbeat_interval_) {
    last_heartbeat_ = now;
    send_command(0x00, {});
  }

  if (now - last_stats_ >= 600000) {  // one quiet line every 10 min
    last_stats_ = now;
    ESP_LOGD(TAG, "frames ok=%u bad=%u, %u DPs known", (unsigned) frames_ok_, (unsigned) frames_bad_,
             (unsigned) dp_values_.size());
  }
}

void MegmeetTuya::handle_frame_(uint8_t version, uint8_t cmd, const uint8_t *data, uint16_t len) {
  if (cmd == 0x07 || cmd == 0x22) {  // DP status report (and its "record" variant)
    handle_dps_(data, len);
    return;
  }
  // Anything that is not a status report is interesting: log it at INFO.
  ESP_LOGI(TAG, "RX ver=%02X cmd=%02X len=%u data=%s", version, cmd, len, format_hex_pretty(data, len).c_str());
}

void MegmeetTuya::handle_dps_(const uint8_t *data, uint16_t len) {
  uint16_t pos = 0;
  while (pos + 4 <= len) {
    uint8_t dp = data[pos];
    uint8_t type = data[pos + 1];
    uint16_t dlen = (uint16_t(data[pos + 2]) << 8) | data[pos + 3];
    pos += 4;
    if (pos + dlen > len) {
      ESP_LOGW(TAG, "truncated DP %u", dp);
      return;
    }
    const uint8_t *v = data + pos;
    pos += dlen;
    int32_t value = 0;
    if (type == DP_BOOL || type == DP_ENUM) {
      value = dlen >= 1 ? v[0] : 0;
    } else if (type == DP_VALUE || type == DP_BITMAP) {
      for (uint16_t k = 0; k < dlen && k < 4; k++)
        value = int32_t((uint32_t(value) << 8) | v[k]);
    } else {
      ESP_LOGI(TAG, "DP %u type %u (len %u): %s", dp, type, dlen, format_hex_pretty(v, dlen).c_str());
      continue;
    }
    publish_dp_(dp, value);
  }
}

void MegmeetTuya::publish_dp_(uint8_t dp, int32_t value) {
  auto it = dp_values_.find(dp);
  bool is_new = it == dp_values_.end();
  if (is_new) {
    ESP_LOGI(TAG, "first report of DP %u = %d", dp, (int) value);
  } else if (it->second != value) {
    ESP_LOGV(TAG, "DP %u = %d", dp, (int) value);
  }
  dp_values_[dp] = value;
  for (auto &e : sensors_) {
    if (e.dp != dp)
      continue;
    float f = value * e.multiply + e.offset;
    if (!e.sensor->has_state() || e.sensor->raw_state != f)
      e.sensor->publish_state(f);
  }
}

bool MegmeetTuya::get_dp(uint8_t dp, int32_t *out) const {
  auto it = dp_values_.find(dp);
  if (it == dp_values_.end())
    return false;
  *out = it->second;
  return true;
}

void MegmeetTuya::send_command(uint8_t cmd, const std::vector<uint8_t> &payload, uint8_t version) {
  std::vector<uint8_t> f = {0x55, 0xAA, version, cmd, uint8_t(payload.size() >> 8), uint8_t(payload.size() & 0xFF)};
  f.insert(f.end(), payload.begin(), payload.end());
  uint8_t sum = 0;
  for (auto b : f)
    sum += b;
  f.push_back(sum);
  send_raw(f);
}

void MegmeetTuya::set_dp_bool(uint8_t dp, bool value, uint8_t version) {
  send_command(0x06, {dp, DP_BOOL, 0x00, 0x01, uint8_t(value ? 1 : 0)}, version);
}

void MegmeetTuya::set_dp_enum(uint8_t dp, uint8_t value, uint8_t version) {
  send_command(0x06, {dp, DP_ENUM, 0x00, 0x01, value}, version);
}

void MegmeetTuya::set_dp_value(uint8_t dp, int32_t value, uint8_t version) {
  uint32_t u = (uint32_t) value;
  send_command(0x06, {dp, DP_VALUE, 0x00, 0x04, uint8_t(u >> 24), uint8_t(u >> 16), uint8_t(u >> 8), uint8_t(u)},
               version);
}

void MegmeetTuya::send_raw(const std::vector<uint8_t> &bytes) {
  ESP_LOGD(TAG, "TX %s", format_hex_pretty(bytes).c_str());
  this->write_array(bytes);
}

void MegmeetTuya::set_tx_inverted(uint8_t tx_gpio, bool inverted) {
#if defined(USE_ESP32_VARIANT_ESP32C3)
  // GPIO_FUNCn_OUT_SEL_CFG_REG (base 0x60004554), bit 8 = OUT_INV_SEL. Only meaningful when the UART TX
  // signal is routed through the GPIO matrix (true for a non-default TX pin such as GPIO20).
  if (tx_gpio > 21)
    return;
  volatile uint32_t *reg = (volatile uint32_t *) (0x60004554 + 4 * tx_gpio);
  uint32_t v = *reg;
  *reg = inverted ? (v | (1u << 8)) : (v & ~(1u << 8));
  ESP_LOGW(TAG, "TX on GPIO%u %s (OUT_SEL_CFG=0x%08X)", tx_gpio, inverted ? "INVERTED" : "normal", (unsigned) *reg);
#else
  ESP_LOGW(TAG, "set_tx_inverted only implemented for ESP32-C3");
#endif
}

}  // namespace megmeet_tuya
}  // namespace esphome
