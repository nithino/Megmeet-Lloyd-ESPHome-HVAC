#include "esphome/core/defines.h"
#ifdef USE_CLIMATE
#include "megmeet_climate.h"
#include <cmath>
#include "esphome/core/log.h"

namespace esphome {
namespace megmeet_tuya {

static const char *const TAG = "megmeet_tuya.climate";

void MegmeetClimate::setup() {
  if (!custom_fans_.empty()) {
    for (auto &c : custom_fans_)
      custom_fan_names_.push_back(c.name);
    this->set_supported_custom_fan_modes(custom_fan_names_);
  }
  this->mode = climate::CLIMATE_MODE_OFF;

  parent_->add_dp_listener(power_dp_, [this](int32_t v) {
    have_power_ = true;
    power_ = v != 0;
    update_mode_();
    schedule_publish_();
  });
  parent_->add_dp_listener(mode_dp_, [this](int32_t v) {
    mode_raw_ = v;
    update_mode_();
    schedule_publish_();
  });
  parent_->add_dp_listener(target_dp_, [this](int32_t v) {
    this->target_temperature = v * temp_mult_;
    schedule_publish_();
  });
  if (current_dp_ >= 0) {
    parent_->add_dp_listener(current_dp_, [this](int32_t v) {
      float t = v * temp_mult_;
      if (std::isnan(this->current_temperature) || this->current_temperature != t) {
        this->current_temperature = t;
        schedule_publish_();
      }
    });
  }
  if (fan_dp_ >= 0) {
    parent_->add_dp_listener(fan_dp_, [this](int32_t v) {
      for (auto &f : fans_) {
        if (f.raw == v) {
          this->set_fan_mode_(f.fan);
          schedule_publish_();
          return;
        }
      }
      for (auto &c : custom_fans_) {
        if (c.raw == v) {
          this->set_custom_fan_mode_(c.name);
          schedule_publish_();
          return;
        }
      }
      ESP_LOGW(TAG, "Unmapped fan value %d", (int) v);
    });
  }
  auto swing_cb = [this]() {
    if (swing_v_ && swing_h_)
      this->swing_mode = climate::CLIMATE_SWING_BOTH;
    else if (swing_v_)
      this->swing_mode = climate::CLIMATE_SWING_VERTICAL;
    else if (swing_h_)
      this->swing_mode = climate::CLIMATE_SWING_HORIZONTAL;
    else
      this->swing_mode = climate::CLIMATE_SWING_OFF;
    schedule_publish_();
  };
  if (swing_v_dp_ >= 0)
    parent_->add_dp_listener(swing_v_dp_, [this, swing_cb](int32_t v) {
      swing_v_ = v != 0;
      swing_cb();
    });
  if (swing_h_dp_ >= 0)
    parent_->add_dp_listener(swing_h_dp_, [this, swing_cb](int32_t v) {
      swing_h_ = v != 0;
      swing_cb();
    });
}

void MegmeetClimate::update_mode_() {
  if (!have_power_ || !power_) {
    this->mode = climate::CLIMATE_MODE_OFF;
    return;
  }
  for (auto &m : modes_) {
    if (m.raw == mode_raw_) {
      this->mode = m.mode;
      return;
    }
  }
  if (mode_raw_ >= 0)
    ESP_LOGW(TAG, "Unmapped mode value %d", (int) mode_raw_);
}

void MegmeetClimate::schedule_publish_() {
  // Query answers and write echoes arrive as bursts of DPs: publish once per burst.
  this->set_timeout("publish", 60, [this]() { this->publish_state(); });
}

climate::ClimateTraits MegmeetClimate::traits() {
  auto traits = climate::ClimateTraits();
  if (current_dp_ >= 0)
    traits.add_feature_flags(climate::CLIMATE_SUPPORTS_CURRENT_TEMPERATURE);
  traits.add_supported_mode(climate::CLIMATE_MODE_OFF);
  for (auto &m : modes_)
    traits.add_supported_mode(m.mode);
  for (auto &f : fans_)
    traits.add_supported_fan_mode(f.fan);
  if (swing_v_dp_ >= 0 || swing_h_dp_ >= 0) {
    traits.add_supported_swing_mode(climate::CLIMATE_SWING_OFF);
    if (swing_v_dp_ >= 0)
      traits.add_supported_swing_mode(climate::CLIMATE_SWING_VERTICAL);
    if (swing_h_dp_ >= 0)
      traits.add_supported_swing_mode(climate::CLIMATE_SWING_HORIZONTAL);
    if (swing_v_dp_ >= 0 && swing_h_dp_ >= 0)
      traits.add_supported_swing_mode(climate::CLIMATE_SWING_BOTH);
  }
  return traits;
}

void MegmeetClimate::control(const climate::ClimateCall &call) {
  if (call.get_mode().has_value()) {
    auto m = *call.get_mode();
    if (m == climate::CLIMATE_MODE_OFF) {
      parent_->write_dp(power_dp_, 0, DP_BOOL);
    } else {
      for (auto &mm : modes_) {
        if (mm.mode != m)
          continue;
        // Set the mode first so the AC never starts briefly in the old mode.
        if (mode_raw_ != mm.raw)
          parent_->write_dp(mode_dp_, mm.raw, DP_ENUM);
        if (!power_)
          parent_->write_dp(power_dp_, 1, DP_BOOL);
        break;
      }
    }
  }
  if (call.get_target_temperature().has_value()) {
    float t = *call.get_target_temperature();
    parent_->write_dp(target_dp_, (int32_t) lroundf(t / temp_mult_), DP_VALUE);
  }
  if (fan_dp_ >= 0) {
    if (call.get_fan_mode().has_value()) {
      for (auto &f : fans_)
        if (f.fan == *call.get_fan_mode())
          parent_->write_dp(fan_dp_, f.raw, DP_ENUM);
    } else if (call.has_custom_fan_mode()) {
      auto name = call.get_custom_fan_mode();
      for (auto &c : custom_fans_)
        if (name == c.name)
          parent_->write_dp(fan_dp_, c.raw, DP_ENUM);
    }
  }
  if (call.get_swing_mode().has_value()) {
    auto s = *call.get_swing_mode();
    bool v = s == climate::CLIMATE_SWING_VERTICAL || s == climate::CLIMATE_SWING_BOTH;
    bool h = s == climate::CLIMATE_SWING_HORIZONTAL || s == climate::CLIMATE_SWING_BOTH;
    if (swing_v_dp_ >= 0)
      parent_->write_dp(swing_v_dp_, v ? 1 : 0, DP_BOOL);
    if (swing_h_dp_ >= 0)
      parent_->write_dp(swing_h_dp_, h ? 1 : 0, DP_BOOL);
  }
  // No optimistic publish: the entity updates when the AC echoes the accepted values.
}

void MegmeetClimate::dump_config() {
  LOG_CLIMATE("", "Megmeet AC", this);
  ESP_LOGCONFIG(TAG, "  DPs: power %u, target %u, mode %u, fan %d, current %d, swing v %d / h %d", power_dp_,
                target_dp_, mode_dp_, fan_dp_, current_dp_, swing_v_dp_, swing_h_dp_);
}

}  // namespace megmeet_tuya
}  // namespace esphome

#endif  // USE_CLIMATE
