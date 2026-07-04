#include "megmeet_climate.h"
#include "esphome/core/log.h"

namespace esphome {
namespace megmeet_uart {

static const char *const TAG = "megmeet_uart.climate";

// Kept in sync manually with the class-level caveat comment in the .h
// file -- if you add a real sample for Cool or Heat, update both.
static const char *const CUSTOM_FAN_MODE_LOW_AMBIGUOUS = "Low/Low-Medium (ambiguous)";

void MegmeetClimate::setup() {
  if (this->current_temperature_sensor_ != nullptr) {
    // Same pattern as ESPHome's own pid/thermostat climate components:
    // grab an initial reading immediately (don't wait for the next
    // change), then subscribe for updates.
    if (this->current_temperature_sensor_->has_state())
      this->current_temperature = this->current_temperature_sensor_->state;

    this->current_temperature_sensor_->add_on_state_callback([this](float state) {
      this->current_temperature = state;
      this->publish_state();
    });
  }
}

void MegmeetClimate::dump_config() {
  ESP_LOGCONFIG(TAG, "Megmeet Climate:");
  ESP_LOGCONFIG(TAG, "  Write/control path: NOT IMPLEMENTED YET (read-only interpretation for now, IR planned)");
  ESP_LOGCONFIG(TAG, "  Mode/fan mapping confidence: LOW-MEDIUM, one sample per value -- see megmeet_climate.h");
  LOG_SENSOR("  ", "Current Temperature Sensor (external)", this->current_temperature_sensor_);
}

climate::ClimateTraits MegmeetClimate::traits() {
  auto traits = climate::ClimateTraits();
  traits.set_visual_min_temperature(16);
  traits.set_visual_max_temperature(30);
  traits.set_visual_temperature_step(1);
  // No CLIMATE_MODE_HEAT -- confirmed unsupported by this unit (no beep,
  // no reaction to a real remote press). No current_temperature support
  // declared -- we have no confirmed field for that yet.
  traits.set_supported_modes({
      climate::CLIMATE_MODE_OFF,
      climate::CLIMATE_MODE_COOL,
      climate::CLIMATE_MODE_DRY,
      climate::CLIMATE_MODE_FAN_ONLY,
      climate::CLIMATE_MODE_AUTO,
  });
  traits.set_supported_fan_modes({
      climate::CLIMATE_FAN_LOW,
      climate::CLIMATE_FAN_MEDIUM,
      climate::CLIMATE_FAN_HIGH,
      climate::CLIMATE_FAN_AUTO,
  });
  traits.set_supported_custom_fan_modes({CUSTOM_FAN_MODE_LOW_AMBIGUOUS});
  return traits;
}

void MegmeetClimate::control(const climate::ClimateCall &call) {
  // No write path exists yet -- see the class-level caveat in the .h
  // file. Every UART write experiment tried so far has come back
  // negative; IR-based control is the planned next step. Deliberately
  // NOT calling publish_state() here, so Home Assistant doesn't get told
  // a value took effect when we have no way to confirm the AC accepted
  // anything.
  ESP_LOGW(TAG, "Control requested (mode/temp/fan change) but no write path is implemented yet -- "
                "ignoring. UART write attempts have all come back negative so far; IR integration is "
                "planned but not built.");
}

void MegmeetClimate::update_power_state(bool ac_on) {
  if (!ac_on) {
    this->mode = climate::CLIMATE_MODE_OFF;
  } else if (this->have_mode_) {
    this->mode = this->last_known_mode_;
  } else {
    // No MODE_EVENT has ever been seen this session -- best-effort
    // default guess only. See the class-level caveat in the .h file:
    // Cool's own byte has never actually been captured, this is just
    // the most likely guess given it's the mode used in every idle
    // baseline test so far.
    this->mode = climate::CLIMATE_MODE_COOL;
  }
  this->publish_state();
}

void MegmeetClimate::update_setpoint(uint8_t celsius) {
  this->target_temperature = celsius;
  this->publish_state();
}

void MegmeetClimate::update_mode_raw(uint8_t raw_byte) {
  climate::ClimateMode mapped;
  switch (raw_byte) {
    case 0x15:
      mapped = climate::CLIMATE_MODE_COOL;
      break;
    case 0xA0:
      mapped = climate::CLIMATE_MODE_DRY;
      break;
    case 0x10:
      mapped = climate::CLIMATE_MODE_FAN_ONLY;
      break;
    case 0x14:
      mapped = climate::CLIMATE_MODE_AUTO;
      break;
    default:
      ESP_LOGW(TAG, "Unmapped MODE_EVENT byte 0x%02X -- mapping is one-sample-per-value, see megmeet_climate.h. "
                    "Ignoring, mode left unchanged.",
               raw_byte);
      return;
  }
  this->last_known_mode_ = mapped;
  this->have_mode_ = true;
  this->mode = mapped;
  this->publish_state();
}

void MegmeetClimate::update_fan_raw(uint8_t raw_byte) {
  climate::ClimateFanMode mapped;
  switch (raw_byte) {
    case 0x40:
      mapped = climate::CLIMATE_FAN_MEDIUM;
      break;
    case 0xA0:
      mapped = climate::CLIMATE_FAN_HIGH;
      break;
    case 0x20:
      mapped = climate::CLIMATE_FAN_AUTO;
      break;
    case 0x15:
      mapped = climate::CLIMATE_FAN_LOW;
      break;
    default:
      ESP_LOGW(TAG, "Unmapped FAN_EVENT byte 0x%02X -- mapping is one-sample-per-value, see megmeet_climate.h. "
                    "Ignoring, fan mode left unchanged.",
               raw_byte);
      return;
  }
  this->clear_custom_fan_mode_();
  this->fan_mode = mapped;
  this->publish_state();
}

void MegmeetClimate::update_fan_toggle() {
  // type=0x2B fired: a real low<->low-medium transition happened, but
  // the marker is byte-for-byte identical either direction, so which of
  // the two it landed on is genuinely unknown -- see the class-level
  // caveat in the .h file. Surfaced honestly as its own custom fan mode
  // string instead of guessing one side or leaving the old value shown.
  this->fan_mode.reset();
  this->set_custom_fan_mode_(CUSTOM_FAN_MODE_LOW_AMBIGUOUS);
  this->publish_state();
}

}  // namespace megmeet_uart
}  // namespace esphome
