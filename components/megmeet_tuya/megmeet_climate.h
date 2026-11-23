#pragma once

// "Master AC" climate entity on top of MegmeetTuya. All state comes from the AC reports (cmd 07),
// so changes made with the IR remote are followed too. Writes are not optimistic: the AC echoes
// an accepted write within ~15 ms and that echo updates the entity; rejected values are not shown.

#include "esphome/core/defines.h"
#ifdef USE_CLIMATE
#include <map>
#include <vector>

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "megmeet_tuya.h"

namespace esphome {
namespace megmeet_tuya {

class MegmeetClimate : public climate::Climate, public Component {
 public:
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA - 1.0f; }

  void set_parent(MegmeetTuya *p) { parent_ = p; }
  void set_power_dp(uint8_t dp) { power_dp_ = dp; }
  void set_target_temperature_dp(uint8_t dp) { target_dp_ = dp; }
  void set_current_temperature_dp(int dp) { current_dp_ = dp; }
  void set_temperature_multiplier(float m) { temp_mult_ = m; }
  void set_mode_dp(uint8_t dp) { mode_dp_ = dp; }
  void set_fan_dp(int dp) { fan_dp_ = dp; }
  void set_swing_vertical_dp(int dp) { swing_v_dp_ = dp; }
  void set_swing_horizontal_dp(int dp) { swing_h_dp_ = dp; }
  void add_mode(climate::ClimateMode m, uint8_t raw) { modes_.push_back({m, raw}); }
  void add_fan_mode(climate::ClimateFanMode m, uint8_t raw) { fans_.push_back({m, raw}); }
  void add_custom_fan_mode(const char *name, uint8_t raw) { custom_fans_.push_back({name, raw}); }

 protected:
  void control(const climate::ClimateCall &call) override;
  climate::ClimateTraits traits() override;
  void update_mode_();
  void schedule_publish_();

  struct ModeMap {
    climate::ClimateMode mode;
    uint8_t raw;
  };
  struct FanMap {
    climate::ClimateFanMode fan;
    uint8_t raw;
  };
  struct CustomFanMap {
    const char *name;
    uint8_t raw;
  };

  MegmeetTuya *parent_{nullptr};
  uint8_t power_dp_{1};
  uint8_t target_dp_{2};
  int current_dp_{-1};
  float temp_mult_{1.0f};
  uint8_t mode_dp_{4};
  int fan_dp_{-1};
  int swing_v_dp_{-1};
  int swing_h_dp_{-1};
  std::vector<ModeMap> modes_;
  std::vector<FanMap> fans_;
  std::vector<CustomFanMap> custom_fans_;
  std::vector<const char *> custom_fan_names_;

  bool have_power_{false};
  bool power_{false};
  int32_t mode_raw_{-1};
  bool swing_v_{false};
  bool swing_h_{false};
};

}  // namespace megmeet_tuya
}  // namespace esphome

#endif  // USE_CLIMATE
