#pragma once

#include "esphome/core/component.h"
#include "esphome/components/climate/climate.h"
#include "esphome/components/sensor/sensor.h"

namespace esphome {
namespace megmeet_uart {

// READ-ONLY for now. Interprets and reports the AC's real state as seen
// on the wire; does NOT send anything to the AC. control() deliberately
// does nothing but log a warning -- every UART write experiment tried
// so far (see megmeet_uart.h) has come back negative, and IR-based
// control is the planned next step, not yet built. Until that lands,
// this entity will show a stale/reverted value in Home Assistant if you
// try to change it there, which is the honest behavior for a channel
// that doesn't exist yet -- don't be tempted to fake a write path here.
//
// Confidence levels vary a lot between the fields reported:
//  - power (on/off) and target_temperature: HIGH. Confirmed via
//    controlled tests -- see the caveat blocks in megmeet_uart.h.
//  - mode: LOW-MEDIUM. Dry/Fan/Auto/Cool have each been captured once
//    via MODE_EVENT (0x10): 0xA0/0x10/0x14/0x15 respectively. Cool's
//    byte (0x15) took longer to find -- every earlier test that ended
//    on Cool did so via a fan-family side-effect frame instead of a
//    real MODE_EVENT (the Heat-rejection quirk documented in
//    megmeet_uart.h), so it went unmapped until a direct Auto->Cool
//    switch was captured. Heat is confirmed UNSUPPORTED by this
//    specific unit (no beep, no reaction) and deliberately left out of
//    supported_modes below.
//  - fan mode: LOW-MEDIUM, same one-sample caveat for Medium/High/Auto/Low
//    (from type=0x41), PLUS a genuine, structural ambiguity: the
//    low<->low-medium transition uses a single toggle marker (type
//    0x2B) that is byte-for-byte IDENTICAL regardless of which
//    direction it's toggling -- there is no way to tell which of the
//    two it landed on from the frame alone. Rather than silently guess
//    one, that case is surfaced as an explicit custom fan mode string
//    so the ambiguity is visible in Home Assistant instead of hidden.
//
// None of the above will be known at all until the first relevant event
// fires each session (same boot-time gap already documented for
// setpoint_celsius in megmeet_uart.h) -- e.g. mode will show whatever
// Home Assistant defaults an unset ClimateMode to until a real
// MODE_EVENT (or a power-on restoring a previously seen one) arrives.
class MegmeetClimate : public climate::Climate, public Component {
 public:
  void setup() override;
  void dump_config() override;

  // Optional -- this AC's own protocol has no confirmed current-room-
  // temperature field (only target/setpoint, see megmeet_uart.h). If
  // set, current_temperature is mirrored from an external sensor (e.g.
  // a `homeassistant:` platform sensor pulling in a separate thermostat/
  // smart-plug-adjacent temperature reading) instead of coming from the
  // AC's own traffic. Same wiring pattern as ESPHome's own pid/thermostat
  // climate components (add_on_state_callback + immediate initial read).
  void set_current_temperature_sensor(sensor::Sensor *sensor) { current_temperature_sensor_ = sensor; }

  // Called by MegmeetUART's frame handlers as real data arrives on the
  // wire -- not meant to be called from anywhere else.
  void update_power_state(bool ac_on);
  void update_setpoint(uint8_t celsius);
  void update_mode_raw(uint8_t raw_byte);
  void update_fan_raw(uint8_t raw_byte);
  void update_fan_toggle();

 protected:
  climate::ClimateTraits traits() override;
  void control(const climate::ClimateCall &call) override;

  climate::ClimateMode last_known_mode_{climate::CLIMATE_MODE_COOL};
  bool have_mode_{false};

  sensor::Sensor *current_temperature_sensor_{nullptr};
};

}  // namespace megmeet_uart
}  // namespace esphome
