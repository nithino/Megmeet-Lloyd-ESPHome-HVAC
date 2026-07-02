#pragma once

#include "esphome/components/button/button.h"
#include "megmeet_uart.h"

namespace esphome {
namespace megmeet_uart {

enum class MegmeetButtonAction {
  REPLAY_LAST_CONTROL,
  SETPOINT_STEP_UP,
  SETPOINT_STEP_DOWN,
};

// Each button is a thin wrapper that calls back into the hub -- the hub
// owns the last-captured CONTROL frame and does the actual transmit.
// See the caveat block at the top of megmeet_uart.h before using these:
// none of this is a confirmed working write protocol, it's a probe.
class MegmeetButton : public button::Button {
 public:
  void set_hub(MegmeetUART *hub) { hub_ = hub; }
  void set_action(MegmeetButtonAction action) { action_ = action; }

 protected:
  void press_action() override;

  MegmeetUART *hub_{nullptr};
  MegmeetButtonAction action_{MegmeetButtonAction::REPLAY_LAST_CONTROL};
};

}  // namespace megmeet_uart
}  // namespace esphome
