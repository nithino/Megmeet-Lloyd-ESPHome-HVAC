#include "megmeet_button.h"

namespace esphome {
namespace megmeet_uart {

void MegmeetButton::press_action() {
  if (this->hub_ == nullptr)
    return;

  switch (this->action_) {
    case MegmeetButtonAction::REPLAY_LAST_CONTROL:
      this->hub_->replay_last_control_frame();
      break;
    case MegmeetButtonAction::SETPOINT_STEP_UP:
      this->hub_->adjust_control_setpoint(1);
      break;
    case MegmeetButtonAction::SETPOINT_STEP_DOWN:
      this->hub_->adjust_control_setpoint(-1);
      break;
  }
}

}  // namespace megmeet_uart
}  // namespace esphome
