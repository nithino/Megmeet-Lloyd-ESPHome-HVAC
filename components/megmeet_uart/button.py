import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import MegmeetUART, megmeet_uart_ns

CONF_MEGMEET_UART_ID = "megmeet_uart_id"
CONF_ACTION = "action"

MegmeetButton = megmeet_uart_ns.class_("MegmeetButton", button.Button)
MegmeetButtonAction = megmeet_uart_ns.enum("MegmeetButtonAction", is_class=True)

# Keep these names in sync with the MegmeetButtonAction enum in
# megmeet_button.h.
ACTIONS = {
    "replay_last_control": MegmeetButtonAction.REPLAY_LAST_CONTROL,
    "setpoint_step_up": MegmeetButtonAction.SETPOINT_STEP_UP,
    "setpoint_step_down": MegmeetButtonAction.SETPOINT_STEP_DOWN,
}

CONFIG_SCHEMA = button.button_schema(
    MegmeetButton,
    entity_category=ENTITY_CATEGORY_CONFIG,
).extend(
    {
        cv.GenerateID(CONF_MEGMEET_UART_ID): cv.use_id(MegmeetUART),
        cv.Required(CONF_ACTION): cv.enum(ACTIONS, lower=True),
    }
)


async def to_code(config):
    var = await button.new_button(config)

    hub = await cg.get_variable(config[CONF_MEGMEET_UART_ID])
    cg.add(var.set_hub(hub))
    cg.add(var.set_action(config[CONF_ACTION]))
