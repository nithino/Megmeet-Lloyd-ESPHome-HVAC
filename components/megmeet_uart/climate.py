import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate

from . import MegmeetUART, megmeet_uart_ns

DEPENDENCIES = ["megmeet_uart"]

CONF_MEGMEET_UART_ID = "megmeet_uart_id"

MegmeetClimate = megmeet_uart_ns.class_("MegmeetClimate", climate.Climate, cg.Component)

CONFIG_SCHEMA = climate.climate_schema(MegmeetClimate).extend(
    {
        cv.GenerateID(CONF_MEGMEET_UART_ID): cv.use_id(MegmeetUART),
    }
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_MEGMEET_UART_ID])
    cg.add(hub.set_climate(var))
