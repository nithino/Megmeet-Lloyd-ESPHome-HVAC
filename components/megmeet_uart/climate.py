import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import climate, sensor
from esphome.const import CONF_SENSOR

from . import MegmeetUART, megmeet_uart_ns

DEPENDENCIES = ["megmeet_uart"]

CONF_MEGMEET_UART_ID = "megmeet_uart_id"

MegmeetClimate = megmeet_uart_ns.class_("MegmeetClimate", climate.Climate, cg.Component)

CONFIG_SCHEMA = climate.climate_schema(MegmeetClimate).extend(
    {
        cv.GenerateID(CONF_MEGMEET_UART_ID): cv.use_id(MegmeetUART),
        # Optional -- this AC's own protocol has no confirmed
        # current-room-temperature field, only target/setpoint. Point
        # this at any existing sensor (e.g. a `homeassistant:` platform
        # sensor mirroring a separate room thermometer) to have it show
        # up as this climate entity's current_temperature. Same
        # convention as ESPHome's own pid/thermostat climate components.
        cv.Optional(CONF_SENSOR): cv.use_id(sensor.Sensor),
    }
)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)

    hub = await cg.get_variable(config[CONF_MEGMEET_UART_ID])
    cg.add(hub.set_climate(var))

    if CONF_SENSOR in config:
        sens = await cg.get_variable(config[CONF_SENSOR])
        cg.add(var.set_current_temperature_sensor(sens))
