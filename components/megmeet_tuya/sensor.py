import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from . import CONF_MEGMEET_TUYA_ID, MegmeetTuya

DEPENDENCIES = ["megmeet_tuya"]
CONF_DATAPOINT = "datapoint"
CONF_MULTIPLY = "multiply"
CONF_OFFSET = "offset"

CONFIG_SCHEMA = sensor.sensor_schema().extend(
    {
        cv.GenerateID(CONF_MEGMEET_TUYA_ID): cv.use_id(MegmeetTuya),
        cv.Required(CONF_DATAPOINT): cv.int_range(min=0, max=255),
        cv.Optional(CONF_MULTIPLY, default=1.0): cv.float_,
        cv.Optional(CONF_OFFSET, default=0.0): cv.float_,
    }
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MEGMEET_TUYA_ID])
    sens = await sensor.new_sensor(config)
    cg.add(parent.register_sensor(config[CONF_DATAPOINT], sens, config[CONF_MULTIPLY], config[CONF_OFFSET]))
