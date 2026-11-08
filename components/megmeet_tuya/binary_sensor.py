import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor
from esphome.const import DEVICE_CLASS_CONNECTIVITY, ENTITY_CATEGORY_DIAGNOSTIC
from . import CONF_MEGMEET_TUYA_ID, MegmeetTuya

DEPENDENCIES = ["megmeet_tuya"]

# The only binary sensor: "UART link" = valid frames from the AC are arriving.
CONFIG_SCHEMA = binary_sensor.binary_sensor_schema(
    device_class=DEVICE_CLASS_CONNECTIVITY,
    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
).extend({cv.GenerateID(CONF_MEGMEET_TUYA_ID): cv.use_id(MegmeetTuya)})


async def to_code(config):
    parent = await cg.get_variable(config[CONF_MEGMEET_TUYA_ID])
    bs = await binary_sensor.new_binary_sensor(config)
    cg.add(parent.set_link_sensor(bs))
