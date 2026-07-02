import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor, text_sensor, uart
from esphome.const import CONF_ID, ENTITY_CATEGORY_DIAGNOSTIC

CODEOWNERS = ["@nithino"]

AUTO_LOAD = ["uart", "sensor", "text_sensor"]
DEPENDENCIES = ["uart"]

megmeet_uart_ns = cg.esphome_ns.namespace("megmeet_uart")

MegmeetUART = megmeet_uart_ns.class_(
    "MegmeetUART",
    cg.Component,
    uart.UARTDevice,
)

# All entities are optional -- omit any you don't want exposed to HA.
CONF_STATUS_RAW = "status_raw"
CONF_CONTROL_SETPOINT_RAW = "control_setpoint_raw"
CONF_HEARTBEAT_TICK = "heartbeat_tick"
CONF_LAST_STATUS_FRAME = "last_status_frame"
CONF_LAST_CONTROL_FRAME = "last_control_frame"
CONF_LAST_HEARTBEAT_FRAME = "last_heartbeat_frame"
CONF_LAST_UNKNOWN_FRAME = "last_unknown_frame"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MegmeetUART),
            # Raw diagnostic values -- see megmeet_uart.cpp for the
            # (unconfirmed) reasoning behind each of these.
            cv.Optional(CONF_STATUS_RAW): sensor.sensor_schema(
                icon="mdi:thermometer",
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_CONTROL_SETPOINT_RAW): sensor.sensor_schema(
                icon="mdi:thermometer-lines",
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_HEARTBEAT_TICK): sensor.sensor_schema(
                icon="mdi:pulse",
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            # Raw hex of the last frame of each type -- handy for
            # watching protocol changes live from the HA dashboard
            # instead of tailing the ESPHome log.
            cv.Optional(CONF_LAST_STATUS_FRAME): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_CONTROL_FRAME): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_HEARTBEAT_FRAME): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_UNKNOWN_FRAME): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)

    if CONF_STATUS_RAW in config:
        sens = await sensor.new_sensor(config[CONF_STATUS_RAW])
        cg.add(var.set_status_raw_sensor(sens))

    if CONF_CONTROL_SETPOINT_RAW in config:
        sens = await sensor.new_sensor(config[CONF_CONTROL_SETPOINT_RAW])
        cg.add(var.set_control_setpoint_raw_sensor(sens))

    if CONF_HEARTBEAT_TICK in config:
        sens = await sensor.new_sensor(config[CONF_HEARTBEAT_TICK])
        cg.add(var.set_heartbeat_tick_sensor(sens))

    if CONF_LAST_STATUS_FRAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_STATUS_FRAME])
        cg.add(var.set_last_status_frame_sensor(sens))

    if CONF_LAST_CONTROL_FRAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_CONTROL_FRAME])
        cg.add(var.set_last_control_frame_sensor(sens))

    if CONF_LAST_HEARTBEAT_FRAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_HEARTBEAT_FRAME])
        cg.add(var.set_last_heartbeat_frame_sensor(sens))

    if CONF_LAST_UNKNOWN_FRAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_UNKNOWN_FRAME])
        cg.add(var.set_last_unknown_frame_sensor(sens))
