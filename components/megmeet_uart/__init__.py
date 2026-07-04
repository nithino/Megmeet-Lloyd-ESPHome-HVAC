import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor, text_sensor, uart
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_RUNNING,
    DEVICE_CLASS_TEMPERATURE,
    ENTITY_CATEGORY_DIAGNOSTIC,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
)

CODEOWNERS = ["@nithino"]

AUTO_LOAD = ["uart", "sensor", "text_sensor", "button", "binary_sensor", "climate"]
DEPENDENCIES = ["uart"]

megmeet_uart_ns = cg.esphome_ns.namespace("megmeet_uart")

MegmeetUART = megmeet_uart_ns.class_(
    "MegmeetUART",
    cg.Component,
    uart.UARTDevice,
)

# All entities are optional -- omit any you don't want exposed to HA.
CONF_STATUS_RAW = "status_raw"
CONF_SETPOINT_CELSIUS = "setpoint_celsius"
CONF_HEARTBEAT_TICK = "heartbeat_tick"
CONF_POWER_REQUESTED = "power_requested"
CONF_AC_RUNNING = "ac_running"
CONF_LAST_STATUS_FRAME = "last_status_frame"
CONF_LAST_CONTROL_FRAME = "last_control_frame"
CONF_LAST_HEARTBEAT_FRAME = "last_heartbeat_frame"
CONF_LAST_UNKNOWN_FRAME = "last_unknown_frame"
CONF_LAST_FAN_EVENT_FRAME = "last_fan_event_frame"
CONF_LAST_MODE_EVENT_FRAME = "last_mode_event_frame"

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
            # Confirmed via a controlled button-press test: tail1 of the
            # event-driven 0x20 frame is the user's setpoint in degrees C,
            # direct, no scaling. Fires once per button press, not on a
            # periodic cycle -- see megmeet_uart.h/.cpp for the full test.
            cv.Optional(CONF_SETPOINT_CELSIUS): sensor.sensor_schema(
                unit_of_measurement=UNIT_CELSIUS,
                device_class=DEVICE_CLASS_TEMPERATURE,
                state_class=STATE_CLASS_MEASUREMENT,
                accuracy_decimals=0,
            ),
            cv.Optional(CONF_HEARTBEAT_TICK): sensor.sensor_schema(
                icon="mdi:pulse",
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            # Confirmed via a controlled power on/off test: type=0x40
            # fires instantly when the mainboard receives a power button
            # press. This is the COMMAND, not the actual state -- see
            # ac_running below for when the AC has really stopped.
            cv.Optional(CONF_POWER_REQUESTED): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_POWER,
            ),
            # Confirmed via the same test, letting the full ~60s shutdown
            # countdown finish uninterrupted: STATUS/CONTROL tail1 both
            # drop to 0x00 and stop updating right as the AC's flap
            # physically closes. This reflects the AC's actual run state,
            # lagging power_requested by up to ~60s on shutdown.
            cv.Optional(CONF_AC_RUNNING): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_RUNNING,
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
            # Fan speed and mode changes are confirmed real, event-driven
            # signals (same pattern as setpoint_celsius), but the exact
            # byte-to-speed/byte-to-mode mapping isn't confirmed yet --
            # each was only sampled once per value in a single controlled
            # test. Exposed as raw hex rather than a guessed enum; see
            # megmeet_uart.h for the full writeup and what IS confirmed
            # (e.g. a missing mode-event frame reliably means the
            # mainboard rejected that mode).
            cv.Optional(CONF_LAST_FAN_EVENT_FRAME): text_sensor.text_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_LAST_MODE_EVENT_FRAME): text_sensor.text_sensor_schema(
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

    if CONF_SETPOINT_CELSIUS in config:
        sens = await sensor.new_sensor(config[CONF_SETPOINT_CELSIUS])
        cg.add(var.set_setpoint_celsius_sensor(sens))

    if CONF_HEARTBEAT_TICK in config:
        sens = await sensor.new_sensor(config[CONF_HEARTBEAT_TICK])
        cg.add(var.set_heartbeat_tick_sensor(sens))

    if CONF_POWER_REQUESTED in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_POWER_REQUESTED])
        cg.add(var.set_power_requested_sensor(sens))

    if CONF_AC_RUNNING in config:
        sens = await binary_sensor.new_binary_sensor(config[CONF_AC_RUNNING])
        cg.add(var.set_ac_running_sensor(sens))

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

    if CONF_LAST_FAN_EVENT_FRAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_FAN_EVENT_FRAME])
        cg.add(var.set_last_fan_event_frame_sensor(sens))

    if CONF_LAST_MODE_EVENT_FRAME in config:
        sens = await text_sensor.new_text_sensor(config[CONF_LAST_MODE_EVENT_FRAME])
        cg.add(var.set_last_mode_event_frame_sensor(sens))
