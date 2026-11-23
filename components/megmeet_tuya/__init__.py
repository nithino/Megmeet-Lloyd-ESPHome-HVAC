import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@nithino"]
DEPENDENCIES = ["uart"]
MULTI_CONF = False

CONF_MEGMEET_TUYA_ID = "megmeet_tuya_id"
CONF_HEARTBEAT_INTERVAL = "heartbeat_interval"
CONF_LINK_TIMEOUT = "link_timeout"
CONF_HANDSHAKE = "handshake"

megmeet_tuya_ns = cg.esphome_ns.namespace("megmeet_tuya")
MegmeetTuya = megmeet_tuya_ns.class_("MegmeetTuya", cg.Component, uart.UARTDevice)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MegmeetTuya),
            # true: heartbeats + init sequence (product/working mode/network status/DP query) on boot and
            # after the AC restarts. false: pure listener, never transmits on its own.
            cv.Optional(CONF_HANDSHAKE, default=True): cv.boolean,
            cv.Optional(CONF_HEARTBEAT_INTERVAL, default="15s"): cv.positive_time_period_milliseconds,
            # No valid frame for this long -> link considered down (sensors keep last value).
            cv.Optional(CONF_LINK_TIMEOUT, default="30s"): cv.positive_time_period_milliseconds,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)

FINAL_VALIDATE_SCHEMA = uart.final_validate_device_schema("megmeet_tuya", baud_rate=9600, require_rx=True)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add(var.set_handshake(config[CONF_HANDSHAKE]))
    cg.add(var.set_heartbeat_interval(config[CONF_HEARTBEAT_INTERVAL]))
    cg.add(var.set_link_timeout(config[CONF_LINK_TIMEOUT]))
