import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID

CODEOWNERS = ["@nithino"]

AUTO_LOAD = ["uart"]
DEPENDENCIES = ["uart"]

megmeet_uart_ns = cg.esphome_ns.namespace("megmeet_uart")

MegmeetUART = megmeet_uart_ns.class_(
    "MegmeetUART",
    cg.Component,
    uart.UARTDevice,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MegmeetUART),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
