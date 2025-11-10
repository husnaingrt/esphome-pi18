import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID, CONF_UPDATE_INTERVAL

AUTO_LOAD = ["uart", "sensor", "text_sensor", "switch", "select", "number"]
DEPENDENCIES = ["uart"]
MULTI_CONF = False

pi18_ns = cg.esphome_ns.namespace("pi18")
PI18Component = pi18_ns.class_("PI18Component", cg.PollingComponent, uart.UARTDevice)

CONF_UART_ID = "uart_id"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(PI18Component),
    cv.Required(CONF_UART_ID): cv.use_id(uart.UARTComponent),
    cv.Optional(CONF_UPDATE_INTERVAL, default="5s"): cv.positive_time_period_milliseconds,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    # IMPORTANT: pass the WHOLE config, not config[CONF_UART_ID]
    await uart.register_uart_device(var, config)
