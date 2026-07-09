import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart
from esphome.const import CONF_ID
from esphome.core import CoroPriority, coroutine_with_priority

AUTO_LOAD = [
    "uart",
    "sensor",
    "text_sensor",
    "text",
    "switch",
    "button",
    "select",
    "number",
]
DEPENDENCIES = ["uart"]
MULTI_CONF = False

CONF_PI18_ID = "pi18_id"
BUTTON_FLUSH_UART = 0
BUTTON_READ_UART = 1
BUTTON_SYNC_CONFIGURATION = 2
SELECT_INPUT_VOLTAGE_RANGE = 0
SELECT_OUTPUT_SOURCE_PRIORITY = 1
SELECT_CHARGER_SOURCE_PRIORITY = 2
SELECT_BATTERY_TYPE = 3
SELECT_SOLAR_POWER_PRIORITY = 4
SELECT_OUTPUT_MODEL = 5
SELECT_AC_OUTPUT_FREQUENCY = 6
SELECT_MACHINE_TYPE = 7

NUMBER_BATTERY_CUTOFF_VOLTAGE = 0
NUMBER_MAX_AC_CHARGING_CURRENT = 1
NUMBER_MAX_CHARGING_CURRENT = 2
NUMBER_BATTERY_MAX_CHARGE_VOLTAGE = 3
NUMBER_BATTERY_FLOAT_VOLTAGE = 4
NUMBER_BATTERY_RECHARGE_VOLTAGE = 5
NUMBER_BATTERY_REDISCHARGE_VOLTAGE = 6

SWITCH_LOAD_POWER = 0
SWITCH_SILENCE_BUZZER = 1
SWITCH_OVERLOAD_BYPASS = 2
SWITCH_LCD_ESCAPE = 3
SWITCH_OVERLOAD_RESTART = 4
SWITCH_OVER_TEMP_RESTART = 5
SWITCH_BACKLIGHT = 6
SWITCH_ALARM_PRIMARY_SOURCE_INTERRUPT = 7
SWITCH_FAULT_CODE_RECORD = 8

pi18_ns = cg.esphome_ns.namespace("pi18")
PI18Component = pi18_ns.class_("PI18Component", cg.PollingComponent, uart.UARTDevice)

CONFIG_SCHEMA = cv.Schema({cv.GenerateID(): cv.declare_id(PI18Component)}).extend(
    cv.polling_component_schema("5s")
).extend(uart.UART_DEVICE_SCHEMA)


@coroutine_with_priority(CoroPriority.CORE)
async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cg.add_global(pi18_ns.using)
