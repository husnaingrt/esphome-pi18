import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import PI18Component

CONF_PI18_ID = "pi18_id"
CONF_MODE = "mode"

CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_PI18_ID): cv.use_id(PI18Component),
    cv.Optional(CONF_MODE): text_sensor.text_sensor_schema(),
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_PI18_ID])
    if CONF_MODE in config:
        ts = await text_sensor.new_text_sensor(config[CONF_MODE])
        cg.add(hub.set_mode_text_sensor(ts))
