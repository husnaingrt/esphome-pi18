import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text

from . import PI18Component, pi18_ns

CONF_PI18_ID = "pi18_id"

PI18CommandText = pi18_ns.class_("PI18CommandText", text.Text, cg.Component)

CONFIG_SCHEMA = text.text_schema(
    PI18CommandText,
    icon="mdi:console",
    entity_category="config",
    mode="TEXT",
).extend(
    {
        cv.Required(CONF_PI18_ID): cv.use_id(PI18Component),
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PI18_ID])
    var = await text.new_text(config, max_length=64)
    await cg.register_component(var, config)
    cg.add(var.set_parent(hub))
