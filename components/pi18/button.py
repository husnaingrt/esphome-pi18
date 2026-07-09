import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import button
from esphome.const import ENTITY_CATEGORY_DIAGNOSTIC

from . import (
    BUTTON_FLUSH_UART,
    BUTTON_READ_UART,
    BUTTON_SYNC_CONFIGURATION,
    CONF_PI18_ID,
    PI18Component,
    pi18_ns,
)

PI18DebugButton = pi18_ns.class_("PI18DebugButton", button.Button)

BUTTONS = (
    ("flush_uart", BUTTON_FLUSH_UART),
    ("read_uart", BUTTON_READ_UART),
    ("sync_configuration", BUTTON_SYNC_CONFIGURATION),
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PI18_ID): cv.use_id(PI18Component),
        **{
            cv.Optional(key): button.button_schema(
                PI18DebugButton,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            )
            for key, _ in BUTTONS
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PI18_ID])
    for key, kind in BUTTONS:
        if setting_config := config.get(key):
            btn = await button.new_button(setting_config, kind)
            await cg.register_parented(btn, hub)
