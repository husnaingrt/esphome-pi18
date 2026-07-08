import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import switch
from esphome.const import ENTITY_CATEGORY_CONFIG, ENTITY_CATEGORY_DIAGNOSTIC

from . import (
    CONF_PI18_ID,
    PI18Component,
    SWITCH_ALARM_PRIMARY_SOURCE_INTERRUPT,
    SWITCH_BACKLIGHT,
    SWITCH_FAULT_CODE_RECORD,
    SWITCH_LCD_ESCAPE,
    SWITCH_LOAD_POWER,
    SWITCH_OVERLOAD_BYPASS,
    SWITCH_OVERLOAD_RESTART,
    SWITCH_OVER_TEMP_RESTART,
    SWITCH_SILENCE_BUZZER,
    pi18_ns,
)

PI18SettingSwitch = pi18_ns.class_("PI18SettingSwitch", switch.Switch)
PI18PollingSwitch = pi18_ns.class_("PI18PollingSwitch", switch.Switch, cg.Component)

SWITCH_SETTINGS = (
    ("load_power", SWITCH_LOAD_POWER),
    ("silence_buzzer", SWITCH_SILENCE_BUZZER),
    ("overload_bypass", SWITCH_OVERLOAD_BYPASS),
    ("lcd_escape", SWITCH_LCD_ESCAPE),
    ("overload_restart", SWITCH_OVERLOAD_RESTART),
    ("over_temp_restart", SWITCH_OVER_TEMP_RESTART),
    ("backlight", SWITCH_BACKLIGHT),
    ("alarm_primary_source_interrupt", SWITCH_ALARM_PRIMARY_SOURCE_INTERRUPT),
    ("fault_code_record", SWITCH_FAULT_CODE_RECORD),
)

CONF_POLLING_ENABLED = "polling_enabled"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PI18_ID): cv.use_id(PI18Component),
        cv.Optional(CONF_POLLING_ENABLED): switch.switch_schema(
            PI18PollingSwitch,
            entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            default_restore_mode="RESTORE_DEFAULT_ON",
        ).extend(cv.COMPONENT_SCHEMA),
        **{
            cv.Optional(key): switch.switch_schema(
                PI18SettingSwitch,
                entity_category=ENTITY_CATEGORY_CONFIG,
            )
            for key, _ in SWITCH_SETTINGS
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PI18_ID])
    if polling_enabled_config := config.get(CONF_POLLING_ENABLED):
        sw = await switch.new_switch(polling_enabled_config)
        await cg.register_component(sw, polling_enabled_config)
        await cg.register_parented(sw, hub)
    for key, kind in SWITCH_SETTINGS:
        if setting_config := config.get(key):
            sw = await switch.new_switch(setting_config, kind)
            await cg.register_parented(sw, hub)
            cg.add(hub.set_switch(kind, sw))
