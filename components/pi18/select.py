import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import select
from esphome.const import ENTITY_CATEGORY_CONFIG

from . import (
    CONF_PI18_ID,
    PI18Component,
    SELECT_AC_OUTPUT_FREQUENCY,
    SELECT_BATTERY_TYPE,
    SELECT_CHARGER_SOURCE_PRIORITY,
    SELECT_INPUT_VOLTAGE_RANGE,
    SELECT_MACHINE_TYPE,
    SELECT_OUTPUT_MODEL,
    SELECT_OUTPUT_SOURCE_PRIORITY,
    SELECT_SOLAR_POWER_PRIORITY,
    pi18_ns,
)

PI18SettingSelect = pi18_ns.class_("PI18SettingSelect", select.Select)

SELECT_SETTINGS = (
    (
        "input_voltage_range",
        SELECT_INPUT_VOLTAGE_RANGE,
        ["Appliance", "UPS"],
    ),
    (
        "output_source_priority",
        SELECT_OUTPUT_SOURCE_PRIORITY,
        ["Solar-Utility-Battery", "Solar-Battery-Utility"],
    ),
    (
        "charger_source_priority",
        SELECT_CHARGER_SOURCE_PRIORITY,
        ["Solar first", "Solar and Utility", "Only solar"],
    ),
    ("battery_type", SELECT_BATTERY_TYPE, ["AGM", "Flooded", "User"]),
    (
        "solar_power_priority",
        SELECT_SOLAR_POWER_PRIORITY,
        ["Battery-Load-Utility", "Load-Battery-Utility"],
    ),
    (
        "output_model",
        SELECT_OUTPUT_MODEL,
        [
            "Single module",
            "Parallel output",
            "Phase 1 of three phase output",
            "Phase 2 of three phase output",
            "Phase 3 of three phase output",
        ],
    ),
    ("ac_output_frequency", SELECT_AC_OUTPUT_FREQUENCY, ["50 Hz", "60 Hz"]),
    ("machine_type", SELECT_MACHINE_TYPE, ["Off-Grid Tie", "Grid-Tie"]),
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PI18_ID): cv.use_id(PI18Component),
        **{
            cv.Optional(key): select.select_schema(
                PI18SettingSelect,
                entity_category=ENTITY_CATEGORY_CONFIG,
            )
            for key, _, _ in SELECT_SETTINGS
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PI18_ID])
    for key, kind, options in SELECT_SETTINGS:
        if setting_config := config.get(key):
            sel = await select.new_select(setting_config, kind, options=options)
            await cg.register_parented(sel, hub)
            cg.add(hub.set_select(kind, sel))
