import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import number
from esphome.const import (
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_VOLTAGE,
    ENTITY_CATEGORY_CONFIG,
    UNIT_VOLT,
)

from . import (
    CONF_PI18_ID,
    NUMBER_BATTERY_CUTOFF_VOLTAGE,
    NUMBER_BATTERY_FLOAT_VOLTAGE,
    NUMBER_BATTERY_MAX_CHARGE_VOLTAGE,
    NUMBER_BATTERY_RECHARGE_VOLTAGE,
    NUMBER_BATTERY_REDISCHARGE_VOLTAGE,
    NUMBER_MAX_AC_CHARGING_CURRENT,
    NUMBER_MAX_CHARGING_CURRENT,
    PI18Component,
    pi18_ns,
)

PI18SettingNumber = pi18_ns.class_("PI18SettingNumber", number.Number)

NUMBER_SETTINGS = (
    (
        "battery_max_charge_voltage",
        NUMBER_BATTERY_MAX_CHARGE_VOLTAGE,
        48.0,
        58.4,
        0.1,
        UNIT_VOLT,
        DEVICE_CLASS_VOLTAGE,
    ),
    (
        "battery_float_voltage",
        NUMBER_BATTERY_FLOAT_VOLTAGE,
        48.0,
        58.4,
        0.1,
        UNIT_VOLT,
        DEVICE_CLASS_VOLTAGE,
    ),
    (
        "battery_recharge_voltage",
        NUMBER_BATTERY_RECHARGE_VOLTAGE,
        44.0,
        51.0,
        0.1,
        UNIT_VOLT,
        DEVICE_CLASS_VOLTAGE,
    ),
    (
        "battery_redischarge_voltage",
        NUMBER_BATTERY_REDISCHARGE_VOLTAGE,
        44.0,
        51.0,
        0.1,
        UNIT_VOLT,
        DEVICE_CLASS_VOLTAGE,
    ),
    (
        "battery_cut_off_voltage",
        NUMBER_BATTERY_CUTOFF_VOLTAGE,
        40.0,
        48.0,
        0.1,
        UNIT_VOLT,
        DEVICE_CLASS_VOLTAGE,
    ),
    (
        "max_ac_charging_current",
        NUMBER_MAX_AC_CHARGING_CURRENT,
        0.0,
        9.0,
        1.0,
        "A",
        DEVICE_CLASS_CURRENT,
    ),
    (
        "max_charging_current",
        NUMBER_MAX_CHARGING_CURRENT,
        0.0,
        9.0,
        1.0,
        "A",
        DEVICE_CLASS_CURRENT,
    ),
)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_PI18_ID): cv.use_id(PI18Component),
        **{
            cv.Optional(key): number.number_schema(
                PI18SettingNumber,
                entity_category=ENTITY_CATEGORY_CONFIG,
                device_class=device_class,
                unit_of_measurement=unit,
            )
            for key, _, _, _, _, unit, device_class in NUMBER_SETTINGS
        },
    }
)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_PI18_ID])
    for key, kind, min_value, max_value, step, _, _ in NUMBER_SETTINGS:
        if setting_config := config.get(key):
            num = await number.new_number(
                setting_config,
                kind,
                min_value=min_value,
                max_value=max_value,
                step=step,
            )
            await cg.register_parented(num, hub)
            cg.add(hub.set_number(kind, num))
