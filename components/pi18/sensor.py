import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    UNIT_VOLT, UNIT_HERTZ, UNIT_WATT, UNIT_VOLT_AMPS, UNIT_PERCENT, UNIT_CELSIUS,
    DEVICE_CLASS_VOLTAGE, DEVICE_CLASS_FREQUENCY, DEVICE_CLASS_POWER,
    DEVICE_CLASS_TEMPERATURE, DEVICE_CLASS_CURRENT,
    STATE_CLASS_MEASUREMENT,
)

from . import PI18Component

# Keys shared with the C++ setters
CONF_GRID_VOLTAGE = "grid_voltage"
CONF_GRID_FREQUENCY = "grid_frequency"
CONF_AC_OUTPUT_VOLTAGE = "ac_output_voltage"
CONF_AC_OUTPUT_FREQUENCY = "ac_output_frequency"
CONF_OUTPUT_APPARENT_POWER = "output_apparent_power"
CONF_OUTPUT_ACTIVE_POWER = "output_active_power"
CONF_LOAD_PERCENT = "load_percent"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_BATTERY_CHARGE_CURRENT = "battery_charge_current"
CONF_BATTERY_DISCHARGE_CURRENT = "battery_discharge_current"
CONF_BATTERY_CAPACITY = "battery_capacity"
CONF_HEATSINK_TEMPERATURE = "heatsink_temperature"
CONF_PV1_POWER = "pv1_power"
CONF_PV1_VOLTAGE = "pv1_voltage"

CONF_PI18_ID = "pi18_id"

SENSOR_SCHEMA_MAP = {
    CONF_GRID_VOLTAGE: sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT, accuracy_decimals=1,
        device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT),
    CONF_GRID_FREQUENCY: sensor.sensor_schema(
        unit_of_measurement=UNIT_HERTZ, accuracy_decimals=1,
        device_class=DEVICE_CLASS_FREQUENCY, state_class=STATE_CLASS_MEASUREMENT),
    CONF_AC_OUTPUT_VOLTAGE: sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT, accuracy_decimals=1,
        device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT),
    CONF_AC_OUTPUT_FREQUENCY: sensor.sensor_schema(
        unit_of_measurement=UNIT_HERTZ, accuracy_decimals=1,
        device_class=DEVICE_CLASS_FREQUENCY, state_class=STATE_CLASS_MEASUREMENT),
    CONF_OUTPUT_APPARENT_POWER: sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT_AMPS, accuracy_decimals=0,
        device_class=DEVICE_CLASS_POWER, state_class=STATE_CLASS_MEASUREMENT),
    CONF_OUTPUT_ACTIVE_POWER: sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT, accuracy_decimals=0,
        device_class=DEVICE_CLASS_POWER, state_class=STATE_CLASS_MEASUREMENT),
    CONF_LOAD_PERCENT: sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT),
    CONF_BATTERY_VOLTAGE: sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT, accuracy_decimals=1,
        device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT),
    CONF_BATTERY_CHARGE_CURRENT: sensor.sensor_schema(
        unit_of_measurement="A", accuracy_decimals=0,
        device_class=DEVICE_CLASS_CURRENT, state_class=STATE_CLASS_MEASUREMENT),
    CONF_BATTERY_DISCHARGE_CURRENT: sensor.sensor_schema(
        unit_of_measurement="A", accuracy_decimals=0,
        device_class=DEVICE_CLASS_CURRENT, state_class=STATE_CLASS_MEASUREMENT),
    CONF_BATTERY_CAPACITY: sensor.sensor_schema(
        unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0,
        state_class=STATE_CLASS_MEASUREMENT),
    CONF_HEATSINK_TEMPERATURE: sensor.sensor_schema(
        unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=0,
        device_class=DEVICE_CLASS_TEMPERATURE, state_class=STATE_CLASS_MEASUREMENT),
    CONF_PV1_POWER: sensor.sensor_schema(
        unit_of_measurement=UNIT_WATT, accuracy_decimals=0,
        device_class=DEVICE_CLASS_POWER, state_class=STATE_CLASS_MEASUREMENT),
    CONF_PV1_VOLTAGE: sensor.sensor_schema(
        unit_of_measurement=UNIT_VOLT, accuracy_decimals=1,
        device_class=DEVICE_CLASS_VOLTAGE, state_class=STATE_CLASS_MEASUREMENT),
}

CONFIG_SCHEMA = cv.Schema({
    cv.Required(CONF_PI18_ID): cv.use_id(PI18Component),
    **{cv.Optional(k): v for k, v in SENSOR_SCHEMA_MAP.items()},
})

async def to_code(config):
    hub = await cg.get_variable(config[CONF_PI18_ID])

    for key in SENSOR_SCHEMA_MAP.keys():
        if key in config:
            # Create the sensor entity from its config
            sens = await sensor.new_sensor(config[key])
            # Bind it to the hub via the generated setter
            cg.add(getattr(hub, f"set_{key}_sensor")(sens))
