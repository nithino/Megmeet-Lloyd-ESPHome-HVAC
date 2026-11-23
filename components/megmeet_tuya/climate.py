import esphome.codegen as cg
from esphome.components import climate
import esphome.config_validation as cv

from . import CONF_MEGMEET_TUYA_ID, MegmeetTuya, megmeet_tuya_ns

DEPENDENCIES = ["megmeet_tuya"]

CONF_POWER_DATAPOINT = "power_datapoint"
CONF_TARGET_TEMPERATURE_DATAPOINT = "target_temperature_datapoint"
CONF_CURRENT_TEMPERATURE_DATAPOINT = "current_temperature_datapoint"
CONF_TEMPERATURE_MULTIPLIER = "temperature_multiplier"
CONF_MODE_DATAPOINT = "mode_datapoint"
CONF_MODES = "modes"
CONF_FAN_MODE_DATAPOINT = "fan_mode_datapoint"
CONF_FAN_MODES = "fan_modes"
CONF_CUSTOM_FAN_MODES = "custom_fan_modes"
CONF_SWING_VERTICAL_DATAPOINT = "swing_vertical_datapoint"
CONF_SWING_HORIZONTAL_DATAPOINT = "swing_horizontal_datapoint"

MegmeetClimate = megmeet_tuya_ns.class_("MegmeetClimate", climate.Climate, cg.Component)

CLIMATE_MODES = {
    "cool": climate.ClimateMode.CLIMATE_MODE_COOL,
    "heat": climate.ClimateMode.CLIMATE_MODE_HEAT,
    "dry": climate.ClimateMode.CLIMATE_MODE_DRY,
    "fan_only": climate.ClimateMode.CLIMATE_MODE_FAN_ONLY,
    "auto": climate.ClimateMode.CLIMATE_MODE_AUTO,
    "heat_cool": climate.ClimateMode.CLIMATE_MODE_HEAT_COOL,
}
FAN_MODES = {
    "auto": climate.ClimateFanMode.CLIMATE_FAN_AUTO,
    "low": climate.ClimateFanMode.CLIMATE_FAN_LOW,
    "medium": climate.ClimateFanMode.CLIMATE_FAN_MEDIUM,
    "middle": climate.ClimateFanMode.CLIMATE_FAN_MIDDLE,
    "high": climate.ClimateFanMode.CLIMATE_FAN_HIGH,
    "quiet": climate.ClimateFanMode.CLIMATE_FAN_QUIET,
    "focus": climate.ClimateFanMode.CLIMATE_FAN_FOCUS,
    "diffuse": climate.ClimateFanMode.CLIMATE_FAN_DIFFUSE,
}

CONFIG_SCHEMA = climate.climate_schema(MegmeetClimate).extend(
    {
        cv.GenerateID(CONF_MEGMEET_TUYA_ID): cv.use_id(MegmeetTuya),
        cv.Optional(CONF_POWER_DATAPOINT, default=1): cv.uint8_t,
        cv.Optional(CONF_TARGET_TEMPERATURE_DATAPOINT, default=2): cv.uint8_t,
        cv.Optional(CONF_CURRENT_TEMPERATURE_DATAPOINT): cv.uint8_t,
        cv.Optional(CONF_TEMPERATURE_MULTIPLIER, default=1.0): cv.positive_float,
        cv.Optional(CONF_MODE_DATAPOINT, default=4): cv.uint8_t,
        # climate mode -> raw enum value reported/accepted by the AC
        cv.Required(CONF_MODES): cv.Schema({cv.one_of(*CLIMATE_MODES, lower=True): cv.uint8_t}),
        cv.Optional(CONF_FAN_MODE_DATAPOINT): cv.uint8_t,
        cv.Optional(CONF_FAN_MODES, default={}): cv.Schema({cv.one_of(*FAN_MODES, lower=True): cv.uint8_t}),
        # free-text fan mode name -> raw value (for levels without a standard name)
        cv.Optional(CONF_CUSTOM_FAN_MODES, default={}): cv.Schema({cv.string_strict: cv.uint8_t}),
        cv.Optional(CONF_SWING_VERTICAL_DATAPOINT): cv.uint8_t,
        cv.Optional(CONF_SWING_HORIZONTAL_DATAPOINT): cv.uint8_t,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    var = await climate.new_climate(config)
    await cg.register_component(var, config)
    parent = await cg.get_variable(config[CONF_MEGMEET_TUYA_ID])
    cg.add(var.set_parent(parent))
    cg.add(var.set_power_dp(config[CONF_POWER_DATAPOINT]))
    cg.add(var.set_target_temperature_dp(config[CONF_TARGET_TEMPERATURE_DATAPOINT]))
    if CONF_CURRENT_TEMPERATURE_DATAPOINT in config:
        cg.add(var.set_current_temperature_dp(config[CONF_CURRENT_TEMPERATURE_DATAPOINT]))
    cg.add(var.set_temperature_multiplier(config[CONF_TEMPERATURE_MULTIPLIER]))
    cg.add(var.set_mode_dp(config[CONF_MODE_DATAPOINT]))
    for name, raw in config[CONF_MODES].items():
        cg.add(var.add_mode(CLIMATE_MODES[name], raw))
    if CONF_FAN_MODE_DATAPOINT in config:
        cg.add(var.set_fan_dp(config[CONF_FAN_MODE_DATAPOINT]))
        for name, raw in config[CONF_FAN_MODES].items():
            cg.add(var.add_fan_mode(FAN_MODES[name], raw))
        for name, raw in config[CONF_CUSTOM_FAN_MODES].items():
            cg.add(var.add_custom_fan_mode(name, raw))
    if CONF_SWING_VERTICAL_DATAPOINT in config:
        cg.add(var.set_swing_vertical_dp(config[CONF_SWING_VERTICAL_DATAPOINT]))
    if CONF_SWING_HORIZONTAL_DATAPOINT in config:
        cg.add(var.set_swing_horizontal_dp(config[CONF_SWING_HORIZONTAL_DATAPOINT]))
