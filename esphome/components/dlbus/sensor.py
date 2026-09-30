import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import pins
from esphome.components import sensor
from esphome.const import CONF_ID, CONF_PIN, CONF_POWER, UNIT_KILOWATT, DEVICE_CLASS_POWER, STATE_CLASS_MEASUREMENT
from esphome.cpp_helpers import gpio_pin_expression

dlbus_ns = cg.esphome_ns.namespace("dlbus")
DlBus = dlbus_ns.class_("DlBus", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DlBus),
    cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,
    cv.Optional(CONF_POWER): sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT,
        accuracy_decimals=1,
        device_class=DEVICE_CLASS_POWER,
        state_class=STATE_CLASS_MEASUREMENT,
    )
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    pin = await gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))

    if CONF_POWER in config:
        sens = await sensor.new_sensor(config[CONF_POWER])
        cg.add(var.set_power_sensor(sens))
