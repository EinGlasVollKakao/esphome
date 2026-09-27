import esphome.config_validation as cv
import esphome.codegen as cg
from esphome import pins
from esphome.const import CONF_ID, CONF_PIN
from esphome.cpp_helpers import gpio_pin_expression

CONF_FOO = "foo"
CONF_BAR = "bar"
CONF_BAZ = "baz"

dlbus_ns = cg.esphome_ns.namespace("dlbus")
DlBus = dlbus_ns.class_("DlBus", cg.Component)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(DlBus),
    cv.Required(CONF_PIN): pins.internal_gpio_input_pin_schema,
})

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)
    pin = await gpio_pin_expression(config[CONF_PIN])
    cg.add(var.set_pin(pin))
