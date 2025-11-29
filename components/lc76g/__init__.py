import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, sensor, text_sensor
from esphome.const import CONF_ID

# New configuration constant for debug mode
CONF_DEBUG = "debug"

DEPENDENCIES = ['i2c', 'sensor', 'text_sensor']

lc76g_ns = cg.esphome_ns.namespace("lc76g")
LC76GComponent = lc76g_ns.class_("LC76GComponent", cg.PollingComponent, i2c.I2CDevice)

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(LC76GComponent),
    
    # Optional debug flag (default: false)
    # If true, populates the raw_nmea_sensor with live data.
    cv.Optional(CONF_DEBUG, default=False): cv.boolean,

    cv.Optional("latitude_sensor"): sensor.sensor_schema(),
    cv.Optional("longitude_sensor"): sensor.sensor_schema(),
    cv.Optional("altitude_sensor"): sensor.sensor_schema(),
    cv.Optional("speed_sensor"): sensor.sensor_schema(),
    cv.Optional("course_sensor"): sensor.sensor_schema(),
    cv.Optional("hdop_sensor"): sensor.sensor_schema(),
    cv.Optional("satellites_sensor"): sensor.sensor_schema(),
    cv.Optional("fix_sensor"): sensor.sensor_schema(),

    cv.Optional("date_sensor"): text_sensor.text_sensor_schema(),
    cv.Optional("time_sensor"): text_sensor.text_sensor_schema(),
    cv.Optional("raw_nmea_sensor"): text_sensor.text_sensor_schema(),
}).extend(cv.polling_component_schema("2s")).extend(i2c.i2c_device_schema(0x50))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    # Pass debug configuration to C++
    if CONF_DEBUG in config:
        cg.add(var.set_debug_mode(config[CONF_DEBUG]))

    if "latitude_sensor" in config:
        sens = await sensor.new_sensor(config["latitude_sensor"])
        cg.add(var.set_latitude_sensor(sens))
    if "longitude_sensor" in config:
        sens = await sensor.new_sensor(config["longitude_sensor"])
        cg.add(var.set_longitude_sensor(sens))
    if "altitude_sensor" in config:
        sens = await sensor.new_sensor(config["altitude_sensor"])
        cg.add(var.set_altitude_sensor(sens))
    if "speed_sensor" in config:
        sens = await sensor.new_sensor(config["speed_sensor"])
        cg.add(var.set_speed_sensor(sens))
    if "course_sensor" in config:
        sens = await sensor.new_sensor(config["course_sensor"])
        cg.add(var.set_course_sensor(sens))
    if "hdop_sensor" in config:
        sens = await sensor.new_sensor(config["hdop_sensor"])
        cg.add(var.set_hdop_sensor(sens))
    if "satellites_sensor" in config:
        sens = await sensor.new_sensor(config["satellites_sensor"])
        cg.add(var.set_satellites_sensor(sens))
    if "fix_sensor" in config:
        sens = await sensor.new_sensor(config["fix_sensor"])
        cg.add(var.set_fix_sensor(sens))

    if "date_sensor" in config:
        ts = await text_sensor.new_text_sensor(config["date_sensor"])
        cg.add(var.set_date_sensor(ts))
    if "time_sensor" in config:
        ts = await text_sensor.new_text_sensor(config["time_sensor"])
        cg.add(var.set_time_sensor(ts))
    if "raw_nmea_sensor" in config:
        ts = await text_sensor.new_text_sensor(config["raw_nmea_sensor"])
        cg.add(var.set_raw_nmea_sensor(ts))