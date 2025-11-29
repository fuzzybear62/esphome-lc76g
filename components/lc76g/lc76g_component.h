#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include <vector>

namespace esphome {
namespace lc76g {

/**
 * LC76GComponent
 *
 * This component communicates with the Quectel LC76G GNSS receiver using
 * the proprietary I²C transport used by the module. The module internally
 * buffers NMEA sentences and exposes them to the host through a
 * length-prefixed binary channel. The component implements:
 *
 *  - Zero-copy NMEA parsing directly from the raw I²C buffer
 *  - FIFO self-healing using capped reads to avoid deadlocks
 *  - Soft-recovery (hot start) after repeated corrupted cycles
 *  - Optional hardware hard-reset using a GPIO pin
 *  - Automatic error counters and safe fallback logic
 *
 * The class also exposes several sensors for parsed GNSS data.
 */
class LC76GComponent : public PollingComponent, public i2c::I2CDevice {
 public:
  // Sensor setters
  void set_latitude_sensor(sensor::Sensor *s) { latitude_sensor_ = s; }
  void set_longitude_sensor(sensor::Sensor *s) { longitude_sensor_ = s; }
  void set_altitude_sensor(sensor::Sensor *s) { altitude_sensor_ = s; }
  void set_speed_sensor(sensor::Sensor *s) { speed_sensor_ = s; }
  void set_course_sensor(sensor::Sensor *s) { course_sensor_ = s; }
  void set_hdop_sensor(sensor::Sensor *s) { hdop_sensor_ = s; }
  void set_satellites_sensor(sensor::Sensor *s) { satellites_sensor_ = s; }
  void set_fix_sensor(sensor::Sensor *s) { fix_sensor_ = s; }

  void set_date_sensor(text_sensor::TextSensor *s) { date_sensor_ = s; }
  void set_time_sensor(text_sensor::TextSensor *s) { time_sensor_ = s; }
  void set_raw_nmea_sensor(text_sensor::TextSensor *s) { raw_nmea_sensor_ = s; }

  // Configuration setters
  void set_debug_mode(bool enable) { debug_mode_ = enable; }

  // Manual actions
  void send_hot_start();

  // ESPHome Lifecycle
  void setup() override;
  void update() override;

 protected:
  // Core logic
  bool process_buffer(size_t len);
  
  // Helpers
  bool get_field(const char* start, size_t len, int index, char* out_buf, size_t out_size);
  float parse_coordinate(const char* val_str, const char* dir_str);
  void mark_sensors_unavailable(); 
  
  // Error handling & Recovery
  void handle_bus_error();
  void perform_soft_recovery();

  // Internal State
  uint32_t last_valid_data_{0};
  int error_counter_{0}; 
  bool debug_mode_{false}; // Default: Silent mode
  
  // Static persistent buffer
  std::vector<uint8_t> buffer_; 

  // Sensor pointers
  sensor::Sensor *latitude_sensor_{nullptr};
  sensor::Sensor *longitude_sensor_{nullptr};
  sensor::Sensor *altitude_sensor_{nullptr};
  sensor::Sensor *speed_sensor_{nullptr};
  sensor::Sensor *course_sensor_{nullptr};
  sensor::Sensor *hdop_sensor_{nullptr};
  sensor::Sensor *satellites_sensor_{nullptr};
  sensor::Sensor *fix_sensor_{nullptr};

  text_sensor::TextSensor *date_sensor_{nullptr};
  text_sensor::TextSensor *time_sensor_{nullptr};
  text_sensor::TextSensor *raw_nmea_sensor_{nullptr};
};

}  // namespace lc76g
}  // namespace esphome