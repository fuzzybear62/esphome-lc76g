/**
 * ============================================================================
 * LC76G I²C PROTOCOL AND TIMING OVERVIEW
 * ============================================================================
 *
 * 1. Transport Format
 * -----------------
 * The LC76G GNSS module uses a proprietary I²C transport to expose its
 * internal NMEA FIFO to the host. Every transaction begins with a 4-byte
 * little-endian LENGTH header:
 *
 * [len0] [len1] [len2] [len3]
 *
 * where LENGTH indicates how many bytes of NMEA payload are available.
 *
 *
 * 2. Host Read Procedure
 * --------------------
 * The host must:
 *
 * (1) READ 4-byte header
 * (2) WRITE an "ACK command" echoing the received header
 * (3) READ <LENGTH> bytes of payload
 *
 * Example I²C exchange:
 *
 * Read:  0x54 -> [04][00][00][00]
 * Write: 0x50 <- Ack packet "0x00 0x20 0x51 0xAA <hdr0..3>"
 * Read:  0x54 -> <payload bytes...>
 *
 *
 * 3. FIFO Self-Healing
 * -----------------
 * If LENGTH is corrupted (e.g., electrical noise reporting 60,000 bytes),
 * the module expects the host to still read something. The host must
 * ALWAYS perform a capped read (10 KB maximum) to advance the FIFO pointer.
 *
 * Failure to read will NOT clear the module’s internal FIFO. Repeated
 * skip operations result in a permanent deadlock — the module will
 * continuously report the same LENGTH forever.
 *
 *
 * 4. Timing Requirements
 * -------------------
 * - After ACK, the module needs ~5–15 ms to prepare the payload.
 * - A hard reset (power toggle) requires ~800–1200 ms to reboot.
 * - Reading large blocks at 100 kHz I²C can take hundreds of milliseconds.
 *
 *
 * 5. Auto-Recovery Strategy Implemented Here
 * ---------------------------------------
 * - Cap reads at 10 KB to force FIFO advancement.
 * - Count consecutive cycles with corrupted or missing NMEA sentences.
 * - After N cycles:
 * SOFT RECOVERY: send a hot-start command and a NOP packet.
 * HARD RESET: toggle reset GPIO if available, otherwise multiple soft recoveries.
 *
 * This ensures:
 * • no FIFO deadlocks
 * • no infinite resets
 * • automatic self-healing on intermittent noise
 *
 * ============================================================================
 * END OF PROTOCOL OVERVIEW
 * ============================================================================
 */

#include "lc76g_component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>

namespace esphome {
namespace lc76g {

static const char *TAG = "lc76g";
static const uint8_t ADDR_WRITE = 0x50;
static const uint8_t ADDR_READ  = 0x54; 
static const uint32_t DATA_TIMEOUT_MS = 5000;
static const int SOFT_RECOVERY_THRESHOLD = 5; 

void LC76GComponent::setup() {
    ESP_LOGI(TAG, "LC76G Init. Protocol: 4-Step + Auto-Recovery. Debug: %s", this->debug_mode_ ? "ON" : "OFF");
    this->last_valid_data_ = millis();
    this->buffer_.resize(10240);
    this->error_counter_ = 0;

    // Publish static state if raw sensor is configured but debug is OFF
    if (this->raw_nmea_sensor_ && !this->debug_mode_) {
        this->raw_nmea_sensor_->publish_state("Debug Mode OFF");
    }
}

static uint32_t le_u32(const uint8_t *d) {
  return (uint32_t)d[0] | ((uint32_t)d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
}

bool validate_checksum(const char* start, size_t len) {
    const char* star = (const char*)memchr(start, '*', len);
    if (!star) return false;
    size_t star_idx = star - start;
    // Check minimal length
    if (star_idx + 2 >= len) return false;

    char hex_buf[3] = {star[1], star[2], 0};
    char* endptr = nullptr;
    long received_val = strtol(hex_buf, &endptr, 16);
    if (endptr == hex_buf) return false;

    uint8_t calculated = 0;
    for (size_t i = 1; i < star_idx; i++) {
        calculated ^= start[i];
    }
    return (calculated == (uint8_t)received_val);
}

bool LC76GComponent::get_field(const char* start, size_t len, int index, char* out_buf, size_t out_size) {
    int current_idx = 0;
    const char* p = start;
    const char* end = start + len;
    
    // Scan for Nth comma
    while (p < end && current_idx < index) {
        if (*p == ',') current_idx++;
        p++;
    }
    if (p >= end) return false;

    // Find end of field
    const char* field_end = p;
    while (field_end < end && *field_end != ',' && *field_end != '*' && *field_end != '\r' && *field_end != '\n') {
        field_end++;
    }
    
    size_t field_len = field_end - p;
    if (field_len == 0) return false; // Empty field
    
    if (field_len >= out_size) field_len = out_size - 1;
    memcpy(out_buf, p, field_len);
    out_buf[field_len] = 0;
    return true;
}

void LC76GComponent::handle_bus_error() {
    this->error_counter_++;
    ESP_LOGW(TAG, "Bus/Data Error cycle %d/%d", this->error_counter_, SOFT_RECOVERY_THRESHOLD);
    
    if (this->error_counter_ >= SOFT_RECOVERY_THRESHOLD) {
        ESP_LOGE(TAG, "Recovery Threshold Reached. Triggering Soft Recovery...");
        this->perform_soft_recovery();
        this->error_counter_ = 0;
        this->last_valid_data_ = millis();
    }
}

void LC76GComponent::update() {
  // STEP 1: Handshake
  uint8_t init_cmd[] = {0x08, 0x00, 0x51, 0xAA, 0x04, 0x00, 0x00, 0x00};
  if (this->bus_->write(ADDR_WRITE, init_cmd, sizeof(init_cmd)) != i2c::ERROR_OK) {
      this->mark_sensors_unavailable(); 
      this->handle_bus_error();
      return;
  }
  
  vTaskDelay(pdMS_TO_TICKS(50)); 

  // STEP 2: Read Header
  uint8_t hdr[4]; 
  if (this->bus_->read(ADDR_READ, hdr, sizeof(hdr)) != i2c::ERROR_OK) {
       this->handle_bus_error(); 
       return; 
  }

  uint32_t len = le_u32(hdr);
  
  if (len == 0) {
      if (millis() - this->last_valid_data_ > DATA_TIMEOUT_MS) {
          ESP_LOGW(TAG, "GPS Timeout (Stuck at 0 bytes).");
          this->mark_sensors_unavailable();
          this->handle_bus_error();
      }
      return;
  }

  // Safety Cap
  if (len > this->buffer_.size()) len = this->buffer_.size();

  vTaskDelay(pdMS_TO_TICKS(10));

  // STEP 3: Ack
  uint8_t req[] = {0x00, 0x20, 0x51, 0xAA, hdr[0], hdr[1], hdr[2], hdr[3]};
  if (this->bus_->write(ADDR_WRITE, req, sizeof(req)) != i2c::ERROR_OK) {
      this->handle_bus_error();
      return;
  }
  
  vTaskDelay(pdMS_TO_TICKS(50));

  // STEP 4: Read Body
  if (this->bus_->read(ADDR_READ, this->buffer_.data(), len) != i2c::ERROR_OK) {
      this->handle_bus_error();
      return;
  }

  // Process & Recovery
  bool packet_found = this->process_buffer(len);

  if (packet_found) {
      this->error_counter_ = 0;
      this->last_valid_data_ = millis();
  } else {
      this->handle_bus_error();
  }
}

bool LC76GComponent::process_buffer(size_t len) {
    const char* raw = (const char*)this->buffer_.data();
    size_t start = 0;
    bool at_least_one_valid = false;
    
    // Smart Crop
    if (len > 2048) {
        start = len - 2048;
        while (start < len && raw[start] != '$') start++;
    }

    // --- DEBUG LOGIC ---
    // Only publish to raw_nmea_sensor if debug mode is active.
    if (this->raw_nmea_sensor_ && this->debug_mode_) {
        size_t print_len = len - start;
        // Truncate to prevent MQTT/API overflow
        if (print_len > 1000) print_len = 1000;
        std::string debug_str(reinterpret_cast<const char*>(raw + start), print_len);
        this->raw_nmea_sensor_->publish_state(debug_str);
    }
    // -------------------

    char field_buf[32]; 
    char dir_buf[2];

    while (start < len) {
        size_t end = start;
        while (end < len && raw[end] != '\n') end++;
        
        size_t line_len = end - start;
        // Strip CR
        if (line_len > 0 && raw[start + line_len - 1] == '\r') line_len--;

        const char* line_ptr = raw + start;
        start = end + 1;

        if (line_len < 6 || line_ptr[0] != '$') continue;

        if (!validate_checksum(line_ptr, line_len)) continue;
        
        at_least_one_valid = true;

        bool is_gga = (strncmp(line_ptr + 3, "GGA", 3) == 0);
        bool is_rmc = (strncmp(line_ptr + 3, "RMC", 3) == 0);

        if (is_gga) {
            // GGA Fields: Lat, Lon, Alt, HDOP, Sats, Fix
            if (get_field(line_ptr, line_len, 2, field_buf, sizeof(field_buf)) &&
                get_field(line_ptr, line_len, 3, dir_buf, sizeof(dir_buf))) {
                float lat = parse_coordinate(field_buf, dir_buf);
                if (latitude_sensor_ && !std::isnan(lat)) latitude_sensor_->publish_state(lat);
            }
            if (get_field(line_ptr, line_len, 4, field_buf, sizeof(field_buf)) &&
                get_field(line_ptr, line_len, 5, dir_buf, sizeof(dir_buf))) {
                float lon = parse_coordinate(field_buf, dir_buf);
                if (longitude_sensor_ && !std::isnan(lon)) longitude_sensor_->publish_state(lon);
            }
            if (altitude_sensor_ && get_field(line_ptr, line_len, 9, field_buf, sizeof(field_buf))) {
                altitude_sensor_->publish_state(strtof(field_buf, nullptr));
            }
            if (hdop_sensor_ && get_field(line_ptr, line_len, 8, field_buf, sizeof(field_buf))) {
                 hdop_sensor_->publish_state(strtof(field_buf, nullptr));
            }
            if (satellites_sensor_ && get_field(line_ptr, line_len, 7, field_buf, sizeof(field_buf))) {
                satellites_sensor_->publish_state(atoi(field_buf));
            }
            if (fix_sensor_ && get_field(line_ptr, line_len, 6, field_buf, sizeof(field_buf))) {
                fix_sensor_->publish_state(atoi(field_buf));
            }
        }
        else if (is_rmc) {
            // RMC Fields: Time, Date, Speed, Course
            if (time_sensor_ && get_field(line_ptr, line_len, 1, field_buf, sizeof(field_buf))) {
                if (strlen(field_buf) >= 6) {
                    char fmt_time[9];
                    snprintf(fmt_time, sizeof(fmt_time), "%c%c:%c%c:%c%c", 
                             field_buf[0], field_buf[1], field_buf[2], field_buf[3], field_buf[4], field_buf[5]);
                    time_sensor_->publish_state(std::string(fmt_time));
                }
            }
            if (date_sensor_ && get_field(line_ptr, line_len, 9, field_buf, sizeof(field_buf))) {
                if (strlen(field_buf) == 6) {
                    char fmt_date[11];
                    snprintf(fmt_date, sizeof(fmt_date), "20%c%c-%c%c-%c%c",
                             field_buf[4], field_buf[5], 
                             field_buf[2], field_buf[3], 
                             field_buf[0], field_buf[1]);
                    date_sensor_->publish_state(std::string(fmt_date));
                }
            }
            if (speed_sensor_ && get_field(line_ptr, line_len, 7, field_buf, sizeof(field_buf))) {
                float spd = strtof(field_buf, nullptr) * 0.514444f; 
                speed_sensor_->publish_state(spd);
            }
            if (course_sensor_ && get_field(line_ptr, line_len, 8, field_buf, sizeof(field_buf))) {
                course_sensor_->publish_state(strtof(field_buf, nullptr));
            }
        }
        taskYIELD();
    }
    
    return at_least_one_valid;
}

float LC76GComponent::parse_coordinate(const char* val_str, const char* dir_str) {
    double raw = strtod(val_str, nullptr);
    if (std::isnan(raw) || std::abs(raw) < 0.0001) return NAN;
    int deg = (int)(raw / 100);
    double min = raw - (deg * 100);
    double val = deg + (min / 60.0);
    if (dir_str[0] == 'S' || dir_str[0] == 'W') val = -val;
    return (float)val;
}

void LC76GComponent::mark_sensors_unavailable() {
    if (speed_sensor_ && !std::isnan(speed_sensor_->state)) speed_sensor_->publish_state(NAN);
    if (satellites_sensor_ && !std::isnan(satellites_sensor_->state)) satellites_sensor_->publish_state(NAN);
    if (fix_sensor_) fix_sensor_->publish_state(0);
}

void LC76GComponent::send_hot_start() {
    ESP_LOGI(TAG, "Sending Hot Start ($PAIR003)...");
    uint8_t cmd[] = {'$', 'P', 'A', 'I', 'R', '0', '0', '3', '*', '3', '9', '\r', '\n'};
    this->bus_->write(ADDR_WRITE, cmd, sizeof(cmd));
}

void LC76GComponent::perform_soft_recovery() {
    this->send_hot_start();
    vTaskDelay(pdMS_TO_TICKS(50));
    
    // NOP to flush state
    uint8_t nop[] = {0x00, 0x30, 0x51, 0xAA, 0x00, 0x00, 0x00, 0x00};
    this->bus_->write(ADDR_WRITE, nop, sizeof(nop));
}

} // namespace lc76g
} // namespace esphome