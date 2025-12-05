# ESPHome LC76G GNSS I²C Component

[![ESPHome](https://img.shields.io/badge/ESPHome-Component-red)](https://esphome.io/)
[![Protocol](https://img.shields.io/badge/Protocol-I2C%20Split%20Addr-blue)](https://www.quectel.com/)

A high-reliability custom component for the **Quectel LC76G** GNSS receiver communicating over the **proprietary I²C transport** layer.

Unlike standard UART GPS modules, the I²C version of this module requires a complex polling handshake with split read/write addresses (`0x50` / `0x54`) and strict timing. This component handles the protocol complexity, buffering, and error recovery transparently.

---

## ⚠️ Hardware Compatibility Note

This component has been developed and tested **exclusively** on the following configuration:

* **Board:** Waveshare ESP32-S3 1.75" Touch LCD (OLED)
* **GPS Module:** Quectel LC76G
* **Antenna:** Active Ceramic Antenna

**Note:** Other hardware combinations, different ESP32 boards, or passive antennas have **not** been tested and may require different tuning or power configurations.

---

## ✨ Key Features

* **Zero-Copy Architecture:** efficient in-place parsing of NMEA streams (no heap fragmentation).
* **Protocol Compliance:** implements the required "Write 0x50 -> Read 0x54 -> Ack 0x50 -> Read Body" sequence.
* **Auto-Recovery:** detects "NACK" loops or data corruption and automatically triggers a **Soft Recovery** (Hot Start) to unfreeze the module without power cycling.
* **FIFO Self-Healing:** protects against buffer desynchronization by enforcing capped reads even on corrupted headers.
* **Smart Crop:** prevents CPU starvation on ESP32 by processing only the latest data if the buffer overflows (>2KB).
* **Native Integration:** exposes all data as standard ESPHome sensors (Latitude, Longitude, Speed, Time, Date, etc.).

-----

## ⚙️ Configuration

### Basic Example

```yaml
external_components:
  - source:
      type: git
      url: https://github.com/fuzzybear62/esphome-lc76g
    components: [ lc76g ]

i2c:
  sda: GPIO15
  scl: GPIO14
  scan: true
  frequency: 400kHz  # Recommended for best performance

lc76g:
  id: my_gps
  i2c_id: bus_a      # Optional if you have multiple I2C buses
  update_interval: 2s
  
  # Set to true to see raw NMEA strings in the "GPS Raw Data" sensor
  debug: false       

  latitude_sensor:
    name: "GPS Latitude"
    accuracy_decimals: 6
    filters:
      - sliding_window_moving_average:
          window_size: 3
          send_every: 1

  longitude_sensor:
    name: "GPS Longitude"
    accuracy_decimals: 6
    filters:
      - sliding_window_moving_average:
          window_size: 3
          send_every: 1

  speed_sensor:
    name: "GPS Speed"
    unit_of_measurement: "km/h"
    accuracy_decimals: 1
    filters:
      - multiply: 3.6  # Convert m/s to km/h

  satellites_sensor:
    name: "GPS Satellites"

  fix_sensor:
    name: "GPS Fix Status"
```

### Full Sensor List

| YAML Key | Description | Type |
| :--- | :--- | :--- |
| `latitude_sensor` | Latitude in degrees | float |
| `longitude_sensor` | Longitude in degrees | float |
| `altitude_sensor` | Altitude above MSL (meters) | float |
| `speed_sensor` | Speed over ground (m/s) | float |
| `course_sensor` | Course over ground (degrees) | float |
| `hdop_sensor` | Horizontal Dilution of Precision | float |
| `satellites_sensor` | Number of satellites in view | int |
| `fix_sensor` | Fix Quality (0=Invalid, 1=GPS, 2=DGPS) | int |
| `date_sensor` | UTC Date (YYYY-MM-DD) | string |
| `time_sensor` | UTC Time (HH:MM:SS) | string |
| `raw_nmea_sensor` | Raw NMEA sentences (debug only) | string |

-----

## 🛠 Actions & Automations

You can force a **Hot Start** manually (e.g., via a button in Home Assistant) if the GPS takes too long to fix.

```yaml
button:
  - platform: template
    name: "GPS Force Hot Start"
    icon: "mdi:refresh"
    on_press:
      - lambda: |-
          id(my_gps).send_hot_start();
```

-----

## 🧠 Technical Details

### The Split-Address Protocol

The LC76G I²C bridge does not behave like a standard sensor. It uses two addresses:

  * **`0x50` (Write Address):** Used for commands and handshakes.
  * **`0x54` (Read Address):** Used to read the FIFO buffer.

**The Polling Cycle:**

1.  **Handshake:** ESP sends "Poll" command to `0x50`.
2.  **Header:** ESP reads 4 bytes from `0x54` (Little Endian Length).
3.  **Ack:** ESP writes the received header back to `0x50` to confirm.
4.  **Body:** ESP reads *Length* bytes from `0x54`.

### Auto-Recovery Strategy

Electrical noise or timing issues can cause the internal FIFO pointer to desynchronize (e.g., the module reports 60,000 bytes available).

  * **Capped Reads:** If length \> 10KB, the component reads 10KB anyway to force the FIFO pointer to advance, preventing deadlocks.
  * **Soft Recovery:** If no valid NMEA data (checksum failure) is received for 5 consecutive cycles, the component automatically sends a **Hot Start** command + NOP sequence to reset the module's state machine.

### Performance Notes

  * **I²C Frequency:**
      * **100 kHz:** Tested and verified to work correctly. It is considered the minimum baseline for operation.
      * **400 kHz:** Strongly recommended. Higher speeds reduce the bus occupancy time when draining large buffers (up to 10KB), improving overall system responsiveness.
  * **Memory:** The component allocates a static 10KB buffer at boot. It does not perform dynamic allocations during the loop, ensuring long-term stability (no heap fragmentation).

-----

## 📝 License

MIT License.