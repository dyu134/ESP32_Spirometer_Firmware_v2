# ESP32 Spirometer Firmware & Python Client

## Overview

This project provides firmware and a Python API for an ESP32-based spirometer device. The device uses dual MLX90393 magnetic field sensors to measure breathing patterns and streams data via Bluetooth Low Energy (BLE). The repository includes:

- **ESP32 Arduino firmware** for sensor reading and BLE communication
- **Python client API** for data collection, calibration, and analysis
- **Example game** demonstrating real-time control using the spirometer

---

## Repository Structure

- `ESP32_Spirometer_Firmware_v2.ino` — Main Arduino firmware for ESP32
- `SpirometerAPI.py` — Python BLE client API for the spirometer
- `Example Game.py` — Flappy Bird-style game using the spirometer as input
- `requirements.txt` — Python dependencies
- `.vscode/`, `.pio/`, `venv/` — Development and build environment files

---

## Hardware

- **Microcontroller:** ESP32-S3 (esp32-s3-devkitc-1)
- **Sensors:** 2x MLX90393 magnetic field sensors (I2C)
  - Sensor 1: I2C pins 48, 34
  - Sensor 2: I2C pins 2, 1
- **Battery monitoring:** ADC pin 3
- **Outputs:**
  - NeoPixel RGB LED (pin 47)
  - 4x LEDs (pins 38, 37, 36, 35) for battery level

---

## Firmware Features

- Reads magnetic field data from two MLX90393 sensors
- Streams sensor data via BLE using three characteristics:
  - `c1_char`: x1, z1
  - `c2_char`: x2, z2
  - `y_char`: y1, y2
- Battery level monitoring and BLE reporting
- Visual feedback via NeoPixel and battery LEDs

---

## Python Client Features (`SpirometerAPI.py`)

- Connects to ESP32 via BLE and streams sensor data from three characteristics
- Combines sensor data into a single sample:
  - `dx = x1 - x2`
  - `dy = y1 + y2`
  - `dz = z1 + z2`
- Calibrates offsets for accurate measurement
- Stability detection using a time window and standard deviation thresholds
- Drift compensation for long-term stability
- Exports calibrated data to CSV or pandas DataFrame
- Reads battery level from BLE
- Thread-safe sample storage
- Real-time access to latest calibrated sample

---

## Example Game

- Flappy Bird-style game using the spirometer's X-axis as input
- Maps calibrated X value (`cal_x`) to a "flap" action
- Keyboard fallback for testing without hardware
- Displays battery level and calibration status

---

## BLE Services & Characteristics

- **Main Data Service UUID:** `4fafc201-1fb5-459e-8fcc-c5c9c331914b`
  - `c1_char_uuid`: `beb5483e-36e1-4688-b7f5-ea07361b26a8` (x1, z1)
  - `c2_char_uuid`: `1f9803c5-26c3-41c4-93a9-7358baa3f1c2` (x2, z2)
  - `y_char_uuid`: `55a61982-180d-40c4-9c7c-437514c66290` (y1, y2)
- **Battery Service UUID:** `0000180F-0000-1000-8000-00805F9B34FB`
  - `battery_char_uuid`: `00002A19-0000-1000-8000-00805F9B34FB`

---

## Installation & Usage

### Arduino Firmware

```sh
# Install PlatformIO CLI or use PlatformIO IDE extension in VSCode
pio run -t upload
pio device monitor
```

### Python Client

```sh
python -m venv venv
.\venv\Scripts\activate
pip install -r requirements.txt
python "Example Game.py"
```

Or use the API directly:

```python
from SpirometerAPI import Spirometer

api = Spirometer(device_id="YOUR_DEVICE_ID")
api.connect()
print(api.get_latest_calibrated_x())
api.disconnect()
```

---

## Requirements

- **Python:** 3.11+
- **Libraries:** bleak, pandas, pygame, typing_extensions

---

## References

- [Adafruit MLX90393 Library](https://github.com/adafruit/Adafruit_MLX90393)
- [bleak Python BLE Library](https://github.com/hbldh/bleak)
- [pandas Data Analysis Library](https://pandas.pydata.org/)
- [pygame Game Library](https://www.pygame.org/)

---

## License

See repository for license details.

---
