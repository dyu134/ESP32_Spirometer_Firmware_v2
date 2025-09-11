---
description: Repository Information Overview
alwaysApply: true
---

# ESP32 Spirometer Firmware Information

## Summary
This project is an ESP32-based spirometer device firmware that uses magnetic field sensors (MLX90393) to measure breathing patterns. The device communicates via Bluetooth Low Energy (BLE) to send sensor data to client applications. It includes both the ESP32 firmware and a Python client API for data collection and an example game demonstrating the device's capabilities.

## Structure
- **ESP32_Spirometer_Firmware_v2.ino**: Main Arduino firmware file
- **platformio.ini**: PlatformIO configuration for building and uploading
- **SpirometerAPI.py**: Python client API for connecting to the device via BLE
- **Example Game.py**: Flappy Bird-style game using the spirometer as input
- **requirements.txt**: Python dependencies for the client applications
- **venv/**: Python virtual environment
- **.vscode/**: VSCode configuration files for development
- **.pio/**: PlatformIO build directory and dependencies

## Language & Runtime
**Languages**: C++ (Arduino), Python 3.11
**Frameworks**: Arduino Framework, ESP32 BLE
**Build System**: PlatformIO
**Target Hardware**: ESP32-S3 (esp32-s3-devkitc-1)

## Dependencies

### Arduino/C++ Dependencies
**Main Dependencies**:
- Adafruit MLX90393 (v2.0.5): Library for the magnetic field sensors
- Adafruit NeoPixel (v1.8.0): Library for controlling RGB LEDs
- ESP32 BLE libraries: BLEDevice, BLEUtils, BLE2902, BLEServer

### Python Dependencies
**Main Dependencies**:
- bleak (v1.1.1): Bluetooth Low Energy library for Python
- pandas (v2.3.2): Data manipulation and analysis library
- pygame (v2.6.1): Game development library for the example game
- typing_extensions (v4.15.0): Type hinting support
- numpy (v2.2.6): Numerical computing

## Build & Installation

### Arduino Firmware
```bash
# Install PlatformIO CLI or use PlatformIO IDE extension in VSCode
# Build and upload the firmware
pio run -t upload
# Monitor serial output
pio device monitor
```

### Python Client
```bash
# Create and activate virtual environment
python -m venv venv
.\venv\Scripts\activate

# Install required Python packages
pip install -r requirements.txt

# Run the example game
python "Example Game.py"

# Or import and use the Spirometer class
from SpirometerAPI import Spirometer
```

## Hardware Configuration
**Microcontroller**: ESP32-S3
**Memory**: 8MB Flash, 2MB PSRAM (QIO mode)
**Sensors**:
- 2x MLX90393 magnetic field sensors (I2C communication)
  - Sensor 1: I2C pins 48, 34
  - Sensor 2: I2C pins 2, 1
- Battery voltage monitoring via ADC pin 3
**Outputs**:
- NeoPixel RGB LED on pin 47 for visual feedback
- 4x LEDs on pins 38, 37, 36, 35 for battery level indication

## Communication
**Protocol**: Bluetooth Low Energy (BLE)
**Device Name**: sensor_MIRS
**Services**:
- Main data service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
- Battery service UUID: 0000180F-0000-1000-8000-00805F9B34FB
**Characteristics**:
- Sensor data characteristic: beb5483e-36e1-4688-b7f5-ea07361b26a8
- Battery level characteristic: 00002A19-0000-1000-8000-00805F9B34FB

## Features
- Real-time magnetic field measurement from dual sensors
- Differential measurement for improved accuracy
- Battery level monitoring and indication
- BLE connectivity for wireless data transmission
- Visual feedback via RGB LED
- Python client API for data collection and analysis
- Example game demonstrating real-time control using the spirometer