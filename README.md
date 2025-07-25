# ACS Teensy Flight Software (2021-2022)

This repository contains the flight software for the ACS (Altitude Control System) project, developed for the 2021-2022 season. The codebase is designed for Teensy 4.0 microcontrollers and integrates sensor data acquisition, flight state management, logging, and communications for high-power rocket flights.

## Features

- **Sensor Integration:** Supports BMP390 (pressure/temperature), BNO055 (IMU), and Adafruit GPS modules.
- **Flight State Management:** Implements launch, ascent, apogee, descent, and landing detection.
- **Data Logging:** Real-time logging to SD card, including backup and main flight data.
- **Radio Communication:** Handles requests and telemetry via XBee radio and ground station serial protocols.
- **BeagleBone Blue Support:** Serial communication for external altitude control and flight state synchronization.
- **Event Logging:** Tracks flight events, sensor status, and system errors.

## Directory Structure

```
.
├── docs/                  # Documentation files
├── lib/                   # Custom libraries
├── main/                  # Main application
│   ├── main.ino           # Arduino sketch
│   └── ...                 # Other source files
├── test/                  # Unit tests and scripts
│   ├── flight_state_test/ # Flight state management tests
│   └── ...                 # Other tests
└── README.md              # Project overview
```

## Getting Started

### Prerequisites

- [Teensyduino](https://www.pjrc.com/teensy/teensyduino.html)
- Arduino IDE or PlatformIO
- Teensy 4.0 hardware
- BMP390, BNO055, Adafruit GPS modules
- SD card breakout board

### Build & Flash

1. Open `main/main.ino` in Arduino IDE.
2. Select Teensy 4.0 as the target board.
3. Install required libraries (Adafruit BMP3XX, BNO055, GPS, SD).
4. Compile and upload to your Teensy device.

### Running Tests

Unit tests and data processing scripts are located in the `test/` directory. Use `g++` to compile C++ test files:

```sh
cd test/flight_state_test
g++ -std=c++11 flight_state_test.cpp -o flight_state_test
./flight_state_test
```
