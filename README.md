# MPU6050 Data Logger with ESP-IDF and FreeRTOS

This project implements a complete embedded firmware pipeline for interfacing with the MPU6050 IMU using ESP-IDF.

It includes:
- Low-level I2C driver for MPU6050
- Raw sensor data acquisition using burst reads
- Conversion to physical units (g, °/s, °C)
- Real-time data logging at 100 Hz using FreeRTOS

The focus of this project is on clean driver design, timing control, and fault-tolerant data acquisition.

---

## Features

- **100 Hz deterministic sampling**
- **Modular architecture** – separate driver, processing, task, and logger layers
- **I²C burst read** of all 6 axes + temperature (14 bytes)
- **Automatic error recovery – retries for transient I2C failures and reinitialization after persistent errors
- **CSV output via `printf`** – no extra metadata, ready for logging or real‑time analysis
- **Configurable** – sample rate, filter order (optional moving average), I²C timeout

---

## System Architecture

MPU6050 → I2C Driver → Raw Data → Processing Layer → Logger → UART

- **Driver Layer**
  - Handles register reads/writes
  - Provides raw sensor values

- **Processing Layer**
  - Converts raw values to physical units
  - Optional moving average filtering

- **Application Layer (FreeRTOS Task)**
  - Maintains fixed sampling rate (100 Hz)
  - Implements retry and recovery logic
  - Sends processed data to logger
 
---

## Key Design Decisions

- **100 Hz Sampling Rate**
  Chosen based on MPU6050 DLPF bandwidth (~44 Hz) to satisfy Nyquist criteria while maintaining clean timing (10 ms period).

- **Burst Read (14 bytes)**
  Ensures all sensor values are read from the same sampling instant.

- **Driver–Processing Separation**
  Driver handles only register-level I2C communication, while conversion and filtering are done in a separate processing layer.

- **Fault Handling Strategy**
  Read operations are retried to handle transient I2C failures. Persistent errors trigger sensor reinitialization, with a short cooldown period before resuming logging.

- **Deterministic Task Timing**
  `vTaskDelayUntil` is used to maintain a fixed sampling rate instead of variable delays.

---

## Hardware

| Component | Connection (ESP32) |
|-----------|-------------------|
| MPU6050   | SDA → GPIO21, SCL → GPIO22 |
| Power     | 3.3 V, GND |

*Adjust the pins in `imu_driver.c` if needed.*

---

## Key Learnings

- Designing reusable embedded drivers
- Handling I2C communication at register level
- Implementing fault-tolerant firmware systems
- Managing timing constraints in FreeRTOS
- Separating hardware interface and application logic
