# 7Semi_ICM20948
### Arduino Library for ICM-20948 9-Axis Motion Sensor  
![Arduino](https://img.shields.io/badge/platform-arduino-blue.svg)
![License](https://img.shields.io/badge/license-MIT-green.svg)
![Status](https://img.shields.io/badge/status-active-brightgreen.svg)

---

## 🧭 Overview
**7Semi_ICM20948** is a lightweight Arduino library for the **TDK ICM-20948** 9-axis IMU.  
It supports both **I²C and SPI** interfaces with flexible configuration, including DLPF, ODR, full-scale selection, and sensor gating.

---

## ⚙️ Features
- I²C (400 kHz+) and SPI (Mode 0) support  
- Accelerometer, Gyroscope, Magnetometer, and Temperature readouts  
- Built-in scale conversion to physical units (g, dps, µT, °C)  
- DLPF (Digital Low-Pass Filter) configuration  
- Adjustable output data rates (ODR)  
- Self-test, decimation (DEC3), and averaging options  

---

# 🔌 Wiring

## SPI Connection
### Wiring (SPI - Arduino UNO)
  ---------------------------
 * - SCLK        → D13
 * - MISO (SDO)  → D12
 * - MOSI (SDI)  → D11
 * - CS          → D10
 * - INT         → D2, optional
 * - 3V3         → 3.3V
 * - GND         → GND


### Wiring (SPI - ESP32 VSPI)
  ---------------------------
 * - SCLK        → GPIO18
 * - MISO (SDO)  → GPIO19
 * - MOSI (SDI)  → GPIO23
 * - CS          → GPIO5
 * - INT         → any GPIO, optional
 * - 3V3         → 3.3V
 * - GND         → GND

## I2C Connection 

### Wiring (I2C - Arduino UNO)
  ---------------------------
 * - SCL  → A5
 * - SDA  → A4
 * - INT  → D2, optional
 * - 3V3  → 3.3V
 * - GND  → GND
 

### Wiring (I2C - ESP32)
 ---------------------------
 * - SCL  → GPIO22
 * - SDA  → GPIO21
 * - INT  → any GPIO, optional
 * - 3V3  → 3.3V
 * - GND  → GND
 

## 🔍 Notes

- **Magnetometer (AK09916)**  
  - The magnetometer is connected internally to the ICM-20948 via an auxiliary I²C master interface.  
  - It works automatically when you call `readMag()`.  
  - If your board exposes **AUX_DA / AUX_CL (SDA/SCL)** pins, you can also connect them to your MCU I²C bus and enable bypass mode.  

- **Voltage Levels**  
  - Always use **3.3 V logic**.  
  - The ICM-20948 is **not 5 V tolerant** unless your breakout includes level shifting.  

- **SPI Mode**  
  - The device operates in **SPI Mode 0 (CPOL = 0, CPHA = 0)**.  
  - SPI clock is **1000000 Hz**.  

- **I²C Mode**  
  - Default address is **0x68**; if AD0 = HIGH, it becomes **0x69**.  
  - Use `imu.begin(Wire, 0x68)` or `imu.begin(Wire, 0x69)` accordingly.  

- **Sensor Orientation**  
  - Axes follow the right-hand convention:
    - +X → right - left
    - +Y → forward - backward
    - +Z → up - down 
  - Use `invertAxes()` if you need to match your board’s orientation.  

- **Temperature Output**  
  - The temperature sensor reports **die temperature**, not ambient.  
  - Typical offset is around **+10 °C** compared to the environment.  

- **Power Notes**  
  - Typical active current: **~3.5 mA** (Accel + Gyro)  
  - Use `setSensors()` to disable unused sensors to save power.  

- **Best Practices**  
  - Always call `applyBasicDefaults()` after `begin()`.  
  - Add a short delay (~100 ms) after `begin()` before reading data.  
  - For very stable readings, apply simple averaging in software (e.g. 10-sample moving average).

- **Interface bring-up**
  - `SPI.begin()` and `Wire.begin()` must match your board type.  
  - On **Arduino UNO / AVR**: use default hardware pins only.  
  - On **ESP32**: call `SPIClass SPI_ESP32(VSPI); SPI_ESP32.begin(SCK, MISO, MOSI, CS);` and  
      `Wire.begin(SDA, SCL);` before `imu.begin()`.  
