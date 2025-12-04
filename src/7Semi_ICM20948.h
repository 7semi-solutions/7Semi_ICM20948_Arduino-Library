/**
 * ICM20948_7Semi.h
 * - Public Arduino C++ header for ICM-20948 (I2C-only in this release)
 *
 * Library Info
 * - Name: ICM20948_7Semi
 * - Version: 0.1.1 (2025-10-24)
 * - Author: 7Semi / Evelta
 * - License: MIT
 * - Transport: I2C @100kHz+
 * - Dependencies: <Arduino.h>, <Wire.h>, "ICM20948_REGS.h"
 */

#ifndef ICM20948_7SEMI_H
#define ICM20948_7SEMI_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "ICM20948_REGS.h"

/**
 * Class
 * - Manages ICM-20948 over I2C (SPI path disabled in this cut)
 * - Caches scale factors for LSB->physical conversions
 */
class ICM20948_7Semi {
public:
  /** - Bus mode selector (internal) */
  enum Mode : uint8_t { MODE_I2C = 0,
                        MODE_SPI = 1 };

  /** - Construct with defaults; call begin() to attach bus */
  ICM20948_7Semi();

  /**
   * begin (I2C)
   * - Stores TwoWire ref, device address; probes WHO_AM_I; applies defaults
   * - Returns true on success
   * - Params:
   *   - wirePort: Wire instance (e.g., Wire)
   *   - address:  I2C addr 0x68 (AD0=0) or 0x69 (AD0=1)
   */
  bool begin(TwoWire &wirePort, uint8_t address = 0x68);
  bool begin(SPIClass &spiPort, uint8_t csPin);

  /**
   * applyBasicDefaults
   * - Wake, enable accel/gyro, DLPF on, max FS, sane INT cfg
   */
  bool applyBasicDefaults();

  /**
   * readAccel
   * - Read accelerometer in g
   * - Params:
   *   - x: out g on X
   *   - y: out g on Y
   *   - z: out g on Z
   */
  bool readAccel(float &x, float &y, float &z);

  /**
   * readGyro
   * - Read gyroscope in dps
   * - Params:
   *   - x: out dps on X
   *   - y: out dps on Y
   *   - z: out dps on Z
   */
  bool readGyro(float &x, float &y, float &z);

  /**
   * readTemperature
   * - Read temperature in °C
   * - Params:
   *   - t: out temperature (°C)
   */
  bool readTemperature(float &t);

  /**
   * readMag
   * - Read AK09916 via bypass; returns microtesla
   * - Params:
   *   - x: out uT on X
   *   - y: out uT on Y
   *   - z: out uT on Z
   */
  bool readMag(float &x, float &y, float &z);

  /**
   * readWhoAmI
   * - Return 0xEA on success, 0x00 if bus/power state blocks read
   */
  uint8_t readWhoAmI();

  /**
 * softReset
 * - Device soft reset; reapply configuration after call
 */
  bool softReset();

  /**
 * sleep
 * - Enter/exit sleep
 * - Params:
 *   - en: true to sleep; false to wake (CLKSEL kept = 1)
 */
  bool sleep(bool en);

  /**
 * lowPower
 * - Toggle low-power mode and duty-cycling
 * - Params:
 *   - LP_EN: enable low-power bit in PWR_MGMT_1
 *   - ACCEL_CYCLE: duty-cycle accelerometer
 *   - GYRO_CYCLE: duty-cycle gyroscope
 */
  bool lowPower(bool LP_EN, bool ACCEL_CYCLE, bool GYRO_CYCLE);

  /**
 * setClkSel
 * - Select clock source (0..7)
 * - Params:
 *   - c: CLKSEL value; use 1 for auto PLL
 */
  bool setClkSel(uint8_t c);

  /**
 * Gyro_SMPLRT
 * - Set gyro ODR (Hz) for DLPF-enabled path (base 1100 Hz)
 * - Params:
 *   - rate_hz: desired rate (>= 4.3 Hz)
 */
  bool Gyro_SMPLRT(float rate_hz);

  /**
 * GyroConfig
 * - Configure gyro DLPF/FS/FCHOICE and axis/averaging
 * - Params:
 *   - DLPFCFG: 0..7 filter config
 *   - FS_SEL:  0..3 full-scale (±250..±2000 dps)
 *   - FCHOICE: 0=DLPF on, 1=bypass (per datasheet mapping)
 *   - XGYRO/YGYRO/ZGYRO: true to enable axis
 *   - AVGCFG: averaging 0..7 (see datasheet)
 */
  bool GyroConfig(uint8_t DLPFCFG, uint8_t FS_SEL, bool FCHOICE,
                  bool XGYRO, bool YGYRO, bool ZGYRO, uint8_t AVGCFG);

  /**
 * Accel_SMPLRT
 * - Set accel ODR (Hz) for DLPF-enabled path (base 1125 Hz)
 * - Params:
 *   - rate_hz: desired rate (1..1125 Hz)
 */
  bool Accel_SMPLRT(uint16_t rate_hz);

  /**
 * AccelConfigure
 * - Configure accel DLPF/FS/DEC3 and optional self-test
 * - Params:
 *   - DLPF: 0..7 filter config
 *   - FS_SEL: 0..3 full-scale (±2..±16 g)
 *   - dlpf_enable: true to enable DLPF
 *   - dec3: decimator (0..3)
 *   - stX/stY/stZ: enable per-axis self-test
 */
  bool AccelConfigure(uint8_t DLPF, uint8_t FS_SEL,
                      bool dlpf_enable, uint8_t dec3,
                      bool stX, bool stY, bool stZ);

  /**
   * setSensors
   * - Enable/disable accel, gyro, temp
   * - Params:
   *   - accel_on: true enable accel
   *   - gyro_on:  true enable gyro
   *   - temp_on:  true enable temp (affects TEMP_DIS)
   */
  bool setSensors(bool accel_on, bool gyro_on, bool temp_on);

  /**
   * Offsets
   * - write/read gyro offsets (3x int16)
   * - write/read accel offsets (3x int16)
   * - Params:
   *   - off/out: arrays [3] order X,Y,Z
   */
  bool writeGyroOffset(const int16_t off[3]);
  bool readGyroOffset(int16_t out[3]);
  bool writeAccelOffset(const int16_t off[3]);
  bool readAccelOffset(int16_t out[3]);

  /**
   * Unified R/W (dispatches to active bus)
   * - read(devAddr, reg, buf, len)
   * - write(devAddr, reg, buf, len)
   */
  bool read(uint8_t devAddr, uint8_t reg, uint8_t *buf, size_t len);
  bool write(uint8_t devAddr, uint8_t reg, const uint8_t *buf, size_t len);

private:
  /**
   * selectBank / enableBypass
   * - selectBank(bank): choose USER BANK 0..3
   * - enableBypass(en): true routes host I2C to AK09916
   */
  bool selectBank(uint8_t bank);
  bool enableBypass(bool en);

  /**
   * I2C helpers
   * - i2cWrite(addr, reg, buf, len)
   * - i2cRead(addr, reg, buf, len)
   */
  bool i2cWrite(uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len);
  bool i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len);

  /**
   * SPI helpers (declared for completeness; SPI begin disabled in this cut)
   * - spiWrite(reg, buf, len)
   * - spiRead(reg, buf, len)
   */
  bool spiWrite(uint8_t reg, const uint8_t *buf, size_t len);
  bool spiRead(uint8_t reg, uint8_t *buf, size_t len);

  /**
   * Single-byte helpers at current address
   * - writeReg(reg, val)
   * - readReg(reg, val)
   */
  bool writeReg(uint8_t reg, uint8_t val);
  bool readReg(uint8_t reg, uint8_t &val);

private:
  Mode _mode = MODE_I2C;     // current bus
  uint8_t _csPin = 0xFF;     // SPI CS (unused for I2C)
  TwoWire *_wire = nullptr;  // Wire handle
  SPIClass *_spi = nullptr;  // SPI handle
  uint8_t _i2cAddr = 0x69;   // device I2C address

  /** - Cached scales for unit conversion */
  float _accelMgPerLSB = 2048.0f;  // LSB/g at ±16g
  float _gyroDpsPerLSB = 16.384f;  // LSB/dps at ±2000 dps
};

#endif /* ICM20948_7SEMI_H */
