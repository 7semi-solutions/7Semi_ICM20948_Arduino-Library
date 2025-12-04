/*
 * ICM20948_7Semi.cpp
 * - Arduino driver source for ICM-20948 over I2C/SPI
 *
 * Library Info
 * - Name: ICM20948_7Semi
 * - Version: 0.1.1 (2025-10-24)
 * - Author: 7Semi / Evelta
 * - Maintainer: 7Semi Embedded (@tech02)
 * - License: MIT
 * - Targets: AVR, SAMD, ESP32, nRF52, RP2040 (Arduino cores)
 * - Transports: I2C @400kHz+, SPI Mode 0 (up to device limits)
 * - Dependencies: Wire.h, SPI.h (as applicable)
 * - Tested: Arduino IDE 2.x, PlatformIO (GCC >= 10)
 * - Threading: Not thread-safe; call from a single task/context
 * - Endianness: Device big-endian register pairs; driver converts to host
 *
 * Features
 * - Simple bring-up: begin(I2C/SPI) probes WHO_AM_I, applies sane defaults
 * - Bank helpers, bypass control to reach AK09916 magnetometer directly
 * - Read helpers for accel/gyro/temp/mag with cached scale factors
 * - Power: sleep, low-power duty cycling, per-sensor gate control
 * - Rate/Filter: sample-rate, DLPF, full-scale selection
 * - Offset R/W for accel and gyro
 * - Optional DMP bridge stubs (kept commented for lean builds)
 *
 * Wiring (typical I2C @ 0x68)
 * - VCC -> 3V3
 * - GND -> GND
 * - SCL -> MCU SCL (w/ pull-up 4.7k)
 * - SDA -> MCU SDA (w/ pull-up 4.7k)
 * - INT -> optional MCU GPIO (active low by default)
 * - AD0 -> GND (addr 0x68) or VCC (addr 0x69)
 *
 * Wiring (typical SPI)
 * - VCC -> 3V3, GND -> GND
 * - SCLK/MOSI/MISO per MCU pins
 * - CS   -> user GPIO (define in begin(spi, cs))
 * - INT  -> optional MCU GPIO
 *
 * Quickstart (I2C)
 * - Create instance
 * - call begin(Wire, 0x68)
 * - call applyBasicDefaults() (done by begin on success)
 * - call readAccel/readGyro/readMag/readTemperature()
 *
 * Minimal Example
 * - #include <Wire.h>
 * - ICM20948_7Semi icm;
 * - void setup(){ Serial.begin(115200); icm.begin(Wire, 0x68); }
 * - void loop(){ float ax,ay,az; if(icm.readAccel(ax,ay,az))
 *
 * Changelog
 * - 0.1.1: Expanded file/documentation comments; clarified scale caching and guards
 * - 0.1.0: Initial public draft
 */

#include "7Semi_ICM20948.h"

/** - Construct with default I2C address; call begin() to attach bus */
ICM20948_7Semi::ICM20948_7Semi()
  : _mode(MODE_I2C), _csPin(0xFF), _wire(nullptr), _spi(nullptr), _i2cAddr(0x68) {}

/* ================= Begin (I2C) ================= */
/**
 * Begin (I2C)
 * - Stores TwoWire reference and target address
 * - Probes WHO_AM_I and applies basic defaults
 * - Returns true if WHO_AM_I matches and defaults succeed
 */
bool ICM20948_7Semi::begin(TwoWire &wirePort, uint8_t address) {
  _mode = MODE_I2C;
  _wire = &wirePort;
  _i2cAddr = address;
  _wire->begin();
  uint8_t who = readWhoAmI();
  if (who != WHO_AM_I_VAL) {
    return false;
  }
  return applyBasicDefaults();
}

/* ================= Begin (SPI) ================= */
/**
 * Begin (SPI)
 * - Stores SPI reference and CS pin, initializes CS high
 * - Probes WHO_AM_I and applies basic defaults
 * - Keep commented until SPI transport is enabled for this build
 */
bool ICM20948_7Semi::begin(SPIClass &spiPort, uint8_t csPin) {
  _mode = MODE_SPI;
  _spi = &spiPort;
  _csPin = csPin;

  // Setup CS and SPI peripheral
  pinMode(_csPin, OUTPUT);
  digitalWrite(_csPin, HIGH);  // idle high
  _spi->begin();
  delay(2);  // small settle

  // Ensure BANK 0 for control regs
  if (!selectBank(0)) return false;

  // SPI requirement: disable I2C interface (USER_CTRL.I2C_IF_DIS = 1)
  uint8_t uc = 0;
  if (readReg(USER_CTRL, uc)) {
    if ((uc & 0x10) == 0) {  // bit4
      uc |= 0x10;
      (void)writeReg(USER_CTRL, uc);
      delay(1);
    }
  } else {
    // If readReg failed (fresh power?), try writing the bit anyway
    uc = 0x10;
    (void)writeReg(USER_CTRL, uc);
    delay(1);
  }

  // Wake + set clock (best-effort; don’t fail if this part can’t read yet
  uint8_t pwr1 = 0;
  if (readReg(PWR_MGMT_1, pwr1)) {
    bool changed = false;
    if (pwr1 & 0x40) {
      pwr1 &= ~0x40;
      changed = true;
    }                             // SLEEP=0
    if ((pwr1 & 0x07) != 0x01) {  // CLKSEL=1
      pwr1 = uint8_t((pwr1 & ~0x07) | 0x01);
      changed = true;
    }
    if (changed) {
      (void)writeReg(PWR_MGMT_1, pwr1);
      delay(10);
    }
  } else {
    // Fallback: just try to write CLKSEL=1, SLEEP=0
    (void)writeReg(PWR_MGMT_1, 0x01);
    delay(10);
  }

  // Probe WHO_AM_I via bus-specific read (more robust early on)
  uint8_t who = 0x00;
  for (uint8_t i = 0; i < 3; ++i) {
    if (spiRead(WHO_AM_I, &who, 1)) break;
    delay(2);
  }
  if (who != WHO_AM_I_VAL) {
    return false;
  }

  // Apply defaults (DLPF on, max FS, INT config, etc.)
  return applyBasicDefaults();
}



/* ================= WHO_AM_I ================= */
/**
 * readWhoAmI
 * - Ensures BANK 0, wakes device (SLEEP=0) and selects CLKSEL=1
 * - Tries up to 3 reads; returns 0x00 on failure
 */
uint8_t ICM20948_7Semi::readWhoAmI() {
  // Ensure USER BANK 0 for WHO_AM_I
  if (!selectBank(0)) return 0x00;

  // Wake + set clock (works for both I2C and SPI via readReg/writeReg)
  uint8_t pwr1 = 0;
  if (readReg(PWR_MGMT_1, pwr1)) {
    bool changed = false;
    if (pwr1 & (1u << 6)) {  // SLEEP bit
      pwr1 &= ~(1u << 6);    // SLEEP = 0
      changed = true;
    }
    if ((pwr1 & 0x07) != 0x01) {              // CLKSEL != 1?
      pwr1 = uint8_t((pwr1 & ~0x07) | 0x01);  // CLKSEL = 1 (auto PLL)
      changed = true;
    }
    if (changed) {
      (void)writeReg(PWR_MGMT_1, pwr1);
      delay(10);
    }
  }

  // Try a few times (readReg dispatches to I2C or SPI)
  for (uint8_t i = 0; i < 3; ++i) {
    uint8_t val = 0x00;
    if (readReg(WHO_AM_I, val))  // expect 0xEA
      return val;
    delay(2);
  }
  return 0x00;
}


/* ================= I2C helpers ================= */
/**
 * i2cWrite
 * - Write N bytes to device addr/reg
 * - Returns true if endTransmission() reports success
 */
bool ICM20948_7Semi::i2cWrite(uint8_t addr, uint8_t reg, const uint8_t *buf, size_t len) {
  _wire->beginTransmission(addr);
  _wire->write(reg);
  _wire->write(buf, len);
  return (_wire->endTransmission() == 0);
}

/**
 * i2cRead
 * - Read N bytes from device addr/reg
 * - Uses repeated start for combined transaction
 */
bool ICM20948_7Semi::i2cRead(uint8_t addr, uint8_t reg, uint8_t *buf, size_t len) {
  _wire->beginTransmission(addr);
  _wire->write(reg);
  if (_wire->endTransmission(false) != 0) return false;
  _wire->requestFrom((int)addr, (int)len);
  for (size_t i = 0; i < len; ++i) {
    if (!_wire->available()) return false;
    buf[i] = _wire->read();
  }
  return true;
}

/* ================= SPI helpers ================= */
/**
 * spiWrite
 * - SPI Mode 0, MSB first, ~4MHz
 * - Sends reg (write) then payload
 */
bool ICM20948_7Semi::spiWrite(uint8_t reg, const uint8_t *buf, size_t len) {
  _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  _spi->transfer(reg & 0x7F);
  for (size_t i = 0; i < len; ++i) _spi->transfer(buf[i]);
  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();
  return true;
}

/**
 * spiRead
 * - SPI Mode 0, MSB first, ~4MHz
 * - Sends reg|0x80 then clocks out len bytes
 */
bool ICM20948_7Semi::spiRead(uint8_t reg, uint8_t *buf, size_t len) {
  _spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  digitalWrite(_csPin, LOW);
  _spi->transfer(reg | 0x80);
  for (size_t i = 0; i < len; ++i) buf[i] = _spi->transfer(0x00);
  digitalWrite(_csPin, HIGH);
  _spi->endTransaction();
  return true;
}

/* ================= Common single-byte R/W ================= */
/**
 * read (bus-agnostic)
 * - I2C: uses addr/reg; SPI: ignores addr and uses reg
 */
bool ICM20948_7Semi::read(uint8_t devAddr, uint8_t reg, uint8_t *buf, size_t len) {
  return (_mode == MODE_SPI) ? spiRead(reg, buf, len) : i2cRead(devAddr, reg, buf, len);
}

/**
 * write (bus-agnostic)
 * - I2C: uses addr/reg; SPI: ignores addr and uses reg
 */
bool ICM20948_7Semi::write(uint8_t devAddr, uint8_t reg, const uint8_t *buf, size_t len) {
  return (_mode == MODE_SPI) ? spiWrite(reg, buf, len) : i2cWrite(devAddr, reg, buf, len);
}

/** - writeReg: single-byte write to current ICM address */
bool ICM20948_7Semi::writeReg(uint8_t reg, uint8_t val) {
  return write(_i2cAddr, reg, &val, 1);
}

/** - readReg: single-byte read to current ICM address */
bool ICM20948_7Semi::readReg(uint8_t reg, uint8_t &val) {
  return read(_i2cAddr, reg, &val, 1);
}

/* ================= Bank select / Bypass ================= */
/**
 * selectBank
 * - Select USER BANK (0..3)
 * - Returns true on write success
 */
bool ICM20948_7Semi::selectBank(uint8_t bank) {
  uint8_t v = (uint8_t)((bank & 0x03) << 4);
  return write(_i2cAddr, REG_BANK_SEL, &v, 1);
}

/**
 * enableBypass
 * - Disables I2C master and enables INT_PIN CFG bypass if requested
 * - Allows host to directly access AK09916 on aux I2C
 */
bool ICM20948_7Semi::enableBypass(bool en) {
  if (!selectBank(0)) return false;

  uint8_t uc = 0;
  if (!read(_i2cAddr, USER_CTRL, &uc, 1)) return false;
  uc &= ~0x20;  // I2C_MST_EN = 0
  if (!write(_i2cAddr, USER_CTRL, &uc, 1)) return false;

  uint8_t ic = 0;
  if (!read(_i2cAddr, INT_PIN_CFG, &ic, 1)) return false;
  if (en) ic |= 0x02;
  else ic &= ~0x02;  // BYPASS_EN
  return write(_i2cAddr, INT_PIN_CFG, &ic, 1);
}

/* ================= Basic configuration ================= */
/**
 * applyBasicDefaults
 * - Bank0: wake, enable sensors, exit LP
 * - Bank2: set DLPF on + max FS, SRD=0 for accel/gyro
 * - Bank0: set INT pin default options
 * - Returns true if all register writes succeed
 */
bool ICM20948_7Semi::applyBasicDefaults() {
  if (!selectBank(0)) return false;
  if (!writeReg(PWR_MGMT_1, 0x01)) return false;  // CLKSEL=1, SLEEP=0
  delay(10);
  if (!writeReg(PWR_MGMT_2, 0x00)) return false;  // Enable accel+gyro
  if (!writeReg(LP_CONFIG, 0x00)) return false;   // Duty-cycling off

  if (!selectBank(2)) return false;
  if (!writeReg(GYRO_CONFIG_1, 0x1F)) return false;    // DLPF on, FS=2000 dps
  if (!writeReg(GYRO_SMPLRT_DIV, 0x00)) return false;  // SRD=0 => ~1.1 kHz base
  if (!writeReg(ACCEL_CONFIG, 0x1F)) return false;     // DLPF on, FS=16g
  if (!writeReg(ACCEL_SMPLRT_DIV_1, 0x00)) return false;
  if (!writeReg(ACCEL_SMPLRT_DIV_2, 0x00)) return false;

  if (!selectBank(0)) return false;
  if (!writeReg(INT_PIN_CFG, 0x30)) return false;  // INT: open-drain, active-low, latch until cleared

  return true;
}

/* ================= Sensor Reads ================= */
/**
 * readAccel
 * - Reads raw 6B and converts to g using cached LSB scale
 * - Returns true on success
 */
bool ICM20948_7Semi::readAccel(float &x, float &y, float &z) {
  uint8_t raw[6];
  if (!selectBank(0)) return false;
  if (!read(_i2cAddr, ACCEL_XOUT_H, raw, 6)) return false;
  x = (int16_t)((raw[0] << 8) | raw[1]) / _accelMgPerLSB;
  y = (int16_t)((raw[2] << 8) | raw[3]) / _accelMgPerLSB;
  z = (int16_t)((raw[4] << 8) | raw[5]) / _accelMgPerLSB;
  return true;
}

/**
 * readGyro
 * - Reads raw 6B and converts to dps using cached LSB scale
 */
bool ICM20948_7Semi::readGyro(float &x, float &y, float &z) {
  uint8_t raw[6];
  if (!selectBank(0)) return false;
  if (!read(_i2cAddr, GYRO_XOUT_H, raw, 6)) return false;
  x = (int16_t)((raw[0] << 8) | raw[1]) / _gyroDpsPerLSB;
  y = (int16_t)((raw[2] << 8) | raw[3]) / _gyroDpsPerLSB;
  z = (int16_t)((raw[4] << 8) | raw[5]) / _gyroDpsPerLSB;
  return true;
}

/**
 * readTemperature
 * - Converts raw LSB to °C using datasheet scale
 */
bool ICM20948_7Semi::readTemperature(float &t) {
  uint8_t raw[2];
  if (!selectBank(0)) return false;
  t=0;
  for (int i = 0; i < 50; i++) {
    if (!read(_i2cAddr, TEMP_OUT_H, raw, 2)) return false;
    int16_t temp = (int16_t)((raw[0] << 8) | raw[1]);
    t += temp / 333.87f + 21.0f;
  }
  t /= 50;
  return true;
}

/**
 * readMag
 * - Enables bypass, triggers single AK09916 measurement
 * - Waits for DRDY, reads XYZ and converts to uT (0.15 uT/LSB)
 * - Returns false if DRDY times out (~20ms)
 */
// bool ICM20948_7Semi::readMag(float &x, float &y, float &z) {
//   if (!enableBypass(true)) return false;

//   uint8_t v = 0x01;
//   if (!i2cWrite(AK09916_I2C_ADDR, AK_CNTL2, &v, 1)) return false;  // single measure

//   uint8_t st1 = 0;
//   uint32_t t0 = millis();
//   do {
//     if (!i2cRead(AK09916_I2C_ADDR, AK_ST1, &st1, 1)) return false;
//     if (st1 & 0x01) break;
//     delay(1);
//   } while (millis() - t0 < 20);
//   if (!(st1 & 0x01)) return false;

//   uint8_t b[7];
//   if (!i2cRead(AK09916_I2C_ADDR, AK_HXL, b, 7)) return false;

//   x = (int16_t)((b[1] << 8) | b[0]) * 0.15f;
//   y = (int16_t)((b[3] << 8) | b[2]) * 0.15f;
//   z = (int16_t)((b[5] << 8) | b[4]) * 0.15f;
//   return true;
// }
bool ICM20948_7Semi::readMag(float &x, float &y, float &z) {
  // ---------- I2C host path (bypass to AK09916) ----------
  if (_mode == MODE_I2C) {
    if (!enableBypass(true)) return false;

    uint8_t v = 0x01; // AK09916 single-measure
    if (!i2cWrite(AK09916_I2C_ADDR, AK_CNTL2, &v, 1)) return false;

    uint8_t st1 = 0;
    uint32_t t0 = millis();
    do {
      if (!i2cRead(AK09916_I2C_ADDR, AK_ST1, &st1, 1)) return false;
      if (st1 & 0x01) break;     // DRDY
      delay(1);
    } while (millis() - t0 < 20);
    if (!(st1 & 0x01)) return false;

    uint8_t b[7];
    if (!i2cRead(AK09916_I2C_ADDR, AK_HXL, b, 7)) return false;

    x = (int16_t)((b[1] << 8) | b[0]) * 0.15f;
    y = (int16_t)((b[3] << 8) | b[2]) * 0.15f;
    z = (int16_t)((b[5] << 8) | b[4]) * 0.15f;
    return true;
  }

  // // ---------- SPI host path (internal I2C master) ----------
  // // 0) Ensure BANK0
  // if (!selectBank(0)) return false;

  // // USER_CTRL: enable internal I2C master; keep I2C_IF_DIS=1 for SPI reliability
  // {
  //   uint8_t uc = 0;
  //   (void)readReg(USER_CTRL, uc);
  //   uc |= 0x20; // I2C_MST_EN
  //   uc |= 0x10; // I2C_IF_DIS
  //   if (!writeReg(USER_CTRL, uc)) return false;
  // }

  // // Disable host BYPASS (INT_PIN_CFG.BYPASS_EN=0)
  // {
  //   uint8_t ic = 0;
  //   (void)readReg(INT_PIN_CFG, ic);
  //   ic &= (uint8_t)~0x02;
  //   if (!writeReg(INT_PIN_CFG, ic)) return false;
  // }

  // // 1) Configure internal I2C master timing in BANK3
  // if (!selectBank(3)) return false;

  // // I2C master clock / ODR config (safe defaults)
  // (void)writeReg(I2C_MST_ODR_CONFIG, 0x00); // default ODR
  // (void)writeReg(I2C_MST_CTRL,       0x07); // ~345 kHz master clock (safe)

  // // Helpers for SLV0 write/read transactions
  // auto slv0_write = [&](uint8_t dev7, uint8_t reg, uint8_t data)->bool {
  //   if (!writeReg(I2C_SLV0_ADDR, (uint8_t)((dev7 << 1) | 0x00))) return false; // write
  //   if (!writeReg(I2C_SLV0_REG,  reg)) return false;
  //   if (!writeReg(I2C_SLV0_DO,   data)) return false;
  //   return writeReg(I2C_SLV0_CTRL, (uint8_t)(0x80 | 0x01)); // EN + LEN=1
  // };

  // auto slv0_read_start = [&](uint8_t dev7, uint8_t reg, uint8_t len)->bool {
  //   if (len == 0) len = 1;
  //   if (len > 24) len = 24; // EXT_SLV_SENS_DATA window limit
  //   if (!writeReg(I2C_SLV0_ADDR, (uint8_t)((dev7 << 1) | 0x01))) return false; // read
  //   if (!writeReg(I2C_SLV0_REG,  reg)) return false;
  //   return writeReg(I2C_SLV0_CTRL, (uint8_t)(0x80 | (len & 0x0F))); // EN + LEN
  // };

  // // 2) Trigger AK09916 single measurement: CNTL2=0x01
  // if (!slv0_write(AK09916_I2C_ADDR, AK_CNTL2, 0x01)) return false;
  // delay(1); // allow the write to go out

  // // 3) Start a read from AK_ST1 of 8 bytes (ST1 + 6 data + ST2)
  // if (!slv0_read_start(AK09916_I2C_ADDR, AK_ST1, 8)) return false;

  // // 4) Read captured bytes from EXT_SLV_SENS_DATA_00..07 in BANK0
  // if (!selectBank(0)) return false;

  // uint8_t buf[8] = {0};
  // uint32_t t0 = millis();
  // while ((millis() - t0) < 20) {
  //   if (!read(_i2cAddr, EXT_SLV_SENS_DATA_00, buf, sizeof(buf))) return false;
  //   if (buf[0] & 0x01) break; // ST1.DRDY
  //   delay(1);
  // }
  // if ((buf[0] & 0x01) == 0) return false; // timeout

  // // Decode XYZ (little-endian) and convert to µT (0.15 µT/LSB)
  // int16_t mx = (int16_t)((buf[2] << 8) | buf[1]);
  // int16_t my = (int16_t)((buf[4] << 8) | buf[3]);
  // int16_t mz = (int16_t)((buf[6] << 8) | buf[5]);

  // x = mx * 0.15f;
  // y = my * 0.15f;
  // z = mz * 0.15f;

  // // Optional: stop SLV0 after read
  // if (selectBank(3)) (void)writeReg(I2C_SLV0_CTRL, 0x00);
  // (void)selectBank(0);
  // return true;
  return false;
}


/* ================= Power Modes ================= */
/**
 * softReset
 * - Issues device reset and waits ~100ms
 * - Configuration must be reapplied after call
 */
bool ICM20948_7Semi::softReset() {
  if (!selectBank(0)) return false;
  uint8_t v = 0x80;
  if (!write(_i2cAddr, PWR_MGMT_1, &v, 1)) return false;
  delay(100);
  return true;
}

/**
 * sleep
 * - en=true: set SLEEP and keep CLKSEL=1
 * - en=false: clear SLEEP and set CLKSEL=1
 */
bool ICM20948_7Semi::sleep(bool en) {
  if (!selectBank(0)) return false;
  uint8_t v = en ? (uint8_t)0x41 : (uint8_t)0x01;
  return write(_i2cAddr, PWR_MGMT_1, &v, 1);
}

/**
 * lowPower
 * - LP_EN: enable low-power mode in PWR_MGMT_1
 * - ACCEL_CYCLE / GYRO_CYCLE: duty-cycle blocks via LP_CONFIG
 * - Returns true if register accesses succeeded
 */
bool ICM20948_7Semi::lowPower(bool LP_EN, bool ACCEL_CYCLE, bool GYRO_CYCLE) {
  if (!selectBank(0)) return false;

  // PWR_MGMT_1: LP_EN bit
  uint8_t pwr1 = 0;
  if (!read(_i2cAddr, PWR_MGMT_1, &pwr1, 1)) return false;
  if (LP_EN) pwr1 |= 0x20;
  else pwr1 &= ~0x20;
  if (!write(_i2cAddr, PWR_MGMT_1, &pwr1, 1)) return false;

  // LP_CONFIG: ACCEL_CYCLE (bit5), GYRO_CYCLE (bit4)
  uint8_t lp = 0;
  if (!read(_i2cAddr, LP_CONFIG, &lp, 1)) return false;
  if (ACCEL_CYCLE) lp |= 0x20;
  else lp &= ~0x20;
  if (GYRO_CYCLE) lp |= 0x10;
  else lp &= ~0x10;
  if (!write(_i2cAddr, LP_CONFIG, &lp, 1)) return false;

  return true;
}

/**
 * setClkSel
 * - Sets CLKSEL [2:0] with range check
 */
bool ICM20948_7Semi::setClkSel(uint8_t c) {
  if (c > 7) return false;
  uint8_t v = 0;
  if (!selectBank(0)) return false;
  if (!read(_i2cAddr, PWR_MGMT_1, &v, 1)) return false;
  v = (uint8_t)((v & ~0x07) | (c & 0x07));
  return writeReg(PWR_MGMT_1, v);
}

/**
 * Gyro_SMPLRT
 * - Sets gyro sample rate divider (base 1100 Hz when DLPF enabled)
 * - rate >= 4.3 Hz (guard)
 */
bool ICM20948_7Semi::Gyro_SMPLRT(float rate) {
  if (rate < 4.3f) return false;
  uint8_t v = (uint8_t)((1100.0f / rate) - 1.0f);
  if (!selectBank(2)) return false;
  return writeReg(GYRO_SMPLRT_DIV, v);
}

/**
 * GyroConfig
 * - DLPFCFG, FS_SEL, FCHOICE + axis enables + AVGCFG
 * - Updates cached _gyroDpsPerLSB scale
 */
bool ICM20948_7Semi::GyroConfig(uint8_t DLPFCFG, uint8_t FS_SEL, bool FCHOICE,
                                bool XGYRO, bool YGYRO, bool ZGYRO, uint8_t AVGCFG) {
  if (!selectBank(2)) return false;

  DLPFCFG &= 0x07;
  FS_SEL &= 0x03;

  uint8_t v1 = (uint8_t)((DLPFCFG << 3) | (FS_SEL << 1) | (FCHOICE ? 1 : 0));

  uint8_t v2 = 0;
  if (!XGYRO) v2 |= 0x20;  // bit 5
  if (!YGYRO) v2 |= 0x10;  // bit 4
  if (!ZGYRO) v2 |= 0x08;  // bit 3
  v2 |= (AVGCFG & 0x07);   // bits [2:0]

  if (!writeReg(GYRO_CONFIG_1, v1)) return false;
  if (!writeReg(GYRO_CONFIG_2, v2)) return false;

  _gyroDpsPerLSB = 32768.0f / (250.0f * (1 << (FS_SEL & 0x03)));
  // Serial.println(String("_gyroDpsPerLSB: ") + _gyroDpsPerLSB);
  return true;
}

/**
 * Accel_SMPLRT
 * - Sets accel ODR via 1125 Hz base divider (DLPF enabled path)
 * - Clamps to 12-bit divider field
 */
bool ICM20948_7Semi::Accel_SMPLRT(uint16_t rate_hz) {
  if (!selectBank(2)) return false;
  if (rate_hz == 0) return false;
  if (rate_hz >= 1125) rate_hz = 1125;

  uint32_t div = (1125u + (rate_hz / 2u)) / rate_hz;  // ≈ round(1125/rate)
  if (div == 0) div = 1;
  div -= 1u;
  if (div > 4095u) div = 4095u;

  uint8_t hi = (uint8_t)((div >> 8) & 0x0F);
  uint8_t lo = (uint8_t)(div & 0xFF);

  if (!writeReg(ACCEL_SMPLRT_DIV_1, hi)) return false;
  if (!writeReg(ACCEL_SMPLRT_DIV_2, lo)) return false;
  return true;
}

/**
 * AccelConfigure
 * - DLPF cfg, FS, DLPF enable, DEC3, optional per-axis ST
 * - Updates cached _accelMgPerLSB scale
 */
bool ICM20948_7Semi::AccelConfigure(uint8_t DLPF, uint8_t FS_SEL,
                                    bool dlpf_enable, uint8_t dec3,
                                    bool stX, bool stY, bool stZ) {
  if (!selectBank(2)) return false;

  DLPF &= 0x07;
  FS_SEL &= 0x03;

  uint8_t v1 = 0;
  v1 |= (uint8_t)(DLPF << 3);
  v1 |= (uint8_t)(FS_SEL << 1);
  v1 |= dlpf_enable ? 1 : 0;

  uint8_t v2 = 0;
  if (stX) v2 |= 0x10;
  if (stY) v2 |= 0x08;
  if (stZ) v2 |= 0x04;
  v2 |= (dec3 & 0x03);

  if (!writeReg(ACCEL_CONFIG, v1)) return false;
  if (!writeReg(ACCEL_CONFIG_2, v2)) return false;

  _accelMgPerLSB = 16384.0f / (1 << FS_SEL);  // LSB per g
  // Serial.println(String("_accelMgPerLSB: ") + _accelMgPerLSB);
  return true;
}

/* ================= Sensor power gate ================= */
/**
 * setSensors
 * - Enables/disables accel, gyro, temp
 * - Clears duty-cycling so blocks are truly off
 * - Sets CLKSEL=1 and SLEEP=0
 */
bool ICM20948_7Semi::setSensors(bool accel_on, bool gyro_on, bool temp_on) {
  if (!selectBank(0)) return false;

  // 1) PWR_MGMT_1: TEMP_DIS and keep SLEEP=0, CLKSEL=1
  uint8_t pwr1 = 0;
  if (!read(_i2cAddr, PWR_MGMT_1, &pwr1, 1)) return false;
  pwr1 &= ~(1u << 6);  // SLEEP=0
  if (temp_on) pwr1 &= ~(1u << 3);
  else pwr1 |= (1u << 3);
  pwr1 = (uint8_t)((pwr1 & ~0x07) | 0x01);  // CLKSEL=001
  if (!write(_i2cAddr, PWR_MGMT_1, &pwr1, 1)) return false;

  // 2) LP_CONFIG: clear duty-cycling so blocks truly stay off
  uint8_t lp = 0;
  if (!read(_i2cAddr, LP_CONFIG, &lp, 1)) return false;
  lp &= ~(0x30);
  if (!write(_i2cAddr, LP_CONFIG, &lp, 1)) return false;

  // 3) PWR_MGMT_2: accel and gyro gate control
  uint8_t pwr2 = 0x00;
  if (!accel_on) pwr2 |= 0x38;
  if (!gyro_on) pwr2 |= 0x07;
  if (!write(_i2cAddr, PWR_MGMT_2, &pwr2, 1)) return false;

  // 4) Ensure FIFO sources are off for disabled sensors (best-effort)
  uint8_t en1 = 0, en2 = 0;
  if (!accel_on) en2 &= (uint8_t)~0x01;
  if (!gyro_on) en2 &= (uint8_t)~0x0E;
  if (!temp_on) en1 &= (uint8_t)~0x01;
  (void)write(_i2cAddr, 0x66, &en1, 1);
  (void)write(_i2cAddr, 0x67, &en2, 1);

  delay(10);

  // 5) Optional read-back verify (best-effort, does not fail call)
  uint8_t rb1 = 0, rb2 = 0;
  (void)read(_i2cAddr, PWR_MGMT_1, &rb1, 1);
  (void)read(_i2cAddr, PWR_MGMT_2, &rb2, 1);

  return true;
}

/* ================= Offsets ================= */
/**
 * writeGyroOffset
 * - Writes 3x int16 offsets to bank2 auto-increment region
 */
bool ICM20948_7Semi::writeGyroOffset(const int16_t off[3]) {
  if (!selectBank(2)) return false;
  uint8_t buf[6] = {
    (uint8_t)(off[0] >> 8),
    (uint8_t)(off[0] & 0xFF),
    (uint8_t)(off[1] >> 8),
    (uint8_t)(off[1] & 0xFF),
    (uint8_t)(off[2] >> 8),
    (uint8_t)(off[2] & 0xFF),
  };
  return write(_i2cAddr, XG_OFFS_USRH, buf, sizeof(buf));
}

/**
 * readGyroOffset
 * - Reads 3x int16 offsets from bank2
 */
bool ICM20948_7Semi::readGyroOffset(int16_t out[3]) {
  if (!selectBank(2)) return false;
  uint8_t buf[6];
  if (!read(_i2cAddr, XG_OFFS_USRH, buf, sizeof(buf))) return false;
  out[0] = (int16_t)((buf[0] << 8) | buf[1]);
  out[1] = (int16_t)((buf[2] << 8) | buf[3]);
  out[2] = (int16_t)((buf[4] << 8) | buf[5]);
  return true;
}

/**
 * writeAccelOffset
 * - Writes 3x int16 accel offsets to bank2
 */
bool ICM20948_7Semi::writeAccelOffset(const int16_t off[3]) {
  if (!selectBank(2)) return false;
  uint8_t buf[6] = {
    (uint8_t)(off[0] >> 8),
    (uint8_t)(off[0] & 0xFF),
    (uint8_t)(off[1] >> 8),
    (uint8_t)(off[1] & 0xFF),
    (uint8_t)(off[2] >> 8),
    (uint8_t)(off[2] & 0xFF),
  };
  return write(_i2cAddr, XA_OFFS_H, buf, sizeof(buf));
}

/**
 * readAccelOffset
 * - Reads 3x int16 accel offsets from bank2
 */
bool ICM20948_7Semi::readAccelOffset(int16_t out[3]) {
  if (!selectBank(2)) return false;
  uint8_t buf[6];
  if (!read(_i2cAddr, XA_OFFS_H, buf, sizeof(buf))) return false;
  out[0] = (int16_t)((buf[0] << 8) | buf[1]);
  out[1] = (int16_t)((buf[2] << 8) | buf[3]);
  out[2] = (int16_t)((buf[4] << 8) | buf[5]);
  return true;
}