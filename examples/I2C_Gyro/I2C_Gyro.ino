/***************************************************************
 * @file    I2C_Gyro.ino
 * @brief   Minimal I2C bring-up for 7Semi ICM-20948 on ESP32/UNO +
 *          continuous gyroscope readout.
 *
 * Features
 * - I2C init on default/custom pins
 * - IMU begin() on I2C (addr 0x68/0x69)
 * - Safe defaults (applyBasicDefaults)
 * - Optional sensor gating: gyro only
 * - Gyro config: DLPF, full-scale, ODR, AVG
 * - Read gyro (dps) at ~2 Hz print
 *
 * Wiring (Arduino UNO, I2C)
 * - SDA  -> A4
 * - SCL  -> A5
 * - VCC  -> 3V3 (or 5V if your board supports it)
 * - GND  -> GND
 *
 * Wiring (ESP32, I2C)
 * - SDA  -> GPIO21 (default)
 * - SCL  -> GPIO22 (default)
 * - VCC  -> 3V3
 * - GND  -> GND
 *
 * Address
 * - Default ICM-20948 I2C address is 0x68 (AD0=LOW). If AD0=HIGH, use 0x69.
 ***************************************************************/

#include <7Semi_ICM20948.h>

/* ====================== User Config =======================
 * - Edit pins/addr if needed
 * - UNO uses fixed SDA=A4, SCL=A5 (no need to call Wire.begin with pins)
 * - ESP32 can use default Wire (SDA=21, SCL=22) or custom pins via Wire.begin(sda,scl)
 */
#define ICM_ADDR 0x69   // 0x68 (AD0=LOW) or 0x69 (AD0=HIGH)

// For ESP32 custom I2C pins, uncomment and adjust:
// #define I2C_SDA 21
// #define I2C_SCL 22

/** - IMU instance */
ICM20948_7Semi imu;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("ICM-20948 (I2C) — Gyro Example"));

  /** Init I2C bus
   * - UNO: Wire.begin() with no pin args
   * - ESP32 (optional): Wire.begin(I2C_SDA, I2C_SCL);
   */
  Wire.begin();
  // Wire.begin(I2C_SDA, I2C_SCL);  // ESP32 custom pins (optional)

  /** Attach IMU to I2C bus
   * - begin(Wire, address) → true on success
   */
  if (!imu.begin(Wire, ICM_ADDR)) {
    Serial.println(F("ERROR: ICM-20948 I2C begin() failed."));
    while (1) delay(200);
  }

  /** Apply safe defaults */
  if (!imu.applyBasicDefaults()) {
    Serial.println(F("ERROR: applyBasicDefaults() failed."));
    while (1) delay(200);
  }

  Serial.println(F("ICM-20948 ready."));

  /** Optional: gate sensors
   * - setSensors(accel_on, gyro_on, temp_on)
   * - Here: enable only gyro (accel/temp off)
   */
  if (!imu.setSensors(/*accel*/ false, /*gyro*/ true, /*temp*/ false)) {
    Serial.println(F("setSensors failed."));
  }

  /* ----------------- GYRO DLPF Config (reference) ----------------
   * GyroConfig(DLPFCFG, FS_SEL, FCHOICE, XGYRO, YGYRO, ZGYRO, AVGCFG)
   * - DLPF     : GYRO_DLPFCFG_0..7 (use 3 or 4 for balanced settings)
   * - FS_SEL   : dps250 / dps500 / dps1000 / dps2000
   * - FCHOICE  : false → DLPF path (recommended), true → bypass (very wide BW)
   * - XGYRO/YGYRO/ZGYRO : per-axis enable (true = enable axis)
   * - AVGCFG   : 0..7 internal averaging (higher = more smoothing, more delay)
   * FCHOICE=0 (DLPF on):
   * - GYRO_DLPFCFG_1 : 3dB ≈ 196.6 Hz, NBW ≈ 229.8 Hz
   * - GYRO_DLPFCFG_2 : 3dB ≈ 151.8 Hz, NBW ≈ 187.6 Hz
   * - GYRO_DLPFCFG_3 : 3dB ≈ 119.5 Hz, NBW ≈ 154.3 Hz   ← good default
   * - GYRO_DLPFCFG_4 : 3dB ≈ 51.2  Hz, NBW ≈ 73.3  Hz
   * - GYRO_DLPFCFG_5 : 3dB ≈ 23.9  Hz, NBW ≈ 35.9  Hz
   * - GYRO_DLPFCFG_6 : 3dB ≈ 5.7   Hz, NBW ≈ 8.3   Hz
   * - GYRO_DLPFCFG_7 : 3dB ≈ 473   Hz, NBW ≈ 499   Hz
   *
   * FCHOICE=1 (Bypass) : very wide (use with care; highest noise)
   */
  if (!imu.GyroConfig(/*DLPFCFG*/ GYRO_DLPFCFG_4,
                      /*FS_SEL*/   dps2000,
                      /*FCHOICE*/  true,       // DLPF ON (recommended)
                      /*X*/ true, /*Y*/ true, /*Z*/ true,
                      /*AVGCFG*/   0)) {
    Serial.println(F("GyroConfig failed."));
  }

  /** Set gyroscope output data rate (ODR)
   * - Gyro_SMPLRT(rate_hz)
   * - Base (DLPF path) = 1100 Hz
   * - Valid range: ~4.3–1100 Hz
   * - Example: 1000 Hz
   */
  if (!imu.Gyro_SMPLRT(1000)) {
    Serial.println(F("Gyro_SMPLRT failed."));
  }
}

void loop() {
  float gx, gy, gz;  // gyro X/Y/Z in dps

  /** - readGyro(x,y,z)
   * - Returns:
   *   - true on success;
   * - Output:
   *   - gx/gy/gz in dps
   */
  if (imu.readGyro(gx, gy, gz)) {
    Serial.print("GYRO [dps]: ");
    Serial.print(gx, 3);
    Serial.print(", ");
    Serial.print(gy, 3);
    Serial.print(", ");
    Serial.println(gz, 3);
  } else {
    Serial.println(F("GYRO read failed"));
  }

  Serial.println(F("-----------------------------"));
  delay(500);
}
