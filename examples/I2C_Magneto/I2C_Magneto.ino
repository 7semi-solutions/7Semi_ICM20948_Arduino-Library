/***************************************************************
 * @file    I2C_Magneto.ino
 * @brief   Minimal I2C bring-up for 7Semi ICM-20948 +
 *          single/continuous AK09916 magnetometer readout.
 *
 * Features
 * - I2C init on default/custom pins
 * - IMU begin() on I2C (addr 0x68/0x69)
 * - Safe defaults (applyBasicDefaults)
 * - Optional sensor gating: mag only
 * - Read magnetometer (uT) at ~2 Hz print
 *
 * Wiring (Arduino UNO, I2C)
 * - SDA  -> A4
 * - SCL  -> A5
 * - VCC  -> 3V3 (or 5V if board supports it)
 * - GND  -> GND
 *
 * Wiring (ESP32, I2C default)
 * - SDA  -> GPIO21
 * - SCL  -> GPIO22
 * - VCC  -> 3V3
 * - GND  -> GND
 *
 * Notes
 * - On I2C hosts, the driver may enable BYPASS so the MCU can
 *   directly talk to AK09916 on the aux bus for a single-shot read.
 *
 * Author  : 7Semi 
 * License : MIT
 ***************************************************************/

#include <Wire.h>
#include <7Semi_ICM20948.h>

/* ====================== User Config =======================
 * - Edit address/pins to match your board/wiring
 * - I2C address: 0x68 (AD0=LOW) / 0x69 (AD0=HIGH)
 * - UNO: Wire.begin() uses fixed A4/A5
 * - ESP32: you can use default Wire.begin() (21/22) or custom pins
 */
#define ICM_ADDR 0x69   // change to 0x69 if AD0=HIGH

// For ESP32 custom I2C pins, uncomment and adjust:
// #define I2C_SDA 21
// #define I2C_SCL 22

/** - IMU instance */
ICM20948_7Semi imu;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("ICM-20948 (I2C) — Magnetometer Example"));

  /** Init I2C bus */
  Wire.begin();                 // UNO/ESP32 default
  // Wire.begin(I2C_SDA, I2C_SCL); // ESP32 custom pins (optional)

  /** Attach IMU to I2C bus */
  if (!imu.begin(Wire, ICM_ADDR)) {
    Serial.println(F("ERROR: ICM-20948 I2C begin() failed."));
    while (1) delay(200);
  }

  /** Apply safe defaults (wake, DLPF on, etc.) */
  if (!imu.applyBasicDefaults()) {
    Serial.println(F("ERROR: applyBasicDefaults() failed."));
    while (1) delay(200);
  }

  Serial.println(F("ICM-20948 ready (I2C)."));

  /** Optional: gate sensors (mag only)
   * - setSensors(accel_on, gyro_on, temp_on)
   * - temp can remain off; it’s not needed for mag
   */
  if (!imu.setSensors(/*accel*/ false, /*gyro*/ false, /*temp*/ false)) {
    Serial.println(F("setSensors failed."));
  }

  /** Optional: WHO_AM_I check (expect 0xEA) */
  uint8_t who = imu.readWhoAmI();
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);
}

void loop() {
  float mx, my, mz;  // magnetometer X/Y/Z in microtesla (uT)

  /** - readMag(x,y,z)
   * - On I2C, driver may enable BYPASS and directly access AK09916:
   *   - Trigger single measurement (CNTL2=0x01)
   *   - Poll DRDY (ST1[0])
   *   - Read XYZ (HXL..HZH), scale to uT (0.15 uT/LSB typical)
   * - Returns true on success
   */
  if (imu.readMag(mx, my, mz)) {
    Serial.print(F("MAG [uT]: "));
    Serial.print(mx, 2); Serial.print(F(", "));
    Serial.print(my, 2); Serial.print(F(", "));
    Serial.println(mz, 2);
  } else {
    Serial.println(F("Mag read failed"));
  }

  Serial.println(F("-----------------------------"));
  delay(500);
}
