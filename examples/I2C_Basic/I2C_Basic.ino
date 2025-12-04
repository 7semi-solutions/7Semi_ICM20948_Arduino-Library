/***************************************************************
 * @file    I2C_Basic.ino
 * @brief   Minimal I2C bring-up for 7Semi ICM-20948 +
 *          basic Accel/Gyro/Temp readout.
 *
 * Wiring (Arduino UNO I2C)
 * - SDA -> A4
 * - SCL -> A5
 * - VCC -> 3V3 (or 5V if your board supports it)
 * - GND -> GND
 *
 * Wiring (ESP32 default I2C)
 * - SDA -> GPIO21
 * - SCL -> GPIO22
 ***************************************************************/

#include <Wire.h>
#include <7Semi_ICM20948.h>

/** User config */
#define ICM_ADDR 0x69  // 0x68 (AD0=LOW) or 0x69 (AD0=HIGH)

/** IMU instance */
ICM20948_7Semi imu;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("ICM-20948 — I2C Basic"));

  // I2C init (UNO/ESP32 defaults). For ESP32 custom pins: Wire.begin(SDA,SCL);
  Wire.begin();

  // Attach IMU
  if (!imu.begin(Wire, ICM_ADDR)) {
    Serial.println(F("ERROR: begin(I2C) failed"));
    while (1) delay(200);
  }

  // Safe defaults (wake, accel/gyro on, DLPF on)
  if (!imu.applyBasicDefaults()) {
    Serial.println(F("ERROR: applyBasicDefaults() failed"));
    while (1) delay(200);
  }

  // WHO_AM_I check (expect 0xEA)
  uint8_t who = imu.readWhoAmI();
  Serial.print(F("WHO_AM_I = 0x")); Serial.println(who, HEX);
  Serial.println(F("Ready.\n"));
}

void loop() {
  float ax, ay, az;
  float gx, gy, gz;
  float mx, my, mz;
  float tC;

  if (imu.readAccel(ax, ay, az)) {
    Serial.print(F("ACC [g]: "));
    Serial.print(ax, 3); Serial.print(", ");
    Serial.print(ay, 3); Serial.print(", ");
    Serial.println(az, 3);
  } else {
    Serial.println(F("ACC read failed"));
  }

  if (imu.readGyro(gx, gy, gz)) {
    Serial.print(F("GYR [dps]: "));
    Serial.print(gx, 2); Serial.print(", ");
    Serial.print(gy, 2); Serial.print(", ");
    Serial.println(gz, 2);
  } else {
    Serial.println(F("GYR read failed"));
  }

  if (imu.readMag(mx, my, mz)) {
    Serial.print(F("MAG [uT]: "));
    Serial.print(mx, 2); Serial.print(", ");
    Serial.print(my, 2); Serial.print(", ");
    Serial.println(mz, 2);
  } else {
    Serial.println(F("GYR read failed"));
  }


  if (imu.readTemperature(tC)) {
    Serial.print(F("TMP [C]: "));
    Serial.println(tC, 2);
  } else {
    Serial.println(F("TMP read failed"));
  }

  Serial.println(F("-----------------------------"));
  delay(500);
}

/* Note on magnetometer:
 * The AK09916 sits on the ICM's auxiliary I2C bus. In I2C setups,
 * the driver may enable BYPASS so readMag() can talk to AK09916
 * directly. If readMag() returns false, check wiring and pull-ups.
 */
