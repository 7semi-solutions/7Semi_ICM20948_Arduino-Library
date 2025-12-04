/***************************************************************
 * @file    SPI_Basic.ino
 * @brief   Minimal SPI bring-up for 7Semi ICM-20948 +
 *          basic Accel/Gyro/Temp readout.
 *
 * Wiring (Arduino UNO SPI)
 * - SCK  -> D13
 * - MOSI -> D11
 * - MISO -> D12
 * - CS   -> D10 (changeable; update CS_PIN)
 * - VCC  -> 3V3
 * - GND  -> GND
 *
 * Wiring (ESP32 VSPI default)
 * - SCK  -> GPIO18
 * - MOSI -> GPIO23
 * - MISO -> GPIO19
 * - CS   -> GPIO5  (changeable; update CS_PIN)
 ***************************************************************/
#include <7Semi_ICM20948.h>

/** User config */
// ESP32 (VSPI) pins (optional to call SPI.begin(SCK,MISO,MOSI,CS))
// static const uint8_t CS_PIN = 5;
// static const uint8_t SCK_PIN = 18;
// static const uint8_t MOSI_PIN = 23;
// static const uint8_t MISO_PIN = 19;

// Arduino UNO default SPI pins
static const uint8_t CS_PIN = 10;
static const uint8_t SCK_PIN = 13;
static const uint8_t MOSI_PIN = 11;
static const uint8_t MISO_PIN = 12;

/** IMU instance */
ICM20948_7Semi imu;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("ICM-20948 — SPI Basic"));

  // SPI init (optionally pass pins; on UNO, SPI.begin() without args is fine)
  // SPI.begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);

  // Attach IMU (SPI, CS)
  if (!imu.begin(SPI, CS_PIN)) {
    Serial.println(F("ERROR: begin(SPI) failed"));
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

  if (imu.readTemperature(tC)) {
    Serial.print(F("TMP [C]: "));
    Serial.println(tC, 2);
  } else {
    Serial.println(F("TMP read failed"));
  }

  Serial.println(F("-----------------------------"));
  delay(500);
}

/* Note on magnetometer for SPI:
 * The AK09916 is behind the ICM’s internal I2C master. Your driver’s
 * readMag() must use I2C_SLVx registers to talk to the mag in SPI mode.
 * If your breakout doesn’t expose aux SDA/SCL, mag may not be available
 * without enabling the internal master sequence in firmware.
 */
