/***************************************************************
 * @file    I2C_Accel.ino
 * @brief   Minimal I2C bring-up for 7Semi ICM-20948 on UNO/ESP32 +
 *          continuous accelerometer readout.
 *
 * Features
 * - I2C init on default/custom pins
 * - IMU begin() on I2C (addr 0x68/0x69)
 * - Safe defaults (applyBasicDefaults)
 * - Optional sensor gating: accel only
 * - Accel config: DLPF, full-scale, ODR, DEC3
 * - Read accel (g) at ~2 Hz print
 *
 * Wiring (Arduino UNO I2C)
 * - SDA  -> A4
 * - SCL  -> A5
 * - VCC  -> 3V3 (or 5V if your board supports it)
 * - GND  -> GND
 *
 * Wiring (ESP32 default I2C)
 * - SDA  -> GPIO21
 * - SCL  -> GPIO22
 * - VCC  -> 3V3
 * - GND  -> GND
 ***************************************************************/

#include <Wire.h>
#include <7Semi_ICM20948.h>

/* ====================== User Config =======================
 * - Edit address/pins to match your board/wiring
 * - I2C address: 0x68 (AD0=LOW) / 0x69 (AD0=HIGH)
 * - UNO: Wire.begin() uses fixed A4/A5
 * - ESP32: you can use default Wire.begin() (21/22) or custom pins
 */
#define ICM_ADDR 0x69  // 0x68 default, change to 0x69 if AD0=HIGH

// For ESP32 custom I2C pins, uncomment and adjust:
// #define I2C_SDA 21
// #define I2C_SCL 22

/** - IMU instance */
ICM20948_7Semi imu;

void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println(F("ICM-20948 (I2C) — Accel Example"));

  /** Init I2C bus */
  Wire.begin();  // UNO/ESP32 default
  // Wire.begin(I2C_SDA, I2C_SCL); // ESP32 custom pins (optional)

  /** Attach IMU to I2C bus */
  if (!imu.begin(Wire, ICM_ADDR)) {
    Serial.println(F("ERROR: ICM-20948 I2C begin() failed."));
    while (1) delay(200);
  }

  Serial.println(F("ICM-20948 ready."));

  /** Optional: gate sensors
   * - setSensors(accel_on, gyro_on, temp_on)
   * - Here: enable only accel (gyro/temp off)
   */
  if (!imu.setSensors(/*accel*/ true, /*gyro*/ false, /*temp*/ false)) {
    Serial.println(F("setSensors failed."));
  }

  /* ---------------- ACCEL DLPF Config (reference) ---------------
   * ACCEL_DLPFCFG (0..7) — 3dB & Noise Bandwidth (datasheet)
   * DLPF path ODR = 1125 / (1 + ACCEL_SMPLRT_DIV), DIV: 0..4095
   *
   * FCHOICE=0 (Bypass) : 3dB ≈ 1209 Hz, NBW ≈ 1248 Hz, Rate ≈ 4500 Hz
   * FCHOICE=1 (DLPF on):
   * - ACCEL_DLPFCFG_0 : 3dB ≈ 246.0 Hz,  NBW ≈ 265.0 Hz
   * - ACCEL_DLPFCFG_1 : 3dB ≈ 246.0 Hz,  NBW ≈ 265.0 Hz
   * - ACCEL_DLPFCFG_2 : 3dB ≈ 111.4 Hz,  NBW ≈ 136.0 Hz
   * - ACCEL_DLPFCFG_3 : 3dB ≈ 50.4  Hz,  NBW ≈ 68.8  Hz   ← good default
   * - ACCEL_DLPFCFG_4 : 3dB ≈ 23.9  Hz,  NBW ≈ 34.4  Hz
   * - ACCEL_DLPFCFG_5 : 3dB ≈ 11.5  Hz,  NBW ≈ 17.0  Hz
   * - ACCEL_DLPFCFG_6 : 3dB ≈ 5.7   Hz,  NBW ≈ 8.3   Hz
   * - ACCEL_DLPFCFG_7 : 3dB ≈ 473   Hz,  NBW ≈ 499   Hz
   */

  /** Configure accelerometer
   * AccelConfigure(DLPF, FS_SEL, dlpf_enable, dec3, stX, stY, stZ)
   * - DLPF       : ACCEL_DLPFCFG_0..7 (use 3 for balanced setting)
   * - FS_SEL     : 0..3 => ±2/±4/±8/±16 g  (g2=0, g4=1, g8=2, g16=3)
   * - dlpf_enable: true to use DLPF path (recommended)
   * - dec3       : 0..3 (DEC3_CFG averaging)
   *                0 = ACCEL_DEC3_AVG_1_OR_4  (depends on FCHOICE)
   *                1 = ACCEL_DEC3_AVG_8
   *                2 = ACCEL_DEC3_AVG_16
   *                3 = ACCEL_DEC3_AVG_32
   * - stX/Y/Z    : enable self-test (false here)
   */
  if (!imu.AccelConfigure(ACCEL_DLPFCFG_3, /*FS_SEL*/ g4,
                          /*dlpf_enable*/ true,
                          /*dec3*/ ACCEL_DEC3_AVG_8,
                          /*stX*/ false, /*stY*/ false, /*stZ*/ false)) {
    Serial.println(F("AccelConfigure failed."));
  }

  /** Set accelerometer output data rate (ODR)
   * - Accel_SMPLRT(rate_hz)
   * - Base (DLPF path) = 1125 Hz
   * - Valid range: 1–1125 Hz
   * - Example: 225 Hz 
   */
  if (!imu.Accel_SMPLRT(225)) {
    Serial.println(F("Accel_SMPLRT failed."));
  }
}

void loop() {
  float ax, ay, az;  // accel X/Y/Z in g

  /** - readAccel(x,y,z)
   * - Returns:
   *   - true on success;
   * - Output:
   *   - ax/ay/az in g
   */
  if (imu.readAccel(ax, ay, az)) {
    Serial.print("ACCEL [g]: ");
    Serial.print(ax, 3);
    Serial.print(", ");
    Serial.print(ay, 3);
    Serial.print(", ");
    Serial.println(az, 3);
  } else {
    Serial.println(F("ACCEL read failed"));
  }

  Serial.println(F("-----------------------------"));
  delay(500);
}
