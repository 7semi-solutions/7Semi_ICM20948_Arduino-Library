#include "7Semi_ICM20948.h"

/** - Construct with default I2C address; call begin() to attach bus */
ICM20948_7Semi::ICM20948_7Semi() {}

bool ICM20948_7Semi::beginI2C(uint8_t address, TwoWire &i2cPort, uint32_t i2cSpeed)
{
  // Free previous BusIO instance 
  if (bus)
  {
    delete bus;
    bus = nullptr;
  }

  // Assign I2C interface implementation
  iface = &i2c;

  // Initialize I2C (400kHz) and probe device
  if (!i2c.beginI2C(address, i2cPort, i2cSpeed))
    return false;

  // Create BusIO abstraction layer
  bus = new BusIO_7Semi<Interface_7Semi>(*iface);
  if (!bus)
    return false;

  // Allow device to stabilize after power-up
  delay(100);

  // Read WHO_AM_I register
  uint8_t who_am_i;
  if (!readWhoAmI(who_am_i))
    return false;

  // Validate device ID (expected 0xEA)
  if (who_am_i != WHO_AM_I_VAL)
    return false;

  if (!selectBank(2))
    return false;

  if (!bus->write(ODR_ALIGN_EN, (uint8_t)0x01))
    return false;

  // Enable I2C interface (I2C_IF_DIS = 0)
  if (!bus->writeBit(USER_CTRL, 4, (uint8_t)0x00))
    return false;

  // Set clock source to PLL (auto select best clock)
  if (!bus->write(PWR_MGMT_1, (uint8_t)0x01))
    return false;

  if(!applyBasicDefaults())
    return false;

  // Initialization successful
  return true;
}

bool ICM20948_7Semi::beginSPI(uint8_t csPin, SPIClass &spiPort, uint32_t spiSpeed)
{
// Free previous BusIO instance 
  if (bus)
  {
    delete bus;
    bus = nullptr;
  }

  // Assign SPI interface implementation
  iface = &spi;

  // Initialize SPI (1MHz) and probe device
  if (!spi.beginSPI(csPin, spiPort, spiSpeed))
    return false;

  // Create BusIO abstraction layer
  bus = new BusIO_7Semi<Interface_7Semi>(*iface);
  if (!bus)
    return false;

    softReset();

  // Allow device to stabilize after power-up
  delay(100);


  // Read WHO_AM_I register
  uint8_t who_am_i;

  if (!readWhoAmI(who_am_i))
    return false;

  // Validate device ID (expected 0xEA)
  if (who_am_i == WHO_AM_I_VAL)
    return false;

  if (!selectBank(2))
    return false;

  if (!bus->write(ODR_ALIGN_EN, (uint8_t)0x01))
    return false;

  // Enable I2C interface (I2C_IF_DIS = 0)
  if (!bus->writeBit(USER_CTRL, 4, (uint8_t)0x00))
    return false;

  // Wake device from sleep mode (SLEEP = 0)
  if (!bus->write(PWR_MGMT_1,(uint8_t)0x01))
    return false;

  if(!applyBasicDefaults())
    return false;

  // Initialization successful
  return true;
}

bool ICM20948_7Semi::readWhoAmI(uint8_t &whoAmI)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Read WHO_AM_I register
  if (!bus->read(WHO_AM_I, whoAmI))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::selectBank(uint8_t bank)
{
  if (bank > 3)
    return false;

  // Mask bank value to valid range (0–3) and shift to bits [5:4]
  uint8_t v = (bank & 0x03) << 4;

  // Write bank selection to REG_BANK_SEL register
  return bus->write(REG_BANK_SEL, v);
}

bool ICM20948_7Semi::softReset()
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Set DEVICE_RESET bit (bit 7)
  if (!bus->write(PWR_MGMT_1, (uint8_t)0x80))
    return false;

  // Allow device to reset
  delay(100);

  selectBank(2);
  bus->write(ODR_ALIGN_EN, (uint8_t)0x01);

  // Success
  return true;
}

bool ICM20948_7Semi::sleep(bool en)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;
 uint8_t v = 1;
    if(en)
   v|= 1<<6;

  // Set or clear SLEEP bit (bit 6)
  if (!bus->write(PWR_MGMT_1, v))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::applyBasicDefaults()
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Wake device and set clock source (CLKSEL=1, SLEEP=0)
  if (!bus->write(PWR_MGMT_1, (uint8_t)0x01))
    return false;

  if (!bus->write(PWR_MGMT_2, (uint8_t)0x00))
    return false;


  // Allow device to stabilize
  delay(10);

  // Disable duty-cycling (LP mode off)
  if (!bus->write(LP_CONFIG, (uint8_t)0x00))
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Configure gyro (DLPF enabled, FS = 2000 dps)
  if (!bus->write(GYRO_CONFIG_1, (uint8_t)0x1F))
    return false;

  // Set gyro sample rate divider (SRD = 0 → max rate)
  if (!bus->write(GYRO_SMPLRT_DIV, (uint8_t)0x00))
    return false;

  // Configure accel (DLPF enabled, FS = 16g)
  if (!bus->write(ACCEL_CONFIG, (uint8_t)0x1F))
    return false;

  // Set accel sample rate divider high byte
  if (!bus->write(ACCEL_SMPLRT_DIV_1, (uint8_t)0x00))
    return false;

  // Set accel sample rate divider low byte
  if (!bus->write(ACCEL_SMPLRT_DIV_2, (uint8_t)0x00))
    return false;

  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Configure interrupt pin (open-drain, active-low, latch)
  if (!bus->write(INT_PIN_CFG, (uint8_t)0x30))
    return false;

  // Configuration successful
  return true;
}

bool ICM20948_7Semi::readAccel(float &x, float &y, float &z)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Read 6 bytes (X, Y, Z)
  uint8_t raw[6];
  if (!bus->read(ACCEL_XOUT_H, raw, 6))
    return false;

  // Convert raw data to g
  x = (int16_t)((raw[0] << 8) | raw[1]) / mg_per_lsb;
  y = (int16_t)((raw[2] << 8) | raw[3]) / mg_per_lsb;
  z = (int16_t)((raw[4] << 8) | raw[5]) / mg_per_lsb;

  // Success
  return true;
}

bool ICM20948_7Semi::readGyro(float &x, float &y, float &z)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Read 6 bytes (X, Y, Z)
  uint8_t raw[6];
  if (!bus->read(GYRO_XOUT_H, raw, 6))
    return false;

  // Convert raw data to dps
  x = (int16_t)((raw[0] << 8) | raw[1]) / degree_per_second;
  y = (int16_t)((raw[2] << 8) | raw[3]) / degree_per_second;
  z = (int16_t)((raw[4] << 8) | raw[5]) / degree_per_second;

  // Success
  return true;
}

bool ICM20948_7Semi::readTemperature(float &temperature)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Initialize accumulator
  temperature = 0;

  // Read and average 5 samples
  for (int i = 0; i < 5; i++)
  {
    // Read temperature registers
    uint8_t raw[2];
    if (!bus->read(TEMP_OUT_H, raw, 2))
      return false;

    // Convert raw to signed value
    int16_t temp = (int16_t)((raw[0] << 8) | raw[1]);

    // Convert to °C and accumulate
    temperature += temp / 333.87f + 21.0f;
  }

  // Compute average temperature
  temperature /= 5.0f;

  // Success
  return true;
}


bool ICM20948_7Semi::readMag(float &x, float &y, float &z)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Read 8 bytes (ST1 + XYZ + ST2)
  uint8_t buf[8];
  if (!bus->read(EXT_SLV_SENS_DATA_00, buf, 8))
    return false;

  int16_t mx = (int16_t)(buf[1] | (buf[2] << 8));
  int16_t my = (int16_t)(buf[3] | (buf[4] << 8));
  int16_t mz = (int16_t)(buf[5] | (buf[6] << 8));

  // Convert to µT (AK09916 sensitivity)
  x = mx * 0.15f;
  y = my * 0.15f;
  z = mz * 0.15f;

  // Success
  return true;
}

bool ICM20948_7Semi::initMag()
{
  if (!bus)
    return false;

  softReset();

  sleep(false);

  if (!selectBank(0))
    return false;

  if (!bus->write(USER_CTRL, (uint8_t)USER_CTRL_I2C_MST_EN))
    return false;

  if (!selectBank(3))
    return false;

  if (!bus->write(I2C_MST_CTRL, (uint8_t)0x07))
    return false;

  delay(10);

  writeSlave4(AK_CNTL3, 0x01);

  delay(100);

  uint8_t i, who;
  for (i = 0; i < 10; i++)
  {
    readSlave4(AK_WIA2, who);
    if (who == AK_WIA2_VAL)
    {
      writeSlave4(AK_CNTL2, 0x08);

      delay(10);

      // ---- Setup auto-read (SLV0) ----
      if (!selectBank(3))
        return false;

      if (!bus->write(I2C_SLV0_ADDR, (uint8_t)(AK09916_I2C_ADDR | 0x80)))
        return false;

      if (!bus->write(I2C_SLV0_REG, (uint8_t)AK_ST1))
        return false;

      if (!bus->write(I2C_SLV0_CTRL, (uint8_t)(I2C_SLVx_EN | 8)))
        return false;

      return true;
    }

     writeSlave4(AK_CNTL2, 0x08);

      delay(10);

      // ---- Setup auto-read (SLV0) ----
      if (!selectBank(3))
        return false;

      if (!bus->write(I2C_SLV0_ADDR, (uint8_t)(AK09916_I2C_ADDR | 0x80)))
        return false;

      if (!bus->write(I2C_SLV0_REG, (uint8_t)AK_ST1))
        return false;

      if (!bus->write(I2C_SLV0_CTRL, (uint8_t)(I2C_SLVx_EN | 8)))
        return false;
    
    return false;
  }
}

bool ICM20948_7Semi::setMagOpMode(ICM20948_Op_Mode opMode)
{
  // Write operation mode to magnetometer
  writeSlave4(AK_CNTL2, (uint8_t)opMode);

  return true;
}

bool ICM20948_7Semi::writeSlave4(uint8_t reg, uint8_t value)
{
  // Select USER BANK 3
  if (!selectBank(3))
    return false;

  // Set slave address (write)
  if (!bus->write(I2C_SLV4_ADDR, (uint8_t)AK09916_I2C_ADDR))
    return false;
  // Set data to write
  if (!bus->write(I2C_SLV4_DO, value))
    return false;

  // Set target register
  if (!bus->write(I2C_SLV4_REG, reg))
    return false;

  // Start transaction (EN bit)
  if (!bus->write(I2C_SLV4_CTRL, (uint8_t)128))
    return false;
  // Wait for completion (with timeout)
  uint32_t start = millis();
  uint8_t status;

  while (millis() - start < 10)
  {
    if (bus->read(I2C_SLV4_CTRL, status))
      if (!(status & 0x80)) 
        return true;
  }

  // Timeout
  return false;
}

bool ICM20948_7Semi::readSlave4(uint8_t reg, uint8_t &value)
{
  // Select USER BANK 3
  if (!selectBank(3))
    return false;

  // Set slave address (read)
  if (!bus->write(I2C_SLV4_ADDR, (uint8_t)(AK09916_I2C_ADDR | 0x80)))
    return false;

  // Set target register
  if (!bus->write(I2C_SLV4_REG, reg))
    return false;

  // Start transaction
  if (!bus->write(I2C_SLV4_CTRL, (uint8_t)128))
    return false;
  // Wait for completion
  uint32_t start = millis();
  uint8_t status;
  while (millis() - start < 10)
  {
    if (bus->read(I2C_SLV4_CTRL, status))
      if (!(status & 0x80)) 
       { if (!bus->read(I2C_SLV4_DI, value))
    return false;
    return true;
}
  }
    return false;
}

bool ICM20948_7Semi::setLowPower(bool enable)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Set or clear LP_EN bit (bit 5)
  if (!bus->writeBit(PWR_MGMT_1, 5, (uint8_t)enable))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getLowPower(bool &enable)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Read LP_EN bit (bit 5)
  uint8_t v = 0;
  if (!bus->readBit(PWR_MGMT_1, 5, v))
    return false;

  // Convert bit value to boolean
  enable = (v == 1);

  // Success
  return true;
}

bool ICM20948_7Semi::setClock(ICM20948_Clock_Source clock)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Set CLKSEL bits [2:0]
  if (!bus->write(PWR_MGMT_1, (uint8_t)clock))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getClock(uint8_t &clock)
{
  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Read CLKSEL bits [2:0]
  if (!bus->read(PWR_MGMT_1, clock))
    return false;

    clock &= 0x07;

  // Success
  return true;
}

bool ICM20948_7Semi::setGyroSampleRate(float sampleRate)
{
  // Validate sample rate range (approx 4.3 Hz to 1100 Hz)
  if ((sampleRate < 4.3f) || (sampleRate > 1100.0f))
    return false;

  // Compute divider value
  uint8_t v = (uint8_t)((1100.0f / sampleRate) - 1.0f);

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Write sample rate divider
  if (!bus->write(GYRO_SMPLRT_DIV, v))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getGyroSampleRate(float &sampleRate)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Read sample rate divider
  uint8_t v = 0;
  if (!bus->read(GYRO_SMPLRT_DIV, v))
    return false;

  // Compute sample rate
  sampleRate = 1100.0f / (1.0f + v);

  // Success
  return true;
}

bool ICM20948_7Semi::setDLPF(ICM20948_Gyro_DLPF dlpf, bool bypass)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Enable or disable DLPF bypass (bit 0)
  if (!bus->writeBit(GYRO_CONFIG_1, 0, (uint8_t)bypass))
    return false;

  // If bypass enabled, skip DLPF configuration
  if (bypass)
    return true;

  // Set DLPF configuration bits [5:3]
  if (!bus->writeBits(GYRO_CONFIG_1, 3, 3, (uint8_t)dlpf))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getDLPF(uint8_t &dlpf, bool &bypass)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Read GYRO_CONFIG_1 register
  uint8_t v = 0;
  if (!bus->read(GYRO_CONFIG_1, v))
    return false;

  // Extract bypass bit (bit 0)
  bypass = (v & 0x01);

  // Extract DLPF bits [5:3]
  dlpf = (v >> 3) & 0x07;

  // Success
  return true;
}

bool ICM20948_7Semi::setGyroScale(ICM20948_Gyro_FullScale fullScale)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Set full-scale range bits [2:1]
  if (!bus->writeBits(GYRO_CONFIG_1, 1, 2, (uint8_t)fullScale))
    return false;

  // Update internal scale (LSB per g)
   degree_per_second = 32768.0f / (250.0f * (1 << ((uint8_t)fullScale)));

  // Success
  return true;
}

bool ICM20948_7Semi::getGyroScale(uint8_t &fullScale)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Read full-scale range bits [2:1]
  if (!bus->readBits(GYRO_CONFIG_1, 1, 2, fullScale))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::selfTestGyro(bool x, bool y, bool z)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Set Z-axis self-test (bit 3)
  if (!bus->writeBit(GYRO_CONFIG_2, 3, (uint8_t)z))
    return false;

  // Set Y-axis self-test (bit 4)
  if (!bus->writeBit(GYRO_CONFIG_2, 4, (uint8_t)y))
    return false;

  // Set X-axis self-test (bit 5)
  if (!bus->writeBit(GYRO_CONFIG_2, 5, (uint8_t)x))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::setAccelSampleRate(uint16_t sampleRate)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Validate input
  if (sampleRate == 0)
    return false;

  // Clamp maximum rate
  if (sampleRate >= 1125)
    return false;

  // Compute divider (rounded)
  uint16_t div = (1125.0f / sampleRate) - 1;

  if (div > 4095u)
    div = 4095u;

  // Split divider into high and low bytes
  uint8_t msb = (uint8_t)((div >> 8) & 0x0F);
  uint8_t lsb = (uint8_t)(div & 0xFF);

  // Write divider high byte
  if (!bus->write(ACCEL_SMPLRT_DIV_1, msb))
    return false;

  // Write divider low byte
  if (!bus->write(ACCEL_SMPLRT_DIV_2, lsb))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getAccelSampleRate(float &sampleRate)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Read divider high byte
  uint8_t msb = 0;
  if (!bus->read(ACCEL_SMPLRT_DIV_1, msb))
    return false;

  // Read divider low byte
  uint8_t lsb = 0;
  if (!bus->read(ACCEL_SMPLRT_DIV_2, lsb))
    return false;

  // Combine 12-bit divider
  uint16_t div = ((msb & 0x0F) << 8) | lsb;

  // Compute sample rate
  sampleRate = 1125.0f / (1.0f + div);

  // Success
  return true;
}

bool ICM20948_7Semi::selfTestAccel(bool x, bool y, bool z)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  if (x)
  { // Set X-axis self-test (bit 7)
    if (!bus->writeBit(ACCEL_CONFIG_2, 7, (uint8_t)x))
      return false;
  }

  if (y)
  { // Set Y-axis self-test (bit 6)
    if (!bus->writeBit(ACCEL_CONFIG_2, 6, (uint8_t)y))
      return false;
  }

  if (z)
  { // Set Z-axis self-test (bit 5)
    if (!bus->writeBit(ACCEL_CONFIG_2, 5, (uint8_t)z))
      return false;
  }
  // Success
  return true;
}

bool ICM20948_7Semi::setAccelScale(ICM20948_Accel_FullScale fullScale)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Set full-scale bits [2:1]
  if (!bus->writeBits(ACCEL_CONFIG, 1, 2, (uint8_t)fullScale))
    return false;

  // Update internal scale (LSB per g)
  mg_per_lsb = 16384.0f / (1 << (uint8_t)fullScale);

  // Success
  return true;
}

bool ICM20948_7Semi::getAccelScale(uint8_t &fullScale)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Read full-scale bits [2:1]
  if (!bus->readBits(ACCEL_CONFIG, 1, 2, fullScale))
    return false;

  // Success
  return true;
}


bool ICM20948_7Semi::setAccelDLPF(uint8_t dlpf, bool bypass)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Set or clear DLPF bypass (bit 0)
  if (!bus->writeBit(ACCEL_CONFIG, 0, (uint8_t)bypass))
    return false;

  // If bypass enabled, skip DLPF configuration
  if (bypass)
    return true;

  // Limit DLPF value to valid range
  dlpf &= 0x07;

  // Set DLPF configuration bits [5:3]
  if (!bus->writeBits(ACCEL_CONFIG, 3, 3, dlpf))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getAccelDLPF(uint8_t &dlpf, bool &bypass)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Read ACCEL_CONFIG register
  uint8_t v = 0;
  if (!bus->read(ACCEL_CONFIG, v))
    return false;

  // Extract bypass bit (bit 0)
  bypass = (v & 0x01);

  // Extract DLPF bits [5:3]
  dlpf = (v >> 3) & 0x07;

  // Success
  return true;
}


bool ICM20948_7Semi::setAccelAveraging(ICM20948_Accel_Average avg)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Set DEC3 averaging bits [1:0]
  if (!bus->writeBits(ACCEL_CONFIG_2, 0, 2, (uint8_t)avg))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getAccelAveraging(uint8_t &avg)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Read DEC3 averaging bits [1:0]
  if (!bus->readBits(ACCEL_CONFIG_2, 0, 2, avg))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::setSensors(bool accel_on, bool gyro_on, bool temp_on)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Build PWR_MGMT_2 mask
  uint8_t v = 0;

  // Disable accel axes if not enabled
  if (!accel_on)
    v |= 0x38; // bits [5:3]

  // Disable gyro axes if not enabled
  if (!gyro_on)
    v |= 0x07; // bits [2:0]

  // Write sensor enable/disable mask
  if (!bus->write(PWR_MGMT_2, v))
    return false;

  // Set or clear TEMP_DIS bit (bit 3)
  if (!bus->writeBit(PWR_MGMT_1, 3, (uint8_t)!temp_on))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getSensors(bool &accel_on, bool &gyro_on, bool &temp_on)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 0
  if (!selectBank(0))
    return false;

  // Read PWR_MGMT_2 register
  uint8_t v = 0;
  if (!bus->read(PWR_MGMT_2, v))
    return false;

  // Decode gyro state (bits [2:0])
  gyro_on = ((v & 0x07) == 0);

  // Decode accel state (bits [5:3])
  accel_on = ((v & 0x38) == 0);

  // Read TEMP_DIS bit (bit 3)
  uint8_t t = 0;
  if (!bus->readBit(PWR_MGMT_1, 3, t))
    return false;

  // Decode temperature state
  temp_on = (t == 0);

  // Success
  return true;
}

bool ICM20948_7Semi::setGyroOffset(uint16_t offsetX, uint16_t offsetY, uint16_t offsetZ)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Pack offset values into byte array
  uint8_t data[6] = {
      (uint8_t)(offsetX >> 8),
      (uint8_t)(offsetX & 0xFF),
      (uint8_t)(offsetY >> 8),
      (uint8_t)(offsetY & 0xFF),
      (uint8_t)(offsetZ >> 8),
      (uint8_t)(offsetZ & 0xFF)};

  // Write offset registers
  if (!bus->write(XG_OFFS_USRH, data, 6))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getGyroOffset(int16_t &offsetX, int16_t &offsetY, int16_t &offsetZ)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 2
  if (!selectBank(2))
    return false;

  // Read offset registers
  uint8_t data[6];
  if (!bus->read(XG_OFFS_USRH, data, 6))
    return false;

  // Convert to signed values
  offsetX = (int16_t)((data[0] << 8) | data[1]);
  offsetY = (int16_t)((data[2] << 8) | data[3]);
  offsetZ = (int16_t)((data[4] << 8) | data[5]);

  // Success
  return true;
}


bool ICM20948_7Semi::setAccelOffset(int16_t offsetX, int16_t offsetY, int16_t offsetZ)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 1 (accel offsets are here)
  if (!selectBank(1))
    return false;

  // Pack offset values into byte array
  uint8_t data[6] = {
      (uint8_t)(offsetX >> 8),
      (uint8_t)(offsetX & 0xFF),
      (uint8_t)(offsetY >> 8),
      (uint8_t)(offsetY & 0xFF),
      (uint8_t)(offsetZ >> 8),
      (uint8_t)(offsetZ & 0xFF)};

  // Write offset registers
  if (!bus->write(XA_OFFS_H, data, 6))
    return false;

  // Success
  return true;
}

bool ICM20948_7Semi::getAccelOffset(int16_t &offsetX, int16_t &offsetY, int16_t &offsetZ)
{
  // Validate bus pointer
  if (!bus)
    return false;

  // Select USER BANK 1
  if (!selectBank(1))
    return false;

  // Read offset registers
  uint8_t data[6];
  if (!bus->read(XA_OFFS_H, data, 6))
    return false;

  // Convert to signed values
  offsetX = (int16_t)((data[0] << 8) | data[1]);
  offsetY = (int16_t)((data[2] << 8) | data[3]);
  offsetZ = (int16_t)((data[4] << 8) | data[5]);

  // Success
  return true;
}
