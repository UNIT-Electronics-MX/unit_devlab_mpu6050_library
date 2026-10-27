/*
  DevLab_MPU6050.cpp

  Implementation file for the DevLab_MPU6050 Arduino library.
  This library communicates with the MPU6050 IMU using direct
  register access over I2C.

  Author:
  Jonathan Mejorado
  UNIT Electronics
*/

#include "DevLab_MPU6050.h"

DevLab_MPU6050::DevLab_MPU6050(TwoWire &wire, uint8_t address)
  : _bus(wire, address) {
  _accelScale = 0;
  _gyroScale = 0;
}

bool DevLab_MPU6050::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock) {
  _bus.setClock(clock);
  _bus.begin(sdaPin, sclPin);
  return initDevice();
}

bool DevLab_MPU6050::begin(uint32_t clock) {
  _bus.setClock(clock);
  _bus.begin();
  return initDevice();
}

bool DevLab_MPU6050::initDevice() {
  delay(50);

  uint8_t whoAmI = WhoAmI();
  if (whoAmI != 0x68) {
    return false;
  }

  writeRegister8(REG_PWR_MGMT_1, 0x01); // Wake up the MPU6050
  delay(10);

  writeRegister8(REG_SMPLRT_DIV, 0x07);
  writeRegister8(REG_CONFIG, 0x06);
  writeRegister8(REG_GYRO_CONFIG, 0x08);
  writeRegister8(REG_ACCEL_CONFIG, 0x10);

  return true;
}

uint8_t DevLab_MPU6050::WhoAmI() {
  return readRegister8(REG_WHO_AM_I);
}

bool DevLab_MPU6050::readData(SensorData &dataOut) {
  uint8_t buf[14];
  if (!_bus.readBytes(REG_ACCEL_XOUT_H, buf, sizeof(buf))) {
    return false;
  }

  // The MPU6050 sends the high byte first (big-endian).
  dataOut.accX = (int16_t)((uint16_t)buf[0] << 8 | buf[1]);
  dataOut.accY = (int16_t)((uint16_t)buf[2] << 8 | buf[3]);
  dataOut.accZ = (int16_t)((uint16_t)buf[4] << 8 | buf[5]);

  dataOut.tempRaw = (int16_t)((uint16_t)buf[6] << 8 | buf[7]);
  dataOut.temperatureC = dataOut.tempRaw / 340.0f + 36.53f;

  dataOut.gyrX = (int16_t)((uint16_t)buf[8] << 8 | buf[9]);
  dataOut.gyrY = (int16_t)((uint16_t)buf[10] << 8 | buf[11]);
  dataOut.gyrZ = (int16_t)((uint16_t)buf[12] << 8 | buf[13]);

  return true;
}

void DevLab_MPU6050::setDLPF(uint8_t dlpSetting) {
  if(dlpSetting > 6) {
    return;
  }
  uint8_t currentConfig = readRegister8(REG_CONFIG);
  writeRegister8(REG_CONFIG,(currentConfig & 0xF8) | (dlpSetting & 0x07));
}

void DevLab_MPU6050::setAccelScale(uint8_t scale) {
  
  _accelScale = scale;

    if(scale > 3) {
    return;
  }
  uint8_t currentConfig = readRegister8(REG_ACCEL_CONFIG);
  writeRegister8(REG_ACCEL_CONFIG, (currentConfig & 0xE7) | ((scale & 0x03) << 3));
}

void DevLab_MPU6050::setGyroScale(uint8_t scale) {
  _gyroScale = scale;

  if(scale > 3) {
    return;
  }
  uint8_t currentConfig = readRegister8(REG_GYRO_CONFIG);
  writeRegister8(REG_GYRO_CONFIG, (currentConfig & 0xE7) | ((scale & 0x03) << 3));
}

void DevLab_MPU6050::sampleRateDiv(uint8_t div) {
  writeRegister8(REG_SMPLRT_DIV, div);
}

void DevLab_MPU6050::setClkSource(uint8_t clkSource) {
  if(clkSource > 7) {
    return;
  }
  uint8_t currentConfig = readRegister8(REG_PWR_MGMT_1);
  writeRegister8(REG_PWR_MGMT_1, (currentConfig & 0xF8) | (clkSource & 0x07));
}

void DevLab_MPU6050::setIntPinCfg(uint8_t intPinCfg) {
  writeRegister8(REG_INT_PIN_CFG, intPinCfg);
}

void DevLab_MPU6050::setIntPinCfg(const interruptPinConfig &cfg) {
  uint8_t value = (cfg.activeLow            << 7) |
                  (cfg.openDrain             << 6) |
                  (cfg.latchUntilCleared     << 5) |
                  (cfg.clearOnRead           << 4) |
                  (cfg.fsyncActiveLow        << 3) |
                  (cfg.enableFSyncInterrupt  << 2) |
                  (cfg.enableI2CBypass       << 1);
  writeRegister8(REG_INT_PIN_CFG, value);
}

void DevLab_MPU6050::enableInterrupt(uint8_t sources) {
  uint8_t currentConfig = readRegister8(REG_INT_ENABLE);
  writeRegister8(REG_INT_ENABLE, currentConfig | sources);
}

void DevLab_MPU6050::disableInterrupt(uint8_t sources) {
  uint8_t currentConfig = readRegister8(REG_INT_ENABLE);
  writeRegister8(REG_INT_ENABLE, currentConfig & ~sources);
}

uint8_t DevLab_MPU6050::interruptStatus() {
  return readRegister8(REG_INT_STATUS);
}





/******* Private Utility functions for register access *******/
void DevLab_MPU6050::writeRegister8(uint8_t reg, uint8_t value) {
  _bus.writeRegister8(reg, value);
}

uint8_t DevLab_MPU6050::readRegister8(uint8_t reg) {
  uint8_t value = 0;
  _bus.readRegister8(reg, value);
  return value;
}

int16_t DevLab_MPU6050::readRegister16(uint8_t reg) {
  uint16_t value = 0;
  _bus.readRegister16BE(reg, value);
  return (int16_t)value;
}
