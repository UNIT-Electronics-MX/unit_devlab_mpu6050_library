/*
  DevLab_MPU6050.cpp

  Implementation file for the DevLab_MPU6050 Arduino library.
  This library communicates with the MPU6050 IMU using direct
  register access over I2C.

  Author:
  UNIT Electronics
*/

#include "DevLab_MPU6050.h"

DevLab_MPU6050::DevLab_MPU6050(TwoWire &wire, uint8_t address) {
  _wire = &wire;
  _address = address;
  _accelScale = 0;
  _gyroScale = 0;
}

bool DevLab_MPU6050::begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock) {
  _wire->begin(sdaPin, sclPin);
  _wire->setClock(clock);

  delay(50);

  uint8_t whoAmI = readRegister8(REG_WHO_AM_I);
  if (whoAmI != 0x68) {
    return false;
  }

  writeRegister8(REG_PWR_MGMT_1, 0x00);
  delay(10);

  writeRegister8(REG_SMPLRT_DIV, 0x07);
  writeRegister8(REG_CONFIG, 0x06);
  writeRegister8(REG_GYRO_CONFIG, 0x08);
  writeRegister8(REG_ACCEL_CONFIG, 0x00);

  return true;
}

bool DevLab_MPU6050::readData(SensorData &dataOut) {
  _wire->beginTransmission(_address);
  _wire->write(REG_ACCEL_XOUT_H);
  if (_wire->endTransmission(false) != 0) {
    return false;
  }

  _wire->requestFrom(_address, (uint8_t)14, (uint8_t)true);

  if (_wire->available() < 14) {
    return false;
  }

  dataOut.accX = (int16_t)(_wire->read() << 8 | _wire->read());
  dataOut.accY = (int16_t)(_wire->read() << 8 | _wire->read());
  dataOut.accZ = (int16_t)(_wire->read() << 8 | _wire->read());

  dataOut.tempRaw = (int16_t)(_wire->read() << 8 | _wire->read());
  dataOut.temperatureC = dataOut.tempRaw / 340.0f + 36.53f;

  dataOut.gyrX = (int16_t)(_wire->read() << 8 | _wire->read());
  dataOut.gyrY = (int16_t)(_wire->read() << 8 | _wire->read());
  dataOut.gyrZ = (int16_t)(_wire->read() << 8 | _wire->read());

  return true;
}

void DevLab_MPU6050::setAccelScale(uint8_t scale) {
  _accelScale = scale;
  writeRegister8(REG_ACCEL_CONFIG, scale);
}

void DevLab_MPU6050::setGyroScale(uint8_t scale) {
  _gyroScale = scale;
  writeRegister8(REG_GYRO_CONFIG, scale);
}

void DevLab_MPU6050::writeRegister8(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

uint8_t DevLab_MPU6050::readRegister8(uint8_t reg) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission(false);

  _wire->requestFrom(_address, (uint8_t)1, (uint8_t)true);
  if (_wire->available() < 1) {
    return 0;
  }

  return _wire->read();
}

int16_t DevLab_MPU6050::readRegister16(uint8_t reg) {
  _wire->beginTransmission(_address);
  _wire->write(reg);
  _wire->endTransmission(false);

  _wire->requestFrom(_address, (uint8_t)2, (uint8_t)true);
  if (_wire->available() < 2) {
    return 0;
  }

  return (int16_t)((uint16_t)_wire->read() << 8 | (uint16_t)_wire->read());
}
