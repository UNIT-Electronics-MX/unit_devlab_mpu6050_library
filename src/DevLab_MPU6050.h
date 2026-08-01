/*
  DevLab_MPU6050.h

  Arduino library for the MPU6050 6-axis IMU
  using direct register communication over I2C.

  Features:
  - Accelerometer reading
  - Gyroscope reading
  - Temperature reading
  - Direct register access
  - Compatible with ESP32 and Arduino-compatible boards

  Author:
  UNIT Electronics

  License:
  MIT License
*/

#ifndef DEVLAB_MPU6050_H
#define DEVLAB_MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDRESS_AD0_LOW  0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69

#define REG_SMPLRT_DIV       0x19
#define REG_CONFIG           0x1A
#define REG_GYRO_CONFIG      0x1B
#define REG_ACCEL_CONFIG     0x1C
#define REG_ACCEL_XOUT_H     0x3B
#define REG_TEMP_OUT_H       0x41
#define REG_GYRO_XOUT_H      0x43
#define REG_PWR_MGMT_1       0x6B
#define REG_WHO_AM_I         0x75

class DevLab_MPU6050 {
public:
  struct SensorData {
    int16_t accX;
    int16_t accY;
    int16_t accZ;

    int16_t gyrX;
    int16_t gyrY;
    int16_t gyrZ;

    int16_t tempRaw;
    float temperatureC;
  };

  DevLab_MPU6050(TwoWire &wire = Wire, uint8_t address = MPU6050_ADDRESS_AD0_LOW);

  bool begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock = 400000);
  bool readData(SensorData &dataOut);

  void setAccelScale(uint8_t scale);
  void setGyroScale(uint8_t scale);

private:
  TwoWire *_wire;
  uint8_t _address;

  uint8_t _accelScale;
  uint8_t _gyroScale;

  void writeRegister8(uint8_t reg, uint8_t value);
  uint8_t readRegister8(uint8_t reg);
  int16_t readRegister16(uint8_t reg);
};

#endif
