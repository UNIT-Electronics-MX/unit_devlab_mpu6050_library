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
  UNIT Electronics,Jonathan Mejorado Lopez

  License:
  MIT License
*/

#ifndef DEVLAB_MPU6050_H
#define DEVLAB_MPU6050_H

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_ADDRESS_AD0_LOW  0x68
#define MPU6050_ADDRESS_AD0_HIGH 0x69

#define REG_INT_PIN_CFG      0x37
#define REG_INT_ENABLE       0x38
#define REG_INT_STATUS       0x3A
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

  struct interruptPinConfig {
    bool activeLow           = false; // INT_LEVEL,      bit7
    bool openDrain           = false; // INT_OPEN,       bit6
    bool latchUntilCleared   = false; // LATCH_INT_EN,   bit5
    bool clearOnRead         = false; // INT_RD_CLEAR,   bit4
    bool fsyncActiveLow      = false; // FSYNC_INT_LEVEL,bit3
    bool enableFSyncInterrupt= false; // FSYNC_INT_EN,   bit2
    bool enableI2CBypass     = false; // I2C_BYPASS_EN,  bit1
  };

  enum InterruptSource : uint8_t {
    INT_DATA_READY    = 0x01, // bit0, DATA_RDY_EN
    INT_I2C_MST       = 0x08, // bit3, I2C_MST_INT_EN
    INT_FIFO_OVERFLOW = 0x10, // bit4, FIFO_OFLOW_EN
    INT_MOTION        = 0x40, // bit6, MOT_INT_EN
  };
  DevLab_MPU6050(TwoWire &wire = Wire, uint8_t address = MPU6050_ADDRESS_AD0_LOW);

  bool begin(uint8_t sdaPin, uint8_t sclPin, uint32_t clock = 400000);
  bool readData(SensorData &dataOut);

  void setAccelScale(uint8_t scale);
  void setGyroScale(uint8_t scale);
  /* Special functions and setting ups for the MPU6050 */

  /**
   * @brief Get the device ID.
   * @return The device ID.
   */
  uint8_t WhoAmI();

  /**
   * @brief Set the Digital Low Pass Filter (DLPF) setting.
   * @param dlpSetting The DLPF setting to apply.
   */
  void setDLPF(uint8_t dlpSetting);

  /**
   * @brief Set the sample rate divider.
   * @param div The sample rate divider to apply.
   */
  void sampleRateDiv(uint8_t div);

  /**
   * @brief Set the clock source.
   * @param clkSource The clock source to apply.
   */
  void setClkSource(uint8_t clkSource);

  /**
   * @brief Set the INT pin / bypass configuration register directly.
   * @param intPinCfg Raw byte to write to REG_INT_PIN_CFG.
   */
  void setIntPinCfg(uint8_t intPinCfg);

  /**
   * @brief Set the INT pin / bypass configuration from a struct.
   * @param cfg The interrupt pin configuration to apply.
   */
  void setIntPinCfg(const interruptPinConfig &cfg);

  /**
   * @brief Enable one or more interrupt sources (OR'd together), leaving others untouched.
   * @param sources Bitwise-OR of InterruptSource values.
   */
  void enableInterrupt(uint8_t sources);

  /**
   * @brief Disable one or more interrupt sources (OR'd together), leaving others untouched.
   * @param sources Bitwise-OR of InterruptSource values.
   */
  void disableInterrupt(uint8_t sources);
  
  /**
   * @brief Read the INT_STATUS register without side effects other than the hardware's own clear-on-read.
   * @return The interrupt status byte.
   */
  uint8_t interruptStatus();

private:
  TwoWire *_wire;
  uint8_t _address;

  uint8_t _accelScale;
  uint8_t _gyroScale;

  void writeRegister8(uint8_t reg, uint8_t value);
  uint8_t readRegister8(uint8_t reg);
  int16_t readRegister16(uint8_t reg);
};

using SensorData = DevLab_MPU6050::SensorData;
using interruptPinConfig = DevLab_MPU6050::interruptPinConfig;
constexpr auto INT_DATA_READY    = DevLab_MPU6050::INT_DATA_READY;
constexpr auto INT_I2C_MST       = DevLab_MPU6050::INT_I2C_MST;
constexpr auto INT_FIFO_OVERFLOW = DevLab_MPU6050::INT_FIFO_OVERFLOW;
constexpr auto INT_MOTION        = DevLab_MPU6050::INT_MOTION;
#endif
