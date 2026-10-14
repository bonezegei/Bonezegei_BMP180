/*
  Bonezegei_BMP180.h - Arduino Library for BMP180 Sensor
  Author: Jofel Batutay (Bonezegei)
  Date: October 14, 2026
*/

#ifndef BONEZEGEI_BMP180_H
#define BONEZEGEI_BMP180_H

#include <Arduino.h>
#include <Wire.h>

#define BMP180_I2C_ADDR           0x77

#define BMP180_REG_CHIP_ID        0xD0
#define BMP180_REG_RESET          0xE0
#define BMP180_REG_CONTROL        0xF4
#define BMP180_REG_RESULT         0xF6

#define BMP180_CMD_READ_TEMP      0x2E
#define BMP180_CMD_READ_PRESSURE  0x34

#define BMP180_MODE_ULTRALOWPOWER 0
#define BMP180_MODE_STANDARD      1
#define BMP180_MODE_HIGHRES       2
#define BMP180_MODE_ULTRAHIGHRES  3

struct BMP180_CalibData {
  int16_t  ac1;
  int16_t  ac2;
  int16_t  ac3;
  uint16_t ac4;
  uint16_t ac5;
  uint16_t ac6;
  int16_t  b1;
  int16_t  b2;
  int16_t  mb;
  int16_t  mc;
  int16_t  md;
};

class Bonezegei_BMP180 {
public:
  Bonezegei_BMP180(uint8_t addr = BMP180_I2C_ADDR);
  
  bool begin(TwoWire &wirePort = Wire);
  bool read(uint8_t mode = BMP180_MODE_ULTRAHIGHRES);
  
  float getTemperature();  // Returns Temperature in Celsius
  float getPressure();     // Returns Pressure in Pascals (Pa)
  float getAltitude(float seaLevelhPa = 1013.25); // Returns Altitude in meters

private:
  uint8_t _addr;
  TwoWire *_wire;
  BMP180_CalibData _calib;
  int32_t _b5;

  float _temperature;
  float _pressure;

  void readCalibrationData();
  void writeRegister(uint8_t reg, uint8_t value);
  uint8_t readRegister8(uint8_t reg);
  uint16_t readRegister16_BE(uint8_t reg);
  int16_t readRegister16_S_BE(uint8_t reg);

  int32_t readRawTemperature();
  int32_t readRawPressure(uint8_t mode);

  int32_t compensateTemp(int32_t ut);
  uint32_t compensatePressure(int32_t up, uint8_t mode);
};

#endif
