/*
  Bonezegei_BMP180.cpp - Arduino Library for BMP180 Sensor
  Author: Jofel Batutay (Bonezegei)
  Date: October 14, 2026
*/

#include "Bonezegei_BMP180.h"

Bonezegei_BMP180::Bonezegei_BMP180(uint8_t addr) {
  _addr = addr;
  _b5 = 0;
  _temperature = 0.0;
  _pressure = 0.0;
}

bool Bonezegei_BMP180::begin(TwoWire &wirePort) {
  _wire = &wirePort;
  _wire->begin();

  // Check Chip ID (BMP180 chip ID is 0x55)
  if (readRegister8(BMP180_REG_CHIP_ID) != 0x55) {
    return false;
  }

  // Soft Reset
  writeRegister(BMP180_REG_RESET, 0xB6);
  delay(100);

  readCalibrationData();

  return true;
}

void Bonezegei_BMP180::readCalibrationData() {
  _calib.ac1 = readRegister16_S_BE(0xAA);
  _calib.ac2 = readRegister16_S_BE(0xAC);
  _calib.ac3 = readRegister16_S_BE(0xAE);
  _calib.ac4 = readRegister16_BE(0xB0);
  _calib.ac5 = readRegister16_BE(0xB2);
  _calib.ac6 = readRegister16_BE(0xB4);
  _calib.b1  = readRegister16_S_BE(0xB6);
  _calib.b2  = readRegister16_S_BE(0xB8);
  _calib.mb  = readRegister16_S_BE(0xBA);
  _calib.mc  = readRegister16_S_BE(0xBC);
  _calib.md  = readRegister16_S_BE(0xBE);
}

bool Bonezegei_BMP180::read(uint8_t mode) {
  int32_t ut = readRawTemperature();
  if (ut == 0) return false;

  int32_t up = readRawPressure(mode);
  if (up == 0) return false;

  int32_t temp_raw = compensateTemp(ut);
  _temperature = temp_raw / 10.0f;

  uint32_t press_raw = compensatePressure(up, mode);
  _pressure = (float)press_raw;

  return true;
}

float Bonezegei_BMP180::getTemperature() { return _temperature; }
float Bonezegei_BMP180::getPressure() { return _pressure; }

float Bonezegei_BMP180::getAltitude(float seaLevelhPa) {
  float atmospheric = _pressure / 100.0f; // Pa to hPa
  return 44330.0f * (1.0f - pow(atmospheric / seaLevelhPa, 0.1903f));
}

int32_t Bonezegei_BMP180::readRawTemperature() {
  writeRegister(BMP180_REG_CONTROL, BMP180_CMD_READ_TEMP);
  delay(5);
  return (int32_t)readRegister16_BE(BMP180_REG_RESULT);
}

int32_t Bonezegei_BMP180::readRawPressure(uint8_t mode) {
  if (mode > 3) mode = 3;
  writeRegister(BMP180_REG_CONTROL, BMP180_CMD_READ_PRESSURE + (mode << 6));

  switch (mode) {
    case BMP180_MODE_ULTRALOWPOWER: delay(5);  break;
    case BMP180_MODE_STANDARD:      delay(8);  break;
    case BMP180_MODE_HIGHRES:       delay(14); break;
    case BMP180_MODE_ULTRAHIGHRES:
    default:                        delay(26); break;
  }

  _wire->beginTransmission(_addr);
  _wire->write(BMP180_REG_RESULT);
  if (_wire->endTransmission() != 0) return 0;

  _wire->requestFrom(_addr, (uint8_t)3);
  if (_wire->available() < 3) return 0;

  uint32_t msb  = _wire->read();
  uint32_t lsb  = _wire->read();
  uint32_t xlsb = _wire->read();

  return (int32_t)(((msb << 16) | (lsb << 8) | xlsb) >> (8 - mode));
}

int32_t Bonezegei_BMP180::compensateTemp(int32_t ut) {
  int32_t x1 = ((ut - (int32_t)_calib.ac6) * (int32_t)_calib.ac5) >> 15;
  int32_t x2 = ((int32_t)_calib.mc << 11) / (x1 + _calib.md);
  _b5 = x1 + x2;
  return (_b5 + 8) >> 4;
}

uint32_t Bonezegei_BMP180::compensatePressure(int32_t up, uint8_t mode) {
  if (mode > 3) mode = 3;

  int32_t b6 = _b5 - 4000;
  int32_t x1 = (_calib.b2 * ((b6 * b6) >> 12)) >> 11;
  int32_t x2 = (_calib.ac2 * b6) >> 11;
  int32_t x3 = x1 + x2;
  int32_t b3 = (((((int32_t)_calib.ac1) * 4 + x3) << mode) + 2) >> 2;

  x1 = (_calib.ac3 * b6) >> 13;
  x2 = (_calib.b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  uint32_t b4 = (_calib.ac4 * (uint32_t)(x3 + 32768)) >> 15;
  uint32_t b7 = ((uint32_t)up - b3) * (50000 >> mode);

  int32_t p;
  if (b7 < 0x80000000) {
    p = (b7 * 2) / b4;
  } else {
    p = (b7 / b4) * 2;
  }

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p = p + ((x1 + x2 + 3791) >> 4);

  return (uint32_t)p;
}

void Bonezegei_BMP180::writeRegister(uint8_t reg, uint8_t value) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->write(value);
  _wire->endTransmission();
}

uint8_t Bonezegei_BMP180::readRegister8(uint8_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->endTransmission();
  _wire->requestFrom(_addr, (uint8_t)1);
  return _wire->read();
}

uint16_t Bonezegei_BMP180::readRegister16_BE(uint8_t reg) {
  _wire->beginTransmission(_addr);
  _wire->write(reg);
  _wire->endTransmission();
  _wire->requestFrom(_addr, (uint8_t)2);
  uint8_t msb = _wire->read();
  uint8_t lsb = _wire->read();
  return (msb << 8) | lsb;
}

int16_t Bonezegei_BMP180::readRegister16_S_BE(uint8_t reg) {
  return (int16_t)readRegister16_BE(reg);
}
