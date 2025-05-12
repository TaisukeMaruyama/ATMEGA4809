#include "encoder.h"
#include <EEPROM.h>

void setZeroPosition(uint16_t zeroPosition) {
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_ZPOS);
    Wire.write(zeroPosition >> 8);
    Wire.write(zeroPosition & 0xFF);
    Wire.endTransmission();
}

void setMaxAngle(uint16_t maxAngle) {
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_MANG);
    Wire.write(maxAngle >> 8);
    Wire.write(maxAngle & 0xFF);
    Wire.endTransmission();
}

float readEncoderAngle() {
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);

    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();

    uint16_t RawAngle = ((uint16_t)highByte << 8) | lowByte;
    uint16_t invertedAngle = 4095 - RawAngle;
    return invertedAngle * (360.0 / 4096.0);
}

void saveCurrentZeroPositionToEEPROM(){
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS,2);
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    uint16_t rawAngle = ((uint16_t)highByte << 8) | lowByte;

    setZeroPosition(rawAngle);

    EEPROM.put(0,rawAngle);
    
}

void restoreZeroPositionFromEEPROM(){
    uint16_t rawAngle;
    EEPROM.get(0,rawAngle);
    setZeroPosition(rawAngle);
}
