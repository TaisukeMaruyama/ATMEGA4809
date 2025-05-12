#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Wire.h>

#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C
#define AS5600_ZPOS 0x01
#define AS5600_MANG 0x05

void setZeroPosition(uint16_t zeroPosition);
void setMaxAngle(uint16_t maxAngle);
float readEncoderAngle();
void saveCurrentZeroPositionToEEPROM();
void restoreZeroPositionFromEEPROM();


#endif