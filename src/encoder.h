#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Wire.h>

#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0E
#define AS5600_ZPOS 0x01
#define AS5600_MANG 0x05
#define AS5600_MPOS 0x03
// #define AS5600_ANGLE 0x0E

#define ADDR_ZERO_POS 0
#define ADDR_INITIAL_ANGLE 2
#define ADDR_NEWSCALE 10
#define ADDR_OFFSET 14
#define ADDR_KNOWN_ANGLES 100
#define ADDR_KNOWN_HEIGHTS 200
#define ADDR_HEIGHT_OFFSET 300
#define ADDR_MPOS 304
#define ADDR_MANG 306

extern float initialAngle;
extern float previousHeight;
extern bool isReferenceSet;
extern float newScale;
extern float heightOffset;
extern float currentAngle;

void setZeroPosition(uint16_t zeroPosition);
void setMaxAngle(uint16_t maxAngle);
float readEncoderAngle();
void saveCurrentZeroPositionToEEPROM();
void restoreZeroPositionFromEEPROM();
void setInitialAngleFromSensor();
void loadInitialAngleFromEEPROM();
float updateHeight();
void calibrationMode();
void restoreCalibrationFromEEPROM();

float interpolateHeight(float angle);

void writeRegister16(uint8_t reg, uint16_t value);
void saveAS5600RegistersToEEPROM(uint16_t zpos, uint16_t mpos, uint16_t mang);
void restoreAS5600RegistersFromEEPROM();

void initEncorder();
void setZeroAndRange();
float readRawAngle();
float readCalibratedAngle();


#endif