#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Wire.h>

#define AS5600_ADDR 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C
#define AS5600_ZPOS 0x01
#define AS5600_MANG 0x05
#define AS5600_MPOS 0x03
#define AS5600_ANGLE 0x0E
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_BURN 0xFF
#define AS5600_BURN_ANGLE 0x80
#define AS5600_BURN_SETTING 0x40

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
uint16_t readRegister16(uint8_t reg);
uint8_t readRegister(uint8_t reg);
void saveAS5600RegistersToEEPROM(uint16_t zpos, uint16_t mpos, uint16_t mang);
void restoreAS5600RegistersFromEEPROM();

void initEncoder();
void setZeroAndRange();
uint16_t readRawAngle();
float readCalibratedAngle();

void writeZPOSandMANG(uint16_t zpos, uint16_t mang);
bool burnAngle();
bool burnSetting();
bool isBurned();
void burnAngleAndMANG(uint16_t zeroPos, uint16_t maxAngle);
uint8_t readBurnCount();

#endif