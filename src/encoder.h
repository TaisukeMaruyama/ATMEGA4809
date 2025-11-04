#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>
#include <Wire.h>

//------------------------------------
// AS5600 I2C Register Map
//------------------------------------
#define AS5600_ADDR 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C
#define AS5600_ANGLE 0x0E
#define AS5600_ZPOS 0x01
#define AS5600_MPOS 0x03
#define AS5600_MANG 0x05
#define AS5600_REG_STATUS 0x0B
#define AS5600_REG_BURN 0xFF
#define AS5600_REG_ZMCO 0x00  // burn count register (0–3)

//------------------------------------
// EEPROM Address Map
//------------------------------------
#define ADDR_ZERO_POS       0
#define ADDR_INITIAL_ANGLE  2
#define ADDR_NEWSCALE      10
#define ADDR_OFFSET        14
#define ADDR_KNOWN_ANGLES 100
#define ADDR_KNOWN_HEIGHTS 200
#define ADDR_HEIGHT_OFFSET 300
#define ADDR_MPOS          304
#define ADDR_MANG          306

//------------------------------------
// Extern Variables
//------------------------------------
extern float initialAngle;
extern float previousHeight;
extern bool  isReferenceSet;
extern float newScale;
extern float heightOffset;
extern float currentAngle;

//------------------------------------
// Encoder Core Functions
//------------------------------------
void   initEncoder();
float  readEncoderAngle();
uint16_t readRawAngle();
bool   isBurned();                              // burn済み判定（ZMCO>0）
bool   setZeroPosition(uint16_t zeroPosition);
bool   setMaxAngle(uint16_t maxAngle);
uint8_t readBurnCount();

//------------------------------------
// Burn-related Functions
//------------------------------------
bool   burnAngleAndMANG(uint16_t zpos, uint16_t mpos, uint16_t *outMang = nullptr);
bool   writeZPOSandMANG_volatile(uint16_t zpos, uint16_t mang);

//------------------------------------
// EEPROM / Calibration Functions
//------------------------------------
void   saveCurrentZeroPositionToEEPROM();
void   restoreZeroPositionFromEEPROM();
void   setInitialAngleFromSensor();
void   loadInitialAngleFromEEPROM();
void   restoreCalibrationFromEEPROM();
float  updateHeight();
float  interpolateHeight(float angle);

//------------------------------------
// Utility
//------------------------------------
uint16_t readRegister16(uint8_t reg);
void     writeRegister16(uint8_t reg, uint16_t value);

#endif
