#include "encoder.h"
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>

// parameter //
bool isReferenceSet = false;
float currentAngle = 0;
float previousHeight = NAN;
float initialAngle = 0;
float relativeAngle = 0;
const float caribHeight = 5.0f;
float height = 0.0;
float heightOffset = 0.0f;

// AS5600 constants
#define AS5600_ADDR 0x36
#define REG_ZMCO 0x00
#define REG_ZPOS 0x01
#define REG_MPOS 0x03
#define REG_MANG 0x05
#define REG_RAW_ANGLE 0x0C
#define REG_BURN 0xFF
#define BURN_CMD_ZPOS_MPOS_MANG 0x40

// float newScale = 1.0f;

float scaleFactor = 1.394124f;
float offset = 0.0f;

// ---- I2C Utility ----
static bool writeRegister16_internal(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    Wire.write((value >> 8) & 0xFF);
    Wire.write(value & 0xFF);
    int rc = Wire.endTransmission();
    return (rc == 0);
}

static uint16_t readRegister16_internal(uint8_t reg) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return 0xFFFF;
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0xFFFF;
    uint16_t hi = Wire.read();
    uint16_t lo = Wire.read();
    return (hi << 8) | lo;
}

static int readRegister8_internal(uint8_t reg) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    if (Wire.endTransmission(false) != 0) return -1;
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1);
    if (Wire.available() < 1) return -1;
    return Wire.read();
}

uint16_t readRawAngle() {
    uint16_t v = readRegister16_internal(REG_RAW_ANGLE);
    if (v == 0xFFFF) return 0;
    return v & 0x0FFF;
}

float readEncoderAngle() {
    uint16_t raw = readRawAngle();
    // keep same mapping as earlier project: optionally invert if needed
    // here we convert raw to degrees 0..360
    return (float)raw * (360.0f / 4096.0f);
}

bool isBurned() {
 Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_BURN);   // 0xFF
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1);
    if (Wire.available()) {
        uint8_t burnStatus = Wire.read();
        return (burnStatus & 0x80);
    }
    return false;
}

// write ZPOS/MPOS/MANG (volatile), then issue burn command (OTP).
// returns true if burn appears to have incremented ZMCO (success)
bool burnAngleAndMANG(uint16_t zpos, uint16_t mpos, uint16_t *outMang) {
    // compute MANG (modular)
    uint16_t mang;
    if (mpos >= zpos) mang = mpos - zpos;
    else mang = 4096 + mpos - zpos;

    if (outMang) *outMang = mang;

    // write registers
    if (!writeRegister16_internal(REG_ZPOS, zpos)) return false;
    delay(20);
    if (!writeRegister16_internal(REG_MPOS, mpos)) return false;
    delay(20);
    if (!writeRegister16_internal(REG_MANG, mang)) return false;
    delay(30); // allow device to commit volatile values

    // read ZMCO (before)
    int before = readRegister8_internal(REG_ZMCO);
    if (before < 0) return false;

    if (before >= 3) {
        // no more burns allowed
        return false;
    }

    // Issue burn command. As per datasheet use 0x40 for burning ZPOS/MPOS/MANG.
    uint8_t status;
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(REG_BURN);
    Wire.write(0x80);
    int rc = Wire.endTransmission();
    if (rc != 0) {
        // try one more time
        delay(20);
        Wire.beginTransmission(AS5600_ADDR);
        Wire.write(REG_BURN);
        Wire.write(0x80);
        rc = Wire.endTransmission();
        if (rc != 0) return false;
    }

    // wait OTP commit (datasheet suggests wait; use 100-200ms)
    delay(200);

    // read ZMCO (after)
    
    return true;
}

bool writeZPOSandMANG_volatile(uint16_t zpos, uint16_t mang) {
    if (!writeRegister16_internal(REG_ZPOS, zpos)) return false;
    delay(10);
    if (!writeRegister16_internal(REG_MANG, mang)) return false;
    delay(10);
    return true;
}

// set ZPOS (volatile)
bool setZeroPosition(uint16_t zeroPosition) {
    return writeRegister16_internal(REG_ZPOS, zeroPosition);
}

// set MANG (volatile)
bool setMaxAngle(uint16_t maxAngle) {
    return writeRegister16_internal(REG_MANG, maxAngle);
}

void saveCurrentZeroPositionToEEPROM() {
    uint16_t rawAngle = readRawAngle();
    setZeroPosition(rawAngle);
    EEPROM.put(ADDR_ZERO_POS, rawAngle);
}

void restoreZeroPositionFromEEPROM() {
    uint16_t rawAngle = 0;
    EEPROM.get(ADDR_ZERO_POS, rawAngle);
    if (rawAngle <= 4095) {
    setZeroPosition(rawAngle);
    }
}

void setInitialAngleFromSensor() {
    initialAngle = readEncoderAngle();
    EEPROM.put(ADDR_INITIAL_ANGLE, initialAngle);
    isReferenceSet = true;
}

// ---- キャリブレーション関連 ----
void restoreCalibrationFromEEPROM() {
    EEPROM.get(ADDR_NEWSCALE, newScale);
    EEPROM.get(ADDR_OFFSET, offset);
    if (isnan(newScale) || newScale == 0) {
        newScale = 1.394124f;
        offset = 0.034143f + caribHeight;
    }
}

void saveCalibrationToEEPROM(float scale, float newOffset) {
    newScale = scale;
    offset = newOffset;
    EEPROM.put(ADDR_NEWSCALE, newScale);
    EEPROM.put(ADDR_OFFSET, newOffset);
}

float interpolateHeight(float angle) {
    const int NUM_POINTS = 5;
    float knownAngles[NUM_POINTS];
    float knownHeight[NUM_POINTS];
    for (int i = 0; i < NUM_POINTS; i++) {
        EEPROM.get(ADDR_KNOWN_ANGLES + i * sizeof(float), knownAngles[i]);
        EEPROM.get(ADDR_KNOWN_HEIGHTS + i * sizeof(float), knownHeight[i]);
    }
    for (int i = 0; i < NUM_POINTS - 1; i++) {
        for (int j = i + 1; j < NUM_POINTS; j++) {
            if (knownAngles[i] > knownAngles[j]) {
                float ta = knownAngles[i]; knownAngles[i] = knownAngles[j]; knownAngles[j] = ta;
                float th = knownHeight[i]; knownHeight[i] = knownHeight[j]; knownHeight[j] = th;
            }
        }
    }
    if (angle <= knownAngles[0]) {
        float t = (angle - knownAngles[0]) / (knownAngles[1] - knownAngles[0]);
        return knownHeight[0] + t * (knownHeight[1] - knownHeight[0]);
    }
    if (angle >= knownAngles[NUM_POINTS - 1]) {
        float t = (angle - knownAngles[NUM_POINTS - 2]) / (knownAngles[NUM_POINTS - 1] - knownAngles[NUM_POINTS - 2]);
        return knownHeight[NUM_POINTS - 2] + t * (knownHeight[NUM_POINTS - 1] - knownHeight[NUM_POINTS - 2]);
    }
    for (int i = 0; i < NUM_POINTS - 1; i++) {
        if (angle >= knownAngles[i] && angle <= knownAngles[i + 1]) {
            float t = (angle - knownAngles[i]) / (knownAngles[i + 1] - knownAngles[i]);
            return knownHeight[i] + t * (knownHeight[i + 1] - knownHeight[i]);
        }
    }
    return 0.0f;
}

float updateHeight() {
    if (!isReferenceSet) return 0.0f;
    currentAngle = readEncoderAngle();
    float rawHeight = interpolateHeight(currentAngle);
    height = rawHeight - heightOffset;
    return height;
}

void initEncoder() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    while (Wire.available()) Wire.read(); 

}

uint8_t readBurnCount() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_BURN);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1);
    if (Wire.available() < 1) return 0xFF;
    return Wire.read();    
}