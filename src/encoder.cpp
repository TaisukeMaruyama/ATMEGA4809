#include "encoder.h"
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>

// --------------------------------------------------
// グローバル変数
// --------------------------------------------------
bool isReferenceSet = false;
float currentAngle = 0;
float previousHeight = NAN;
float initialAngle = 0;
float relativeAngle = 0;
const float caribHeight = 5.0f;
float height = 0.0;
float heightOffset = 0.0f;

float newScale = 1.394124f;
float offset = 0.0f;

// --------------------------------------------------
// I2C Utility
// --------------------------------------------------
uint16_t readRegister16(uint8_t reg) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0;
    uint16_t val = ((uint16_t)Wire.read() << 8) | Wire.read();
    return val;
}

bool writeRegister16_checked(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    int rc = Wire.endTransmission();
    return (rc == 0);
}

void writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    Wire.endTransmission();
}

uint8_t readBurnCount() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(0x00);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1);
    if (Wire.available() < 1) return 0xFF;
    return Wire.read();
}

// --------------------------------------------------
// Burn Status
// --------------------------------------------------
bool isBurned() {
    uint8_t zmco = readBurnCount() & 0x03;
    return (zmco > 0 && zmco <= 3);
}

// --------------------------------------------------
// AS5600 Burn Function (完全版)
// --------------------------------------------------
bool burnAngleAndMANG(uint16_t zpos, uint16_t mpos, uint16_t *outMang) {
    // ---- MANG計算 ----
    uint16_t mang;
    if (mpos >= zpos) mang = mpos - zpos;
    else mang = 4096 + mpos - zpos;
    if (outMang) *outMang = mang;

    // ---- レジスタ書き込み ----
    if (!writeRegister16_checked(AS5600_ZPOS, zpos)) return false;
    delay(20);
    if (!writeRegister16_checked(AS5600_MPOS, mpos)) return false;
    delay(20);
    if (!writeRegister16_checked(AS5600_MANG, mang)) return false;
    delay(50);

    // ---- burn前の ZMCO ----
    uint8_t before = readBurnCount() & 0x03;
    if (before >= 3) return false; // OTP容量上限

    // ---- BURN_SETTING ----
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_BURN);
    Wire.write(0x40);
    Wire.endTransmission();
    delay(50);

        // ---- BURN_ANGLE ----
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_BURN);
    Wire.write(0x80);
    Wire.endTransmission();
    delay(1200);


    // ---- OTPリロード（0x01 → 0x11 → 0x10）----
    Wire.beginTransmission(AS5600_ADDR); Wire.write(0xFF); Wire.write(0x01); Wire.endTransmission(); delay(5);
    Wire.beginTransmission(AS5600_ADDR); Wire.write(0xFF); Wire.write(0x11); Wire.endTransmission(); delay(5);
    Wire.beginTransmission(AS5600_ADDR); Wire.write(0xFF); Wire.write(0x10); Wire.endTransmission(); delay(5);

    uint8_t after = readBurnCount() & 0x03;
    if (after > before) return true;


    // ---- 再リロード ----
    Wire.beginTransmission(AS5600_ADDR); Wire.write(0xFF); Wire.write(0x01); Wire.endTransmission(); delay(5);
    Wire.beginTransmission(AS5600_ADDR); Wire.write(0xFF); Wire.write(0x11); Wire.endTransmission(); delay(5);
    Wire.beginTransmission(AS5600_ADDR); Wire.write(0xFF); Wire.write(0x10); Wire.endTransmission(); delay(5);

    // ---- burn後の ZMCO ----
    delay(200);
    return (after > before);

   
}

// --------------------------------------------------
// 通常動作用関数群
// --------------------------------------------------
bool setZeroPosition(uint16_t zeroPosition) {
    return writeRegister16_checked(AS5600_ZPOS, zeroPosition);
}

bool setMaxAngle(uint16_t maxAngle) {
    return writeRegister16_checked(AS5600_MANG, maxAngle);
}

uint16_t readRawAngle() {
    return readRegister16(AS5600_AS5601_REG_RAW_ANGLE) & 0x0FFF;
}

float readEncoderAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return NAN;
    uint16_t val = ((uint16_t)Wire.read() << 8) | Wire.read();
    val &= 0x0FFF;
    return val * (360.0 / 4096.0);
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

void restoreCalibrationFromEEPROM() {
    EEPROM.get(ADDR_NEWSCALE, newScale);
    EEPROM.get(ADDR_OFFSET, offset);
    if (isnan(newScale) || newScale == 0) {
        newScale = 1.394124f;
        offset = 0.034143f + caribHeight;
    }
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
