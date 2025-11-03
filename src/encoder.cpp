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

// float newScale = 1.0f;

float scaleFactor = 1.394124f;
float offset = 0.0f;

// ---- I2C Utility ----
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
    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    int rc = Wire.endTransmission(); // 0 = success
    return (rc == 0);
}

void writeRegister16(uint8_t reg, uint16_t value) {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(reg);
    Wire.write((uint8_t)(value >> 8));
    Wire.write((uint8_t)(value & 0xFF));
    Wire.endTransmission();
}

// ---- Burn / status ----
bool isBurned() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_REG_BURN);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)1);
    if (Wire.available()) {
        uint8_t burnStatus = Wire.read();
        return (burnStatus & 0x80);
    }
    return false;
}

uint16_t readRawAngle() {
    return readRegister16(AS5600_AS5601_REG_RAW_ANGLE) & 0x0FFF;
}

// Burn: write ZPOS, MPOS, MANG then issue BURN_ANGLE
void burnAngleAndMANG(uint16_t zpos, uint16_t mpos) {
    // write ZPOS
    writeRegister16(AS5600_ZPOS, zpos);
    delay(30);

    // write MPOS
    writeRegister16(AS5600_MPOS, mpos);
    delay(30);

    // compute and write MANG
    uint16_t mang;
    if (mpos >= zpos) mang = mpos - zpos;
    else mang = 4096 + mpos - zpos;
    writeRegister16(AS5600_MANG, mang);
    delay(50); // allow device to commit volatile writes

    // --- OTP (burn) with return-code check ---
    Wire.beginTransmission((uint8_t)AS5600_ADDR);
    Wire.write(AS5600_REG_BURN);    // 0xFF
    Wire.write(AS5600_BURN_ANGLE);  // 0x80
    int rc = Wire.endTransmission(); // 0 = success
    if (rc != 0) {
        delay(20);
        Wire.beginTransmission((uint8_t)AS5600_ADDR);
        Wire.write(AS5600_REG_BURN);
        Wire.write(AS5600_BURN_ANGLE);
        rc = Wire.endTransmission();
        if (rc != 0) {
            return;
        }
    }

    delay(100); // OTP処理時間（デバイスによって推奨値を参照）
    uint8_t burn = readBurnCount(); // read back burn count / status

}
// Write ZPOS and MANG without burning (helper used during manual flow)
void writeZPOSandMANG(uint16_t zpos, uint16_t mang) {
    writeRegister16(AS5600_ZPOS, zpos);
    delay(10);
    writeRegister16(AS5600_MANG, mang);
    delay(10);
}

// ---- 通常動作用 ----
void setZeroPosition(uint16_t zeroPosition) {
    writeRegister16(AS5600_ZPOS, zeroPosition);
}

void setMaxAngle(uint16_t maxAngle) {
    writeRegister16(AS5600_MANG, maxAngle);
}

float readEncoderAngle() {
    // always read RAW angle register 0x0C and scale
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