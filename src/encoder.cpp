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
    Wire.endTransmission(false);  
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

// --------------------------------------------------
// over sampling
// --------------------------------------------------
float readEncoderAngle() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    if (Wire.available() < 2) return 0.0f;
    uint16_t val = ((uint16_t)Wire.read() << 8) | Wire.read();
    val &= 0x0FFF;
    return val * (360.0f / 4096.0f);    
}

uint16_t readRawAngleOversampled(uint16_t samples = 64) {
    float sum = 0.0f;
    int succes = 0;
    for (uint16_t i = 0; i < samples; i++) {
        float v = readEncoderAngle();
        if(v <= 0.0f){
            delayMicroseconds(150);
            continue;
        }
        sum += v;
        succes++;
        delayMicroseconds(150);
    }
    if(succes == 0) return 0.0f;
    return sum / succes;
}

float readEncoderAngleOversampled(uint16_t samples = 64) {
    uint32_t sum = 0.0f;
    int succes = 0;
    for (uint16_t i = 0; i < samples; i++) {
        float v = readEncoderAngle();
        if(v <= 0.0f){
            delayMicroseconds(150);
            continue;
        }
        sum += v;
        succes++;
        delayMicroseconds(150);        
    }
    if(succes == 0) return NAN;
        return sum / succes;
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
    if (before >= 3) return false;

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

void computeLinearCalibration(){
    const int NUM_POINTS = 5;
    float knownAngles[NUM_POINTS];
    float knownHeights[NUM_POINTS];

    // EEPROMから読み込み
    for (int i = 0; i < NUM_POINTS; i++) {
        EEPROM.get(ADDR_KNOWN_ANGLES + i * sizeof(float), knownAngles[i]);
        EEPROM.get(ADDR_KNOWN_HEIGHTS + i * sizeof(float), knownHeights[i]);
    }

    // --- NaN/未初期化値があればデフォルトに戻す ---
    bool valid = true;
    for (int i = 0; i < NUM_POINTS; i++) {
        if (!isfinite(knownAngles[i]) || !isfinite(knownHeights[i])) valid = false;
    }
    if (!valid) {
        newScale = 1.0f;
        offset   = 0.0f;
        return;
    }

    // --- 最小二乗法計算 ---
    float sumX = 0, sumY = 0, sumXY = 0, sumXX = 0;
    for (int i = 0; i < NUM_POINTS; i++) {
        sumX  += knownAngles[i];
        sumY  += knownHeights[i];
        sumXY += knownAngles[i] * knownHeights[i];
        sumXX += knownAngles[i] * knownAngles[i];
    }

    float n = (float)NUM_POINTS;
    float denom = (n * sumXX - sumX * sumX);
    if (fabs(denom) < 1e-6f) {
        newScale = 1.0f;
        offset = 0.0f;
        return;
    }

    newScale = (n * sumXY - sumX * sumY) / denom;  // 傾き a
    offset   = (sumY - newScale * sumX) / n;       // 切片 b

    // 結果をEEPROMに保存
    EEPROM.put(ADDR_NEWSCALE, newScale);
    EEPROM.put(ADDR_OFFSET, offset);
}


float updateHeight() {
    currentAngle = readEncoderAngleOversampled(64);
    if(!isfinite(currentAngle))return height;
    height = newScale * currentAngle + offset;
    return height;
}

void saveCurrentZeroPositionToEEPROM() {
    uint16_t rawAngle = readRawAngleOversampled(64);
    setZeroPosition(rawAngle);
    EEPROM.put(ADDR_ZERO_POS, rawAngle);
}

void restoreZeroPositionFromEEPROM() {
    uint16_t rawAngle = 0;
    EEPROM.get(ADDR_ZERO_POS, rawAngle);
    if (rawAngle <= 4095) setZeroPosition(rawAngle);
}

void setInitialAngleFromSensor() {
    initialAngle = readEncoderAngleOversampled(64);
    EEPROM.put(ADDR_INITIAL_ANGLE, initialAngle);
    isReferenceSet = true;
}

void restoreCalibrationFromEEPROM() {
    EEPROM.get(ADDR_NEWSCALE, newScale);
    EEPROM.get(ADDR_OFFSET, offset);
    if (isnan(newScale) || newScale == 0) {
        newScale = 1.394124f;
        offset = 0.034143f;
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


void initEncoder() {
    Wire.beginTransmission(AS5600_ADDR);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
    while (Wire.available()) Wire.read();
}
