#include "encoder.h"
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>

// AS5600 register //
#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C
#define AS5600_ZPOS 0x01
#define AS5600_MANG 0x05
#define AS5600_MPOS 0x03

// EEPROM address map (bytes)
#define ADDR_ZERO_POS        0      // uint16_t (2B)
#define ADDR_INITIAL_ANGLE   2      // float (4B) -- stored at 2 in your main
#define ADDR_NEWSCALE       10      // float
#define ADDR_OFFSET         14      // float
#define ADDR_KNOWN_ANGLES  100      // 5 floats = 20B
#define ADDR_KNOWN_HEIGHTS 200      // 5 floats = 20B
#define ADDR_HEIGHT_OFFSET  300     // float

// parameter //
bool isReferenceSet = false;
float currentAngle = 0;
float previousHeight = NAN;
float initialAngle = 0;
float relativeAngle = 0;
const float proveLength = 80.68612f;
const float caribHeight = 5.0f;
const float minHeight = 1.9f;
const float maxHeight = 50.0f;
float height = 0.0;
float smoothHeight = 0.0;
const float smoothingFactor = 0.7f; // RC Filter

float scaleFactor = 1.394124f;
float offset = 0.0f;
float heightOffset = 0.0f;

const uint16_t defaultMaxAngle = 0x0400;

const int NUM_POINTS = 5;
const float EPS_ANGLE = 0.035f;

void writeRegister16(uint8_t reg, uint16_t value){
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.begin(reg);
    Wire.write((value >> 8)& 0x0F);
    Wire.write(value & 0xFF);
    Wire.endTransmission();
}

uint16_t readRegister16(uint8_t reg) {
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(reg);
    Wire.endTransmission();
    Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
    uint16_t val = (Wire.read() << 8) | Wire.read();
    return val & 0x0FFF;
}

void saveAS5600RegistersToEEPROM(uint16_t zpos, uint16_t mpos, uint16_t mang) {
    EEPROM.put(ADDR_ZERO_POS, zpos);
    EEPROM.put(ADDR_MPOS, mpos);
    EEPROM.put(ADDR_MANG, mang);
}

void restoreAS5600RegistersFromEEPROM() {
    uint16_t zpos, mpos, mang;
    EEPROM.get(ADDR_ZERO_POS, zpos);
    EEPROM.get(ADDR_MPOS, mpos);
    EEPROM.get(ADDR_MANG, mang);
    if (zpos == 0xFFFF) zpos = 0;
    if (mang == 0xFFFF) mang = defaultMaxAngle;

    writeRegister16(AS5600_ZPOS, zpos);
    writeRegister16(AS5600_MPOS, mpos);
    writeRegister16(AS5600_MANG, mang);
}

//--------------------------------------------------
// Core functions
//--------------------------------------------------
float readEncoderAngle() {
    Wire.beginTransmission(AS5600_AS5601_DEV_ADDRESS);
    Wire.write(AS5600_AS5601_REG_RAW_ANGLE);
    Wire.endTransmission(false);
    Wire.requestFrom(AS5600_AS5601_DEV_ADDRESS, 2);
    uint8_t highByte = Wire.read();
    uint8_t lowByte = Wire.read();
    uint16_t rawAngle = ((uint16_t)highByte << 8) | lowByte;
    return (rawAngle * 360.0f / 4096.0f);
}

void saveCurrentZeroPositionToEEPROM() {
    uint16_t raw = (uint16_t)(readEncoderAngle() / 360.0f * 4096.0f);
    writeRegister16(AS5600_ZPOS, raw);
    EEPROM.put(ADDR_ZERO_POS, raw);
}

void setInitialAngleFromSensor() {
    initialAngle = readEncoderAngle();
    EEPROM.put(ADDR_INITIAL_ANGLE, initialAngle);
    isReferenceSet = true;
}

void initEncorder() {
    Wire.begin();
    Wire.setClock(400000);
    restoreAS5600RegistersFromEEPROM();
    EEPROM.get(ADDR_INITIAL_ANGLE, initialAngle);
    isReferenceSet = true;
}

// --- helper: simple pairwise sort by angle (bubble - fine for N=5) ---
void sortPairs(float angles[], float heights[], int N){
    for(int i=0;i<N-1;i++){
        for(int j=i+1;j<N;j++){
            if(angles[i] > angles[j]){
                float ta = angles[i]; angles[i]=angles[j]; angles[j]=ta;
                float th = heights[i]; heights[i]=heights[j]; heights[j]=th;
            }
        }
    }
}

// --- helper: MAD-based robust scale estimator ---
float mad_scale(float res[], int N){
    float absd[NUM_POINTS];
    for(int i=0;i<N;i++) absd[i] = fabs(res[i]);
    // sort absd
    for(int i=0;i<N-1;i++) for(int j=i+1;j<N;j++) if(absd[i] > absd[j]){ float t=absd[i]; absd[i]=absd[j]; absd[j]=t; }
    float med = absd[N/2];
    return 1.4826f * med; // convert MAD -> approx sigma
}

// --- robust calibration routine — modifies knownHeight[] in-place ---
// NOTE: This is intended to be called during calibration, NOT during normal measurement.
//       If you want to save adjusted heights, call writeHeightsToEEPROM() afterwards manually.
void robustCalibratePoints(float knownAngles[], float knownHeight[], int N=NUM_POINTS){
    if(N < 2) return;
    sortPairs(knownAngles, knownHeight, N);

    // initial ordinary least squares for a,b
    float sumA=0,sumH=0,sumA2=0,sumAH=0;
    for(int i=0;i<N;i++){
        sumA += knownAngles[i];
        sumH += knownHeight[i];
        sumA2 += knownAngles[i]*knownAngles[i];
        sumAH += knownAngles[i]*knownHeight[i];
    }
    float denom = N*sumA2 - sumA*sumA;
    if(fabs(denom) < 1e-6f) return; // degenerate
    float a = (N*sumAH - sumA*sumH) / denom;
    float b = (sumH - a*sumA) / N;

    // IRLS with Huber weights (small N so a few iterations)
    const int MAX_IT = 4;
    const float HUBER_C = 1.2f; // tuning constant; adjust in calibration routine if needed
    for(int it=0; it<MAX_IT; it++){
        float res[NUM_POINTS];
        for(int i=0;i<N;i++) res[i] = knownHeight[i] - (a*knownAngles[i] + b);
        float s = mad_scale(res, N);
        if(s < 1e-6f) s = 1e-6f;

        float wsum=0, wsumA=0, wsumH=0, wsumA2=0, wsumAH=0;
        for(int i=0;i<N;i++){
            float r = res[i];
            float absr = fabs(r);
            float thresh = HUBER_C * s;
            float wi = 1.0f;
            if(absr > thresh) wi = thresh / absr; // Huber-type downweight

            wsum += wi;
            wsumA += wi * knownAngles[i];
            wsumH += wi * knownHeight[i];
            wsumA2 += wi * knownAngles[i] * knownAngles[i];
            wsumAH += wi * knownAngles[i] * knownHeight[i];
        }
        float denom_w = wsum * wsumA2 - wsumA * wsumA;
        if(fabs(denom_w) < 1e-6f) break;
        float a_new = (wsum * wsumAH - wsumA * wsumH) / denom_w;
        float b_new = (wsumH - a_new * wsumA) / wsum;
        if(fabs(a_new - a) < 1e-8f && fabs(b_new - b) < 1e-6f){ a = a_new; b = b_new; break; }
        a = a_new; b = b_new;
    }

    // compute final residuals and robust scale
    float finalRes[NUM_POINTS];
    for(int i=0;i<N;i++) finalRes[i] = knownHeight[i] - (a*knownAngles[i] + b);
    float sigma = mad_scale(finalRes, N);
    if(sigma < 1e-6f) sigma = 1e-6f;

    // shrinkage: mix original height and expected using residual-based weight
    const float ALPHA = 1.5f; 
    for(int i=0;i<N;i++){
        float expected = a*knownAngles[i] + b;
        float r = finalRes[i];
        float ratio = fabs(r) / (ALPHA * sigma);
        float w = 1.0f / (1.0f + ratio*ratio); // 0..1, small residual -> w~1
        float newH = w * knownHeight[i] + (1.0f - w) * expected;
        knownHeight[i] = newH;
    }
}

// --- read 5 points from EEPROM into arrays ---
void readCalibrationFromEEPROM(float angles[], float heights[], int N=NUM_POINTS){
    for(int i=0;i<N;i++){
        float a; EEPROM.get(ADDR_KNOWN_ANGLES + i*sizeof(float), a); angles[i] = a;
        float h; EEPROM.get(ADDR_KNOWN_HEIGHTS + i*sizeof(float), h); heights[i] = h;
    }
}

// --- write heights back to EEPROM (angles kept unchanged) ---
void writeHeightsToEEPROM(float heights[], int N=NUM_POINTS){
    for(int i=0;i<N;i++){
        EEPROM.put(ADDR_KNOWN_HEIGHTS + i*sizeof(float), heights[i]);
}
}

// --- replacement interpolateHeight ---
// Behavior: reads calibration points, performs piecewise linear interpolation, returns height
// IMPORTANT: This function does NOT modify EEPROM or stored calibration points.
float interpolateHeight(float angle){
    float knownAngles[NUM_POINTS];
    float knownHeight[NUM_POINTS];

    // read from EEPROM (read-only)
    readCalibrationFromEEPROM(knownAngles, knownHeight, NUM_POINTS);

    // sort pairs (so interpolation assumes ascending angles)
    sortPairs(knownAngles, knownHeight, NUM_POINTS);


    // edge handling and piecewise linear interpolation (same as original)
    if(angle <= knownAngles[0]){
        float denom = (knownAngles[1] - knownAngles[0]);
        if(fabs(denom) < 1e-6f) return knownHeight[0];
        float t = (angle - knownAngles[0]) / denom;
        return knownHeight[0] + t * (knownHeight[1] - knownHeight[0]);
    }
    if(angle >= knownAngles[NUM_POINTS -1]){
        float denom = (knownAngles[NUM_POINTS -1] - knownAngles[NUM_POINTS -2]);
        if(fabs(denom) < 1e-6f) return knownHeight[NUM_POINTS -1];
        float t = (angle - knownAngles[NUM_POINTS -2]) / denom;
        return knownHeight[NUM_POINTS -2] + t * (knownHeight[NUM_POINTS -1] - knownHeight[NUM_POINTS -2]);
    }

    for(int i=0;i<NUM_POINTS-1;i++){
        if(angle >= knownAngles[i] && angle <= knownAngles[i+1]){
            float denom = (knownAngles[i+1] - knownAngles[i]);
            if(fabs(denom) < 1e-6f) return knownHeight[i];
            float t = (angle - knownAngles[i]) / denom;
            return knownHeight[i] + t * (knownHeight[i+1] - knownHeight[i]);
        }
    }

    // fallback
    return 0.0f;
}


float updateHeight(){
    if(!isReferenceSet) return 0.0f;
    currentAngle = readEncoderAngle();
    float relativeAngle = currentAngle - initialAngle;
    float rawHeight =  interpolateHeight(currentAngle) - heightOffset;

    if(height < minHeight){
        rawHeight = minHeight;
    }
    if(height > maxHeight){
        rawHeight = maxHeight;
    }     

    height = rawHeight;
    return height;
}
