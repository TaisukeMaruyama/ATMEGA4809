#include "encoder.h"
#include <EEPROM.h>
#include <Wire.h>
#include <math.h>

// AS5600 register //
#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C
#define AS5600_ZMC0 0x00
#define AS5600_ZPOS 0x01
#define AS5600_MPOS 0x03
#define AS5600_MANG 0x05

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
const float smoothingFactor = 0.7f; //RC Fillter

float scaleFactor = 1.394124f;
float offset = 0.0f;
float heightOffset = 0.0f;

void restoreCalibrationFromEEPROM(){
    EEPROM.get(10,newScale);
    EEPROM.get(14,offset);
    if(isnan(newScale) || newScale == 0){
        newScale = 1.394124f;
        offset = 0.034143f + caribHeight;
    }
}

void saveCalibrationToEEPROM(float scale,float newOffset){
    newScale = scale;
    offset = newOffset;
    EEPROM.put(10,newScale);
    EEPROM.put(14,newOffset);
}

const uint16_t defaultMaxAngle = 0x0400;

float interpolateHeight(float angle){
    const int NUM_POINTS = 5;
    float knownAngles[NUM_POINTS];
    float knownHeight[NUM_POINTS];

    for(int i=0; i<NUM_POINTS; i++){
        EEPROM.get(100+i*sizeof(float),knownAngles[i]);
        EEPROM.get(200+i*sizeof(float),knownHeight[i]);
    }

    for(int i=0; i<NUM_POINTS-1; i++){
        for(int j=i+1; j<NUM_POINTS; j++){
            if(knownAngles[i] > knownAngles[j]){
                float tmpA = knownAngles[i];
                knownAngles[i]=knownAngles[j];
                knownAngles[j]=tmpA;
                float tmpH = knownHeight[i];
                knownHeight[i] = knownHeight[j];
                knownHeight[j] = tmpH;
            }
        }
    }

    
    if(angle <= knownAngles[0]){
        float t = (angle - knownAngles[0])/ (knownAngles[1]-knownAngles[0]);
        return knownHeight[0] + t * (knownHeight[1] - knownHeight[0]);
    }
    if(angle >= knownAngles[NUM_POINTS -1]){
        float t = (angle - knownAngles[NUM_POINTS -2])/(knownAngles[NUM_POINTS - 1] - knownAngles[NUM_POINTS - 2]);
        return knownHeight[NUM_POINTS -2]+t*(knownHeight[NUM_POINTS-1]-knownHeight[NUM_POINTS -2]);
    } 
   
    for(int i=0; i<NUM_POINTS-1; i++){
        if(angle >= knownAngles[i] && angle <= knownAngles[i+1]){
            float t = (angle - knownAngles[i]) / (knownAngles[i+1] - knownAngles[i]);
            return knownHeight[i] + t * (knownHeight[i+1] - knownHeight[i]);
        }
    }
    return 0.0f;
}

void initEncorder(){
    Wire.begin();
    Wire.setClock(400000);
    restoreZeroPositionFromEEPROM();

    EEPROM.get(2,initialAngle);
    isReferenceSet = true;

    setMaxAngle(defaultMaxAngle);
}

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

void setInitialAngleFromSensor(){
    initialAngle = readEncoderAngle();
    EEPROM.put(2,initialAngle);
    isReferenceSet = true;  
}


float updateHeight(){
    if(!isReferenceSet) return 0.0f;
    currentAngle = readEncoderAngle();
    float relativeAngle = currentAngle - initialAngle;
    // float relativeRad = radians(relativeAngle);
    float rawHeight =  interpolateHeight(currentAngle);
    height = rawHeight - heightOffset;
    
    /*
    if(height < minHeight){
        height = minHeight;
    }
    if(height > maxHeight){
        height = maxHeight;
    }     
*/
    return height;

}

