#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Wire.h>
#include "encoder.h"
#include "batt.h"
#include "sleep.h"
#include "display.h"
#include <Fonts/FreeSans18pt7b.h>
#include <Fonts/FreeSans9pt7b.h>
#include <Fonts/FreeMonoBoldOblique9pt7b.h>
#include <EEPROM.h>

// IO settings //
#define GreenLed 16 //powerLed
#define TFT_CS  7
#define TFT_DC  10
#define TFT_SDA 4
#define TFT_SCL 6
#define TFT_RST 9
#define TFT_WIDTH 160
#define TFT_HEIGHT 80
#define TFT_POWER_PIN 13
const int ButtonPin = 12;

// AS5600 register //
#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C
#define AS5600_ZMC0 0x00
#define AS5600_ZPOS 0x01
#define AS5600_MPOS 0x03
#define AS5600_MANG 0x05
uint16_t zeroPosition = 0x0000;
uint16_t maxAngle = 0x0400; //maxAngle 90deg
#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C
#define ADDR_HEIGHT_OFFSET 300

// height variable //

// power LED variable //
bool GreenLedState = false;

// prototype //
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
float readEncoderAngle();
void setZeroPosition(uint16_t zeroPosition);
void setMaxAngle(uint16_t maxAngle);


// carib //
float calibJigLow = 5.0f;
float calibJigHeigh = 30.3f;
float newScale = 1.0f;
bool calibrationDone = false;

void setup() {

    // I2C settings
    Wire.begin();
    Wire.setClock(400000);


    pinMode(ButtonPin,INPUT_PULLUP);

    delay(100);
    if(digitalRead(ButtonPin)==LOW){
    pinMode(TFT_POWER_PIN,OUTPUT);
    digitalWrite(TFT_POWER_PIN,HIGH);    
    tft.initR(INITR_GREENTAB); //for greentab setting
    tft.invertDisplay(true);
    tft.fillScreen(ST7735_BLACK);
    tft.setRotation(1);
    tft.setTextSize(1);
      
    calibrationMode();         
            
    }

    // EEPROM settings
    EEPROM.get(300,heightOffset);
    if(isnan(heightOffset)) heightOffset = 0.0f;
    isReferenceSet = true;

    // AS5600 MaxAngle settings


    // I2C settings
    Wire.begin();
    Wire.setClock(400000);

    pinMode(TFT_POWER_PIN,OUTPUT);
    digitalWrite(TFT_POWER_PIN,HIGH);

    
    tft.initR(INITR_GREENTAB); //for greentab setting
    tft.invertDisplay(true);
    tft.fillScreen(ST7735_BLACK);
    tft.setRotation(1);
    tft.setTextSize(1);
  
    // startup message
    tft.setTextColor(0xf7be);
    tft.setFont(&FreeMonoBoldOblique9pt7b);
    tft.setCursor(40,70);
    tft.println("KO");
    tft.setCursor(65,70);
    tft.println("PROPO");

    delay(3000);
    tft.fillRect(33,60,95,30,ST7735_BLACK);    


}

void(*resetFunc)(void) = 0;
float medianOfArray(float *buf, int n) {
    float tmp[n];
    for (int i = 0; i < n; i++) tmp[i] = buf[i];
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (tmp[i] > tmp[j]) {
                float t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    if (n % 2 == 1) return tmp[n / 2];
    else return (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5f;
}

float trimmedMean(float *buf, int n, float trim_frac) {
    int cut = (int)(n * trim_frac);
    if (cut * 2 >= n) return medianOfArray(buf, n); // fallback
    float tmp[n];
    for (int i = 0; i < n; i++) tmp[i] = buf[i];
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (tmp[i] > tmp[j]) {
                float t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    float sum = 0;
    int cnt = 0;
    for (int i = cut; i < n - cut; i++) {
        sum += tmp[i];
        cnt++;
    }
    return sum / cnt;
}


float getStableAngleRobust(int preDiscard = 10, int sampleN = 300, float trim_frac = 0.10f, int wait_ms = 10) {
    float lastAngle = readEncoderAngle();
    int stableCount = 0;
    while (true) {
        float current = readEncoderAngle();
        float diff = fabs(current - lastAngle);
        if (diff < 0.005) {
            stableCount++;
        } else {
            stableCount = 0;
        }
        if (stableCount > 30) break;
        lastAngle = current;
        delay(10);
    }

    tft.fillRect(0, 60, 160, 20, ST7735_BLACK);
    tft.setCursor(10, 70);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Stable... Averaging");

    // discard
    for (int i = 0; i < preDiscard; i++) {
        readEncoderAngle();
        delay(wait_ms);
    }

    // sample collect
    float samples[sampleN];
    for (int i = 0; i < sampleN; i++) {
        samples[i] = readEncoderAngle();
        delay(wait_ms);
    }

    // robust average
    float median = medianOfArray(samples, sampleN);
    float tmean = trimmedMean(samples, sampleN, trim_frac);

    tft.fillRect(0, 60, 160, 20, ST7735_BLACK);
    tft.setCursor(10, 70);
    tft.print("Done");

    return tmean; 
}


/*
float getStableAngle(float threshold = 0.005, int stableCount = 30, int avgCount = 1000){
    float lastAngle = readEncoderAngle();
    int consecutiveStable = 0;
    unsigned long lastDisplayTime = 0;
    int dotCount = 0;

    tft.fillRect(0,60,160,20,ST7735_BLACK);
    tft.setCursor(10,70);
    tft.setTextColor(ST7735_WHITE);
    tft.setTextSize(1);
    tft.print("searching");

    while(true){
        float current = readEncoderAngle();
        float diff = fabs(current - lastAngle);

        if(diff < threshold){
            consecutiveStable++;
        }else{
            consecutiveStable =0;
        }
        lastAngle = current;

        if(millis() - lastDisplayTime > 500){
            lastDisplayTime = millis();
            dotCount = (dotCount + 1) % 4;
            tft.fillRect(80,60,60,20,ST7735_BLACK);
            tft.setCursor(80,70);
            for(int i=0; i < dotCount; i++) tft.print(".");
        }
        
        if(consecutiveStable >= stableCount){

            tft.fillRect(0,60,160,20,ST7735_BLACK);
            tft.setCursor(10,70);
            tft.setTextColor(ST7735_WHITE);
            tft.print("Stable");

            float sum = 0;
            for(int i=0; i<avgCount; i++){
                sum += readEncoderAngle();
                delay(5);
            }
            return sum / avgCount;
        }
        delay(10);
    }
}
    */

void calibrationMode(){

    uint16_t zpos,mpos,mang;

    const int NUM_POINTS = 5;
    const int SAMPLE_COUNT = 30;
    float knownHeights[NUM_POINTS] = {2.5f,5.0f,16.0f,27.0f,38.0f};
    float measuredAngles[NUM_POINTS];

    tft.setTextSize(1);
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(10,40);
    tft.setTextColor(ST7735_WHITE);
    tft.println("CalibrationMode");
    tft.setCursor(10,60);
    tft.println("PressBTN");

    unsigned long pressStart = millis();
    while (digitalRead(ButtonPin) == LOW)
    {
        if(millis() - pressStart > 5000){
            break;
        }    
    }
    while (digitalRead(ButtonPin) == HIGH);

    const float rangeDeg = 45.0f;
    uint16_t rangeCount = (uint16_t)(4096.0f * rangeDeg / 360.0f);
    mang = rangeCount;

    tft.setTextSize(1);
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(10,40);
    tft.setTextColor(ST7735_WHITE);
    tft.println("set ZERO Position");
    tft.setCursor(10,60);
    tft.println("PressBTN");

    while(digitalRead(ButtonPin)==LOW);
    while(digitalRead(ButtonPin)==HIGH);

    float rawAngleDeg = readEncoderAngle();
    zpos = (uint16_t)(rawAngleDeg/ 360.0f * 4096.0f) & 0x0FFF;
    mpos = (zpos + rangeCount) & 0x0FFF;

    writeRegister16(AS5600_ZPOS,zpos);
    writeRegister16(AS5600_MPOS,mpos);
    writeRegister16(AS5600_MANG,mang);
    saveAS5600RegistersToEEPROM(zpos,mpos,mang);

    delay(1000);


    for(int i=0; i<NUM_POINTS; i++){

        float sumAngle = 0.0f;
        float minAngle = 999.0f;
        float maxAngle = -999.0f;
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10,40);
        tft.print("set ");
        tft.print(knownHeights[i],1);
        tft.println("mm");
    
    while(digitalRead(ButtonPin)==LOW);
    while(digitalRead(ButtonPin)==HIGH);

    for(int s=0; s<SAMPLE_COUNT; s++){
        float a = readEncoderAngle();
        sumAngle += a;
        if(a<minAngle) minAngle = a;
        if(a>maxAngle) maxAngle = a;
        delay(10);
    }
    float stableAngle = getStableAngleRobust(10,400,0.10f,12);
    measuredAngles[i] = stableAngle;


    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(10,30);
    tft.print("Height: ");
    tft.print(knownHeights[i],3);
    tft.println(" mm");

    tft.setCursor(10,50);
    tft.print("AvgAngle: ");
    tft.print(stableAngle,5);
    tft.println(" deg");

    tft.setCursor(10,65);
    tft.print("MinAngle: ");
    tft.print(minAngle,5);
    tft.println(" deg");

    tft.setCursor(10,80);
    tft.print("MaxAngle: ");
    tft.print(maxAngle,5);
    tft.println(" deg");

    while(digitalRead(ButtonPin)==LOW);
    while(digitalRead(ButtonPin)==HIGH);

    }

//All Result show 
tft.fillScreen(ST7735_BLACK);
tft.setCursor(10,30);
tft.print("All data collected");
tft.setCursor(10,50);
tft.print("Press to show");

while(digitalRead(ButtonPin)==LOW);
while(digitalRead(ButtonPin)==HIGH);

//Complete show
tft.fillScreen(ST7735_BLACK);
tft.setCursor(10,30);
tft.print("HeightAngle");
for(int i=0; i<NUM_POINTS; i++){
    tft.setCursor(10,40+i*10);
    tft.print(knownHeights[i],1);
    tft.print("mm ");
    tft.print(measuredAngles[i],5);
}

//Complete confirm
tft.setCursor(10,90);
tft.println("press to save");

while(digitalRead(ButtonPin)==LOW);
while(digitalRead(ButtonPin)==HIGH);    

for(int i=0; i<NUM_POINTS; i++){
    EEPROM.put(100 + i * sizeof(float), measuredAngles[i]);
    EEPROM.put(200 + i * sizeof(float), knownHeights[i]);
}

heightOffset = 0.0f;
EEPROM.put(300,heightOffset);



    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(10,80);
    tft.println("Calibration saved");


    delay(3000);
    

    resetFunc();    
    
}

void loop() {

    static unsigned long buttonPuressStart = 0;
    static bool buttonPressed = false;

    if(digitalRead(ButtonPin) == LOW){
        if(!buttonPressed){
            buttonPressed = true;
            float currentAngle = readEncoderAngle();
            float measuredHeight = interpolateHeight(currentAngle);

            const float referenceHeight = 5.0f;

            heightOffset = measuredHeight - referenceHeight;
            EEPROM.put(300,heightOffset);

        }       
    }else{
        if(buttonPressed){
            
            }
            buttonPressed = false;
    }    
    


    // battery survey
    updateBatteryStatus(tft);


    //height calucurate   
     //carib calucrate 
     if(digitalRead(ButtonPin) == LOW){
        setInitialAngleFromSensor();
        saveCurrentZeroPositionToEEPROM();
      }
    
      float height = updateHeight(); 
    updateHeightDisplay(tft,height,previousHeight);

    // sleep control
    updateSleepStatus(height, TFT_POWER_PIN);
    handleSleepLED(GreenLed);        

    delay(50);
}


