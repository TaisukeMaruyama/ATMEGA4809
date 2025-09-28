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
    restoreCalibrationFromEEPROM();
    restoreZeroPositionFromEEPROM();
    EEPROM.get(2,initialAngle);
    isReferenceSet = true;

    // AS5600 MaxAngle settings
    setMaxAngle(maxAngle);


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

void calibrationMode(){

    const int NUM_POINTS = 5;
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

    for(int i=0; i<NUM_POINTS; i++){
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10,40);
        tft.print("set ");
        tft.print(knownHeights[i],1);
        tft.println("mm");
    

    if(i >= 2){
        float sumX=0,sumY=0,sumXX=0,sumYY=0,sumXY=0;
        for(int j=0; j<i;j++){
            sumX += measuredAngles[j];
            sumY += knownHeights[j];
            sumXX += measuredAngles[j]*measuredAngles[j];
            sumYY += knownHeights[j]*knownHeights[j];
            sumXY += measuredAngles[j]*knownHeights[j];
        }
    
    float n = i;
    float r_num = n * sumXY - sumX*sumY;
    float r_den = sqrt((n*sumXX - sumX*sumX)*(n*sumYY - sumY*sumY));
    float r = (fabs(r_den)<1e-6)? 0.0 : r_num/r_den;

    tft.setCursor(10,60);
    tft.print("r= ");
    tft.println(r,3);

    }

    while (digitalRead(ButtonPin) == LOW);
    while (digitalRead(ButtonPin) == HIGH);
    
    measuredAngles[i] = readEncoderAngle();
}

float sumX =0,sumY = 0,sumXX = 0,sumXY = 0;
for(int i=0; i<NUM_POINTS; i++){
    sumX += measuredAngles[i];
    sumY += knownHeights[i];
    sumXY += measuredAngles[i] * knownHeights[i];
    sumXX += measuredAngles[i] * measuredAngles[i];
}
float denom = NUM_POINTS * sumXX - sumX * sumX;
if(fabs(denom) < 1e-6){
    newScale = 1.0f;    
}else{
    newScale = (NUM_POINTS * sumXY - sumX * sumY) / denom;    
}

float newOffset = 0;

saveCalibrationToEEPROM(newScale,newOffset);
    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(10,40);
    tft.println("complete");
    tft.setCursor(10,55);
    tft.print("Scale :");
    tft.println(newScale,5);

    delay(10000);

    resetFunc();    
    
}

void loop() {

    static unsigned long buttonPuressStart = 0;
    static bool buttonPressed = false;

    if(digitalRead(ButtonPin) == LOW){
        if(!buttonPressed){
            buttonPressed = true;
        }       
    }else{
        if(buttonPressed){
            
                setInitialAngleFromSensor();
                saveCurrentZeroPositionToEEPROM();
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


