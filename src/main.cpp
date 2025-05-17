#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Wire.h>
#include "encoder.h"
#include "batt.h"
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
bool isReferenceSet = false;
static float heightForSleep = 0.0f;
float currentAngle = 0;
float previousHeight = -1;
float initialAngle = 0;
float relativeAngle = 0;
const float proveLength = 80.68612f;
const float minHeight = 1.9f;
const float maxHeight = 50.0f;
float height = 0.0;
float smoothHeight = 0.0;
const float smoothingFactor = 0.7f; //RC Fillter

// power LED variable //
bool GreenLedState = false;
int fadeValue = 0;
int fadeAmount = 10;
bool fadeDirectionUp = true;

// sleep variable //
bool sleepMode = 0;
unsigned long lastInteractionTime = 0;
const unsigned long inactivityThreshold = 60000;
const float sleepValue = 1.0;

// battery variable
int batteryThreshold = 290;

// prototype //
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
float readEncoderAngle();
void setZeroPosition(uint16_t zeroPosition);
void setMaxAngle(uint16_t maxAngle);


void setup() {

    // EEPROM settings
    restoreZeroPositionFromEEPROM();
    EEPROM.get(2,initialAngle);
    isReferenceSet = true;

    // AS5600 MaxAngle settings
    setMaxAngle(maxAngle);


    // I2C settings
    Wire.begin();
    Wire.setClock(400000);

    pinMode(ButtonPin,INPUT_PULLUP);
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

void loop() {

    // battery indicate green or red //
    int BattValue = getBatteryRaw();
    bool batteryGood = (BattValue > batteryThreshold);
    static bool prevBatteryGood =!batteryGood; 

    if(batteryGood != prevBatteryGood){
        uint16_t frameColor = batteryGood ? 0x2d13 : 0xe003;        
        tft.drawRoundRect(30, 30, 100, 70, 8, frameColor); 
        tft.fillRoundRect(30, 30, 100, 23, 8, frameColor); 
        prevBatteryGood = batteryGood;       
    }

    tft.setCursor(35,45);
    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(0xf7be);
    tft.println("RideHeight");

   
    

    //height calucurate   
    currentAngle = readEncoderAngle();

     //carib calucrate 
     if(digitalRead(ButtonPin) == LOW){
        float angle = readEncoderAngle();
        initialAngle = angle;
        saveCurrentZeroPositionToEEPROM();
        EEPROM.put(2,initialAngle);
        isReferenceSet = true;       
      }

    // height calucrate
    if(isReferenceSet == true){
      relativeAngle = currentAngle - initialAngle;
      float rerativeAngleRad = radians(relativeAngle);
      height = proveLength * tan(rerativeAngleRad) + 5.0f;
     // height crop
     
      if(height < minHeight){
        height = minHeight;
      }
      if(height > maxHeight){
        height = maxHeight;
      }
        

      smoothHeight = height; 
    }
    

    // anti flicker    
    if(smoothHeight != previousHeight){
        char heightText[10],previousText[10];
        dtostrf(smoothHeight,4,1,heightText);
        dtostrf(previousHeight,4,1,previousText);

        bool isSingleDigit = (heightText[1] == '.');
        
        if(heightText[0] != previousText[0]){
            tft.fillRect(33,56,38,41,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
            if(isSingleDigit){
            tft.setCursor(60,87);
            }else{
            tft.setCursor(46,87);
            }          
            tft.print(heightText[0]);                       
        } 
        
        if (heightText[1] != previousText[1]){
            tft.fillRect(67,56,40,41,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
            tft.setCursor(68,87);
            tft.print(heightText[1]);            
        }
        tft.setFont(&FreeSans18pt7b);
            tft.setCursor(85,87);
        tft.print(heightText[2]);   
    
        if(heightText[3] != previousText[3]){
            tft.fillRect(94,56,23,41,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
                tft.setCursor(95,87);
            tft.print(heightText[3]);
        }
        previousHeight = smoothHeight;
    }

    // sleep control
    if(abs(smoothHeight - heightForSleep) > sleepValue){
        lastInteractionTime = millis();
        heightForSleep = smoothHeight;
    }

    if(millis() - lastInteractionTime > inactivityThreshold){
        digitalWrite(TFT_POWER_PIN,LOW);
        sleepMode = 1;
    } else {
        digitalWrite(TFT_POWER_PIN, HIGH);
        sleepMode = 0;
    }

    unsigned long currentMillis = millis();

    if(sleepMode == 1){
                GreenLedState = false;
                                
                if(fadeDirectionUp){
                    fadeValue += fadeAmount;
                    if(fadeValue >= 180){
                        fadeValue = 180;
                        fadeDirectionUp = false;
                    }
                }else{
                    fadeValue -= fadeAmount;
                    if(fadeValue <= 0){
                        fadeValue = 0;
                        fadeDirectionUp = true;
                    }
                }
                analogWrite(GreenLed,fadeValue);
            }else{
                GreenLedState = true;
                analogWrite(GreenLed , 180);            
            }   
        

    delay(50);
}


