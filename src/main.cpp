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

// power LED variable //
bool GreenLedState = false;

// prototype //
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);
float readEncoderAngle();
void setZeroPosition(uint16_t zeroPosition);
void setMaxAngle(uint16_t maxAngle);


void setup() {

    initEncorder(); //encorder max angle define

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

    // battery survey
    updateBatteryStatus(tft);
       
     //carib calucrate 
     if(digitalRead(ButtonPin) == LOW){
        setInitialAngleFromSensor();
        saveCurrentZeroPositionToEEPROM();
      }
    
      float height = updateHeight();
 

    // Dipslay print    
    updateHeightDisplay(tft,height,previousHeight);

    // sleep control
    updateSleepStatus(height, TFT_POWER_PIN);
    handleSleepLED(GreenLed);        

    delay(50);
}


