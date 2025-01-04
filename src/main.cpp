#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Wire.h>
#include "encoder.h"
#include <Fonts/FreeSans18pt7b.h>

#define TFT_CS  7
#define TFT_DC  10
#define TFT_SDA 4
#define TFT_SCL 6
#define TFT_RST 9
#define TFT_WIDTH 160
#define TFT_HEIGHT 80

#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C

float currentAngle = 0;
float previousAngle = -1;


Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

float readEncoderAngle();



void setup() {
    Wire.begin();
    Wire.setClock(400000);

    
    tft.initR(INITR_GREENTAB);
    tft.invertDisplay(true);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(ST7735_WHITE);
    tft.setRotation(3);
    tft.setTextSize(1);
    tft.drawRoundRect(30, 30, 100, 70, 5, ST7735_BLUE);
    tft.fillRoundRect(30, 30, 100, 20, 5, ST7735_BLUE);
    tft.setCursor(45,38);
    tft.println("RIDE HEIGHT");


    

}

void drawRightString(const char *text, int x , int y){
  int16_t x1,y1;
  uint16_t w,h;
  tft.getTextBounds(text,x,y,&x1,&y1,&w,&h);
  tft.setCursor(x-w,y);
  tft.print(text);
}


void loop() {

    currentAngle = readEncoderAngle();

    if(currentAngle != previousAngle){
        char angleText[10];
        dtostrf(currentAngle,4,1,angleText);
        tft.fillRect(31,51,93,45,ST7735_BLACK);
        tft.setFont(&FreeSans18pt7b);
        tft.setCursor(50,85);
        tft.print(angleText);

        previousAngle = currentAngle;
        
    }

    
/*
    tft.fillRect(31,51,93,45,ST7735_BLACK);
    tft.setFont(&FreeSans18pt7b);
    tft.setCursor(50,80);
    tft.println(currentAngle);
*/
    
    // dtostrf(currentAngle,4,1,angleText);
    // drawRightString(angleText,150,85);
        
  delay(50);
}

