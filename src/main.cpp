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

void loop() {

    currentAngle = readEncoderAngle();

    if(currentAngle != previousAngle){
        char currentText[10],previousText[10];
        dtostrf(currentAngle,4,1,currentText);
        dtostrf(previousAngle,4,1,previousText);

        bool isSingleDigit = (currentText[1] == '.');
      
        if(currentText[0] != previousText[0]){
            tft.fillRect(31,51,40,45,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
            if(isSingleDigit){
            tft.setCursor(60,85);
            }else{
            tft.setCursor(46,85);
            }
            
            tft.print(currentText[0]);
            // tft.print(currentText[1]);
                         
                
        } 
        if (currentText[1] != previousText[1]){
            tft.fillRect(67,51,40,45,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
            tft.setCursor(68,85);
            tft.print(currentText[1]);            
        }

        tft.setFont(&FreeSans18pt7b);
            tft.setCursor(85,85);
        tft.print(currentText[2]);   
    
      if(currentText[3] != previousText[3]){
            tft.fillRect(94,51,23,45,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
                tft.setCursor(95,85);
            tft.print(currentText[3]);            

      }
        previousAngle = currentAngle;
        
    }
    
    
  delay(50);
}

