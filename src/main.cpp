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
const int ButtonPin = 12;

#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C
#define AS5600_ZMC0 0x00
#define AS5600_ZPOS 0x01
#define AS5600_MPOS 0x03
#define AS5600_MANG 0x05
uint16_t zeroPosition = 0x0000;
uint16_t maxAngle = 0x0400;


#define AS5600_AS5601_DEV_ADDRESS 0x36
#define AS5600_AS5601_REG_RAW_ANGLE 0x0C

bool isReferenceSet = false;
float currentAngle = 0;
float previousHeight = -1;
float initialAngle = 0;
float relativeAngle = 0;
float height = 0.0;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

float readEncoderAngle();
void setZeroPosition(uint16_t zeroPosition);
void setMaxAngle(uint16_t maxAngle);




void setup() {
    Wire.begin();
    Wire.setClock(400000);

    pinMode(ButtonPin,INPUT_PULLUP);
    

    
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

    if(digitalRead(ButtonPin) == LOW){
        initialAngle = readEncoderAngle();
        isReferenceSet = true;
    }

    if(isReferenceSet == true){
        relativeAngle = currentAngle - initialAngle;

          if(relativeAngle > 1){
    height = -0.951 * ((currentAngle - 360.0) - initialAngle)+0.01;
    }else{
    height = -0.951 * relativeAngle + 0.01;
    }

    // height = height + 3.0;
    if(height < 0){
    height = 0.0;
    }
    if(height > 50.0){
    height = 0.0;
    }

     height = height + 3.18;
       

    }

    if(height != previousHeight){
        char heightText[10],previousText[10];
        dtostrf(height,4,1,heightText);
        dtostrf(previousHeight,4,1,previousText);

        bool isSingleDigit = (heightText[1] == '.');
      
        if(heightText[0] != previousText[0]){
            tft.fillRect(31,51,40,45,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
            if(isSingleDigit){
            tft.setCursor(60,85);
            }else{
            tft.setCursor(46,85);
            }
            
            tft.print(heightText[0]);
            // tft.print(currentText[1]);
                         
                
        } 
        if (heightText[1] != previousText[1]){
            tft.fillRect(67,51,40,45,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
            tft.setCursor(68,85);
            tft.print(heightText[1]);            
        }

        tft.setFont(&FreeSans18pt7b);
            tft.setCursor(85,85);
        tft.print(heightText[2]);   
    
      if(heightText[3] != previousText[3]){
            tft.fillRect(94,51,23,45,ST7735_BLACK);
            tft.setFont(&FreeSans18pt7b);
                tft.setCursor(95,85);
            tft.print(heightText[3]);            

      }
        previousHeight = height;
        
    }
    
    
  delay(50);
}

