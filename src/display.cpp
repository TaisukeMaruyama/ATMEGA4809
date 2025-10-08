#include "display.h"
#include <Fonts/FreeSans18pt7b.h>

#define DOT_FONT &FreeSans18pt7b
#define DISPLAY_AREA_X 33
#define DISPLAY_AREA_Y 56
#define DISPLAY_AREA_W 100
#define DISPLAY_AREA_H 41

void updateHeightDisplay(Adafruit_ST7735 &tft,float height, float &previousHeight){
        if(height != previousHeight){

        char heightText[10],previousText[10];
        dtostrf(height,4,1,heightText);
        dtostrf(previousHeight,4,1,previousText);

        bool isSingleDigit = (heightText[1] == '.');
        
        if(heightText[0] != previousText[0]){
            tft.fillRect(33,56,33,41,ST7735_BLACK);
            // tft.drawRect(33,56,33,47,ST7735_RED);
            tft.setFont(&FreeSans18pt7b);
            if(isSingleDigit){
            tft.setCursor(60,87);
            }else{
            tft.setCursor(46,87);
            }          
            tft.print(heightText[0]);                       
        } 
        
        if (heightText[1] != previousText[1]){
            tft.fillRect(67,56,21,41,ST7735_BLACK);
            // tft.drawRect(67,56,21,47,ST7735_RED);
            tft.setFont(&FreeSans18pt7b);
            tft.setCursor(68,87);
            tft.print(heightText[1]);            
        }

        if(heightText[2] != previousText[2]){
            tft.fillRect(89,56,6,41,ST7735_BLACK);
            // tft.drawRect(89,56,6,47,ST7735_RED);
            tft.setFont(&FreeSans18pt7b);
            tft.setCursor(85,87);
            tft.print(heightText[2]);
        
        }
    
        if(heightText[3] != previousText[3]){
            tft.fillRect(95,56,32,41,ST7735_BLACK);
            // tft.drawRect(95,56,32,47,ST7735_RED);
            tft.setFont(&FreeSans18pt7b);
            tft.setCursor(95,87);
            tft.print(heightText[3]);
        }

        
        tft.fillRect(87,60,5,30,ST7735_BLACK);
        tft.setFont(DOT_FONT);
        tft.setCursor(87,85);
        tft.print('.');
        

        previousHeight = height;
    }

}

