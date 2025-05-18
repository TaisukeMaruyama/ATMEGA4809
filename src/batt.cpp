#include "batt.h"
#include <Arduino.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>
#include <Fonts/FreeSans9pt7b.h>

#define BattPin A11
int batteryThreshold = 290;

int getBatteryRaw(){
    return analogRead(BattPin);
}

void updateBatteryStatus(Adafruit_ST7735 &tft){
    int BattValue = getBatteryRaw();
    bool batteryGood = (BattValue > batteryThreshold);
    static bool prevBatteryGood =!batteryGood; 

    if(batteryGood != prevBatteryGood){
        uint16_t frameColor = batteryGood ? 0x2d13 : 0xe003;        
        tft.drawRoundRect(30, 30, 100, 70, 8, frameColor); 
        tft.fillRoundRect(30, 30, 100, 23, 8, frameColor); 
        prevBatteryGood = batteryGood;       
    }

    tft.setCursor(35,47);
    tft.setFont(&FreeSans9pt7b);
    tft.setTextColor(0xf7be);
    tft.println("RideHeight");


}
