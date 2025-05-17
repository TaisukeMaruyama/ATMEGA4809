#ifndef BATT_H
#define BATT_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

int getBatteryRaw();
void updateBatteryStatus(Adafruit_ST7735 & tft);

#endif