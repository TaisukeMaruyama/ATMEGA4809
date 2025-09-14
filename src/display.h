#ifndef DISPLAY_H
#define DISPLAY_H

#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

void updateHeightDisplay(Adafruit_ST7735 &tft,float height, float &previousHeight);

#endif