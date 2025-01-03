#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7735.h>

#define TFT_CS  7
#define TFT_DC  10
#define TFT_SDA 4
#define TFT_SCL 6
#define TFT_RST 9
#define TFT_WIDTH 80
#define TFT_HEIGHT 160

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);


void setup() {
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


  
}

