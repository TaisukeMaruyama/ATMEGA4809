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

#define GreemLed 14
#define RedLed 15

#define TFT_CS  7
#define TFT_DC  10
#define TFT_SDA 4
#define TFT_SCL 6
#define TFT_RST 9
#define TFT_WIDTH 160
#define TFT_HEIGHT 80
#define TFT_POWER_PIN 13
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
bool sleepMode = 0;
bool GreenLedState = false;
bool RedLedState = false;
float currentAngle = 0;
float previousHeight = -1;
float initialAngle = 0;
float relativeAngle = 0;
float height = 0.0;
unsigned long lastUpdateTime = 0;
unsigned long previousMillis = 0;
int LedOnTime = 20;
int LedOffTime = 3000;



unsigned long lastInteractionTime = 0;
const unsigned long inactivityThreshold = 60000;
const float sleepValue = 0.5;

int batteryThreshold = 340;

Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

float readEncoderAngle();
void setZeroPosition(uint16_t zeroPosition);
void setMaxAngle(uint16_t maxAngle);




void setup() {

    if(EEPROM.read(0) != 0xAA){
        EEPROM.write(0,0xAA);
        EEPROM.write(1,0);
    }    


    Wire.begin();
    Wire.setClock(400000);

    pinMode(ButtonPin,INPUT_PULLUP);
    pinMode(TFT_POWER_PIN,OUTPUT);
    digitalWrite(TFT_POWER_PIN,HIGH);

    pinMode(GreemLed, OUTPUT);
    pinMode(RedLed,OUTPUT);

    
    tft.initR(INITR_GREENTAB);
    tft.invertDisplay(true);
    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(0xf7be);
    tft.setRotation(1);
    tft.setTextSize(1);
    tft.drawRoundRect(30, 30, 100, 70, 8, 0x2d13);
    tft.fillRoundRect(30, 30, 100, 23, 8, 0x2d13);
    tft.setCursor(35,40);
    tft.setFont(&FreeSans9pt7b);
    tft.println("RideHeight");

    tft.setFont(&FreeMonoBoldOblique9pt7b);
    tft.setCursor(40,79);
    tft.println("KO");
    tft.setCursor(65,79);
    tft.println("PROPO");

    delay(5000);

    tft.fillRect(33,60,95,30,ST7735_BLACK);



    

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
    height = -1.346 * ((currentAngle - 360.0) - initialAngle);
    }else{
    height = -1.346 * relativeAngle;
    }

    // height = height + 3.0;
    if(height < 0){
    height = 0.0;
    }
    if(height > 50.0){
    height = 0.0;
    }

     height = height + 5.00f;      

    }

    if(abs(height - previousHeight) > sleepValue){
        lastInteractionTime = millis();
    }

    if(millis() - lastInteractionTime > inactivityThreshold){
        digitalWrite(TFT_POWER_PIN,LOW);
        sleepMode = 1;
    } else {
        digitalWrite(TFT_POWER_PIN, HIGH);
        sleepMode = 0;
    }

    if(height != previousHeight){
        char heightText[10],previousText[10];
        dtostrf(height,4,1,heightText);
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
            // tft.print(currentText[1]);
                         
                
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
        previousHeight = height;
        lastUpdateTime = millis();
        digitalWrite(TFT_POWER_PIN,HIGH);
        
    }

    unsigned long currentMillis = millis();
    int BattValue = getBatteryRaw();


    if(sleepMode == 0){
        if(BattValue > batteryThreshold){
            digitalWrite(GreemLed,HIGH);
            digitalWrite(RedLed,LOW);
        }else{
            digitalWrite(GreemLed,LOW);
            digitalWrite(RedLed,HIGH);
        }
    }else{
        if(BattValue > batteryThreshold){
        if(GreenLedState){
            if(currentMillis - previousMillis >= LedOnTime){
                GreenLedState = false;
                digitalWrite(GreemLed , LOW);
                previousMillis = currentMillis;
            }
        }else{
            if(currentMillis - previousMillis >= LedOffTime){
                GreenLedState = true;
                digitalWrite(GreemLed , HIGH);
                previousMillis = currentMillis;
            
            }
        }
    }else{
        if(RedLedState){
            if(currentMillis - previousMillis >= LedOnTime){
                RedLedState = false;
                digitalWrite(RedLed , LOW);
                previousMillis = currentMillis;
            }
        }else{
            if(currentMillis - previousMillis >= LedOffTime){
                RedLedState = true;
                digitalWrite(RedLed , HIGH);
                previousMillis = currentMillis;
            
            }
        }
    }

    }


  delay(50);
}

