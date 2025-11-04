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
#define GreenLed 16 // powerLed
#define TFT_CS 7
#define TFT_DC 10
#define TFT_SDA 4
#define TFT_SCL 6
#define TFT_RST 9
#define TFT_WIDTH 160
#define TFT_HEIGHT 80
#define TFT_POWER_PIN 13
const int ButtonPin = 12;

// TFT instance //
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void calibrationMode();

// power LED variable //
bool GreenLedState = false;

// carib //
float calibJigLow = 5.0f;
float calibJigHeigh = 30.3f;
float newScale = 1.0f;

void(*resetFunc)(void) = 0;

//==============================
// Utility: Robust Statistics
//==============================
float medianOfArray(float *buf, int n) {
    float tmp[n];
    for (int i = 0; i < n; i++) tmp[i] = buf[i];
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (tmp[i] > tmp[j]) {
                float t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    if (n % 2 == 1) return tmp[n / 2];
    else return (tmp[n / 2 - 1] + tmp[n / 2]) * 0.5f;
}

float trimmedMean(float *buf, int n, float trim_frac) {
    int cut = (int)(n * trim_frac);
    if (cut * 2 >= n) return medianOfArray(buf, n);
    float tmp[n];
    for (int i = 0; i < n; i++) tmp[i] = buf[i];
    for (int i = 0; i < n - 1; i++) {
        for (int j = i + 1; j < n; j++) {
            if (tmp[i] > tmp[j]) {
                float t = tmp[i];
                tmp[i] = tmp[j];
                tmp[j] = t;
            }
        }
    }
    float sum = 0;
    int cnt = 0;
    for (int i = cut; i < n - cut; i++) {
        sum += tmp[i];
        cnt++;
    }
    return sum / cnt;
}

float getStableAngleRobust(int preDiscard = 10, int sampleN = 300, float trim_frac = 0.10f, int wait_ms = 10) {
    float lastAngle = readEncoderAngle();
    int stableCount = 0;
    while (true) {
        float current = readEncoderAngle();
        float diff = fabs(current - lastAngle);
        if (diff < 0.005) stableCount++;
        else stableCount = 0;
        if (stableCount > 30) break;
        lastAngle = current;
        delay(10);
    }

    tft.fillRect(0, 60, 160, 20, ST7735_BLACK);
    tft.setCursor(10, 70);
    tft.setTextColor(ST7735_WHITE);
    tft.print("Stable... Averaging");

    for (int i = 0; i < preDiscard; i++) {
        readEncoderAngle();
        delay(wait_ms);
    }

    float samples[sampleN];
    for (int i = 0; i < sampleN; i++) {
        samples[i] = readEncoderAngle();
        delay(wait_ms);
    }

    float tmean = trimmedMean(samples, sampleN, trim_frac);

    tft.fillRect(0, 60, 160, 20, ST7735_BLACK);
    tft.setCursor(10, 70);
    tft.print("Done");

    return tmean;
}

//==============================
// setup()
//==============================
void setup() {

    Wire.begin();
    Wire.setClock(400000);
    initEncoder();
    pinMode(ButtonPin, INPUT_PULLUP);
    pinMode(TFT_POWER_PIN, OUTPUT);
    digitalWrite(TFT_POWER_PIN, HIGH);

    delay(300);

    if (digitalRead(ButtonPin) == LOW) {
        pinMode(TFT_POWER_PIN, OUTPUT);
        digitalWrite(TFT_POWER_PIN, HIGH);
        tft.initR(INITR_GREENTAB);
        tft.invertDisplay(true);
        tft.fillScreen(ST7735_BLACK);
        tft.setRotation(1);
        tft.setTextSize(1);
        calibrationMode();
       
    }

    tft.initR(INITR_GREENTAB);
    tft.invertDisplay(true);
    tft.fillScreen(ST7735_BLACK);
    tft.setRotation(1);
    tft.setTextSize(1);
    tft.setTextColor(ST7735_WHITE);

    if (!isBurned()) {
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10, 40);
        tft.println("Burn Mode");
        tft.setCursor(10, 55);
        tft.println("Move to MIN position");
        tft.setCursor(10, 70);
        tft.println("Press BTN");
        uint8_t burnCount = readBurnCount();
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10, 85);
        tft.print("ZMCO: ");
        tft.println(burnCount & 0x03);
    

        // wait for button press
        while (digitalRead(ButtonPin) == HIGH);
        while (digitalRead(ButtonPin) == LOW);

        uint16_t zpos = readRawAngle(); // raw counts 0..4095
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10, 40);
        tft.print("MIN angle: ");
        tft.println(zpos);
        tft.setCursor(10, 55);
        tft.println("Move to MAX position");
        tft.setCursor(10, 70);
        tft.println("Press BTN");

        while (digitalRead(ButtonPin) == HIGH);
        while (digitalRead(ButtonPin) == LOW);

        uint16_t mpos = readRawAngle();
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10, 40);
        tft.print("MAX angle: ");
        tft.println(mpos);
        
        delay(5000);

        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10, 30);
        tft.println("Burning...");
        uint16_t mangOut = 0;
        bool ok = burnAngleAndMANG(zpos, mpos, &mangOut);
        

        if(isBurned()){
            tft.fillScreen(ST7735_BLACK);
            tft.setCursor(10,40);
            tft.println("Burn Scusses");
        }else{
            tft.fillScreen(ST7735_BLACK);
            tft.setCursor(10,40);
            tft.println("Burn Failed");

        }
        delay(5000);

       resetFunc();
    
    }

    restoreCalibrationFromEEPROM();
    restoreZeroPositionFromEEPROM();
    EEPROM.get(300, heightOffset);
    if (isnan(heightOffset)) heightOffset = 0.0f;
    isReferenceSet = true;

    // setMaxAngle(0x0200); // 45 degrees range (runtime override if needed)

    tft.fillScreen(ST7735_BLACK);
    tft.setTextColor(0xf7be);
    tft.setFont(&FreeMonoBoldOblique9pt7b);
    tft.setCursor(40, 70);
    tft.println("KO");
    tft.setCursor(65, 70);
    tft.println("PROPO");

    delay(2000);
    tft.fillRect(33, 60, 95, 30, ST7735_BLACK);
}

//==============================
// Calibration Mode
//==============================


void calibrationMode() {

    const int NUM_POINTS = 5;
    float knownHeights[NUM_POINTS] = {2.5f, 5.0f, 16.0f, 27.0f, 38.0f};
    float measuredAngles[NUM_POINTS];

    tft.fillScreen(ST7735_BLACK);
    tft.setTextSize(1);
    tft.setCursor(10, 30);
    tft.println("CalibrationMode");
    // tft.setCursor(10, 40);
    // tft.println("Press BTN");

    bool burned = isBurned();
    tft.setCursor(10,40);
    if(burned){
        tft.println("Burn Success");
    }else{
        tft.println("Burn False");
    }
/*

    uint16_t zpos = readRegister16(AS5600_ZPOS);
    uint16_t mpos = readRegister16(AS5600_MPOS);
    uint16_t mang = readRegister16(AS5600_MANG);

    tft.setCursor(10,50);
    tft.print("ZPOS: 0x");
    tft.println(zpos,HEX);

    tft.setCursor(10,60);
    tft.print("MPOS: 0x");
    tft.println(mpos,HEX);

    tft.setCursor(10,70);
    tft.print("MANG: 0x");
    tft.println(mang,HEX);

    float rawAngle = readRawAngle()*(360.0/4096.0);
    float scaledAngle = readEncoderAngle();
    tft.setCursor(10,80);
    tft.print("RAW: ");
    tft.print(rawAngle,5);
    tft.println(" deg");
    tft.setCursor(10,90);
    tft.print("ANG: ");
    tft.print(scaledAngle,5);
    tft.println(" deg");

*/
    while (digitalRead(ButtonPin) == LOW);
    while (digitalRead(ButtonPin) == HIGH);

    for (int i = 0; i < NUM_POINTS; i++) {
        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10, 40);
        tft.print("Set ");
        tft.print(knownHeights[i], 1);
        tft.println("mm");
        tft.setCursor(10, 60);
        tft.println("Press BTN");

        while (digitalRead(ButtonPin) == LOW);
        while (digitalRead(ButtonPin) == HIGH);

        float avgAngle = getStableAngleRobust(10, 200, 0.1f, 5);
        measuredAngles[i] = avgAngle;

        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10, 20);
        tft.print("Height: ");
        tft.print(knownHeights[i], 1);
        tft.println("mm");
        tft.setCursor(10, 40);
        tft.print("AvgAngle: ");
        tft.print(avgAngle, 3);
        tft.println(" deg");

        while (digitalRead(ButtonPin) == LOW);
        while (digitalRead(ButtonPin) == HIGH);
    }

    for (int i = 0; i < NUM_POINTS; i++) {
        EEPROM.put(100 + i * sizeof(float), measuredAngles[i]);
        EEPROM.put(200 + i * sizeof(float), knownHeights[i]);
    }

    tft.fillScreen(ST7735_BLACK);
    tft.setCursor(10, 40);
    tft.println("Calibration saved");
    delay(3000);
    resetFunc();
}

//==============================
// loop()
//==============================
void loop() {
    static bool buttonPressed = false;

    if (digitalRead(ButtonPin) == LOW) {
        if (!buttonPressed) {
            buttonPressed = true;
            float currentAngle = readEncoderAngle();
            float measuredHeight = interpolateHeight(currentAngle);
            const float referenceHeight = 5.0f;
            heightOffset = measuredHeight - referenceHeight;
            EEPROM.put(300, heightOffset);
        }
    } else {
        buttonPressed = false;
    }

    updateBatteryStatus(tft);

    if (digitalRead(ButtonPin) == LOW) {
        setInitialAngleFromSensor();
        saveCurrentZeroPositionToEEPROM();
    }

    float height = updateHeight();
    updateHeightDisplay(tft, height, previousHeight);

    updateSleepStatus(height, TFT_POWER_PIN);
    handleSleepLED(GreenLed);

    delay(50);
}
