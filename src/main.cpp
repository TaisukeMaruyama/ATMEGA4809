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

float norm = 0.0f;
float avg = 0.0f;
uint32_t range = 0;
uint32_t delta = 0;
uint64_t add = 0;
// TFT instance //
Adafruit_ST7735 tft = Adafruit_ST7735(TFT_CS, TFT_DC, TFT_RST);

void calibrationMode();
float readAngleNormalizedOversampled(uint16_t zpos, uint16_t mpos, uint16_t samples = 64);

// power LED variable //
bool GreenLedState = false;

// carib //
float calibJigLow = 5.0f;
float calibJigHeigh = 30.3f;
extern float newScale;

void(*resetFunc)(void) = 0;

float trimmedMeanLocal(float *buf, int n, float trim_frac) {
    int cut = (int)(n * trim_frac);
    if (cut * 2 >= n) {
        // return median
        for (int i = 0; i < n - 1; ++i) for (int j = i + 1; j < n; ++j) if (buf[i] > buf[j]) { float t = buf[i]; buf[i] = buf[j]; buf[j] = t; }
        return buf[n/2];
    }
    // sort
    for (int i = 0; i < n - 1; ++i) for (int j = i + 1; j < n; ++j) if (buf[i] > buf[j]) { float t = buf[i]; buf[i] = buf[j]; buf[j] = t; }
    float sum = 0; int cnt = 0;
    for (int i = cut; i < n - cut; ++i) { sum += buf[i]; cnt++; }
    return sum / (float)cnt;
}

// ==============================
// 高分解能正規化角度（オーバーサンプリング）
// ==============================
float readAngleNormalizedOversampled(uint16_t zpos, uint16_t mpos, uint16_t samples) {
    if (samples == 0) samples = 64;
    uint64_t sum_local = 0;
    int succes = 0;

    for (uint16_t i = 0; i < samples; ++i) {
         Wire.beginTransmission(AS5600_ADDR);
        Wire.write(0x0C); // RAW_ANGLE MSB
        Wire.endTransmission(false);
        delayMicroseconds(50);
        Wire.requestFrom((uint8_t)AS5600_ADDR, (uint8_t)2);
        if (Wire.available() < 2) {
            delay(2);
            continue;
        }
        uint16_t raw = ((uint16_t)Wire.read() << 8) | Wire.read();
        raw &= 0x0FFF;
        sum_local += raw;
        succes++;        
        delayMicroseconds(200);
    }

    if (succes == 0) {
        return norm * 360.0f;
    }

    float avg_local = (float)sum_local / (float)succes; // 0..4095
    int32_t delta_local = (int32_t)avg_local - (int32_t)zpos;
    if (delta_local < 0) delta_local += 4096;
    int32_t range_local = (mpos >= zpos) ? (mpos - zpos) : (4096 + mpos - zpos);
    if (range_local <= 0) range_local = 4096; // フォールバック

    if (delta_local > range_local) delta_local = range_local;

    float norm_local = (float)delta_local / (float)range_local;

    // グローバルに反映（UI表示用）
    avg = avg_local;
    delta = (uint32_t)delta_local;
    range = (uint32_t)range_local;
    norm = norm_local;

    return norm_local * 360.0f;
}

// ==============================
// getStableAngleRobust 互換（UI用）
// ==============================
float getStableAngleRobust_local(uint16_t zpos, uint16_t mpos, int preDiscard = 10, int sampleN = 300, float trim_frac = 0.10f, int wait_ms = 10) {
    float last = readEncoderAngleOversampled(4);
    int stableCount = 0;
    while (true) {
        float cur = readEncoderAngleOversampled(4);
        float diff = fabs(cur - last);
        if (diff < 0.005f) stableCount++;
        else stableCount = 0;
        if (stableCount > 20) break;
        last = cur;
        delay(10);
    }

    // 前捨て
    for (int i = 0; i < preDiscard; ++i) { readEncoderAngleOversampled(1); delay(wait_ms); }

    // サンプル取得（配列に格納してtrimmed mean）
    int N = sampleN;
    if (N > 512) N = 512;
    float *samples = (float*)malloc(sizeof(float) * N);
    if (!samples) { // メモリ不足時のフォールバック
        float acc = 0;
        for (int i = 0; i < 50; ++i) { acc += readAngleNormalizedOversampled(zpos, mpos, 8); delay(wait_ms); }
        return acc / 50.0f;
    }
    for (int i = 0; i < N; ++i) {
        samples[i] = readAngleNormalizedOversampled(zpos, mpos, 8);
        delay(wait_ms);
    }
    float tmean = trimmedMeanLocal(samples, N, trim_frac);
    free(samples);
    return tmean;
}

// ==============================
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
        Wire.setClock(100000);
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

        uint16_t zpos = readRawAngleOversampled(64);
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

        uint16_t mpos = readRawAngleOversampled(64);
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
    
    computeLinearCalibration();
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

    // Burn get ZPOS/MANG
    uint16_t zpos = readRegister16(AS5600_ZPOS) & 0x0FFF;
    uint16_t mpos = readRegister16(AS5600_MPOS) & 0x0FFF;

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
        uint8_t burnCount = readBurnCount();
        tft.println(burnCount);
        tft.setCursor(10,55);
        tft.print("ZPOS : "); tft.println(zpos);
        tft.setCursor(10,70);
        tft.print("MPOS : "); tft.println(mpos);
    }else{
        tft.println("Burn False");
    }

    
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

        float angleDeg = getStableAngleRobust_local(zpos,mpos,10,150,0.10f,10)*360.0f;
        measuredAngles[i] = angleDeg;

        tft.fillScreen(ST7735_BLACK);
        tft.setCursor(10, 20);
        tft.print("Height: ");
        tft.print(knownHeights[i], 1);
        tft.println("mm");
        tft.setCursor(10, 40);
        tft.print("norm: ");
        tft.print(angleDeg, 3);
        tft.println(" deg");

        tft.setCursor(10, 55);
        tft.print("delta: ");
        tft.print(delta, 3);


        tft.setCursor(10, 70);
        tft.print("range: ");
        tft.print(range, 3);

        tft.setCursor(10, 85);
        tft.print("avg: ");
        tft.print(avg, 3);




        while (digitalRead(ButtonPin) == LOW);
        while (digitalRead(ButtonPin) == HIGH);
    }

    // ---- EEPROM save ----
    for (int i=0; i<NUM_POINTS; i++){
        EEPROM.put(ADDR_KNOWN_ANGLES + i*sizeof(float), measuredAngles[i]);
        EEPROM.put(ADDR_KNOWN_HEIGHTS + i*sizeof(float), knownHeights[i]);
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
            // set reference offset via current measured angle
            uint16_t zpos = readRegister16(AS5600_ZPOS) & 0x0FFF;
            uint16_t mpos = readRegister16(AS5600_MPOS) & 0x0FFF;
            float curAngle = readEncoderAngleOversampled(64);
            float measuredHeight = interpolateHeight(curAngle);
            const float referenceHeight = 5.0f;
            heightOffset = measuredHeight - referenceHeight;
            EEPROM.put(300, heightOffset);
        }
    } else {
        if(buttonPressed) {
         setInitialAngleFromSensor();
         saveCurrentZeroPositionToEEPROM();
        }
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
