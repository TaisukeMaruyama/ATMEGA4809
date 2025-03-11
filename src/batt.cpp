#include "batt.h"
#include <Arduino.h>

#define BattPin A11

int getBatteryRaw(){
    return analogRead(BattPin);
}