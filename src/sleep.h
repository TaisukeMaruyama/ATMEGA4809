#ifndef SLEEP_H
#define SLEEP_H

void updateSleepStatus(float currentHeight, int tftPowerPin);
void handleSleepLED(int ledPin);
bool isSleeping();

#endif