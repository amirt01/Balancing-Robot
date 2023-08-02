//
// Created by amirt on 8/1/2023.
//

#ifndef BALANCINGROBOT_STATUSLED_H
#define BALANCINGROBOT_STATUSLED_H

#include <WiFiNINA.h>

//TODO: add color and intricate status support

class StatusLED {
public:
    void InitLED() {
        pinMode(LEDR, OUTPUT);
        digitalWrite(LEDR, HIGH);

        blinkTime = millis() + BLINK_DELAY;
    }

    void UpdateLED() {
        if (millis() >= blinkTime) {
            digitalWrite(LEDR, blinkMode ? HIGH : LOW);
            blinkMode = !blinkMode;
            blinkTime += BLINK_DELAY;
        }
    }

private:
    bool blinkMode = false;
    uint32_t blinkTime = 0;

    const unsigned long BLINK_DELAY = 500UL;
};


#endif //BALANCINGROBOT_STATUSLED_H
