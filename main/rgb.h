#ifndef RGB_LED_H
#define RGB_LED_H

#include <Arduino.h>

#define RGB_BUILTIN 2
#define RGB_BRIGHTNESS 255
#define LED_UPDATE_INTERVAL 50

// LED state variables
static unsigned long lastLedUpdate = 0;
static int breathBrightness = 0;
static bool breathingUp = true;

inline void setupRGB() {
    rgbLedWrite(RGB_BUILTIN, 0, 0, 0); // Turn off LED initially
}

inline void handleRGB() {
    unsigned long currentMillis = millis();
    if (currentMillis - lastLedUpdate > LED_UPDATE_INTERVAL) {
        if (breathingUp) {
            breathBrightness += 5;
            if (breathBrightness >= RGB_BRIGHTNESS) {
                breathBrightness = RGB_BRIGHTNESS;
                breathingUp = false;
            }
        } else {
            breathBrightness -= 5;
            if (breathBrightness <= 10) {
                breathBrightness = 10;
                breathingUp = true;
            }
        }
        
        // Apply breathing effect with a rainbow color pattern
        float hue = (float)millis() / 10000.0f * 360.0f; // Complete color cycle every 10 seconds
        float h = fmod(hue, 360) / 360.0f;
        float s = 1.0f;
        float v = breathBrightness / 255.0f;
        
        float c = v * s;
        float x = c * (1 - abs(fmod(h * 6, 2) - 1));
        float m = v - c;
        
        uint8_t red, green, blue;
        
        if(h < 1.0f/6.0f) {
            red = (c + m) * 255;
            green = (x + m) * 255;
            blue = m * 255;
        } else if(h < 2.0f/6.0f) {
            red = (x + m) * 255;
            green = (c + m) * 255;
            blue = m * 255;
        } else if(h < 3.0f/6.0f) {
            red = m * 255;
            green = (c + m) * 255;
            blue = (x + m) * 255;
        } else if(h < 4.0f/6.0f) {
            red = m * 255;
            green = (x + m) * 255;
            blue = (c + m) * 255;
        } else if(h < 5.0f/6.0f) {
            red = (x + m) * 255;
            green = m * 255;
            blue = (c + m) * 255;
        } else {
            red = (c + m) * 255;
            green = m * 255;
            blue = (x + m) * 255;
        }
        
        rgbLedWrite(RGB_BUILTIN, red, green, blue);
        lastLedUpdate = currentMillis;
    }
}

#endif