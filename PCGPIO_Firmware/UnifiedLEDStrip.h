#ifndef UNIFIED_LED_STRIP_H
#define UNIFIED_LED_STRIP_H

#include <Arduino.h>

#define MAX_LEDS 300 // Fixed maximum number of LEDs

enum LEDType { WS2812, SK6812_RGBW, APA102 };

struct RGBW {
    uint8_t r, g, b, w;
    uint32_t packed;
};

class UnifiedLEDStrip {
public:
    UnifiedLEDStrip(LEDType type, uint8_t dataPin, uint8_t clockPin, uint16_t numLEDs);
    void setPixelColor(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b, uint8_t w = 0);
    void show();
    uint16_t getNumLEDs() { return numLEDs; }
    LEDType getType() { return ledType; }

private:
    LEDType ledType;
    uint8_t dataPin, clockPin;
    uint16_t numLEDs;
    RGBW buffer[MAX_LEDS];
    void sendBitsNonAVR(int bitsPerLED);
};

#endif