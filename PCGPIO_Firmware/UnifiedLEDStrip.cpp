#include "UnifiedLEDStrip.h"
#include <Arduino.h>

UnifiedLEDStrip::UnifiedLEDStrip(LEDType type, uint8_t dataPin, uint8_t clockPin, uint16_t numLEDs) {
    if (numLEDs == 0 || numLEDs > MAX_LEDS) {
        Serial.println("ERROR: Invalid number of LEDs");
        return;
    }
    if (type == APA102 && clockPin == 255) {
        Serial.println("ERROR: APA102 requires a clock pin");
        return;
    }
    if (dataPin < 0 || dataPin > 70 || (clockPin != 255 && (clockPin < 0 || clockPin > 70))) {
        Serial.println("ERROR: Invalid pin assignment");
        return;
    }
    this->ledType = type;
    this->dataPin = dataPin;
    this->clockPin = clockPin;
    this->numLEDs = numLEDs;
    pinMode(dataPin, OUTPUT);
    if (clockPin != 255) pinMode(clockPin, OUTPUT);
    for (uint16_t i = 0; i < numLEDs; i++) {
        buffer[i].r = buffer[i].g = buffer[i].b = buffer[i].w = 0;
        buffer[i].packed = 0;
    }
}

void UnifiedLEDStrip::setPixelColor(uint16_t pixel, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
    if (pixel >= numLEDs) {
        Serial.print("ERROR: Pixel index out of range: ");
        Serial.println(pixel);
        return;
    }
    buffer[pixel].r = r;
    buffer[pixel].g = g;
    buffer[pixel].b = b;
    buffer[pixel].w = w;
    buffer[pixel].packed = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
    if (ledType == SK6812_RGBW) buffer[pixel].packed = (buffer[pixel].packed << 8) | w;
}

void UnifiedLEDStrip::show() {
    int bitsPerLED = (ledType == SK6812_RGBW) ? 32 : 24;
    sendBitsNonAVR(bitsPerLED);
}

void UnifiedLEDStrip::sendBitsNonAVR(int bitsPerLED) {
    for (uint16_t i = 0; i < numLEDs; i++) {
        uint32_t color = buffer[i].packed;
        for (int bit = bitsPerLED - 1; bit >= 0; bit--) {
            bool b = (color & (1UL << bit)) != 0;
            digitalWrite(dataPin, HIGH);
            if (b) {
                for (volatile int d = 0; d < 8; d++) {} // ~0.8us, calibrate per platform
            } else {
                for (volatile int d = 0; d < 4; d++) {} // ~0.4us
            }
            digitalWrite(dataPin, LOW);
            if (b) {
                for (volatile int d = 0; d < 5; d++) {} // ~0.45us
            } else {
                for (volatile int d = 0; d < 9; d++) {} // ~0.85us
            }
        }
    }
}