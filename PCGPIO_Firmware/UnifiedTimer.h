#ifndef UNIFIED_TIMER_H
#define UNIFIED_TIMER_H

#include <Arduino.h>

class UnifiedTimer {
public:
    static void begin(void (*isr)(), unsigned long interval, bool oneShot = false);
    static void end();
    static bool isRunning();
    static void callUserISR();
    static bool isOneShot();
    static void setRunning(bool state);

private:
    static void (*userISR)();
    static bool oneShot;
    static bool running;
};

#endif