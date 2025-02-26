#include "UnifiedTimer.h"
#include <Arduino.h>

// Define static members
void (*UnifiedTimer::userISR)() = nullptr;
bool UnifiedTimer::oneShot = false;
bool UnifiedTimer::running = false;

void UnifiedTimer::callUserISR() {
    if (userISR) userISR();
}

bool UnifiedTimer::isOneShot() {
    return oneShot;
}

void UnifiedTimer::setRunning(bool state) {
    running = state;
}

#if defined(__AVR__)
// AVR Implementation
ISR(TIMER1_COMPA_vect) {
    UnifiedTimer::callUserISR();
    if (UnifiedTimer::isOneShot()) {
        TCCR1B &= ~(1 << CS11);
        UnifiedTimer::setRunning(false);
    }
}

void UnifiedTimer::begin(void (*isr)(), unsigned long interval, bool oneShot) {
    userISR = isr;
    UnifiedTimer::oneShot = oneShot;
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = (F_CPU / 8UL) * interval / 1000000UL;
    TCCR1B |= (1 << WGM12);
    TCCR1B |= (1 << CS11);
    TIMSK1 |= (1 << OCIE1A);
    interrupts();
    running = true;
}

void UnifiedTimer::end() {
    TCCR1B &= ~(1 << CS11);
    TIMSK1 &= ~(1 << OCIE1A);
    userISR = nullptr;
    running = false;
}

#elif defined(ESP32)
// ESP32 Implementation
#include <driver/timer.h>
static timer_group_t timer_group = TIMER_GROUP_0;
static timer_idx_t timer_idx = TIMER_0;

static void IRAM_ATTR timerISR(void* arg) {
    UnifiedTimer::callUserISR();
    if (UnifiedTimer::isOneShot()) {
        timer_pause(timer_group, timer_idx);
        UnifiedTimer::setRunning(false);
    }
    timer_group_clr_intr_status_in_isr(timer_group, timer_idx);
}

void UnifiedTimer::begin(void (*isr)(), unsigned long interval, bool oneShot) {
    userISR = isr;
    UnifiedTimer::oneShot = oneShot;
    timer_config_t config = {
        .alarm_en = TIMER_ALARM_EN,
        .counter_en = TIMER_PAUSE,
        .intr_type = TIMER_INTR_LEVEL,
        .counter_dir = TIMER_COUNT_UP,
        .auto_reload = oneShot ? TIMER_AUTORELOAD_DIS : TIMER_AUTORELOAD_EN,
        .divider = 80 // 80 MHz / 80 = 1 MHz, 1 µs per tick
    };
    timer_init(timer_group, timer_idx, &config);
    timer_set_counter_value(timer_group, timer_idx, 0);
    timer_set_alarm_value(timer_group, timer_idx, interval); // interval in microseconds
    timer_enable_intr(timer_group, timer_idx);
    timer_isr_register(timer_group, timer_idx, timerISR, NULL, ESP_INTR_FLAG_IRAM, NULL);
    timer_start(timer_group, timer_idx);
    running = true;
}

void UnifiedTimer::end() {
    timer_pause(timer_group, timer_idx);
    timer_disable_intr(timer_group, timer_idx);
    userISR = nullptr;
    running = false;
}

#elif defined(ARDUINO_ARCH_RP2040)
// Pico Implementation
#include "mbed.h"

static mbed::Ticker repeatingTicker;
static mbed::Timeout oneShotTimeout;

// Wrapper for one-shot timer to call user ISR and stop
static void oneShotWrapper() {
    UnifiedTimer::callUserISR();      // Call the user ISR via the public method
    UnifiedTimer::setRunning(false);  // Update the running state via the public method
}

void UnifiedTimer::begin(void (*isr)(), unsigned long interval, bool oneShot) {
    end(); // Stop any existing timer first
    UnifiedTimer::userISR = isr;
    UnifiedTimer::oneShot = oneShot;
    float intervalSec = static_cast<float>(interval) / 1000000.0f; // Convert us to seconds
    if (oneShot) {
        oneShotTimeout.attach(&oneShotWrapper, intervalSec);
    } else {
        repeatingTicker.attach(isr, intervalSec);
    }
    UnifiedTimer::running = true;
}

void UnifiedTimer::end() {
    if (UnifiedTimer::oneShot) {
        oneShotTimeout.detach();
    } else {
        repeatingTicker.detach();
    }
    UnifiedTimer::userISR = nullptr;
    UnifiedTimer::running = false;
}

#else
// Default implementation for unsupported platforms
void UnifiedTimer::begin(void (*isr)(), unsigned long interval, bool oneShot) {
    Serial.println("ERROR: UnifiedTimer not implemented for this platform");
    running = false;
}

void UnifiedTimer::end() {
    Serial.println("ERROR: UnifiedTimer not implemented for this platform");
    running = false;
}
#endif

bool UnifiedTimer::isRunning() {
    return running;
}