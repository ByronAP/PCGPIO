#include <Arduino.h>
#include "UnifiedTimer.h"
#include "UnifiedLEDStrip.h"
#include "UnifiedStepper.h"

// Constants
#define MAX_INTERRUPTS 6  // Maximum number of interrupt slots
#define MAX_STRIPS 10     // Maximum number of LED strips

// Enum for pin states
enum PinState { UNSET = -1, INPUT_PIN, OUTPUT_PIN, INPUT_PULLUP_PIN, PWM_PIN, INTERRUPT_PIN };

// Global Variables
PinState pinStates[70];
char buffer[64];
int bufIndex = 0;
UnifiedLEDStrip* strips[MAX_STRIPS];
uint8_t stripCount = 0;
bool ledStripActive = false;
UnifiedLEDStrip* ledStrip = nullptr;
bool interruptAttached[MAX_INTERRUPTS] = {false};
uint8_t attachedPins[MAX_INTERRUPTS];
void (*interruptHandlers[MAX_INTERRUPTS])() = {nullptr};
StepperManager& stepperManager = StepperManager::getInstance();
int interruptModes[MAX_INTERRUPTS];

// Structure to hold interrupt event data
struct InterruptEvent {
    unsigned long timestamp;
    int slot;
    int pinState;
};

// Circular buffer for interrupt events
#define EVENT_QUEUE_SIZE 16
InterruptEvent eventQueue[EVENT_QUEUE_SIZE];
int eventHead = 0;
int eventTail = 0;

// ISR Wrapper Functions
void ISR_slot0() { genericISR(0); }
void ISR_slot1() { genericISR(1); }
void ISR_slot2() { genericISR(2); }
void ISR_slot3() { genericISR(3); }
void ISR_slot4() { genericISR(4); }
void ISR_slot5() { genericISR(5); }

// Array of ISR Functions
void (*isrFunctions[MAX_INTERRUPTS])() = {
    ISR_slot0, ISR_slot1, ISR_slot2,
    ISR_slot3, ISR_slot4, ISR_slot5
};

// Generic ISR Handler
void genericISR(int slot) {
    unsigned long ts = micros();
    int pin = attachedPins[slot];
    int state = digitalRead(pin);
    
    eventQueue[eventHead].timestamp = ts;
    eventQueue[eventHead].slot = slot;
    eventQueue[eventHead].pinState = state;
    eventHead = (eventHead + 1) % EVENT_QUEUE_SIZE;
}

// Utility Functions
void sendEcho(unsigned long ts, const char* msg, const char* id) {
    Serial.print(ts);
    Serial.print(",E,");
    Serial.print(msg);
    Serial.print(",");
    Serial.println(id);
}

void sendError(unsigned long ts, const char* msg, const char* id) {
    Serial.print(ts);
    Serial.print(",ERROR,");
    Serial.print(msg);
    Serial.print(",");
    Serial.println(id);
}

int freeRam() {
    extern int __heap_start, *__brkval;
    int v;
    return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval);
}

// Command Processing Function
void processCommand(char* cmd) {
    unsigned long ts = micros();
    char* token = strtok(cmd, ",");
    if (!token) return;

    char c = token[0];
    char* id = nullptr;
    char* lastToken = nullptr;

    // Find the last token as the ID
    while ((token = strtok(NULL, ",")) != NULL) {
        id = token;
        lastToken = token;
    }

    if (id == nullptr) {
        sendError(ts, "Missing command ID", "NO_ID");
        return;
    }

    // Reset strtok to parse parameters before ID
    strtok(cmd, ",");

    if (c == 'T') {
        Serial.print(ts);
        Serial.print(",T,");
        Serial.println(id);
    }
    else if (c == 'B') {
        char* pinStr;
        int pins[16];
        int pinCount = 0;

        while ((pinStr = strtok(NULL, ",")) != NULL && pinCount < 16 && pinStr != lastToken) {
            int pin = atoi(pinStr);
            if (pin < 0 || pin > 69) {
                sendError(ts, "Invalid pin number in bulk read", id);
                return;
            }
            pins[pinCount++] = pin;
            if (pinStates[pin] == UNSET) pinMode(pin, INPUT);
        }

        if (pinCount == 0) {
            sendError(ts, "No pins specified for bulk read", id);
            return;
        }

        Serial.print(ts);
        Serial.print(",B");
        for (int i = 0; i < pinCount; i++) {
            Serial.print(",");
            Serial.print(pins[i]);
            Serial.print(",");
            Serial.print(digitalRead(pins[i]));
        }
        Serial.print(",");
        Serial.println(id);
    }
    else if (c == 'C') {
        char* baudStr = strtok(NULL, ",");
        if (baudStr && baudStr != lastToken) {
            long baud = atol(baudStr);
            if (baud > 0) {
                Serial.end();
                Serial.begin(baud);
                sendEcho(ts, "Baud rate set", id);
            } else {
                sendError(ts, "Invalid baud rate", id);
            }
        } else {
            sendError(ts, "Missing baud rate", id);
        }
    }
    else if (c == 'V') {
        char* pairStr;
        int pinCount = 0;

        while ((pairStr = strtok(NULL, ",")) != NULL && pinCount < 16 && pairStr != lastToken) {
            char* pinStr = strtok(pairStr, ":");
            char* valStr = strtok(NULL, ":");
            if (!pinStr || !valStr) {
                sendError(ts, "Invalid pin:value pair in bulk write", id);
                return;
            }

            int pin = atoi(pinStr);
            int val = atoi(valStr);
            if (pin < 0 || pin > 69) {
                sendError(ts, "Invalid pin number in bulk write", id);
                return;
            }
            if (val != 0 && val != 1) {
                sendError(ts, "Invalid value (must be 0 or 1) in bulk write", id);
                return;
            }

            pinMode(pin, OUTPUT);
            pinStates[pin] = OUTPUT_PIN;
            digitalWrite(pin, val);
            pinCount++;
        }

        if (pinCount == 0) {
            sendError(ts, "No pin:value pairs specified for bulk write", id);
            return;
        }

        sendEcho(ts, "Bulk write done", id);
    }
    else if (c == 'G') {
        Serial.print(ts);
        Serial.print(",G,");

        #if defined(ARDUINO_AVR_MEGA2560)
            Serial.print("Mega2560");
        #elif defined(ESP32)
            Serial.print("ESP32");
        #elif defined(ESP8266)
            Serial.print("ESP8266");
        #elif defined(ARDUINO_AVR_UNO)
            Serial.print("Uno");
        #elif defined(ARDUINO_SAMD_ZERO)
            Serial.print("Zero");
        #elif defined(ARDUINO_ARCH_RP2040)
            Serial.print("RP2040");
        #else
            Serial.print("Unknown");
        #endif
        Serial.print(",");

        #if defined(ARDUINO_ARCH_AVR)
            Serial.print("AVR");
        #elif defined(ARDUINO_ARCH_ESP32)
            Serial.print("ESP32");
        #elif defined(ARDUINO_ARCH_ESP8266)
            Serial.print("ESP8266");
        #elif defined(ARDUINO_ARCH_SAMD)
            Serial.print("SAMD");
        #elif defined(ARDUINO_ARCH_STM32)
            Serial.print("STM32");
        #elif defined(ARDUINO_ARCH_RP2040)
            Serial.print("RP2040");
        #else
            Serial.print("Unknown");
        #endif
        Serial.print(",");

        #if defined(F_CPU)
            Serial.print(F_CPU / 1000000UL);
        #else
            Serial.print("0");
        #endif
        Serial.print(",");

        Serial.print(NUM_DIGITAL_PINS);
        Serial.print(",");

        Serial.print(NUM_ANALOG_INPUTS);
        Serial.print(",");

        int pwmPins[NUM_DIGITAL_PINS];
        int pwmCount = 0;
        for (int pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
            #if defined(ESP32) || defined(ESP8266)
                if (digitalPinHasPWM(pin)) {
                    pwmPins[pwmCount++] = pin;
                }
            #else
                if (digitalPinToTimer(pin) != NOT_ON_TIMER) {
                    pwmPins[pwmCount++] = pin;
                }
            #endif
        }
        Serial.print(pwmCount);
        for (int i = 0; i < pwmCount; i++) {
            Serial.print(",");
            Serial.print(pwmPins[i]);
        }
        Serial.print(",");

        int interruptPins[NUM_DIGITAL_PINS];
        int interruptCount = 0;
        for (int pin = 0; pin < NUM_DIGITAL_PINS; pin++) {
            if (digitalPinToInterrupt(pin) != NOT_AN_INTERRUPT) {
                interruptPins[interruptCount++] = pin;
            }
        }
        Serial.print(interruptCount);
        for (int i = 0; i < interruptCount; i++) {
            Serial.print(",");
            Serial.print(interruptPins[i]);
        }
        Serial.print(",");

        Serial.print(MAX_INTERRUPTS);
        Serial.print(",");

        int slotsInUse = 0;
        for (int i = 0; i < MAX_INTERRUPTS; i++) {
            if (interruptAttached[i]) {
                slotsInUse++;
            }
        }
        Serial.print(slotsInUse);
        Serial.print(",");

        #if defined(ARDUINO_ARCH_AVR) && defined(ARDUINO_AVR_MEGA2560)
            Serial.print(6);
        #elif defined(ARDUINO_ARCH_AVR) && defined(ARDUINO_AVR_UNO)
            Serial.print(3);
        #elif defined(ESP32)
            Serial.print(4);
        #elif defined(ESP8266)
            Serial.print(2);
        #elif defined(ARDUINO_ARCH_SAMD)
            Serial.print(8);
        #elif defined(ARDUINO_ARCH_RP2040)
            Serial.print(4);
        #else
            Serial.print(0);
        #endif
        Serial.print(",");

        Serial.print(freeRam());
        Serial.print(",");

        #if defined(FLASHEND)
            Serial.print(FLASHEND + 1);
        #elif defined(ESP32) || defined(ESP8266)
            Serial.print(ESP.getFlashChipSize());
        #elif defined(ARDUINO_ARCH_SAMD)
            Serial.print(262144);
        #elif defined(ARDUINO_ARCH_RP2040)
            Serial.print(2097152);
        #else
            Serial.print(0);
        #endif
        Serial.print(",");

        #if defined(ESP32)
            uint64_t chipId = ESP.getEfuseMac();
            Serial.print((uint32_t)(chipId >> 32), HEX);
            Serial.print((uint32_t)chipId, HEX);
        #elif defined(ESP8266)
            Serial.print(ESP.getChipId(), HEX);
        #elif defined(ARDUINO_ARCH_RP2040)
            Serial.print(rp2040.getChipID(), HEX);
        #else
            Serial.print("0");
        #endif
        Serial.print(",");
        Serial.println(id);
    }
    else if (c == 'Q') {
        sendEcho(ts, "Resetting", id);
        delay(10);

        #if defined(ARDUINO_ARCH_AVR)
            asm volatile ("jmp 0");
        #elif defined(ESP32)
            ESP.restart();
        #elif defined(ESP8266)
            ESP.restart();
        #elif defined(ARDUINO_ARCH_SAMD)
            NVIC_SystemReset();
        #elif defined(ARDUINO_ARCH_STM32)
            NVIC_SystemReset();
        #elif defined(ARDUINO_ARCH_RP2040)
            watchdog_reboot(0, 0, 0);
        #else
            sendEcho(ts, "Reset not fully supported, attempting watchdog", id);
            while (true) {}
        #endif
    }
    else if (c == 'D') {
        int pin = atoi(strtok(NULL, ","));
        if (pinStates[pin] == UNSET) pinMode(pin, INPUT);
        Serial.print(ts);
        Serial.print(",D,");
        Serial.print(pin);
        Serial.print(",");
        Serial.print(digitalRead(pin));
        Serial.print(",");
        Serial.println(id);
    }
    else if (c == 'A') {
        int pin = atoi(strtok(NULL, ","));
        Serial.print(ts);
        Serial.print(",A,");
        Serial.print(pin);
        Serial.print(",");
        Serial.print(analogRead(pin));
        Serial.print(",");
        Serial.println(id);
    }
    else if (c == 'W') {
        int pin = atoi(strtok(NULL, ","));
        int val = atoi(strtok(NULL, ","));
        pinMode(pin, OUTPUT);
        pinStates[pin] = OUTPUT_PIN;
        digitalWrite(pin, val);
        sendEcho(ts, "Digital write done", id);
    }
    else if (c == 'P') {
        int pin = atoi(strtok(NULL, ","));
        char* valStr = strtok(NULL, ",");
        if (strcmp(valStr, "stop") == 0) {
            analogWrite(pin, 0);
            pinStates[pin] = OUTPUT_PIN;
        } else {
            int val = atoi(valStr);
            analogWrite(pin, val);
            pinStates[pin] = PWM_PIN;
        }
        sendEcho(ts, "PWM set", id);
    }
    else if (c == 'M') {
        int pin = atoi(strtok(NULL, ","));
        int mode = atoi(strtok(NULL, ","));
        if (mode == 0) {
            pinMode(pin, INPUT);
            pinStates[pin] = INPUT_PIN;
        } else if (mode == 1) {
            pinMode(pin, OUTPUT);
            pinStates[pin] = OUTPUT_PIN;
        } else if (mode == 2) {
            pinMode(pin, INPUT_PULLUP);
            pinStates[pin] = INPUT_PULLUP_PIN;
        }
        sendEcho(ts, "Mode set", id);
    }
    else if (c == 'I') {
        int slot = atoi(strtok(NULL, ","));
        int pin = atoi(strtok(NULL, ","));
        int mode = atoi(strtok(NULL, ","));
        int intr = digitalPinToInterrupt(pin);
        if (intr == NOT_AN_INTERRUPT) {
            sendError(ts, "Not an interrupt pin", id);
            return;
        }
        if (slot >= 0 && slot < MAX_INTERRUPTS) {
            #if defined(ARDUINO_ARCH_RP2040)
                attachInterrupt(intr, isrFunctions[slot], static_cast<PinStatus>(mode));
            #else
                attachInterrupt(intr, isrFunctions[slot], mode);
            #endif
            interruptAttached[slot] = true;
            attachedPins[slot] = pin;
            interruptModes[slot] = mode;
            pinStates[pin] = INTERRUPT_PIN;
            sendEcho(ts, "Interrupt attached", id);
        } else {
            sendError(ts, "No interrupt slots", id);
        }
    }
    else if (c == 'R') {
        int pin = atoi(strtok(NULL, ","));
        for (int i = 0; i < MAX_INTERRUPTS; i++) {
            if (interruptAttached[i] && attachedPins[i] == pin) {
                detachInterrupt(digitalPinToInterrupt(pin));
                interruptAttached[i] = false;
                pinStates[pin] = UNSET;
                break;
            }
        }
        sendEcho(ts, "Interrupt detached", id);
    }
    else if (c == 'N') {
        char* action = strtok(NULL, ",");
        if (strcmp(action, "setup") == 0) {
            int stepPin = atoi(strtok(NULL, ","));
            int dirPin = atoi(strtok(NULL, ","));
            int idx = stepperManager.addStepper(stepPin, dirPin);
            if (idx >= 0) {
                Serial.print("Stepper ");
                Serial.print(idx);
                Serial.print(" set up,");
                Serial.println(id);
            } else {
                Serial.print("Failed to add stepper,");
                Serial.println(id);
            }
        } else if (strcmp(action, "step") == 0) {
            int idx = atoi(strtok(NULL, ","));
            long steps = atol(strtok(NULL, ","));
            unsigned long sps = atol(strtok(NULL, ","));
            stepperManager.step(idx, steps, sps);
            sendEcho(ts, "Stepper step set", id);
        } else if (strcmp(action, "dispose") == 0) {
            int idx = atoi(strtok(NULL, ","));
            stepperManager.dispose(idx);
            sendEcho(ts, "Stepper disposed", id);
        }
    }
    else if (c == 'L') {
        char* action = strtok(NULL, ",");
        if (strcmp(action, "setup") == 0) {
            int type = atoi(strtok(NULL, ","));
            int dp = atoi(strtok(NULL, ","));
            int cp = atoi(strtok(NULL, ","));
            int num = atoi(strtok(NULL, ","));
            if (stripCount >= MAX_STRIPS) {
                sendError(ts, "Max strips reached", id);
                return;
            }
            strips[stripCount] = new UnifiedLEDStrip((LEDType)type, dp, cp, num);
            ledStrip = strips[stripCount];
            ledStripActive = true;
            stripCount++;
            sendEcho(ts, "LED strip set up", id);
        } else if (strcmp(action, "set") == 0) {
            int pix = atoi(strtok(NULL, ","));
            int r = atoi(strtok(NULL, ","));
            int g = atoi(strtok(NULL, ","));
            int b = atoi(strtok(NULL, ","));
            if (ledStripActive) {
                ledStrip->setPixelColor(pix, r, g, b);
                sendEcho(ts, "Pixel set", id);
            } else {
                sendError(ts, "No LED strip", id);
            }
        } else if (strcmp(action, "show") == 0) {
            if (ledStripActive) {
                ledStrip->show();
                sendEcho(ts, "LEDs shown", id);
            } else {
                sendError(ts, "No LED strip", id);
            }
        }
    }
    else {
        sendError(ts, "Unknown command", id);
    }
}

// Setup Function
void setup() {
    Serial.begin(115200);
    for (int i = 0; i < 70; i++) pinStates[i] = UNSET;
}

// Loop Function
void loop() {
    while (Serial.available() > 0) {
        char c = Serial.read();
        if (c == '\n') {
            buffer[bufIndex] = '\0';
            processCommand(buffer);
            bufIndex = 0;
        } else if (bufIndex < 63) {
            buffer[bufIndex++] = c;
        } else {
            bufIndex = 0;
            sendError(micros(), "Buffer overflow", "NO_ID");
        }
    }

    while (eventTail != eventHead) {
        InterruptEvent event = eventQueue[eventTail];
        int slot = event.slot;
        int pin = attachedPins[slot];
        int mode = interruptModes[slot];
        
        const char* modeStr;
        switch (mode) {
            case RISING: modeStr = "RISING"; break;
            case FALLING: modeStr = "FALLING"; break;
            case CHANGE: modeStr = "CHANGE"; break;
            default: modeStr = "UNKNOWN"; break;
        }
        
        Serial.print(event.timestamp);
        Serial.print(",I,");
        Serial.print(pin);
        Serial.print(",");
        Serial.print(modeStr);
        Serial.print(",");
        Serial.print(event.pinState);
        Serial.print(",INT");
        Serial.println();
        
        eventTail = (eventTail + 1) % EVENT_QUEUE_SIZE;
    }

    #ifdef PLATFORM_ESP32
    yield();
    #endif
}