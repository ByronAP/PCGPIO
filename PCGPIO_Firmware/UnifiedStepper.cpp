#include "UnifiedStepper.h"
#include <Arduino.h>

StepperManager::StepperManager() : stepperCount(0) {}

StepperManager::~StepperManager() {
    for (int i = 0; i < stepperCount; i++) {
        steppers[i].active = false;
    }
}

int StepperManager::addStepper(int stepPin, int dirPin) {
    if (stepperCount >= MAX_STEPPERS) {
        Serial.println("ERROR: Maximum steppers reached");
        return -1;
    }
    Stepper& s = steppers[stepperCount];
    s.stepPin = stepPin;
    s.dirPin = dirPin;
    s.stepsRemaining = 0;
    s.stepInterval = 0;
    s.lastStepTime = 0;
    s.active = true;
    pinMode(stepPin, OUTPUT);
    pinMode(dirPin, OUTPUT);
    digitalWrite(stepPin, LOW);
    digitalWrite(dirPin, LOW);
    return stepperCount++;
}

void StepperManager::step(int stepperIdx, long steps, unsigned long stepsPerSecond) {
    if (stepperIdx < 0 || stepperIdx >= stepperCount || !steppers[stepperIdx].active) {
        Serial.println("ERROR: Invalid stepper index");
        return;
    }
    Stepper& s = steppers[stepperIdx];
    s.stepsRemaining = steps;
    s.stepInterval = 1000000UL / stepsPerSecond; // Convert to microseconds
    digitalWrite(s.dirPin, steps >= 0 ? HIGH : LOW); // Set direction
    if (!UnifiedTimer::isRunning()) {
        UnifiedTimer::begin(stepISRWrapper, 100); // Start timer at 100us interval
    }
}

void StepperManager::dispose(int stepperIdx) {
    if (stepperIdx < 0 || stepperIdx >= stepperCount) return;
    steppers[stepperIdx].active = false;
    steppers[stepperIdx].stepsRemaining = 0;
    digitalWrite(steppers[stepperIdx].stepPin, LOW);
}

void StepperManager::stepISRWrapper() {
    StepperManager::getInstance().stepISR();
}

void StepperManager::stepISR() {
    bool anyRemaining = false;
    for (int i = 0; i < stepperCount; i++) {
        Stepper& s = steppers[i];
        if (s.active && s.stepsRemaining != 0) {
            unsigned long now = micros();
            if (now - s.lastStepTime >= s.stepInterval) {
                digitalWrite(s.stepPin, HIGH);
                delayMicroseconds(10); // Pulse width
                digitalWrite(s.stepPin, LOW);
                s.lastStepTime = now;
                s.stepsRemaining += (s.stepsRemaining > 0) ? -1 : 1;
            }
            if (s.stepsRemaining != 0) anyRemaining = true;
        }
    }
    if (!anyRemaining && UnifiedTimer::isRunning()) {
        UnifiedTimer::end(); // Stop timer if no steps remain
    }
}