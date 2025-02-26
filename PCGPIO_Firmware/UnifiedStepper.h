#ifndef UNIFIED_STEPPER_H
#define UNIFIED_STEPPER_H

#include "UnifiedTimer.h"

#define MAX_STEPPERS 2 // Maximum number of steppers supported

class StepperManager {
public:
    static StepperManager& getInstance() {
        static StepperManager instance;
        return instance;
    }

    int addStepper(int stepPin, int dirPin);
    void step(int stepperIdx, long steps, unsigned long stepsPerSecond);
    void dispose(int stepperIdx);

private:
    StepperManager();
    ~StepperManager();

    struct Stepper {
        int stepPin;
        int dirPin;
        volatile long stepsRemaining;
        volatile unsigned long stepInterval;
        volatile unsigned long lastStepTime;
        bool active;
    };

    Stepper steppers[MAX_STEPPERS];
    int stepperCount;
    UnifiedTimer timer;

    static void stepISRWrapper();
    void stepISR();
};

#endif