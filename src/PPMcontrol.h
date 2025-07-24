#ifndef PPM_CONTROL_H
#define PPM_CONTROL_H

#include <Arduino.h>
#include <PPMReader.h>
#include <Servo.h>

//void initPPMControl();
void ppmControlTask(void*);

// --- PPM and Servo Setup ---
#define PPM_INPUT_PIN 8
#define NUM_CHANNELS 6

// extern volatile bool ppmControlEnabled = true;
// extern unsigned lastRollValue = 1500;
// extern unsigned lastPitchValue = 1500;
// extern unsigned lastThrottleValue = 1000;
// extern const int DEADZONE = 3;

class PPMControl {
    public:
        void initPPMControl();
        //void ppmControlTask(void*);
    private:
};

#endif // PPM_CONTROL_H