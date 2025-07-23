#include "PPMControl.h"
#include "arduino_freertos.h"
//#include "freertos/task.h"

#define PPM_INPUT_PIN 8
#define NUM_CHANNELS 6

PPMReader ppm(PPM_INPUT_PIN, NUM_CHANNELS);
Servo ppmServoRoll, ppmServoPitch, ppmServoThrottle;

unsigned lastRollValue = 1500;
unsigned lastPitchValue = 1500;
unsigned lastThrottleValue = 1000;
const int DEADZONE = 3;

volatile bool ppmControlEnabled = false;

void initPPMControl() {
    ppmServoRoll.attach(6);
    ppmServoPitch.attach(7);
    ppmServoThrottle.attach(10);
}

void ppmControlTask(void*) {
    
    initPPMControl();

    while (1) {
        if (ppmControlEnabled) {
            unsigned rollValue     = ppm.latestValidChannelValue(1, 1500);
            unsigned pitchValue    = ppm.latestValidChannelValue(2, 1500);
            unsigned throttleValue = ppm.latestValidChannelValue(3, 1000);

            if (abs((int)rollValue - (int)lastRollValue) > DEADZONE) {
                ppmServoRoll.writeMicroseconds(rollValue);
                lastRollValue = rollValue;
            }
            if (abs((int)pitchValue - (int)lastPitchValue) > DEADZONE) {
                ppmServoPitch.writeMicroseconds(pitchValue);
                lastPitchValue = pitchValue;
            }
            if (abs((int)throttleValue - (int)lastThrottleValue) > DEADZONE) {
                ppmServoThrottle.writeMicroseconds(throttleValue);
                lastThrottleValue = throttleValue;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
