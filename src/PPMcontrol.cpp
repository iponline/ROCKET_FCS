#include "Arduino.h"
#include "PPMcontrol.h"
#include "Servo.h"
#include "arduino_freertos.h"

//#include "freertos/task.h"

#define PPM_INPUT_PIN 8
#define NUM_CHANNELS 6

PPMReader ppm(PPM_INPUT_PIN, NUM_CHANNELS);
Servo ppmServoRoll, ppmServoPitch, ppmServoThrottle;

volatile bool ppmControlEnabled = true;

unsigned lastRollValue = 1500;
unsigned lastPitchValue = 1500;
unsigned lastThrottleValue = 1000;
const int DEADZONE = 3;

const int PPM_MIN = 1000;
const int PPM_MAX = 2000;

// Adjust these based on how far your servo can rotate safely
//const int SERVO_MIN = 300;
const int SERVO_MIN = 600;
const int SERVO_MAX = 2200;
//const int SERVO_MAX = 2300;

const int ROLL_CENTER_OFFSET     = 0;   // microseconds to add to roll center
const int PITCH_CENTER_OFFSET    = 0;    // subtract to shift pitch center backward
const int THROTTLE_CENTER_OFFSET = 0;      // no offset

#ifndef constrain
#define constrain(amt, low, high) ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))
#endif


//volatile bool ppmControlEnabled = false;

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

                //int rollAngle  =  map(constrain(rollValue, 1000, 2000), 1000, 2000, 0, 180);
                int rollPWM = map(constrain(rollValue, PPM_MIN, PPM_MAX), PPM_MIN, PPM_MAX, SERVO_MIN, SERVO_MAX) + ROLL_CENTER_OFFSET;
                ppmServoRoll.writeMicroseconds(rollPWM);

                Serial.print("Roll: ");
                Serial.println(rollPWM);

                //ppmServoRoll.write(rollAngle);
                lastRollValue = rollValue;
            }
            if (abs((int)pitchValue - (int)lastPitchValue) > DEADZONE) {

                //int pitchAngle    = map(constrain(pitchValue, 1000, 2000), 1000, 2000, 0, 180);
                int pitchPWM = map(constrain(pitchValue, PPM_MIN, PPM_MAX), PPM_MIN, PPM_MAX, SERVO_MIN, SERVO_MAX) + PITCH_CENTER_OFFSET;
                ppmServoPitch.writeMicroseconds(pitchPWM);
                 
                Serial.print("Pitch: ");
                Serial.println(pitchPWM);
                //ppmServoPitch.writeMicroseconds(pitchAngle);
                lastPitchValue = pitchValue;
            }
            if (abs((int)throttleValue - (int)lastThrottleValue) > DEADZONE) {
                //int throttleAngle = map(constrain(throttleValue, 1000, 2000), 1000, 2000, 0, 180);
                int throttlePWM = map(constrain(throttleValue, PPM_MIN, PPM_MAX), PPM_MIN, PPM_MAX, SERVO_MIN, SERVO_MAX) + THROTTLE_CENTER_OFFSET;
                ppmServoThrottle.writeMicroseconds(throttlePWM);
                Serial.print("Throttle: ");
                Serial.println(throttlePWM);
                //ppmServoThrottle.writeMicroseconds(throttleValue);
                lastThrottleValue = throttleValue;
            }

           
            
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }

}
