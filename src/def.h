#ifndef DEF_H_
#define DEF_H_

// --- PPM and Servo Setup ---
#define PPM_INPUT_PIN 8
#define NUM_CHANNELS 6


// Last known values to avoid unnecessary updates
unsigned lastRollValue = 1500;
unsigned lastPitchValue = 1500;
unsigned lastThrottleValue = 1000;
const int DEADZONE = 3;

// Flag to activate/deactivate PPM reading
volatile bool ppmControlEnabled = false;





#endif