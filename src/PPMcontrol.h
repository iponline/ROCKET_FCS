#ifndef PPM_CONTROL_H
#define PPM_CONTROL_H

#include <Arduino.h>

// Hardware mapping
#define PPM_INPUT_PIN 8
#define NUM_CHANNELS  6

// Initialize PPM reader hardware (non-blocking)
void ppmInit();

// Read latest valid values for ch1..ch3 (roll, pitch, throttle), in microseconds.
// If no fresh frame yet, returns last known (or sane defaults: 1500/1500/1000).
void ppmRead3(int& ch1_us, int& ch2_us, int& ch3_us);

#endif // PPM_CONTROL_H
