#ifndef PPM_CONTROL_H
#define PPM_CONTROL_H

#include <Arduino.h>
#include <PPMReader.h>
#include <Servo.h>

void initPPMControl();
void ppmControlTask(void*);
extern volatile bool ppmControlEnabled;

#endif // PPM_CONTROL_H