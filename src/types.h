#ifndef TYPES_H_
#define TYPES_H_

#include <avr/io.h>


typedef struct {
    int16_t  accSmooth[3];
    int16_t  gyroData[3];
    int16_t  magADC[3];
    int16_t  gyroADC[3];
    int16_t  accADC[3];
} imu_t;

#endif


  