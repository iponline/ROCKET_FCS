#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_DEVICE_ID 0x68 

struct RawIMUData {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
};

class IMU {
public:
    IMU(uint8_t address = 0x68, TwoWire& w = Wire);
    bool begin();
    bool readRaw(RawIMUData& data);
    void wakeup();
    void setGyroOffsets(int16_t gx_off, int16_t gy_off, int16_t gz_off);

private:
    uint8_t addr;
    TwoWire& wire;
    int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
};

#endif