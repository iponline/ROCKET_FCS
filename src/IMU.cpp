#include <Wire.h>
#include <Kalman.h>
#include "IMU.h"

IMU::IMU(uint8_t address, TwoWire& w) : addr(address), wire(w) {}

bool IMU::begin() {
    wire.begin();
    delay(100);
    // Wake up sensor
    wakeup();
    // Optional: configure accelerometer and gyro full scale ranges (see TKJ example)
    // Default settings are fine for most hobby use
    return true;
}

void IMU::wakeup() {
    // Write 0 to power management register (0x6B) to wake up MPU6050
    wire.beginTransmission(addr);
    wire.write(0x6B);
    wire.write(0);
    wire.endTransmission();
}

void IMU::setGyroOffsets(int16_t gx_off, int16_t gy_off, int16_t gz_off) {
    gx_offset = gx_off;
    gy_offset = gy_off;
    gz_offset = gz_off;
}

bool IMU::readRaw(RawIMUData& data) {
    uint8_t buf[14];
    if (!readRegisters(0x3B, buf, 14)) return false;
    data.ax = (int16_t)(buf[0] << 8 | buf[1]);
    data.ay = (int16_t)(buf[2] << 8 | buf[3]);
    data.az = (int16_t)(buf[4] << 8 | buf[5]);
    data.temp = (int16_t)(buf[6] << 8 | buf[7]);
    data.gx = ((int16_t)(buf[8]  << 8 | buf[9]))  - gx_offset;
    data.gy = ((int16_t)(buf[10] << 8 | buf[11])) - gy_offset;
    data.gz = ((int16_t)(buf[12] << 8 | buf[13])) - gz_offset;
    return true;
}

bool IMU::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    wire.beginTransmission(addr);
    wire.write(reg);
    if (wire.endTransmission(false) != 0) return false;
    wire.requestFrom(addr, len, true);
    for (uint8_t i = 0; i < len; i++) {
        if (wire.available()) buf[i] = wire.read();
        else return false;
    }
    return true;
}



