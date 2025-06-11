#include <Wire.h>
#include <Kalman.h>
#include "IMU.h"

RawIMUData raw_imu;
IMU_Data imu_data;



IMU::IMU(uint8_t address, TwoWire& w) : addr(address), wire(w) {}

bool IMU::begin() {
    Wire.begin();
    delay(100);
    // Wake up sensor
    wakeup();
    // Optional: configure accelerometer and gyro full scale ranges (see TKJ example)
    // Default settings are fine for most hobby use
    return true;
}

void IMU::wakeup() {
    // Write 0 to power management register (0x6B) to wake up MPU6050
    Wire.beginTransmission(addr);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0);
    Wire.endTransmission();
}

void IMU::setGyroOffsets(int16_t gx_off, int16_t gy_off, int16_t gz_off) {
    gx_offset = gx_off;
    gy_offset = gy_off;
    gz_offset = gz_off;
}

bool IMU::readRaw(RawIMUData& data) {
    uint8_t buf[14];
    if (!readRegisters(MPU6050_ACCEL_OUT, buf, 14)) return false;
    data.ax = (int16_t)(buf[0] << 8 | buf[1]);
    data.ay = (int16_t)(buf[2] << 8 | buf[3]);
    data.az = (int16_t)(buf[4] << 8 | buf[5]);
    data.temp = (int16_t)(buf[6] << 8 | buf[7]);
    data.gx = ((int16_t)(buf[8]  << 8 | buf[9]))  - gx_offset;
    data.gy = ((int16_t)(buf[10] << 8 | buf[11])) - gy_offset;
    data.gz = ((int16_t)(buf[12] << 8 | buf[13])) - gz_offset;
    return true;
}

bool IMU::IMUData(IMU_Data& data) {

    // Float Conversion
    imu_data.ax = raw_imu.ax / 16384.0f; // ±2g
    imu_data.ay = raw_imu.ay / 16384.0f;
    imu_data.az = raw_imu.az / 16384.0f;
    imu_data.gx = raw_imu.gx / 131.0f;   // ±250dps
    imu_data.gy = raw_imu.gy / 131.0f;
    imu_data.gz = raw_imu.gz / 131.0f;
    imu_data.temp = raw_imu.temp / 340.0f + 36.53f;
    return true;

}

bool IMU::readRegisters(uint8_t reg, uint8_t* buf, uint8_t len) {
    Wire.beginTransmission(addr);
    Wire.write(reg);
    if (wire.endTransmission(false) != 0) return false;
    Wire.requestFrom(addr, len, true);
    for (uint8_t i = 0; i < len; i++) {
        if (wire.available()) buf[i] = wire.read();
        else return false;
    }
    return true;
}

bool IMU::KalmanData(FilteredData& fdata){
    
    Kalman kalmanX;
    Kalman kalmanY;
    
    IMU imu;

    bool kalmanInitialized = false;
    float rad2deg = 57.29578f;
    uint32_t lastTick = millis();

    imu.readRaw(raw_imu);
    imu.IMUData(imu_data);

    float accXangle = atan2(imu_data.ay, imu_data.az) * rad2deg;
    float accYangle = atan2(-imu_data.ax, sqrt(imu_data.ay*imu_data.ay + imu_data.az*imu_data.az)) * rad2deg;
    
    if (!kalmanInitialized) {
        kalmanX.setAngle(accXangle);
        kalmanY.setAngle(accYangle);
        kalmanInitialized = true;
    }




    return true;

}



