#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_DEVICE_ID   0x68 
#define MPU6050_INT_ENABLE  0x38     ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS  0x3A     ///< Interrupt status register
#define MPU6050_ACCEL_OUT   0x3B     ///< base address for sensor data reads
#define MPU6050_PWR_MGMT_1  0x6B     ///< Primary power/sleep control register

struct RawIMUData {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
};

struct FilteredData{
    float kalAngleX;
    float kalAngleY;
};

struct IMU_Data {
    float ax, ay, az;
    float temp;
    float gx, gy, gz;
};

class IMU {
public:
    IMU(uint8_t address = MPU6050_DEVICE_ID, TwoWire& w = Wire);
    bool begin();
    bool readRaw(RawIMUData& data);
    bool IMUData(IMU_Data& data);
    bool KalmanData(FilteredData& fdata);
    void wakeup();
    void setGyroOffsets(int16_t gx_off, int16_t gy_off, int16_t gz_off);

private:
    uint8_t addr;
    TwoWire& wire;
    int16_t gx_offset = 0, gy_offset = 0, gz_offset = 0;

    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);
};

#endif