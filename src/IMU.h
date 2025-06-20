#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

#define MPU6050_DEVICE_ID   0x68 
#define MPU6050_INT_ENABLE  0x38     ///< Interrupt enable configuration register
#define MPU6050_INT_STATUS  0x3A     ///< Interrupt status register
#define MPU6050_ACCEL_OUT   0x3B     ///< base address for sensor data reads
#define MPU6050_PWR_MGMT_1  0x6B     ///< Primary power/sleep control register
#define MPU6050_INT_PIN     22


struct RawIMUData {
    int16_t ax, ay, az;
    int16_t temp;
    int16_t gx, gy, gz;
};

struct FilteredData{
    double kalAngleX;
    double kalAngleY;
};

struct LPFData{

    double lpfX;
    double lpfY;
    
};

struct IMU_Data {
    float ax, ay, az;
    float temp;
    float gx, gy, gz;
};

struct IMUCalibration {
    float ax_offset, ay_offset, az_offset;
    float gx_offset, gy_offset, gz_offset;
};

class IMU {
public:
    IMU(uint8_t address = MPU6050_DEVICE_ID, TwoWire& w = Wire);
    bool begin();
    bool readRaw(RawIMUData& data);
    bool IMUData(IMU_Data& data, const RawIMUData& raw);
    bool EKFData(FilteredData& fdata, double dt);
    void wakeup();
    bool enableDataReadyInterrupt();
    bool calibrate(IMUCalibration& cal, int samples = 500, int delay_ms = 2);
    void setCalibration(const IMUCalibration& cal);
    //double Kalman_filter(double angle, double gyroRate, double accelAngle, double dt);
    double EKF_filter(double& angle, double& bias, double& rate, double P[2][2],
        double newRate, double newAngle, double dt);
    float lowPassFilter(float newValue, float prevValue, float alpha);

private:

    uint8_t addr;
    TwoWire& wire;
    IMUCalibration calib = {0,0,0,0,0,0};
  
    bool readRegisters(uint8_t reg, uint8_t* buf, uint8_t len);

    // Kalman filter state (ต้องใช้ member variable ไม่ใช่ local variable)
    double angleX = 0, biasX = 0, rateX = 0;
    double PX[2][2] = {{0, 0}, {0, 0}};
    double angleY = 0, biasY = 0, rateY = 0;
    double PY[2][2] = {{0, 0}, {0, 0}};
};

#endif