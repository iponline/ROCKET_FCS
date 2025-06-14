#include <Wire.h>
#include "IMU.h"

IMU::IMU(uint8_t address, TwoWire& w) : addr(address), wire(w) {}

bool IMU::begin() {

    Wire.begin();
    delay(100);
    wakeup();
    return true;
}

void IMU::wakeup() {

    // Write 0 to power management register (0x6B) to wake up MPU6050
    Wire.beginTransmission(addr);
    Wire.write(MPU6050_PWR_MGMT_1);
    Wire.write(0);
    Wire.endTransmission();

}

bool IMU::readRaw(RawIMUData& data) {

    uint8_t buf[14];
    if (!readRegisters(MPU6050_ACCEL_OUT, buf, 14)) return false;
    data.ax = (int16_t)(buf[0] << 8 | buf[1]);
    data.ay = (int16_t)(buf[2] << 8 | buf[3]);
    data.az = (int16_t)(buf[4] << 8 | buf[5]);
    data.temp = (int16_t)(buf[6] << 8 | buf[7]);
    data.gx = ((int16_t)(buf[8]  << 8 | buf[9])) ;
    data.gy = ((int16_t)(buf[10] << 8 | buf[11])) ;
    data.gz = ((int16_t)(buf[12] << 8 | buf[13])) ;
    return true;

}

bool IMU::IMUData(IMU_Data& data, const RawIMUData& raw) {

    data.ax = (raw.ax - calib.ax_offset) / 16384.0f; // ±2g
    data.ay = (raw.ay - calib.ay_offset) / 16384.0f;
    data.az = (raw.az - calib.az_offset) / 16384.0f;
    data.gx = (raw.gx - calib.gx_offset) / 131.0f;   // ±250dps
    data.gy = (raw.gy - calib.gy_offset) / 131.0f;
    data.gz = (raw.gz - calib.gz_offset) / 131.0f;
    data.temp = raw.temp / 340.0f + 36.53f;
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

bool IMU::KalmanData(FilteredData& fdata, double dt){
    
    RawIMUData raw;
    IMU_Data data;
    readRaw(raw);
    IMUData(data, raw);

    double rad2deg = 57.29578;
    double accXangle = atan2(data.ay, data.az) * rad2deg;
    double accYangle = atan2(-data.ax, sqrt(data.ay * data.ay + data.az * data.az)) * rad2deg;

    // ใช้ Kalman_filter สำหรับแต่ละแกน
    fdata.kalAngleX = Kalman_filter(angleX, biasX, rateX, PX, data.gx, accXangle, dt);
    fdata.kalAngleY = Kalman_filter(angleY, biasY, rateY, PY, data.gy, accYangle, dt);

    return true;

}

double IMU::Kalman_filter(double& angle, double& bias, double& rate, double P[2][2],
    double newRate, double newAngle, double dt) {

    // กำหนดค่า Gain ตามต้องการ
    double Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;

    // Predict
    rate = newRate - bias;
    angle += dt * rate;

    P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Update
    double S = P[0][0] + R_measure;
    double K[2];
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    double y = newAngle - angle;
    angle += K[0] * y;
    bias += K[1] * y;

    double P00_temp = P[0][0], P01_temp = P[0][1];
    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}


bool IMU::enableDataReadyInterrupt() {

    // Open Data Ready interrupt (bit0) ที่ 0x38
    wire.beginTransmission(MPU6050_DEVICE_ID);
    wire.write(MPU6050_INT_ENABLE); // INT_ENABLE
    wire.write(0x01); // DATA_RDY_EN
    wire.endTransmission();

    return true;

}

bool IMU::calibrate(IMUCalibration& cal, int samples, int delay_ms) {
    
    int32_t ax_sum = 0, ay_sum = 0, az_sum = 0;
    int32_t gx_sum = 0, gy_sum = 0, gz_sum = 0;
    RawIMUData raw;

    Serial.println("Calibrating MPU6050... (Keep still!)");
    for (int i = 0; i < samples; ++i) {
        if (readRaw(raw)) {
            ax_sum += raw.ax;
            ay_sum += raw.ay;
            az_sum += raw.az;
            gx_sum += raw.gx;
            gy_sum += raw.gy;
            gz_sum += raw.gz;
        }
        delay(delay_ms);
    }
    cal.ax_offset = ax_sum / (float)samples;
    cal.ay_offset = ay_sum / (float)samples;
    cal.az_offset = (az_sum / (float)samples) - 16384.0f; // ต้องลบ gravity ด้วย
    cal.gx_offset = gx_sum / (float)samples;
    cal.gy_offset = gy_sum / (float)samples;
    cal.gz_offset = gz_sum / (float)samples;

    setCalibration(cal);

    return true;
}

void IMU::setCalibration(const IMUCalibration& cal) {
    calib = cal;
}




