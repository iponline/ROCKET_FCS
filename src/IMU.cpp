#include <Wire.h>
#include <Kalman.h>
#include "IMU.h"

RawIMUData raw_imu;
IMU_Data imu_data;
//FilteredData fdata;


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

// void IMU::setGyroOffsets(int16_t gx_off, int16_t gy_off, int16_t gz_off) {
//     gx_offset = gx_off;
//     gy_offset = gy_off;
//     gz_offset = gz_off;
// }

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

bool IMU::IMUData(IMU_Data& data) {

    // Float Conversion
    data.ax = (raw_imu.ax - calib.ax_offset) / 16384.0f; // ±2g
    data.ay = (raw_imu.ay - calib.ay_offset) / 16384.0f;
    data.az = (raw_imu.az - calib.az_offset) / 16384.0f;
    data.gx = (raw_imu.gx - calib.gx_offset) / 131.0f;   // ±250dps
    data.gy = (raw_imu.gy - calib.gy_offset) / 131.0f;
    data.gz = (raw_imu.gz - calib.gz_offset) / 131.0f;
    data.temp = raw_imu.temp / 340.0f + 36.53f;
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

bool IMU::KalmanData(FilteredData& fdata, float dt){
    
    Kalman kalmanX;
    Kalman kalmanY;

    kalmanX.setQangle(0.01f);     // ตั้งค่าใหม่สำหรับ Q_angle
    kalmanX.setQbias(0.003f);     // ตั้งค่าใหม่สำหรับ Q_bias
    kalmanX.setRmeasure(0.1f);    // ตั้งค่าใหม่สำหรับ R_measure

    kalmanY.setQangle(0.01f);     // ตั้งค่าใหม่สำหรับ Q_angle
    kalmanY.setQbias(0.003f);     // ตั้งค่าใหม่สำหรับ Q_bias
    kalmanY.setRmeasure(0.1f);    // ตั้งค่าใหม่สำหรับ R_measure

    
    IMU imu;

    bool kalmanInitialized = false;
    float rad2deg = 57.29578f;
    

    imu.readRaw(raw_imu);
    imu.IMUData(imu_data);

    float accXangle = atan2(imu_data.ay, imu_data.az) * rad2deg;
    float accYangle = atan2(-imu_data.ax, sqrt(imu_data.ay*imu_data.ay + imu_data.az*imu_data.az)) * rad2deg;
    
    if (!kalmanInitialized) {
        kalmanX.setAngle(accXangle);
        kalmanY.setAngle(accYangle);
        kalmanInitialized = true;
    }

    fdata.kalAngleX = kalmanX.getAngle(accXangle, imu_data.gx, dt);
    fdata.kalAngleY = kalmanY.getAngle(accYangle, imu_data.gy, dt);

        //Serial.print("Kalman X: "); Serial.print(kalAngleX, 2);
        //Serial.print("  Kalman Y: "); Serial.println(kalAngleY, 2);

        //vTaskDelay(pdMS_TO_TICKS(20)); // 50 Hz
    

    return true;

}

bool IMU::enableDataReadyInterrupt() {

    // Open Data Ready interrupt (bit0) ที่ 0x38
    wire.beginTransmission(MPU6050_DEVICE_ID);
    wire.write(MPU6050_INT_ENABLE); // INT_ENABLE
    wire.write(0x01); // DATA_RDY_EN
    wire.endTransmission();

    // Optional: เปิด Active HIGH, push-pull, latch clear
    // wire.beginTransmission(addr);
    // wire.write(0x37); // INT_PIN_CFG
    // wire.write(0x00); // Default config
    // wire.endTransmission();
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

    // Serial.println("MPU6050 calibration complete.");
    // Serial.println("Calibration Results:");
    // Serial.print("Accel Offset: ");
    // Serial.print(cal.ax_offset, 2); Serial.print(", ");
    // Serial.print(cal.ay_offset, 2); Serial.print(", ");
    // Serial.println(cal.az_offset, 2);

    // Serial.print("Gyro Offset: ");
    // Serial.print(cal.gx_offset, 2); Serial.print(", ");
    // Serial.print(cal.gy_offset, 2); Serial.print(", ");
    // Serial.println(cal.gz_offset, 2);
    // delay(2000);

    return true;
}

void IMU::setCalibration(const IMUCalibration& cal) {
    calib = cal;
}




