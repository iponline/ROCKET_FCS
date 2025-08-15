#include <Wire.h>
#include "IMU.h"
#include <math.h>
#include "Utilities.h"

// Helper: multiply matrix (a: m x n, b: n x p) → out: m x p
void matMult(double* a, double* b, double* out, int m, int n, int p) {
    for (int i=0;i<m;i++)
        for (int j=0;j<p;j++) {
            out[i*p+j]=0;
            for (int k=0;k<n;k++)
                out[i*p+j] += a[i*n+k]*b[k*p+j];
        }
}

// Helper: transpose
void matTrans(double* a, double* out, int m, int n) {
    for(int i=0;i<m;i++)
        for(int j=0;j<n;j++)
            out[j*m+i] = a[i*n+j];
}

// Helper: inverse 3x3 (basic, assume det≠0)
bool matInv3(double* in, double* out) {
    double det = in[0]*(in[4]*in[8]-in[5]*in[7]) - in[1]*(in[3]*in[8]-in[5]*in[6]) + in[2]*(in[3]*in[7]-in[4]*in[6]);
    if (fabs(det) < 1e-9) return false;
    out[0] = (in[4]*in[8]-in[5]*in[7])/det;
    out[1] = (in[2]*in[7]-in[1]*in[8])/det;
    out[2] = (in[1]*in[5]-in[2]*in[4])/det;
    out[3] = (in[5]*in[6]-in[3]*in[8])/det;
    out[4] = (in[0]*in[8]-in[2]*in[6])/det;
    out[5] = (in[2]*in[3]-in[0]*in[5])/det;
    out[6] = (in[3]*in[7]-in[4]*in[6])/det;
    out[7] = (in[1]*in[6]-in[0]*in[7])/det;
    out[8] = (in[0]*in[4]-in[1]*in[3])/det;
    return true;
}





IMU::IMU(uint8_t address, TwoWire& w) : addr(address), wire(w) {}

bool IMU::begin() {

    Wire.begin();
    delay(100);
    wakeup();

    // Set sample rate to 1000Hz
    Wire.beginTransmission(addr);
    Wire.write(0x19); // SMPLRT_DIV
    Wire.write(0);    // 1000 Hz
    Wire.endTransmission();

    // Set DLPF to allow 1000Hz output rate (e.g. config 3 = 42Hz bandwidth)
    Wire.beginTransmission(addr);
    Wire.write(0x1A); // CONFIG
    Wire.write(0x03); // DLPF_CFG = 3
    Wire.endTransmission();


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

// bool IMU::EKFData(FilteredData& fdata, double dt){
    
//     RawIMUData raw;
//     IMU_Data data;
//     readRaw(raw);
//     IMUData(data, raw);

//     double rad2deg = 57.29578;
//     double accXangle = atan2(data.ay, data.az) * rad2deg;
//     double accYangle = atan2(-data.ax, sqrt(data.ay * data.ay + data.az * data.az)) * rad2deg;

//     double EKFAngleX = EKF_filter(angleX, biasX, rateX, PX, data.gx, accXangle, dt);
//     double EKFAngleY = EKF_filter(angleY, biasY, rateY, PY, data.gy, accYangle, dt);

//     // ---------- Low Pass Filter 50Hz ----------
//     static double lpfX = 0.0;
//     static double lpfY = 0.0;
//     const float alpha = 0.7304f; // สำหรับ 50Hz ที่ 1kHz
 
//     lpfX = alpha * lpfX + (1.0 - alpha) * EKFAngleX;
//     lpfY = alpha * lpfY + (1.0 - alpha) * EKFAngleY;
 
//     //fdata.kalAngleX = lpfX;
//     //fdata.kalAngleY = lpfY;

//     // ---------- ส่งข้อมูลออกที่ 100Hz เท่านั้น ----------
//     static int sample_count = 0;
//     sample_count++;
//     if (sample_count >= 10) { // เรียก 10 รอบ = 10ms = 100Hz ที่ fs = 1000Hz
//         sample_count = 0;
//         fdata.kalAngleX = lpfX;
//         fdata.kalAngleY = lpfY;
//         return true; // อัปเดต fdata และแจ้งว่าส่งข้อมูล
//     } else {
//         return false; // ข้ามการอัปเดต fdata
//     }



//     // ใช้ Kalman_filter สำหรับแต่ละแกน
//     //fdata.kalAngleX = EKF_filter(angleX, biasX, rateX, PX, data.gx, accXangle, dt);
//    // fdata.kalAngleY = EKF_filter(angleY, biasY, rateY, PY, data.gy, accYangle, dt);

//     //static double lpfX = 0, lpfY = 0;
//     ///const float alpha = 0.7304f; // สำหรับ 50Hz ที่ 1kHz


//     return true;

// }

// double IMU::EKF_filter(double& angle, double& bias, double& rate, double P[2][2],
//     double newRate, double newAngle, double dt) {

//     // Process and measurement noise
//     const double Q_angle = 0.001, Q_bias = 0.003, R_measure = 0.03;

//     // State prediction
//     rate = newRate - bias;
//     angle += dt * rate;

//     // Jacobian of the state transition (F)
//     // F = [[1, -dt],
//     //      [0,  1]]
//     double F[2][2] = {
//         {1.0, -dt},
//         {0.0, 1.0}
//     };

//     // Process covariance prediction: P = F * P * F^T + Q
//     double P_pred[2][2];
//     P_pred[0][0] = F[0][0]*P[0][0] + F[0][1]*P[1][0];
//     P_pred[0][1] = F[0][0]*P[0][1] + F[0][1]*P[1][1];
//     P_pred[1][0] = F[1][0]*P[0][0] + F[1][1]*P[1][0];
//     P_pred[1][1] = F[1][0]*P[0][1] + F[1][1]*P[1][1];

//     double Ft[2][2] = {
//         {1.0, 0.0},
//         {-dt, 1.0}
//     };

//     P[0][0] = P_pred[0][0]*Ft[0][0] + P_pred[0][1]*Ft[1][0] + Q_angle;
//     P[0][1] = P_pred[0][0]*Ft[0][1] + P_pred[0][1]*Ft[1][1];
//     P[1][0] = P_pred[1][0]*Ft[0][0] + P_pred[1][1]*Ft[1][0];
//     P[1][1] = P_pred[1][0]*Ft[0][1] + P_pred[1][1]*Ft[1][1] + Q_bias;

//     // Measurement matrix H (Jacobian): H = [1, 0]
//     // Innovation
//     double y = newAngle - angle;

//     // Innovation covariance: S = H * P * H^T + R
//     double S = P[0][0] + R_measure;

//     // Kalman Gain: K = P * H^T / S
//     double K[2];
//     K[0] = P[0][0] / S;
//     K[1] = P[1][0] / S;

//     // State update
//     angle += K[0] * y;
//     bias  += K[1] * y;

//     // Covariance update: P = (I - K*H)*P
//     double P00_temp = P[0][0];
//     double P01_temp = P[0][1];
//     P[0][0] -= K[0] * P00_temp;
//     P[0][1] -= K[0] * P01_temp;
//     P[1][0] -= K[1] * P00_temp;
//     P[1][1] -= K[1] * P01_temp;

//     return angle;
// }


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

    //handleLED(20,0.01,true);

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

float IMU::lowPassFilter(float newValue, float prevValue, float alpha) {
    return alpha * prevValue + (1.0f - alpha) * newValue;
}

void IMU::EKFInitQ() {

    ekfQ.q = {1, 0, 0, 0};
    ekfQ.bg[0] = ekfQ.bg[1] = ekfQ.bg[2] = 0;
    memset(ekfQ.P, 0, sizeof(ekfQ.P));
    for (int i=0;i<7;i++) ekfQ.P[i][i] = 0.01;

}

void IMU::EKFPredictQ(EKFStateQ& ekf, double gx, double gy, double gz, double dt) {

    // Gyro - bias
    double wx = gx - ekf.bg[0];
    double wy = gy - ekf.bg[1];
    double wz = gz - ekf.bg[2];

    // Convert to rad/s if จำเป็น (หากยังเป็น deg/s)
    wx *= (M_PI/180.0);
    wy *= (M_PI/180.0);
    wz *= (M_PI/180.0);

    // Quaternion propagation (see earlier answer)
    double qw = ekf.q.w, qx = ekf.q.x, qy = ekf.q.y, qz = ekf.q.z;
    double dq_w = 0.5 * (-qx*wx - qy*wy - qz*wz);
    double dq_x = 0.5 * (qw*wx + qy*wz - qz*wy);
    double dq_y = 0.5 * (qw*wy - qx*wz + qz*wx);
    double dq_z = 0.5 * (qw*wz + qx*wy - qy*wx);

    ekf.q.w += dq_w * dt;
    ekf.q.x += dq_x * dt;
    ekf.q.y += dq_y * dt;
    ekf.q.z += dq_z * dt;

    // Normalize
    double norm = sqrt(ekf.q.w*ekf.q.w + ekf.q.x*ekf.q.x + ekf.q.y*ekf.q.y + ekf.q.z*ekf.q.z);
    ekf.q.w /= norm; ekf.q.x /= norm; ekf.q.y /= norm; ekf.q.z /= norm;

    // Bias assumed constant; process noise can be added if ต้องการ

    // Covariance update (approx. as Identity + Q)
    double Q[7][7] = {0};
    const double q_gyro = 1e-5, q_bias = 1e-7; // ปรับได้ตาม IMU
    for (int i=0; i<4; i++) Q[i][i] = q_gyro;
    for (int i=4; i<7; i++) Q[i][i] = q_bias;
    for(int i=0;i<7;++i)
        for(int j=0;j<7;++j)
            ekf.P[i][j] += Q[i][j];
}

void IMU::EKFUpdateQ(EKFStateQ& ekf, double ax, double ay, double az) {

    // Normalize accel
    double norm = sqrt(ax*ax + ay*ay + az*az);
    if (norm < 1e-6) return;
    ax /= norm; ay /= norm; az /= norm;

    // Predict gravity from quaternion (โลก = [0 0 1])
    double qw = ekf.q.w, qx = ekf.q.x, qy = ekf.q.y, qz = ekf.q.z;
    double g_b[3];
    g_b[0] = 2*(qx*qz - qw*qy);
    g_b[1] = 2*(qw*qx + qy*qz);
    g_b[2] = qw*qw - qx*qx - qy*qy + qz*qz;

    // Innovation (measurement residual): y = z - h(x)
    double y[3] = { ax - g_b[0], ay - g_b[1], az - g_b[2] };

    // -------- Jacobian H (3x7) --------
    double H[3][7] = {0};

    // Partial derivatives for g_b w.r.t. quaternion (see paper, here for scalar-first [w, x, y, z]):
    H[0][0] = -2*qy;          // ∂g_bx/∂qw
    H[0][1] =  2*qz;          // ∂g_bx/∂qx
    H[0][2] = -2*qw;          // ∂g_bx/∂qy
    H[0][3] =  2*qx;          // ∂g_bx/∂qz

    H[1][0] =  2*qx;
    H[1][1] =  2*qw;
    H[1][2] =  2*qz;
    H[1][3] =  2*qy;

    H[2][0] =  2*qw;
    H[2][1] = -2*qx;
    H[2][2] = -2*qy;
    H[2][3] =  2*qz;
    // w.r.t bias is zero (cols 4,5,6)

    // -------- Measurement noise (3x3) --------
    double R[3][3] = { {0.01,0,0}, {0,0.01,0}, {0,0,0.01} }; // Tune as needed

    // ---- S = H*P*H^T + R (3x3) ----
    double PHt[7][3] = {0}; // P*H^T
    for(int i=0;i<7;++i) for(int j=0;j<3;++j)
        for(int k=0;k<7;++k)
            PHt[i][j] += ekf.P[i][k]*H[j][k];

    double S[3][3] = {0};
    for(int i=0;i<3;++i) for(int j=0;j<3;++j)
        for(int k=0;k<7;++k)
            S[i][j] += H[i][k]*PHt[k][j];
    for(int i=0;i<3;++i) for(int j=0;j<3;++j)
        S[i][j] += R[i][j];

    // ---- Kalman Gain: K = P*H^T*inv(S) (7x3) ----
    double Sinv[3][3];
    if(!matInv3((double*)S, (double*)Sinv)) return; // Safety

    double K[7][3] = {0};
    for(int i=0;i<7;++i)
        for(int j=0;j<3;++j)
            for(int k=0;k<3;++k)
                K[i][j] += PHt[i][k]*Sinv[k][j];

    // ---- State update: x = x + K*y ----
    double dx[7] = {0};
    for(int i=0;i<7;++i)
        for(int j=0;j<3;++j)
            dx[i] += K[i][j]*y[j];

    // Quaternion update:  
    ekf.q.w += dx[0];
    ekf.q.x += dx[1];
    ekf.q.y += dx[2];
    ekf.q.z += dx[3];
    // Normalize
    double qnorm = sqrt(ekf.q.w*ekf.q.w + ekf.q.x*ekf.q.x + ekf.q.y*ekf.q.y + ekf.q.z*ekf.q.z);
    ekf.q.w /= qnorm; ekf.q.x /= qnorm; ekf.q.y /= qnorm; ekf.q.z /= qnorm;

    // Bias update:
    ekf.bg[0] += dx[4];
    ekf.bg[1] += dx[5];
    ekf.bg[2] += dx[6];

    // ---- Covariance update: P = (I - K*H)*P ----
    double KH[7][7] = {0};
    for(int i=0;i<7;++i)
        for(int j=0;j<7;++j)
            for(int k=0;k<3;++k)
                KH[i][j] += K[i][k]*H[k][j];

    for(int i=0;i<7;++i)
        for(int j=0;j<7;++j)
            ekf.P[i][j] -= KH[i][j]*ekf.P[i][j];
}


void IMU::quaternionToEuler(const Quaternion& q, double& roll, double& pitch, double& yaw) const {
    roll = atan2(2.0 * (q.w*q.x + q.y*q.z), 1.0 - 2.0 * (q.x*q.x + q.y*q.y));
    pitch = asin(2.0 * (q.w*q.y - q.z*q.x));
    yaw = atan2(2.0 * (q.w*q.z + q.x*q.y), 1.0 - 2.0 * (q.y*q.y + q.z*q.z));
}


bool IMU::EKFQuaternionData(FilteredData& fdata, double dt) {

    RawIMUData raw;
    IMU_Data data;
    readRaw(raw);
    IMUData(data, raw);

    EKFPredictQ(ekfQ, data.gx, data.gy, data.gz, dt);   // 1. Prediction
    EKFUpdateQ(ekfQ, data.ax, data.ay, data.az);        // 2. Update

    double roll, pitch, yaw;
    quaternionToEuler(ekfQ.q, roll, pitch, yaw);

    // ----------- LPF 50Hz (1000Hz loop) -----------
    static double lpfRoll = 0.0, lpfPitch = 0.0, lpfYaw = 0.0;
    const float alpha = 0.7304f;
    lpfRoll  = alpha * lpfRoll  + (1.0 - alpha) * roll;
    lpfPitch = alpha * lpfPitch + (1.0 - alpha) * pitch;
    lpfYaw   = alpha * lpfYaw   + (1.0 - alpha) * yaw;

    static int sample_count = 0;
    sample_count++;
    if (sample_count >= 10) {
        sample_count = 0;
        fdata.roll  = lpfRoll  * 57.29578;
        fdata.pitch = lpfPitch * 57.29578;
        fdata.yaw   = lpfYaw   * 57.29578;
        return true;
    } else {
        return false;
    }
}



