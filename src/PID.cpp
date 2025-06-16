#include "PID.h"

PID::PID(float kp, float ki, float kd)
    : _kp(kp), _ki(ki), _kd(kd), _integral(0), _prevError(0), _outMin(-10000), _outMax(10000) {}

void PID::setTunings(float kp, float ki, float kd) {
    _kp = kp; _ki = ki; _kd = kd;
}

void PID::setLimits(float minOut, float maxOut) {
    _outMin = minOut; _outMax = maxOut;
}

void PID::reset() {
    _integral = 0;
    _prevError = 0;
}

float PID::compute(float setpoint, float measured, float dt) {
    
    float error = setpoint - measured;
    _integral += error * dt;
    float derivative = (dt > 0) ? (error - _prevError) / dt : 0;

    float output = _kp * error + _ki * _integral + _kd * derivative;
    if (output > _outMax) output = _outMax;
    if (output < _outMin) output = _outMin;

    _prevError = error;
    return output;
}

void PID::getTunings(float &kp, float &ki, float &kd) const {

    kp = _kp;
    ki = _ki;
    kd = _kd;

}
