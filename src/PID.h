#ifndef PID_H
#define PID_H

class PID {

public:

    PID(float kp = 0.0, float ki = 0.0, float kd = 0.0);

    void setTunings(float kp, float ki, float kd);
    void setLimits(float minOut, float maxOut);
    void getTunings(float &kp, float &ki, float &kd) const;
    void reset();

    // คำนวณ PID, dt เป็นวินาที (เช่น 0.02 = 20ms)
    float compute(float setpoint, float measured, float dt);

private:
    float _kp, _ki, _kd;
    float _integral;
    float _prevError;
    float _outMin, _outMax;
};

#endif
