#ifndef MotorPID_h
#define MotorPID_h

#include <Arduino.h>

class PIDController {
  public:
    PIDController(float kp, float ki, float kd, float setpoint);
    float compute(float input);
    void setSetpoint(float newSetpoint);
    void setGains(float newKp, float newKi, float newKd);
    
  private:
    float kp, ki, kd;
    float setpoint;
    float prevError, integral;
    unsigned long lastTime;
};

class MotorControl {
  public:
    MotorControl(int motorPin1, int motorPin2, int enablePin);
    void setMotorSpeed(float speed);
  private:
    int motorPin1, motorPin2, enablePin;
};

float smoothValue(float newValue, float previousValue, float alpha);

#endif