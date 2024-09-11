#include "MotorPID.h"

// PIDController class constructor
PIDController::PIDController(float kp, float ki, float kd, float setpoint) {
  this->kp = kp;
  this->ki = ki;
  this->kd = kd;
  this->setpoint = setpoint;
  prevError = 0;
  integral = 0;
  lastTime = millis();
}

// Compute PID output
float PIDController::compute(float input) {
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - lastTime) / 1000.0;
  
  float error = setpoint - input;
  integral += error * deltaTime;
  float derivative = (error - prevError) / deltaTime;

  float output = kp * error + ki * integral + kd * derivative;

  prevError = error;
  lastTime = currentTime;

  return output;
}

void PIDController::setSetpoint(float newSetpoint) {
  setpoint = newSetpoint;
}

void PIDController::setGains(float newKp, float newKi, float newKd) {
  kp = newKp;
  ki = newKi;
  kd = newKd;
}

// MotorControl class constructor
MotorControl::MotorControl(int motorPin1, int motorPin2, int enablePin) {
  this->motorPin1 = motorPin1;
  this->motorPin2 = motorPin2;
  this->enablePin = enablePin;

  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  pinMode(enablePin, OUTPUT);
}

// Set motor speed using PWM
void MotorControl::setMotorSpeed(float speed) {
  int pwmValue = constrain(speed, 0, 255);
  analogWrite(enablePin, pwmValue);

  digitalWrite(motorPin1, HIGH);
  digitalWrite(motorPin2, LOW);  // For forward motion
}

// Exponential smoothing function
float smoothValue(float newValue, float previousValue, float alpha) {
  return alpha * newValue + (1 - alpha) * previousValue;
}
