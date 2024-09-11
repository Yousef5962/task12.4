#include <MotorPID.h>

// Create a PID controller object
PIDController motorPID(1.0, 0.1, 0.01, 100.0);

// Create a MotorControl object
MotorControl motor(9, 10, 11);

float smoothedOutput = 0;
float alpha = 0.9;  // Smoothing factor

void setup() {
  Serial.begin(9600);
}

void loop() {
  // Simulate some input (RPM)
  float input = analogRead(A0) / 4.0;

  // Compute PID output
  float output = motorPID.compute(input);
  
  // Apply exponential smoothing for soft start
  smoothedOutput = smoothValue(output, smoothedOutput, alpha);

  // Set motor speed with smoothed output
  motor.setMotorSpeed(smoothedOutput);

  delay(100);
}
