#ifndef IMU_CONTROL_H
#define IMU_CONTROL_H

#include <Wire.h>
#include <MPU6050.h>
#include "thrust_control.h"

// Variable and constant declarations
extern MPU6050 imu;
extern const int imuPin, thruster1, thruster2;
extern float gyroWeight, dt, kp, ki, kd;
extern float pitch, accelPitch, gyroRate, error, previousError, integral, derivative;
extern unsigned long previous_time;

// Function prototypes
void setupIMU() {
  Serial.begin(9600);
  Wire.begin();
  imu.initialize();

  if (imu.testConnection()) Serial.println("MPU6050 connection successful");
  else Serial.println("MPU6050 connection failed");

  pinMode(thruster1, OUTPUT);
  pinMode(thruster2, OUTPUT);
}

void updateIMU() {
  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accelPitch = atan2(ay, az) * 180 / PI;
  gyroRate = gx / 131.0;
  pitch = gyroWeight * (pitch + gyroRate * dt) + (1 - gyroWeight) * accelPitch;

  error = pitch / 90;
  integral += error;
  derivative = error - previousError;

  float correction = kp * error + ki * integral + kd * derivative;
  thrustControl(correction);

  previousError = error;
}

#endif