#ifndef IMU_CONTROL_H
#define IMU_CONTROL_H

#include <Wire.h>
#include <MPU6050.h>
#include "thrust_control.h"

MPU6050 imu;
const int imuPin = A0, thruster1 = A1, thruster2 = A2;
float gyroWeight = 0.98, dt = 0.01;
float kp = 0.5, ki = 0.1, kd = 0.1;
float pitch = 0, accelPitch, gyroRate, error = 0, previousError = 0;
float integral = 0, derivative = 0;
unsigned long previous_time = 0;

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
    pitch = gyroWeight * (pitch + gyroRate / dt) + (1 - gyroWeight) * accelPitch;

    error = pitch / 90;
    integral += error;
    derivative = error - previousError;

    float correction = kp * error + ki * integral + kd * derivative;
    thrustControl(correction);

    previousError = error;
}

//  V---- uncomment if using ----V   ¯\_(ツ)_/¯ 

// float calculateError(float pitch) {
//     if (pitch > 0) {
//         error = 1;
//     } else if (pitch < 0) {
//         error = -1;
//     } else {
//         error = 0;
//     }
//     return error;
// }

#endif