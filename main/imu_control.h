#ifndef IMU_CONTROL_H
#define IMU_CONTROL_H

#include <Wire.h>
#include <MPU6050.h>
#include "thrust_control.h"

// Initialize the MPU6050 object
MPU6050 imu;

// Thruster pins for four-thruster setup
const int thruster1 = A1, thruster2 = A2, thruster3 = A3, thruster4 = A4;

// PID constants and state variables for each axis
float kp = 0.5, ki = 0.1, kd = 0.1;
float pitchError = 0, rollError = 0, yawError = 0;
float previousPitchError = 0, previousRollError = 0, previousYawError = 0;
float pitchIntegral = 0, rollIntegral = 0, yawIntegral = 0;
float pitchCorrection = 0, rollCorrection = 0, yawCorrection = 0;

void setupIMU() {
    Serial.begin(9600);
    Wire.begin();
    imu.initialize();

    if (imu.testConnection()) Serial.println("MPU6050 connection successful");
    else Serial.println("MPU6050 connection failed");

    pinMode(thruster1, OUTPUT);
    pinMode(thruster2, OUTPUT);
    pinMode(thruster3, OUTPUT);
    pinMode(thruster4, OUTPUT);
}

// Simulate IMU readings for pitch, roll, and yaw
void simulateIMUInput(float simPitch, float simRoll, float simYaw) {
    // Pitch control
    pitchError = simPitch / 90;
    pitchIntegral += pitchError;
    pitchCorrection = kp * pitchError + ki * pitchIntegral + kd * (pitchError - previousPitchError);
    previousPitchError = pitchError;

    // Roll control
    rollError = simRoll / 90;
    rollIntegral += rollError;
    rollCorrection = kp * rollError + ki * rollIntegral + kd * (rollError - previousRollError);
    previousRollError = rollError;

    // Yaw control
    yawError = simYaw / 90;
    yawIntegral += yawError;
    yawCorrection = kp * yawError + ki * yawIntegral + kd * (yawError - previousYawError);
    previousYawError = yawError;

    // Apply corrections through thrust control
    thrustControl(pitchCorrection, rollCorrection, yawCorrection);
}

#endif