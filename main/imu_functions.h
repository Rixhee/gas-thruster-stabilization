#ifndef imu_functions_h
#define imu_functions_h

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

void setupIMU();
void loopIMU();
float* getYPR();

#endif