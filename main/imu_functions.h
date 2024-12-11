#ifndef imu_functions_h
#define imu_functions_h

#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

MPU6050 mpu;

#define INTERRUPT_PIN 2

bool dmpReady = false;
uint8_t mpuIntStatus;
uint8_t devStatus;
uint8_t fifoBuffer[64];
float ypr[3];
Quaternion q;
VectorFloat gravity;

// Angular velocity variables
float angularVelocity[3] = {0.0, 0.0, 0.0};
float previousGyro[3] = {0.0, 0.0, 0.0};
unsigned long previousMillis = 0;

volatile bool mpuInterrupt = false;
void dmpDataReady() { mpuInterrupt = true; }

void setupIMU() {
  Wire.begin();
  Wire.setClock(400000);

  Serial.begin(115200);
  while (!Serial) Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful")
                                      : F("MPU6050 connection failed"));

  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);

  if (devStatus == 0) {
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

  } else {
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}

void loopIMU() {
  if (!dmpReady) return;

  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity); 
    
    // Get current gyroscope readings (angular velocity)
    int16_t gx, gy, gz;
    mpu.getRotation(&gx, &gy, &gz);

    // Convert the raw gyro data to degrees per second (or radians if needed)
    float currentGyro[3] = {
      gx / 131.0, 
      gy / 131.0,
      gz / 131.0
    };

    // Calculate the angular velocity (change in angle over time)
    unsigned long currentMillis = millis();
    unsigned long deltaTime = currentMillis - previousMillis;

    if (deltaTime > 0) {
      angularVelocity[0] = (currentGyro[0] - previousGyro[0]) / deltaTime * 1000;
      angularVelocity[1] = (currentGyro[1] - previousGyro[1]) / deltaTime * 1000;
      angularVelocity[2] = (currentGyro[2] - previousGyro[2]) / deltaTime * 1000;
    }

    // Store the current gyro values for the next loop iteration
    previousGyro[0] = currentGyro[0];
    previousGyro[1] = currentGyro[1];
    previousGyro[2] = currentGyro[2];

    previousMillis = currentMillis;
  }
}

float* getYPR() {
  return ypr;
}

float* getAngularVelocity() {
  return angularVelocity;
}

#endif