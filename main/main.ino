#include <Wire.h>
#include <MPU6050.h>

MPU6050 imu;

const int imuPin = A0;
const int thruster1 = A1;
const int thruster2 = A2;

float gyroWeight = 0.98;
float dt = 0.01;

// PID variables
float kp = 0.5;
float ki = 0.1;
float kd = 0.1;
s
float pitch = 0;
float accelPitch;
float gyroRate;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

unsigned long previous_time = 0;

void setup() {
  Serial.begin(9600);

  Wire.begin();
  imu.initialize(); 

  if (imu.testConnection()) {
        Serial.println("MPU6050 connection successful");
    } else {
        Serial.println("MPU6050 connection failed");
    }
  
  pinMode(thruster1, OUTPUT);
  pinMode(thruster2, OUTPUT);
}

float calculateError(float pitch) {
  if (pitch > 0) {
    error = 1;
  } else if (pitch < 0) {
    error = -1;
  } else {
    error = 0;
  }

  return error;
}

void thrustControl(float correction) {
  if (correction == 0) {
    digitalWrite(thruster1, LOW);
    digitalWrite(thruster2, LOW);
  } else {
    unsigned long current_time = millis();
    if (digitalRead(thruster1) == LOW && digitalRead(thruster2) == LOW) {
      previous_time = current_time;
    } 

    if (current_time - previous_time < abs(correction)) {
      if (error < 0) {
        digitalWrite(thruster1, HIGH);
        digitalWrite(thruster2, LOW);
      } else {
        digitalWrite(thruster2, HIGH);
        digitalWrite(thruster1, LOW);
      }
    } else {
      digitalWrite(thruster1, LOW);
      digitalWrite(thruster2, LOW);
    }
  }

}

void loop() {

  int16_t ax, ay, az, gx, gy, gz;
  imu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  accelPitch = atan2(ay, az) * 180/PI;
  gyroRate = gx/131.0;

  pitch = gyroWeight*(pitch + gyroRate/dt) + (1-gyroWeight)*accelPitch;
  
  error = pitch/90;
  integral += error;
  derivative = error - previousError;

  float correction = kp*error + ki*integral + kd*derivative;

  thrustControl(correction);
  
  previousError = error;
}
