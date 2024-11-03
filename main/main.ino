#include "imu_functions.h"

const int thrusterFront = 12;
const int thrusterBack = 13;

// PID variables
float kp = 100;
float ki = 0.1;
float kd = 0.1;

float yaw = 0;
float pitch = 0;
float roll = 0;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

const unsigned long dt = 100;

unsigned long previous_time = 0;

void setup() {
  Serial.begin(9600);

  setupIMU();
  
  pinMode(thrusterFront, OUTPUT);
  pinMode(thrusterBack, OUTPUT);
}

//float calculateError(float pitch) {
//  if (pitch > 0) {
//    error = 1;
//  } else if (pitch < 0) {
//    error = -1;
//  } else {
//    error = 0;
//  }
//
//  return error;
//}

void thrustControl(float correction) {
  if (correction == 0 || pitch == 0) {
    digitalWrite(thrusterFront, LOW);
    digitalWrite(thrusterBack, LOW);
  } else {
    unsigned long current_time = millis();
    if (digitalRead(thrusterFront) == LOW && digitalRead(thrusterBack) == LOW) {
      previous_time = current_time;
    } 

    if (current_time - previous_time < abs(correction)) {
      if (error > 0) {
        digitalWrite(thrusterFront, HIGH);
        digitalWrite(thrusterBack, LOW);
      } else {
        digitalWrite(thrusterBack, HIGH);
        digitalWrite(thrusterFront, LOW);
      }
    } else {
      digitalWrite(thrusterFront, LOW);
      digitalWrite(thrusterBack, LOW);
    }
  }

}

void loop() {

  loopIMU();

  float* yprValues = getYPR();

  yaw = yprValues[0];
  pitch = yprValues[1];
  roll = yprValues[2];

  Serial.print("Yaw: ");
  Serial.print(yaw);
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Roll: ");
  Serial.println(roll);
  
  error = pitch;
  integral += error;
  derivative = (error - previousError)/(dt);

  float correction = kp*error + ki*integral + kd*derivative;

  thrustControl(correction);
  
//  Serial.print("Pitch: ");
//  Serial.print(pitch);
//  Serial.print(", Correction: ");
//  Serial.println(correction);
  
  previousError = error;

  delay(dt);
}
