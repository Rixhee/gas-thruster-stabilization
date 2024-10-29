#include <Wire.h>

const int imuPin = ;
const int thruster1 = ;
const int thruster2 = ;

// PID variables
float kp = 0;
float ki = 0;
float kd = 0;

float imu_sense = 0;
float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

unsigned long previous_time = 0

void setup() {
  Serial.begin(9600);
  pinMode(thruster1, OUTPUT);
  pinMode(thruster2, OUTPUT);
}

float calculateError() {
  if (imu_sense > 0) {
    error = 1;
  } else if (imu_sense < 0) {
    error = -1;
  } else {
    error = 0;
  }

  return error;
}

void loop() {
  error = calculateError();
  integral += error;
  derivative = error - previousError;

  float correction = kp*error + ki*integral + kd*derivative;

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

  previousError = error;
}
