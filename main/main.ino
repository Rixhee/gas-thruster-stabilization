#include <Wire.h>

const int imuPin = ;
const int thruster1 = ;
const int thruster2 = ;

// PID variables
float kp = 0;
float ki = 0;
float kd = 0;

float imu_sense = 0;

float previous_time;

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
  } else if {
    if (digitalRead(thruster1) == LOW and digitalRead(thruster2) == LOW) {
      previous_time = millis();
    } else if (millis() - previous_time < correction) {
      if (error < 0) {
        digitalWrite(thruster1, HIGH);
      } else {
        digitalWrite(thruster2, HIGH);
      }
    }
  }
}
