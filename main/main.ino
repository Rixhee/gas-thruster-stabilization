#include "imu_functions.h"

const int thrusterFront = 11;
const int thrusterBack = 12;

// PID variables
float kp = 1500;
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
float fixed_correction;

void setup() {
  Serial.begin(115200);

  setupIMU();
  
  pinMode(thrusterFront, OUTPUT);
  pinMode(thrusterBack, OUTPUT);
}

unsigned long debounce_threshold = 50; // 50 ms debounce threshold
unsigned long last_state_change_time = 0;

void thrustControl(float correction) {
    if (abs(roll) > 0.03) {
        unsigned long current_time = millis();
        
        // Set initial time and fixed_correction if both thrusters are off
        if (digitalRead(thrusterFront) == LOW && digitalRead(thrusterBack) == LOW) {
            previous_time = current_time;
            fixed_correction = correction;
        }

        // Check if the correction period is ongoing
        if (current_time - previous_time < abs(fixed_correction)) {
            // Only change thruster state if debounce threshold is met
            if (current_time - last_state_change_time > debounce_threshold) {
                if (error > 0) {
                    digitalWrite(thrusterFront, HIGH);
                    digitalWrite(thrusterBack, LOW);
                } else {
                    digitalWrite(thrusterBack, HIGH);
                    digitalWrite(thrusterFront, LOW);
                }
                // Update the last state change time to apply debounce
                last_state_change_time = current_time;
            }
        } else {
            // If correction period has ended, turn off both thrusters
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

//  Serial.print("Yaw: ");
//  Serial.print(yaw);
//  Serial.print(", Pitch: ");
//  Serial.print(pitch);
//  Serial.print(", Roll: ");
//  Serial.println(roll);
  
  error = roll;
  integral += error;
  derivative = (error - previousError)/(dt);

  float correction = kp*error+ ki*integral + kd*derivative;

  thrustControl(correction);
  
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Correction: ");
  Serial.println(correction);

  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');

    if (input == "reset") {
      setupIMU();
    } else {
      int index_delimiter = input.indexOf(" ");
    String selectedVariable = input.substring(0, index_delimiter);
    float value = input.substring(index_delimiter).toFloat();

    if (selectedVariable == "kp") {
      kp = value;
      Serial.print("kp: ");
      Serial.println(kp);
    } else if (selectedVariable == "ki") {
      ki = value;
      Serial.print("ki: ");
      Serial.println(ki);
    } else if (selectedVariable == "kd") {
      kd = value;
      Serial.print("kd: ");
      Serial.println(kd);
      }
    }

    
  }
  previousError = error;

//  digitalWrite(thrusterFront, LOW);
  
//  digitalWrite(thrusterFront, HIGH);
//  digitalWrite(thrusterBack, HIGH);
//
//  delay(10);
//
//  digitalWrite(thrusterFront, LOW);
//  digitalWrite(thrusterBack, LOW);
//
//  delay(10);

//  digitalWrite(thrusterFront, LOW);
//  digitalWrite(thrusterBack, HIGH);
}
