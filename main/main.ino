#include "imu_functions.h"

const int thrusterFront = 11;
const int thrusterBack = 12;

// PID variables
float kp = 2000;
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
unsigned long lastStateChangeTime = 0;
bool thrusterOn;

void thrustControl(float correction) {
    if (abs(roll) > 0.03) {
        unsigned long current_time = millis();
        
        if (!thrusterOn) {
            previous_time = current_time;
            fixed_correction = max(correction, 50);
            digitalWrite(thrusterFront, LOW);
            digitalWrite(thrusterBack, LOW);
        }

        if (current_time - previous_time < fixed_correction) {
            if (current_time - lastStateChangeTime > debounce_threshold) {
                thrusterOn = true;
                
                if (error > 0) {
                    digitalWrite(thrusterFront, HIGH);
                    digitalWrite(thrusterBack, LOW);
                } else if (error < 0) {
                    digitalWrite(thrusterBack, HIGH);
                    digitalWrite(thrusterFront, LOW);
                }
                
                lastStateChangeTime = current_time;
            }
        } else {
            digitalWrite(thrusterFront, LOW);
            digitalWrite(thrusterBack, LOW);
            thrusterOn = false;
        }
    } else {
        digitalWrite(thrusterFront, LOW);
        digitalWrite(thrusterBack, LOW);
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

  float correction = abs(kp*error+ ki*integral + kd*derivative);

  thrustControl(correction);
  
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Correction: ");
  Serial.println(correction);
  Serial.flush();

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
}
