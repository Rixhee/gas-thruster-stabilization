#include "imu_functions.h"

const int thrusterFront = 12;
const int thrusterBack = 11;

// PID variables
float kp = 100;
float ki = 0.1;
float kd = 75;

float yaw = 0;
float pitch = 0;
float roll = 0;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

const unsigned long dt = 100;

unsigned long previous_time = 0;
int fixedCycles;

void setup() {
  Serial.begin(115200);
  setupIMU();
  pinMode(thrusterFront, OUTPUT);
  pinMode(thrusterBack, OUTPUT);
}

bool thrusterOn = false;
int cycleCounter = 0;
int threshold = 0;

uint8_t debounceThreshold = 50;
unsigned long lastStateChangeTime = 0;
unsigned long previousTime = 0;

void thrustControl(float correction) {
    if (abs(roll) > 0.02) {
      unsigned long current_time = millis();
        if (!thrusterOn) {
            // Set the number of cycles the thruster should stay on based on correction
            fixedCycles = max(int(abs(correction)), threshold); 
            cycleCounter = 0;
            digitalWrite(thrusterFront, LOW);
            digitalWrite(thrusterBack, LOW);
//            Serial.print("Fixed Cycles: ");
//            Serial.println(fixedCycles);
            thrusterOn = true; 
        }

        // Check if the thruster should stay on
        if (cycleCounter < fixedCycles) {
          if (current_time - lastStateChangeTime > debounceThreshold) {
            if (error > 0) {
                digitalWrite(thrusterFront, HIGH);
                digitalWrite(thrusterBack, LOW);
            } else if (error < 0) {
                digitalWrite(thrusterBack, HIGH);
                digitalWrite(thrusterFront, LOW);
            }
            cycleCounter++; 
            lastStateChangeTime = current_time;
          }
        } else {
            // Turn the thruster off after the fixed number of cycles
            digitalWrite(thrusterFront, LOW);
            digitalWrite(thrusterBack, LOW);
            thrusterOn = false; 
        }
    } else {
        // If roll is small, turn off thrusters
        digitalWrite(thrusterFront, LOW);
        digitalWrite(thrusterBack, LOW);
        thrusterOn = false;
    }

//    Serial.print("Cycle Counter: ");
//    Serial.println(cycleCounter);
}

void loop() {
  loopIMU();

  float* yprValues = getYPR();
  yaw = yprValues[0];
  pitch = yprValues[1];
  roll = yprValues[2];
  
  error = roll;
  integral += error;
  derivative = (error - previousError) / dt;

  // Calculate correction for the PID
  float correction = kp * error + ki * integral + kd * derivative;
  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Correction: ");
  Serial.println(correction);
  Serial.flush();

  // Control the thrusters
  thrustControl(correction);

  previousError = error;

  // Process Serial commands for tuning parameters
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
      } else if (selectedVariable == "threshold") {
        debounceThreshold = value;
        Serial.print("threshold: ");
        Serial.println(threshold);
      }
    }
  }
}
