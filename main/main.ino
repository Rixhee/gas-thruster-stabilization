#include "imu_functions.h"

// Define thruster pins
const int thrusterFront = 5;
const int thrusterBack = 2;
const int thrusterLeft = 16;
const int thrusterRight = 23;

// PID constants (shared for both axes)
float kp = 1;
float ki = 0.01;
float kd = 0.01;

// PID variables
float roll = 0;
float pitch = 0;
float errorFB = 0;
float previousErrorFB = 0;
float integralFB = 0;
float derivativeFB = 0;

float errorLR = 0;
float previousErrorLR = 0;
float integralLR = 0;
float derivativeLR = 0;

// Control parameters
float threshold = 0.1; 
unsigned long cycleDuration = 100;

// Duty cycles and timing
unsigned long lastCycleTimeFB = 0;
unsigned long lastCycleTimeLR = 0;
unsigned long onTimeFB = 0;
unsigned long onTimeLR = 0;

bool thrusterState = false;

float dutyCycleFB = 0;
float dutyCycleLR = 0;

// Setup function
void setup() {
  Serial.begin(115200);
  setupIMU();
  pinMode(thrusterFront, OUTPUT);
  pinMode(thrusterBack, OUTPUT);
  pinMode(thrusterLeft, OUTPUT);
  pinMode(thrusterRight, OUTPUT);
}

// Front-Back thruster control
void thrustControlFrontBack(float correction) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastCycleTimeFB;

    // Check if a new cycle should start
    if (elapsedTime >= cycleDuration) {
        lastCycleTimeFB = currentTime;
        dutyCycleFB = constrain(abs(correction), 0.0, 1.0); 
        onTimeFB = dutyCycleFB * cycleDuration;
    }

    // Control front-back thrusters based on correction
    if (elapsedTime < onTimeFB && abs(roll) > threshold) {
        if (correction > 0) {
            digitalWrite(thrusterFront, HIGH);
            digitalWrite(thrusterBack, LOW);
        } else if (correction < 0) {
            digitalWrite(thrusterBack, HIGH);
            digitalWrite(thrusterFront, LOW);
        }
        thrusterState = true;
    } else {
        digitalWrite(thrusterFront, LOW);
        digitalWrite(thrusterBack, LOW);
        thrusterState = false;
    }
}

// Left-Right thruster control
void thrustControlLeftRight(float correction) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastCycleTimeLR;

    // Check if a new cycle should start
    if (elapsedTime >= cycleDuration) {
        lastCycleTimeLR = currentTime;
        dutyCycleLR = constrain(abs(correction), 0.0, 1.0); 
        onTimeLR = dutyCycleLR * cycleDuration;
    }

    // Control left-right thrusters based on correction
    if (elapsedTime < onTimeLR && abs(pitch) > threshold) {
        if (correction > 0) {
            digitalWrite(thrusterLeft, HIGH);
            digitalWrite(thrusterRight, LOW);
        } else if (correction < 0) {
            digitalWrite(thrusterRight, HIGH);
            digitalWrite(thrusterLeft, LOW);
        }
        thrusterState = true;
    } else {
        digitalWrite(thrusterLeft, LOW);
        digitalWrite(thrusterRight, LOW);
        thrusterState = false;
    }
}

// Main loop
void loop() {
  loopIMU();

  float* yprValues = getYPR();
  float yaw = yprValues[0];
  roll = yprValues[1];
  pitch = yprValues[2];

  // Calculate PID error for roll (front-back thrusters)
  errorFB = roll;
  unsigned long currentTime = millis();
  unsigned long dtFB = currentTime - lastCycleTimeFB;
  integralFB += errorFB * dtFB;
  integralFB = constrain(integralFB, 0, 0.5);
  derivativeFB = (errorFB - previousErrorFB) / dtFB;

  // Calculate correction for front-back thrusters
  float correctionFrontBack = kp * errorFB + ki * integralFB + kd * derivativeFB;

  // Calculate PID error for pitch (left-right thrusters)
  errorLR = pitch;
  unsigned long dtLR = currentTime - lastCycleTimeLR;
  integralLR += errorLR * dtLR;
  integralLR = constrain(integralLR, 0, 0.5);
  derivativeLR = (errorLR - previousErrorLR) / dtLR;

  // Calculate correction for left-right thrusters
  float correctionLeftRight = kp * errorLR + ki * integralLR + kd * derivativeLR;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Front-Back Correction: ");
  Serial.print(correctionFrontBack);
  Serial.print(", Left-Right Correction: ");
  Serial.println(correctionLeftRight);

  // Use separate control functions for front-back and left-right thrusters
  thrustControlFrontBack(correctionFrontBack);
  thrustControlLeftRight(correctionLeftRight);

  previousErrorFB = errorFB;
  previousErrorLR = errorLR;

  Serial.print("Front-Back Duty Cycle: ");
  Serial.println(dutyCycleFB * 100);
  Serial.print("Left-Right Duty Cycle: ");
  Serial.println(dutyCycleLR * 100);

  // Process Serial input for tuning parameters
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');

    if (input == "reset") {
      setupIMU();
    }
    
    int index_delimiter = input.indexOf(" ");
    String variable = input.substring(0, index_delimiter);
    float value = input.substring(index_delimiter + 1).toFloat();

    if (variable == "kp") {
      kp = value;
      Serial.print("Updated kp: ");
      Serial.println(kp);
    } else if (variable == "ki") {
      ki = value;
      Serial.print("Updated ki: ");
      Serial.println(ki);
    } else if (variable == "kd") {
      kd = value;
      Serial.print("Updated kd: ");
      Serial.println(kd);
    } else if (variable == "cd") {
      cycleDuration = value;
      Serial.print("Updated Cycle Duration: ");
      Serial.println(cycleDuration);
    }
  }
}
