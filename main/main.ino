#include "imu_functions.h"

const int thrusterFront = 5;
const int thrusterBack = 2;
const int thrusterLeft = 16;
const int thrusterRight = 23;

float kp = 0.4;
float ki = 0.01;
float kd = 0.5;

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

float threshold = 0.02; 
unsigned long cycleDuration = 100;

float angVelX;
float angVelY;
float angVelZ;

unsigned long lastCycleTimeFB = 0;
unsigned long lastCycleTimeLR = 0;
unsigned long onTimeFB = 0;
unsigned long onTimeLR = 0;

bool thrusterState = false;

float dutyCycleFB = 0;
float dutyCycleLR = 0;

unsigned long lastCounterMotionTime = 0;
unsigned long counterMotionDuration = 10; 

// Setup function
void setup() {
  Serial.begin(115200);
  setupIMU();
  pinMode(thrusterFront, OUTPUT);
  pinMode(thrusterBack, OUTPUT);
  pinMode(thrusterLeft, OUTPUT);
  pinMode(thrusterRight, OUTPUT);
}

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
//            digitalWrite(thrusterBack, LOW);
        } else if (correction < 0) {
            digitalWrite(thrusterBack, HIGH);
//            digitalWrite(thrusterFront, LOW);
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
//            digitalWrite(thrusterRight, LOW);
        } else if (correction < 0) {
            digitalWrite(thrusterRight, HIGH);
//            digitalWrite(thrusterLeft, LOW);
        }
        thrusterState = true;
    } else {
        digitalWrite(thrusterLeft, LOW);
        digitalWrite(thrusterRight, LOW);
        thrusterState = false;
    }
}

// Counter-motion correction based on gyro rates
void applyCounterMotion(float gyroX, float gyroY) {
    unsigned long currentTime = millis();
    bool counterMotionApplied = false;

    // Check if enough time has passed since the last counter-motion
    if (currentTime - lastCounterMotionTime >= counterMotionDuration) {

        // Apply counter-motion only if the angular velocity is reducing (indicating movement towards stabilized position)
        if (angVelX > 40 && roll > 0.5) {
            digitalWrite(thrusterBack, HIGH); 
            lastCounterMotionTime = currentTime;
            counterMotionApplied = true;
        } else if (angVelX < -40 && roll < -0.5) {
            digitalWrite(thrusterFront, HIGH);
            lastCounterMotionTime = currentTime;
            counterMotionApplied = true;
        }
        
        if (angVelY > 40 && pitch > 0.5) { 
            digitalWrite(thrusterRight, HIGH); 
            lastCounterMotionTime = currentTime;
            counterMotionApplied = true;
        } else if (angVelY < -40 && pitch < -0.5) {
            digitalWrite(thrusterLeft, HIGH);
            lastCounterMotionTime = currentTime;
            counterMotionApplied = true;
        }
    }

    // Turn off counter-motion after the duration (10 ms)
    if (counterMotionApplied) {
        unsigned long counterMotionElapsedTime = currentTime - lastCounterMotionTime;
        if (counterMotionElapsedTime >= counterMotionDuration) {
            digitalWrite(thrusterBack, LOW); 
            digitalWrite(thrusterRight, LOW); 
        }
    }
}

// Main loop
void loop() {
  loopIMU();

  float* yprValues = getYPR();
  float yaw = yprValues[0];
  roll = yprValues[1];
  pitch = yprValues[2];

  float* angXYZ = getAngularVelocity();
  angVelX = angXYZ[0];
  angVelY = angXYZ[1];
  angVelZ = angXYZ[2];

//  Serial.print("AngVel X: ");
//  Serial.print(angVelX);
//  Serial.print("AngVel Y: ");
//  Serial.println(angVelY);

  // Calculate PID error for roll (front-back thrusters)
  errorFB = roll;
  unsigned long currentTime = millis();
  unsigned long dtFB = currentTime - lastCycleTimeFB;
  integralFB += errorFB * dtFB;
  integralFB = constrain(integralFB, 0, 0.5);
  derivativeFB = (errorFB - previousErrorFB) / dtFB;

  float correctionFrontBack = kp * errorFB + ki * integralFB + kd * derivativeFB;

  // Calculate PID error for pitch (left-right thrusters)
  errorLR = pitch;
  unsigned long dtLR = currentTime - lastCycleTimeLR;
  integralLR += errorLR * dtLR;
  integralLR = constrain(integralLR, 0, 0.5);
  derivativeLR = (errorLR - previousErrorLR) / dtLR;

  float correctionLeftRight = kp * errorLR + ki * integralLR + kd * derivativeLR;

//  Serial.print("Roll: ");
//  Serial.print(roll);
  Serial.print("Front-Back Correction: ");
  Serial.print(correctionFrontBack);
  Serial.print(", Left-Right Correction: ");
  Serial.println(correctionLeftRight);

  thrustControlFrontBack(correctionFrontBack);
  thrustControlLeftRight(correctionLeftRight);

  previousErrorFB = errorFB;
  previousErrorLR = errorLR;

  applyCounterMotion(angVelX, angVelY);

//  Serial.print("Front-Back Duty Cycle: ");
//  Serial.println(dutyCycleFB * 100);
//  Serial.print("Left-Right Duty Cycle: ");
//  Serial.println(dutyCycleLR * 100);

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

//  Serial.print("Front: ");
//  Serial.print(digitalRead(thrusterFront) == HIGH ? "ON" : "OFF");
//
//  Serial.print(", Back: ");
//  Serial.print(digitalRead(thrusterBack) == HIGH ? "ON" : "OFF");
//
//  Serial.print(", Left: ");
//  Serial.print(digitalRead(thrusterLeft) == HIGH ? "ON" : "OFF");
//
//  Serial.print(", Right: ");
//  Serial.println(digitalRead(thrusterRight) == HIGH ? "ON" : "OFF");
}
