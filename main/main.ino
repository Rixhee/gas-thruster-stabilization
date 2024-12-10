#include "imu_functions.h"

const int front = 5;
const int back = 2;

// PID
float kp = 1;
float ki = 0.01;
float kd = 0.5;

float pitch = 0;
float roll = 0;

float angVelX = 0;
float angVelY = 0;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

float angleThreshold = 0.30;
float velThreshold = 25;

unsigned long onTime = 0;
unsigned long initialTime = 0;
unsigned long previousTime = 0;
unsigned long elapsedTime = 0;

bool thrusterState = false;

void setup() {
  Serial.begin(115200);
  setupIMU();
  pinMode(front, OUTPUT);
  pinMode(back, OUTPUT);
}

void counterControl(float correction) {
  elapsedTime = millis() - initialTime;
  
  if (thrusterState == false) {
    if (abs(angVelX) > velThreshold && abs(roll) < angleThreshold) {
      initialTime = millis();
      onTime = correction;
      if (onTime > 10) {
        thrusterState = true; 
      }
    }
  } else {
    // Update thruster control logic to stop if conditions are not met
    if (elapsedTime < onTime && abs(angVelX) > velThreshold && abs(roll) < angleThreshold) {
      // Continue with current thruster state
      if (angVelX < 0) {
        digitalWrite(front, HIGH);
        digitalWrite(back, LOW);
      } else if (angVelX > 0) {
        digitalWrite(back, HIGH);
        digitalWrite(front, LOW);
      }
    } else {
      // Stop thrusters if the conditions are no longer met or if time is up
      digitalWrite(front, LOW);
      digitalWrite(back, LOW);
      thrusterState = false;
    }
  }
}

void loop() {
  loopIMU();

  float* YPR = getYPR();
  roll = YPR[1];
  pitch = YPR[2];

  float* angVelXYZ = getAngularVelocity();
  angVelY = angVelXYZ[0];
  angVelX = angVelXYZ[1];

  // Calculate PID error
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - previousTime;
  previousTime = currentTime;

  error = angVelX;
  integral += error * dt;
  integral = constrain(integral, 0, 10);
  derivative = (error - previousError) / dt;

  float correction = kp * error + ki * integral + kd * derivative;

  Serial.print("AngVelX: ");
  Serial.print(angVelX);
  Serial.print(", angVelY: ");
  Serial.print(angVelY);
  Serial.print(", Correction: ");
  Serial.println(correction);
  
  counterControl(correction);

  previousError = error;

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
    } else if (variable == "at") {
      angleThreshold = value;
      Serial.print("Updated Angle Threshold: ");
      Serial.println(angleThreshold);
    } else if (variable == "vt") {
      velThreshold = value;
      Serial.print("Updated Vel Threshold");
      Serial.println(velThreshold);
    } 
  } 
}
