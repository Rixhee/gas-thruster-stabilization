#include "imu_functions.h"

const int thrusterFront = 5;
const int thrusterBack = 2;

// PID variables
float kp = 1;
float ki = 0.001;
float kd = 0.01;

float yaw = 0;
float pitch = 0;
float roll = 0;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

float rollThreshold = 0.02; 
unsigned long cycleDuration = 100;
unsigned long lastCycleTime = 0;
unsigned long onTime = 0;

bool thrusterState = false;

float dutyCycle = 0;

void setup() {
  Serial.begin(115200);
  setupIMU();
  pinMode(thrusterFront, OUTPUT);
  pinMode(thrusterBack, OUTPUT);
}

void thrustControl(float correction) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastCycleTime;

    // Check if a new cycle should start
    if (elapsedTime >= cycleDuration) {
        lastCycleTime = currentTime;

        dutyCycle = constrain(abs(correction), 0.0, 1.0); 
        onTime = dutyCycle * cycleDuration;
    }

    // Control thrusters ON/OFF based on elapsed time and roll threshold
    if (elapsedTime < onTime && abs(roll) > rollThreshold) {
        if (error > 0) {
            digitalWrite(thrusterFront, HIGH);
            digitalWrite(thrusterBack, LOW);
        } else if (error < 0) {
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


void loop() {
  loopIMU();

  float* yprValues = getYPR();
  yaw = yprValues[0];
  roll = yprValues[1];
  pitch = yprValues[2];

  // Calculate PID error
  error = roll;
  unsigned long currentTime = millis();
  unsigned long dt = currentTime - lastCycleTime;
  integral += error * dt;
  integral = constrain(integral, 0, 0.5);
  derivative = (error - previousError) / dt;

  float correction = kp * error + ki * integral + kd * derivative;

  Serial.print("Roll: ");
  Serial.print(roll);
  Serial.print(", Correction: ");
  Serial.println(correction);

  thrustControl(correction);

  previousError = error;

  Serial.print("Duty Cycle: ");
  Serial.println(dutyCycle * 100);

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
