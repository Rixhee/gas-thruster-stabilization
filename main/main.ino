#include "imu_functions.h"

const int front = 5;
const int back = 2;

// PID
float kp = 1;
float ki = 0.001;
float kd = 0.01;
float kt = 0.1;

float yaw = 0;
float pitch = 0;
float roll = 0;

float angVelX = 0;
float angVelY = 0;
float angVelZ = 0;

float error = 0;
float previousError = 0;
float integral = 0;
float derivative = 0;

float threshold = 0.03;

unsigned long cycleDuration = 100;
unsigned long lastCycleTime = 0;
unsigned long onTime = 0;

bool thrusterState = false;

float dutyCycle = 0;

void setup() {
  Serial.begin(115200);
  setupIMU();
  pinMode(front, OUTPUT);
  pinMode(back, OUTPUT);
}

float counterThreshold = 45;
float rollThresholdMax = 0.4;
float rollThresholdMin = 0.1;

void counterControl() {
  if (abs(roll) > rollThresholdMin && abs(roll) < rollThresholdMax && abs(angVelX) > counterThreshold) {
    // Counter thruster activation logic
    if (roll > 0) {
      digitalWrite(back, HIGH);
      digitalWrite(front, LOW);
    } else if (roll < 0) {
      digitalWrite(front, HIGH);
      digitalWrite(back, LOW);
    }
  } else if (thrusterState == false) {
    // Turn off both thrusters if neither condition is met
    digitalWrite(front, LOW);
    digitalWrite(back, LOW);
  }
}

void thrustControl(float correction) {
    unsigned long currentTime = millis();
    unsigned long elapsedTime = currentTime - lastCycleTime;

    if (elapsedTime >= cycleDuration) {
        lastCycleTime = currentTime;

        // Calculate duty cycle based on correction
        if (correction > 0.6) {
           dutyCycle = 1; 
        }
        dutyCycle = constrain(abs(correction), 0.0, 1.0); 
        onTime = dutyCycle * cycleDuration;
    }

    if (elapsedTime < onTime && abs(roll) > threshold) {
        // Apply primary thruster correction based on error
        if (error > 0) {
            digitalWrite(front, HIGH);
            digitalWrite(back, LOW);
        } else if (error < 0) {
            digitalWrite(back, HIGH);
            digitalWrite(front, LOW);
        }
        thrusterState = true;
    } else {
        // Reset thrusters after onTime
        digitalWrite(front, LOW);
        digitalWrite(back, LOW);
        thrusterState = false;
    }
}

void loop() {
  loopIMU();

  float* yprValues = getYPR();
  yaw = yprValues[0];
  roll = yprValues[1];
  pitch = yprValues[2];

  float* angVelXYZ = getAngularVelocity();
  angVelY = angVelXYZ[0];
  angVelX = angVelXYZ[1];
  angVelZ = angVelXYZ[2];

  Serial.print("AngVel X: ");
  Serial.print(angVelX);
  Serial.print(", AngVel Y: ");
  Serial.println(angVelY);

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
  Serial.print(", Pitch: ");
  Serial.print(pitch);
  Serial.print(", Correction: ");
  Serial.println(correction);

//  thrustControl(correction);
  counterControl();

  previousError = error;

//  Serial.print("Duty Cycle: ");
//  Serial.println(dutyCycle * 100);

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
    } else if (variable == "ct") {
      counterThreshold = value;
      Serial.print("Updated Cycle Duration: ");
      Serial.println(counterThreshold);
    } else if (variable = "rtmax") {
      rollThresholdMax = value;
      Serial.print("Updated Roll Threshold");
      Serial.println(rollThresholdMax);
    } else if (variable = "rtmin") {
      rollThresholdMin = value;
      Serial.print("Updated Roll Threshold");
      Serial.println(rollThresholdMin);
    }
  } 
}
