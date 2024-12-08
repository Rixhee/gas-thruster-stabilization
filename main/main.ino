#include "imu_functions.h"
#include "thrust_control.h"

void setup() {
  Serial.begin(115200);
  setupIMU();
  pinMode(thrusterFront, OUTPUT);
  pinMode(thrusterBack, OUTPUT);
  pinMode(thrusterLeft, OUTPUT);
  pinMode(thrusterRight, OUTPUT);
}

void loop() {
  loopIMU();

  float* yprValues = getYPR(); // * 60 for rough degrees
  float yaw = yprValues[0] * 60;
  float roll = yprValues[1] * 60;
  float pitch = yprValues[2] * 60;

  Serial.println("yaw: " + String(yaw) + " pitch: " + String(pitch) + " roll: " + String(roll));

  // Control the thrusters
  thrustControl(pitch, roll);

  // Process Serial commands for tuning parameters
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n');
    if (input == "reset") {
      setupIMU();
    } else {
      int index_delimiter = input.indexOf(" ");
      String selectedVariable = input.substring(0, index_delimiter);
      float value = input.substring(index_delimiter).toFloat();

      // if (selectedVariable == "kp") {
      //   kp = value;
      //   Serial.print("kp: ");
      //   Serial.println(kp);
      // } else if (selectedVariable == "ki") {
      //   ki = value;
      //   Serial.print("ki: ");
      //   Serial.println(ki);
      // } else if (selectedVariable == "kd") {
      //   kd = value;
      //   Serial.print("kd: ");
      //   Serial.println(kd);
      // } else 
      if (selectedVariable == "tp") {
        TARGET_PITCH = value;
        Serial.print("target pitch: ");
        Serial.println(TARGET_PITCH);
      } else if (selectedVariable == "tr") {
        TARGET_ROLL = value;
        Serial.print("target roll: ");
        Serial.println(TARGET_ROLL);
      } else if (selectedVariable == "threshold") {
        threshold = value;
        Serial.print("threshold: ");
        Serial.println(threshold);
      }
    }
  }
}
