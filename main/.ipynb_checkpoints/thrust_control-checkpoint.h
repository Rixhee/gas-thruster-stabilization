#ifndef THRUST_CONTROL_H
#define THRUST_CONTROL_H

#include <Arduino.h>

// Function to adjust thrust for four thrusters based on corrections for pitch, roll, and yaw
void thrustControl(float pitchCorrection, float rollCorrection, float yawCorrection) {
    // Calculate thrust values for each thruster based on orientation corrections
    int thrust1 = constrain(pitchCorrection - rollCorrection + yawCorrection, -1, 1);
    int thrust2 = constrain(pitchCorrection + rollCorrection - yawCorrection, -1, 1);
    int thrust3 = constrain(-pitchCorrection + rollCorrection + yawCorrection, -1, 1);
    int thrust4 = constrain(-pitchCorrection - rollCorrection - yawCorrection, -1, 1);

    // Set the valves based on calculated thrust values
    digitalWrite(thruster1, thrust1 > 0 ? HIGH : LOW);
    digitalWrite(thruster2, thrust2 > 0 ? HIGH : LOW);
    digitalWrite(thruster3, thrust3 > 0 ? HIGH : LOW);
    digitalWrite(thruster4, thrust4 > 0 ? HIGH : LOW);

    // // Serial monitor for debugging and observing outputs
    // Serial.print("Thrust1: "); Serial.print(thrust1 > 0 ? "OPEN" : "CLOSED");
    // Serial.print("\tThrust2: "); Serial.print(thrust2 > 0 ? "OPEN" : "CLOSED");
    // Serial.print("\tThrust3: "); Serial.print(thrust3 > 0 ? "OPEN" : "CLOSED");
    // Serial.print("\tThrust4: "); Serial.println(thrust4 > 0 ? "OPEN" : "CLOSED");
}

#endif