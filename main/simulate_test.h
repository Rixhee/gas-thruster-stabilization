#ifndef SIMULATE_TEST_H
#define SIMULATE_TEST_H

#include "imu_control.h"
#include "thrust_control.h"

float simulatedPitch = 0;  // Simulated pitch value for testing
float testCorrection = 0;  // Correction output for test
bool isStable = false;     // Track if "balance" is achieved

const float targetPitch = 0;      // Target pitch angle for "balance"
const float tolerance = 5.0;      // Degrees tolerance for acceptable balance

void setupSimulation() {
    Serial.begin(9600);
    Serial.println("Simulation started: Balancing see-saw test");
}

void simulatePitchInput() {
    // a simulated pitch oscillation to represent a tilting "see-saw"
    simulatedPitch = 15 * sin(millis() / 1000.0);

    // error and correction
    float error = (simulatedPitch - targetPitch) / 90;  // Normalize error
    integral += error;
    float derivative = error - previousError;
    testCorrection = kp * error + ki * integral + kd * derivative;
    previousError = error;

    // applying thrust to correct pitch
    thrustControl(testCorrection);

    // is "balance" achieved (within tolerance)
    if (abs(simulatedPitch - targetPitch) <= tolerance) {
        if (!isStable) {
            Serial.println("Balance achieved within tolerance range.");
            isStable = true;
        }
    } else {
        if (isStable) {
            Serial.println("Out of balance.");
            isStable = false;
        }
    }

    // print simulated values
    Serial.print("Simulated Pitch: "); Serial.print(simulatedPitch);
    Serial.print("\tCorrection: "); Serial.print(testCorrection);
    Serial.print("\tStability: "); Serial.println(isStable ? "Stable" : "Unstable");

    delay(1000)
}

#endif