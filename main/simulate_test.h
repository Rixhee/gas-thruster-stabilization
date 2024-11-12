#ifndef SIMULATE_TEST_H
#define SIMULATE_TEST_H

#include "imu_control.h"

// Initialize the test simulation environment
void setupSimulation() {
    Serial.begin(9600);
    Serial.println("Simulation started for 4-thruster balance test");
}

// Function to apply simulated pitch, roll, and yaw values
void simulateOrientationInput() {
    // Simulate slight pitch, roll, and yaw oscillations
    static float simulatedPitch = 0.0;
    static float simulatedRoll = 0.0;
    static float simulatedYaw = 0.0;

    // pitch, roll, and yaw using sine functions
    simulatedPitch = 10 * sin(millis() / 1000.0);
    simulatedRoll = 5 * sin(millis() / 1200.0);
    simulatedYaw = 8 * sin(millis() / 1400.0);

    // simulated values to the control function
    simulateIMUInput(simulatedPitch, simulatedRoll, simulatedYaw);

    // balance score based on pitch, roll, and yaw
    float balanceScore = abs(simulatedPitch) + abs(simulatedRoll) + abs(simulatedYaw);
    float tolerance = 0.5; // Define tolerance for "balanced" state
    bool isBalanced = (abs(simulatedPitch) <= tolerance) &&
                     (abs(simulatedRoll) <= tolerance) &&
                     (abs(simulatedYaw) <= tolerance);

    // print out the current state and balance status
    Serial.print("Simulated Pitch: "); Serial.print(simulatedPitch);
    Serial.print("\tSimulated Roll: "); Serial.print(simulatedRoll);
    Serial.print("\tSimulated Yaw: "); Serial.print(simulatedYaw);
    Serial.print("\tBalanced: "); Serial.println(isBalanced ? "Yes" : "No");
    Serial.print("Balance Score: "); Serial.println(balanceScore);

    delay(1000);
}

#endif