#ifndef THRUST_CONTROL_H
#define THRUST_CONTROL_H

#include "pid_controller.h"

const int thrusterFront = 16, thrusterBack = 23, thrusterLeft = 2, thrusterRight = 5;
float kp = 1, ki = .01, kd = .1;

// Target orientation
float TARGET_PITCH = 0;
float TARGET_ROLL = 0;
float threshold = 5;
float minVelocity = 10;
float minOnTime = 100;
float maxOnTime = 1000;
float counterAngle = 2;
float counterVelocity = 5;

class Thruster {
private:
    int pin;
    float previousError = 0;
    PIDController pidController;

public:
    Thruster(int pin) 
    : pin(pin), pidController() {
        pinMode(pin, OUTPUT);
    }

    int getPin() const { return pin; }

    int thrust(float setpoint, float currentAngle, float currentVelocity, bool positive) {
        float error = abs(setpoint) - abs(currentAngle);
        Serial.print("SETPOINT: " + String(setpoint) + " CURR ANGLE: " + String(currentAngle) + " ----- ");
        
        // Calculate thrust using PID     (abs(error)*5) - (abs(currentVelocity)*2.5);//
        float output = abs(pidController.calculate(setpoint, currentAngle, kp, ki, kd))*10;
        // if (currentVelocity > 30) {
        //     output = output * abs(currentVelocity / 100);
        // }
        // output = constrain(output, 0, 100);

        Serial.print("ERROR: " + String(error) + " Thrust output: " + String(output) + " ----- ");

        // Apply thrust if conditions are met // && abs(currentVelocity) < minVelocity 
        if (abs(error) > threshold && (positive && setpoint < currentAngle || !positive && setpoint > currentAngle) 
        // (positive && currentVelocity < .1 || !positive && currentVelocity > -.1)
        ) {
            Serial.println("Thrusting: " + String(output));
            return (output); // Send PWM signal
        } else {
            Serial.println("No thrust");
            return 0; // No thrust if conditions are not met
        }
    }
};

class Seesaw {
private:
    // Thruster control parameters
    unsigned long currentOnTime = 0; // Current on time for the active thruster
    unsigned long lastThrustStartTime = 0; // Time when the last thrust started

    Thruster thruster1;
    Thruster thruster2;
    float previousThrust = 0;
    int previousThrusterPin = -1;
    bool isThrusterActive = false;
    int activeThrusterPin = -1;

    // Counter thrust tracking
    bool isCounterThrustActive = false;
    unsigned long counterThrustStartTime = 0;
    unsigned long counterThrustDuration = 0;
    int counterThrusterPin = -1;

    void performCounterThrust(float error, float currentVelocity) {
        if (previousThrusterPin == -1 || previousThrust == 0) return;

        float dampingFactor = constrain(abs(error) / 50, 0, 1);
        
        // Determine counter pin
        counterThrusterPin = (previousThrusterPin == thruster1.getPin()) ? 
                              thruster2.getPin() : thruster1.getPin();
        
        // Calculate counter thrust duration based on previous thrust
        counterThrustDuration = min(500.0, max(0.0, (previousThrust * dampingFactor * 1.5)));
        
        // Activate counter thruster
        digitalWrite(counterThrusterPin, HIGH);
        digitalWrite(previousThrusterPin, LOW);
        
        // Mark counter thrust as active
        isCounterThrustActive = true;
        counterThrustStartTime = millis();
        
        Serial.println("Countering previous thrust of: " + String(previousThrust) + 
                       " from thruster pin " + String(previousThrusterPin) + 
                       " dampening: " + String(dampingFactor));
    }

    void checkAndEndCounterThrust() {
        unsigned long currentMillis = millis();
        
        // Check if counter thrust duration has elapsed
        if (isCounterThrustActive && 
            (currentMillis - counterThrustStartTime >= counterThrustDuration)) {
            digitalWrite(counterThrusterPin, LOW);
            isCounterThrustActive = false;
            counterThrusterPin = -1;
            previousThrust = 0;
            previousThrusterPin = -1;
        }
    }

    void updateThrusterState(float thrust, int thrusterPin) {
        unsigned long currentMillis = millis();
        
        // // Calculate on time based on thrust (scaled to 100-1000ms)
        currentOnTime = constrain(thrust * 10, minOnTime, maxOnTime);

        // if (thrust * 10 >= minOnTime) {
        //     currentOnTime = min(thrust * 10, maxOnTime);
        // } else { // If thrust is too low, don't activate the thruster
        //     currentOnTime = 0;
        // } 

        // If no thruster is active, activate the current thruster
        if (!isThrusterActive) {
            digitalWrite(thrusterPin, HIGH);
            isThrusterActive = true;
            activeThrusterPin = thrusterPin;
            lastThrustStartTime = currentMillis;
            previousThrusterPin = thrusterPin;
            previousThrust = thrust;
        }
        // If a different thruster is currently active
        else if (activeThrusterPin != thrusterPin) {
            // Check if minimum on time has passed
            if (currentMillis - lastThrustStartTime >= minOnTime) {
                // Turn off current thruster
                digitalWrite(activeThrusterPin, LOW);
                
                // Activate new thruster
                digitalWrite(thrusterPin, HIGH);
                activeThrusterPin = thrusterPin;
                lastThrustStartTime = currentMillis;
                previousThrusterPin = thrusterPin;
                previousThrust = thrust;
            }
        }
        // If the same thruster is active, reset the timer if needed
        else {
            lastThrustStartTime = currentMillis;
        }
    }

    void checkAndDeactivateThruster() {
        unsigned long currentMillis = millis();
        
        // Check if current thruster has exceeded its on time
        if (isThrusterActive && 
            (currentMillis - lastThrustStartTime >= currentOnTime)) {
            digitalWrite(activeThrusterPin, LOW);
            isThrusterActive = false;
            activeThrusterPin = -1;
            previousThrust = 0;
            previousThrusterPin = -1;
        }
    }

public:
    // Constructor with optional parameters for min and max on times
    Seesaw(int pin1, int pin2)
        : thruster1(pin1), thruster2(pin2) {}

    void balance(float setpoint, float currentAngle, float currentVelocity) {
        float error = abs(setpoint) - abs(currentAngle);
        // Check and end counter thrust if duration has elapsed
        checkAndEndCounterThrust();
        // Check and deactivate thruster if it has been on too long
        checkAndDeactivateThruster();

        // Perform counter thrust if needed
        if (previousThrusterPin != -1 && previousThrust > 0 && 
            (abs(error) > counterAngle && abs(currentVelocity) > counterVelocity)) {
            performCounterThrust(abs(error), currentVelocity);
            return;
        }

        // Calculate thrust for each thruster
        float thrust1 = thruster1.thrust(setpoint, currentAngle, currentVelocity, true);
        float thrust2 = thruster2.thrust(setpoint, currentAngle, currentVelocity, false);
        
        Serial.println(String(thrust1) + " i---i " + String(thrust2));
        
        // Activate appropriate thruster based on thrust
        if (thrust1 > 0) {
            updateThrusterState(thrust1, thruster1.getPin());
        }
        if (thrust2 > 0) {
            updateThrusterState(thrust2, thruster2.getPin());
        }
    }

};

// Single thruster configuration
// Thruster thrusterR(thrusterRight);
// Thruster thrusterL(thrusterLeft);

static Seesaw pitchControl(thrusterFront, thrusterBack);
static Seesaw rollControl(thrusterRight, thrusterLeft);

// Main control function for testing
void thrustControl(float pitch, float pitchVel, float roll, float rollVel) {
    pitchControl.balance(TARGET_PITCH, pitch, pitchVel);
    rollControl.balance(TARGET_ROLL, roll, rollVel);
}

// class thruster(angle,vel,setpoint)
//   if angle == setpoint:
//     digitialwrite(low)
//   if velocity <= 0 && angle < setpoint-threshold:
//     digitalwrite(high)
//   if velocity > 0 && angle < 0:
//     digitalwrite(high)

#endif



// bool counterThrust(float setpoint, float currentAngle, float currentVelocity) {
//         float error = abs(setpoint) - abs(currentAngle);
//         if (previousThrusterPin != -1 && previousThrust > 0 
//             && (abs(error) > threshold || currentVelocity > 20)) {
            
//             float dampingFactor = constrain(abs(error) / 50, 0, 1);
//             Serial.println("Countering previous thrust of: " + String(previousThrust) + 
//                            " from thruster pin " + String(previousThrusterPin) + 
//                            " dampening: " + String(dampingFactor));
            
//             int counterPin = (previousThrusterPin == thruster1.getPin()) ? 
//                              thruster2.getPin() : thruster1.getPin();
            
//             digitalWrite(counterPin, HIGH);
//             digitalWrite(previousThrusterPin, LOW);
            
//             delay(min(500.0, max(0.0, (previousThrust * dampingFactor * 1.5))));
            
//             digitalWrite(counterPin, LOW);
//             previousThrust = 0;
//             previousThrusterPin = -1;
//             return true;
//         }
//         return false;
//     }