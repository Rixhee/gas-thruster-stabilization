#ifndef THRUST_CONTROL_H
#define THRUST_CONTROL_H

#include "pid_controller.h"

const int thrusterFront = 16, thrusterBack = 23, thrusterLeft = 5, thrusterRight = 2;
float kp = 1, ki = .01, kd = .1;
float outputMin = 0, outputMax = 100;

// Target orientation
float TARGET_PITCH = 0;
float TARGET_ROLL = 0;
float threshold = 5;

// Added variables to keep track of previous thrust
float previousThrust = 0; // Last thrust applied
int previousThrusterIndex = -1; // Index of the last thruster that applied thrust
float holdThrust = 0;
int holdThrusterIndex = -1;

class Thruster {
private:
    int pin;
    float previousError = 0;
    PIDController pidController;
    const float minVel = 0;
    const float velThresh = 20;
    
    unsigned long PERIOD = 100; // Total period in milliseconds
    unsigned long lastUpdateTime = 0; // Last time PWM state was updated
    int pwmState = 0;  // Current state of the PWM output

    int pwm(int dutyCyclePercent) {
        // Constrain the input to be between 0 and 100
        dutyCyclePercent = constrain(dutyCyclePercent, 0, 100);
        
        // Calculate the elapsed time since the last update
        unsigned long currentMillis = millis();
        unsigned long elapsedTime = currentMillis - lastUpdateTime;
        
        // Calculate the ON time and OFF time in milliseconds
        unsigned long onTime = (PERIOD * dutyCyclePercent) / 100;
        unsigned long offTime = PERIOD - onTime;

        // Update the PWM output state based on elapsed time
        if (elapsedTime >= onTime + offTime) {
            lastUpdateTime = currentMillis; // Reset last update time
            pwmState = 0; // Reset to OFF state
        } else if (elapsedTime < onTime) {
            pwmState = 1; // Set to ON state
        } else {
            pwmState = 0; // Set to OFF state
        }

        return pwmState; // Return the current PWM state (0 or 1)
    }

public:
    Thruster(int pin, float KP, float KI, float KD) 
    : pin(pin), pidController(KP, KI, KD) {
        pinMode(pin, OUTPUT);
    }

    int getPin() const { return pin; }

    bool holdThruster(float error, float currentAngle, float currentVelocity) {
      return false;
        // Check if we need to hold thrust
        if (holdThrusterIndex == getPin() && holdThrust > 0 
          && abs(error) <= threshold && currentVelocity <= velThresh) {
            Serial.println("Holding thrust: " + String(holdThrust) + " on thruster " + String(holdThrusterIndex));
            digitalWrite(holdThrusterIndex, HIGH);
            delay( holdThrust );
            return true;
        }
        return false;
    }

    bool counterThrust(float error, float currentAngle, float currentVelocity) {
        // Check if a transition is necessary
        if (previousThrusterIndex != getPin() && previousThrusterIndex != -1 && previousThrust > 0 
        && (abs(error) > threshold || currentVelocity > velThresh) && (error * previousError) > 0) {
            float dampingFactor = constrain(abs(error) / 50, 0, 1);
            Serial.println("Countering previous thrust of: " + String(previousThrust) + " from thruster " + String(previousThrusterIndex) + " dampening: "+String(dampingFactor));
            digitalWrite(getPin(), HIGH);
            digitalWrite(previousThrusterIndex, LOW);
            delay( min(500.0, max(0.0, (previousThrust * dampingFactor * 1.5) )) );
            digitalWrite(getPin(), LOW);
            previousThrust = 0; // Reset previous thrust after counter
            return true;
        }
        return false;
    }

    int thrust(float setpoint, float currentAngle, float currentVelocity, bool positive) {
        float error = abs(setpoint) - abs(currentAngle);
        // Hold previous thrust if applicable  // Counter previous thrust if applicable
        if (holdThruster(error, currentAngle, currentVelocity) || counterThrust(error, currentAngle, currentVelocity)) {
          return 0;
        }
        // Calculate thrust using PID
        float output = abs(pidController.calculate(setpoint, currentAngle) * 5);
        if (currentVelocity > 30) {
            output = output * abs(currentVelocity / 100);
        }
        output = constrain(output,0,100);

        Serial.print("ERROR: "+String(error)+" Thrust output: " + String(output) + " ----- ");

        // Apply thrust if conditions are met && abs(currentVelocity) < minVel
        if (abs(error) > threshold && (positive && setpoint <= 0 || !positive && setpoint >= 0)) {
            Serial.println("Thrusting: " + String(output));
            previousThrust = output; // Record current thrust to counter on next call
            previousThrusterIndex = getPin(); // Record index of the current thruster
            holdThrust = output; 
            holdThrusterIndex = getPin(); 
            return pwm(output); // Send PWM signal
        } else {
            Serial.println("No thrust");
            return 0; // No thrust if conditions are not met
        }
    }
};

class Seesaw {
private:
    Thruster thruster1;
    Thruster thruster2;
    bool switchy = false;

public:
    Seesaw(int pin1, int pin2, float KP, float KI, float KD)
        : thruster1(pin1, KP, KI, KD), thruster2(pin2, KP, KI, KD) {}

    void balance(float setpoint, float currentAngle, float currentVelocity) {
        bool thrust1 = thruster1.thrust(setpoint, currentAngle, currentVelocity, false) > 0;
        bool thrust2 = thruster2.thrust(setpoint, currentAngle, currentVelocity, true) > 0;
        Serial.println(String(thrust1)+" i---i "+String(thrust2));
        if (thrust1 && thrust2) {
          if (switchy) {
            digitalWrite(thruster1.getPin(), HIGH);
            digitalWrite(thruster2.getPin(), LOW);
          } else {
            digitalWrite(thruster1.getPin(), LOW);
            digitalWrite(thruster2.getPin(), HIGH);
          } 
          switchy = !switchy;
        } else if (thrust1) {
            digitalWrite(thruster1.getPin(), HIGH);
            digitalWrite(thruster2.getPin(), LOW);
        } else if (thrust2) {
           digitalWrite(thruster1.getPin(), LOW);
           digitalWrite(thruster2.getPin(), HIGH);
        } else {
            digitalWrite(thruster1.getPin(), LOW);
            digitalWrite(thruster2.getPin(), LOW);
        }

        // float thrust1 = thruster1.thrust(setpoint, currentAngle, currentVelocity);
        // float thrust2 = thruster2.thrust(-setpoint, -currentAngle, -currentVelocity);
        // float thrust = 0;
        //  if (thrust1 > 0) {
        //   digitalWrite(thruster1.getPin(), HIGH);
        //   digitalWrite(thruster2.getPin(), LOW);
        //   thrust = thrust1;
        // }
        // if (thrust2 > 0) {
        //   digitalWrite(thruster1.getPin(), LOW);
        //   digitalWrite(thruster2.getPin(), HIGH);
        //   thrust = thrust2;
        // }
        // delay(thrust);
        // digitalWrite(thruster1.getPin(), LOW);
        // digitalWrite(thruster2.getPin(), LOW);

    }
};

// Single thruster configuration
// Thruster thrusterR(thrusterRight);
// Thruster thrusterL(thrusterLeft);

static Seesaw rollControl(thrusterRight, thrusterLeft, kp, ki, kd);

// Main control function for testing
void thrustControl(float currentAngle, float currentVelocity) {
    rollControl.balance(TARGET_ROLL, currentAngle, currentVelocity);
}

// class thruster(angle,vel,setpoint)
//   if angle == setpoint:
//     digitialwrite(low)
//   if velocity <= 0 && angle < setpoint-threshold:
//     digitalwrite(high)
//   if velocity > 0 && angle < 0:
//     digitalwrite(high)

#endif