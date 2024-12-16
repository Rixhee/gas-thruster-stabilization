#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

class PIDController {
private:
    float integral, previousError;
    unsigned long lastTime;

public:
    PIDController() {
        integral = 0;
        previousError = 0;
        lastTime = millis(); // Initialize the last time
    }

    float calculate(float setpoint, float currentAngle, float Kp, float Ki, float Kd) {
        unsigned long currentTime = millis();
        float dt = (currentTime - lastTime) / 1000.0; // Convert ms to seconds
        float error = setpoint - currentAngle;

        // Proportional term
        float Pout = Kp * error;

        // Integral term
        integral += error * dt;
        float Iout = Ki * integral;

        // Derivative term
        float derivative = (error - previousError) / dt;
        float Dout = Kd * derivative;

        // Total output
        float output = Pout + Iout + Dout;

        // Save the error for the next iteration
        previousError = error;
        lastTime = currentTime;

        return output;
    }
};

#endif // PID_CONTROLLER_H



// float kp = 1, ki = .001, kd = .01;
// float outputMin = 0, outputMax = 100;

// // Target orientation
// float TARGET_PITCH = 0;
// float TARGET_ROLL = 0;
// float threshold = 1;

// const int thrusterFront = 16, thrusterBack = 23, thrusterLeft = 5, thrusterRight = 2;
// class Thruster {
// private:
//     int pin;
//     float previousAngle = 0;
//     float previousVelocity = 0;
//     PIDController pidController;
//     PIDController pidControllerVelocity;
//     const float minVel = 0; 

//     unsigned long PERIOD = 100; // Total period in milliseconds
//     unsigned long lastUpdateTime = 0; // Last time PWM state was updated
//     int pwmState = 0; // Current state of the PWM output

//     int pwm(int dutyCyclePercent) {
//         // Constrain the input to be between 0 and 100
//         dutyCyclePercent = constrain(dutyCyclePercent, 0, 100);
        
//         // Calculate the elapsed time since the last update
//         unsigned long currentMillis = millis();
//         unsigned long elapsedTime = currentMillis - lastUpdateTime;
        
//         // Calculate the ON time and OFF time in milliseconds
//         unsigned long onTime = (PERIOD * dutyCyclePercent) / 100;
//         unsigned long offTime = PERIOD - onTime;

//         // Update the PWM output state based on elapsed time
//         if (elapsedTime >= onTime + offTime) {
//             lastUpdateTime = currentMillis; // Reset last update time
//             pwmState = 0; // Reset to OFF state
//         } else if (elapsedTime < onTime) {
//             pwmState = 1; // Set to ON state
//         } else {
//             pwmState = 0; // Set to OFF state
//         }

//         return pwmState; // Return the current PWM state (0 or 1)
//     }

// public:
//     Thruster(int pin, float KP, float KI, float KD, float KP_velocity, float KI_velocity, float KD_velocity) 
//     : pin(pin), 
//       pidController(KP, KI, KD), 
//       pidControllerVelocity(KP_velocity, KI_velocity, KD_velocity) {
//         pinMode(pin, OUTPUT);
//     }

//     int getPin() const { return pin; }

//     // Added setpoint for desired velocity
//     int thrust(float angleSetpoint, float currentAngle, float currentVelocity, float desiredVelocity=0) {
//         // Calculate thrust using angle PID
//         float angleOutput = pidController.calculate(angleSetpoint, currentAngle);
        
//         // Calculate thrust using velocity PID
//         float velocityOutput = pidControllerVelocity.calculate(desiredVelocity, currentVelocity);
        
//         // Combine the outputs (could use different scaling)
//         float output = angleOutput * (velocityOutput * .3); // Scale outputs as needed

//         // Serial output for debugging
//         Serial.print("Thrust output (Angle): " + String(angleOutput) + ", Velocity: " + String(velocityOutput) + " ----- ");

//         // Apply thrust if conditions are met
//         if (abs(angleSetpoint - currentAngle) > threshold && currentVelocity <= minVel) {
//             Serial.println("Thrusting: " + String(output));
//             return pwm(constrain(output, outputMin, outputMax));  // Send PWM signal
//         } else { 
//             Serial.println("No thrust");
//             return 0;  // No thrust if conditions are not met
//         }
//     }
// };

// class Seesaw {
// private:
//     Thruster thruster1;
//     Thruster thruster2;
//     bool switchy = false;

// public:
//     public:
//     Seesaw(int pin1, int pin2, float KP, float KI, float KD, float KP_velocity, float KI_velocity, float KD_velocity)
//         : thruster1(pin1, KP, KI, KD, KP_velocity, KI_velocity, KD_velocity), 
//           thruster2(pin2, KP, KI, KD, KP_velocity, KI_velocity, KD_velocity) {}

//     void balance(float setpoint, float currentAngle, float currentVelocity) {
//         bool thrust1 = thruster1.thrust(setpoint, currentAngle, currentVelocity) > 0;
//         bool thrust2 = thruster2.thrust(-setpoint, -currentAngle, -currentVelocity) > 0;
//         Serial.println(String(thrust1)+" i---i "+String(thrust2));
//         if (thrust1 && thrust2) {
//           if (switchy) {
//             digitalWrite(thruster1.getPin(), HIGH);
//             digitalWrite(thruster2.getPin(), LOW);
//           } else {
//             digitalWrite(thruster1.getPin(), LOW);
//             digitalWrite(thruster2.getPin(), HIGH);
//           } 
//           switchy = !switchy;
//         } else if (thrust1) {
//             digitalWrite(thruster1.getPin(), HIGH);
//             digitalWrite(thruster2.getPin(), LOW);
//         } else if (thrust2) {
//            digitalWrite(thruster1.getPin(), LOW);
//            digitalWrite(thruster2.getPin(), HIGH);
//         } else {
//             digitalWrite(thruster1.getPin(), LOW);
//             digitalWrite(thruster2.getPin(), LOW);
//         }
//     }
// };