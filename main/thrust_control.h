#ifndef THRUST_CONTROL_H
#define THRUST_CONTROL_H

#include "AdaptivePIDController.h"

// Constants
// const int thrusterFront = 23, thrusterBack = 16, thrusterLeft = 2, thrusterRight = 5;
const int thrusterFront = 16, thrusterBack = 23, thrusterLeft = 5, thrusterRight = 2;
const int PSI = 60;  // Thrust pressure in PSI
int TARGET_PITCH = 0;   // Target pitch in degrees                      (tp)
int TARGET_ROLL = 0;  // Target roll in degrees                         (tr)
int threshold = 1;  // Threshold for minimal pitch and roll adjustments (th)

// U CAN SET THESE VALUES FOR CONSTANT
float kp = 0;
float ki = 0;
float kd = 0;

// Cycle counting constants
const int MAX_CYCLES = 5;  // Maximum number of cycles a thruster can be activated

// Cycle counters for each axis
int pitchCycleCounter = 0;
int rollCycleCounter = 0;

unsigned long previous_time = 0;

// Create adaptive PID controllers
AdaptivePIDController pitchPID;//(kp,ki,kd);
AdaptivePIDController rollPID;//(kp,ki,kd);

// Thrust control functions
void thrustControl(float current_pitch, float current_roll);
void no_opps(int &front_thrust, int &back_thrust, int &left_thrust, int &right_thrust);

// No opposing thrusters function (unchanged)
void no_opps(int &front_thrust, int &back_thrust, int &left_thrust, int &right_thrust) {
  if (front_thrust > 0 && back_thrust > 0) {
    front_thrust = back_thrust = 0;
  }
  if (left_thrust > 0 && right_thrust > 0) {
    left_thrust = right_thrust = 0;
  }
}

// Main thrust control function with adaptive PID and cycle counting
void thrustControl(float current_pitch, float current_roll) {
  int front_thrust = 0, back_thrust = 0, left_thrust = 0, right_thrust = 0;
  
  // Calculate pitch and roll errors
  float pitch_error = TARGET_PITCH - current_pitch;
  float roll_error = TARGET_ROLL - current_roll;
  
  // If both pitch and roll errors are within the threshold, reset cycle counters and return
  if (abs(pitch_error) < threshold && abs(roll_error) < threshold) {
    Serial.println("Pitch and Roll errors are within threshold. No thrust applied.");
    pitchCycleCounter = 0;
    rollCycleCounter = 0;
    digitalWrite(thrusterFront, LOW);
    digitalWrite(thrusterBack, LOW);
    digitalWrite(thrusterLeft, LOW);
    digitalWrite(thrusterRight, LOW);
    return;
  }
  
  unsigned long current_time = millis();
  unsigned long dt = current_time - previous_time;
  
  // Apply Adaptive PID control for pitch and roll
  float pitch_correction = pitchPID.compute(TARGET_PITCH, current_pitch, dt / 1000.0);
  float roll_correction = rollPID.compute(TARGET_ROLL, current_roll, dt / 1000.0);
  
  // Pitch correction with cycle counting
  if (abs(pitch_error) > threshold && pitchCycleCounter <= MAX_CYCLES) {
    if (pitch_correction < 0.15) {                // This was changed from '>' to '<' in the last push
      front_thrust = PSI;
      pitchCycleCounter++;
    } else if (pitch_correction > -0.15) {        // This was changed from '<' to '>' in the last push
      back_thrust = PSI;
      pitchCycleCounter++;
    }
  } else {
    // Reset pitch cycle counter when close to target
    pitchCycleCounter = 0;
  }
  
  // Roll correction with cycle counting
  if (abs(roll_error) > threshold && rollCycleCounter <= MAX_CYCLES) {
    if (roll_correction < 0.15) {                // This was changed from '>' to '<' in the last push
      left_thrust = PSI;
      rollCycleCounter++;
    } else if (roll_correction > -0.15) {        // This was changed from '<' to '>' in the last push
      right_thrust = PSI;
      rollCycleCounter++;
    }
  } else {
    // Reset roll cycle counter when close to target
    rollCycleCounter = 0;
  }
  
  // Debug logging with adaptive PID gains
  Serial.print("Pitch PID Gains (P, I, D): ");
  Serial.print(pitchPID.getKp());
  Serial.print(", ");
  Serial.print(pitchPID.getKi());
  Serial.print(", ");
  Serial.println(pitchPID.getKd());
  
  Serial.print("Roll PID Gains (P, I, D): ");
  Serial.print(rollPID.getKp());
  Serial.print(", ");
  Serial.print(rollPID.getKi());
  Serial.print(", ");
  Serial.println(rollPID.getKd());
  
  Serial.print("Pitch Cycle Counter: ");
  Serial.println(pitchCycleCounter);
  Serial.print("Roll Cycle Counter: ");
  Serial.println(rollCycleCounter);
  
  Serial.print("Pitch Error: ");
  Serial.println(pitch_error);
  Serial.print("Roll Error: ");
  Serial.println(roll_error);
  
  // Prevent opposing thrusters from firing
  no_opps(front_thrust, back_thrust, left_thrust, right_thrust);
  
  // Print calculated thrust values
  Serial.print("Thrusts (Front, Back, Left, Right): ");
  Serial.print(front_thrust);
  Serial.print(", ");
  Serial.print(back_thrust);
  Serial.print(", ");
  Serial.print(left_thrust);
  Serial.print(", ");
  Serial.println(right_thrust);
  
  // Update previous time for the next cycle
  previous_time = current_time;
  
  // Apply thrust values to the motors
  digitalWrite(thrusterFront, front_thrust ? HIGH : LOW);
  digitalWrite(thrusterBack, back_thrust ? HIGH : LOW);
  digitalWrite(thrusterLeft, left_thrust ? HIGH : LOW);
  digitalWrite(thrusterRight, right_thrust ? HIGH : LOW);
}

#endif