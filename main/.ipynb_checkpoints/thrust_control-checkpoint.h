#ifndef THRUST_CONTROL_H
#define THRUST_CONTROL_H

// Constants
const int thrusterFront = 11, thrusterBack = 12, thrusterLeft = 13, thrusterRight = 14;
const int PSI = 60;  // Thrust pressure in PSI
const int threshold = 2;  // Threshold for minimal pitch and roll adjustments

// PID variables
float kp = 150, ki = 0, kd = 0;

// IMU variables
float yaw = 0, pitch = 0, roll = 0;

// PID error and integral variables for pitch and roll
float previousErrorPitch = 0, previousErrorRoll = 0, integralPitch = 0, integralRoll = 0;

unsigned long previous_time = 0;

// Thrust control functions
void thrustControl(float current_pitch, float current_roll);
float pid_control(float current_error, float &previous_error, float &integral, unsigned long dt, float kp, float ki, float kd);
void no_opps(int &front_thrust, int &back_thrust, int &left_thrust, int &right_thrust);

// No opposing thrusters function
void no_opps(int &front_thrust, int &back_thrust, int &left_thrust, int &right_thrust) {
  if (front_thrust > 0 && back_thrust > 0) {
    front_thrust = back_thrust = 0;
  }
  if (left_thrust > 0 && right_thrust > 0) {
    left_thrust = right_thrust = 0;
  }
}

// PID control function
float pid_control(float current_error, float &previous_error, float &integral, unsigned long dt, float kp, float ki, float kd) {
  integral += current_error * dt;
  float derivative = (current_error - previous_error) / dt;
  previous_error = current_error;
  return kp * current_error + ki * integral + kd * derivative;
}

// Main thrust control function
void thrustControl(float current_pitch, float current_roll) {
  int front_thrust = 0, back_thrust = 0, left_thrust = 0, right_thrust = 0;

  // Calculate pitch and roll errors
  float pitch_error = TARGET_PITCH - current_pitch;
  float roll_error = TARGET_ROLL - current_roll;

  // If both pitch and roll errors are within the threshold, no thrust needed
  if (abs(pitch_error) < threshold && abs(roll_error) < threshold) {
    Serial.println("Pitch and Roll errors are within threshold. No thrust applied.");
    return;
  }

  unsigned long dt = millis() - previous_time;

  // Apply PID control for pitch and roll
  float pitch_correction = pid_control(pitch_error, previousErrorPitch, integralPitch, dt, kp, ki, kd);
  float roll_correction = pid_control(roll_error, previousErrorRoll, integralRoll, dt, kp, ki, kd);

  // Apply corrections based on errors
  if (pitch_error > 0) {
    front_thrust = PSI;
  } else if (pitch_error < 0) {
    back_thrust = PSI;
  }

  if (roll_error > 0) {
    left_thrust = PSI;
  } else if (roll_error < 0) {
    right_thrust = PSI;
  }

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

  // Update previous errors and time for the next cycle
  previous_time = millis();

  // Apply thrust values to the motors
  digitalWrite(thrusterFront, front_thrust ? HIGH : LOW);
  digitalWrite(thrusterBack, back_thrust ? HIGH : LOW);
  digitalWrite(thrusterLeft, left_thrust ? HIGH : LOW);
  digitalWrite(thrusterRight, right_thrust ? HIGH : LOW);
}

#endif