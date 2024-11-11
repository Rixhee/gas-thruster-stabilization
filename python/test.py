import math
import time
import numpy as np

# PID constants (default values)
kp = 1.2
ki = 0.21
kd = 1.1

# Initialize PID state variables
pitch_error = 0
roll_error = 0
yaw_error = 0
previous_pitch_error = 0
previous_roll_error = 0
previous_yaw_error = 0
pitch_integral = 0
roll_integral = 0
yaw_integral = 0

# Function to simulate IMU input (pitch, roll, yaw)
def simulate_imu_input():
    # Simulate slight pitch, roll, and yaw oscillations using sine functions
    simulated_pitch = 10 * math.sin(time.time() / 1.0)
    simulated_roll = 5 * math.sin(time.time() / 1.2)
    simulated_yaw = 8 * math.sin(time.time() / 1.4)
    
    return simulated_pitch, simulated_roll, simulated_yaw

# Function to simulate thrust control
def thrust_control(pitch_correction, roll_correction, yaw_correction):
    # Calculate thrust values for each thruster based on orientation corrections
    thrust1 = max(min(pitch_correction - roll_correction + yaw_correction, 1), -1)
    thrust2 = max(min(pitch_correction + roll_correction - yaw_correction, 1), -1)
    thrust3 = max(min(-pitch_correction + roll_correction + yaw_correction, 1), -1)
    thrust4 = max(min(-pitch_correction - roll_correction - yaw_correction, 1), -1)

    # Simulate setting valves for each thruster (printing instead of hardware control)
    print(f"Thrust1 (Front-Left): {'OPEN' if thrust1 > 0 else 'CLOSED'}")
    print(f"Thrust2 (Front-Right): {'OPEN' if thrust2 > 0 else 'CLOSED'}")
    print(f"Thrust3 (Back-Left): {'OPEN' if thrust3 > 0 else 'CLOSED'}")
    print(f"Thrust4 (Back-Right): {'OPEN' if thrust4 > 0 else 'CLOSED'}")

# Function to apply PID control and simulate the thrust control
def simulate_imu_control(sim_pitch, sim_roll, sim_yaw):
    global pitch_error, roll_error, yaw_error
    global previous_pitch_error, previous_roll_error, previous_yaw_error
    global pitch_integral, roll_integral, yaw_integral

    # Pitch control
    pitch_error = sim_pitch  # No scaling needed
    pitch_integral += pitch_error
    pitch_integral = max(min(pitch_integral, 10), -10)  # Limit integral to avoid windup
    pitch_correction = kp * pitch_error + ki * pitch_integral + kd * (pitch_error - previous_pitch_error)
    previous_pitch_error = pitch_error

    # Roll control
    roll_error = sim_roll
    roll_integral += roll_error
    roll_integral = max(min(roll_integral, 10), -10)  # Limit integral to avoid windup
    roll_correction = kp * roll_error + ki * roll_integral + kd * (roll_error - previous_roll_error)
    previous_roll_error = roll_error

    # Yaw control
    yaw_error = sim_yaw
    yaw_integral += yaw_error
    yaw_integral = max(min(yaw_integral, 10), -10)  # Limit integral to avoid windup
    yaw_correction = kp * yaw_error + ki * yaw_integral + kd * (yaw_error - previous_yaw_error)
    previous_yaw_error = yaw_error

    # Apply corrections through thrust control
    thrust_control(pitch_correction, roll_correction, yaw_correction)

# Main simulation loop
def run_simulation():
    balance_scores = []

    for _ in range(100):  # Simulate 10 cycles for tuning
        # Get simulated IMU input
        simulated_pitch, simulated_roll, simulated_yaw = simulate_imu_input()

        # Apply control logic to the simulated IMU data
        simulate_imu_control(simulated_pitch, simulated_roll, simulated_yaw)

        # Simulate balance status
        balance_score = abs(simulated_pitch) + abs(simulated_roll) + abs(simulated_yaw)
        tolerance = 5.0  # Relax tolerance for more flexibility
        is_balanced = (abs(simulated_pitch) <= tolerance) and (abs(simulated_roll) <= tolerance) and (abs(simulated_yaw) <= tolerance)

        # Output current state and balance status
        print(f"Simulated Pitch: {simulated_pitch:.2f}\tSimulated Roll: {simulated_roll:.2f}\tSimulated Yaw: {simulated_yaw:.2f}")
        print(f"Balanced: {'Yes' if is_balanced else 'No'}")
        print(f"Balance Score: {balance_score:.2f}")
        
        # Record balance score for hyperparameter tuning
        balance_scores.append(balance_score)
        
        # Delay to simulate a 1-second interval
        # time.sleep(1)

    return np.mean(balance_scores)  # Return the average balance score

# Hyperparameter tuning function
def tune_hyperparameters():
    best_score = float('inf')
    best_params = None

    # Define the PID values to test
    kp_values = np.arange(0.01, 2.01, 0.1)  # Example kp values
    ki_values = np.arange(0.01, 2.01, 0.1)  # Generate ki values from 1.0 to 2.0 with step 0.01
    kd_values = np.arange(0.01, 2.01, 0.1)  # Example kd values

    # Declare global variables at the beginning of the function
    global kp, ki, kd

    # Loop over all combinations of PID parameters
    for kp in kp_values:
        for ki in ki_values:
            for kd in kd_values:
                print(f"Testing with kp={kp}, ki={ki}, kd={kd}")

                # Set PID values
                kp, ki, kd = kp, ki, kd  # Update global PID constants

                # Run simulation and get the average balance score
                avg_score = run_simulation()

                # Update best score and parameters (closer to 0 is better)
                if abs(avg_score) < abs(best_score):
                    best_score = avg_score
                    best_params = (kp, ki, kd)

    print(f"Best Parameters: kp={best_params[0]}, ki={best_params[1]}, kd={best_params[2]}")
    print(f"Best Average Balance Score (closest to 0): {best_score}")

# Run hyperparameter tuning
tune_hyperparameters()