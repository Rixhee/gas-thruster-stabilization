import math
import numpy as np

class FourThrustSimulator:
    _instance = None
    
    def __init__(self, initial_pitch=0, initial_roll=0, initial_yaw=0, speed=1):
        # Physical constants
        self.mass = 10.0  # kg
        self.mass *= 2.20462
        self.arm_length = 0.2  # meters (20cm)
        self.gravity = 9.81  # m/s^2
        self.dt = 0.01  # seconds
        self.speed = speed
        self.time = 0
        
        # Moments of inertia (kg*m^2)
        self.I_xx = self.mass * (self.arm_length ** 2) / 2  # Roll
        self.I_yy = self.I_xx  # Pitch
        self.I_zz = self.mass * (self.arm_length ** 2)  # Yaw
        
        # State variables (in radians)
        self.pitch = math.radians(initial_pitch)
        self.roll = math.radians(initial_roll)
        self.yaw = math.radians(initial_yaw)
        
        # Angular velocities (rad/s)
        self.pitch_rate = 0.0
        self.roll_rate = 0.0
        self.yaw_rate = 0.0
        
        # Base damping coefficients
        self.base_damping_pitch = 0.05
        self.base_damping_roll = 0.05
        self.base_damping_yaw = 0.15

        # Damping growth factors
        self.damping_growth_rate = 0.1  # How quickly damping increases
        self.max_damping = 0.5         # Maximum damping coefficient

        # Initialize current damping (will increase over time)
        self.damping_pitch = self.base_damping_pitch
        self.damping_roll = self.base_damping_roll
        self.damping_yaw = self.base_damping_yaw

        # Energy loss tracking
        self.time = 0
        self.last_energy = None

        # Thrust values (N)
        self.thrusts = {
            'front': 0,
            'back': 0,
            'left': 0,
            'right': 0
        }

        # Thruster positions relative to COM (meters)
        self.thruster_positions = {
            'front': (0, self.arm_length/2, 0),
            'back': (0, -self.arm_length/2, 0),
            'left': (-self.arm_length/2, 0, 0),
            'right': (self.arm_length/2, 0, 0)
        }

    def psi_to_newtons(self, psi):
        """Convert PSI to Newtons (approximate conversion)"""
        # 1 PSI ≈ 6.89476 kPa
        # Force = Pressure * Area (assuming 1 square inch nozzle)
        # 1 square inch = 0.00064516 square meters
        return psi * 6.89476 * 0.00064516 * 1000  # Convert to Newtons

    def calculate_torques(self):
        """Calculate torques using proper cross products and moment arms"""
        torques = np.zeros(3)
        
        for thruster, position in self.thruster_positions.items():
            force = np.array([0, 0, self.thrusts[thruster]])
            pos = np.array(position)
            torque = np.cross(pos, force)
            torques += torque
            
        return torques

    def apply_thrust(self, front_psi=0, back_psi=0, left_psi=0, right_psi=0):
        """Apply thrust forces and update the system state"""
        # Convert PSI to Newtons
        self.thrusts['front'] = self.psi_to_newtons(front_psi)
        self.thrusts['back'] = self.psi_to_newtons(back_psi)
        self.thrusts['left'] = self.psi_to_newtons(left_psi)
        self.thrusts['right'] = self.psi_to_newtons(right_psi)
        
        # Calculate torques
        torques = self.calculate_torques()
        
        # Calculate angular accelerations using proper moments of inertia
        pitch_acc = torques[0] / self.I_yy
        roll_acc = torques[1] / self.I_xx
        yaw_acc = torques[2] / self.I_zz
        
        # Update angular velocities
        self.pitch_rate += pitch_acc * self.dt * self.speed
        self.roll_rate += roll_acc * self.dt * self.speed
        self.yaw_rate += yaw_acc * self.dt * self.speed

    def update(self):
        """Update drone state using physics-based equations of motion with increasing energy loss"""
        # Calculate gravitational torques (pendulum effect)
        g_torque_pitch = -(self.mass * self.gravity * self.arm_length) * math.sin(self.pitch)
        g_torque_roll = -(self.mass * self.gravity * self.arm_length) * math.sin(self.roll)

        # Calculate angular accelerations from gravitational torques
        pitch_acc = g_torque_pitch / self.I_yy
        roll_acc = g_torque_roll / self.I_xx

        # Update damping based on time and motion
        # Damping increases with time but is capped at max_damping
        time_factor = min(self.time / 10.0, 1.0)  # Reaches max effect after 10 seconds

        # Calculate current energy (kinetic + potential)
        current_energy = (
            0.5 * (self.I_yy * self.pitch_rate**2 + self.I_xx * self.roll_rate**2) +  # Kinetic
            self.mass * self.gravity * self.arm_length * (1 - math.cos(self.pitch))    # Potential
        )

        # Increase damping based on both time and energy
        energy_factor = 1.0
        if self.last_energy is not None:
            # Increase damping more when energy is higher
            energy_factor = min(current_energy / self.last_energy, 1.5) if self.last_energy > 0 else 1.0

        self.last_energy = current_energy

        # Calculate new damping coefficients
        self.damping_pitch = min(
            self.base_damping_pitch * (1 + self.damping_growth_rate * time_factor * energy_factor),
            self.max_damping
        )
        self.damping_roll = min(
            self.base_damping_roll * (1 + self.damping_growth_rate * time_factor * energy_factor),
            self.max_damping
        )

        # Add damping effects (increasing with time)
        self.pitch_rate -= self.damping_pitch * self.pitch_rate
        self.roll_rate -= self.damping_roll * self.roll_rate
        self.yaw_rate -= self.damping_yaw * self.yaw_rate

        # Update angular velocities using accelerations
        self.pitch_rate += pitch_acc * self.dt * self.speed
        self.roll_rate += roll_acc * self.dt * self.speed

        # Update angles using angular velocities
        self.pitch += self.pitch_rate * self.dt * self.speed
        self.roll += self.roll_rate * self.dt * self.speed
        self.yaw += self.yaw_rate * self.dt * self.speed

        # Constrain angles to prevent unrealistic behavior
        self.pitch = max(min(self.pitch, math.pi/2), -math.pi/2)
        self.roll = max(min(self.roll, math.pi/2), -math.pi/2)
        self.yaw = self.yaw % (2 * math.pi)

        self.time += self.dt

    def is_stable(self, target_pitch=0, target_roll=0, target_yaw=0):
        """Check if the drone is stable within specified tolerances"""
        angle_tolerance = math.radians(0.5)  # 0.5 degrees
        rate_tolerance = 0.1  # rad/s
        
        return (
            abs(self.pitch - math.radians(target_pitch)) < angle_tolerance and
            abs(self.roll - math.radians(target_roll)) < angle_tolerance and
            abs(self.yaw - math.radians(target_yaw)) < angle_tolerance and
            abs(self.pitch_rate) < rate_tolerance and
            abs(self.roll_rate) < rate_tolerance and
            abs(self.yaw_rate) < rate_tolerance
        )

    def get_ypr(self):
        """Get Yaw, Pitch, Roll values in degrees"""
        return [
            math.degrees(self.yaw),
            math.degrees(self.pitch),
            math.degrees(self.roll)
        ]
    
    def get_ypr_rates(self):
        """Get Yaw, Pitch, Roll values in degrees"""
        return [
            math.degrees(self.yaw_rate),
            math.degrees(self.pitch_rate),
            math.degrees(self.roll_rate)
        ]
    
    def get_motion_data(self):
        """Get detailed motion data in degrees and deg/s"""
        return {
            'pitch': math.degrees(self.pitch),
            'pitch_rate': math.degrees(self.pitch_rate),
            'roll': math.degrees(self.roll),
            'roll_rate': math.degrees(self.roll_rate),
            'yaw': math.degrees(self.yaw),
            'yaw_rate': math.degrees(self.yaw_rate),
            'is_stable': self.is_stable()
        }

    def reset(self, initial_pitch=0, initial_roll=0, initial_yaw=0, speed=1):
        """Reset the simulator to initial conditions"""
        self.__init__(initial_pitch, initial_roll, initial_yaw, speed)
        self.time = 0
        self.last_energy = None

    @classmethod
    def get_instance(cls, initial_angle=0, speed=1):
        if cls._instance is None:
            cls._instance = cls(initial_angle, speed)
        return cls._instance

def get_simulated_angle(max_steps=300):
    simulator = FourThrustSimulator.get_instance()
    if not simulator.is_stable() and max_steps > 0:
        simulator.update()
    return simulator.get_motion_data()

import time

# Constants
PSI = 60  # Thrst pressure in PSI

# Constants for PWM
PERIOD = .2  # Control period in seconds
MASS = 10
start_time = time.time()

# Set the target orientations
TARGET_PITCH = 10   # Target pitch in degrees
TARGET_ROLL = 0   # Target roll in degrees
THRESHOLD = 2      # Threshold for minimal pitch and roll adjustments

# PWM function using current time to control on/off behavior and avoid wasting thrust
def pwm(duty_cycle, period=PERIOD):
    """
    Controls when the thruster is active or off based on the required thrust using current time.
    The function ensures no thrust is wasted by adjusting the duty cycle to match the necessary thrust.

    Args:
        duty_cycle (float): The duty cycle (0 to 1).
        period (float): The period (how long each control cycle lasts).

    Returns:
        int: The PSI value (0 or PSI) based on the required thrust and current time.
    """

    # Get the current time in seconds
    current_time = time.time()

    # Calculate how long the thruster should be on during this period
    on_time = duty_cycle * period

    # Determine whether the thruster is on or off
    if (current_time - start_time) % period < on_time:
        return PSI  # Thruster is on (X PSI)
    else:
        return 0  # Thruster is off (0 PSI)

def thrust_control_pwm(current_pitch: float, current_roll: float):
    """
    Controls thrusters using PWM to achieve desired pitch and roll.
    
    Args:
        current_pitch (float): Current pitch angle in degrees.
        current_roll (float): Current roll angle in degrees.
        
    Returns:
        tuple: PWM duty cycle values for all thrusters (between 0 and 1).
    """
    MAX_ERROR = 10  # Maximum error used for normalization
    
    # Calculate errors
    pitch_error = TARGET_PITCH - current_pitch
    roll_error = TARGET_ROLL - current_roll

    # Normalize errors to duty cycles between 0 and 1
    pitch_correction = min(max(abs(pitch_error) / MAX_ERROR, 0), 1)
    roll_correction = min(max(abs(roll_error) / MAX_ERROR, 0), 1)

    # Initialize thruster PWM values
    front_pwm = 0
    back_pwm = 0
    left_pwm = 0
    right_pwm = 0

    # Handle pitch correction
    if pitch_error > 0:  # Nose up → Use front thruster
        front_pwm = pitch_correction
    elif pitch_error < 0:  # Nose down → Use back thruster
        back_pwm = pitch_correction

    # Handle roll correction
    if roll_error > 0:  # Tilt left → Use left thruster
        left_pwm = max(left_pwm, roll_correction)
    elif roll_error < 0:  # Tilt right → Use right thruster
        right_pwm = max(right_pwm, roll_correction)

    # Ensure no opposing thrusters are on simultaneously
    if front_pwm > 0 and back_pwm > 0:
        front_pwm = back_pwm = 0
    if left_pwm > 0 and right_pwm > 0:
        left_pwm = right_pwm = 0

    # Convert PWM duty cycles to PSI values using the PWM function
    thrusts = (
        pwm(front_pwm),
        pwm(back_pwm),
        pwm(left_pwm),
        pwm(right_pwm),
    )

    print(f"Duty Cycles (Front, Back, Left, Right): {front_pwm}, {back_pwm}, {left_pwm}, {right_pwm}")
    print(f"Thrusts (PSI Values): {thrusts}")

    return thrusts

def no_opps(front_thrust, back_thrust, left_thrust, right_thrust):
    """
    Checks and disables opposing thrusters if both are active.
    
    Args:
        front_thrust (float): Front thruster value
        back_thrust (float): Back thruster value
        left_thrust (float): Left thruster value
        right_thrust (float): Right thruster value
        
    Returns:
        tuple: Updated thrust values with opposing pairs disabled
    """
    # Check front vs back thrusters
    if front_thrust > 0 and back_thrust > 0:
        front_thrust = back_thrust = 0
        
    # Check left vs right thrusters
    if left_thrust > 0 and right_thrust > 0:
        left_thrust = right_thrust = 0
        
    return front_thrust, back_thrust, left_thrust, right_thrust

def thrust_control(current_pitch: float, current_roll: float):
    """
    Controls thrusters to achieve target pitch and roll angles.
    
    Thrust effects:
    - FRONT thruster ON: pitch increases (nose up)
    - BACK thruster ON: pitch decreases (nose down)
    - LEFT thruster ON: roll increases (tilt left)
    - RIGHT thruster ON: roll decreases (tilt right)
    
    Args:
        current_pitch (float): Current pitch angle in degrees
        current_roll (float): Current roll angle in degrees
        
    Returns:
        tuple: Calculated thrust values for all four thrusters
    """
    # Initialize all thrusters to zero
    front_thrust = back_thrust = left_thrust = right_thrust = 0
    
    # Calculate errors
    pitch_error = TARGET_PITCH - current_pitch
    roll_error = TARGET_ROLL - current_roll
    
    # If both pitch and roll errors are below the threshold, set all thrusts to zero
    if abs(pitch_error) < THRESHOLD and abs(roll_error) < THRESHOLD:
        print("Pitch and Roll errors are within threshold. No thrust applied.")
        return 0, 0, 0, 0
    
    print(f"Pitch Error: {pitch_error}")
    print(f"Roll Error: {roll_error}")
    
    # Pitch correction (nose up or down)
    if pitch_error > 0:  # Need to increase pitch (nose up) - activate front thruster
        front_thrust = PSI
    elif pitch_error < 0:  # Need to decrease pitch (nose down) - activate back thruster
        back_thrust = PSI
        
    # Roll correction (tilt left or right)
    if roll_error > 0:  # Need to increase roll (tilt left) - activate left thruster
        left_thrust = PSI
    elif roll_error < 0:  # Need to decrease roll (tilt right) - activate right thruster
        right_thrust = PSI
    
    # Prevent opposing thrusters from firing simultaneously (no-op)
    thrusts = no_opps(
        front_thrust,
        back_thrust,
        left_thrust,
        right_thrust
    )
    
    # Print final thrust values
    print(f"Thrusts (Front, Back, Left, Right): {thrusts}")
    
    return thrusts

# Test function to iterate through test cases
def test_thrust_control():
    test_cases = [
        {"current_pitch": 15, "current_roll": 0},  # Nose up
        {"current_pitch": -15, "current_roll": 0},  # Nose down
        {"current_pitch": 0, "current_roll": 15},  # Tilt left
        {"current_pitch": 0, "current_roll": -15},  # Tilt right
        {"current_pitch": 10, "current_roll": -10},  # Combined
        {"current_pitch": 0, "current_roll": 0},  # Balanced
    ]

    for case in test_cases:
        current_pitch = case["current_pitch"]
        current_roll = case["current_roll"]

        thrusts = thrust_control_pwm(current_pitch, current_roll)
        print(
            f"Pitch: {current_pitch}, Roll: {current_roll} -> Thrusts (FL, FR, BL, BR): {thrusts}"
        )


# Run the test
test_thrust_control()

# import time

# # Target pitch and roll values
# target_pitch = 0
# target_roll = 0

# # Run the simulation to debug the correction logic
# def run_simulation(current_pitch, current_roll, steps=300):
#     print(f"Starting simulation with initial pitch: {current_pitch}° and roll: {current_roll}°")
#     print("Step | Pitch Error | Roll Error | Target Pitch | Target Roll | Thrusts (Front, Back, Left, Right) | Current Pitch | Current Roll | Yaw")
#     print("-" * 120)
    
#     # Initialize the FourThrustSimulator with the initial pitch and roll
#     simulator = FourThrustSimulator.get_instance(speed=10)

#     for step in range(steps):
#         # Get the current pitch, roll, and yaw from the simulator
#         current_pitch, current_roll, current_yaw = simulator.get_ypr()

#         # Calculate thrusts to bring pitch and roll toward targets
#         front_left_thrust, front_right_thrust, back_left_thrust, back_right_thrust = thrust_control(current_pitch, current_roll)
        
#         # Print the current state, errors, and thrust values for debugging
#         print(f"{step + 1:4d} | "
#               f"{target_pitch - current_pitch:+8.2f} | {target_roll - current_roll:+8.2f} | "
#               f"{target_pitch:+8.2f} | {target_roll:+8.2f} | "
#               f"{front_left_thrust:3d}, {front_right_thrust:3d}, {back_left_thrust:3d}, {back_right_thrust:3d} | "
#               f"{current_pitch:+8.2f} | {current_roll:+8.2f} | {current_yaw:+8.2f}")
        

#         # Apply thrusts to the simulator
#         simulator.apply_thrust(front_left_thrust, front_right_thrust, back_left_thrust, back_right_thrust)

#         # Update the simulator's state to apply physics and thrusts
#         simulator.update()
#         print(simulator.get_ypr())
        
#         # Check if the simulator has stabilized
#         if simulator.is_stable(simulator.get_ypr()[0],simulator.get_ypr()[1],simulator.get_ypr()[2]):
#             print("\nisStable!")
#             break
        
#         time.sleep(0.1)  # Small delay for readability (adjust or remove as needed)
    
#     print("Simulation complete.")

# # Usage example
# if __name__ == "__main__":
#     current_pitch = float(input("Enter current pitch (-50 to 50 degrees): "))
#     current_roll = float(input("Enter current roll (-50 to 50 degrees): "))
    
#     if -50 <= current_pitch <= 50 and -50 <= current_roll <= 50:
#         run_simulation(current_pitch, current_roll)
#     else:
#         print("Please enter valid pitch and roll values between -50 and 50 degrees.")


import numpy as np
import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go

# Initialize the app
app = dash.Dash(__name__)
graphid = "drone-graph"

# Create a layout with a graph, interval, and controls
app.layout = html.Div([
    html.Div([
        html.Div(id='ypr-display', style={'fontSize': 18, 'margin': '10px'}),
    ]),
    html.Div([
        html.Button('Reset', id='reset-button', n_clicks=0),
        dcc.Input(
            id='initial-pitch',
            type='number',
            min=-90,
            max=90,
            placeholder='Initial Pitch Angle (-90 to 90)',
            value=10,
            style={'margin': '10px'}
        ),
        dcc.Input(
            id='initial-roll',
            type='number',
            min=-90,
            max=90,
            placeholder='Initial Roll Angle (-90 to 90)',
            value=0,
            style={'margin': '10px'}
        ),
        dcc.Input(
            id='initial-yaw',
            type='number',
            min=-180,
            max=180,
            placeholder='Initial Yaw Angle (-180 to 180)',
            value=0,
            style={'margin': '10px'}
        ),
        html.Div([
            html.Label('Front Thruster (PSI):'),
            dcc.Input(
                id='front-thrust',
                type='number',
                min=0,
                max=100,
                value=0,
                style={'width': '50px', 'margin': '10px'}
            ),
            html.Label('Back Thruster (PSI):'),
            dcc.Input(
                id='back-thrust',
                type='number',
                min=0,
                max=100,
                value=0,
                style={'width': '50px', 'margin': '10px'}
            ),
            html.Label('Left Thruster (PSI):'),
            dcc.Input(
                id='left-thrust',
                type='number',
                min=0,
                max=100,
                value=0,
                style={'width': '50px', 'margin': '10px'}
            ),
            html.Label('Right Thruster (PSI):'),
            dcc.Input(
                id='right-thrust',
                type='number',
                min=0,
                max=100,
                value=0,
                style={'width': '50px', 'margin': '10px'}
            ),
            html.Button('Orbit Left', id='orbit-left', n_clicks=0),
            html.Button('Orbit Right', id='orbit-right', n_clicks=0),
            html.Button('Orbit Up', id='orbit-up', n_clicks=0),
            html.Button('Orbit Down', id='orbit-down', n_clicks=0),
        ]),
    ], style={'margin': '10px'}),

    dcc.Store(id='camera-store', data={'azimuth': 25, 'elevation': 30, 'distance': 2}),
    dcc.Graph(id=graphid),
    dcc.Interval(
        id='update-interval',
        interval=20, # Xms speed
        n_intervals=0
    )
])

# Initialize the simulator instance
simulator = FourThrustSimulator.get_instance()

def create_rotation_matrix(yaw, pitch, roll):
    """Create a proper rotation matrix using ZYX (yaw, pitch, roll) convention"""
    # Convert angles to radians if they aren't already
    yaw = np.radians(yaw)
    pitch = np.radians(pitch)
    roll = np.radians(roll)
    
    # Individual rotation matrices
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])
    
    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])
    
    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])
    
    # Combine rotations in ZYX order (yaw, pitch, roll)
    R = Rz @ Ry @ Rx
    return R

@app.callback(
    [Output(graphid, 'figure'), Output('camera-store', 'data'), Output('ypr-display', 'children')],
    [Input('update-interval', 'n_intervals'), 
     Input('front-thrust', 'value'), 
     Input('back-thrust', 'value'),
     Input('left-thrust', 'value'), 
     Input('right-thrust', 'value'), 
     Input('orbit-left', 'n_clicks'),
     Input('orbit-right', 'n_clicks'), 
     Input('orbit-up', 'n_clicks'), 
     Input('orbit-down', 'n_clicks')],
    [State('camera-store', 'data')]
)
def update_drone_graph(n, front_thrust, back_thrust, left_thrust, right_thrust,
                       orbit_left, orbit_right, orbit_up, orbit_down, camera_data):
    
    if front_thrust != 0 or back_thrust != 0 or left_thrust != 0 or right_thrust != 0:
        # Apply thrust to the simulator
        simulator.apply_thrust(front_thrust, back_thrust, left_thrust, right_thrust)
    else:
        yaw, pitch, roll = simulator.get_ypr()
        yaw_rate, pitch_rate, roll_rate = simulator.get_ypr_rates()
        front_thrust, back_thrust, left_thrust, right_thrust = thrust_control(pitch, roll)
        # front_thrust, back_thrust, left_thrust, right_thrust = thrust_control(pitch, roll)
        simulator.apply_thrust(front_thrust, back_thrust, left_thrust, right_thrust)
    
    simulator.update()

    # Retrieve yaw, pitch, and roll from the simulator
    yaw, pitch, roll = simulator.get_ypr()
    # Update the YPR display text
    ypr_text = f"Pitch: {pitch:.2f}°, Roll: {roll:.2f}°, Yaw: {yaw:.2f}°"
    
    # Define the drone frame vertices (scaled down from previous version)
    length = 0.5  # Smaller size for better visualization
    vertices = np.array([
        [length, 0, 0],  # Front
        [-length, 0, 0], # Back
        [0, -length, 0], # Left
        [0, length, 0]   # Right
    ])
    
    # Create rotation matrix and apply to vertices
    R = create_rotation_matrix(yaw, pitch, roll)
    rotated_vertices = vertices @ R.T
    
    # Extract coordinates
    x_values = rotated_vertices[:, 0]
    y_values = rotated_vertices[:, 1]
    z_values = rotated_vertices[:, 2]
    
    # Scale the cross size relative to the drone frame
    cross_length = length/3

    # Define cross vertices
    cross_vertices = np.array([
        # Z-axis (vertical)
        [0, 0, -cross_length],
        [0, 0, cross_length],
        # Cross X-axis
        [-cross_length, -cross_length, 0],
        [cross_length, cross_length, 0],
        # Cross Y-axis
        [cross_length, -cross_length, 0],
        [-cross_length, cross_length, 0]
    ])

    # Apply the same rotation matrix to cross vertices
    rotated_cross = cross_vertices @ R.T
    
    # Create the visualization traces
    traces = [
        # Drone frame
        go.Scatter3d(
            x=np.array([x_values[0], x_values[1]]),  # Front to Back
            y=np.array([y_values[0], y_values[1]]),
            z=np.array([z_values[0], z_values[1]]),
            mode='lines',
            line=dict(color='black', width=4),
            name='Front-Back Connection'
        ),
        go.Scatter3d(
            x=np.array([x_values[2], x_values[3]]),  # Left to Right
            y=np.array([y_values[2], y_values[3]]),
            z=np.array([z_values[2], z_values[3]]),
            mode='lines',
            line=dict(color='black', width=4),
            name='Left-Right Connection'
        ),
        
        # Motors
        go.Scatter3d(
            x=x_values,
            y=y_values,
            z=z_values,
            mode='markers+text',
            marker=dict(size=8, color='orange', symbol='circle'),
            text=[f'Back: {back_thrust}', f'Front: {front_thrust}', f'Left: {left_thrust}', f'Right: {right_thrust}'],
            textposition='bottom center',
            name='Motors'
        ),
        
        # Z axis (vertical)
        go.Scatter3d(
            x=rotated_cross[0:2, 0],
            y=rotated_cross[0:2, 1],
            z=rotated_cross[0:2, 2],
            mode='lines',
            line=dict(color='red', width=2),
            name='Z axis'
        ),
        # Cross X axis
        go.Scatter3d(
            x=rotated_cross[2:4, 0],
            y=rotated_cross[2:4, 1],
            z=rotated_cross[2:4, 2],
            mode='lines',
            line=dict(color='blue', width=2),
            name='X axis'
        ),
        # Ctross Y axis
        go.Scatter3d(
            x=rotated_cross[4:6, 0],
            y=rotated_cross[4:6, 1],
            z=rotated_cross[4:6, 2],
            mode='lines',
            line=dict(color='blue', width=2),
            name='Y axis'
        )
    ]

    # Update camera position
    camera_data['azimuth'] += (orbit_right - orbit_left) * 2
    camera_data['elevation'] += (orbit_up - orbit_down) * 2
    
    # Create the layout
    layout = go.Layout(
        scene=dict(
            xaxis=dict(range=[-1, 1], showgrid=True, zeroline=True),
            yaxis=dict(range=[-1, 1], showgrid=True, zeroline=True),
            zaxis=dict(range=[-1, 1], showgrid=True, zeroline=True),
            camera=dict(
                eye=dict(
                    x=camera_data['distance'] * np.cos(np.radians(camera_data['azimuth'])) * 
                      np.cos(np.radians(camera_data['elevation'])),
                    y=camera_data['distance'] * np.sin(np.radians(camera_data['azimuth'])) * 
                      np.cos(np.radians(camera_data['elevation'])),
                    z=camera_data['distance'] * np.sin(np.radians(camera_data['elevation']))
                )
            ),
            aspectmode='cube'
        ),
        title='Drone Simulation',
        showlegend=False,
        margin=dict(l=0, r=0, t=40, b=0)
    )
    
    return {'data': traces, 'layout': layout}, camera_data, ypr_text

@app.callback(
    Output('update-interval', 'n_intervals'),
    Input('reset-button', 'n_clicks'),
    [State('initial-pitch', 'value'),
     State('initial-roll', 'value'),
     State('initial-yaw', 'value')]
)
def reset_simulation(n_clicks, initial_pitch, initial_roll, initial_yaw):
    if n_clicks > 0:
        simulator.reset(initial_pitch=initial_pitch, initial_roll=initial_roll, initial_yaw=initial_yaw)
    return 0

if __name__ == '__main__':
    app.run_server(debug=True)