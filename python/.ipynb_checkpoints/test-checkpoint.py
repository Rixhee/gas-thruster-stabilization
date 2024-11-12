# seesaw.py
import math

thresh = .2

class SeeSawSimulator:
    _instance = None
    
    def __init__(self, initial_angle, speed=1):
        self.angle = initial_angle
        self.angular_velocity = 0.0
        self.gravity = 9.81
        self.length = 5.0  # Length in meters
        self.base_damping = 0.05
        self.dt = 1
        self.time = 0
        self.speed = speed
        
        # Thruster properties
        self.left_thrust = 0   # Thrust force in Newtons (from PSI)
        self.right_thrust = 0
        self.thrust_point = self.length-.5  # Distance from center where thrusters are mounted (meters)
        
        # For IMU-like readings
        self.yaw = 0  # 0 since we're only simulating 2D motion
        self.pitch = self.angle  # primary/only motion axis
        self.roll = 0  # 0 since we're only simulating 2D motion
        
    def psi_to_newtons(self, psi):
        """Convert PSI to Newtons (approximate conversion)"""
        # 1 PSI ≈ 6.89476 kPa
        # Force = Pressure * Area (assuming 1 square inch nozzle)
        # 1 square inch = 0.00064516 square meters
        return (psi * 6.89476 * 0.00064516 * 1000) / 300  # Convert to Newtons
        
    def calculate_damping(self):
        velocity_factor = abs(self.angular_velocity) * 0.1
        time_factor = min(0.1, self.time * 0.0001)
        return self.base_damping * (1 + velocity_factor + time_factor)
        
    def calculate_thrust_torque(self):
        # Calculate torque from thrusters
        # Positive torque rotates counterclockwise
        left_torque = -self.left_thrust * self.thrust_point  # Negative because left thrust creates clockwise rotation
        right_torque = self.right_thrust * self.thrust_point
        
        # Convert to angular acceleration
        # τ = I*α, where I is moment of inertia and α is angular acceleration
        # For a rod, I = (1/12) * mass * length^2
        # Assuming mass of 10kg for the seesaw
        mass = 10.0
        moment_of_inertia = (1/12) * mass * self.length**2
        thrust_angular_acc = (left_torque + right_torque) / moment_of_inertia
        
        return thrust_angular_acc
        
    def calculate_acceleration(self):
        # Gravitational acceleration
        angle_rad = math.radians(self.angle)
        angular_acceleration = -(self.gravity / self.length) * math.sin(angle_rad)
        
        # Damping
        current_damping = self.calculate_damping()
        angular_acceleration -= current_damping * self.angular_velocity
        
        # Add thrust effects
        angular_acceleration += self.calculate_thrust_torque()
        
        return angular_acceleration
    
    def update(self):
        acceleration = self.calculate_acceleration()
        self.angular_velocity += acceleration * self.dt * self.speed
        self.angle += self.angular_velocity * self.dt * self.speed
        
        if abs(self.angle) > 50:
            self.angle = 50 if self.angle > 0 else -50
            bounce_factor = 0.5 / (1 + abs(self.angular_velocity) * 0.1)
            self.angular_velocity *= bounce_factor
        
        self.time += self.dt
        
        # Reset thrusters after each update (they need to be explicitly set each time)
        self.left_thrust = 0
        self.right_thrust = 0
        
        # Update IMU-like values
        self.pitch = self.angle
            
    def is_stable(self):
        return abs(self.angle) < thresh and abs(self.angular_velocity) < thresh
    
    def apply_thrust(self, left_psi=0, right_psi=0):
        """Apply thrust to the seesaw (in PSI)"""
        self.left_thrust = self.psi_to_newtons(left_psi)
        self.right_thrust = self.psi_to_newtons(right_psi)
        
    # IMU-like getter functions
    def get_ypr(self):
        """Get Yaw, Pitch, Roll values (similar to your IMU)"""
        return [self.yaw, self.pitch, self.roll]
    
    def get_motion_data(self):
        """Get detailed motion data"""
        return {
            'angle': self.angle,
            'angular_velocity': self.angular_velocity,
            'angular_acceleration': self.angular_acceleration,
            'is_stable': self.is_stable()
        }
    
    @classmethod
    def get_instance(cls, initial_angle=0, speed=1):
        if cls._instance is None:
            cls._instance = SeeSawSimulator(initial_angle, speed)
        return cls._instance

def get_simulated_angle(max_steps=300, left_thrust=0, right_thrust=0):
    simulator = SeeSawSimulator.get_instance()
    if not simulator.is_stable() and max_steps > 0:
        simulator.apply_thrust(left_thrust, right_thrust)
        simulator.update()
    return simulator.angle, simulator

def reset_seesaw(initial_angle=0, speed=1):
    SeeSawSimulator._instance = SeeSawSimulator(initial_angle, speed=speed)
    return SeeSawSimulator._instance.angle

psi = 60

# Thrust control function
def thrust_control(correction: float):
    if correction == 0:
        return (0, 0)  # Both thrusters off
    elif correction > 0:
        return (psi, 0)  # Left thruster on, right off
    else:
        return (0, psi)  # Right thruster on, left off

# Correction function (PID control)
def calculate_correction(pitch: float, previous_error: float, integral: float, dt: float, kp: float, ki: float, kd: float):
    error = pitch  # The error is simply the current pitch angle
    integral += error * dt  # Integral is the sum of all past errors over time
    derivative = (error - previous_error) / dt  # Derivative is the rate of change of the error
    correction = kp * error + ki * integral + kd * derivative
    return correction, error, integral  # Return the correction and updated error and integral for the next loop

# PID Constants
kp = 100
ki = 0.1
kd = 0.1

# Initialize variables
previous_error = 0
integral = 0
dt = 0.1 

pitch = 10

# Calculate the correction using PID
correction, previous_error, integral = calculate_correction(pitch, previous_error, integral, dt, kp, ki, kd)

# Control the thrusters based on the correction value
left_thrust, right_thrust = thrust_control(correction)

# Print the thrust control results
print(f"Left Thrust: {left_thrust}, Right Thrust: {right_thrust}")

# app.py
import numpy as np
import dash
from dash import dcc, html
from dash.dependencies import Input, Output, State
import plotly.graph_objs as go

# Initialize the app
app = dash.Dash(__name__)
graphid = "thrust-graph"

# Create a layout with a graph, interval, and controls
app.layout = html.Div([
    html.Div([
        html.Button('Reset', id='reset-button', n_clicks=0),
        dcc.Input(
            id='angle-input',
            type='number',
            min=-50,
            max=50,
            placeholder='Initial angle (-50 to 50)',
            value=10,
            style={'margin': '10px'}
        ),
        html.Div([
            html.Label('Left Thruster (PSI):'),
            dcc.Input(
                id='left-thrust',
                type='number',
                min=0,
                max=100,
                value=0,
                style={'margin': '10px'}
            ),
            html.Label('Right Thruster (PSI):'),
            dcc.Input(
                id='right-thrust',
                type='number',
                min=0,
                max=100,
                value=0,
                style={'margin': '10px'}
            ),
        ]),
    ], style={'margin': '10px'}),
    
    dcc.Graph(id=graphid),
    dcc.Interval(
        id='update-interval',
        interval=20,  # Update every Xms
        n_intervals=0
    )
])

@app.callback(
    Output(graphid, 'figure'),
    Input('update-interval', 'n_intervals'),
    Input('left-thrust', 'value'),
    Input('right-thrust', 'value')
)

def update_seesaw_graph(n, left_thrust, right_thrust):
    global previous_error, pitch, integral
    if not previous_error:
        previous_error = 0
    if not pitch:
        pitch = 0
    if not integral:
        integral = 0
    # Set PID constants
    kp = 100
    ki = 0.1
    kd = 0.1
    dt = 0.1  # Time step, for example 100ms

    # # Simulate the current pitch angle (can be fetched from the simulation or IMU)
    # pitch = float(simulator.get_ypr[1] or 0)

    # Calculate the correction using PID control
    correction, previous_error, integral = calculate_correction(pitch, previous_error, integral, dt, kp, ki, kd)

    # Get the left and right thrust values based on the correction
    left_thrust, right_thrust = thrust_control(correction)

    # Get the updated angle from seesaw simulator
    angle, simulator = get_simulated_angle(
        left_thrust=left_thrust,
        right_thrust=right_thrust
    )
    
    pitch = simulator.get_ypr()[1]
    
    # Calculate the endpoints of the seesaw
    length = 100
    x_values = [-length / 2, length / 2]
    y_values = [-np.tan(np.radians(angle)) * length / 2,
                np.tan(np.radians(angle)) * length / 2]

    traces = [
        # Seesaw beam
        go.Scatter(
            x=x_values,
            y=y_values,
            mode='lines',
            line=dict(color='blue', width=6),
            name='Seesaw'
        ),
        # Pivot point
        go.Scatter(
            x=[0],
            y=[0],
            mode='markers',
            marker=dict(size=15, color='red'),
            name='Pivot Point'
        ),
        # Angle indicator
        go.Scatter(
            x=[length / 2],
            y=[np.tan(np.radians(angle)) * length / 2],
            mode='text',
            text=[f'Angle: {angle:.1f}°'],
            textposition='top right',
            name='Angle'
        )
    ]

    # Add thruster indicators if active
    if left_thrust:
        traces.append(go.Scatter(
            x=[-length / 2],
            y=[-np.tan(np.radians(angle)) * length / 2],
            mode='markers+text',
            marker=dict(size=10, color='orange'),
            text=[f'↑\n{left_thrust} PSI'],
            textposition='bottom center',
            name='Left Thruster'
        ))

    if right_thrust:
        traces.append(go.Scatter(
            x=[length / 2],
            y=[np.tan(np.radians(angle)) * length / 2],
            mode='markers+text',
            marker=dict(size=10, color='orange'),
            text=[f'↑\n{right_thrust} PSI'],
            textposition='bottom center',
            name='Right Thruster'
        ))

    layout = go.Layout(
        xaxis=dict(
            range=[-length, length],
            showgrid=True,
            zeroline=True
        ),
        yaxis=dict(
            range=[-length, length],
            showgrid=True,
            zeroline=True,
            scaleanchor='x',
            scaleratio=1
        ),
        title='Real-time Seesaw Simulation with Thrusters',
        showlegend=False,
        margin=dict(l=40, r=40, t=40, b=40)
    )

    return {'data': traces, 'layout': layout}


@app.callback(
    Output('update-interval', 'n_intervals'),
    Input('reset-button', 'n_clicks'),
    State('angle-input', 'value')
)

def reset_simulation(n_clicks, initial_angle):
    if n_clicks > 0:
        reset_seesaw(float(initial_angle) if initial_angle is not None else 0.0)
    return 0

if __name__ == '__main__':
    app.run_server(debug=True)