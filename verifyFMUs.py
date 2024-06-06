import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.animation import FuncAnimation
from fmpy import simulate_fmu
from scipy.interpolate import interp1d
import pandas as pd
import os

# Simulation parameters
start_time = 0.0
stop_time = 60.0 # 60.0 seconds = 1.0 minutes
step_size = 0.1

# Time vector
time = np.arange(start_time, stop_time + step_size, step_size)

# Constants for later use
# IMU Plot constant
gravity = 9.81  # Gravitational acceleration [m/s^2]



# Input functions and parameters (START) --------------------------------------------------
# Thruster functions ----------
def input_bow_force(t):
    return 0.0

def input_bow_angle(t):
    return 0.0

def input_port_force(t):
    return 500.0

def input_port_angle(t):
    return 30 / (t * 100 + 1)

def input_starboard_force(t):
    return 500.0

def input_starboard_angle(t):
    return 30 / (t * 100 + 1)

# Environment functions ----------
def input_water_current_speed(t):
    return 0.0

def input_water_current_angle(t):
    return 0.0

def input_wind_speed(t):
    return 0.0

def input_wind_angle(t):
    return 0.0

def input_wave_height_front(t):
    return 0.2 * np.sin(1.0 * t + -0.2)

def input_wave_height_left(t):
    return 0.2 * np.sin(1.0 * t + -0.1)

def input_wave_height_center(t):
    return 0.2 * np.sin(1.0 * t + 0.0)

def input_wave_height_right(t):
    return 0.2 * np.sin(1.0 * t + 0.2)

def input_wave_height_back(t):
    return 0.2 * np.sin(1.0 * t + 0.4)
# Input functions and parameters (STOP) --------------------------------------------------



# Prepare input data for thrusters
def prepare_thruster_input_data(force_func, angle_func):
    input_data = np.zeros(len(time), dtype=[('time', np.double), ('InputForce', np.double), ('InputAngle', np.double)])
    input_data['time'] = time
    input_data['InputForce'] = force_func(time)
    input_data['InputAngle'] = angle_func(time)
    return input_data

# Simulate each thruster FMU
def simulate_thruster(fmu_path, fmu_name, input_data):
    result = simulate_fmu(fmu_path, start_time=start_time, stop_time=stop_time, input=input_data, output=['OutputForce', 'OutputAngle'])

    # Interpolate input data to match the time points in result
    interp_force = interp1d(input_data['time'], input_data['InputForce'], kind='linear', fill_value="extrapolate")
    interp_angle = interp1d(input_data['time'], input_data['InputAngle'], kind='linear', fill_value="extrapolate")

    input_force_interp = interp_force(result['time'])
    input_angle_interp = interp_angle(result['time'])

    # Plot the results
    plt.figure()
    plt.subplot(2, 1, 1)
    plt.plot(result['time'], input_force_interp, label='InputForce')
    plt.plot(result['time'], result['OutputForce'], label='OutputForce')
    plt.xlabel('Time [s]')
    plt.ylabel('Force [N]')
    plt.title(f'{fmu_name} - Force')
    plt.legend()
    plt.grid(True)

    plt.subplot(2, 1, 2)
    plt.plot(result['time'], input_angle_interp, label='InputAngle')
    plt.plot(result['time'], result['OutputAngle'], label='OutputAngle')
    plt.xlabel('Time [s]')
    plt.ylabel('Angle [deg]')
    plt.title(f'{fmu_name} - Angle')
    plt.legend()
    plt.grid(True)

    plt.tight_layout()
    plt.show()
    
    return result

# Bow thruster
bow_input_data = prepare_thruster_input_data(input_bow_force, input_bow_angle)
bow_result = simulate_thruster('build/ThrusterBowFMU.fmu', 'Thruster Bow', bow_input_data)
bow_force = bow_result['OutputForce']
bow_angle = bow_result['OutputAngle']

# Port thruster
port_input_data = prepare_thruster_input_data(input_port_force, input_port_angle)
port_result = simulate_thruster('build/ThrusterPortFMU.fmu', 'Thruster Port', port_input_data)
port_force = port_result['OutputForce']
port_angle = port_result['OutputAngle']

# Starboard thruster
starboard_input_data = prepare_thruster_input_data(input_starboard_force, input_starboard_angle)
starboard_result = simulate_thruster('build/ThrusterStarboardFMU.fmu', 'Thruster Starboard', starboard_input_data)
starboard_force = starboard_result['OutputForce']
starboard_angle = starboard_result['OutputAngle']

# Interpolate thruster outputs to match the hull simulation time vector
def interpolate_thruster_data(thruster_result):
    time_thruster = np.linspace(start_time, stop_time, len(thruster_result['OutputForce']))
    interp_force = interp1d(time_thruster, thruster_result['OutputForce'], kind='linear', fill_value="extrapolate")
    interp_angle = interp1d(time_thruster, thruster_result['OutputAngle'], kind='linear', fill_value="extrapolate")
    return interp_force(time), interp_angle(time)

bow_force, bow_angle = interpolate_thruster_data(bow_result)
port_force, port_angle = interpolate_thruster_data(port_result)
starboard_force, starboard_angle = interpolate_thruster_data(starboard_result)

# Prepare input data for the hull simulation
input_data = np.zeros(len(time), dtype=[('time', np.double), 
                                        ('InputThrusterBowForce', np.double), 
                                        ('InputThrusterBowAngle', np.double),
                                        ('InputThrusterPortForce', np.double), 
                                        ('InputThrusterPortAngle', np.double),
                                        ('InputThrusterStarboardForce', np.double),
                                        ('InputThrusterStarboardAngle', np.double),
                                        ('InputWaterCurrentSpeed', np.double),
                                        ('InputWaterCurrentAngle', np.double),
                                        ('InputWindSpeed', np.double),
                                        ('InputWindAngle', np.double),
                                        ('InputWaveHeight[0]', np.double),
                                        ('InputWaveHeight[1]', np.double),
                                        ('InputWaveHeight[2]', np.double),
                                        ('InputWaveHeight[3]', np.double),
                                        ('InputWaveHeight[4]', np.double)])

input_data['time'] = time
input_data['InputThrusterBowForce'] = bow_force
input_data['InputThrusterBowAngle'] = bow_angle
input_data['InputThrusterPortForce'] = port_force
input_data['InputThrusterPortAngle'] = port_angle
input_data['InputThrusterStarboardForce'] = starboard_force
input_data['InputThrusterStarboardAngle'] = starboard_angle
input_data['InputWaterCurrentSpeed'] = input_water_current_speed(time)
input_data['InputWaterCurrentAngle'] = input_water_current_angle(time)
input_data['InputWindSpeed'] = input_wind_speed(time)
input_data['InputWindAngle'] = input_wind_angle(time)
input_data['InputWaveHeight[0]'] = input_wave_height_front(time)
input_data['InputWaveHeight[1]'] = input_wave_height_left(time)
input_data['InputWaveHeight[2]'] = input_wave_height_center(time)
input_data['InputWaveHeight[3]'] = input_wave_height_right(time)
input_data['InputWaveHeight[4]'] = input_wave_height_back(time)

# Function to simulate and plot results for the Hull FMU
def simulate_and_plot_hull(fmu_path):
    result = simulate_fmu(fmu_path, start_time=start_time, stop_time=stop_time, input=input_data, output=[
        'OutputHullPosition[0]', 'OutputHullPosition[1]', 'OutputHullPosition[2]', 
        'OutputHullPosition[3]', 'OutputHullPosition[4]', 'OutputHullPosition[5]',
        'OutputHullVelocity[0]', 'OutputHullVelocity[1]', 'OutputHullVelocity[2]',
        'OutputHullVelocity[3]', 'OutputHullVelocity[4]', 'OutputHullVelocity[5]'
    ])

    # Verify the structure of the result
    print(result.dtype.names)  # Should print the names of the fields in the structured array

    # Extract output positions and orientations
    x_position = result['OutputHullPosition[0]']
    y_position = result['OutputHullPosition[1]']
    z_position = result['OutputHullPosition[2]']
    roll_position = result['OutputHullPosition[3]']
    pitch_position = result['OutputHullPosition[4]']
    yaw_position = result['OutputHullPosition[5]']
    x_velocity = result['OutputHullVelocity[0]']
    y_velocity = result['OutputHullVelocity[1]']
    z_velocity = result['OutputHullVelocity[2]']
    roll_velocity = result['OutputHullVelocity[3]']
    pitch_velocity = result['OutputHullVelocity[4]']
    yaw_velocity = result['OutputHullVelocity[5]']

    # Interpolate positions to match the time steps
    interp_x = interp1d(np.linspace(start_time, stop_time, len(x_position)), x_position, kind='linear', fill_value="extrapolate")
    interp_y = interp1d(np.linspace(start_time, stop_time, len(y_position)), y_position, kind='linear', fill_value="extrapolate")
    interp_z = interp1d(np.linspace(start_time, stop_time, len(z_position)), z_position, kind='linear', fill_value="extrapolate")
    interp_roll = interp1d(np.linspace(start_time, stop_time, len(roll_position)), roll_position, kind='linear', fill_value="extrapolate")
    interp_pitch = interp1d(np.linspace(start_time, stop_time, len(pitch_position)), pitch_position, kind='linear', fill_value="extrapolate")
    interp_yaw = interp1d(np.linspace(start_time, stop_time, len(yaw_position)), yaw_position, kind='linear', fill_value="extrapolate")
    interp_x_vel = interp1d(np.linspace(start_time, stop_time, len(x_velocity)), x_velocity, kind='linear', fill_value="extrapolate")
    interp_y_vel = interp1d(np.linspace(start_time, stop_time, len(y_velocity)), y_velocity, kind='linear', fill_value="extrapolate")
    interp_z_vel = interp1d(np.linspace(start_time, stop_time, len(z_velocity)), z_velocity, kind='linear', fill_value="extrapolate")
    interp_roll_vel = interp1d(np.linspace(start_time, stop_time, len(roll_velocity)), roll_velocity, kind='linear', fill_value="extrapolate")
    interp_pitch_vel = interp1d(np.linspace(start_time, stop_time, len(pitch_velocity)), pitch_velocity, kind='linear', fill_value="extrapolate")
    interp_yaw_vel = interp1d(np.linspace(start_time, stop_time, len(yaw_velocity)), yaw_velocity, kind='linear', fill_value="extrapolate")

    x_position_resampled = interp_x(time)
    y_position_resampled = interp_y(time)
    z_position_resampled = interp_z(time)
    roll_position_resampled = interp_roll(time)
    pitch_position_resampled = interp_pitch(time)
    yaw_position_resampled = interp_yaw(time)
    x_velocity_resampled = interp_x_vel(time)
    y_velocity_resampled = interp_y_vel(time)
    z_velocity_resampled = interp_z_vel(time)
    roll_velocity_resampled = interp_roll_vel(time)
    pitch_velocity_resampled = interp_pitch_vel(time)
    yaw_velocity_resampled = interp_yaw_vel(time)

    # Check lengths of arrays
    print(f"Length of x_position_resampled: {len(x_position_resampled)}")
    print(f"Length of y_position_resampled: {len(y_position_resampled)}")
    print(f"Length of z_position_resampled: {len(z_position_resampled)}")
    print(f"Length of roll_position_resampled: {len(roll_position_resampled)}")
    print(f"Length of pitch_position_resampled: {len(pitch_position_resampled)}")
    print(f"Length of yaw_position_resampled: {len(yaw_position_resampled)}")
    print(f"Length of x_velocity_resampled: {len(x_velocity_resampled)}")
    print(f"Length of y_velocity_resampled: {len(y_velocity_resampled)}")
    print(f"Length of z_velocity_resampled: {len(z_velocity_resampled)}")
    print(f"Length of roll_velocity_resampled: {len(roll_velocity_resampled)}")
    print(f"Length of pitch_velocity_resampled: {len(pitch_velocity_resampled)}")
    print(f"Length of yaw_velocity_resampled: {len(yaw_velocity_resampled)}")
    print(f"Length of time: {len(time)}")

    if not (len(x_position_resampled) == len(y_position_resampled) == len(z_position_resampled) == len(roll_position_resampled) == len(pitch_position_resampled) == len(yaw_position_resampled) == len(x_velocity_resampled) == len(y_velocity_resampled) == len(z_velocity_resampled) == len(roll_velocity_resampled) == len(pitch_velocity_resampled) == len(yaw_velocity_resampled) == len(time)):
        raise ValueError("All arrays must be of the same length")

    df = pd.DataFrame({
        'time': time,
        'x_position': x_position_resampled,
        'y_position': y_position_resampled,
        'z_position': z_position_resampled,
        'roll_position': roll_position_resampled,
        'pitch_position': pitch_position_resampled,
        'yaw_position': yaw_position_resampled,
        'x_velocity': x_velocity_resampled,
        'y_velocity': y_velocity_resampled,
        'z_velocity': z_velocity_resampled,
        'roll_velocity': roll_velocity_resampled,
        'pitch_velocity': pitch_velocity_resampled,
        'yaw_velocity': yaw_velocity_resampled
    })

    df.to_csv('hull_fmu_output.csv', index=False)

    # Set up the figure and subplots
    fig = plt.figure(figsize=(12, 6))

    # 3D plot for the hull position
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_xlim(min(x_position_resampled), max(x_position_resampled))
    ax1.set_ylim(min(y_position_resampled), max(y_position_resampled))
    if np.all(z_position_resampled == 0):
        ax1.set_zlim(-1, 1)  # Handle case where all z positions are zero
    else:
        ax1.set_zlim(min(z_position_resampled), max(z_position_resampled))
    ax1.set_xlabel('X Position [m]')
    ax1.set_ylabel('Y Position [m]')
    ax1.set_zlabel('Z Position [m]')
    ax1.set_title('3D Animation of Hull Position and Orientation')
    
    # Time series plot for the angles
    ax2 = fig.add_subplot(322)
    ax3 = fig.add_subplot(324)
    ax4 = fig.add_subplot(326)

    line, = ax1.plot([], [], [], label='Hull Position Trace')
    point, = ax1.plot([], [], [], 'ro')

    roll_line, = ax2.plot([], [], label='Roll Position')
    pitch_line, = ax3.plot([], [], label='Pitch Position')
    yaw_line, = ax4.plot([], [], label='Yaw Position')

    def init():
        line.set_data([], [])
        line.set_3d_properties([])
        point.set_data([], [])
        point.set_3d_properties([])

        roll_line.set_data([], [])
        pitch_line.set_data([], [])
        yaw_line.set_data([], [])

        return line, point, roll_line, pitch_line, yaw_line

    def update(frame):
        line.set_data(x_position_resampled[:frame], y_position_resampled[:frame])
        line.set_3d_properties(z_position_resampled[:frame])
        point.set_data([x_position_resampled[frame]], [y_position_resampled[frame]])
        point.set_3d_properties([z_position_resampled[frame]])

        roll_line.set_data(time[:frame], roll_position_resampled[:frame])
        pitch_line.set_data(time[:frame], pitch_position_resampled[:frame])
        yaw_line.set_data(time[:frame], yaw_position_resampled[:frame])

        ax2.set_xlim(0, time[frame])
        ax2.set_ylim(min(roll_position_resampled), max(roll_position_resampled) if max(roll_position_resampled) != min(roll_position_resampled) else max(roll_position_resampled) + 1)
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Roll [rad]')
        ax2.set_title('Roll Position')
        ax2.legend()
        ax2.grid(True)

        ax3.set_xlim(0, time[frame])
        ax3.set_ylim(min(pitch_position_resampled), max(pitch_position_resampled) if max(pitch_position_resampled) != min(pitch_position_resampled) else max(pitch_position_resampled) + 1)
        ax3.set_xlabel('Time [s]')
        ax3.set_ylabel('Pitch [rad]')
        ax3.set_title('Pitch Position')
        ax3.legend()
        ax3.grid(True)

        ax4.set_xlim(0, time[frame])
        ax4.set_ylim(min(yaw_position_resampled), max(yaw_position_resampled) if max(yaw_position_resampled) != min(yaw_position_resampled) else max(yaw_position_resampled) + 1)
        ax4.set_xlabel('Time [s]')
        ax4.set_ylabel('Yaw [rad]')
        ax4.set_title('Yaw Position')
        ax4.legend()
        ax4.grid(True)

        return line, point, roll_line, pitch_line, yaw_line

    ani = FuncAnimation(fig, update, frames=len(time), init_func=init, blit=False, interval=100, repeat=False)
    plt.show()

# Function to simulate and plot results for the GNSS FMU
def simulate_and_plot_gnss(fmu_path):
    df = pd.read_csv('hull_fmu_output.csv')

    input_data = np.zeros(len(df), dtype=[
        ('time', np.double),
        ('InputHullPosition[0]', np.double), 
        ('InputHullPosition[1]', np.double),
        ('InputHullPosition[2]', np.double),
        ('InputHullPosition[3]', np.double),
        ('InputHullPosition[4]', np.double),
        ('InputHullPosition[5]', np.double),
        ('InputHullVelocity[0]', np.double),
        ('InputHullVelocity[1]', np.double),
        ('InputHullVelocity[2]', np.double),
        ('InputHullVelocity[3]', np.double),
        ('InputHullVelocity[4]', np.double),
        ('InputHullVelocity[5]', np.double)
    ])

    input_data['time'] = df['time']
    input_data['InputHullPosition[0]'] = df['x_position']
    input_data['InputHullPosition[1]'] = df['y_position']
    input_data['InputHullPosition[2]'] = df['z_position']
    input_data['InputHullPosition[3]'] = df['roll_position']
    input_data['InputHullPosition[4]'] = df['pitch_position']
    input_data['InputHullPosition[5]'] = df['yaw_position']
    input_data['InputHullVelocity[0]'] = df['x_velocity']
    input_data['InputHullVelocity[1]'] = df['y_velocity']
    input_data['InputHullVelocity[2]'] = df['z_velocity']
    input_data['InputHullVelocity[3]'] = df['roll_velocity']
    input_data['InputHullVelocity[4]'] = df['pitch_velocity']
    input_data['InputHullVelocity[5]'] = df['yaw_velocity']

    # Calculate ground truth absolute velocity and yaw angle
    ground_truth_velocity = np.sqrt(df['x_velocity']**2 + df['y_velocity']**2)
    ground_truth_yaw = df['yaw_position']

    result = simulate_fmu(fmu_path, start_time=start_time, stop_time=stop_time, input=input_data, output=[
        'OutputAntenna1Position[0]', 'OutputAntenna1Position[1]', 'OutputAntenna1Position[2]', 
        'OutputAntenna2Position[0]', 'OutputAntenna2Position[1]', 'OutputAntenna2Position[2]',
        'OutputVelocitySpeed', 'OutputVelocityAngle'
    ])

    # Extract output positions and velocities
    antenna1_x = result['OutputAntenna1Position[0]']
    antenna1_y = result['OutputAntenna1Position[1]']
    antenna1_z = result['OutputAntenna1Position[2]']
    antenna2_x = result['OutputAntenna2Position[0]']
    antenna2_y = result['OutputAntenna2Position[1]']
    antenna2_z = result['OutputAntenna2Position[2]']
    velocity_speed = result['OutputVelocitySpeed']
    velocity_angle = result['OutputVelocityAngle']

    # Interpolate positions to match the time steps
    interp_antenna1_x = interp1d(np.linspace(start_time, stop_time, len(antenna1_x)), antenna1_x, kind='linear', fill_value="extrapolate")
    interp_antenna1_y = interp1d(np.linspace(start_time, stop_time, len(antenna1_y)), antenna1_y, kind='linear', fill_value="extrapolate")
    interp_antenna1_z = interp1d(np.linspace(start_time, stop_time, len(antenna1_z)), antenna1_z, kind='linear', fill_value="extrapolate")
    interp_antenna2_x = interp1d(np.linspace(start_time, stop_time, len(antenna2_x)), antenna2_x, kind='linear', fill_value="extrapolate")
    interp_antenna2_y = interp1d(np.linspace(start_time, stop_time, len(antenna2_y)), antenna2_y, kind='linear', fill_value="extrapolate")
    interp_antenna2_z = interp1d(np.linspace(start_time, stop_time, len(antenna2_z)), antenna2_z, kind='linear', fill_value="extrapolate")
    interp_velocity_speed = interp1d(np.linspace(start_time, stop_time, len(velocity_speed)), velocity_speed, kind='linear', fill_value="extrapolate")
    interp_velocity_angle = interp1d(np.linspace(start_time, stop_time, len(velocity_angle)), velocity_angle, kind='linear', fill_value="extrapolate")

    antenna1_x_resampled = interp_antenna1_x(time)
    antenna1_y_resampled = interp_antenna1_y(time)
    antenna1_z_resampled = interp_antenna1_z(time)
    antenna2_x_resampled = interp_antenna2_x(time)
    antenna2_y_resampled = interp_antenna2_y(time)
    antenna2_z_resampled = interp_antenna2_z(time)
    velocity_speed_resampled = interp_velocity_speed(time)
    velocity_angle_resampled = interp_velocity_angle(time)

    # Simulated ground truth positions (InputHullPosition)
    ground_truth_x = input_data['InputHullPosition[0]']
    ground_truth_y = input_data['InputHullPosition[1]']
    ground_truth_z = input_data['InputHullPosition[2]']

    # Set up the figure and subplots
    fig = plt.figure(figsize=(12, 6))

    # 3D plot for the positions
    ax1 = fig.add_subplot(121, projection='3d')
    ax1.set_xlim(min(ground_truth_x), max(ground_truth_x))
    ax1.set_ylim(min(ground_truth_y), max(ground_truth_y))
    if np.all(ground_truth_z == 0):
        ax1.set_zlim(-1, 1)  # Handle case where all z positions are zero
    else:
        ax1.set_zlim(min(ground_truth_z), max(ground_truth_z))
    ax1.set_xlabel('X Position [m]')
    ax1.set_ylabel('Y Position [m]')
    ax1.set_zlabel('Z Position [m]')
    ax1.set_title('3D Animation of Antenna Positions')
    
    # Time series plot for the velocity speed
    ax2 = fig.add_subplot(222)
    ax2.set_xlim(start_time, stop_time)
    ax2.set_ylim(min(min(velocity_speed_resampled), min(ground_truth_velocity)), max(max(velocity_speed_resampled), max(ground_truth_velocity)))
    ax2.set_xlabel('Time [s]')
    ax2.set_ylabel('Velocity Speed [m/s]')
    ax2.set_title('Velocity Speed Over Time')
    ax2.grid(True)

    # Time series plot for the velocity angle
    ax3 = fig.add_subplot(224)
    ax3.set_xlim(start_time, stop_time)
    ax3.set_ylim(min(min(velocity_angle_resampled), min(ground_truth_yaw)), max(max(velocity_angle_resampled), max(ground_truth_yaw)))
    ax3.set_xlabel('Time [s]')
    ax3.set_ylabel('Velocity Angle [rad]')
    ax3.set_title('Velocity Angle Over Time')
    ax3.grid(True)

    line1, = ax1.plot([], [], [], 'b-', label='Simulated Ground Truth Position')
    points1 = ax1.scatter([], [], [], c='red', s=10, alpha=0.2, label='Antenna 1 Position Data')
    points2 = ax1.scatter([], [], [], c='green', s=10, alpha=0.2, label='Antenna 2 Position Data')
    velocity_line, = ax2.plot([], [], label='GNSS Velocity Speed')
    ground_truth_velocity_line, = ax2.plot([], [], label='Simulated Ground Truth Velocity Speed')
    angle_line, = ax3.plot([], [], label='GNSS Velocity Angle')
    ground_truth_angle_line, = ax3.plot([], [], label='Simulated Ground Truth Velocity Angle')

    def init():
        line1.set_data([], [])
        line1.set_3d_properties([])
        points1._offsets3d = ([], [], [])
        points2._offsets3d = ([], [], [])
        velocity_line.set_data([], [])
        ground_truth_velocity_line.set_data([], [])
        angle_line.set_data([], [])
        ground_truth_angle_line.set_data([], [])
        return line1, points1, points2, velocity_line, ground_truth_velocity_line, angle_line, ground_truth_angle_line

    def update(frame):
        line1.set_data(ground_truth_x[:frame], ground_truth_y[:frame])
        line1.set_3d_properties(ground_truth_z[:frame])

        # Update points for antenna 1 and 2 positions
        points1._offsets3d = (antenna1_x_resampled[:frame], antenna1_y_resampled[:frame], antenna1_z_resampled[:frame])
        points2._offsets3d = (antenna2_x_resampled[:frame], antenna2_y_resampled[:frame], antenna2_z_resampled[:frame])
        
        velocity_line.set_data(time[:frame], velocity_speed_resampled[:frame])
        ground_truth_velocity_line.set_data(time[:frame], ground_truth_velocity[:frame])
        angle_line.set_data(time[:frame], velocity_angle_resampled[:frame])
        ground_truth_angle_line.set_data(time[:frame], ground_truth_yaw[:frame])

        ax2.set_xlim(0, time[frame])
        ax3.set_xlim(0, time[frame])

        return line1, points1, points2, velocity_line, ground_truth_velocity_line, angle_line, ground_truth_angle_line

    ani = FuncAnimation(fig, update, frames=len(time), init_func=init, blit=False, interval=100, repeat=False)
    ax1.legend()
    ax2.legend()
    ax3.legend()
    plt.show()

# Function to Simulate and Plot IMU FMU
def simulate_and_plot_imu(fmu_path, csv_path):
    # Load hull FMU output data
    df = pd.read_csv(csv_path)

    # Prepare input data for IMU FMU
    input_data = np.zeros(len(df), dtype=[
        ('time', np.double),
        ('InputHullPosition[0]', np.double), 
        ('InputHullPosition[1]', np.double),
        ('InputHullPosition[2]', np.double),
        ('InputHullPosition[3]', np.double),
        ('InputHullPosition[4]', np.double),
        ('InputHullPosition[5]', np.double),
        ('InputHullVelocity[0]', np.double),
        ('InputHullVelocity[1]', np.double),
        ('InputHullVelocity[2]', np.double),
        ('InputHullVelocity[3]', np.double),
        ('InputHullVelocity[4]', np.double),
        ('InputHullVelocity[5]', np.double)
    ])

    input_data['time'] = df['time']
    input_data['InputHullPosition[0]'] = df['x_position']
    input_data['InputHullPosition[1]'] = df['y_position']
    input_data['InputHullPosition[2]'] = df['z_position']
    input_data['InputHullPosition[3]'] = df['roll_position']
    input_data['InputHullPosition[4]'] = df['pitch_position']
    input_data['InputHullPosition[5]'] = df['yaw_position']
    input_data['InputHullVelocity[0]'] = df['x_velocity']
    input_data['InputHullVelocity[1]'] = df['y_velocity']
    input_data['InputHullVelocity[2]'] = df['z_velocity']
    input_data['InputHullVelocity[3]'] = df['roll_velocity']
    input_data['InputHullVelocity[4]'] = df['pitch_velocity']
    input_data['InputHullVelocity[5]'] = df['yaw_velocity']

    # Simulate IMU FMU
    result = simulate_fmu(fmu_path, start_time=start_time, stop_time=stop_time, input=input_data, output=[
        'OutputAccelerationLinear[0]', 'OutputAccelerationLinear[1]', 'OutputAccelerationLinear[2]',
        'OutputVelocityAngular[0]', 'OutputVelocityAngular[1]', 'OutputVelocityAngular[2]'
    ])

    # Extract simulated IMU output
    result_time = result['time']
    acceleration_x = result['OutputAccelerationLinear[0]']
    acceleration_y = result['OutputAccelerationLinear[1]']
    acceleration_z = result['OutputAccelerationLinear[2]']
    angular_velocity_roll = result['OutputVelocityAngular[0]']
    angular_velocity_pitch = result['OutputVelocityAngular[1]']
    angular_velocity_yaw = result['OutputVelocityAngular[2]']

    # Interpolate IMU output to match hull FMU output time steps
    interp_acceleration_x = interp1d(result_time, acceleration_x, kind='linear', fill_value='extrapolate')
    interp_acceleration_y = interp1d(result_time, acceleration_y, kind='linear', fill_value='extrapolate')
    interp_acceleration_z = interp1d(result_time, acceleration_z, kind='linear', fill_value='extrapolate')
    interp_angular_velocity_roll = interp1d(result_time, angular_velocity_roll, kind='linear', fill_value='extrapolate')
    interp_angular_velocity_pitch = interp1d(result_time, angular_velocity_pitch, kind='linear', fill_value='extrapolate')
    interp_angular_velocity_yaw = interp1d(result_time, angular_velocity_yaw, kind='linear', fill_value='extrapolate')

    acceleration_x_resampled = interp_acceleration_x(time)
    acceleration_y_resampled = interp_acceleration_y(time)
    acceleration_z_resampled = interp_acceleration_z(time)
    angular_velocity_roll_resampled = interp_angular_velocity_roll(time)
    angular_velocity_pitch_resampled = interp_angular_velocity_pitch(time)
    angular_velocity_yaw_resampled = interp_angular_velocity_yaw(time)

    # Calculate ground truth acceleration from velocity
    ground_truth_acceleration_x = np.gradient(input_data['InputHullVelocity[0]'], step_size)
    ground_truth_acceleration_y = np.gradient(input_data['InputHullVelocity[1]'], step_size)
    ground_truth_acceleration_z = np.gradient(input_data['InputHullVelocity[2]'], step_size) - gravity

    # Plot linear acceleration and angular velocity comparison in one figure
    fig, axs = plt.subplots(3, 2, figsize=(15, 20))
    fig.suptitle('IMU Comparison')

    # Linear acceleration plots
    axs[0, 0].plot(time, ground_truth_acceleration_x, label='Simulated Ground Truth Acceleration X')
    axs[0, 0].plot(time, acceleration_x_resampled, label='IMU Output Acceleration X')
    axs[0, 0].set_xlabel('Time [s]')
    axs[0, 0].set_ylabel('Acceleration X [m/s**2]')
    axs[0, 0].set_title('Linear Acceleration X')
    axs[0, 0].legend()
    axs[0, 0].grid(True)

    axs[1, 0].plot(time, ground_truth_acceleration_y, label='Simulated Ground Truth Acceleration Y')
    axs[1, 0].plot(time, acceleration_y_resampled, label='IMU Output Acceleration Y')
    axs[1, 0].set_xlabel('Time [s]')
    axs[1, 0].set_ylabel('Acceleration Y [m/s**2]')
    axs[1, 0].set_title('Linear Acceleration Y')
    axs[1, 0].legend()
    axs[1, 0].grid(True)

    axs[2, 0].plot(time, ground_truth_acceleration_z, label='Simulated Ground Truth Acceleration Z')
    axs[2, 0].plot(time, acceleration_z_resampled, label='IMU Output Acceleration Z')
    axs[2, 0].set_xlabel('Time [s]')
    axs[2, 0].set_ylabel('Acceleration Z [m/s**2]')
    axs[2, 0].set_title('Linear Acceleration Z')
    axs[2, 0].legend()
    axs[2, 0].grid(True)

    # Angular velocity plots
    axs[0, 1].plot(time, input_data['InputHullVelocity[3]'], label='Simulated Ground Truth Angular Velocity Roll')
    axs[0, 1].plot(time, angular_velocity_roll_resampled, label='IMU Output Angular Velocity Roll')
    axs[0, 1].set_xlabel('Time [s]')
    axs[0, 1].set_ylabel('Angular Velocity Roll [rad/s]')
    axs[0, 1].set_title('Angular Velocity Roll')
    axs[0, 1].legend()
    axs[0, 1].grid(True)

    axs[1, 1].plot(time, input_data['InputHullVelocity[4]'], label='Simulated Ground Truth Angular Velocity Pitch')
    axs[1, 1].plot(time, angular_velocity_pitch_resampled, label='IMU Output Angular Velocity Pitch')
    axs[1, 1].set_xlabel('Time [s]')
    axs[1, 1].set_ylabel('Angular Velocity Pitch [rad/s]')
    axs[1, 1].set_title('Angular Velocity Pitch')
    axs[1, 1].legend()
    axs[1, 1].grid(True)

    axs[2, 1].plot(time, input_data['InputHullVelocity[5]'], label='Simulated Ground Truth Angular Velocity Yaw')
    axs[2, 1].plot(time, angular_velocity_yaw_resampled, label='IMU Output Angular Velocity Yaw')
    axs[2, 1].set_xlabel('Time [s]')
    axs[2, 1].set_ylabel('Angular Velocity Yaw [rad/s]')
    axs[2, 1].set_title('Angular Velocity Yaw')
    axs[2, 1].legend()
    axs[2, 1].grid(True)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.show()

# Simulate and plot for the Hull FMU
simulate_and_plot_hull('build/HullFMU.fmu')

# Simulate and plot for the GNSS FMU
simulate_and_plot_gnss('build/GNSSFMU.fmu')

# Run the simulation and plotting function for IMU FMU
simulate_and_plot_imu('build/IMUFMU.fmu', 'hull_fmu_output.csv')

# Delete Hull Simulated Data .CSV as we dont need it anymore after all the visualization
os.remove("hull_fmu_output.csv")
