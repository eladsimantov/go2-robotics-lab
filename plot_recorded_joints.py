#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pickle
import numpy as np
from scipy.interpolate import interp1d

# Load the recorded data
with open("recorded_data_joystick.pkl", "rb") as f:
    data = pickle.load(f)

# Extract time and joint positions
lowstate = data["lowstate"]
lowcmd = data.get("lowcmd", [])  # Get command data if available

print(f"Loaded {len(lowstate)} lowstate samples from recorded data.")
if lowcmd:
    print(f"Loaded {len(lowcmd)} lowcmd samples from recorded data.")
    cmd_times = [sample["time"] for sample in lowcmd]
else:
    print("No lowcmd data found in recording.")

times = [sample["time"] for sample in lowstate]
num_motors = 12

jointGroups = {
    "FR_hip": 0,
    "FR_thigh": 1,
    "FR_calf": 2,
    "FL_hip": 3,
    "FL_thigh": 4,
    "FL_calf": 5,
    "RR_hip": 6,
    "RR_thigh": 7,
    "RR_calf": 8,
    "RL_hip": 9,
    "RL_thigh": 10,
    "RL_calf": 11,
}
# Create a mapping of leg positions to motor indices
leg_layout = {
    "RearLeft": [9, 10, 11],      # Rear left
    "FrontLeft": [3, 4, 5],        # Front left
    "RearRight": [6, 7, 8],        # Rear right
    "FrontRight": [0, 1, 2],        # Front right
}

leg_positions = ["RearLeft", "FrontLeft", "RearRight", "FrontRight"]

# Extract joint positions, velocities and tau estimates for each motor
joint_positions_est = []
joint_velocities_est = []
joint_taus_est = []
for motor_id in range(num_motors):
    q_motor = [sample["motor_states"][motor_id]["actual_q"] for sample in lowstate]
    dq_motor = [sample["motor_states"][motor_id]["actual_dq"] for sample in lowstate]
    tau_motor = [sample["motor_states"][motor_id]["actual_tau_est"] for sample in lowstate]
    joint_positions_est.append(q_motor)
    joint_velocities_est.append(dq_motor)   
    joint_taus_est.append(tau_motor)   

# Extract command data if available
if lowcmd:
    joint_cmd_positions = []
    joint_cmd_velocities = []
    joint_cmd_taus = []
    joint_cmd_kp = []
    joint_cmd_kd = []
    joint_cmd_modes = []
    
    for motor_id in range(num_motors):
        q_cmd = [sample["motor_states"][motor_id]["command_q"] for sample in lowcmd]
        dq_cmd = [sample["motor_states"][motor_id]["command_dq"] for sample in lowcmd]
        tau_cmd = [sample["motor_states"][motor_id]["command_tau"] for sample in lowcmd]
        kp_cmd = [sample["motor_states"][motor_id]["command_kp"] for sample in lowcmd]
        kd_cmd = [sample["motor_states"][motor_id]["command_kd"] for sample in lowcmd]
        mode_cmd = [sample["motor_states"][motor_id]["command_mode"] for sample in lowcmd]
        
        joint_cmd_positions.append(q_cmd)
        joint_cmd_velocities.append(dq_cmd)   
        joint_cmd_taus.append(tau_cmd)   
        joint_cmd_kp.append(kp_cmd)   
        joint_cmd_kd.append(kd_cmd)   
        joint_cmd_modes.append(mode_cmd)   

# Extract foot force samples and quaternion data from IMU  
foot_forces = []
quaternions_imu = []
for sample in lowstate:
    foot_forces.append(sample["foot_force"])
    quaternions_imu.append(sample["quaternion"])

# Plot all joints by leg with subplots according to leg layout
plt.figure(figsize=(12, 8))
for leg in leg_positions:
    plt.subplot(2, 2, leg_positions.index(leg) + 1)
    for motor_id in leg_layout[leg]:
        joint_name = [name for name, idx in jointGroups.items() if idx == motor_id][0]
        rad2deg = 180.0 / np.pi
        plt.plot(times, np.array(joint_positions_est[motor_id]) * rad2deg, label=f'{joint_name}')    
    plt.title(f'{leg} Leg Joint Positions')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Position (deg)')
    plt.legend(loc='upper right', ncol=1)
    plt.grid(True)
plt.tight_layout()
plt.show()

# Plot velocities in another figure
plt.figure(figsize=(12, 8))
for leg in leg_positions:
    plt.subplot(2, 2, leg_positions.index(leg) + 1)
    for motor_id in leg_layout[leg]:
        joint_name = [name for name, idx in jointGroups.items() if idx == motor_id][0]
        rad2deg = 180.0 / np.pi
        plt.plot(times, np.array(joint_velocities_est[motor_id]) * rad2deg, label=f'{joint_name}')   
    plt.title(f'{leg} Leg Joint Velocities')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Velocity (deg/s)')
    plt.legend(loc='upper right', ncol=1)
    plt.grid(True)
plt.tight_layout()
plt.show()

# Plot tau estimates in another figure
plt.figure(figsize=(12, 8))
for leg in leg_positions:
    plt.subplot(2, 2, leg_positions.index(leg) + 1)
    for motor_id in leg_layout[leg]:
        joint_name = [name for name, idx in jointGroups.items() if idx == motor_id][0]
        plt.plot(times, np.array(joint_taus_est[motor_id]), label=f'{joint_name}')   
    plt.title(f'{leg} Leg Joint Tau Estimates')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Tau Estimate (Nm)')
    plt.legend(loc='upper right', ncol=1)
    plt.grid(True)
plt.tight_layout()
plt.show()

# Plot command data if available
if lowcmd:
    # Plot command positions vs actual positions
    plt.figure(figsize=(12, 8))
    for leg in leg_positions:
        plt.subplot(2, 2, leg_positions.index(leg) + 1)
        for motor_id in leg_layout[leg]:
            joint_name = [name for name, idx in jointGroups.items() if idx == motor_id][0]
            rad2deg = 180.0 / np.pi
            # Interpolate command data to match lowstate timing for comparison
            if len(cmd_times) > 1:
                cmd_interp = interp1d(cmd_times, joint_cmd_positions[motor_id], 
                                    kind='linear', bounds_error=False, fill_value='extrapolate')
                cmd_interpolated = cmd_interp(times)
                plt.plot(times, np.array(joint_positions_est[motor_id]) * rad2deg, 
                        label=f'{joint_name} actual', linestyle='-')
                plt.plot(times, np.array(cmd_interpolated) * rad2deg, 
                        label=f'{joint_name} cmd', linestyle='--', alpha=0.7)
        plt.title(f'{leg} Leg: Command vs Actual Positions')
        plt.xlabel('Time (s)')
        plt.ylabel('Joint Position (deg)')
        plt.legend(bbox_to_anchor=(1.05, 1), loc='upper left')
        plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Plot Kp commands
    plt.figure(figsize=(12, 8))
    for leg in leg_positions:
        plt.subplot(2, 2, leg_positions.index(leg) + 1)
        for motor_id in leg_layout[leg]:
            joint_name = [name for name, idx in jointGroups.items() if idx == motor_id][0]
            plt.plot(cmd_times, joint_cmd_kp[motor_id], label=f'{joint_name}')   
        plt.title(f'{leg} Leg Kp Commands')
        plt.xlabel('Time (s)')
        plt.ylabel('Kp (Nm/rad)')
        plt.legend(loc='upper right', ncol=1)
        plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Plot Kd commands
    plt.figure(figsize=(12, 8))
    for leg in leg_positions:
        plt.subplot(2, 2, leg_positions.index(leg) + 1)
        for motor_id in leg_layout[leg]:
            joint_name = [name for name, idx in jointGroups.items() if idx == motor_id][0]
            plt.plot(cmd_times, joint_cmd_kd[motor_id], label=f'{joint_name}')   
        plt.title(f'{leg} Leg Kd Commands')
        plt.xlabel('Time (s)')
        plt.ylabel('Kd (Nm*s/rad)')
        plt.legend(loc='upper right', ncol=1)
        plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Plot Tau feedforward commands
    plt.figure(figsize=(12, 8))
    for leg in leg_positions:
        plt.subplot(2, 2, leg_positions.index(leg) + 1)
        for motor_id in leg_layout[leg]:
            joint_name = [name for name, idx in jointGroups.items() if idx == motor_id][0]
            plt.plot(cmd_times, joint_cmd_taus[motor_id], label=f'{joint_name}')   
            plt.plot(times, np.array(joint_taus_est[motor_id]), label=f'{joint_name} est', linestyle='--', alpha=0.7)
        plt.title(f'{leg} Leg Tau Feedforward Commands VS Estimated Tau')
        plt.xlabel('Time (s)')
        plt.ylabel('Tau Command (Nm)')
        plt.legend(loc='upper right', ncol=1)
        plt.grid(True)
    plt.tight_layout()
    plt.show()

    print("All command plots displayed!")
else:
    print("No command data available to plot.")
