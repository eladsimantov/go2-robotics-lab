#!/usr/bin/env python3
import matplotlib.pyplot as plt
import pickle
import numpy as np

# Load the recorded data
with open("recorded_data_joystick.pkl", "rb") as f:
    data = pickle.load(f)

lowstate = data["lowstate"]

# Extract time and joint positions
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
joint_positions = []
joint_velocities = []
joint_taus = []
for motor_id in range(num_motors):
    q_motor = [sample["motor_states"][motor_id]["actual_q"] for sample in lowstate]
    dq_motor = [sample["motor_states"][motor_id]["actual_dq"] for sample in lowstate]
    tau_motor = [sample["motor_states"][motor_id]["actual_tau_est"] for sample in lowstate]
    joint_positions.append(q_motor)
    joint_velocities.append(dq_motor)   
    joint_taus.append(tau_motor)   


# Plot all joints by leg with subplots according to leg layout
plt.figure(figsize=(12, 8))
for leg in leg_positions:
    plt.subplot(2, 2, leg_positions.index(leg) + 1)
    for motor_id in leg_layout[leg]:
        joint_name = [name for name, idx in jointGroups.items() if idx == motor_id][0]
        rad2deg = 180.0 / np.pi
        plt.plot(times, np.array(joint_positions[motor_id]) * rad2deg, label=f'{joint_name}')    
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
        plt.plot(times, np.array(joint_velocities[motor_id]) * rad2deg, label=f'{joint_name}')   
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
        plt.plot(times, np.array(joint_taus[motor_id]), label=f'{joint_name}')   
    plt.title(f'{leg} Leg Joint Tau Estimates')
    plt.xlabel('Time (s)')
    plt.ylabel('Joint Tau Estimate (Nm)')
    plt.legend(loc='upper right', ncol=1)
    plt.grid(True)
plt.tight_layout()
plt.show()
