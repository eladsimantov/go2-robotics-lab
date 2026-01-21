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

# Extract joint positions for each motor
joint_positions = []
for motor_id in range(num_motors):
    q_motor = [sample["motor_states"][motor_id]["actual_q"] for sample in lowstate]
    joint_positions.append(q_motor)

# Plot all joints
plt.figure(figsize=(12, 8))
for motor_id in range(num_motors):
    plt.plot(times, joint_positions[motor_id], label=f'Joint {motor_id}')

plt.xlabel('Time (s)')
plt.ylabel('Joint Position (rad)')
plt.title('Recorded Joint Positions Over Time')
plt.legend(loc='best', ncol=2)
plt.grid(True)
plt.tight_layout()
plt.show()

# Plot velocities in another figure
joint_velocities = []
for motor_id in range(num_motors):
    dq_motor = [sample["motor_states"][motor_id]["actual_dq"] for sample in lowstate]
    joint_velocities.append(dq_motor)   
plt.figure(figsize=(12, 8))
for motor_id in range(num_motors):
    plt.plot(times, joint_velocities[motor_id], label=f'Joint {motor_id}')  
plt.xlabel('Time (s)')
plt.ylabel('Joint Velocity (rad/s)')
plt.title('Recorded Joint Velocities Over Time')
plt.legend(loc='best', ncol=2)
plt.grid(True)
plt.tight_layout()
plt.show()
