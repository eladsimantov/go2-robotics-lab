"""
This script loads recorded data from a pickle file and prepares it for simulation or analysis.
"""
import pickle
import numpy as np

# Load the recorded data
with open("recordings/ShankHands_recorded_data_joystick.pkl", "rb") as f:
    data = pickle.load(f)

lowstate = data["lowstate"]
print(f"Loaded {len(lowstate)} samples from recorded data.")
print("First sample motor states:", lowstate[0]["motor_states"])

# Extract time and joint positions
times = [sample["time"] for sample in lowstate]
num_motors = 12

# Extract joint positions for each motor
joint_positions = []
for motor_id in range(num_motors):
    q_motor = [sample["motor_states"][motor_id]["actual_q"] for sample in lowstate]
    joint_positions.append(q_motor)
    
# Extract joint velocities for each motor
joint_velocities = []
for motor_id in range(num_motors):
    dq_motor = [sample["motor_states"][motor_id]["actual_dq"] for sample in lowstate]
    joint_velocities.append(dq_motor)

print("Times:", times)
print(f"Joint Positions: {[[f'{q:.3g}' for q in positions] for positions in joint_positions]}")
print(f"Joint Velocities: {[[f'{dq:.3g}' for dq in velocities] for velocities in joint_velocities]}")



