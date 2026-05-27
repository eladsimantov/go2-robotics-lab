import numpy as np
from unitreeGo2Model import forward_kinematics, inverse_kinematics, standing_configuration

print("=========================================================")
print("1. VERIFYING IK WITH CUSTOM BASE POSE")
print("=========================================================")

# Define base pose
base_pos = np.array([0.1, -0.05, 0.28])
base_rpy = np.array([0.05, -0.1, 0.15])  # roll, pitch, yaw in radians

# Get standing joint angles (12 values)
q0_joints = standing_configuration()

# Calculate forward kinematics in the world frame with the base pose
foot_positions_world = forward_kinematics(
    configuration=q0_joints,
    base_position=base_pos,
    base_rpy=base_rpy
)

print("Forward Kinematics (World Frame):")
for name, pos in foot_positions_world.items():
    print(f"  {name}: {pos}")

# Solve inverse kinematics to retrieve joint angles
success, iterations, q_sol = inverse_kinematics(
    target_positions=foot_positions_world,
    initial_configuration=q0_joints,
    base_position=base_pos,
    base_rpy=base_rpy
)

print(f"\nIK success: {success}")
print(f"IK iterations: {iterations}")

# The solution q_sol should match q0_joints because the base pose matches
print("\nComparing solved joints with original standing joints:")
for leg_name, idx in [("FR", 0), ("FL", 3), ("RR", 6), ("RL", 9)]:
    sol_leg = q_sol[idx:idx+3]
    orig_leg = q0_joints[idx:idx+3]
    diff = sol_leg - orig_leg
    print(f"  {leg_name}: solved={sol_leg}, diff={diff}")
    assert np.allclose(sol_leg, orig_leg, atol=1e-5), f"IK failed for {leg_name}!"

print("\nAnalytical IK with base pose verified successfully!")

print("\n=========================================================")
print("2. PLANNING TASK-SPACE BASE MOTION TRAJECTORY")
print("=========================================================")
print("Objective: Move base backward and rotate (pitch/roll/yaw) while feet stay planted.")

# 1. Initial base pose
start_pos = np.array([0.0, 0.0, 0.28])     # Standing height of 28cm
start_rpy = np.array([0.0, 0.0, 0.0])      # No rotation

# 2. Target base pose (Leaning back and slightly pitched up)
end_pos = np.array([-0.06, 0.0, 0.25])    # Shifted backward by 6cm, lowered by 3cm
end_rpy = np.array([-0.034, -0.34, 0.0])  # Pitched up by 20 deg and roll sideways by 2 degrees.

# 3. Find the world coordinates of the feet at the starting stand pose
# These coordinates represent the planted feet on the ground.
planted_feet_world = forward_kinematics(
    configuration=q0_joints,
    base_position=start_pos,
    base_rpy=start_rpy
)
print("Planted feet positions in world frame (remain constant):")
for name, pos in planted_feet_world.items():
    print(f"  {name}: {pos}")

# 4. Generate smooth trajectory (Minimum-jerk / cosine interpolation)
duration = 2.0  # seconds
dt = 0.05       # 20 Hz control loop
steps = int(duration / dt)
time_steps = np.linspace(0, duration, steps)

planned_positions = []
planned_rpys = []
joint_trajectory = []

print(f"\nGenerating trajectory with {steps} steps...")
for t in time_steps:
    # Cosine interpolation profile for smooth acceleration/deceleration
    s = 0.5 * (1 - np.cos(np.pi * t / duration))
    pos = start_pos + s * (end_pos - start_pos)
    rpy = start_rpy + s * (end_rpy - start_rpy)
    
    planned_positions.append(pos)
    planned_rpys.append(rpy)
    
    # Solve inverse kinematics for this trajectory point
    # We pass the constant foot coordinates and the moving base pose
    success, _, q_sol = inverse_kinematics(
        target_positions=planted_feet_world,
        initial_configuration=q0_joints,
        base_position=pos,
        base_rpy=rpy
    )
    
    if not success:
        print(f"  Warning: Trajectory out of reach at t={t:.2f}s (pos={pos}, rpy={rpy})")
        
    joints = q_sol[7:19] if q_sol.size == 19 else q_sol
    joint_trajectory.append(joints)

print("Trajectory generation complete.")

# 5. Print a few key points from the generated joint trajectory
print("\nSample points along trajectory:")
sample_indices = [0, steps // 2, steps - 1]
for idx in sample_indices:
    t = time_steps[idx]
    pos = planned_positions[idx]
    rpy = planned_rpys[idx]
    joints = joint_trajectory[idx]
    print(f"t={t:.2f}s | Base Pos={pos} RPY={rpy}")
    print(f"  Joints (deg): FL={np.degrees(joints[3:6])} FR={np.degrees(joints[0:3])}")

