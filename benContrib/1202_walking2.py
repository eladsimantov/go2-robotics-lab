import time
import sys
import numpy as np
import pickle

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

###############################################################################
# 1. Define stand-up and stand-down joint positions
###############################################################################
stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763
], dtype=float)

stand_down_joint_pos = np.array([
    0.0473455, 1.22187, -2.44375,
    -0.0473455, 1.22187, -2.44375,
    0.0473455, 1.22187, -2.44375,
    -0.0473455, 1.22187, -2.44375
], dtype=float)

###############################################################################
# 2. Simulation timing, gains, etc.
###############################################################################
dt = 0.002                # Main control loop time step (seconds)
stand_up_time = 3.0       # Time to transition from stand_down to stand_up
num_motors = 12
crc = CRC()

###############################################################################
# 3. Load your 14 synthetic poses
#    Each item in synthetic_poses is shape: (12, N)
#    dt_synth is the time step for your PCA data (0.00285 s)
###############################################################################
with open("synthetic_poses.pkl", "rb") as f:
    data_synth = pickle.load(f)

synthetic_poses = data_synth["synthetic_poses"]  # List of length 14, each (12, N)
dt_synth = data_synth["time_step"]               # sec

###############################################################################
# 4. Prompt user to start
###############################################################################
input("Press ENTER to start simulation...")

###############################################################################
# 5. Initialize communications
###############################################################################
if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")  # Simulation local loopback
else:
    ChannelFactoryInitialize(0, sys.argv[1])

pub = ChannelPublisher("rt/lowcmd", LowCmd_)
pub.Init()

# Create a LowCmd message
cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0] = 0xFE
cmd.head[1] = 0xEF
cmd.level_flag = 0xFF
cmd.gpio = 0

# Set the base mode and zero out everything
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
    cmd.motor_cmd[i].q = 0.0
    cmd.motor_cmd[i].kp = 0.0
    cmd.motor_cmd[i].dq = 0.0
    cmd.motor_cmd[i].kd = 0.0
    cmd.motor_cmd[i].tau = 0.0

###############################################################################
# 6. Main loop: stand-up, then iterate through each synthetic pose for 5 s
###############################################################################
start_time = time.perf_counter()
running_time = 0.0
max_time = 9999.0  # or some large number so we don't exit prematurely

pose_duration = 5.0  # each pose runs for 5 seconds

pose_index = 0  # index in synthetic_poses

while running_time < max_time:
    step_start = time.perf_counter()
    running_time = time.perf_counter() - start_time

    if running_time < stand_up_time:
        # ---------------------------------------------------------------
        # PHASE 1: Interpolate from stand-down to stand-up
        # ---------------------------------------------------------------
        phase = np.tanh(running_time / 1.2)  # or any smooth interpolation
        for i in range(num_motors):
            cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
            cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0
    else:
        # ---------------------------------------------------------------
        # PHASE 2: Cycle through each synthetic pose for 5 seconds
        # ---------------------------------------------------------------
        time_in_phase = running_time - stand_up_time

        # Figure out which pose we are on
        # - Each pose has a duration of pose_duration = 5 s
        # - pose_index goes from 0 to 13 (because we have 14 poses total)
        current_pose_index = int(time_in_phase // pose_duration)

        if current_pose_index >= len(synthetic_poses):
            # We've played all 14 poses for 5 seconds each -> stop or hold last pose
            print("All synthetic poses completed.")
            break

        pose_index = current_pose_index
        time_in_pose = time_in_phase - pose_index * pose_duration  # how many seconds into this pose

        # Select the relevant synthetic pose array
        # shape: (12, N)
        current_pose_data = synthetic_poses[pose_index]
        N = current_pose_data.shape[1]

        # We progress in the current pose at dt_synth intervals
        # This means each column of current_pose_data is one dt_synth step
        step_in_pose = int(time_in_pose / dt_synth) % N  # loop if you want, or clamp if not

        for motor_id in range(num_motors):
            cmd.motor_cmd[motor_id].q = current_pose_data[motor_id, step_in_pose]
            # Gains
            cmd.motor_cmd[motor_id].kp = 50.0
            cmd.motor_cmd[motor_id].dq = 0.0
            cmd.motor_cmd[motor_id].kd = 3.5
            cmd.motor_cmd[motor_id].tau = 0.0

    # Publish the command
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)

    # Keep loop timing
    time_until_next_step = dt - (time.perf_counter() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

print("Done.")
