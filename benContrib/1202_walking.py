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
# 2. Simulation timing and global objects
###############################################################################
dt = 0.002       # Control loop time step
stand_up_time = 3.0
max_time = 100.0
running_time = 0.0

crc = CRC()

###############################################################################
# 3. Load PCA-based synthetic pose data (the 1st 3 PCs walking)
###############################################################################
with open("synthetic_poses.pkl", "rb") as f:
    data_synth = pickle.load(f)

synthetic_poses = data_synth["synthetic_poses"]  # shape: (12, N)
dt_synth = 0.00285             #[seconds], time_step = 1/sampling_rate
num_motors = 12
N = synthetic_poses.shape[1]                     # number of time steps in the synthetic gait

###############################################################################
# 4. Prompt user to start
###############################################################################
input("Press ENTER to start simulation...")

###############################################################################
# 5. Initialize the communications & publisher
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
# 6. Main control loop
#    - First phase: transition from stand-down to stand-up
#    - Then: run synthetic PCA-based walking
###############################################################################
start_time = time.perf_counter()

while running_time < max_time:
    step_start = time.perf_counter()
    running_time = time.perf_counter() - start_time

    if running_time < stand_up_time:
        # ---------------------------------------------------------------
        # PHASE 1: Interpolate from stand-down pose to stand-up pose
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
        # PHASE 2: Walking motion from the first 3 PCs (synthetic_poses)
        # ---------------------------------------------------------------
        # We have synthetic_poses of shape (12, N).
        # Each column is one time step of dt_synth.
        # We can loop repeatedly over that pattern by modulo indexing.
        time_in_walk = running_time - stand_up_time
        idx = int(time_in_walk / dt_synth) % N  # loop repeatedly over the pattern

        for i in range(num_motors):
            # Position command from PCA
            cmd.motor_cmd[i].q = synthetic_poses[i, idx]

            # Simple PD gains for position control
            cmd.motor_cmd[i].kp = 50.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

    # Publish the command
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)

    # Maintain the control loop timing
    time_until_next_step = dt - (time.perf_counter() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

print("Done.")
