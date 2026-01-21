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
dt = 0.002
running_time = 0.0
crc = CRC()
stand_up_time = 3.0

###############################################################################
# 3. Load your PCA reconstruction (reconstructed_q_forward) from a file
###############################################################################
with open("mean_pose.pkl", "rb") as f:
    data_pca = pickle.load(f)

mean_pose = data_pca['mean_pose']
num_motors = 12

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
# 7) Main control loop: Stand-up then hold the natural pose
# ---------------------------------------------------------
max_time = 10

while running_time < max_time:
    step_start = time.perf_counter()
    running_time += dt

    if running_time < stand_up_time:
        # During the stand-up phase, interpolate between stand-down and stand-up poses.
        phase = np.tanh(running_time / 1.2)
        for i in range(num_motors):
            cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
            cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0
    else:
        # After stand-up, hold the reconstructed natural pose.
        phase = np.tanh(running_time / 1.2)  # Optionally use phase for gain scheduling
        for i in range(num_motors):
            cmd.motor_cmd[i].q = mean_pose[i]
            cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

    # Publish the command with the computed CRC.
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)

    # Maintain the control loop timing.
    time_until_next_step = dt - (time.perf_counter() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

print("Done.")