import pickle
import sys
import time

import numpy as np

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

stand_up_joint_pos = np.array(
    [0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763, 0.00571868, 0.608813, -1.21763, -0.00571868,
        0.608813, -1.21763], dtype=float)

stand_down_joint_pos = np.array(
    [0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187,
        -2.44375], dtype=float)

# ---------------------------------------------------------
# 1) Load original data
# ---------------------------------------------------------
with open("command_q_forward.pkl", "rb") as f:
    data_pca = pickle.load(f)

forward_times = data_pca["time_array"]  # shape (N,)
commands = data_pca["command_q_forward"]  # shape (12, N)
num_motors, N = commands.shape

# ---------------------------------------------------------
# 2) Apply a slowdown factor to the entire timeline
# ---------------------------------------------------------
slowdown_factor = 3.0  # e.g., 3 means the motion will take 3x longer
time_slow = forward_times * slowdown_factor

# Example stand-up time, dt, etc.
stand_up_time = 3.0
dt = 0.002  # 25 Hz or however you like
running_time = 0.0

# ---------------------------------------------------------
# 3) Init publisher
# ---------------------------------------------------------
if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")
else:
    ChannelFactoryInitialize(0, sys.argv[1])

pub = ChannelPublisher("rt/lowcmd", LowCmd_)
pub.Init()
cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0] = 0xFE
cmd.head[1] = 0xEF
cmd.level_flag = 0xFF
cmd.gpio = 0
crc = CRC()

# set the modes, gains, etc.
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01
    cmd.motor_cmd[i].q = 0.0
    cmd.motor_cmd[i].kp = 0.0
    cmd.motor_cmd[i].dq = 0.0
    cmd.motor_cmd[i].kd = 0.0
    cmd.motor_cmd[i].tau = 0.0

input("Press ENTER to start...")

# ---------------------------------------------------------
# 4) Main control loop
# ---------------------------------------------------------
max_time = stand_up_time + time_slow[-1] + 2.0  # a bit of margin

while running_time < max_time:
    step_start = time.perf_counter()
    running_time += dt

    if running_time < stand_up_time:
        phase = np.tanh(running_time / 1.2)
        for i in range(12):
            cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
            cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0
    else:
        # past stand-up => "walk" by picking nearest time index in the slowed timeline
        walk_time = running_time - stand_up_time
        # find which frame matches 'walk_time' in 'time_slow'
        idx = np.searchsorted(time_slow, walk_time)
        if idx >= N:
            idx = N - 1  # clamp to last frame

        # Apply the command angles
        phase = np.tanh(running_time /  1.2)
        for i in range(num_motors):
            cmd.motor_cmd[i].q = commands[i, idx]
            cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

    # Publish
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)

    # sleep
    time_until_next_step = dt - (time.perf_counter() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

print("Done.")
