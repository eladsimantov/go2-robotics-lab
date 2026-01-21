import time
import sys
import numpy as np
import pickle

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
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
dt = 0.1
running_time = 0.0
crc = CRC()
stand_up_time = 3.0

###############################################################################
# 3. Load your PCA reconstruction (reconstructed_q_forward) from a file
###############################################################################
with open("command_q_forward.pkl", "rb") as f:
    data_pca = pickle.load(f)

forward_times = data_pca['time_array']
commands = data_pca['command_q_forward']
motor_indices = data_pca['motor_indices']
mn = data_pca['mn']

dt = np.round(np.mean(np.diff(forward_times)), 6)
# Print size of each array
print("commands:", commands.shape)
print("mn:", mn.shape)

new_zero = stand_up_joint_pos #mn.flatten() +
print(mn[5,0])
print(stand_up_joint_pos[5])
print(new_zero[5])
print("stand_up_joint_pos:", stand_up_joint_pos.shape)
print("new_zero:", new_zero.shape)
reconstructed_q_forward = np.tile(new_zero[:, np.newaxis], (1, 1000))# reconstructed_q_forward = new_zero[:, np.newaxis] + commands
# reconstructed_q_forward = commands + new_zero
print("reconstructed_q_forward:", reconstructed_q_forward.shape)

num_motors = reconstructed_q_forward.shape[0]
# num_motors = 12
num_pcs = 4 # number of PCs to use

# shape: (12, N) => 12 joints, N time steps

N = reconstructed_q_forward.shape[1]  # number of frames in your playback

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
# 6. Main loop
###############################################################################
frame_idx = 0  # for indexing into our reconstructed_q_forward
max_time = 12.0 + (N * dt)  # stand-up for 3s, then playback frames at ~dt

while running_time < max_time:
    step_start = time.perf_counter()
    running_time += dt

    #------------------------------------------
    # Phase 1: Stand up for first 3 seconds
    #------------------------------------------
    if running_time < stand_up_time:
        # Transition from stand_down to stand_up
        phase = np.tanh(running_time / 1.2)
        for i in range(12):
            cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + \
                                 (1 - phase) * stand_down_joint_pos[i]
            cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

    #------------------------------------------
    # Phase 2: Playback reconstructed angles
    #------------------------------------------
    else:
        new_stand = np.zeros((12, commands.shape[1]))
        for j in range(new_stand.shape[1]):
            for i in range(12):
                new_stand[i, j] = stand_up_joint_pos[i]
        reconstructed_q_forward = new_stand + commands
        # We'll map time to frames. For simplicity:
        #    frame_idx increments by 1 each loop => ~500 Hz
        # If your original data was at a different rate,
        # you may want to do time-based interpolation.
        if frame_idx < N:
            # for each of the joints
            for i in range(num_motors):
                # Use the reconstructed angle from your PCA
                cmd.motor_cmd[i].q = reconstructed_q_forward[i, frame_idx]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
            frame_idx += 1
        else:
            print(running_time)
            # If we run out of frames, just hold the last posture
            for i in range(num_motors):
                cmd.motor_cmd[i].q = reconstructed_q_forward[i, -1]
                cmd.motor_cmd[i].kp = 50.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0

    #------------------------------------------
    # 7. Publish the command
    #------------------------------------------
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)

    #------------------------------------------
    # 8. Sleep until next iteration
    #------------------------------------------
    time_until_next_step = dt - (time.perf_counter() - step_start)
    print(time_until_next_step)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

print("Simulation complete.")
