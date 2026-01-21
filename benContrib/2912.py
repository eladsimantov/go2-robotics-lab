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
running_time = 0.0
crc = CRC()

###############################################################################
# 3. Load your PCA results (PC_q) from a file
###############################################################################
# The code below assumes you have saved your principal component matrix (12x12)
# to a file named "pc_q.pkl" using pickle. Adjust the path/filename as needed.
with open("pc_q.pkl", "rb") as f:
    PC_q = pickle.load(f)  # shape: (12, 12), or at least (12, ?) if 12 joints

# We'll use just the first principal component:
first_pc = PC_q[:, 2]  # shape = (12,)

###############################################################################
# 4. Optional: Prompt user to start
###############################################################################
input("Press ENTER to start simulation...")

###############################################################################
# 5. Initialize the communications & publisher
###############################################################################
if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")
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
q = [[] for _ in range(12)]

stand_up_time = 3.0
while running_time<100:
    step_start = time.perf_counter()
    running_time += dt

    #------------------------------------------
    # Phase 1: Stand up for first 3 seconds
    #------------------------------------------
    if running_time < stand_up_time:
        # Transition from stand_down to stand_up in about 3 seconds
        # We apply a smoother "tanh" transition over 1.2 seconds to avoid jerks
        phase = np.tanh(running_time / 1.2)
        for i in range(12):
            cmd.motor_cmd[i].q = (phase * stand_up_joint_pos[i]
                                  + (1 - phase) * stand_down_joint_pos[i])
            cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

    #------------------------------------------
    # Phase 2: "Walk" with the first PCA vector
    #------------------------------------------
    else:
        # The simplest approach is to add a sinusoidal offset
        # to each joint, using the first PCA component
        walk_time = running_time - stand_up_time  # time in walking phase
        freq = 0.2    # Frequency of oscillation (Hz)
        amplitude = 0.5  # Scale the PCA shape -- !! TODO: 1st score is 0.8

        # Compute the sinusoidal factor
        sine_val = np.sin(2.0 * np.pi * freq * walk_time)

        for i in range(12):
            # Base posture is the stand_up position
            base_pos = stand_up_joint_pos[i]
            # PCA offset
            pca_offset = amplitude * sine_val * first_pc[i]

            cmd.motor_cmd[i].q = base_pos + pca_offset
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kp = 50.0
            cmd.motor_cmd[i].kd = 3.5
            cmd.motor_cmd[i].tau = 0.0

            q[i].append(base_pos + pca_offset)
        print("Running time: ", running_time)


    #------------------------------------------
    # 7. Publish the command
    #------------------------------------------
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)

    #------------------------------------------
    # 8. Sleep until next iteration
    #------------------------------------------
    time_until_next_step = dt - (time.perf_counter() - step_start)
    if time_until_next_step > 0:
        time.sleep(time_until_next_step)

# save the trajectory
with open("q.pkl", "wb") as f:
    pickle.dump(q, f)
