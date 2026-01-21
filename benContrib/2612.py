import time
import sys
import numpy as np
import pickle

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

with open("pc_q.pkl", "rb") as f:
    PC_q = pickle.load(f) # shape: (12, 12)
print("PC_q loaded from pc_q.pkl")

# The first principal component vector (12 elements):
first_pc_vector = PC_q[:, 0]  # shape: (12,)

# Define stand-up joint positions
stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763
], dtype=float)

dt = 0.002
running_time = 0.0
crc = CRC()

input("Press enter to start")

def stand_up():
    print("Standing up")
    stand_up_time = 3.0
    running_time_local = 0.0

    # Re-define stand-up vs. stand-down positions:
    stand_up_joint_pos_local = np.array([
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

    while True:
        step_start = time.perf_counter()
        running_time_local += dt

        # Gradually transition from "stand down" to "stand up" over 3 seconds
        if running_time_local < stand_up_time:
            phase = np.tanh(running_time_local / 1.2)  # smooth transition
            for i in range(12):
                cmd.motor_cmd[i].q = (
                    phase * stand_up_joint_pos_local[i]
                    + (1 - phase) * stand_down_joint_pos[i]
                )
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
        else:
            return  # Done standing up => exit function

        # Compute CRC and publish:
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        # Sleep to maintain loop timing:
        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

if __name__ == '__main__':

    # Initialize the communication
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    # Prepare the command struct
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(12):
        cmd.motor_cmd[i].mode = 0x0A  # Position control mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0
    # Gains
    kp_value = 1
    kd_value = 1.0

    # STAND-UP PHASE
    print("Standing up...")
    stand_up()  # blocks until the robot is standing

    # WALKING WITH PC
    print("Walking with the first principal component shape")
    walking_time = 0.0
    # Adjust these to taste:
    pc_amplitude = 1    # amplitude for sinusoidal PC activation
    frequency = 2       # frequency in Hz

    # First PC shape:
    pc_vector = first_pc_vector  # shape (12,)

    while True:
        step_start = time.perf_counter()
        walking_time += dt

        # Compute sinusoidal factor:
        phase = np.sin(2 * np.pi * frequency * walking_time)

        # Update each motor's position based on the PC shape:
        for i in range(12):
            cmd.motor_cmd[i].q = (
                stand_up_joint_pos[i]
                + pc_amplitude * phase * pc_vector[i]
            )
            cmd.motor_cmd[i].dq = 0.0
            cmd.motor_cmd[i].kp = kp_value
            cmd.motor_cmd[i].kd = kd_value
            cmd.motor_cmd[i].tau = 0.0

        # Compute CRC and publish:
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        # Sleep to maintain loop timing:
        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
