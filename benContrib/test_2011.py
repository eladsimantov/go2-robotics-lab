import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

dt = 0.002
running_time = 0.0
crc = CRC()

def stand_up():
    print("Standing up")
    dt = 0.002
    runing_time = 0.0
    crc = CRC()

    stand_up_joint_pos = np.array([
        0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763, -0.00571868, 0.608813, -1.21763
    ],
        dtype=float)

    stand_down_joint_pos = np.array([
        0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
        1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
    ],
        dtype=float)

    while True:
        step_start = time.perf_counter()

        runing_time += dt

        if (runing_time < 3.0):
            # Stand up in first 3 second

            # Total time for standing up or standing down is about 1.2s
            phase = np.tanh(runing_time / 1.2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (
                        1 - phase) * stand_down_joint_pos[i]
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
        else:
            return

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)



input("Press enter to start")

if __name__ == '__main__':

    stand_up_joint_pos = np.array([
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763,
        0.00571868, 0.608813, -1.21763,
        -0.00571868, 0.608813, -1.21763
    ], dtype=float)

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
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x0A  # Position control mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0

    stand_up()

    # Gait parameters
    freq = 0.5  # Reduced frequency
    amp = 0.1  # Reduced amplitude
    phase_offsets = [0.0, np.pi, np.pi, 0.0]  # Phase offsets for legs 0-3

    # Control gains
    kp_value = 20.0
    kd_value = 0.5

    # Indices for joints
    joint_indices = {
        0: [0, 1, 2],
        1: [3, 4, 5],
        2: [6, 7, 8],
        3: [9, 10, 11]
    }

    stand_up_time = 2.0  # Time to stand up

    # Walking phase
    print("Walking")

    while True:
        step_start = time.perf_counter()  # Move to the top of the loop
        running_time += dt  # Update running_time on every iteration

        # Loop through each leg
        for leg in range(4):
            phase = 2 * np.pi * freq * (running_time - stand_up_time) + phase_offsets[leg]

            # Hip Abduction/Adduction
            hip_abd_index = joint_indices[leg][0]
            cmd.motor_cmd[hip_abd_index].q = 0.0
            cmd.motor_cmd[hip_abd_index].dq = 0.0
            cmd.motor_cmd[hip_abd_index].kp = kp_value
            cmd.motor_cmd[hip_abd_index].kd = kd_value

            # Hip Flexion/Extension
            hip_flex_index = joint_indices[leg][1]
            cmd.motor_cmd[hip_flex_index].q = amp * np.sin(phase)
            cmd.motor_cmd[hip_flex_index].dq = 0.0
            cmd.motor_cmd[hip_flex_index].kp = kp_value
            cmd.motor_cmd[hip_flex_index].kd = kd_value

            # Knee Flexion/Extension
            knee_index = joint_indices[leg][2]
            cmd.motor_cmd[knee_index].q = amp * np.sin(phase + np.pi)
            cmd.motor_cmd[knee_index].dq = 0.0
            cmd.motor_cmd[knee_index].kp = kp_value
            cmd.motor_cmd[knee_index].kd = kd_value

        cmd.crc = crc.Crc(cmd)  # Moved outside the if-else blocks
        pub.Write(cmd)          # Moved outside the if-else blocks

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
