import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# Define stand-up joint positions (same as your provided values)
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
    running_time = 0.0

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

    while True:
        step_start = time.perf_counter()
        running_time += dt

        if running_time < stand_up_time:
            # Stand up in first 3 seconds
            phase = np.tanh(running_time / 1.2)
            for i in range(12):
                cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i] + (1 - phase) * stand_down_joint_pos[i]
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
        else:
            return  # Exit the stand-up function

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)

if __name__ == '__main__':

    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    # Create a publisher to publish the data defined in UserData class
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
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].tau = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].kd = 0.0

    # Control gains
    kp_value = 50.0
    kd_value = 1.0

    # Gait parameters
    frequency = 0.5   # Step frequency (Hz)
    amplitude = 0.2   # Joint position amplitude (radians)
    phase_offsets = [0, np.pi, np.pi, 0]  # Phases for trot gait

    # Joint indices for each leg
    joint_indices = {
        0: [0, 1, 2],   # Front Left Leg
        1: [3, 4, 5],   # Front Right Leg
        2: [6, 7, 8],   # Rear Left Leg
        3: [9, 10, 11]  # Rear Right Leg
    }

    # Main control loop
    print("Standing up")
    stand_up_time = 3.0
    running_time = 0.0

    # Stand-up phase
    stand_up()
    # while running_time < stand_up_time:
    #     step_start = time.perf_counter()
    #     running_time += dt
    #
    #     phase = np.tanh(running_time / 1.2)
    #     for i in range(12):
    #         cmd.motor_cmd[i].q = phase * stand_up_joint_pos[i]
    #         cmd.motor_cmd[i].dq = 0.0
    #         cmd.motor_cmd[i].kp = phase * kp_value
    #         cmd.motor_cmd[i].kd = kd_value
    #         cmd.motor_cmd[i].tau = 0.0
    #
    #     cmd.crc = crc.Crc(cmd)
    #     pub.Write(cmd)
    #
    #     time_until_next_step = dt - (time.perf_counter() - step_start)
    #     if time_until_next_step > 0:
    #         time.sleep(time_until_next_step)

    # Walking phase
    print("Walking")
    walking_time = 0.0

    while True:
        step_start = time.perf_counter()
        walking_time += dt

        joint_positions = []
        for leg in range(4):
            hip_abd_index = joint_indices[leg][0]
            hip_flex_index = joint_indices[leg][1]
            knee_index = joint_indices[leg][2]

            # Hip Abduction/Adduction (kept at stand-up position)
            cmd.motor_cmd[hip_abd_index].q = stand_up_joint_pos[hip_abd_index]
            cmd.motor_cmd[hip_abd_index].dq = 0.0
            cmd.motor_cmd[hip_abd_index].kp = kp_value
            cmd.motor_cmd[hip_abd_index].kd = kd_value
            cmd.motor_cmd[hip_abd_index].tau = 0.0

            # Hip Flexion/Extension (sinusoidal pattern)
            leg_phase = phase_offsets[leg]
            hip_flexion_angle = amplitude * np.sin(2 * np.pi * frequency * walking_time + leg_phase)
            cmd.motor_cmd[hip_flex_index].q = stand_up_joint_pos[hip_flex_index] + hip_flexion_angle
            cmd.motor_cmd[hip_flex_index].dq = 0.0
            cmd.motor_cmd[hip_flex_index].kp = kp_value
            cmd.motor_cmd[hip_flex_index].kd = kd_value
            cmd.motor_cmd[hip_flex_index].tau = 0.0

            # Knee Flexion/Extension (synchronized with hip flexion)
            knee_flexion_angle = amplitude * np.sin(2 * np.pi * frequency * walking_time + leg_phase + np.pi)
            cmd.motor_cmd[knee_index].q = stand_up_joint_pos[knee_index] + knee_flexion_angle
            cmd.motor_cmd[knee_index].dq = 0.0
            cmd.motor_cmd[knee_index].kp = kp_value
            cmd.motor_cmd[knee_index].kd = kd_value
            cmd.motor_cmd[knee_index].tau = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
