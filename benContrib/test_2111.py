import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

dt = 0.002
crc = CRC()


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

    # Stand-up phase
    stand_up()

    # Indices for joints
    joint_indices = {
        0: [0, 1, 2],  # Front Right Leg
        1: [3, 4, 5],  # Front Left Leg
        2: [6, 7, 8],  # Rear Right Leg
        3: [9, 10, 11]  # Rear Left Leg
    }

    # Calculate offsets from stand-up position
    hip_flex_offset = stand_up_joint_pos[1::3]  # Hip flexion joints
    knee_offset = stand_up_joint_pos[2::3]  # Knee joints

    # Gait parameters
    freq = 0.5  # Step frequency in Hz
    T = 1 / freq  # Gait cycle period
    duty_factor = 0.6  # Proportion of gait cycle in stance phase
    swing_time = (1 - duty_factor) * T
    stance_time = duty_factor * T
    step_length = 0.1  # Amplitude of hip flexion angle (radians)
    foot_height = 0.2  # Additional knee flexion during swing phase (radians)

    # Phase offsets for a trot gait (diagonal legs move together)
    phase_offsets = [0.0, T / 2, 0.0, T / 2]  # Legs 0-3

    # Control gains
    kp_value = 50.0
    kd_value = 1.0

    # Desired body pitch angle (in radians)
    body_pitch_angle = 0.0  # Adjust as needed

    # Walking phase
    print("Walking")
    walking_time = 0.0

    while True:
        step_start = time.perf_counter()
        walking_time += dt

        for leg in range(4):
            # Calculate leg phase within the gait cycle
            leg_phase = (walking_time + phase_offsets[leg]) % T

            # Determine if the leg is in stance or swing phase
            if leg_phase < stance_time:
                # **Stance Phase**
                phase_ratio = leg_phase / stance_time
                # Hip flexion decreases to move foot backward
                hip_flex_angle = hip_flex_offset[leg] - step_length * phase_ratio
                # Knee angle remains constant
                knee_angle = knee_offset[leg]
            else:
                # **Swing Phase**
                phase_ratio = (leg_phase - stance_time) / swing_time
                # Hip flexion increases to move foot forward
                hip_flex_angle = hip_flex_offset[leg] - step_length + step_length * phase_ratio
                # Knee flexion increases to lift foot
                knee_angle = knee_offset[leg] - foot_height * np.sin(np.pi * phase_ratio)

            # **Set joint commands**

            # Hip Abduction/Adduction (kept at stand-up position)
            hip_abd_index = joint_indices[leg][0]
            cmd.motor_cmd[hip_abd_index].q = stand_up_joint_pos[leg * 3 + 0]
            cmd.motor_cmd[hip_abd_index].dq = 0.0
            cmd.motor_cmd[hip_abd_index].kp = kp_value
            cmd.motor_cmd[hip_abd_index].kd = kd_value

            # Hip Flexion/Extension
            hip_flex_index = joint_indices[leg][1]
            cmd.motor_cmd[hip_flex_index].q = hip_flex_angle + body_pitch_angle
            cmd.motor_cmd[hip_flex_index].dq = 0.0
            cmd.motor_cmd[hip_flex_index].kp = kp_value
            cmd.motor_cmd[hip_flex_index].kd = kd_value

            # Knee Flexion/Extension
            knee_index = joint_indices[leg][2]
            cmd.motor_cmd[knee_index].q = knee_angle
            cmd.motor_cmd[knee_index].dq = 0.0
            cmd.motor_cmd[knee_index].kp = kp_value
            cmd.motor_cmd[knee_index].kd = kd_value

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        time_until_next_step = dt - (time.perf_counter() - step_start)
        if time_until_next_step > 0:
            time.sleep(time_until_next_step)
