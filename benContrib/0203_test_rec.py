#!/usr/bin/env python3
import time
import sys
import pickle
import numpy as np
import signal
from scipy.interpolate import interp1d

# DDS communications (publisher and subscriber)
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
# Message definitions for commands and measurements
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, WirelessController_
from unitree_sdk2py.utils.crc import CRC

###############################################################################
# Global storage for logging
###############################################################################
lowstate_records = []
lowcmd_records   = []
joystick_records = []
running = True

###############################################################################
# DDS Callback Handlers
###############################################################################
def LowCmdHandler(msg: LowCmd_, t0):
    global lowcmd_records, running
    if not running:
        return
    current_time = time.time() - t0
    data = {
        "time": current_time,
        "motor_states": [
            {
                "command_mode": msg.motor_cmd[i].mode,
                "command_q":    msg.motor_cmd[i].q,
                "command_dq":   msg.motor_cmd[i].dq,
                "command_tau":  msg.motor_cmd[i].tau,
                "command_kp":   msg.motor_cmd[i].kp,
                "command_kd":   msg.motor_cmd[i].kd
            }
            for i in range(12)
        ]
    }
    lowcmd_records.append(data)

def LowStateHandler(msg: LowState_, t0):
    global lowstate_records, running
    if not running:
        return
    current_time = time.time() - t0
    data = {
        "time": current_time,
        "quaternion": list(msg.imu_state.quaternion),
        "gyroscope": list(msg.imu_state.gyroscope),
        "accelerometer": list(msg.imu_state.accelerometer),
        "Euler": list(msg.imu_state.rpy),
        "motor_states": [
            {
                "actual_q":       msg.motor_state[i].q,
                "actual_dq":      msg.motor_state[i].dq,
                "actual_tau_est": msg.motor_state[i].tau_est
            }
            for i in range(12)
        ],
        "foot_force": [msg.foot_force[i] for i in range(4)]
    }
    lowstate_records.append(data)

def JoystickHandler(msg: WirelessController_, t0):
    global joystick_records, running
    if not running:
        return
    current_time = time.time() - t0
    data = {
        "time": current_time,
        "lx": msg.lx,
        "ly": msg.ly,
        "rx": msg.rx,
        "ry": msg.ry,
        "keys_raw": msg.keys
    }
    joystick_records.append(data)

def signal_handler(sig, frame):
    global running
    print("Terminating logging...")
    running = False

signal.signal(signal.SIGINT, signal_handler)

###############################################################################
# Main function
###############################################################################
def main():
    # Initialize DDS communications in simulation mode.
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    # Create subscribers for measurements
    sub_lowstate = ChannelSubscriber("rt/lowstate", LowState_)
    sub_lowcmd   = ChannelSubscriber("rt/lowcmd",   LowCmd_)
    sub_joystick = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)

    t0 = time.time()
    sub_lowstate.Init(lambda msg: LowStateHandler(msg, t0), 10)
    sub_lowcmd.Init(lambda msg: LowCmdHandler(msg, t0), 10)
    sub_joystick.Init(lambda msg: JoystickHandler(msg, t0), 10)

    # Load recorded command data
    num_motors = 12
    # lowcontrol_data2502   lowcontrol_data_sit2stand
    with open("lowcontrol_data2502.pkl", "rb") as f:
        raw = pickle.load(f)
    lowcmd_raw = raw["lowcmd"]
    num_raw_samples = len(lowcmd_raw)
    raw_time = np.zeros(num_raw_samples)
    raw_q    = np.zeros((num_raw_samples, num_motors))
    raw_dq   = np.zeros((num_raw_samples, num_motors))
    raw_tau  = np.zeros((num_raw_samples, num_motors))
    raw_kp   = np.zeros((num_raw_samples, num_motors))
    raw_kd   = np.zeros((num_raw_samples, num_motors))
    for i in range(num_raw_samples):
        raw_time[i] = lowcmd_raw[i]["time"]
        for j in range(num_motors):
            raw_q[i, j]   = lowcmd_raw[i]["motor_states"][j]["command_q"]
            raw_dq[i, j]  = lowcmd_raw[i]["motor_states"][j]["command_dq"]
            raw_tau[i, j] = lowcmd_raw[i]["motor_states"][j]["command_tau"]
            raw_kp[i, j]  = lowcmd_raw[i]["motor_states"][j]["command_kp"]
            raw_kd[i, j]  = lowcmd_raw[i]["motor_states"][j]["command_kd"]

    # Use SIMULATE_DT = 0.005 (200 Hz) from your config.
    dt = 0.005  
    sim_end_time = raw_time[-1]
    sim_time_vector = np.arange(0.0, sim_end_time, dt)

    # Interpolate recorded data onto simulation time grid.
    interp_q   = []
    interp_dq  = []
    interp_tau = []
    interp_kp  = []
    interp_kd  = []
    for m in range(num_motors):
        interp_q.append(   interp1d(raw_time, raw_q[:, m],   kind='linear', fill_value="extrapolate"))
        interp_dq.append(  interp1d(raw_time, raw_dq[:, m],  kind='linear', fill_value="extrapolate"))
        interp_tau.append( interp1d(raw_time, raw_tau[:, m], kind='linear', fill_value="extrapolate"))
        interp_kp.append(  interp1d(raw_time, raw_kp[:, m],  kind='linear', fill_value="extrapolate"))
        interp_kd.append(  interp1d(raw_time, raw_kd[:, m],  kind='linear', fill_value="extrapolate"))

    # Initialize publisher for sending commands.
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()
    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0]    = 0xFE
    cmd.head[1]    = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio       = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # PMSM mode
        cmd.motor_cmd[i].q   = 0.0
        cmd.motor_cmd[i].dq  = 0.0
        cmd.motor_cmd[i].kp  = 0.0
        cmd.motor_cmd[i].kd  = 0.0
        cmd.motor_cmd[i].tau = 0.0
    crc = CRC()

    # Initialize filtered command values (for q) using exponential filtering.
    filtered_q = np.zeros(num_motors)
    alpha = 0.2  # smoothing factor (0<alpha<1); lower = smoother

    # Define maximum allowed joint angle (2pi)
    max_angle = 2 * np.pi

    input("Press ENTER to start command sending and logging in simulation...")

    start_wall_time = time.time()
    for t in sim_time_vector:
        # Synchronize to simulation time (dt = 0.005 s).
        elapsed_wall_time = time.time() - start_wall_time
        if elapsed_wall_time < t:
            time.sleep(t - elapsed_wall_time)

        # Get new interpolated commands.
        new_q   = np.array([float(interp_q[m](t)) for m in range(num_motors)])
        new_dq  = np.array([float(interp_dq[m](t)) for m in range(num_motors)])
        new_tau = np.array([float(interp_tau[m](t)) for m in range(num_motors)])
        new_kp  = np.array([float(interp_kp[m](t)) for m in range(num_motors)])
        new_kd  = np.array([float(interp_kd[m](t)) for m in range(num_motors)])

        # Apply low-pass filtering to the joint positions.
        if t == sim_time_vector[0]:
            filtered_q = new_q.copy()
        else:
            filtered_q = alpha * new_q + (1 - alpha) * filtered_q

        # Filter out commands with high magnitude: clip to ±2pi.
        filtered_q = np.clip(filtered_q, -max_angle, max_angle)

        # Optionally, scale the gains (here reduced by 50%) to mitigate stiffness.
        new_kp = 0.5 * new_kp
        new_kd = 0.5 * new_kd

        # Update command message using filtered positions and interpolated other signals.
        for motor_id in range(num_motors):
            cmd.motor_cmd[motor_id].q   = float(filtered_q[motor_id])
            cmd.motor_cmd[motor_id].dq  = new_dq[motor_id]
            cmd.motor_cmd[motor_id].tau = new_tau[motor_id]
            cmd.motor_cmd[motor_id].kp  = new_kp[motor_id]
            cmd.motor_cmd[motor_id].kd  = new_kd[motor_id]

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

    print("Finished sending commands.")
    time.sleep(1.0)
    global running
    running = False

    # Save recorded data for later analysis.
    combined_data = {
        "lowstate": lowstate_records,
        "lowcmd": lowcmd_records,
        "wirelesscontroller": joystick_records,
        "sent_commands": {
            "time": sim_time_vector.tolist(),
        }
    }
    with open("data_simulation2.pkl", "wb") as f:
        pickle.dump(combined_data, f)

    print("Data saved successfully. Exiting.")

if __name__ == "__main__":
    main()
