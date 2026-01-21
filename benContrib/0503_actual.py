#!/usr/bin/env python3
import time
import sys
import pickle
import numpy as np
import signal

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
# Callback Handlers (for optional logging)
###############################################################################
def LowCmdHandler(msg: LowCmd_, start_time):
    global lowcmd_records, running
    if not running:
        return
    current_time = time.time() - start_time
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

def LowStateHandler(msg: LowState_, start_time):
    global lowstate_records, running
    if not running:
        return
    current_time = time.time() - start_time
    data = {
        "time": current_time,
        "motor_states": [
            {
                "actual_q":       msg.motor_state[i].q,
                "actual_dq":      msg.motor_state[i].dq,
                "actual_tau_est": msg.motor_state[i].tau_est
            }
            for i in range(12)
        ]
    }
    lowstate_records.append(data)

def JoystickHandler(msg: WirelessController_, start_time):
    global joystick_records, running
    if not running:
        return
    current_time = time.time() - start_time
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
# Main
###############################################################################
def main():
    # 1) Initialize DDS communications
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    # 2) Create subscribers (optional, for logging)
    t0 = time.time()
    sub_lowstate = ChannelSubscriber("rt/lowstate", LowState_)
    sub_lowcmd   = ChannelSubscriber("rt/lowcmd",   LowCmd_)
    sub_joystick = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)

    sub_lowstate.Init(lambda msg: LowStateHandler(msg, t0), 10)
    sub_lowcmd.Init(lambda msg: LowCmdHandler(msg, t0), 10)
    sub_joystick.Init(lambda msg: JoystickHandler(msg, t0), 10)

    # 3) Load recorded data that contains "actual_q", "actual_dq", "actual_tau_est"
    with open("lowcontrol_data_sit2stand.pkl", "rb") as f:
        recorded = pickle.load(f)
    recorded_lowstate = recorded["lowstate"]

    # 4) Extract time, actual_q, actual_dq, actual_tau_est from the recorded data
    num_samples = len(recorded_lowstate)
    num_motors = 12
    times = np.zeros(num_samples)
    all_q = np.zeros((num_samples, num_motors))
    all_dq = np.zeros((num_samples, num_motors))
    all_tau = np.zeros((num_samples, num_motors))

    for i, entry in enumerate(recorded_lowstate):
        times[i] = entry["time"]
        for j in range(num_motors):
            all_q[i, j]   = entry["motor_states"][j]["actual_q"]
            all_dq[i, j]  = entry["motor_states"][j]["actual_dq"]
            all_tau[i, j] = entry["motor_states"][j]["actual_tau_est"]

    # 5) Create a publisher to send these signals directly as commands
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    for i in range(20):
        cmd.motor_cmd[i].mode = 0x01  # PMSM mode
        cmd.motor_cmd[i].q   = 0.0
        cmd.motor_cmd[i].dq  = 0.0
        cmd.motor_cmd[i].kp  = 0.0
        cmd.motor_cmd[i].kd  = 0.0
        cmd.motor_cmd[i].tau = 0.0
    crc = CRC()

    # 6) Define simulation time vector at 200 Hz (dt = 0.005)
    dt = 0.005
    sim_end_time = times[-1]
    sim_time_vector = np.arange(0, sim_end_time, dt)

    input("Press ENTER to start replaying measured signals as commands...")

    start_wall_time = time.time()
    for t in sim_time_vector:
        # Synchronize wall-clock to simulation time
        elapsed_wall_time = time.time() - start_wall_time
        if elapsed_wall_time < t:
            time.sleep(t - elapsed_wall_time)
        # Find the nearest recorded sample index for current simulation time
        # (Using nearest-neighbor approach)
        i = np.argmin(np.abs(times - t))
        
        # 7) Populate the command from the measured signals
        for motor_id in range(num_motors):
            # Clip commanded angles based on motor group limits:
            # Body (motors 0,3,6,9): -48° to 48°  → approx -0.8378 to 0.8378 rad
            # Thigh (motors 1,4,7,10): -200° to 90°  → approx -3.4907 to 1.5708 rad
            # Shank (motors 2,5,8,11): -156° to -48°  → approx -2.7227 to -0.8378 rad
            q_val = all_q[i, motor_id]
            if motor_id in [0, 3, 6, 9]:
                lower, upper = -np.deg2rad(48), np.deg2rad(48)
            elif motor_id in [1, 4, 7, 10]:
                lower, upper = -np.deg2rad(200), np.deg2rad(90)
            elif motor_id in [2, 5, 8, 11]:
                lower, upper = -np.deg2rad(156), -np.deg2rad(48)
            else:
                lower, upper = -np.inf, np.inf  # Should not occur
            q_cmd = np.clip(q_val, lower, upper)
            
            cmd.motor_cmd[motor_id].q   = float(q_cmd)
            cmd.motor_cmd[motor_id].dq  = all_dq[i, motor_id]
            cmd.motor_cmd[motor_id].tau = all_tau[i, motor_id]
            # kp and kd remain zero (direct feedthrough)
            cmd.motor_cmd[motor_id].kp  = 0.0
            cmd.motor_cmd[motor_id].kd  = 0.0

        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

    print("Finished sending commands.")
    time.sleep(1.0)
    global running
    running = False

    # 8) Optionally save the new data logs
    combined_data = {
        "lowstate": lowstate_records,
        "lowcmd": lowcmd_records,
        "wirelesscontroller": joystick_records,
        "sent_commands": {
            "time": sim_time_vector.tolist(),
            "q": all_q.tolist(),
            "dq": all_dq.tolist(),
            "tau": all_tau.tolist()
        }
    }
    with open("sim_actual_sit2stand.pkl", "wb") as f:
        pickle.dump(combined_data, f)

    print("Data saved to 'sim_actual_sit2stand.pkl'. Exiting.")

if __name__ == "__main__":
    main()
