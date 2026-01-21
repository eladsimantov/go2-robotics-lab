#!/usr/bin/env python3
import time
import sys
import pickle
import numpy as np
import signal
from scipy.interpolate import interp1d

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, WirelessController_
from unitree_sdk2py.utils.crc import CRC

###############################################################################
# Global variables for logging and measured state
###############################################################################
lowstate_records = []
lowcmd_records   = []
joystick_records = []
running = True
last_sample_time = 0
sample_interval = 0.01  # Sample every 0.01 seconds (100 Hz) - adjust as needed
recording_duration = 10.0  # Set recording duration (in seconds)

# We'll store the latest measured joint angles/vels and also the initial measured angles
latest_q_meas = None
latest_dq_meas = None
initial_q_meas = None
init_measured_captured = False

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
    global latest_q_meas, latest_dq_meas
    global initial_q_meas, init_measured_captured
    global last_sample_time

    if not running:
        return

    current_time = time.time() - t0
    
    # Only record at specified sample rate
    if current_time - last_sample_time < sample_interval:
        # Still update latest measurements for real-time tracking
        q_array = np.array([msg.motor_state[i].q for i in range(12)])
        dq_array = np.array([msg.motor_state[i].dq for i in range(12)])
        latest_q_meas = q_array
        latest_dq_meas = dq_array
        if not init_measured_captured:
            initial_q_meas = q_array.copy()
            init_measured_captured = True
        return
    
    last_sample_time = current_time
    
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

    # Update latest measured joint positions and velocities
    q_array = np.array([msg.motor_state[i].q for i in range(12)])
    dq_array = np.array([msg.motor_state[i].dq for i in range(12)])
    latest_q_meas = q_array
    latest_dq_meas = dq_array

    # Capture the initial measured angles at t=0
    if not init_measured_captured:
        initial_q_meas = q_array.copy()
        init_measured_captured = True

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
    global latest_q_meas, latest_dq_meas, initial_q_meas, init_measured_captured
    # Initialize DDS
    if len(sys.argv) > 1:
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        ChannelFactoryInitialize(1, "lo")

    # Create subscribers
    sub_lowstate = ChannelSubscriber("rt/lowstate", LowState_)
    sub_lowcmd   = ChannelSubscriber("rt/lowcmd",   LowCmd_)
    sub_joystick = ChannelSubscriber("rt/wirelesscontroller", WirelessController_)
    print("Subscribers initialized. Starting logging...")

    print("Current lowstate data: ", sub_lowstate.Read())

    t0 = time.time()
    sub_lowstate.Init(lambda msg: LowStateHandler(msg, t0), 10)
    sub_lowcmd.Init(lambda msg: LowCmdHandler(msg, t0), 10)
    sub_joystick.Init(lambda msg: JoystickHandler(msg, t0), 10)
    
    # Wait a moment for subscribers to start receiving data
    print("\nWaiting for robot connection...")
    time.sleep(2.0)
    
    if len(lowstate_records) == 0:
        print("WARNING: Not receiving any lowstate data!")
        print("Make sure the robot is powered on and connected.")
        print("Press Ctrl+C to exit or wait to continue anyway...")
    
    print(f"\nRECORDING STARTED - Duration: {recording_duration} seconds")
    print(f"Current samples captured: {len(lowstate_records)}")
    print("Use the joystick to control the robot now!")
    print("=" * 50)
    
    # Record for specified duration with progress updates
    start_time = time.time()
    while time.time() - start_time < recording_duration:
        time.sleep(1.0)
        elapsed = time.time() - start_time
        print(f"Recording... {elapsed:.1f}s / {recording_duration}s - Samples: {len(lowstate_records)}")
    
    # time.sleep(recording_duration)
    
    print("\nRECORDING COMPLETE")
    print("=" * 50)
    print(f"Captured {len(lowstate_records)} configuration samples")
    print(f"Sample rate: {sample_interval} seconds ({1/sample_interval:.1f} Hz)")
    global running
    running = False
    time.sleep(0.5)  # Give callbacks time to finish

    # Save data
    combined_data = {
        "lowstate": lowstate_records,
        "lowcmd": lowcmd_records,
        "wirelesscontroller": joystick_records
    }
    try:
        with open("recorded_data_joystick.pkl", "wb") as f:
            pickle.dump(combined_data, f)
        print("Data saved successfully to 'recorded_data_joystick.pkl'.")
    except Exception as e:
        print("Error saving data:", e)

if __name__ == "__main__":
    main()
