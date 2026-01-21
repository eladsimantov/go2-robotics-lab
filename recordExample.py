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

last_sample_time = 0
sample_interval = 0.1  # Sample every 0.1 seconds (10 Hz) - adjust as needed

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

    # Set recording duration (in seconds)
    recording_duration = 10.0  # Change this to your desired duration
    
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

    # # Load recorded data
    # num_motors = 12
    # with open("sim_sit2stand.pkl", "rb") as f:    #      lowcontrol_data2502
    #     raw = pickle.load(f)
    # lowcmd_raw = raw["lowcmd"]
    # num_raw_samples = len(lowcmd_raw)
    # raw_time = np.zeros(num_raw_samples)
    # raw_q    = np.zeros((num_raw_samples, num_motors))
    # raw_dq   = np.zeros((num_raw_samples, num_motors))
    # raw_tau  = np.zeros((num_raw_samples, num_motors))
    # raw_kp   = np.zeros((num_raw_samples, num_motors))
    # raw_kd   = np.zeros((num_raw_samples, num_motors))

    # for i in range(num_raw_samples):
    #     raw_time[i] = lowcmd_raw[i]["time"]
    #     for j in range(num_motors):
    #         raw_q[i, j]   = lowcmd_raw[i]["motor_states"][j]["command_q"]
    #         raw_dq[i, j]  = lowcmd_raw[i]["motor_states"][j]["command_dq"]
    #         raw_tau[i, j] = lowcmd_raw[i]["motor_states"][j]["command_tau"]
    #         raw_kp[i, j]  = lowcmd_raw[i]["motor_states"][j]["command_kp"]
    #         raw_kd[i, j]  = lowcmd_raw[i]["motor_states"][j]["command_kd"]

    # # We'll run at 200 Hz (dt=0.005)
    # dt = 0.005
    # sim_end_time = raw_time[-1]
    # sim_time_vector = np.arange(0.0, sim_end_time, dt)

    # # Interpolate feedforward data onto simulation time grid
    # interp_q   = []
    # interp_dq  = []
    # interp_tau = []
    # interp_kp  = []
    # interp_kd  = []
    # for m in range(num_motors):
    #     interp_q.append(   interp1d(raw_time, raw_q[:, m],   kind='linear', fill_value="extrapolate"))
    #     interp_dq.append(  interp1d(raw_time, raw_dq[:, m],  kind='linear', fill_value="extrapolate"))
    #     interp_tau.append( interp1d(raw_time, raw_tau[:, m], kind='linear', fill_value="extrapolate"))
    #     interp_kp.append(  interp1d(raw_time, raw_kp[:, m],  kind='linear', fill_value="extrapolate"))
    #     interp_kd.append(  interp1d(raw_time, raw_kd[:, m],  kind='linear', fill_value="extrapolate"))

    # # Publisher
    # pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    # pub.Init()
    # cmd = unitree_go_msg_dds__LowCmd_()
    # cmd.head[0]    = 0xFE
    # cmd.head[1]    = 0xEF
    # cmd.level_flag = 0xFF
    # cmd.gpio       = 0
    # for i in range(20):
    #     cmd.motor_cmd[i].mode = 0x01
    #     cmd.motor_cmd[i].q   = 0.0
    #     cmd.motor_cmd[i].dq  = 0.0
    #     cmd.motor_cmd[i].kp  = 0.0
    #     cmd.motor_cmd[i].kd  = 0.0
    #     cmd.motor_cmd[i].tau = 0.0
    # crc = CRC()

    # # --- PID Gains ---
    # kp_fb = 25.0
    # kd_fb = 1.0
    # ki_fb = 5.0  # Integral gain to reduce steady-state error

    # # We'll clamp final torque to avoid saturations
    # max_torque = 20.0  # [Nm], tune as needed

    # # Ramp duration to avoid the initial impulse
    # ramp_time = 8.5  # seconds, tune as needed

    # # We'll store integral and previous error for each motor
    # integral_error = np.zeros(num_motors)
    # prev_error = np.zeros(num_motors)

    # input("Press ENTER to start command sending and logging in simulation...")

    # start_wall_time = time.time()
    # for step_idx, t in enumerate(sim_time_vector):
    #     # Keep in sync with real-time sim
    #     elapsed_wall_time = time.time() - start_wall_time
    #     if elapsed_wall_time < t:
    #         time.sleep(t - elapsed_wall_time)

    #     # Interpolate feedforward
    #     ff_q   = np.array([float(interp_q[m](t))   for m in range(num_motors)])
    #     ff_dq  = np.array([float(interp_dq[m](t))  for m in range(num_motors)])
    #     ff_tau = np.array([float(interp_tau[m](t)) for m in range(num_motors)])
    #     ff_kp  = np.array([float(interp_kp[m](t))  for m in range(num_motors)])
    #     ff_kd  = np.array([float(interp_kd[m](t))  for m in range(num_motors)])

    #     # 1) Ramp-in for the first ramp_time seconds
    #     if init_measured_captured and t < ramp_time:
    #         frac = t / ramp_time
    #         q_cmd = initial_q_meas + frac * (ff_q - initial_q_meas)
    #     else:
    #         q_cmd = ff_q

    #     # 2) If we have measured states, compute PID feedback
    #     feedback_tau = np.zeros(num_motors)
    #     if latest_q_meas is not None and latest_dq_meas is not None:
    #         error_q = q_cmd - latest_q_meas

    #         # Optionally use future step for derivative of the command:
    #         # if step_idx < len(sim_time_vector) - 1:
    #         #     ff_q_next = np.array([float(interp_q[m](sim_time_vector[step_idx+1])) for m in range(num_motors)])
    #         #     desired_dq = (ff_q_next - ff_q) / dt
    #         # else:
    #         #     desired_dq = ff_dq
    #         # error_dq = desired_dq - latest_dq_meas

    #         # But we'll just do standard approach: error_dq = ff_dq - dq_meas
    #         error_dq = ff_dq - latest_dq_meas

    #         # Integral
    #         integral_error += error_q * dt
    #         # Derivative
    #         derivative_error = (error_q - prev_error) / dt
    #         prev_error = error_q

    #         # PID torque
    #         feedback_tau = (
    #             kp_fb * error_q
    #             + ki_fb * integral_error
    #             + kd_fb * derivative_error
    #         )

    #     # 3) Combine feedforward torque + feedback torque
    #     final_tau = ff_tau + feedback_tau

    #     # 4) Clamp final torque
    #     final_tau = np.clip(final_tau, -max_torque, max_torque)

    #     # Update LowCmd
    #     for motor_id in range(num_motors):
    #         cmd.motor_cmd[motor_id].q   = float(q_cmd[motor_id])
    #         cmd.motor_cmd[motor_id].dq  = ff_dq[motor_id]
    #         cmd.motor_cmd[motor_id].kp  = ff_kp[motor_id]
    #         cmd.motor_cmd[motor_id].kd  = ff_kd[motor_id]
    #         cmd.motor_cmd[motor_id].tau = final_tau[motor_id]

    #     cmd.crc = crc.Crc(cmd)
    #     pub.Write(cmd)

    # print("Finished sending commands.")
    # time.sleep(1.0)
    # global running
    # running = False

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
