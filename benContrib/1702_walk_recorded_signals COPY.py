import time
import sys
import numpy as np
import pickle

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

###############################################################################
# 1. Load the recorded low-level command data.
###############################################################################
with open("lowcontrol_data.pkl", "rb") as f:
    data = pickle.load(f)

lowcmd_data = data["lowcmd"]
num_lowcmd_samples = len(lowcmd_data)
num_motors = 12
command_q_all = np.zeros((num_lowcmd_samples, num_motors))
for i in range(num_lowcmd_samples):
    for j in range(num_motors):
        command_q_all[i, j] = lowcmd_data[i]["motor_states"][j]["command_q"]

N = command_q_all.shape[0]
dt_synth = 1/350  # Recorded data sample period (350 Hz)

###############################################################################
# 2. Set simulation parameters.
###############################################################################
dt = 0.002         # Control loop period (500 Hz)
total_time = 10.0  # Total simulation time
kp = 10.0          # Default gains for replay phase
kd = 1.0

###############################################################################
# 3. Define stand-up interpolation parameters.
###############################################################################
# These are the same arrays used in the proper standing snippet.
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

stand_up_time = 3.0  # Duration (in seconds) for the stand-up interpolation

###############################################################################
# 4. Initialize communications.
###############################################################################
if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")  # Loopback for simulation
else:
    ChannelFactoryInitialize(0, sys.argv[1])

pub = ChannelPublisher("rt/lowcmd", LowCmd_)
pub.Init()

cmd = unitree_go_msg_dds__LowCmd_()
cmd.head[0] = 0xFE
cmd.head[1] = 0xEF
cmd.level_flag = 0xFF
cmd.gpio = 0

# Initialize motor commands
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01  # PMSM mode
    cmd.motor_cmd[i].q   = 0.0
    cmd.motor_cmd[i].kp  = 0.0
    cmd.motor_cmd[i].dq  = 0.0
    cmd.motor_cmd[i].kd  = 0.0
    cmd.motor_cmd[i].tau = 0.0

###############################################################################
# 5. Main loop: First, smoothly stand up; then replay recorded commands.
###############################################################################
input("Press ENTER to start simulation...")

crc = CRC()
start_time = time.perf_counter()

while True:
    now = time.perf_counter()
    elapsed = now - start_time
    if elapsed > total_time:
        break

    if elapsed < stand_up_time:
        # ----- Stand-Up Phase -----
        # Instead of an abrupt command, we smoothly interpolate between the
        # stand-down and stand-up poses. The tanh function provides a gradual
        # ramp from 0 to nearly 1.
        phase = np.tanh(elapsed / 1.2)
        for motor_id in range(num_motors):
            cmd.motor_cmd[motor_id].q = (phase * stand_up_joint_pos[motor_id] +
                                         (1 - phase) * stand_down_joint_pos[motor_id])
            # Gain scheduling: softer gains initially then stiffer as we approach stand-up.
            cmd.motor_cmd[motor_id].kp = phase * 50.0 + (1 - phase) * 20.0
            cmd.motor_cmd[motor_id].dq = 0.0
            cmd.motor_cmd[motor_id].kd = 3.5
            cmd.motor_cmd[motor_id].tau = 0.0
    else:
        # ----- Replay Phase -----
        # After stand-up, we replay the recorded joint commands.
        # Adjust elapsed time by subtracting the stand-up duration.
        t = elapsed - stand_up_time
        step_in_pose = int(t / dt_synth) % N
        for motor_id in range(num_motors):
            cmd.motor_cmd[motor_id].q = command_q_all[step_in_pose, motor_id]
            cmd.motor_cmd[motor_id].kp = kp
            cmd.motor_cmd[motor_id].dq = 0.0
            cmd.motor_cmd[motor_id].kd = kd
            cmd.motor_cmd[motor_id].tau = 0.0

    # Compute CRC and publish the command.
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)

    # Maintain the control loop timing.
    time.sleep(max(0.0, dt - (time.perf_counter() - now)))

print("Done.")
