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
# 1. Load a single PCA pose array
#    The file might contain something like:
#       {
#         'pose': np.array([...]) # shape (12, N)
#         'dt_synth': 0.00285
#       }
###############################################################################
with open("original_signals.pkl", "rb") as f:
    data_pca = pickle.load(f)

single_pose = data_pca['poses'][0]       # shape: (12, N)
dt_synth   = data_pca['time_step']    # e.g., 0.00285 seconds
N = single_pose.shape[1]
num_motors = 12

stand_up_joint_pos = np.array([
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763,
    0.00571868, 0.608813, -1.21763,
    -0.00571868, 0.608813, -1.21763
], dtype=float)

###############################################################################
# 2. Set up main simulation parameters
###############################################################################
dt = 0.002            # Control loop step (500 Hz)
total_time = 10.0      # Run for 5 seconds
low_kp = 10.0         # Low P-gain
low_kd = 1.0          # Low D-gain

###############################################################################
# 3. Initialize communications
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

# Basic motor initialization
for i in range(20):
    cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
    cmd.motor_cmd[i].q   = 0.0
    cmd.motor_cmd[i].kp  = 0.0
    cmd.motor_cmd[i].dq  = 0.0
    cmd.motor_cmd[i].kd  = 0.0
    cmd.motor_cmd[i].tau = 0.0

###############################################################################
# 4. Main loop: execute single_pose for 5 seconds at low gains
###############################################################################
input("Press ENTER to start simulation...")

crc = CRC()
start_time = time.perf_counter()

while True:
    now = time.perf_counter()
    elapsed = now - start_time
    if elapsed > total_time:
        break

    step_in_pose = int(elapsed / dt_synth) % N

    for motor_id in range(num_motors):
        # Position from PCA
        # cmd.motor_cmd[motor_id].q   = stand_up_joint_pos[motor_id] + single_pose[motor_id, step_in_pose]
        cmd.motor_cmd[motor_id].q = single_pose[motor_id, step_in_pose]

        # Slow/soft gains
        cmd.motor_cmd[motor_id].kp  = low_kp
        cmd.motor_cmd[motor_id].dq  = 0.0
        cmd.motor_cmd[motor_id].kd  = low_kd
        cmd.motor_cmd[motor_id].tau = 0.0

    # Compute CRC & publish
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)

    # Maintain loop rate at dt
    time.sleep(max(0.0, dt - (time.perf_counter() - now)))

print("Done.")
