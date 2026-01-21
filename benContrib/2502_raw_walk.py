import time
import sys
import numpy as np
import pickle

from unitree_sdk2py.core.channel import ChannelPublisher
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

# Load recorded data
with open("lowcontrol_data_sit2stand.pkl", "rb") as f:
    data = pickle.load(f)

lowcmd_data = data["lowcmd"]
num_lowcmd_samples = len(lowcmd_data)
num_motors = 12

# Allocate arrays for recorded data
command_time = np.zeros(num_lowcmd_samples)
command_q_all    = np.zeros((num_lowcmd_samples, num_motors))
command_dq_all   = np.zeros((num_lowcmd_samples, num_motors))
command_tau_all  = np.zeros((num_lowcmd_samples, num_motors))
command_kp_all   = np.zeros((num_lowcmd_samples, num_motors))
command_kd_all   = np.zeros((num_lowcmd_samples, num_motors))

for i in range(num_lowcmd_samples):
    command_time[i] = lowcmd_data[i]["time"]
    for j in range(num_motors):
        command_q_all[i, j]   = lowcmd_data[i]["motor_states"][j]["command_q"]
        command_dq_all[i, j]  = lowcmd_data[i]["motor_states"][j]["command_dq"]
        command_tau_all[i, j] = lowcmd_data[i]["motor_states"][j]["command_tau"]
        command_kp_all[i, j]  = lowcmd_data[i]["motor_states"][j]["command_kp"]
        command_kd_all[i, j]  = lowcmd_data[i]["motor_states"][j]["command_kd"]

# Initialize communications (simulation mode via loopback)
if len(sys.argv) < 2:
    ChannelFactoryInitialize(1, "lo")
else:
    ChannelFactoryInitialize(0, sys.argv[1])
pub = ChannelPublisher("rt/lowcmd", LowCmd_)
pub.Init()

# Prepare command message
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

input("Press ENTER to start simulation...")

# Replay the raw recorded signals exactly as logged
for i in range(num_lowcmd_samples):
    for motor_id in range(num_motors):
        cmd.motor_cmd[motor_id].q   = command_q_all[i, motor_id]
        cmd.motor_cmd[motor_id].dq  = command_dq_all[i, motor_id]
        cmd.motor_cmd[motor_id].tau = command_tau_all[i, motor_id]
        cmd.motor_cmd[motor_id].kp  = command_kp_all[i, motor_id]
        cmd.motor_cmd[motor_id].kd  = command_kd_all[i, motor_id]
    cmd.crc = crc.Crc(cmd)
    pub.Write(cmd)
    
    # Enforce recorded timing (average dt ≈ 0.002857 sec)
    if i < num_lowcmd_samples - 1:
        dt = command_time[i+1] - command_time[i]
        time.sleep(max(0, dt))

print("Execution complete.")
