import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.utils.crc import CRC

from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__SportModeState_



from scipy.spatial.transform import Rotation

from trajGen import MinJerk

# # ----------------------------------------------- #
# # patch for importing module not pip installed.
# sys.path.append("../../../go2-robotics-lab/Go2py")
# from Go2Py.robot.model import Go2Model
# benben = Go2Model()
# sys.path.append("../../unitree_mujocu/example/python")
# # ----------------------------------------------- #


def to_unitree(theta_1, theta_2, theta_3):
    """
    Convert analytical leg angles to Unitree's motor convention.
    Ref: https://observablehq.com/@christophe-yamahata/inverse-kinematics-go2-robot

    Mapping:
        q_hip   = theta_1
        q_thigh = -theta_2
        q_calf  = -90 - theta_3
    """
    return np.array([theta_1, (-theta_2), ( -90.0 - theta_3 )])

class ControlParametersMIMO:
    def __init__(self, Kp=np.zeros(12), Kd=np.zeros(12), Tau_ff=np.zeros(12), Q_des=np.zeros(12), dQ_des=np.zeros(12)):
        self.Kp = Kp
        self.Kd = Kd
        self.Tau_ff = Tau_ff
        self.Q_des = Q_des
        self.dQ_des = dQ_des

    def update(self, i, kp=None, kd=None, tau_ff=None, q_des=None, dq_des=None):
        if kp is not None:
            self.Kp[i] = kp
        if kd is not None:
            self.Kd[i] = kd
        if tau_ff is not None:
            self.Tau_ff[i] = tau_ff
        if q_des is not None:
            self.Q_des[i] = q_des
        if dq_des is not None:
            self.dQ_des[i] = dq_des

def is_safe(foot_forces, threshold=30.0):
    """
    Check if the foot forces are above a safe threshold.
    """
    return np.all(np.abs(foot_forces) > threshold)

latest_lowstate = None
def LowStateMessageHandler(msg: LowState_):
    global latest_lowstate
    latest_lowstate = msg
    # low_state = msg
    # print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
    # print("IMU state: ", msg.imu_state)
    # print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)

latest_highstate = None
def HighStateMessageHandler(msg: SportModeState_):
    global latest_highstate
    latest_highstate = msg

C  = ControlParametersMIMO()

# --- Constants & Setup ---
# LEAN_BACK_Q_RAD = np.deg2rad([
#     0.0,  27.0, -89.0,   # FR
#     0.0,  19.0, -68.0,   # FL
#     0.0,  19.0, -68.0,   # RR
#     0.0,  19.0, -68.0    # RL
# ])

LEAN_BACK_Q_RAD = np.deg2rad([
    -5.0,  35.0, -85.0,   # FR
    -12.0, 30.0, -90.0,   # FL
    -19.0, 40.0, -90.0,   # RR
    -19.0, 40.0, -95.0    # RL
])

LIE_DOWN_Q_RAD = np.array([-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                             -0.5, 1.36, -2.65, 0.5, 1.36, -2.65])

SHAKE_HANDS_THETA_DEG = np.array([-9.0, 52.0, 20.0])
SHAKE_HANDS_SINEAMP_RAD = np.deg2rad(15)

STAND_UP_Q_RAD = np.array([0.0, 0.67, -1.3] * 4)
STAND_DOWN_Q_RAD = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375, 0.0473455,
    1.22187, -2.44375, -0.0473455, 1.22187, -2.44375
])

SHAKE_HANDS_Q_RAD = np.concatenate([np.deg2rad(to_unitree(*SHAKE_HANDS_THETA_DEG)), LEAN_BACK_Q_RAD[3:12]])

standUp_Time = 3.0 # Time to complete the stand up
leanBack_Time = 2.0 # Time to complete the lean
liftHand_Time = 2.0 # Time to lift the hand up
waveHand_Time = 3.0 # Time to wave the hand

DT = 0.002
SIM_TIME = (standUp_Time + leanBack_Time + liftHand_Time)*2 + waveHand_Time + 5.0 # Total simulation time (including some hold time at the end)
running_time = 0.0
crc = CRC()

# Initialize Trajectory
standUp_traj = MinJerk(STAND_DOWN_Q_RAD, STAND_UP_Q_RAD, T=1.0)
leanBack_traj = MinJerk(STAND_UP_Q_RAD, LEAN_BACK_Q_RAD, T=1.0) # Time is treated as a phase percentage, so actual time is controlled in the loop
liftHand_traj = MinJerk(LEAN_BACK_Q_RAD, SHAKE_HANDS_Q_RAD, T=1.0) # Placeholder, will update the trajectory points in the loop for lifting the hand
ending_traj = MinJerk(STAND_DOWN_Q_RAD, LIE_DOWN_Q_RAD, T=1.0) # A rigid trajectory for lying down at the end.

input("Press enter to start Lean Back")

if __name__ == '__main__':
    if len(sys.argv) < 2:
        """
        If no interface is given, by default we will initialize the channel to use the simulation 
        interface. To use on the real robot, please provide the interface name (e.g., python3 go2_shake_hands.py enp0s31f6) in the command line argument.
        """
        ChannelFactoryInitialize(1, "lo")
    elif sys.argv[1] == "lo":
        ChannelFactoryInitialize(1, "lo")
    else:
        """
        Initiallize the channel with the provided interface name. 
        For example, if the robot is connected to the computer via an Ethernet cable, 
        the interface name is likely to be "enp0s31f6" or similar. You can check your network interfaces using the command `ip addr` or `ifconfig` in the terminal.
        """
        ChannelFactoryInitialize(0, sys.argv[1])
    
    # create subscriber to receive LowState messages
    lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
    lowstate_subscriber.Init(handler=LowStateMessageHandler)    

    # create subscriber to receive SportModeState messages
    highstate_subscriber = ChannelSubscriber("rt/sportmodestate", SportModeState_)
    highstate_subscriber.Init(handler=HighStateMessageHandler)

    # Create a publisher to publish the data defined in UserData class
    pub = ChannelPublisher("rt/lowcmd", LowCmd_)
    pub.Init()

    cmd = unitree_go_msg_dds__LowCmd_()
    cmd.head[0] = 0xFE
    cmd.head[1] = 0xEF
    cmd.level_flag = 0xFF
    cmd.gpio = 0
    # ------------------------------------------------- #
    # --------- Damped Mode Beginning ----------------- #
    # ------------------------------------------------- #
    for i in range(12):
        cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
        cmd.motor_cmd[i].q = 0.0
        cmd.motor_cmd[i].kp = 0.0
        cmd.motor_cmd[i].dq = 0.0
        cmd.motor_cmd[i].kd = 0.0
        cmd.motor_cmd[i].tau = 0.0
        C.update(i, kp=0.0, kd=0.0, tau_ff=0.0, q_des=0.0, dq_des=0.0)

    while running_time < SIM_TIME:
        step_start = time.perf_counter()
        running_time += DT

        # ------------------------------------------------- #
        # --------- Stand Down -> Stand Up ---------------- #
        # ------------------------------------------------- #
        if (running_time < standUp_Time):
            # Stand up first
            # Total time for standing up or standing down is about 1.2s
            phase = np.min([running_time / standUp_Time, 1.0]) # Ensure phase does not exceed 1.0
            for i in range(12):
                cmd.motor_cmd[i].q = phase * STAND_UP_Q_RAD[i] + (
                    1 - phase) * STAND_DOWN_Q_RAD[i]
                cmd.motor_cmd[i].kp = phase * 50.0 + (1 - phase) * 20.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=cmd.motor_cmd[i].kp, kd=cmd.motor_cmd[i].kd, tau_ff=cmd.motor_cmd[i].tau, q_des=cmd.motor_cmd[i].q, dq_des=cmd.motor_cmd[i].dq)

        # ------------------------------------------------- #
        # --------- Stand Up -> Lean Back ----------------- #
        # ------------------------------------------------- #
        elif (running_time < standUp_Time + leanBack_Time):
            # Then lean back
            phase = np.min([(running_time - standUp_Time)/leanBack_Time, 1.0])
            q_des, v_des, _ = leanBack_traj.eval(t=phase)
            for i in range(12):
                cmd.motor_cmd[i].q = q_des[i]
                cmd.motor_cmd[i].kp = 70.0 + phase*30*np.array([-1,-1,-1,0,1,1,0,1,1,0,1,1])[i] # Higher kp for RL hip and calf
                cmd.motor_cmd[i].dq = v_des[i]
                cmd.motor_cmd[i].kd = 3.5 
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=cmd.motor_cmd[i].kp, kd=cmd.motor_cmd[i].kd, tau_ff=cmd.motor_cmd[i].tau, q_des=cmd.motor_cmd[i].q, dq_des=cmd.motor_cmd[i].dq)

        # ------------------------------------------------- #
        # ---------        Lift Hand       ---------------- #
        # ------------------------------------------------- #
        elif (running_time < standUp_Time + leanBack_Time + liftHand_Time):            
            # Then lift hand
            phase = np.min([(running_time - standUp_Time - leanBack_Time)/liftHand_Time, 1.0])
            q_des, v_des, _ = liftHand_traj.eval(t=phase)
            for i in range(12):
                cmd.motor_cmd[i].q = q_des[i]
                cmd.motor_cmd[i].kp = np.array([15,15,15,70,100,100,70,100,100,70,100,100])[i] # Lower kp for the front legs to allow for waving motion 
                cmd.motor_cmd[i].dq = v_des[i]
                cmd.motor_cmd[i].kd = np.array([1,1,1,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5])[i] 
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=cmd.motor_cmd[i].kp, kd=cmd.motor_cmd[i].kd, tau_ff=cmd.motor_cmd[i].tau, q_des=cmd.motor_cmd[i].q, dq_des=cmd.motor_cmd[i].dq)
        
        # ------------------------------------------------- #
        # --------- Lift Hand -> Wave Hand ---------------- #
        # ------------------------------------------------- #
        elif (running_time < standUp_Time + leanBack_Time + liftHand_Time + waveHand_Time):
            # Hold the position for a while to wave the hand
            phase = np.min([(running_time - standUp_Time - leanBack_Time - liftHand_Time)/waveHand_Time, 1.0])
            q_des, v_des, _ = liftHand_traj.eval(t=1.0)
            q_des[2] = SHAKE_HANDS_Q_RAD[2] + SHAKE_HANDS_SINEAMP_RAD * np.sin(2*np.pi*2*phase) # Add waving motion to the front right leg
            for i in range(12):
                cmd.motor_cmd[i].q = q_des[i]
                cmd.motor_cmd[i].kp = np.array([15,15,15,70,100,100,70,100,100,70,100,100])[i] # Lower kp for the front legs to allow for waving motion 
                cmd.motor_cmd[i].dq = v_des[i]
                cmd.motor_cmd[i].kd = np.array([1,1,1,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5])[i]
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=cmd.motor_cmd[i].kp, kd=cmd.motor_cmd[i].kd, tau_ff=cmd.motor_cmd[i].tau, q_des=cmd.motor_cmd[i].q, dq_des=cmd.motor_cmd[i].dq)
                
        # ------------------------------------------------- #
        # ---------        ~Lift Hand      ---------------- #
        # ------------------------------------------------- #
        elif (running_time < standUp_Time + leanBack_Time + 2*liftHand_Time + waveHand_Time ):
            # Then lower the hand back down
            phase = np.min([(running_time - standUp_Time - leanBack_Time - liftHand_Time - waveHand_Time)/liftHand_Time, 1.0])
            q_des, v_des, _ = liftHand_traj.eval(t=1.0 - phase) # Reverse the trajectory for lowering the hand
            for i in range(12):
                cmd.motor_cmd[i].q = q_des[i]
                cmd.motor_cmd[i].kp = np.array([15,15,15,70,100,100,70,100,100,70,100,100])[i] # Lower kp for the front legs to allow for waving motion 
                cmd.motor_cmd[i].dq = v_des[i]
                cmd.motor_cmd[i].kd = np.array([1,1,1,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5,3.5])[i]
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=cmd.motor_cmd[i].kp, kd=cmd.motor_cmd[i].kd, tau_ff=cmd.motor_cmd[i].tau, q_des=cmd.motor_cmd[i].q, dq_des=cmd.motor_cmd[i].dq)

        # ------------------------------------------------- #
        # --------- Lean Back -> Stand Up ----------------- #
        # ------------------------------------------------- #
        elif (running_time < standUp_Time + 2*leanBack_Time + 2*liftHand_Time + waveHand_Time):
            # Then stand back up
            phase = np.min([(running_time - standUp_Time - leanBack_Time - 2*liftHand_Time - waveHand_Time)/leanBack_Time, 1.0])
            q_des, v_des, _ = leanBack_traj.eval(t=1.0 - phase) # Reverse the trajectory for standing back up
            for i in range(12):
                cmd.motor_cmd[i].q = q_des[i]
                cmd.motor_cmd[i].kp = 70.0 + (1-phase)*30*np.array([-1,-1,-1,0,1,1,0,1,1,0,1,1])[i] # Higher kp for RL hip and calf during the transition back to standing up
                cmd.motor_cmd[i].dq = v_des[i]
                cmd.motor_cmd[i].kd = 3.5 
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=cmd.motor_cmd[i].kp, kd=cmd.motor_cmd[i].kd, tau_ff=cmd.motor_cmd[i].tau, q_des=cmd.motor_cmd[i].q, dq_des=cmd.motor_cmd[i].dq)

        # ------------------------------------------------- #
        # --------- Stand Up -> Stand Down ---------------- #
        # ------------------------------------------------- #
        elif (running_time < 2*standUp_Time + 2*leanBack_Time + 2*liftHand_Time + waveHand_Time):
            # Then sit back down
            phase = np.min([(running_time - standUp_Time - 2*leanBack_Time - 2*liftHand_Time - waveHand_Time)/standUp_Time, 1.0])
            q_des, v_des, _ = standUp_traj.eval(t=1.0 - phase) # Reverse the trajectory for standing back up
            for i in range(12):
                cmd.motor_cmd[i].q = q_des[i]
                cmd.motor_cmd[i].kp = 30.0*(phase) + (1-phase)*70.0 # Lower kp during the transition back to sitting down
                cmd.motor_cmd[i].dq = v_des[i]
                cmd.motor_cmd[i].kd = 3.5 
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=cmd.motor_cmd[i].kp, kd=cmd.motor_cmd[i].kd, tau_ff=cmd.motor_cmd[i].tau, q_des=cmd.motor_cmd[i].q, dq_des=cmd.motor_cmd[i].dq)
        elif (running_time < 2*standUp_Time + 2*leanBack_Time + 2*liftHand_Time + waveHand_Time + 2.0):
            # move to lying down position to reach damping mode
            phase = np.min([(running_time - 2*standUp_Time - 2*leanBack_Time - 2*liftHand_Time - waveHand_Time)/2.0, 1.0])
            q_des, __ , _ = ending_traj.eval(t=phase) # A rigid trajectory for lying down at the end.
            for i in range(12):
                cmd.motor_cmd[i].q = q_des[i]
                cmd.motor_cmd[i].kp = 30.0 
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 3.5 
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=cmd.motor_cmd[i].kp, kd=cmd.motor_cmd[i].kd, tau_ff=cmd.motor_cmd[i].tau, q_des=cmd.motor_cmd[i].q, dq_des=cmd.motor_cmd[i].dq)
        else:
            for i in range(12):
                # End in damping mode with zero kp and kd.
                cmd.motor_cmd[i].mode = 0x01  # (PMSM) mode
                cmd.motor_cmd[i].q = 0.0
                cmd.motor_cmd[i].kp = 0.0
                cmd.motor_cmd[i].dq = 0.0
                cmd.motor_cmd[i].kd = 0.0
                cmd.motor_cmd[i].tau = 0.0
                C.update(i, kp=0.0, kd=0.0, tau_ff=0.0, q_des=0.0, dq_des=0.0)
            
        # Read Sensor data
        if latest_lowstate:
            imu_quat = latest_lowstate.imu_state.quaternion
            # print(latest_lowstate)
            if latest_highstate:
                # print(latest_highstate)
                bodyPos = latest_highstate.position
                # footForce = latest_highstate.foot_force
                footPosBody = latest_highstate.foot_position_body
                # print(f"footForce {footForce}")
                # print(f"footPosBody: {footPosBody}")
                # print(f"Body position: {bodyPos}")
                translation = np.array(latest_highstate.position)
                Rb = Rotation.from_quat([imu_quat[1], imu_quat[2], imu_quat[3], imu_quat[0]]).as_matrix()
                T = np.hstack([Rb, translation.reshape(3,1)])
                # print(benben.inverseKinematics(T, feet_pos))
                # if running_time > standUp_Time:
                    # print(f"FK: {benben.forwardKinematics(T, q_des)}")
                # benben.updateKinematicsPose(q_des,T)
                # print(f"COM: {benben.robot.com()}")

            # print(translation)
            # print(f"Latest IMU Quaternion: w={imu_quat[0]:.2f}, x={imu_quat[1]:.2f}, y={imu_quat[2]:.2f}, z={imu_quat[3]:.2f}")

        if latest_highstate:
            x = latest_highstate.position[0]
            y = latest_highstate.position[1]
            z = latest_highstate.position[2]
            # print(f"Robot position: x={x:.3f}, y={y:.3f}, z={z:.3f}")

        # Send command
        cmd.crc = crc.Crc(cmd)
        pub.Write(cmd)

        # Timing control
        elapsed = time.perf_counter() - step_start
        if DT - elapsed > 0:
            time.sleep(DT - elapsed)
    print("Shake Hands sequence complete.")
    
