"""
Low-level shake-hands example for the Go2 robot.

The controller follows the same publisher/subscriber pattern as the working stand example, but the
motion logic is organized as a seven-phase sequence:
1. Stand up from the default low-level posture.
2. Lean backward into the support pose by defining a task space endpoint.
3. Move the front-right leg into the shake-hands configuration.
4. Hold the shake-hands pose and wave the front-right leg.
5. Return the front-right leg back to the shake-hands configuration.
6. Recover from the lean-back pose and stand back up.
7. Return to the default low-level posture and stop.

Each phase uses joint-space interpolation with position control and simple gain scheduling.

---------------------------------------
Author: Elad Siman Tov
Date: 2026-07
"""
import time
import sys
import numpy as np

from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_
from unitree_sdk2py.utils.crc import CRC
from unitree_sdk2py.utils.thread import RecurrentThread
import unitree_legged_const as go2
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient
from trajGen import MinJerk


def to_unitree(theta_1, theta_2, theta_3):
    return np.array([theta_1, -theta_2, -90.0 - theta_3])


# Base pose parameters for Leaning Back (extracted from the working manual configuration)
# This includes a shift to the left (y = 4.7cm) and a roll to the left (roll = 5.2 deg)
# to shift the center of mass over the FL-RR-RL support triangle before lifting the FR leg.
LEAN_BACK_BASE_POS = np.array([-0.055, 0.03, 0.23])   # x-forward, y-left, z-up (relative to the standing pose)
LEAN_BACK_BASE_RPY = np.array([0.00, -0.2, 0.1])  # Roll=5.2 deg, Pitch=-3.0 deg, Yaw=-3.3 deg

try:
    import os
    sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "..")))
    from unitreeGo2Model import forward_kinematics, inverse_kinematics, standing_configuration, unitree_joints_from_full_configuration, is_contact
    
    # Calculate leanBack_q_rad dynamically using analytical IK
    q0_joints = standing_configuration()
    start_pos = np.array([0.0, 0.0, 0.28])     # Standing height of 28cm
    start_rpy = np.array([0.0, 0.0, 0.0])      # No rotation
    
    planted_feet_world = forward_kinematics(
        configuration=q0_joints,
        base_position=start_pos,
        base_rpy=start_rpy
    )
    
    success, _, q_sol = inverse_kinematics(
        target_positions=planted_feet_world,
        initial_configuration=q0_joints,
        base_position=LEAN_BACK_BASE_POS,
        base_rpy=LEAN_BACK_BASE_RPY
    )
    
    if success:
        leanBack_q_rad = unitree_joints_from_full_configuration(q_sol)
        print("Successfully calculated leanBack_q_rad dynamically via analytical IK:")
        print("  leanBack_q_rad (deg):", np.degrees(leanBack_q_rad))
    else:
        raise ValueError("IK calculation failed to converge.")
except Exception as e:
    print(f"Failed to calculate leanBack_q_rad dynamically ({e}). Using default fallback values.")
    leanBack_q_rad = np.deg2rad([
        -5.0, 35.0, -85.0,
        -12.0, 30.0, -90.0,
        -19.0, 40.0, -90.0,
        -19.0, 40.0, -95.0,
    ])

lieDown_q_rad = np.array([-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                             -0.5, 1.36, -2.65, 0.5, 1.36, -2.65])

shakeHands_thetaFR_deg = np.array([-20.0, 60.0, 20.0])
shakeHands_qFR_rad = np.deg2rad(to_unitree(*shakeHands_thetaFR_deg))
shakeHands_sinAmp_rad = np.deg2rad(20)
shakeHands_nWaves = 2

standUp_q_rad = np.array([-0.04, 0.67, -1.3, 0.04, 0.67, -1.3, -0.04, 0.67, -1.3, 0.04, 0.67, -1.3])
standDown_q_rad = np.array([
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
    0.0473455, 1.22187, -2.44375, -0.0473455, 1.22187, -2.44375,
])

shakeHands_q_rad = np.concatenate([
    shakeHands_qFR_rad,
    leanBack_q_rad[3:12],
])
speedFactor = 1.3  # Speed factor to adjust the duration of each phase
standUp_Time = 3.0/speedFactor
leanBack_Time = 2.0/speedFactor
liftHand_Time = 2.0/speedFactor
waveHand_Time = 2.5/speedFactor
lowerHand_Time = 2.0/speedFactor
leanBackReturn_Time = 2.0/speedFactor
standDown_Time = 3.0/speedFactor
lieDown_Time = 2.0/speedFactor
damping_Time = 3.0/speedFactor

SIM_TIME = (
    standUp_Time
    + leanBack_Time
    + liftHand_Time
    + waveHand_Time
    + lowerHand_Time
    + leanBackReturn_Time
    + standDown_Time
    + lieDown_Time
    + damping_Time
)

class ShakeHands:
    def __init__(self, is_simulation: bool = False):
        self.is_simulation = is_simulation
        self.Kp = 60.0
        self.Kd = 5.0
        self.dt = 0.002

        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        self.low_state = None
        self.startPos = np.zeros(12)
        self.firstRun = True
        self.running_time = 0.0

        self.lowCmdWriteThreadPtr = None
        self.crc = CRC()

        self.standUp_traj = MinJerk(standDown_q_rad, standUp_q_rad, T=1.0)
        self.leanBack_traj = MinJerk(standUp_q_rad, leanBack_q_rad, T=1.0)
        self.liftHand_traj = MinJerk(leanBack_q_rad, shakeHands_q_rad, T=1.0)
        self.ending_traj = MinJerk(standDown_q_rad, lieDown_q_rad, T=1.0)

    def Init(self):
        self.InitLowCmd()

        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10)

        if not self.is_simulation:
            self.sc = SportClient()
            self.sc.SetTimeout(5.0)
            self.sc.Init()

            self.msc = MotionSwitcherClient()
            self.msc.SetTimeout(5.0)
            self.msc.Init()

            status, result = self.msc.CheckMode()
            while result and result.get('name'):
                self.sc.StandDown()
                self.msc.ReleaseMode()
                status, result = self.msc.CheckMode()
                time.sleep(1)

    def Start(self):
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=self.dt, target=self.LowCmdWrite, name="writebasiccmd"
        )
        self.lowCmdWriteThreadPtr.Start()

    def InitLowCmd(self):
        self.low_cmd.head[0] = 0xFE
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF
        self.low_cmd.gpio = 0
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01
            self.low_cmd.motor_cmd[i].q = go2.PosStopF
            self.low_cmd.motor_cmd[i].kp = 0
            self.low_cmd.motor_cmd[i].dq = go2.VelStopF
            self.low_cmd.motor_cmd[i].kd = 0
            self.low_cmd.motor_cmd[i].tau = 0

    def LowStateMessageHandler(self, msg: LowState_):
        self.low_state = msg

    def _lerp(self, start, end, phase):
        return (1.0 - phase) * start + phase * end

    def _write_joint_command(self, q_des, kp_des, kd_des, dq_des=None, tau_des=0.0):
        q_des = np.asarray(q_des, dtype=float)
        kp_des = np.asarray(kp_des, dtype=float)
        kd_des = np.asarray(kd_des, dtype=float)
        if dq_des is None:
            dq_des = np.zeros(12)
        else:
            dq_des = np.asarray(dq_des, dtype=float)

        for i in range(12):
            self.low_cmd.motor_cmd[i].q = float(q_des[i])
            self.low_cmd.motor_cmd[i].kp = float(kp_des[i])
            self.low_cmd.motor_cmd[i].dq = float(dq_des[i])
            self.low_cmd.motor_cmd[i].kd = float(kd_des[i])
            self.low_cmd.motor_cmd[i].tau = float(tau_des)

    def LowCmdWrite(self):
        if self.low_state is None:
            return

        # Print foot contact and force states every 0.5 seconds (250 steps)
        if int(self.running_time / self.dt) % 250 == 0:
            print(f"[Time: {self.running_time:.2f}s] Contacts -> "
                  f"FR: {is_contact(self.low_state, 'FR')}, "
                  f"FL: {is_contact(self.low_state, 'FL')}, "
                  f"RR: {is_contact(self.low_state, 'RR')}, "
                  f"RL: {is_contact(self.low_state, 'RL')} | "
                  f"Forces: {[self.low_state.foot_force[i] for i in range(4)]}")

        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False

        self.running_time += self.dt

        # Define time boundaries for each phase
        t1 = standUp_Time
        t2 = t1 + leanBack_Time
        t3 = t2 + liftHand_Time
        t4 = t3 + waveHand_Time
        t5 = t4 + lowerHand_Time
        t6 = t5 + leanBackReturn_Time
        t7 = t6 + standDown_Time
        t8 = t7 + lieDown_Time

        # Stand up phase
        if self.running_time < t1:
            phase = np.min([self.running_time / standUp_Time, 1.0])
            q_des, v_des, _ = self.standUp_traj.eval(t=phase)
            kp_des = np.full(12, phase * 50.0 + (1 - phase) * 20.0)
            kd_des = np.full(12, 3.5)
            dq_des = np.zeros(12)

        # Lean back phase
        elif self.running_time < t2:
            phase = np.min([(self.running_time - t1) / leanBack_Time, 1.0])
            q_des, v_des, _ = self.leanBack_traj.eval(t=phase)
            kp_des = 70.0 + phase * 30.0 * np.array([-1, -1, -1, 0, 1, 1, 0, 1, 1, 0, 1, 1])
            kd_des = np.full(12, 3.5)
            dq_des = v_des

        # Lift hand phase
        elif self.running_time < t3:
            phase = np.min([(self.running_time - t2) / liftHand_Time, 1.0])
            q_des, v_des, _ = self.liftHand_traj.eval(t=phase)
            kp_des = np.array([15, 15, 15, 70, 100, 100, 70, 100, 100, 70, 100, 100], dtype=float)
            kd_des = np.array([1, 1, 1, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5], dtype=float)
            dq_des = v_des

        # Wave hand phase
        elif self.running_time < t4:
            phase = np.min([(self.running_time - t3) / waveHand_Time, 1.0])
            q_des, _, _ = self.liftHand_traj.eval(t=1.0)
            q_des[0] += shakeHands_sinAmp_rad/4 * np.sin(2.0 * np.pi * shakeHands_nWaves * phase)
            q_des[1] += shakeHands_sinAmp_rad/4 * np.sin(2.0 * np.pi * shakeHands_nWaves * phase)
            q_des[2] += shakeHands_sinAmp_rad * np.sin(2.0 * np.pi * shakeHands_nWaves * phase)
            kp_des = np.array([15, 15, 15, 70, 100, 100, 70, 100, 100, 70, 100, 100], dtype=float)
            kd_des = np.array([1, 1, 1, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5], dtype=float)
            dq_des = np.zeros(12)

        # lower hand phase
        elif self.running_time < t5:
            phase = np.min([(self.running_time - t4) / lowerHand_Time, 1.0])
            q_des, v_des, _ = self.liftHand_traj.eval(t=1.0 - phase)
            kp_des = np.array([15, 15, 15, 70, 100, 100, 70, 100, 100, 70, 100, 100], dtype=float)
            kd_des = np.array([1, 1, 1, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5], dtype=float)
            dq_des = v_des

        # Lean back return phase
        elif self.running_time < t6:
            phase = np.min([(self.running_time - t5) / leanBackReturn_Time, 1.0])
            q_des, v_des, _ = self.leanBack_traj.eval(t=1.0 - phase)
            kp_des = 70.0 + (1.0 - phase) * 30.0 * np.array([-1, -1, -1, 0, 1, 1, 0, 1, 1, 0, 1, 1])
            kd_des = np.full(12, 3.5)
            dq_des = v_des

        # Stand down phase
        elif self.running_time < t7:
            phase = np.min([(self.running_time - t6) / standDown_Time, 1.0])
            q_des, v_des, _ = self.standUp_traj.eval(t=1.0 - phase)
            kp_des = np.full(12, 30.0 * phase + (1.0 - phase) * 70.0)
            kd_des = np.full(12, 3.5)
            dq_des = v_des

        # Lie down phase
        elif self.running_time < t8:
            phase = np.min([(self.running_time - t7) / lieDown_Time, 1.0])
            q_des, _, _ = self.ending_traj.eval(t=phase)
            kp_des = np.full(12, 30.0)
            kd_des = np.full(12, 3.5)
            dq_des = np.zeros(12)

        else:
            for i in range(12):
                self.low_cmd.motor_cmd[i].mode = 0x01
                self.low_cmd.motor_cmd[i].q = 0.0
                self.low_cmd.motor_cmd[i].kp = 0.0
                self.low_cmd.motor_cmd[i].dq = 0.0
                self.low_cmd.motor_cmd[i].kd = 0.0
                self.low_cmd.motor_cmd[i].tau = 0.0
            self.low_cmd.crc = self.crc.Crc(self.low_cmd)
            self.lowcmd_publisher.Write(self.low_cmd)
            return

        self._write_joint_command(q_des, kp_des, kd_des, dq_des=dq_des)
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        self.lowcmd_publisher.Write(self.low_cmd)

if __name__ == '__main__':

    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    input("Press Enter to continue...")

    is_sim = False
    if len(sys.argv) < 2:
        ChannelFactoryInitialize(1, "lo")
        is_sim = True
    elif sys.argv[1] == "lo":
        ChannelFactoryInitialize(1, "lo")
        is_sim = True
    else:
        ChannelFactoryInitialize(0, sys.argv[1])

    shakehands = ShakeHands(is_simulation=is_sim)
    shakehands.Init()
    shakehands.Start()

    while True:
        try:
            if shakehands.running_time >= SIM_TIME:
                time.sleep(1)
                print("Done!")
                sys.exit(0)
            time.sleep(1)
        except KeyboardInterrupt:
            for i in range(12):
                shakehands.low_cmd.motor_cmd[i].mode = 0x01
                shakehands.low_cmd.motor_cmd[i].q = 0.0
                shakehands.low_cmd.motor_cmd[i].kp = 0.0
                shakehands.low_cmd.motor_cmd[i].dq = 0.0
                shakehands.low_cmd.motor_cmd[i].kd = 0.0
                shakehands.low_cmd.motor_cmd[i].tau = 0.0
            shakehands.low_cmd.crc = shakehands.crc.Crc(shakehands.low_cmd)
            shakehands.lowcmd_publisher.Write(shakehands.low_cmd)
            break
            
