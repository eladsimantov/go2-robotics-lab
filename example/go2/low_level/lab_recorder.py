"""
Records the robot state and the commands sent to it into a pickle file, so a run on the real robot
can be compared with the same run in the unitree_mujoco simulation (see plot_recordings.py).

Based on recordExample.py (master branch) and keeps the same data format:
    data["lowstate"]       - joint q/dq/tau_est, IMU quaternion/gyro/acc/rpy, foot forces
    data["lowcmd"]         - commanded q/dq/tau/kp/kd for each joint
    data["sportmodestate"] - base position/velocity (published by the simulator; usually not
                             available on the real robot while in low-level mode)
    data["meta"]           - extra information from the controller (e.g. phase durations)

Notes on the simulator (unitree_mujoco): it does not fill imu_state.rpy or foot_force, so roll,
pitch and yaw are computed from the quaternion in plot_recordings.py, and foot forces are zero.

Usage inside a controller (see student_shake_hands.py):
    recorder = LabRecorder("my_run")
    recorder.Start()
    ...
    recorder.Save()          # -> recordings/my_run.pkl

Standalone usage (e.g. to record the built-in "Shake Hands" (R2 + B) on the real robot):
    python3 lab_recorder.py enp0s31f6 unitree_shake_hands
    python3 lab_recorder.py lo sim_run
Stop the recording with Ctrl+C.

Author: Elad Siman Tov
Advanced Robotics Lab, Mechanical Engineering, Technion.
"""
import os
import sys
import time
import pickle

from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_, LowState_, SportModeState_

RECORDINGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")


class LabRecorder:
    def __init__(self, name, sample_interval=0.01, meta=None):
        """
        name: file name (without extension) saved under the recordings folder.
        sample_interval: seconds between saved samples (0.01 s = 100 Hz).
        meta: optional dictionary saved with the data (e.g. phase durations).
        """
        self.path = os.path.join(RECORDINGS_DIR, name + ".pkl")
        self.sample_interval = sample_interval
        self.meta = meta if meta is not None else {}
        self.records = {"lowstate": [], "lowcmd": [], "sportmodestate": []}
        self.last_sample_time = {key: -1.0 for key in self.records}
        self.subscribers = []
        self.running = False
        self.t0 = None

    def Start(self):
        """Call after ChannelFactoryInitialize(...)."""
        self.t0 = time.time()
        self.running = True
        topics = [
            ("rt/lowstate", LowState_, "lowstate", self._lowstate_sample),
            ("rt/lowcmd", LowCmd_, "lowcmd", self._lowcmd_sample),
            ("rt/sportmodestate", SportModeState_, "sportmodestate", self._sportmodestate_sample),
        ]
        for topic, msg_type, key, to_sample in topics:
            subscriber = ChannelSubscriber(topic, msg_type)
            subscriber.Init(lambda msg, key=key, to_sample=to_sample: self._handler(msg, key, to_sample), 10)
            self.subscribers.append(subscriber)
        print(f"Recording started -> {self.path}")

    def _handler(self, msg, key, to_sample):
        if not self.running:
            return
        current_time = time.time() - self.t0
        # Only record at the specified sample rate
        if current_time - self.last_sample_time[key] < self.sample_interval:
            return
        self.last_sample_time[key] = current_time
        sample = to_sample(msg)
        sample["time"] = current_time
        self.records[key].append(sample)

    @staticmethod
    def _lowstate_sample(msg: LowState_):
        return {
            "quaternion": list(msg.imu_state.quaternion),  # [w, x, y, z]
            "gyroscope": list(msg.imu_state.gyroscope),
            "accelerometer": list(msg.imu_state.accelerometer),
            "Euler": list(msg.imu_state.rpy),  # zeros in simulation
            "motor_states": [
                {
                    "actual_q": msg.motor_state[i].q,
                    "actual_dq": msg.motor_state[i].dq,
                    "actual_tau_est": msg.motor_state[i].tau_est,
                }
                for i in range(12)
            ],
            "foot_force": [msg.foot_force[i] for i in range(4)],  # zeros in simulation
        }

    @staticmethod
    def _lowcmd_sample(msg: LowCmd_):
        return {
            "motor_states": [
                {
                    "command_mode": msg.motor_cmd[i].mode,
                    "command_q": msg.motor_cmd[i].q,
                    "command_dq": msg.motor_cmd[i].dq,
                    "command_tau": msg.motor_cmd[i].tau,
                    "command_kp": msg.motor_cmd[i].kp,
                    "command_kd": msg.motor_cmd[i].kd,
                }
                for i in range(12)
            ]
        }

    @staticmethod
    def _sportmodestate_sample(msg: SportModeState_):
        return {
            "position": list(msg.position),
            "velocity": list(msg.velocity),
        }

    def Save(self):
        if not self.running:
            return
        self.running = False
        time.sleep(0.1)  # Give callbacks time to finish
        os.makedirs(RECORDINGS_DIR, exist_ok=True)
        data = dict(self.records)
        data["meta"] = self.meta
        try:
            with open(self.path, "wb") as f:
                pickle.dump(data, f)
            print(f"Recording saved to '{self.path}' "
                  f"({len(data['lowstate'])} lowstate, {len(data['lowcmd'])} lowcmd, "
                  f"{len(data['sportmodestate'])} sportmodestate samples).")
            if len(data["lowstate"]) == 0:
                print("WARNING: No lowstate data was received. Check the network interface / simulator.")
        except Exception as e:
            print("Error saving recording:", e)


if __name__ == "__main__":
    if len(sys.argv) < 2 or sys.argv[1] == "lo":
        ChannelFactoryInitialize(1, "lo")
    else:
        ChannelFactoryInitialize(0, sys.argv[1])
    name = sys.argv[2] if len(sys.argv) > 2 else "recording"

    recorder = LabRecorder(name)
    recorder.Start()
    print("Press Ctrl+C to stop recording.")
    try:
        while True:
            time.sleep(1.0)
            print(f"Recording... {time.time() - recorder.t0:.0f}s - Samples: {len(recorder.records['lowstate'])}")
    except KeyboardInterrupt:
        pass
    recorder.Save()
