"""
Plots and compares recordings made with lab_recorder.py (e.g. simulation vs. real robot).

Usage:
    python3 plot_recordings.py                              # student_shake_hands_sim.pkl vs student_shake_hands_robot.pkl
    python3 plot_recordings.py recordings/a.pkl recordings/b.pkl ...

Time t = 0 is the first command sent by the controller, so different runs line up.
Figures are also saved as .png files in the recordings folder for your report.

Author: Elad Siman Tov
Advanced Robotics Lab, Mechanical Engineering, Technion.
"""
import os
import sys
import pickle
import numpy as np
import matplotlib.pyplot as plt

RECORDINGS_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "recordings")
DEFAULT_FILES = [
    os.path.join(RECORDINGS_DIR, "student_shake_hands_sim.pkl"),
    os.path.join(RECORDINGS_DIR, "student_shake_hands_robot.pkl"),
]

legs = ["FR", "FL", "RR", "RL"]
joints = ["hip", "thigh", "calf"]
rad2deg = 180.0 / np.pi


def quaternion_to_rpy(quat):
    """quat = [w, x, y, z] (Unitree / MuJoCo order) -> [roll, pitch, yaw] in radians."""
    w, x, y, z = quat
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def load_recording(path):
    with open(path, "rb") as f:
        data = pickle.load(f)
    lowstate = data["lowstate"]
    lowcmd = data.get("lowcmd", [])
    sportmodestate = data.get("sportmodestate", [])

    # t = 0 at the first command, so the controller phases line up between recordings
    t0 = lowcmd[0]["time"] if lowcmd else (lowstate[0]["time"] if lowstate else 0.0)

    rec = {
        "name": os.path.splitext(os.path.basename(path))[0],
        "meta": data.get("meta", {}),
        "t": np.array([s["time"] for s in lowstate]) - t0,
        "q": np.array([[m["actual_q"] for m in s["motor_states"]] for s in lowstate]).reshape(-1, 12),
        "rpy": np.array([quaternion_to_rpy(s["quaternion"]) for s in lowstate]).reshape(-1, 3),
        "foot_force": np.array([s["foot_force"] for s in lowstate]).reshape(-1, 4),
        "t_cmd": np.array([s["time"] for s in lowcmd]) - t0,
        "q_cmd": np.array([[m["command_q"] for m in s["motor_states"]] for s in lowcmd]).reshape(-1, 12),
        "kp_cmd": np.array([[m["command_kp"] for m in s["motor_states"]] for s in lowcmd]).reshape(-1, 12),
        "t_base": np.array([s["time"] for s in sportmodestate]) - t0,
        "base_pos": np.array([s["position"] for s in sportmodestate]).reshape(-1, 3),
    }
    # Commands of 0 kp (e.g. damping at the end) are not position targets - hide them in the plots
    rec["q_cmd"] = np.where(rec["kp_cmd"] > 0, rec["q_cmd"], np.nan)
    print(f"Loaded '{path}': {len(rec['t'])} lowstate, {len(rec['t_cmd'])} lowcmd, "
          f"{len(rec['t_base'])} base position samples.")
    return rec


def draw_phases(ax, rec):
    """Vertical lines at the end of each controller phase (if the controller saved them)."""
    phase_times = rec["meta"].get("phase_times")
    if not phase_times:
        return
    t_end = 0.0
    for duration in phase_times.values():
        t_end += duration
        ax.axvline(t_end, color="gray", linestyle=":", linewidth=0.8)


def save_figure(fig, file_name):
    os.makedirs(RECORDINGS_DIR, exist_ok=True)
    path = os.path.join(RECORDINGS_DIR, file_name)
    fig.savefig(path, dpi=150)
    print(f"Saved figure '{path}'")


def plot_base(recs):
    has_base = any(len(r["t_base"]) > 0 for r in recs)
    ncols = 2 if has_base else 1
    fig, axes = plt.subplots(3, ncols, figsize=(7 * ncols, 8), sharex=True, squeeze=False)
    fig.suptitle("Robot base orientation (from IMU quaternion)" + (" and position" if has_base else ""))
    for i, angle in enumerate(["Roll", "Pitch", "Yaw"]):
        ax = axes[i, 0]
        for k, rec in enumerate(recs):
            ax.plot(rec["t"], rec["rpy"][:, i] * rad2deg, color=f"C{k}", label=rec["name"])
        draw_phases(ax, recs[0])
        ax.set_ylabel(f"{angle} (deg)")
        ax.grid(True)
    axes[0, 0].legend(loc="upper right")
    axes[-1, 0].set_xlabel("Time (s)")

    if has_base:
        for i, axis in enumerate(["X - X(0)", "Y - Y(0)", "Z"]):
            ax = axes[i, 1]
            for k, rec in enumerate(recs):
                if len(rec["t_base"]) == 0:
                    continue
                pos = rec["base_pos"][:, i] - (rec["base_pos"][0, i] if i < 2 else 0.0)
                ax.plot(rec["t_base"], pos, color=f"C{k}", label=rec["name"])
            draw_phases(ax, recs[0])
            ax.set_ylabel(f"Base {axis} (m)")
            ax.grid(True)
        axes[0, 1].legend(loc="upper right")
        axes[-1, 1].set_xlabel("Time (s)")
    fig.tight_layout()
    save_figure(fig, "compare_base.png")


def plot_joints(recs):
    fig, axes = plt.subplots(4, 3, figsize=(14, 10), sharex=True)
    fig.suptitle("Joint angles: actual (solid) vs. commanded (dashed)")
    for leg_id, leg in enumerate(legs):
        for joint_id, joint in enumerate(joints):
            motor_id = 3 * leg_id + joint_id
            ax = axes[leg_id, joint_id]
            for k, rec in enumerate(recs):
                ax.plot(rec["t"], rec["q"][:, motor_id] * rad2deg, color=f"C{k}", label=f"{rec['name']} actual")
                if len(rec["t_cmd"]) > 0:
                    ax.plot(rec["t_cmd"], rec["q_cmd"][:, motor_id] * rad2deg, color=f"C{k}",
                            linestyle="--", alpha=0.6, label=f"{rec['name']} cmd")
            draw_phases(ax, recs[0])
            ax.set_title(f"{leg}_{joint} (motor {motor_id})", fontsize=9)
            ax.grid(True)
        axes[leg_id, 0].set_ylabel("Angle (deg)")
    for ax in axes[-1, :]:
        ax.set_xlabel("Time (s)")
    axes[0, 2].legend(loc="upper right", fontsize=7)
    fig.tight_layout()
    save_figure(fig, "compare_joints.png")


def plot_gains(recs):
    rec = next((r for r in recs if len(r["t_cmd"]) > 0), None)
    if rec is None:
        return
    fig, axes = plt.subplots(1, 4, figsize=(16, 4), sharey=True)
    fig.suptitle(f"Commanded proportional gains kp ({rec['name']})")
    for leg_id, leg in enumerate(legs):
        ax = axes[leg_id]
        for joint_id, joint in enumerate(joints):
            ax.plot(rec["t_cmd"], rec["kp_cmd"][:, 3 * leg_id + joint_id], label=joint)
        draw_phases(ax, rec)
        ax.set_title(f"{leg} leg")
        ax.set_xlabel("Time (s)")
        ax.grid(True)
        ax.legend(loc="upper right")
    axes[0].set_ylabel("kp (Nm/rad)")
    fig.tight_layout()
    save_figure(fig, "compare_kp.png")


def plot_foot_forces(recs):
    # The simulator does not publish foot forces (all zeros) - only plot recordings that have them
    recs = [r for r in recs if np.any(r["foot_force"] != 0)]
    if not recs:
        print("No foot force data in these recordings (not available in simulation).")
        return
    fig, axes = plt.subplots(2, 2, figsize=(10, 6), sharex=True, sharey=True)
    fig.suptitle("Foot force sensors")
    for foot_id, leg in enumerate(legs):  # foot_force order is [FR, FL, RR, RL]
        ax = axes.flat[foot_id]
        for rec in recs:
            ax.plot(rec["t"], rec["foot_force"][:, foot_id], label=rec["name"])
        draw_phases(ax, recs[0])
        ax.set_title(f"{leg} foot")
        ax.set_ylabel("Force sensor reading")
        ax.grid(True)
    for ax in axes[-1, :]:
        ax.set_xlabel("Time (s)")
    axes[0, 0].legend(loc="upper right")
    fig.tight_layout()
    save_figure(fig, "compare_foot_forces.png")


if __name__ == "__main__":
    files = sys.argv[1:] if len(sys.argv) > 1 else [f for f in DEFAULT_FILES if os.path.exists(f)]
    if not files:
        print("No recordings found. Run your controller first, or pass the .pkl files as arguments:")
        print("  python3 plot_recordings.py recordings/student_shake_hands_sim.pkl recordings/student_shake_hands_robot.pkl")
        sys.exit(1)

    recs = [load_recording(f) for f in files]
    plot_base(recs)
    plot_joints(recs)
    plot_gains(recs)
    plot_foot_forces(recs)
    plt.show()
