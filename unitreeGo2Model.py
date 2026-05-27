try:
    import pinocchio as pin
    HAS_PINOCCHIO = True
except ImportError:
    HAS_PINOCCHIO = False
import numpy as np
from functools import lru_cache
from typing import Mapping, Sequence


def rpy_to_quaternion(roll: float, pitch: float, yaw: float) -> np.ndarray:
    cy = np.cos(yaw * 0.5)
    sy = np.sin(yaw * 0.5)
    cp = np.cos(pitch * 0.5)
    sp = np.sin(pitch * 0.5)
    cr = np.cos(roll * 0.5)
    sr = np.sin(roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([x, y, z, w])


def rpy_to_rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    c_r, s_r = np.cos(roll), np.sin(roll)
    Rx = np.array([
        [1.0, 0.0, 0.0],
        [0.0, c_r, -s_r],
        [0.0, s_r, c_r]
    ])
    
    c_p, s_p = np.cos(pitch), np.sin(pitch)
    Ry = np.array([
        [c_p, 0.0, s_p],
        [0.0, 1.0, 0.0],
        [-s_p, 0.0, c_p]
    ])
    
    c_y, s_y = np.cos(yaw), np.sin(yaw)
    Rz = np.array([
        [c_y, -s_y, 0.0],
        [s_y, c_y, 0.0],
        [0.0, 0.0, 1.0]
    ])
    
    return Rz @ Ry @ Rx



DEFAULT_URDF_PATH = "/home/user_robodog/go2_lab/go2-robotics-lab/Go2py/Go2Py/assets/urdf/go2.urdf"
DEFAULT_FOOT_FRAMES = ("FR_foot", "FL_foot", "RR_foot", "RL_foot")
DEFAULT_STAND_UNITREE_Q = np.array([0.0, 0.67, -1.3] * 4, dtype=float)
UNITREE_Q_ORDER = (
    "FR_0",
    "FR_1",
    "FR_2",
    "FL_0",
    "FL_1",
    "FL_2",
    "RR_0",
    "RR_1",
    "RR_2",
    "RL_0",
    "RL_1",
    "RL_2",
)
UNITREE_TO_PIN = {
    "FR_0": 10,
    "FR_1": 11,
    "FR_2": 12,
    "FL_0": 7,
    "FL_1": 8,
    "FL_2": 9,
    "RR_0": 16,
    "RR_1": 17,
    "RR_2": 18,
    "RL_0": 13,
    "RL_1": 14,
    "RL_2": 15,
}

__all__ = [
    "Go2Model",
    "Go2ModelAnalytical",
    "forward_kinematics",
    "inverse_kinematics",
    "solve_foot_ik",
    "solve_feet_ik",
    "standing_configuration",
    "neutral_configuration",
    "full_configuration_from_unitree_joints",
    "unitree_joints_from_full_configuration",
]


class Go2Kinematics:
    def __init__(self, urdf_path: str = DEFAULT_URDF_PATH, frame_name: str = "FL_foot"):
        if not HAS_PINOCCHIO:
            raise ImportError(
                "Pinocchio is not installed on this device. "
                "Please use the analytical kinematics solver 'Go2ModelAnalytical' instead."
            )
        self.urdf_path = urdf_path
        self.frame_name = frame_name
        self.model = pin.buildModelFromUrdf(self.urdf_path, pin.JointModelFreeFlyer())
        self.data = self.model.createData()
        self.base_frame = "base_link"
        self.foot_frames = DEFAULT_FOOT_FRAMES

    def neutral_configuration(self):
        return pin.neutral(self.model)

    def standing_configuration(self):
        return self.unitree_q_to_pin_q(DEFAULT_STAND_UNITREE_Q)

    def make_configuration(self, unitree_q):
        return self.unitree_q_to_pin_q(unitree_q)

    def unitree_q_to_pin_q(self, unitree_q, q=None):
        if q is None:
            q = self.neutral_configuration()

        unitree_q = np.asarray(unitree_q, dtype=float).reshape(12)
        for unitree_name, pin_index in UNITREE_TO_PIN.items():
            unitree_index = UNITREE_Q_ORDER.index(unitree_name)
            q[pin_index] = unitree_q[unitree_index]

        return q

    def pin_q_to_unitree_q(self, q):
        q = np.asarray(q, dtype=float)
        unitree_q = np.zeros(12)

        for unitree_name, pin_index in UNITREE_TO_PIN.items():
            unitree_index = UNITREE_Q_ORDER.index(unitree_name)
            unitree_q[unitree_index] = q[pin_index]

        return unitree_q

    def normalize_configuration(
        self,
        configuration=None,
        base_position=None,
        base_rpy=None,
        default_to_standing: bool = True,
    ):
        if configuration is None:
            q = self.standing_configuration() if default_to_standing else self.neutral_configuration()
        else:
            configuration = np.asarray(configuration, dtype=float).reshape(-1)
            if configuration.size == 19:
                q = configuration.copy()
            elif configuration.size == 12:
                q = self.unitree_q_to_pin_q(configuration)
            else:
                raise ValueError(
                    "configuration must have 12 Unitree joint values or 19 Pinocchio configuration values"
                )

        if base_position is not None:
            q[0:3] = np.asarray(base_position, dtype=float).reshape(3)
        if base_rpy is not None:
            q[3:7] = rpy_to_quaternion(*base_rpy)

        return q

    def set_unitree_leg_positions(self, unitree_q, q=None):
        return self.unitree_q_to_pin_q(unitree_q, q=q)

    def to_unitree_leg_positions(self, q):
        return self.pin_q_to_unitree_q(q)

    def update_kinematics(self, q):
        pin.forwardKinematics(self.model, self.data, q)
        pin.updateFramePlacements(self.model, self.data)

    def foot_pose(self, frame_name, q=None):
        return self.frame_pose(frame_name, q=q)

    def foot_positions(self, q=None):
        return self.frame_positions(q=q, frame_names=self.foot_frames)

    def frame_pose(self, frame_name, q=None):
        if q is None:
            q = self.standing_configuration()

        self.update_kinematics(q)
        frame_id = self.model.getFrameId(frame_name)
        return self.data.oMf[frame_id]

    def frame_positions(self, q=None, frame_names=None):
        if q is None:
            q = self.standing_configuration()

        if frame_names is None:
            frame_names = self.foot_frames

        self.update_kinematics(q)
        return {
            frame_name: self.data.oMf[self.model.getFrameId(frame_name)].translation.copy()
            for frame_name in frame_names
        }

    def print_frame_pose(self, q=None):
        if q is None:
            q = self.standing_configuration()

        if self.model.existFrame(self.frame_name):
            foot_pose = self.frame_pose(self.frame_name, q)

            print(f"--- {self.frame_name} Pose ---")
            print("Position (x, y, z):", foot_pose.translation)
            print("Rotation Matrix:\n", foot_pose.rotation)
        else:
            print(f"Frame '{self.frame_name}' not found. Check your URDF frame names.")

    def solve_frame_position_ik(
        self,
        target_positions,
        q0=None,
        base_position=None,
        base_rpy=None,
        fixed_base: bool = True,
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
    ):
        q_ik = self.normalize_configuration(
            configuration=q0,
            base_position=base_position,
            base_rpy=base_rpy,
            default_to_standing=False,
        )

        target_items = list(target_positions.items())
        success = False
        iterations = 0

        for iterations in range(it_max):
            self.update_kinematics(q_ik)
            pin.computeJointJacobians(self.model, self.data, q_ik)

            error_blocks = []
            jacobian_blocks = []

            for frame_name, target_position in target_items:
                frame_id = self.model.getFrameId(frame_name)
                pose = self.data.oMf[frame_id]
                target_position = np.asarray(target_position, dtype=float).reshape(3)
                error_blocks.append(target_position - pose.translation)
                
                J_full = pin.getFrameJacobian(
                    self.model,
                    self.data,
                    frame_id,
                    pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                )[:3, :]
                
                if fixed_base:
                    jacobian_blocks.append(J_full[:, 6:])
                else:
                    jacobian_blocks.append(J_full)

            err = np.concatenate(error_blocks)

            if np.linalg.norm(err) < eps:
                success = True
                break

            J = np.vstack(jacobian_blocks)
            if fixed_base:
                delta_v_joints = np.linalg.solve(J.T @ J + damp * np.eye(J.shape[1]), J.T @ err)
                delta_v = np.zeros(18)
                delta_v[6:] = delta_v_joints
            else:
                delta_v = np.linalg.solve(J.T @ J + damp * np.eye(J.shape[1]), J.T @ err)
                
            q_ik = pin.integrate(self.model, q_ik, delta_v * dt)

        return success, iterations, q_ik

    def solve_whole_body_ik(
        self,
        q0=None,
        base_target=None,
        foot_targets=None,
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
    ):
        if q0 is None:
            q_ik = self.neutral_configuration()
        else:
            q_ik = q0.copy()

        if base_target is None:
            base_target = pin.SE3.Identity()

        if foot_targets is None:
            reference_q = self.standing_configuration()
            foot_targets = self.frame_positions(reference_q, self.foot_frames)

        tasks = [(self.base_frame, base_target, "pose")]
        tasks.extend((frame_name, target_position, "position") for frame_name, target_position in foot_targets.items())

        success = False
        iterations = 0

        for iterations in range(it_max):
            self.update_kinematics(q_ik)
            pin.computeJointJacobians(self.model, self.data, q_ik)

            error_blocks = []
            jacobian_blocks = []

            for frame_name, target_value, task_kind in tasks:
                frame_id = self.model.getFrameId(frame_name)
                pose = self.data.oMf[frame_id]

                if task_kind == "pose":
                    iMf = pose.actInv(target_value)
                    error_blocks.append(pin.log(iMf).vector)
                    jacobian_blocks.append(
                        pin.getFrameJacobian(self.model, self.data, frame_id, pin.ReferenceFrame.LOCAL)
                    )
                else:
                    target_position = np.asarray(target_value, dtype=float).reshape(3)
                    error_blocks.append(target_position - pose.translation)
                    jacobian_blocks.append(
                        pin.getFrameJacobian(
                            self.model,
                            self.data,
                            frame_id,
                            pin.ReferenceFrame.LOCAL_WORLD_ALIGNED,
                        )[:3, :]
                    )

            err = np.concatenate(error_blocks)

            if np.linalg.norm(err) < eps:
                success = True
                break

            J = np.vstack(jacobian_blocks)
            delta_v = np.linalg.solve(J.T @ J + damp * np.eye(J.shape[1]), J.T @ err)
            q_ik = pin.integrate(self.model, q_ik, delta_v * dt)

        return success, iterations, q_ik

    def solve_foot_ik(
        self,
        desired_position=None,
        q0=None,
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
    ):
        if desired_position is None:
            desired_position = np.array([0.2, 0.1, -0.35])

        return self.solve_frame_position_ik(
            {self.frame_name: desired_position},
            q0=q0,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )

    def run_demo(self):
        q_demo = self.standing_configuration()
        self.print_frame_pose(q_demo)
        print("All foot poses:")
        for frame_name, position in self.foot_positions(q_demo).items():
            print(frame_name, position)

        success, iterations, q_ik = self.solve_whole_body_ik(q0=self.neutral_configuration())

        if success:
            print(f"Whole-body IK converged in {iterations} iterations.")
            print("Actuated Leg Joint Angles (Unitree order):\n", self.pin_q_to_unitree_q(q_ik))
        else:
            print("Whole-body IK failed to converge within maximum iterations.")


class Go2Model:
    def __init__(self, urdf_path: str = DEFAULT_URDF_PATH, default_frame: str = "FL_foot"):
        self._kinematics = Go2Kinematics(urdf_path=urdf_path, frame_name=default_frame)

    def _normalize_configuration(self, configuration=None, base_position=None, base_rpy=None, default_to_standing: bool = True):
        return self._kinematics.normalize_configuration(
            configuration=configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            default_to_standing=default_to_standing,
        )

    def neutral_configuration(self):
        return self._kinematics.neutral_configuration()

    def standing_configuration(self):
        return self._kinematics.standing_configuration()

    def full_configuration_from_unitree_joints(
        self,
        unitree_joint_positions: Sequence[float],
        configuration=None,
        base_position=None,
        base_rpy=None,
    ):
        q = self._normalize_configuration(
            configuration=configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            default_to_standing=False,
        )
        return self._kinematics.unitree_q_to_pin_q(unitree_joint_positions, q=q)

    def unitree_joints_from_full_configuration(self, configuration: Sequence[float]):
        return self._kinematics.pin_q_to_unitree_q(configuration)

    def to_pin_configuration(self, unitree_q: Sequence[float], q=None, base_position=None, base_rpy=None):
        return self.full_configuration_from_unitree_joints(
            unitree_q,
            configuration=q,
            base_position=base_position,
            base_rpy=base_rpy,
        )

    def to_unitree_configuration(self, q: Sequence[float]):
        return self.unitree_joints_from_full_configuration(q)

    def forward_kinematics(self, configuration=None, base_position=None, base_rpy=None, frame_names=None):
        configuration = self._normalize_configuration(
            configuration=configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            default_to_standing=True,
        )

        if frame_names is None:
            frame_names = self._kinematics.foot_frames

        if isinstance(frame_names, str):
            return self._kinematics.frame_pose(frame_names, q=configuration).translation.copy()

        return self._kinematics.frame_positions(q=configuration, frame_names=frame_names)

    def solve_foot_ik(
        self,
        target_position: Sequence[float],
        initial_configuration=None,
        base_position=None,
        base_rpy=None,
        fixed_base: bool = True,
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
        frame_name: str = "FL_foot",
    ):
        target_position = np.asarray(target_position, dtype=float).reshape(3)
        return self._kinematics.solve_frame_position_ik(
            {frame_name: target_position},
            q0=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            fixed_base=fixed_base,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )

    def solve_feet_ik(
        self,
        foot_targets: Mapping[str, Sequence[float]],
        initial_configuration=None,
        base_position=None,
        base_rpy=None,
        fixed_base: bool = True,
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
    ):
        normalized_targets = {
            frame_name: np.asarray(target_position, dtype=float).reshape(3)
            for frame_name, target_position in foot_targets.items()
        }

        return self._kinematics.solve_frame_position_ik(
            normalized_targets,
            q0=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            fixed_base=fixed_base,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )

    def inverse_kinematics(
        self,
        target_positions,
        initial_configuration=None,
        base_position=None,
        base_rpy=None,
        fixed_base: bool = True,
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
        frame_name: str | None = None,
    ):
        if isinstance(target_positions, dict):
            return self.solve_feet_ik(
                target_positions,
                initial_configuration=initial_configuration,
                base_position=base_position,
                base_rpy=base_rpy,
                fixed_base=fixed_base,
                it_max=it_max,
                dt=dt,
                damp=damp,
                eps=eps,
            )

        return self.solve_foot_ik(
            target_positions,
            initial_configuration=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            fixed_base=fixed_base,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
            frame_name=frame_name or self._kinematics.frame_name,
        )

# Analytical Kinematics helper functions and classes

L_HIP = 0.0955
L_THIGH = 0.213
L_CALF = 0.213

HIP_OFFSETS = {
    "FL": np.array([0.1934, 0.0465, 0.0]),
    "FR": np.array([0.1934, -0.0465, 0.0]),
    "RL": np.array([-0.1934, 0.0465, 0.0]),
    "RR": np.array([-0.1934, -0.0465, 0.0]),
}

Y_SIGNS = {
    "FL": 1.0,
    "FR": -1.0,
    "RL": 1.0,
    "RR": -1.0,
}


def quaternion_to_rpy(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q[0], q[1], q[2], q[3]
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    if np.abs(sinp) >= 1:
        pitch = np.copysign(np.pi / 2, sinp)
    else:
        pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return np.array([roll, pitch, yaw])


def solve_leg_ik(leg_name: str, p_foot_base: np.ndarray) -> np.ndarray:
    p_hip = HIP_OFFSETS[leg_name]
    y_sign = Y_SIGNS[leg_name]
    
    p_h = p_foot_base - p_hip
    x_h, y_h, z_h = p_h[0], p_h[1], p_h[2]
    
    y_local = y_sign * L_HIP
    d_sq = y_h**2 + z_h**2
    if d_sq < y_local**2:
        raise ValueError("Target position is mathematically unreachable (YZ projection too close to hip axis).")
        
    z_local = -np.sqrt(d_sq - y_local**2)
    
    cos_q0 = (y_local * y_h + z_local * z_h) / d_sq
    sin_q0 = (y_local * z_h - z_local * y_h) / d_sq
    q0 = np.arctan2(sin_q0, cos_q0)
    
    x_local = x_h
    cos_q2 = (x_local**2 + z_local**2 - L_THIGH**2 - L_CALF**2) / (2.0 * L_THIGH * L_CALF)
    if not (-1.01 <= cos_q2 <= 1.01):
        raise ValueError("Target position is out of leg reach.")
    cos_q2 = np.clip(cos_q2, -1.0, 1.0)
    
    sin_q2 = -np.sqrt(1.0 - cos_q2**2)
    q2 = np.arctan2(sin_q2, cos_q2)
    
    A = -(L_THIGH + L_CALF * cos_q2)
    B = -L_CALF * sin_q2
    det = A**2 + B**2
    
    sin_q1 = (A * x_local - B * z_local) / det
    cos_q1 = (B * x_local + A * z_local) / det
    q1 = np.arctan2(sin_q1, cos_q1)
    
    return np.array([q0, q1, q2])


class Go2ModelAnalytical:
    def __init__(self):
        self.l_hip = L_HIP
        self.l_thigh = L_THIGH
        self.l_calf = L_CALF
        self.foot_frames = DEFAULT_FOOT_FRAMES
        self.hip_offsets = HIP_OFFSETS
        self.y_signs = Y_SIGNS

    def neutral_configuration(self):
        return np.zeros(12)

    def standing_configuration(self):
        return DEFAULT_STAND_UNITREE_Q.copy()

    def forward_kinematics_leg(self, leg_name: str, q: np.ndarray) -> np.ndarray:
        q0, q1, q2 = q[0], q[1], q[2]
        y_sign = self.y_signs[leg_name]
        p_hip = self.hip_offsets[leg_name]

        x_local = -self.l_thigh * np.sin(q1) - self.l_calf * np.sin(q1 + q2)
        y_local = y_sign * self.l_hip
        z_local = -self.l_thigh * np.cos(q1) - self.l_calf * np.cos(q1 + q2)

        x_hip = x_local
        y_hip = y_local * np.cos(q0) - z_local * np.sin(q0)
        z_hip = y_local * np.sin(q0) + z_local * np.cos(q0)

        return p_hip + np.array([x_hip, y_hip, z_hip])

    def full_configuration_from_unitree_joints(
        self,
        unitree_joint_positions: Sequence[float],
        configuration=None,
        base_position=None,
        base_rpy=None,
    ):
        q = np.zeros(19)
        if configuration is not None:
            configuration = np.asarray(configuration, dtype=float).reshape(-1)
            if configuration.size == 19:
                q = configuration.copy()
        if base_position is not None:
            q[0:3] = np.asarray(base_position, dtype=float).reshape(3)
        if base_rpy is not None:
            q[3:7] = rpy_to_quaternion(*base_rpy)
        q[7:19] = np.asarray(unitree_joint_positions, dtype=float).reshape(12)
        return q

    def unitree_joints_from_full_configuration(self, configuration: Sequence[float]):
        configuration = np.asarray(configuration, dtype=float).reshape(-1)
        if configuration.size == 19:
            return configuration[7:19].copy()
        elif configuration.size == 12:
            return configuration.copy()
        else:
            raise ValueError("Invalid configuration size")

    def to_pin_configuration(self, unitree_q: Sequence[float], q=None, base_position=None, base_rpy=None):
        return self.full_configuration_from_unitree_joints(
            unitree_q,
            configuration=q,
            base_position=base_position,
            base_rpy=base_rpy,
        )

    def to_unitree_configuration(self, q: Sequence[float]):
        return self.unitree_joints_from_full_configuration(q)

    def forward_kinematics(self, configuration=None, base_position=None, base_rpy=None, frame_names=None):
        if configuration is None:
            configuration = self.standing_configuration()
        else:
            configuration = np.asarray(configuration, dtype=float).reshape(-1)
            if configuration.size == 19:
                if base_position is None:
                    base_position = configuration[0:3]
                if base_rpy is None:
                    base_rpy = quaternion_to_rpy(configuration[3:7])
                configuration = configuration[7:19]
            elif configuration.size != 12:
                raise ValueError("configuration must have 12 Unitree joint values or 19 Pinocchio configuration values")

        if base_position is None:
            base_position = np.zeros(3)
        if base_rpy is None:
            base_rpy = np.zeros(3)

        R_base = rpy_to_rotation_matrix(*base_rpy)

        if frame_names is None:
            frame_names = self.foot_frames

        is_single_frame = isinstance(frame_names, str)
        if is_single_frame:
            frame_names = [frame_names]

        results = {}
        for name in frame_names:
            leg_name = name[:2]
            leg_idx_map = {"FR": 0, "FL": 3, "RR": 6, "RL": 9}
            idx = leg_idx_map[leg_name]
            q_leg = configuration[idx:idx+3]
            
            p_foot_base = self.forward_kinematics_leg(leg_name, q_leg)
            p_foot_world = base_position + R_base @ p_foot_base
            results[name] = p_foot_world

        if is_single_frame:
            return results[frame_names[0]]
        return results

    def solve_foot_ik(
        self,
        target_position: Sequence[float],
        initial_configuration=None,
        base_position=None,
        base_rpy=None,
        frame_name: str = "FL_foot",
    ):
        target_position = np.asarray(target_position, dtype=float).reshape(3)
        foot_targets = {frame_name: target_position}
        return self.solve_feet_ik(
            foot_targets,
            initial_configuration=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
        )

    def solve_feet_ik(
        self,
        foot_targets: Mapping[str, Sequence[float]],
        initial_configuration=None,
        base_position=None,
        base_rpy=None,
    ):
        if base_position is None:
            base_position = np.zeros(3)
        if base_rpy is None:
            base_rpy = np.zeros(3)

        R_base = rpy_to_rotation_matrix(*base_rpy)

        joints_out = self.standing_configuration()
        if initial_configuration is not None:
            initial_configuration = np.asarray(initial_configuration, dtype=float).reshape(-1)
            if initial_configuration.size == 19:
                joints_out = initial_configuration[7:19].copy()
            elif initial_configuration.size == 12:
                joints_out = initial_configuration.copy()

        success = True
        for foot_name, p_foot_world in foot_targets.items():
            leg_name = foot_name[:2]
            p_foot_base = R_base.T @ (p_foot_world - base_position)
            try:
                q_leg = solve_leg_ik(leg_name, p_foot_base)
                leg_idx_map = {"FR": 0, "FL": 3, "RR": 6, "RL": 9}
                idx = leg_idx_map[leg_name]
                joints_out[idx:idx+3] = q_leg
            except ValueError:
                success = False

        is_19 = (initial_configuration is not None and initial_configuration.size == 19)
        if is_19:
            q_sol = np.zeros(19)
            q_sol[0:3] = base_position
            q_sol[3:7] = rpy_to_quaternion(*base_rpy)
            q_sol[7:19] = joints_out
        else:
            q_sol = joints_out

        return success, 1, q_sol

    def inverse_kinematics(
        self,
        target_positions,
        initial_configuration=None,
        base_position=None,
        base_rpy=None,
        frame_name: str | None = None,
    ):
        if isinstance(target_positions, dict):
            return self.solve_feet_ik(
                target_positions,
                initial_configuration=initial_configuration,
                base_position=base_position,
                base_rpy=base_rpy,
            )

        return self.solve_foot_ik(
            target_positions,
            initial_configuration=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            frame_name=frame_name or "FL_foot",
        )


@lru_cache(maxsize=None)
def _get_default_model(urdf_path: str = DEFAULT_URDF_PATH, default_frame: str = "FL_foot"):
    if HAS_PINOCCHIO:
        return Go2Model(urdf_path=urdf_path, default_frame=default_frame)
    else:
        return Go2ModelAnalytical()


def neutral_configuration(urdf_path: str = DEFAULT_URDF_PATH):
    return _get_default_model(urdf_path).neutral_configuration()


def standing_configuration(urdf_path: str = DEFAULT_URDF_PATH):
    return _get_default_model(urdf_path).standing_configuration()


def full_configuration_from_unitree_joints(
    unitree_joint_positions,
    urdf_path: str = DEFAULT_URDF_PATH,
    configuration=None,
    base_position=None,
    base_rpy=None,
):
    return _get_default_model(urdf_path).full_configuration_from_unitree_joints(
        unitree_joint_positions,
        configuration=configuration,
        base_position=base_position,
        base_rpy=base_rpy,
    )


def unitree_joints_from_full_configuration(configuration, urdf_path: str = DEFAULT_URDF_PATH):
    return _get_default_model(urdf_path).unitree_joints_from_full_configuration(configuration)


def to_pin_configuration(unitree_q, urdf_path: str = DEFAULT_URDF_PATH, q=None, base_position=None, base_rpy=None):
    return full_configuration_from_unitree_joints(
        unitree_q,
        urdf_path=urdf_path,
        configuration=q,
        base_position=base_position,
        base_rpy=base_rpy,
    )


def to_unitree_configuration(q, urdf_path: str = DEFAULT_URDF_PATH):
    return unitree_joints_from_full_configuration(q, urdf_path=urdf_path)


def forward_kinematics(
    configuration=None,
    base_position=None,
    base_rpy=None,
    frame_names=None,
    urdf_path: str = DEFAULT_URDF_PATH,
):
    return _get_default_model(urdf_path).forward_kinematics(
        configuration=configuration,
        base_position=base_position,
        base_rpy=base_rpy,
        frame_names=frame_names,
    )


def solve_foot_ik(
    target_position,
    initial_configuration=None,
    base_position=None,
    base_rpy=None,
    fixed_base: bool = True,
    urdf_path: str = DEFAULT_URDF_PATH,
    it_max: int = 5000,
    dt: float = 0.1,
    damp: float = 1e-6,
    eps: float = 1e-4,
    frame_name: str = "FL_foot",
):
    model = _get_default_model(urdf_path)
    if isinstance(model, Go2Model):
        return model.solve_foot_ik(
            target_position,
            initial_configuration=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            fixed_base=fixed_base,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
            frame_name=frame_name,
        )
    else:
        return model.solve_foot_ik(
            target_position,
            initial_configuration=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            frame_name=frame_name,
        )


def solve_feet_ik(
    foot_targets,
    initial_configuration=None,
    base_position=None,
    base_rpy=None,
    fixed_base: bool = True,
    urdf_path: str = DEFAULT_URDF_PATH,
    it_max: int = 5000,
    dt: float = 0.1,
    damp: float = 1e-6,
    eps: float = 1e-4,
):
    model = _get_default_model(urdf_path)
    if isinstance(model, Go2Model):
        return model.solve_feet_ik(
            foot_targets,
            initial_configuration=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            fixed_base=fixed_base,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )
    else:
        return model.solve_feet_ik(
            foot_targets,
            initial_configuration=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
        )


def inverse_kinematics(
    target_positions,
    initial_configuration=None,
    base_position=None,
    base_rpy=None,
    fixed_base: bool = True,
    urdf_path: str = DEFAULT_URDF_PATH,
    it_max: int = 5000,
    dt: float = 0.1,
    damp: float = 1e-6,
    eps: float = 1e-4,
    frame_name: str | None = None,
):
    if isinstance(target_positions, dict):
        return solve_feet_ik(
            target_positions,
            initial_configuration=initial_configuration,
            base_position=base_position,
            base_rpy=base_rpy,
            fixed_base=fixed_base,
            urdf_path=urdf_path,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )

    return solve_foot_ik(
        target_positions,
        initial_configuration=initial_configuration,
        base_position=base_position,
        base_rpy=base_rpy,
        fixed_base=fixed_base,
        urdf_path=urdf_path,
        it_max=it_max,
        dt=dt,
        damp=damp,
        eps=eps,
        frame_name=frame_name or "FL_foot",
    )


def main():
    if HAS_PINOCCHIO:
        print("Pinocchio is available. Running Go2Kinematics demo:")
        Go2Kinematics().run_demo()
    else:
        print("Pinocchio is not available. Running Go2ModelAnalytical demo:")
        model = Go2ModelAnalytical()
        q_demo = model.standing_configuration()
        print("Standing joint angles (Unitree order):", q_demo)
        
        # Test forward kinematics
        feet = model.forward_kinematics(q_demo)
        print("\nForward Kinematics (feet positions):")
        for name, pos in feet.items():
            print(f"  {name}: {pos}")
            
        # Test inverse kinematics
        success, iterations, q_sol = model.solve_feet_ik(feet)
        print(f"\nInverse Kinematics success: {success}")
        print("Solved joint angles:", q_sol)


if __name__ == "__main__":
    main()