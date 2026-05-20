import pinocchio as pin
import numpy as np
from functools import lru_cache
from typing import Mapping, Sequence


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
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
    ):
        if q0 is None:
            q_ik = self.neutral_configuration()
        else:
            q_ik = q0.copy()

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

    def _normalize_configuration(self, configuration=None, default_to_standing: bool = True):
        if configuration is None:
            return self.standing_configuration() if default_to_standing else self.neutral_configuration()

        configuration = np.asarray(configuration, dtype=float).reshape(-1)

        if configuration.size == 19:
            return configuration

        if configuration.size == 12:
            return self.to_pin_configuration(configuration)

        raise ValueError(
            "configuration must have 12 Unitree joint values or 19 Pinocchio configuration values"
        )

    def neutral_configuration(self):
        return self._kinematics.neutral_configuration()

    def standing_configuration(self):
        return self._kinematics.standing_configuration()

    def full_configuration_from_unitree_joints(
        self,
        unitree_joint_positions: Sequence[float],
        configuration=None,
    ):
        if configuration is None:
            configuration = self.neutral_configuration()

        return self._kinematics.unitree_q_to_pin_q(unitree_joint_positions, q=configuration)

    def unitree_joints_from_full_configuration(self, configuration: Sequence[float]):
        return self._kinematics.pin_q_to_unitree_q(configuration)

    def to_pin_configuration(self, unitree_q: Sequence[float], q=None):
        return self.full_configuration_from_unitree_joints(unitree_q, configuration=q)

    def to_unitree_configuration(self, q: Sequence[float]):
        return self.unitree_joints_from_full_configuration(q)

    def forward_kinematics(self, configuration=None, frame_names=None):
        configuration = self._normalize_configuration(configuration, default_to_standing=True)

        if frame_names is None:
            frame_names = self._kinematics.foot_frames

        if isinstance(frame_names, str):
            return self._kinematics.frame_pose(frame_names, q=configuration).translation.copy()

        return self._kinematics.frame_positions(q=configuration, frame_names=frame_names)

    def solve_foot_ik(
        self,
        target_position: Sequence[float],
        initial_configuration=None,
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
        frame_name: str = "FL_foot",
    ):
        initial_configuration = self._normalize_configuration(initial_configuration, default_to_standing=False)

        target_position = np.asarray(target_position, dtype=float).reshape(3)
        return self._kinematics.solve_frame_position_ik(
            {frame_name: target_position},
            q0=initial_configuration,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )

    def solve_feet_ik(
        self,
        foot_targets: Mapping[str, Sequence[float]],
        initial_configuration=None,
        it_max: int = 5000,
        dt: float = 0.1,
        damp: float = 1e-6,
        eps: float = 1e-4,
    ):
        initial_configuration = self._normalize_configuration(initial_configuration, default_to_standing=False)

        normalized_targets = {
            frame_name: np.asarray(target_position, dtype=float).reshape(3)
            for frame_name, target_position in foot_targets.items()
        }

        return self._kinematics.solve_frame_position_ik(
            normalized_targets,
            q0=initial_configuration,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )

    def inverse_kinematics(
        self,
        target_positions,
        initial_configuration=None,
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
                it_max=it_max,
                dt=dt,
                damp=damp,
                eps=eps,
            )

        return self.solve_foot_ik(
            target_positions,
            initial_configuration=initial_configuration,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
            frame_name=frame_name or self._kinematics.frame_name,
        )

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
        return self._kinematics.solve_whole_body_ik(
            q0=q0,
            base_target=base_target,
            foot_targets=foot_targets,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )


@lru_cache(maxsize=None)
def _get_default_model(urdf_path: str = DEFAULT_URDF_PATH, default_frame: str = "FL_foot"):
    return Go2Model(urdf_path=urdf_path, default_frame=default_frame)


def neutral_configuration(urdf_path: str = DEFAULT_URDF_PATH):
    return _get_default_model(urdf_path).neutral_configuration()


def standing_configuration(urdf_path: str = DEFAULT_URDF_PATH):
    return _get_default_model(urdf_path).standing_configuration()


def full_configuration_from_unitree_joints(unitree_joint_positions, urdf_path: str = DEFAULT_URDF_PATH, configuration=None):
    return _get_default_model(urdf_path).full_configuration_from_unitree_joints(
        unitree_joint_positions,
        configuration=configuration,
    )


def unitree_joints_from_full_configuration(configuration, urdf_path: str = DEFAULT_URDF_PATH):
    return _get_default_model(urdf_path).unitree_joints_from_full_configuration(configuration)


def to_pin_configuration(unitree_q, urdf_path: str = DEFAULT_URDF_PATH, q=None):
    return full_configuration_from_unitree_joints(unitree_q, urdf_path=urdf_path, configuration=q)


def to_unitree_configuration(q, urdf_path: str = DEFAULT_URDF_PATH):
    return unitree_joints_from_full_configuration(q, urdf_path=urdf_path)


def forward_kinematics(configuration=None, frame_names=None, urdf_path: str = DEFAULT_URDF_PATH):
    return _get_default_model(urdf_path).forward_kinematics(configuration=configuration, frame_names=frame_names)


def solve_foot_ik(
    target_position,
    initial_configuration=None,
    urdf_path: str = DEFAULT_URDF_PATH,
    it_max: int = 5000,
    dt: float = 0.1,
    damp: float = 1e-6,
    eps: float = 1e-4,
    frame_name: str = "FL_foot",
):
    return _get_default_model(urdf_path).solve_foot_ik(
        target_position,
        initial_configuration=initial_configuration,
        it_max=it_max,
        dt=dt,
        damp=damp,
        eps=eps,
        frame_name=frame_name,
    )


def solve_feet_ik(
    foot_targets,
    initial_configuration=None,
    urdf_path: str = DEFAULT_URDF_PATH,
    it_max: int = 5000,
    dt: float = 0.1,
    damp: float = 1e-6,
    eps: float = 1e-4,
):
    return _get_default_model(urdf_path).solve_feet_ik(
        foot_targets,
        initial_configuration=initial_configuration,
        it_max=it_max,
        dt=dt,
        damp=damp,
        eps=eps,
    )


def inverse_kinematics(
    target_positions,
    initial_configuration=None,
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
            urdf_path=urdf_path,
            it_max=it_max,
            dt=dt,
            damp=damp,
            eps=eps,
        )

    return solve_foot_ik(
        target_positions,
        initial_configuration=initial_configuration,
        urdf_path=urdf_path,
        it_max=it_max,
        dt=dt,
        damp=damp,
        eps=eps,
        frame_name=frame_name or _get_default_model(urdf_path)._kinematics.frame_name,
    )


def main():
    Go2Kinematics().run_demo()

    


if __name__ == "__main__":
    main()