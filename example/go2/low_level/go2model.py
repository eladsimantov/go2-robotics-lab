class Go2LegModel:
    """
    This class represents a low-level kinematic and dynamic model of a Go2 Leg.
    It includes methods for inverse and forward kinematics calculations.
    In addition, it is used to compute the mass matrix, Coriolis forces, and gravity effects on the leg.
    Based on the nice visualizations from: https://observablehq.com/@christophe-yamahata/inverse-kinematics-go2-robot
    Modified the angles to be compatible with the Unitree Go2 robot.
    """
    def __init__(self):
        self.q = [0.0] * 12  # Initialize leg joint angles (3 DOF)
        self.x = [0.0] * 12  # Initialize end-effector positions (4 legs, 3D each)
        self.L1 = 0.067  # Length from body base to Joint 2 (J2) in meters
        self.L2 = 0.213  # Length from Joint 2 (J2) to Joint 3 (J3) in meters
        self.L3 = 0.210  # Length from Joint 3 (J3) to paw in meters
        self.L4 = 0.094  # Off-axis distance from Joint 1 (J1) to paw in meters

    def inverse_kinematics(self,target_pos):
        pass

    def forward_kinematics(self,joint_angles):
        pass

    def compute_mass_matrix(self):
        pass


