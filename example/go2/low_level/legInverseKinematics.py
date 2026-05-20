import numpy as np

class go2leg:
    def __init__(self, legType="FR", L1=0.067, L2=0.213, L3=0.213, L4=0.0955):
        self.legType = legType 
        self.L1 = L1  # Hip height
        self.L2 = L2  # Thigh length
        self.L3 = L3  # Calf length
        self.L4 = L4  # Foot length

    def IK(self, xyz, initial_theta=None, max_iters=100, tol=1e-6, step=1e-6, damping=1e-4):
        """
        Numerically solve inverse kinematics with a damped Jacobian iteration.

        Input:
        - xyz: Desired foot position in the leg's local coordinate frame [m]
        - initial_theta: Optional starting guess [rad] for (hip, knee, ankle)
        - max_iters: Maximum solver iterations
        - tol: Convergence tolerance on position error [m]
        - step: Finite-difference step used to estimate the Jacobian
        - damping: Damping factor for the least-squares update

        Output:
        - theta: Joint angles [rad] in the same convention used by FK
        """
        target = np.asarray(xyz, dtype=float).reshape(3)

        if initial_theta is None:
            theta = np.zeros(3, dtype=float)
        else:
            theta = np.asarray(initial_theta, dtype=float).reshape(3).copy()

        def fk(theta_vec):
            return np.asarray(self.FK(theta_vec[0], theta_vec[1], theta_vec[2]), dtype=float)

        for _ in range(max_iters):
            current = fk(theta)
            error = target - current

            if np.linalg.norm(error) < tol:
                break

            jacobian = np.zeros((3, 3), dtype=float)
            for joint_index in range(3):
                perturbed = theta.copy()
                perturbed[joint_index] += step
                jacobian[:, joint_index] = (fk(perturbed) - current) / step

            lhs = jacobian.T @ jacobian + (damping ** 2) * np.eye(3)
            rhs = jacobian.T @ error

            try:
                delta_theta = np.linalg.solve(lhs, rhs)
            except np.linalg.LinAlgError:
                delta_theta, *_ = np.linalg.lstsq(jacobian, error, rcond=None)

            theta += delta_theta

            if np.linalg.norm(delta_theta) < tol:
                break

        # theta = self.leg_type_correction(self.legType, theta)
        return theta
    
    def FK(self, theta_1, theta_2, theta_3):
        """
        Input: 
        - theta_1: Hip angle [RAD]
        - theta_2: Knee angle [RAD]
        - theta_3: Ankle angle [RAD]
        Output: 
        - xyz: Foot position in the leg's local coordinate frame [m]
        """
        # theta_1, theta_2, theta_3 = self.leg_type_correction(self.legType, [theta_1, theta_2, theta_3])

        phi = theta_2 + theta_3
        x = -self.L2*np.sin(theta_1)*np.cos(theta_2) + self.L3*np.sin(theta_1)*np.sin(phi) + self.L4*np.cos(theta_1)
        y = self.L2*np.cos(theta_1)*np.cos(theta_2) - self.L3*np.sin(phi)*np.cos(theta_1) + self.L4*np.sin(theta_1)
        z = self.L1 + self.L2*np.sin(theta_2) + self.L3*np.cos(phi) 
        xyz = [x, y, z]  # position (x, y, z)
        return xyz
    
    @staticmethod
    def to_unitree(theta_1, theta_2, theta_3):
        """
        Convert analytical leg angles to Unitree's motor convention.
        Ref: https://observablehq.com/@christophe-yamahata/inverse-kinematics-go2-robot

        Mapping:
            q_hip   =  theta_1
            q_thigh = -theta_2
            q_calf  = -90 - theta_3
        """
        return np.array([theta_1, (-theta_2), ( -90.0 - theta_3 )])
    
    @staticmethod
    def leg_type_correction(legType, theta):
        """
        Apply leg type correction to the joint angles.
        If the leg is a left leg (FL or RL), we need to invert the hip angle.
        """
        if legType in ["FL", "RL"]:
            theta[0] = -theta[0]  # Invert hip angle for left legs
        return theta


if __name__ == "__main__":
    FRleg = go2leg(legType="FR")
    deg2rad = np.pi / 180.0

    # FR_theta_deg = np.array([18, 22.0, 37.0])  # Example joint angles (hip, knee, ankle)
    # FR_theta_rad = deg2rad * FR_theta_deg

    qFR_rad = np.array([0,0.67,-1.3])
    qFR_deg = qFR_rad / deg2rad
    FR_theta_deg = FRleg.to_unitree(*qFR_deg) # works both ways!
    FR_theta_rad = deg2rad * FR_theta_deg
    
    FRxyz = FRleg.FK(*FR_theta_rad)
    print("Input joint angles (hip, knee, ankle) in degrees:", FR_theta_deg)
    print("Unitree's motor angles (hip, thigh, calf) in degrees:", qFR_deg)
    print("Unitree's motor angles (hip, thigh, calf) in degrees:", go2leg.to_unitree(*FR_theta_deg))
    # print("Calculated foot position from FK:", FRxyz)

    # print("\nTesting IK with the FK result as the target...")
    # IK_theta = FRleg.IK(FRxyz, initial_theta=FR_theta_rad)
    # print("Recovered joint angles from IK:", IK_theta / deg2rad)
    # print(go2leg.to_unitree(0.0473455/deg2rad, 1.22187/deg2rad,-2.44375/deg2rad))

    # move in z axis backwards 100mm in the FR leg. 
    print(FRxyz)
    target_xyz = np.array(FRxyz) + np.array([0, -0.050, 0.100])
    IK_theta = FRleg.IK(target_xyz, initial_theta=FR_theta_rad)
    print("\nTarget foot position for IK:", target_xyz)
    print("Recovered joint angles from IK:", IK_theta / deg2rad)
    print(
        "Recovered joint angles from IK in degrees: "
        f"{np.array2string(IK_theta / deg2rad , precision=1, suppress_small=True)}"
    )
    print(
        "Unitree's motor angles from IK (hip, thigh, calf) in degrees: "
        f"{np.array2string(go2leg.to_unitree(*IK_theta / deg2rad) , precision=1, suppress_small=True)}"
    )

