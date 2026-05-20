


if __name__ == '__main__':
    import unitree_sdk2py
    print("imported unitree python sdk successfully")
    import numpy as np
    from unitreeGo2Model import forward_kinematics, inverse_kinematics, standing_configuration
    print("imported unitreeGo2Model successfully")

    q0 = standing_configuration()
    feet = forward_kinematics(q0)
    print(feet["FR_foot"])
    print(feet["FL_foot"])
    print(feet["RR_foot"])
    print(feet["RL_foot"])

    target_feet = {
        "FL_foot": np.array([0.18661782, 0.142, -0.33906387]),
        "FR_foot": np.array([0.18661782, -0.142, -0.33906387]),
        "RL_foot": np.array([-0.20, 0.10, -0.33906387]),
        "RR_foot": np.array([-0.20, -0.10, -0.33906387]),
    }

    success, iterations, q_sol = inverse_kinematics(
        target_feet,
        initial_configuration=q0,
    )
    print("IK success:", success)
    print("IK iterations:", iterations)
    print("IK solution (radians):", q_sol[7:19])
    print("IK solution (degrees):", np.degrees(q_sol[7:19]))
    print("Initial configuration (degrees):", np.degrees(q0[7:19]))