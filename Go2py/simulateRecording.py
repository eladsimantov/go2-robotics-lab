import pickle
import numpy as np
from Go2Py.sim.mujoco import Go2Sim
from Go2Py.robot.model import Go2Model
import time
from scipy.spatial.transform import Rotation

# ------------ Load recorded data -------------- #
import os

# Try to find the recorded data file
possible_paths = [
    "/home/user_robodog/go2_lab/go2-robotics-lab/recordings/ShankHands_recorded_data_joystick.pkl",
    "/home/user_robodog/go2_lab/go2-robotics-lab/recorded_data_joystick.pkl",
    "../recorded_data_joystick.pkl",
    "recorded_data_joystick.pkl"
]

data_file = None
for path in possible_paths:
    if os.path.exists(path):
        data_file = path
        print(f"Found recorded data at: {data_file}")
        break

if data_file is None:
    print("ERROR: Could not find recorded data file!")
    print("Tried paths:")
    for path in possible_paths:
        print(f"  - {path}")
    exit(1)

with open(data_file, "rb") as f:
    data = pickle.load(f)
lowstate = data["lowstate"]

# Extract time and joint positions
times = [sample["time"] for sample in lowstate]
num_motors = 12

# Extract joint positions, velocities and torques for each motor
joint_positions = []
joint_velocities = []
joint_taus = []
for motor_id in range(num_motors):
    q_motor = [sample["motor_states"][motor_id]["actual_q"] for sample in lowstate]
    dq_motor = [sample["motor_states"][motor_id]["actual_dq"] for sample in lowstate]
    tau_motor = [sample["motor_states"][motor_id]["actual_tau_est"] for sample in lowstate]
    joint_positions.append(q_motor)
    joint_velocities.append(dq_motor)
    joint_taus.append(tau_motor)
    
# Convert to numpy arrays for easier indexing
joint_positions = np.array(joint_positions)
joint_velocities = np.array(joint_velocities)
joint_taus = np.array(joint_taus)

# # Print summary of loaded data debug info
# print(f"Loaded {len(lowstate)} samples from recorded data.")
# print("First sample motor states:", lowstate[0]["motor_states"])
# print(lowstate)
# print("Times:", times)
# print(f"Joint Positions: {[[f'{q:.3g}' for q in positions] for positions in joint_positions]}")
# print(f"Joint Velocities: {[[f'{dq:.3g}' for dq in velocities] for velocities in joint_velocities]}")
# print(f"Joint Tau Estimates: {[[f'{tau:.3g}' for tau in taus] for taus in joint_taus]}")


# ----------------- Simulate using the recorded data ---------------- #

benben = Go2Model()

def standExample(simTime: float = 5.0):
    robot = Go2Sim()
    qinit = np.array(joint_positions)[:,0]

    qstand = np.array([0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                        0.0, 0.67, -1.3, 0.0, 0.67, -1.3])
    qcrouch = np.array([0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                        -0.2, 1.36, -2.65, 0.2, 1.36, -2.65])
    robot.standUpReset()
    start_time = time.time()
    step_counter = 0
    state = robot.getJointStates()
    q0 = state['q']
    dq0 = state['dq']
    
    # Get actual recorded duration
    recorded_duration = times[-1] if times else simTime
    print(f"Recorded duration: {recorded_duration:.2f} seconds")
    print(f"Number of samples: {len(times)}")
    
    # Target framerate for visualization (Hz)
    target_fps = 100
    frame_time = 1.0 / target_fps
    last_frame_time = time.time()
    
    while time.time()-start_time < simTime:
        time_percent = (time.time()-start_time)/simTime
        state = robot.getJointStates()
        q = state['q']

        translation, quat = robot.getPose()
        Rb = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        T = np.hstack([Rb, translation.reshape(3,1)])
        T = np.vstack([T, np.array([0., 0., 0., 1.])])

        # q_des = benben.inverseKinematics(T, x)
        # q_des = qstand 
        # q_des = time_percent * qstand + (1-time_percent)*q0 
        # dq_des = np.zeros(12)

        q_des = joint_positions[:, min(step_counter, len(joint_positions[0])-1)]
        dq_des = joint_velocities[:, min(step_counter, len(joint_velocities[0])-1)]
        tau_ff = joint_taus[:, min(step_counter, len(joint_taus[0])-1)]

        kp = 60.0*np.ones(12)
        kv = 5.0*np.ones(12)
        # tau_ff = np.zeros(12).reshape(12,1)
        robot.setCommands(q_des=q_des, dq_des=dq_des, kp=kp, kv=kv, tau_ff=tau_ff)
        robot.step()
        
        # Maintain target framerate
        elapsed = time.time() - last_frame_time
        if elapsed < frame_time:
            time.sleep(frame_time - elapsed)
        last_frame_time = time.time()
        
        step_counter+=1

# state = robot.getJointStates()
# T_up = 
# benben.forwardKinematics()
# benben.inverseKinematics
# Tbase = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
# Xlegs = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
# qup = benben.inverseKinematics(Tbase,Xlegs)

if __name__ == "__main__":
    # Use the actual recorded duration instead of fixed 5 seconds
    recorded_duration = times[-1] if times else 5.0
    standExample(simTime=recorded_duration)
    print("L1 = " + str(benben.l1))
    print("L2 = " + str(benben.l2))
    print("L3 = " + str(benben.l3))
    print("Success")