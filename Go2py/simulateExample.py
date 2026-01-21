import numpy as np
from Go2Py.sim.mujoco import Go2Sim
from Go2Py.robot.model import Go2Model
import time
from scipy.spatial.transform import Rotation

benben = Go2Model()

def standExample(simTime: float = 5.0):
    robot = Go2Sim()
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
    while time.time()-start_time < simTime:
        time_percent = (time.time()-start_time)/simTime
        # print(time_percent)
        state = robot.getJointStates()
        q = state['q']

        translation, quat = robot.getPose()
        Rb = Rotation.from_quat([quat[1], quat[2], quat[3], quat[0]]).as_matrix()
        T = np.hstack([Rb, translation.reshape(3,1)])
        T = np.vstack([T, np.array([0., 0., 0., 1.])])

        # q_des = benben.inverseKinematics(T, x)
        q_des = qstand 
        # q_des = time_percent * qstand + (1-time_percent)*q0 
        dq_des = np.zeros(12)
        kp = 60.0*np.ones(12)
        kv = 5.0*np.ones(12)
        tau_ff = np.zeros(12).reshape(12,1)
        robot.setCommands(q_des=q_des, dq_des=dq_des, kp=kp, kv=kv, tau_ff=tau_ff)
        robot.step()
        step_counter+=1

# state = robot.getJointStates()
# T_up = 
# benben.forwardKinematics()
# benben.inverseKinematics
# Tbase = np.array([[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]])
# Xlegs = np.array([0,0,0,0,0,0,0,0,0,0,0,0])
# qup = benben.inverseKinematics(Tbase,Xlegs)

if __name__ == "__main__":
    standExample(simTime=5) # simulate standing up for 5 seconds
    print("L1 = " + str(benben.l1))
    print("L2 = " + str(benben.l2))
    print("L3 = " + str(benben.l3))
    print("Success")