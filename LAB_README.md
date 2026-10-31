# Simulation Environment
Run in a new terminal (CNTL+ALT+T) the following commands to start the simulation environment:

```bash
cd ~
source /home/user_robodog/go2_lab/unitree_mujoco/go2sim/bin/activate
cd /home/user_robodog/go2_lab/unitree_mujoco/simulate_python
python3 unitree_mujoco.py
```

# Simulate Your Controller
Run in a second terminal (CNTL+ALT+T) the commands with your example python file.

```bash
cd ~
source /home/user_robodog/go2_lab/unitree_mujoco/go2sim/bin/activate
cd /home/user_robodog/go2_lab/go2-robotics-lab/example/go2/low_level
python3 student_shake_hands.py
```

The run is recorded automatically to `recordings/student_shake_hands_sim.pkl`.


# Deploy to real time
MUST CHECK SIMULATION BEFORE!
MUST APPROVE BY INSTRUCTOR!

Get the interface via:
```bash
ifconfig
```

Run your controller with the interface identifier ('enp0s31f6')
```bash
cd ~
source /home/user_robodog/go2_lab/go2-robotics-lab/.venv/bin/activate
cd /home/user_robodog/go2_lab/go2-robotics-lab/example/go2/low_level
python3 student_shake_hands.py enp0s31f6
```

The run is recorded automatically to `recordings/student_shake_hands_robot.pkl`.


# Compare Simulation and Real Robot
Plot the simulation and the real robot recordings on the same figures
(base roll/pitch/yaw, base position (simulation only), joint angles, kp gains and foot forces (real robot only)):

```bash
cd ~
source /home/user_robodog/go2_lab/go2-robotics-lab/.venv/bin/activate
cd /home/user_robodog/go2_lab/go2-robotics-lab/example/go2/low_level
python3 plot_recordings.py
```

The figures are also saved as `.png` files in the `recordings` folder for your report.
Each new run overwrites the previous recording with the same name, so rename files you want to keep.
You can also plot specific files: `python3 plot_recordings.py recordings/a.pkl recordings/b.pkl`


# Record Without a Controller (optional)
To record the robot while using the joystick (e.g. the built-in "Shake Hands" (R2 + B)), run and stop with Ctrl+C:
```bash
python3 lab_recorder.py enp0s31f6 unitree_shake_hands
python3 plot_recordings.py recordings/unitree_shake_hands.pkl recordings/student_shake_hands_robot.pkl
```
