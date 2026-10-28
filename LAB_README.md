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