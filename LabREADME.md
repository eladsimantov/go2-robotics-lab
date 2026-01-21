# Lab Instructions

## Steps to Record Joystick Commands
### Prerequisites
- Activate the virtual environment:
  ```bash
  source .venv/bin/activate
  ```
- Install required dependencies if not already done:
  ```bash
  pip install -r extendedRequirements.txt
  ```
### Recording and Plotting Joystick Commands
This guide explains how to record joystick commands using the provided `recordExample.py` script and plot the recorded data.
- Turn on the Go2 robot and ensure it is connected to the network via Ethernet cable.
- Turn on the joystick controller, and ensure it can communicate with the Go2 robot.
- Run the Go2 recording program:
  ```bash
  .venv/bin/python recordExample.py enp1s0
  ```
- Note that the `enp1s0` interface name may vary; use the appropriate network interface for your setup (ifconfig can help identify it).

- The script `recordExample.py` would save the recorded data to a file named `recorded_data_joystick.pkl`.
- To plot the recorded data, use the `plot_recorded_joints.py` script via 
the command:
  ```bash
  .venv/bin/python plot_recorded_joints.py
  ```
- This will generate plots of the recorded joint values over time with some low sampling rate specified in the record script.


# How to run simulation with recorded data
- Run the `simulateRecording.py` script to simulate the robot using the recorded data:
  ```bash
  .venv/bin/python /home/user_robodog/go2_lab/go2-robotics-lab/Go2py/simulateRecording.py
  ```
- This script will load the recorded data and simulate the robot's movements based on the recorded joint positions and velocities and feedforward torque estimates.
