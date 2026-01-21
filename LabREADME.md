# Lab Instructions

## Steps to Record Joystick Commands
This guide explains how to record joystick commands using the provided `recordExample.py` script and plot the recorded data.
- Turn on the Go2 robot and ensure it is connected to the network via Ethernet cable.
- Turn on the joystick controller, and ensure it can communicate with the Go2 robot.
- Activate the virtual environment:
  ```bash
  source .venv/bin/activate
  ```
- Install required dependencies if not already done:
  ```bash
  pip install -r extendedRequirements.txt
  ```
- Run the Go2 robot control program:
  ```bash
  .venv/bin/python go2_control.py enp1s0
  ```
- Note that the `enp1s0` interface name may vary; use the appropriate network interface for your setup (ifconfig can help identify it).

- The script `recordExample.py` would save the recorded data to a file named `recorded_data_joystick.pkl`.
- To plot the recorded data, use the `plot_recorded_joints.py` script via 
the command:
  ```bash
  .venv/bin/python plot_recorded_joints.py
  ```
- This will generate plots of the recorded joint values over time with some low sampling rate specified in the record script.