import time
import sys
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

if __name__ == "__main__":

    # Initialize simulation mode
    ChannelFactoryInitialize(1, "lo")  # '1' for simulation, 'lo' for loopback interface

    sport_client = SportClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    # Stand up the robot
    print("Standing up the robot...")
    sport_client.StandUp()
    time.sleep(2)  # Wait for the robot to stand up

    # Command the robot to move forward at 0.5 m/s
    print("Moving forward...")
    sport_client.Move(0.5, 0.0, 0.0)  # vx = 0.5 m/s, vy = 0.0 m/s, vtheta = 0.0 rad/s

    # Let the robot walk for 5 seconds
    time.sleep(5)

    # Stop the robot
    print("Stopping the robot...")
    sport_client.StopMove()
    time.sleep(1)

    # Stand down the robot
    print("Standing down the robot...")
    sport_client.StandDown()
    time.sleep(2)

    print("Simulation complete.")
