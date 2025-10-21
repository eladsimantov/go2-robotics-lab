# --- Standard Python Libraries ---
import time  # Used for time.sleep()
import sys   # Used to get command-line arguments (like the network interface)

# --- Core Unitree SDK Imports for Communication ---
# These are the fundamental building blocks for talking to the robot.
# They handle the underlying DDS (network) communication.
from unitree_sdk2py.core.channel import ChannelPublisher, ChannelFactoryInitialize
from unitree_sdk2py.core.channel import ChannelSubscriber, ChannelFactoryInitialize

# --- Imports for Low-Level Message Types ---
# These define the data structure for 'LowCmd' and 'LowState'.
# Think of them as the "blueprints" for the data packets we send and receive.
# The SDK has two slightly different paths for these, so both are imported.
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowCmd_
from unitree_sdk2py.idl.default import unitree_go_msg_dds__LowState_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowCmd_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import LowState_

# --- Utility Imports ---
from unitree_sdk2py.utils.crc import CRC  # CRC is a checksum to make sure data isn't corrupted
from unitree_sdk2py.utils.thread import RecurrentThread  # A helper to run a function at a fixed frequency (e.g., 500Hz)
import unitree_legged_const as go2  # Contains constants like motor IDs and stop flags

# --- Client Imports (The "Unlock" Mechanism) ---
# These are the high-level clients we use to "ask permission" to take low-level control.
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.go2.sport.sport_client import SportClient


# A class to hold all our robot's logic and state
class Custom:
    def __init__(self):
        # --- Control Parameters ---
        self.Kp = 60.0  # Default Proportional gain (stiffness) for the position controller
        self.Kd = 5.0   # Default Derivative gain (damping) for the position controller

        # --- State Machine & Timing Variables ---
        # These are used by the example's built-in motion sequence
        self.time_consume = 0  # Timer variable (not used in this version)
        self.rate_count = 0    # Loop counter (not used in this version)
        self.sin_count = 0     # Sine wave generator (not used in this version)
        self.motiontime = 0    # General-purpose timer for motion
        self.dt = 0.002        # Timestep (1 / 500Hz), used for interpolation

        # --- Command & State Objects ---
        # Pre-allocate the command message object we will send
        self.low_cmd = unitree_go_msg_dds__LowCmd_()
        # Initialize the state variable. This will be filled by the subscriber.
        self.low_state = None

        # --- Example Motion Sequence (Hardcoded Poses) ---
        # These are the target joint angles (in radians) for the 12 motors
        # [FR_hip, FR_thigh, FR_calf, FL_hip, FL_thigh, FL_calf, ...]
        # _targetPos_1: A lying-down pose
        self._targetPos_1 = [0.0, 1.36, -2.65, 0.0, 1.36, -2.65,
                             -0.2, 1.36, -2.65, 0.2, 1.36, -2.65]
        # _targetPos_2: A "crouched" standing pose
        self._targetPos_2 = [0.0, 0.67, -1.3, 0.0, 0.67, -1.3,
                             0.0, 0.67, -1.3, 0.0, 0.67, -1.3]
        # _targetPos_3: A different pose (used for the end of the sequence)
        self._targetPos_3 = [-0.35, 1.36, -2.65, 0.35, 1.36, -2.65,
                             -0.5, 1.36, -2.65, 0.5, 1.36, -2.65]

        # --- State Machine Interpolation Variables ---
        self.startPos = [0.0] * 12  # Will store the robot's starting position
        # Durations for each phase of the motion (in loop counts)
        self.duration_1 = 500  # 500 * 0.002s = 1.0 second
        self.duration_2 = 500  # 1.0 second
        self.duration_3 = 1000 # 2.0 seconds
        self.duration_4 = 900  # 1.8 seconds
        # Percentages for interpolating between poses (0.0 to 1.0)
        self.percent_1 = 0
        self.percent_2 = 0
        self.percent_3 = 0
        self.percent_4 = 0

        self.firstRun = True  # A flag to capture the startPos on the first loop
        self.done = False     # Flag for when the sequence is complete

        # --- Threading ---
        # This will hold the pointer to our 500Hz control loop thread
        self.lowCmdWriteThreadPtr = None

        # --- Utilities ---
        self.crc = CRC()  # Initialize the checksum calculator

    # Public methods
    def Init(self):
        # This function sets up everything.

        # 1. Initialize the low_cmd structure with safe, default values
        self.InitLowCmd()

        # 2. Set up the Publisher
        # This creates the "pipe" to send commands TO the robot
        # "rt/lowcmd" means "real-time low-level command"
        self.lowcmd_publisher = ChannelPublisher("rt/lowcmd", LowCmd_)
        self.lowcmd_publisher.Init()

        # 3. Set up the Subscriber
        # This creates the "pipe" to receive sensor data FROM the robot
        # "rt/lowstate" means "real-time low-level state"
        # When a message arrives, it will automatically call 'self.LowStateMessageHandler'
        self.lowstate_subscriber = ChannelSubscriber("rt/lowstate", LowState_)
        self.lowstate_subscriber.Init(self.LowStateMessageHandler, 10) # 10 is buffer size

        # --- THIS IS THE "UNLOCK" MECHANISM ---
        # 4. Initialize the HIGH-LEVEL clients
        # We need these to tell the robot's main brain to "let go"
        self.sc = SportClient()  # The client for high-level commands (like "StandDown")
        self.sc.SetTimeout(5.0)
        self.sc.Init()

        self.msc = MotionSwitcherClient()  # The client that controls *who* is in charge
        self.msc.SetTimeout(5.0)
        self.msc.Init()

        # 5. The "Unlock" Loop
        # This is the most critical part for low-level control.
        status, result = self.msc.CheckMode()  # Check what the robot is currently doing
        # Loop as long as the robot is in *any* high-level mode (e.g., 'name' is not empty)
        while result['name']:
            print(f"Robot is in high-level mode: {result['name']}. Releasing...")
            self.sc.StandDown()       # Command 1: Tell the robot to lie down (a safe state)
            self.msc.ReleaseMode()    # Command 2: Tell the brain to "release" control
            status, result = self.msc.CheckMode() # Check again
            time.sleep(1)             # Wait 1 second for the change to take effect
        # After this loop, the robot is "limp" and listening for our low-level commands.
        # --- END OF "UNLOCK" MECHANISM ---

    def Start(self):
        # This starts the 500Hz control thread.
        self.lowCmdWriteThreadPtr = RecurrentThread(
            interval=0.002,           # The interval (0.002s = 500Hz)
            target=self.LowCmdWrite,  # The function to call every 0.002s
            name="writebasiccmd"
        )
        self.lowCmdWriteThreadPtr.Start()

    # Private methods
    def InitLowCmd(self):
        # This function fills the 'low_cmd' message with safe, default values.
        # It's crucial to initialize all fields.
        self.low_cmd.head[0] = 0xFE  # Standard header bytes
        self.low_cmd.head[1] = 0xEF
        self.low_cmd.level_flag = 0xFF # High-level flag (standard)
        self.low_cmd.gpio = 0
        # The Go2 has 12 motors, but the command structure has 20 slots
        for i in range(20):
            self.low_cmd.motor_cmd[i].mode = 0x01  # 0x01 = (PMSM) mode (this is the default position/PD controller)
            self.low_cmd.motor_cmd[i].q = go2.PosStopF # A "stop" flag for position
            self.low_cmd.motor_cmd[i].kp = 0          # Kp = 0 (no stiffness)
            self.low_cmd.motor_cmd[i].dq = go2.VelStopF # A "stop" flag for velocity
            self.low_cmd.motor_cmd[i].kd = 0          # Kd = 0 (no damping)
            self.low_cmd.motor_cmd[i].tau = 0         # tau = 0 (no torque)
        # This initializes the robot to be completely "limp"

    def LowStateMessageHandler(self, msg: LowState_):
        # This function is a "callback". It runs automatically
        # every time a new 'rt/lowstate' message arrives.
        self.low_state = msg  # Store the latest sensor data in our class variable
        # You can uncomment these lines to debug and see sensor data
        # print("FR_0 motor state: ", msg.motor_state[go2.LegID["FR_0"]])
        # print("IMU state: ", msg.imu_state)
        # print("Battery state: voltage: ", msg.power_v, "current: ", msg.power_a)

    def LowCmdWrite(self):
        # --- THIS IS YOUR MAIN CONTROL LOOP (runs at 500Hz) ---
        # This is where you will put your CPG or Impedance Control logic.

        # Check if we've received our first sensor message
        if self.low_state is None:
            return # Skip this loop if we have no sensor data yet

        # --- This is the example's built-in state machine ---
        # It's a 4-part motion sequence using linear interpolation (lerp).

        # On the very first run, capture the robot's current joint angles
        if self.firstRun:
            for i in range(12):
                self.startPos[i] = self.low_state.motor_state[i].q
            self.firstRun = False

        # --- Phase 1: Move from startPos to _targetPos_1 ---
        self.percent_1 += 1.0 / self.duration_1   # Increment the percentage
        self.percent_1 = min(self.percent_1, 1) # Clamp it at 1.0
        if self.percent_1 < 1:
            for i in range(12):
                # This is the linear interpolation: q = (1-p)*start + p*end
                self.low_cmd.motor_cmd[i].q = (1 - self.percent_1) * self.startPos[i] + self.percent_1 * self._targetPos_1[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp # Use default Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd # Use default Kd
                self.low_cmd.motor_cmd[i].tau = 0

        # --- Phase 2: Move from _targetPos_1 to _targetPos_2 (crouch) ---
        # This only runs after Phase 1 is 100% done
        if (self.percent_1 == 1) and (self.percent_2 <= 1):
            self.percent_2 += 1.0 / self.duration_2
            self.percent_2 = min(self.percent_2, 1)
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = (1 - self.percent_2) * self._targetPos_1[i] + self.percent_2 * self._targetPos_2[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        # --- Phase 3: Hold _targetPos_2 (crouch) ---
        # This only runs after Phase 2 is 100% done
        if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 < 1):
            self.percent_3 += 1.0 / self.duration_3
            self.percent_3 = min(self.percent_3, 1)
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = self._targetPos_2[i] # Just hold the position
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0

        # --- Phase 4: Move from _targetPos_2 to _targetPos_3 ---
        # This only runs after Phase 3 is 100% done
        if (self.percent_1 == 1) and (self.percent_2 == 1) and (self.percent_3 == 1) and (self.percent_4 <= 1):
            self.percent_4 += 1.0 / self.duration_4
            self.percent_4 = min(self.percent_4, 1)
            for i in range(12):
                self.low_cmd.motor_cmd[i].q = (1 - self.percent_4) * self._targetPos_2[i] + self.percent_4 * self._targetPos_3[i]
                self.low_cmd.motor_cmd[i].dq = 0
                self.low_cmd.motor_cmd[i].kp = self.Kp
                self.low_cmd.motor_cmd[i].kd = self.Kd
                self.low_cmd.motor_cmd[i].tau = 0
        # --- End of example state machine ---

        # --- Final Steps before Sending ---
        # 1. Calculate the checksum for the command packet
        self.low_cmd.crc = self.crc.Crc(self.low_cmd)
        # 2. Publish the command to the robot
        self.lowcmd_publisher.Write(self.low_cmd)


# --- This is the main part of the script that runs first ---
if __name__ == '__main__':

    # Print a critical safety warning.
    print("WARNING: Please ensure there are no obstacles around the robot while running this example.")
    print("WARNING: The robot WILL GO LIMP and then move its legs.")
    input("Press Enter to continue...") # Wait for user confirmation

    # --- Initialize the Network ---
    # This sets up the underlying DDS communication
    if len(sys.argv) > 1:
        # If you provided a network interface (e.g., 'enp3s0'), use it
        ChannelFactoryInitialize(0, sys.argv[1])
    else:
        # Otherwise, use the default
        ChannelFactoryInitialize(0)

    # --- Create and Run the Controller ---
    custom = Custom()  # Create an instance of our class
    custom.Init()      # Run the initialization (this is where the "unlock" happens)
    custom.Start()     # Start the 500Hz control thread

    # --- Keep the main thread alive ---
    # The 'LowCmdWrite' function is running in a separate thread.
    # This main thread just waits until the motion is done.
    while True:
        if custom.percent_4 == 1.0: # Check if the motion sequence is finished
            time.sleep(1)
            print("Done!")
            sys.exit(0) # Exit the script
        time.sleep(1) # Wait and check again
