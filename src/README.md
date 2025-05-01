src directory that holds all project packages

## Building
Running this code works best on Linux, as colcon has problems even in a docker container on Windows. To build:
```bash
colcon build
source install/setup.bash
```
Then run ```ros2 run {package name} {entrypoint name}```

### CAN Messages
CAN messages are used to communicate with (most of) the motors. Do this to ensure the CAN interface is active:
```bash
modprobe can
sudo ip link set can0 type can bitrate 500000
sudo ip link set can0 up
```

You can check if can0 is up with ```ip link show type can```

### Swerve
As of 01/15/24, the swerve motor can be controller via the xbox controller through these steps:

1. Ensure you have the wr_xbox_controller, wr_swerve_motor, and wr_can_comms packages
2. Ensure the CAN interface is active (see CAN Messages). 
3. Ensure the xbox controller is connected (bluetooth or cable)
4. Enable all three packages in separate terminals:
```bash
ros2 run wr_can_comms can_comms
ros2 run wr_swerve_motor swerve_motor
ros2 run wr_xbox_controller xbox_controller
```
5. Use the left joystick to move the motor

### Running the arm of the Robot

To execute the movements on the actual robot, follow these steps:

1. **Power On:** Turn on both switches of the robot.
2. **Connect to the Robot:** SSH into the robot.
3. **Navigate to the Correct Branch:** Open the `dev/arm_test` branch.
4. **Build and Start:** Run the following commands:
   
   ```bash
   colcon build
   ./cannableStart.sh
   ```

5. **Launch the Robot Code:**
   - Open a new terminal and run:
     
     ```bash
     source install/setup.bash
     ```
   
   - Navigate to `src/launch`
   - Execute:
     
     ```bash
     ros2 launch arm_ik_launch.py
     ```

Now, the robot should start mimicking the RViz model. **Be careful, as the robot's movements can be fast.**

## Important Note
For the robot to match the IK solver correctly, it needs to be **initialized properly** with the arm at a **90-degree position**.

To control the arm with forward kinematics, you can:
   - In the basestation, open a new terminal and run:
     
     ```bash
     source install/setup.bash
     ros2 launch wr_xbox_controller rm_xbox
     ```


   - In the rover, open a new terminal and run:
     
     ```bash
     source install/setup.bash
     ```
   
   - Navigate to `src/launch`
   - Execute:
     
     ```bash
     ros2 launch arm_falcon_launch.py
     ```
