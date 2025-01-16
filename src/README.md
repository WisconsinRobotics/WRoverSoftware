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
