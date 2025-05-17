# IK Build Instructions

To build IK, refer to [`/README-IK`](./README-IK).

---

# HOW TO START THE ROVER (with Arm and Swerve)

## Step 1: Power and Network

* Turn on the rover and the Wi-Fi.
* Connect to the `WRover_Basestation_5G` network.

---

## Step 2: Open Two Terminals (Basestation & Rover)

### On **Basestation**:

1. Open a terminal:

   ```bash
   git checkout dev/baseStation
   colcon build --symlink-install
   ```

2. Open a new terminal:

   ```bash
   source install/setup.bash
   ./baseStation.sh
   ```

3. Open another terminal:

   ```bash
   source install/setup.bash
   ros2 launch arm_ik demo.launch.py
   ```

---

### On **Rover**:

1. Open a terminal:

   ```bash
   ssh wiscrobo@192.169.1.134
   colcon build
   ```

2. Open a new terminal:

   ```bash
   ssh wiscrobo@192.169.1.134
   source install/setup.bash
   ./roverStart.sh
   ```

⚠️ **Watch out for the manipulator going crazy!**

---

## Final Reminder

✅ When starting the rover, make sure **all motors are zeroed**.
