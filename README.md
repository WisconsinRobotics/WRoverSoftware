# Wisconsin Robotics -- 2024-2025 Software System
Codebase for the URC rover at UW-Madison, 2024-2025.


## Teleop
Modules and logic used during teleoperated control:

### ▸ Arm-forward-Kinematics
Refer to README inside branch dev/arm_test

### ▸ Arm-IK
Refer to README inside branch dev/ik_drawing

### ▸ Drive
Refer to README inside branch dev/basestation

### ▸ Full Teleop (Requires 3 Terminals)

You must run three processes in parallel. Open **three separate terminals** and paste the corresponding commands below.

---

### ✅ Terminal 1 — Base Station

```bash
cd /home/wiscrobo/workspace/WRoverSoftware
git checkout dev/basestation
colcon build
./basestation.sh
```

---

### ✅ Terminal 2 — IK Solver

```bash
cd /home/wiscrobo/workspace/IK_Solver/WRoverSoftware
cd src/relaxed_ik_ros2/relaxed_ik_core
cargo build
cd ../../..
source /opt/ros/humble/setup.bash
colcon build
. install/setup.bash
ros2 launch arm_ik demo.launch.py
```

---

### ✅ Terminal 3 — Rover Station (SSH)

Make sure you are connected to `WRoverBasestation_5G` **before running this**.

```bash
ssh wiscrobo@192.168.1.134
```

When prompted, enter the password:

```
i#3Er0b0
```

Then run:

```bash
cd /home/wiscrobo/workspace/WRoverSoftware
git checkout dev/roverStation
colcon build
# If build fails, run:
# rm -r build install log && colcon build
./roverStart.sh
```

Enter the **same password** when prompted.

---

Let me know if you want a title at the top, numbered steps, or environment notes!



---

## Autonomous
Refer to README inside dev/StateMachine
