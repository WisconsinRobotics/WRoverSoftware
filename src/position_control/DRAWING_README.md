# Running the Demo for Drawing with the Arm

## Setup
First, follow the instructions for setting up the `relaxed_ik_ros2` code: [RELAZED_IK.md](../../README.md)

## Running the IK Solver

To run the IK solver, open a new terminal and execute:

```bash
source install/setup.py
ros2 launch arm_ik demo.launch.py 
```

This will open two windows:
1. **RViz Window** – Displays the arm model.
2. **Drawing Platform** – After drawing, press the "Send" button, and the simulated arm should move accordingly.

## Running on the Actual Robot

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
