
## Installations Required

### Prerequisites
Ensure you have Python 3 and `pip` installed on your system.
You need to be running a linux machine.

### Required Libraries
Install the following Python libraries using `pip`:

```bash
pip install flask folium PyQt6 PyQt6-WebEngine
```

If you want to not mess up stuff, set up a virtual environment before running this.

## Publishing GPS Data

If you have a GPS node publishing to `/fix`, this should work as expected. However, if you want to test with fake values that don't rely on a GPS unit, use the following command in a terminal:

```bash
ros2 topic pub /fix sensor_msgs/NavSatFix "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'gps'}, status: {status: 0, service: 1}, latitude: 43.07, longitude: -89.41, altitude: 0.0, position_covariance: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], position_covariance_type: 0}"
```

## Running the GUI Code

Follow these steps to launch the GUI:

1. Run the following command inside the `WRoverSoftware/` directory:
   ```bash
   colcon build
   ```
2. Open a new terminal and source the setup file:
   ```bash
   source install/setup.bash
   ```
3. Now, launch the GUI:
   ```bash
   ros2 launch get_gps_data gps_gui_start_launch.py
   ```

Your GUI should now display and update with the GPS location.

>>>>>>> d8c6f4786f364224b94f6539cd283bf66a9d26e2


## GPS Setup instructions

Link to library used: https://github.com/gokulp01/ros2-ublox-zedf9p

1. Run the following command inside the `WRoverSoftware/` directory:
   ```bash
   colcon build
   ```
2. To run the driver:
   ```bash
   ros2 launch ublox_gps ublox_gps_node_zedf9p-launch.py
   ```
   Then values will be published to /fix topic

Refer to the readme of the library for error fixes and option customization (They have quite a few of those listed under Common Errors)


