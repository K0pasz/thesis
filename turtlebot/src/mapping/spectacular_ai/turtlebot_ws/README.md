# Turtlebot4 mapping with Spectacular AI

## Dependencies
ROS2 Humble framework:

* https://docs.ros.org/en/dashing/Installation/Ubuntu-Install-Binary.html

On the Turtlebot 4 install the Spectacular AI Python package from its wheel. It is available on our Google Drive folder. Select the wheel that supports the Python version of the robot.
```
pip install /PATH/TO/*.whl
```

## Build

Make sure to have your ROS environment sourced i.e. `ros2` on command line works and that you have OAK-D device connected

From this folder, run the following commands on the robot:
```
colcon build
source install/setup.bash
```

## Usage

Run this on the robot:
```
ros2 launch src/spectacular_ai_depthai_turtlebot/launch/mapping_turtlebot.py
```
This way the mapping will start automatically.

Now you can start the nodes on the laptop from `laptop_ws` (for usage see its `README.md`).

## Published topics by the node

- `/slam/odometry`
- `/slam/keyframe`
- `/slam/left`
- `/tf`
- `/slam/pointcloud`
- `/slam/camera_info`
