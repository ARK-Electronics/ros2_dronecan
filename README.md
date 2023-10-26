This package provides an interface between the Dronecan protocol on CAN and ROS2.

This example uses the **px4_msgs** package for publishing data.

## Create a ROS2 workspace
```
mkdir dronecan_workspace
cd dronecan_workspace
mkdir src && cd src
git clone https://github.com/ARK-Electronics/ros2_dronecan.git
git clone https://github.com/PX4/px4_msgs
cd ..
```

```
colcon build --symlink install
source install/setup.bash
ros2 run ros2_dronecan ark_flow
```