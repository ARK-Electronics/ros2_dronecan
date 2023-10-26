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
Build and source
```
colcon build --symlink install
source install/setup.bash
ros2 run ros2_dronecan ark_flow
```
You can check that the topics are publishing
```
ros2 topichz /dronecan/distance_sensor
```
> average rate: 49.646
> 	min: 0.010s max: 0.030s std dev: 0.00261s window: 51

```
ros2 topic hz /dronecan/sensor_optical_flow
```
> average rate: 60.975
> 	min: 0.009s max: 0.032s std dev: 0.00309s window: 62

