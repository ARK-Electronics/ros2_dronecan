#!/usr/bin/env python3
import rclpy
import time
import dronecan
import dronecan.app.dynamic_node_id
import dronecan.app.node_monitor

from rclpy.node import Node
from px4_msgs.msg import DistanceSensor
from px4_msgs.msg import SensorOpticalFlow

# Dronecan message definitions
# https://dronecan.github.io/Specification/7._List_of_standard_data_types/

class DronecanListenerNode(Node):
    def __init__(self):
        super().__init__('dronecan_listener')

        self.flow_publisher = self.create_publisher(SensorOpticalFlow, '/dronecan/sensor_optical_flow', 10)
        self.range_publisher = self.create_publisher(DistanceSensor, '/dronecan/distance_sensor', 10)

        self.cannode = dronecan.make_node('can0', node_id=100, bitrate=1000000)
        self.cannode_monitor = dronecan.app.node_monitor.NodeMonitor(self.cannode)
        self.dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(self.cannode, self.cannode_monitor)

        self.cannode.add_handler(dronecan.com.hex.equipment.flow.Measurement, self.flow_callback)
        self.cannode.add_handler(dronecan.uavcan.equipment.range_sensor.Measurement, self.range_callback)

        # Timer callback for processing received messages -- 250Hz ... there is probably a better way to do this.
        self.timer = self.create_timer(0.004, self.timer_callback)

    def timer_callback(self):
        self.cannode.spin(0)

    def flow_callback(self, msg):
        # PX4 CAN Node Flow Publisher
        # https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/uavcannode/Publishers/FlowMeasurement.hpp
        m = msg.message
        flow = SensorOpticalFlow()

        flow.integration_timespan_us = int(m.integration_interval * 1e6);
        flow.delta_angle = [m.rate_gyro_integral[0], m.rate_gyro_integral[1], float('nan')]
        flow.pixel_flow = [m.flow_integral[0], m.flow_integral[1]]
        flow.quality = m.quality

        self.flow_publisher.publish(flow)


    def range_callback(self, msg):
        # PX4 CAN Node Range Publisher
        # https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/uavcannode/Publishers/RangeSensorMeasurement.hpp
        m = msg.message
        d = DistanceSensor()
        d.current_distance = m.range

        if m.sensor_type == m.SENSOR_TYPE_UNDEFINED:
            d.type = d.MAV_DISTANCE_SENSOR_INFRARED
        elif m.sensor_type == m.SENSOR_TYPE_SONAR:
            d.type = d.MAV_DISTANCE_SENSOR_ULTRASOUND
        elif m.sensor_type == m.SENSOR_TYPE_LIDAR:
            d.type = d.MAV_DISTANCE_SENSOR_LASER
        elif m.sensor_type == m.SENSOR_TYPE_RADAR:
            d.type = d.MAV_DISTANCE_SENSOR_RADAR

        if m.reading_type == m.READING_TYPE_UNDEFINED:
            d.signal_quality = 0
        elif m.reading_type == m.READING_TYPE_VALID_RANGE:
            d.signal_quality = 100
        elif m.reading_type == m.READING_TYPE_TOO_CLOSE:
            d.signal_quality = 0
        elif m.reading_type == m.READING_TYPE_TOO_FAR:
            d.signal_quality = 0

        d.h_fov = m.field_of_view

        self.range_publisher.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = DronecanListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


# Connected directly to FC

# TOPIC: sensor_optical_flow
#  sensor_optical_flow
#     timestamp: 191878100 (0.000580 seconds ago)
#     timestamp_sample: 191878100 (0 us before timestamp)
#     device_id: 8682243 (Type: 0x84, UAVCAN:0 (0x7B))
#     pixel_flow: [0.00000, 0.00000]
#     delta_angle: [-0.00001, -0.00008, nan]
#     distance_m: 0.00000
#     integration_timespan_us: 15872
#     error_count: 0
#     max_flow_rate: nan
#     min_ground_distance: nan
#     max_ground_distance: nan
#     delta_angle_available: True
#     distance_available: False
#     quality: 117
#     mode: 0



# TOPIC: distance_sensor
#  distance_sensor
#     timestamp: 294001790 (0.013518 seconds ago)
#     device_id: 9009923 (Type: 0x89, UAVCAN:0 (0x7B))
#     min_distance: 0.08000
#     max_distance: 30.00000
#     current_distance: 0.74316
#     variance: 0.00000
#     h_fov: 0.10473
#     v_fov: 0.10473
#     q: [0.00000, 0.00000, 0.00000, 0.00000] (Roll: 0.0 deg, Pitch: -0.0 deg, Yaw: 0.0 deg)
#     signal_quality: 100
#     type: 0
#     orientation: 2


# TODO
# When connected via ROS2 and Jetson CAN