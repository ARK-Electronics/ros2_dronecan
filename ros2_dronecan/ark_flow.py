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

        self.flow_publisher = self.create_publisher(SensorOpticalFlow, '/fmu/in/sensor_optical_flow', 10)
        self.range_publisher = self.create_publisher(DistanceSensor, '/fmu/in/distance_sensor', 10)

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

        flow.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        flow.timestamp_sample = flow.timestamp

        flow.integration_timespan_us = int(m.integration_interval * 1e6)
        flow.delta_angle = [m.rate_gyro_integral[0], m.rate_gyro_integral[1], float('nan')]
        flow.pixel_flow = [m.flow_integral[0], m.flow_integral[1]]
        flow.quality = m.quality
        flow.max_flow_rate = float('nan')
        flow.min_ground_distance = float('nan')
        flow.max_ground_distance = float('nan')

        self.flow_publisher.publish(flow)


    def range_callback(self, msg):
        # PX4 CAN Node Range Publisher
        # https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/uavcannode/Publishers/RangeSensorMeasurement.hpp
        m = msg.message
        d = DistanceSensor()
        d.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        d.current_distance = m.range

        if m.sensor_type == m.SENSOR_TYPE_UNDEFINED:
            d.type = d.MAV_DISTANCE_SENSOR_INFRARED
        elif m.sensor_type == m.SENSOR_TYPE_SONAR:
            d.type = d.MAV_DISTANCE_SENSOR_ULTRASOUND
        elif m.sensor_type == m.SENSOR_TYPE_LIDAR:
            d.type = d.MAV_DISTANCE_SENSOR_LASER
        elif m.sensor_type == m.SENSOR_TYPE_RADAR:
            d.type = d.MAV_DISTANCE_SENSOR_RADAR

        d.signal_quality = -1

        if m.reading_type == m.READING_TYPE_VALID_RANGE:
            d.signal_quality = 100

        d.h_fov = m.field_of_view
        d.v_fov = m.field_of_view
        d.orientation = d.ROTATION_DOWNWARD_FACING

        d.min_distance = 0.08
        d.max_distance = 30.0

        self.range_publisher.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = DronecanListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
