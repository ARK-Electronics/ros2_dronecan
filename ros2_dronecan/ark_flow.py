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

        # print(dronecan.to_yaml(msg))
        self.flow_publisher.publish(flow)

        # -------------------------------------------------------------------------------------------------
        # Dronecan message definition
        # -------------------------------------------------------------------------------------------------
        # uint64 timestamp               # time since system start (microseconds)
        # uint64 timestamp_sample

        # uint32 device_id               # unique device ID for the sensor that does not change between power cycles

        # float32[2] pixel_flow          # (radians) optical flow in radians where a positive value is produced by a RH rotation about the body axis

        # float32[3] delta_angle         # (radians) accumulated gyro radians where a positive value is produced by a RH rotation about the body axis. Set to NaN if flow sensor does not have 3-axis gyro data.
        # bool delta_angle_available

        # float32 distance_m             # (meters) Distance to the center of the flow field
        # bool distance_available

        # uint32 integration_timespan_us # (microseconds) accumulation timespan in microseconds

        # uint8 quality                  # quality, 0: bad quality, 255: maximum quality

        # uint32 error_count

        # float32 max_flow_rate          # (radians/s) Magnitude of maximum angular which the optical flow sensor can measure reliably

        # float32 min_ground_distance    # (meters) Minimum distance from ground at which the optical flow sensor operates reliably
        # float32 max_ground_distance    # (meters) Maximum distance from ground at which the optical flow sensor operates reliably

        # uint8 MODE_UNKNOWN        = 0
        # uint8 MODE_BRIGHT         = 1
        # uint8 MODE_LOWLIGHT       = 2
        # uint8 MODE_SUPER_LOWLIGHT = 3

        # uint8 mode

    def range_callback(self, msg):
        # PX4 CAN Node Range Publisher
        # https://github.com/PX4/PX4-Autopilot/blob/main/src/drivers/uavcannode/Publishers/RangeSensorMeasurement.hpp

        # -------------------------------------------------------------------------------------------------
        # Dronecan message definition
        # -------------------------------------------------------------------------------------------------
        # uavcan.Timestamp timestamp

        # uint8 sensor_id

        # uavcan.CoarseOrientation beam_orientation_in_body_frame

        # float16 field_of_view                # Radians

        # uint5 SENSOR_TYPE_UNDEFINED = 0
        # uint5 SENSOR_TYPE_SONAR     = 1
        # uint5 SENSOR_TYPE_LIDAR     = 2
        # uint5 SENSOR_TYPE_RADAR     = 3
        # uint5 sensor_type

        # uint3 READING_TYPE_UNDEFINED   = 0   # Range is unknown
        # uint3 READING_TYPE_VALID_RANGE = 1   # Range field contains valid distance
        # uint3 READING_TYPE_TOO_CLOSE   = 2   # Range field contains min range for the sensor
        # uint3 READING_TYPE_TOO_FAR     = 3   # Range field contains max range for the sensor
        # uint3 reading_type

        # float16 range                        # Meters

        m = msg.message
        d = DistanceSensor()
        d.current_distance = m.range

        if m.sensor_type == m.SENSOR_TYPE_UNDEFINED:
            d.type = 4 # out of range == unknowm
        elif m.sensor_type == m.SENSOR_TYPE_SONAR:
            d.type = distanceMAV_DISTANCE_SENSOR_ULTRASOUND
        elif m.sensor_type == m.SENSOR_TYPE_LIDAR:
            d.type = d.MAV_DISTANCE_SENSOR_LASER
        elif m.sensor_type == m.SENSOR_TYPE_RADAR:
            d.type = d.MAV_DISTANCE_SENSOR_RADAR

        if m.reading_type == m.READING_TYPE_UNDEFINED:
            d.signal_quality = 0
        elif m.reading_type == m.READING_TYPE_VALID_RANGE:
            d.signal_quality = 99
        elif m.reading_type == m.READING_TYPE_TOO_CLOSE:
            d.signal_quality = 0
        elif m.reading_type == m.READING_TYPE_TOO_FAR:
            d.signal_quality = 0

        d.h_fov = m.field_of_view

        # print(dronecan.to_yaml(msg))
        self.range_publisher.publish(d)


def main(args=None):
    rclpy.init(args=args)
    node = DronecanListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
