#!/usr/bin/env python3
import rclpy, dronecan, time, math

from rclpy.node import Node
from std_msgs.msg import String
# from argparse import ArgumentParser

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.cannode = dronecan.make_node('can0', node_id=100, bitrate=1000000)
        self.cannode_monitor = dronecan.app.node_monitor.NodeMonitor(self.cannode)
        self.dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(self.cannode, self.cannode_monitor)
        self.i = 0

        self.cannode.add_handler(dronecan.com.hex.equipment.flow.Measurement, self.flow_callback)
        self.cannode.add_handler(dronecan.uavcan.equipment.range_sensor.Measurement, self.range_callback)

        # Timer callback for processing received messages -- 200Hz
        self.timer = self.create_timer(0.005, self.timer_callback)


    def timer_callback(self):
        self.cannode.spin(0)


    def flow_callback(self, msg):
        # Print received DC message
        print("received flow message")
        print(dronecan.to_yaml(msg))

        # Now publish to ROS2
        # msg = String()
        # msg.data = 'Hello World: %d' % self.i
        # self.publisher_.publish(msg)
        # self.get_logger().info('Publishing: "%s"' % msg.data)
        # self.i += 1

    def range_callback(self, msg):
        print("received ramge message")
        print(dronecan.to_yaml(msg))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
