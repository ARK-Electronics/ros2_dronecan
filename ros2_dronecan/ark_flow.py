#!/usr/bin/env python3
import rclpy, dronecan, time, math

from rclpy.node import Node
from std_msgs.msg import String
# from argparse import ArgumentParser

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)

        node = dronecan.make_node(port='can0', node_id=100, bitrate=1000000)
        node_monitor = dronecan.app.node_monitor.NodeMonitor(node)

        dynamic_node_id_allocator = dronecan.app.dynamic_node_id.CentralizedServer(node, node_monitor)

        node.add_handler(None, lambda msg: dronecan_msg_callback)

        node.spin(0)

    def dronecan_msg_callback(self):
        # Print received DC message
        print(dronecan.to_yaml(msg))

        # Now publish to ROS2
        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


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
