#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomRepublisher(Node):
    def __init__(self):
        super().__init__('odom_republisher')

        # Subscriber to /pf/pose/odom
        self.subscription = self.create_subscription(
            Odometry,
            '/pf/pose/odom',
            self.odom_callback,
            10
        )

        # Publisher to /odom
        self.publisher = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.get_logger().info('Odom republisher node started.')

    def odom_callback(self, msg: Odometry):
        # Simply republish the message
        self.publisher.publish(msg)
        self.get_logger().debug('Republished odometry message.')

def main(args=None):
    rclpy.init(args=args)
    node = OdomRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
