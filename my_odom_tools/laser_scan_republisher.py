#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster


class LaserScanRepublisher(Node):
    def __init__(self):
        super().__init__('laser_scan_republisher')

        self.pose_topic = '/pf/pose/odom'
        self.input_scan_topic = '/scan'
        self.output_scan_topic = '/scan_transformed'
        self.new_frame = 'scan_virtual'

        self.br = TransformBroadcaster(self)

        self.latest_pose = None
        self.latest_stamp = None

        self.create_subscription(Odometry, self.pose_topic, self.pose_callback, 10)
        self.create_subscription(LaserScan, self.input_scan_topic, self.laser_callback, 10)
        self.laser_pub = self.create_publisher(LaserScan, self.output_scan_topic, 10)

        self.get_logger().info("LaserScanRepublisher node started.")

    def pose_callback(self, msg: Odometry):
        self.latest_pose = msg.pose.pose
        self.latest_stamp = msg.header.stamp

    def laser_callback(self, msg: LaserScan):
        if self.latest_pose is None or self.latest_stamp is None:
            self.get_logger().warn("No pose received yet — cannot transform scan.")
            return

        # Broadcast transform with scan timestamp
        t = TransformStamped()
        t.header.stamp = msg.header.stamp  # match scan time
        t.header.frame_id = 'map'
        t.child_frame_id = self.new_frame
        t.transform.translation.x = self.latest_pose.position.x
        t.transform.translation.y = self.latest_pose.position.y
        t.transform.translation.z = self.latest_pose.position.z
        t.transform.rotation = self.latest_pose.orientation

        self.br.sendTransform(t)

        # Republish scan in the new frame
        new_msg = LaserScan()
        new_msg.header.stamp = msg.header.stamp
        new_msg.header.frame_id = self.new_frame

        new_msg.angle_min = msg.angle_min
        new_msg.angle_max = msg.angle_max
        new_msg.angle_increment = msg.angle_increment
        new_msg.time_increment = msg.time_increment
        new_msg.scan_time = msg.scan_time
        new_msg.range_min = msg.range_min
        new_msg.range_max = msg.range_max
        new_msg.ranges = msg.ranges
        new_msg.intensities = msg.intensities

        self.laser_pub.publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserScanRepublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
