#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformBroadcaster

class LaserScanRepublisher(Node):
    def __init__(self):
        super().__init__('laser_scan_republisher')

        self.pose_topic = '/pf/pose/odom'
        self.laser_scan_topic = '/scan'
        self.output_scan_topic = '/scan_transformed'
        self.new_frame = 'scan_virtual'

        # Broadcasts the pose as a transform
        self.br = TransformBroadcaster(self)

        # Subscribe to pf pose
        self.create_subscription(PoseStamped, self.pose_topic, self.pose_callback, 10)

        # Subscribe to incoming laser scan
        self.create_subscription(LaserScan, self.laser_scan_topic, self.laser_callback, 10)

        # Publisher for the transformed laser scan
        self.laser_pub = self.create_publisher(LaserScan, self.output_scan_topic, 10)

        self.latest_pose = None

    def pose_callback(self, msg: PoseStamped):
        self.latest_pose = msg.pose

        # Broadcast transform from map to the new virtual scan frame
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = self.new_frame
        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)

    def laser_callback(self, msg: LaserScan):
        # Republish the same scan, just changing the frame_id to our new virtual frame
        if self.latest_pose is None:
            self.get_logger().warn("No pose received yet. Cannot transform laser scan.")
            return

        new_msg = LaserScan()
        new_msg.header.stamp = self.get_clock().now().to_msg()
        new_msg.header.frame_id = self.new_frame  # Frame from the transform we broadcast
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