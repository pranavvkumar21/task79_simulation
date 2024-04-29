#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped

class OdometryConverter(Node):

    def __init__(self):
        super().__init__('odometry_converter')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odometry_callback,
            10)
        self.publisher_ = self.create_publisher(
            PoseWithCovarianceStamped,
            '/odom_pose',
            10)
        self.previous_position = None

    def odometry_callback(self, msg):
        # Create PoseWithCovarianceStamped message
        pose_with_covariance_msg = PoseWithCovarianceStamped()
        pose_with_covariance_msg.header = msg.header
        pose_with_covariance_msg.pose.pose = msg.pose.pose
        pose_with_covariance_msg.pose.covariance = msg.pose.covariance

        # Publish the pose
        self.publisher_.publish(pose_with_covariance_msg)

        # Store current position as previous position
        self.previous_position = msg.pose.pose.position

def main(args=None):
    rclpy.init(args=args)
    odometry_converter = OdometryConverter()
    rclpy.spin(odometry_converter)
    odometry_converter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
