#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header
import numpy as np

class PosePublisher(Node):
    def __init__(self):
        super().__init__('pose_publisher')
        self.publisher_ = self.create_publisher(PoseWithCovarianceStamped, '/tracked_pose_with_covariance', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.pose_msg = PoseWithCovarianceStamped()

    def timer_callback(self):
        self.pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.pose_msg.header.frame_id = 'map'
        # Example pose values
        self.pose_msg.pose.pose.position.x = 1.0
        self.pose_msg.pose.pose.position.y = 2.0
        self.pose_msg.pose.pose.position.z = 0.0
        self.pose_msg.pose.pose.orientation.x = 0.0
        self.pose_msg.pose.pose.orientation.y = 0.0
        self.pose_msg.pose.pose.orientation.z = 0.0
        self.pose_msg.pose.pose.orientation.w = 1.0
        # Example covariance values
        self.pose_msg.pose.covariance = np.zeros((6,6)).flatten().tolist()
        self.publisher_.publish(self.pose_msg)
        print("published")


def main(args=None):
    rclpy.init(args=args)
    pose_publisher = PosePublisher()
    rclpy.spin(pose_publisher)
    pose_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

