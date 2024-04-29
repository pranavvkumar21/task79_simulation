#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        
        self.subscription = self.create_subscription(
            PoseStamped,
            '/slam_out_pose',
            self.pose_callback,
            10)
        
    def pose_callback(self, msg):
        self.get_logger().info("Received pose:\n{}".format(msg))

def main(args=None):
    rclpy.init(args=args)

    pose_subscriber = PoseSubscriber()

    try:
        rclpy.spin(pose_subscriber)
    except KeyboardInterrupt:
        pass

    pose_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
