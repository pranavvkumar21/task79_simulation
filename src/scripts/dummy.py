#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class MultiSubscriber(Node):
    def __init__(self, topics):
        super().__init__('multi_subscriber')
        self.subscribers = []
        for topic in topics:
            print("hu")
            self.subscribers.append(self.create_subscription(PointStamped, topic, lambda msg, topic=topic: self.callback(msg, topic),10))
            
        print(self.subscribers[1].topic)

    def callback(self, msg, topic):
        print("hu")
        self.get_logger().info('Received point on topic %s: x=%f, y=%f, z=%f' % (topic, msg.point.x, msg.point.y, msg.point.z))


def main(args=None):
    rclpy.init(args=args)
    # List of topics
    topics = ['/odom_pose', '/tracked_pose_with_covariance']  # Add more topics as needed
    multi_subscriber = MultiSubscriber(topics)
    rclpy.spin(multi_subscriber)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
