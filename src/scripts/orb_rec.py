#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2, Image

def pose_callback(msg):
    print("Received PoseStamped message:")
    print("Position: x={}, y={}, z={}".format(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z))
    print("Orientation: x={}, y={}, z={}, w={}".format(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w))
    
def map_callback(msg):
	print("received map")

def main(args=None):
    rclpy.init(args=args)

    node = rclpy.create_node('pose_subscriber')

    # Subscriber to the "/orb_slam2_rgbd/pose" topic with PoseStamped messages
    subscription = node.create_subscription(
        PoseStamped,
        '/orb_slam2_rgbd/pose',
        pose_callback,
        10)
        
    sub2 = node.create_subscription(
        PointCloud2,
        '/orb_slam2_rgbd/map_points',
        map_callback,
        10)
    sub3 = node.create_subscription(
        Image,
        '/orb_slam2_rgbd/debug_image',
        map_callback,
        10)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Cleanup
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

