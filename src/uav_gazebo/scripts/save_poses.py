#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import datetime
import os

class PoseSubscriber(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        
        self.tracked_pose_sub = self.create_subscription(
            PoseStamped, '/tracked_pose', self.tracked_pose_callback, 10)
        
        self.slam_out_pose_sub = self.create_subscription(
            PoseStamped, '/slam_out_pose', self.slam_out_pose_callback, 10)
        
        self.orb_slam_pose_sub = self.create_subscription(
            PoseStamped, '/orb_slam2_rgbd/pose', self.orb_slam_pose_callback, 10)
        
        self.tracked_positions = []
        self.slam_out_positions = []
        self.orb_slam_positions = []
        
    def tracked_pose_callback(self, msg):
        position = msg.pose.position
        self.tracked_positions.append((position.x, position.y, position.z))
        self.get_logger().info("Received pose from cartographer:\n")
        
    def slam_out_pose_callback(self, msg):
        position = msg.pose.position
        self.slam_out_positions.append((position.x, position.y, position.z))
        self.get_logger().info("Received pose from hector_slam:\n")
    def orb_slam_pose_callback(self, msg):
        position = msg.pose.position
        self.orb_slam_positions.append((position.x, position.y, position.z))
        self.get_logger().info("Received pose from orb_slam:\n")
    def save_positions_to_csv(self, positions, filename):
        with open(filename, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['X', 'Y', 'Z'])
            writer.writerows(positions)

def main(args=None):
    rclpy.init(args=args)

    pose_subscriber = PoseSubscriber()

    try:
        rclpy.spin(pose_subscriber)
    except KeyboardInterrupt:
        pass
    plot_path = "./plot_data/"
    current_datetime = datetime.datetime.now()
    formatted_datetime = current_datetime.strftime("%m_%d_%H_%M_%S")	
    os.chdir(plot_path)
    os.mkdir(formatted_datetime)
    os.chdir(formatted_datetime)
    
    # Save positions to CSV files
    pose_subscriber.save_positions_to_csv(pose_subscriber.tracked_positions, 'tracked_pose.csv')
    pose_subscriber.save_positions_to_csv(pose_subscriber.slam_out_positions, 'slam_out_pose.csv')
    pose_subscriber.save_positions_to_csv(pose_subscriber.orb_slam_positions, 'orb_slam_pose.csv')

    pose_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
