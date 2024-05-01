# Launch file to subscribe to Gazebo topics and 
# republish them in ROS2

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    # Declare individual nodes
    imu_publisher_node = Node(
        package="uav_gazebo",
        executable="imu_publisher",
    )

    laser_scan_node = Node(
        package="uav_gazebo",
        executable="laserscan"
    )

    # Declare the launch options
    ld.add_action(imu_publisher_node)
    ld.add_action(laser_scan_node)

    # Add any conditioned actions
    #ld.add_action()

    return ld