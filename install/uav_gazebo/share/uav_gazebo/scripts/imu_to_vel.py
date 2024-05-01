#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from task79_interfaces.msg import State, Vector, VectorStamped
from task79.mpf_utils import *
from geometry_msgs.msg import Vector3, PoseWithCovarianceStamped, TwistWithCovarianceStamped
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import numpy as np

#___Global Variables:
QUEUE_SIZE = 5

#__Classes:
class IMUToVelocity(Node):

    def __init__(self, queue=QUEUE_SIZE):
        super().__init__('imu_to_velocity')
        
        self.imu_sub = self.create_subscription(
            Imu,'imu',
            self.imu_callback,queue)
        
        self.attitude_pub = self.create_publisher(
            VectorStamped,'attitude_estimator', queue)
        
        self.velocity_pub = self.create_publisher(
            VectorStamped, 'velocity_estimator', queue)

        self.last_imu_msg = None
        self.last_imu_time = None
        
        self.velocity = VectorCovarianceStamped()
        self.attitude = VectorCovarianceStamped()
        self.acceleration = VectorCovarianceStamped()
        self.velocity.vector = [0.0, 0.0, 0.0]

    def imu_callback(self, imu_msg):
        if self.last_imu_msg is None:
            self.last_imu_msg = imu_msg
            self.last_imu_time = self.get_clock().now()
            return
        
        current_time = self.get_clock().now()
        dt = (current_time - self.last_imu_time).nanoseconds / 1e9
        #print('dt: \n', dt)
        self.attitude.header = imu_msg.header
        self.velocity.header = imu_msg.header

        acc = [imu_msg.linear_acceleration.x, 
               imu_msg.linear_acceleration.y, 
               imu_msg.linear_acceleration.z]

        # Extract orientation quaternion from IMU message
        at = [imu_msg.orientation.x, imu_msg.orientation.y,
        imu_msg.orientation.z, imu_msg.orientation.w]
        #print('Attitude in quaternion: \n', at)

        # Convert quaternion to Euler angles (attitude)
        r = R.from_quat(at)
        self.attitude.vector = r.as_euler('XYZ', degrees=True)
        self.attitude.covariance = imu_msg.orientation_covariance
        #print('Attitude in euler angles: \n', self.attitude.vector)
        #print('Attitude covariance: \n', self.attitude.covariance)

        # Integrate linear acceleration to get velocity
        self.velocity.vector = [self.velocity.vector[i] + acc[i]*dt for i in range(3)]
        
        self.velocity.covariance = imu_msg.linear_acceleration_covariance
        #print('Linear velocity: \n', self.velocity.vector)
        #print('Linear velocity covariance: \n', self.velocity.covariance)

        # Update time and message for next iteration
        self.last_imu_msg = imu_msg
        self.last_imu_time = current_time

        # Publish velocity message & covariance
        velocity_msg = VectorStamped()
        velocity_msg.header = imu_msg.header
        velocity_msg.vector.x = self.velocity.vector[0]
        velocity_msg.vector.y = self.velocity.vector[1]
        velocity_msg.vector.z = self.velocity.vector[2]

        # covariance
        velocity_msg.vector.covariance = self.velocity.covariance
        #print('Velocity msg: \n', velocity_msg)
        self.velocity_pub.publish(velocity_msg)

        # Publish attitude message
        attitude_msg = VectorStamped()
        attitude_msg.header = imu_msg.header
        attitude_msg.vector.x = self.attitude.vector[0]
        attitude_msg.vector.y = self.attitude.vector[1]
        attitude_msg.vector.z = self.attitude.vector[2]

        # covariance
        attitude_msg.vector.covariance = self.attitude.covariance
        #print('Attitude msg: \n', attitude_msg)
        self.attitude_pub.publish(attitude_msg)
        #print('Attitude msg: \n', attitude_msg)

        # Extract angular velocity from IMU message
        # angular_velocity = imu_msg.angular_velocity
        #self.get_logger().info('Publishing attitude and velocity...')


#___Main Method:
def main(args=None):
    # initialize node and start publishing
    rclpy.init(args=args)
    imu_to_velocity = IMUToVelocity()
    rclpy.spin(imu_to_velocity)

    # shuts down node and releases everything
    imu_to_velocity.destroy_node()
    rclpy.shutdown()

    return None

#___Driver Program:
if __name__ == '__main__':
    try:
        main()
    except Exception:
        pass
