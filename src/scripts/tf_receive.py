#!/usr/bin/env python3

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped
import yaml

def read_transforms_from_file(file_path):
    with open(file_path, 'r') as file:
        transforms_yaml = yaml.safe_load_all(file)
        return transforms_yaml

def main():
    rospy.init_node('static_tf_publisher')
    static_transform_publisher = tf2_ros.StaticTransformBroadcaster()
    print("hii")
    file_path = 'static_tf.yaml'  # Replace with your file path
    with open(file_path, 'r') as file:
        print("helloo")
        transforms = yaml.safe_load_all(file)
        print("helloo")
        for i in transforms:
            print("hi")
            if i != None:
                transforms  = i["transforms"]
                break
        print("helloo")
        rate = rospy.Rate(10)  # Adjust as needed

        while not rospy.is_shutdown():
            for transform_yaml in transforms:
                transform = TransformStamped()
                transform.header.stamp = rospy.Time.now()
                transform.header.frame_id = transform_yaml['header']['frame_id']
                transform.child_frame_id = transform_yaml['child_frame_id']
                transform.transform.translation.x = transform_yaml['transform']['translation']['x']
                transform.transform.translation.y = transform_yaml['transform']['translation']['y']
                transform.transform.translation.z = transform_yaml['transform']['translation']['z']
                transform.transform.rotation.x = transform_yaml['transform']['rotation']['x']
                transform.transform.rotation.y = transform_yaml['transform']['rotation']['y']
                transform.transform.rotation.z = transform_yaml['transform']['rotation']['z']
                transform.transform.rotation.w = transform_yaml['transform']['rotation']['w']

                static_transform_publisher.sendTransform(   [transform])
                print("published")

            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

