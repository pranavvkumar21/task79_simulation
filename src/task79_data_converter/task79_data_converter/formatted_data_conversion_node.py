#!/usr/bin/env python3

# ------------------
# Author: Achille Martin - achille.martin@noc.ac.uk
# Date: 26/02/2024
# Bugs: No known bugs
#
# @copyright 2024 National Oceanography Centre
# 
# Context: Progeny Task79 project
# 
# This file is subject to the terms and conditions defined in file
# 'LICENSE.txt', which is part of this source code package.
# 
# ------------------

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

import threading
from inspect import cleandoc as cl
import pandas as pd
from math import pi
from numpy import sign
from datetime import (
    datetime,
    timezone,
)
import time
from os import (
    listdir,
    getcwd,
)
from os.path import (
    isfile,
    isdir,
    join,
    realpath,
    expandvars,
    dirname,
    pardir,
)
import sys

importer_folder = dirname(
	realpath(__file__)
)
parent_of_importer_folder = join(
	importer_folder, 
	pardir,
)
parent_of_parent_of_importer_folder = join(
	parent_of_importer_folder, 
	pardir,
)

sys.path.append(parent_of_importer_folder)
sys.path.append(parent_of_parent_of_importer_folder)

from geometry_msgs.msg import (
    Point,
    Pose,
    PoseWithCovariance,
    PoseWithCovarianceStamped,
)
from task79_interfaces.msg import (
    Vector, 
    VectorStamped,
    SensorConfigStamped, 
    SensorStatusStamped,
    ModuleConfigStamped,
)

from tools.file_management.load_data_from_file import load_data_from_file
from tools.unit_conversion.unit_converter import wrap_to_pi
from tools.ros2_management.ros2_extended_functionality_helper import (
    collect_ros_param,
)

class FormattedDataConverter(Node):
 
    def __init__(
            self,
            default_node_name,
            default_node_logging_level,
            default_node_pub_queue_size,
            default_node_sub_queue_size,
            default_package_path,
            default_node_spin_frequency_hz,
            default_sensor_data_folder,
            default_is_aiding_data_grouped,
            default_sensor_covariance_length,
            default_sensor_covariance,
            default_sensor_sampling_proportion,
            default_environment,
            default_depth,
            default_timestamp_delta_acceptance):
        super().__init__(
            default_node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
        
        # Define and initialise handy attributes
        
        ## Sensor config record contains a list of SensorConfig
        self.sensor_config_record = []
        
        ## Sensor dataset record contains a list of dict
        ## Each dict is composed of:
        ## * sensor_type
        ## * sensor_source
        ## * sensor_data
        self.sensor_dataset_record = [] 
        
        ## Aiding data publisher record list contains a list of dict
        ## Each dict is composed of:
        ## * sensor_type
        ## * sensor_source
        ## * publisher_topic
        ## * publisher_object
        ## These publishers are used when is_aiding_data_grouped is False
        self.aiding_data_publisher_record = []

        ## Define and initialise handy attributes
        self.default_node_logging_level = default_node_logging_level
        self.default_node_pub_queue_size = default_node_pub_queue_size
        self.default_node_sub_queue_size = default_node_sub_queue_size
        self.package_path = default_package_path
        self.node_spin_frequency_hz = default_node_spin_frequency_hz
        self.sensor_data_folder = default_sensor_data_folder
        self.is_aiding_data_grouped = default_is_aiding_data_grouped
        self.default_sensor_covariance_length = default_sensor_covariance_length
        self.default_sensor_covariance = default_sensor_covariance
        self.default_sensor_sampling_proportion = default_sensor_sampling_proportion
        self.default_environment = default_environment
        self.default_depth = default_depth
        self.timestamp_delta_acceptance = default_timestamp_delta_acceptance
       
        ## Initialise attributes with "empty" values
        self.sensor_timestamp_equivalence_shift = None
        self.sensor_dataset_config = dict()

        # Set node logging level
        rclpy.logging.set_logger_level(
            self.get_logger().name,
            self.default_node_logging_level,
        )
        
        # Set up publishers
        self.aiding_position_source_pub = self.create_publisher(
	        VectorStamped, 
	        '/aiding_data',
            self.default_node_pub_queue_size,
	    )
        self.attitude_source_pub = self.create_publisher(
	        VectorStamped, 
	        '/attitude_estimator',
            self.default_node_pub_queue_size,
	    )
        self.velocity_source_pub = self.create_publisher(
	        VectorStamped, 
	        '/velocity_estimator',
            self.default_node_pub_queue_size,
	    )
        self.sensor_config_status_pub = self.create_publisher(
	        SensorStatusStamped, 
	        '/sensor_config_status',
            self.default_node_pub_queue_size,
	    )

        self.get_logger().info(
            cl(
                f"""
                FormattedDataConverter::__init__ -
                Created the publishers
                """
            )
        )
        
        # Set up subscribers
        self.sensor_config_update_sub = self.create_subscription(
            SensorConfigStamped,
            '/sensor_config_update',
            self.on_sensor_config_update_received,
            self.default_node_sub_queue_size,
        )
        self.module_config_update_sub = self.create_subscription(
            ModuleConfigStamped,
            '/module_config_update',
            self.on_module_config_update_received,
            self.default_node_sub_queue_size,
        )
        
        self.get_logger().info(
            cl(
                f"""
                FormattedDataConverter::__init__ -
                Created the subscribers
                """
            )
        )

        # Declare ROS parameters
        # TODO: declare each ROS parameter individually
        # even if nested, following ROS2 recommendations
        #
        # NOTE: the current chosen declaration implementation
        # consists in collecting all ROS2 params regularly
        # into a dictionary,
        # using `collect_ros_param()` method
        # This is possible, even if their name / type is unknown,
        # thanks to Node constructor args
        self.param_dict = dict()
        
        # Collect ROS params on initialisation
        # Expected to be related to
        # Dataset config
        self.param_dict = collect_ros_param(self)
        
        # Collect dataset config on initialisation
        self.sensor_dataset_config = self.get_dataset_startup_config()
 
    def run(self):
        # Set sensor past and current timestamp equivalence
        # to establish a relationship between past mission timestamp
        # and current timestamp
        # This way, past mission data won't trigger age checks
        # in the MPF
        if self.sensor_timestamp_equivalence_shift is None:
            self.set_sensor_timestamp_equivalence()

        # Transmit sensor status to user
        self.transmit_sensor_config_status()
        
        # Transmit sensor data to MPF
        self.transmit_sensor_data()

    def update_sensor_config(self, input_data):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::update_sensor_config -
                Received request to update the sensor config with data 
                {input_data}
                """
            )
        )

        input_sensor_type = input_data.sensor_type
        input_sensor_covariance = input_data.sensor_covariance.tolist()
        input_is_sensor_enabled = input_data.is_sensor_enabled
        
        # Setup flags for sensor config record update
        is_sensor_type_recorded = False
        sensor_config_to_update_index = None

        # Scan through the sensor config records
        for sensor_config_index, sensor_config in enumerate(self.sensor_config_record):
            # Confirm whether sensor config has already been recorded
            if sensor_config.sensor_type == input_sensor_type:
                is_sensor_type_recorded = True
                sensor_config_to_update_index = sensor_config_index
                break

        # Perform the sensor record update depending on the flags
        if is_sensor_type_recorded:
            # In the case of the sensor type being already recorded,
            # update the relevant fields
            self.sensor_config_record[sensor_config_to_update_index].is_sensor_enabled = \
                input_is_sensor_enabled
            if self.validate_sensor_covariance_format(input_sensor_covariance):
                self.sensor_config_record[sensor_config_to_update_index].sensor_covariance = \
                    input_sensor_covariance
            else:
                self.get_logger().warn(
                    cl(
                        f"""
                        FormattedDataConverter::update_sensor_config -
                        Covariance format of {input_sensor_covariance} NOT right.
                        NOT updating the sensor covariance
                        """
                    )
                )
        else:
            # In the case of the sensor type not being already recorded,
            # check that sensor data is available
            is_sensor_data_available = self.check_sensor_data_availability(input_sensor_type)
            if is_sensor_data_available:
                # If sensor data available for sensor type,
                # Add new entry to the sensor config record
                self.sensor_config_record.append(input_data)
                # Confirm format of sensor covariance
                if not self.validate_sensor_covariance_format(input_sensor_covariance):
                    self.get_logger().warn(
                        cl(
                            f"""
                            FormattedDataConverter::update_sensor_config -
                            Covariance format of {input_sensor_covariance} NOT right.
                            Updating sensor log record with default sensor covariance:
                            {self.default_sensor_covariance}
                            """
                        )
                    )
                    self.sensor_config_record[-1].sensor_covariance = self.default_sensor_covariance
                # Load datasets related to sensor type
                self.load_sensor_data(input_sensor_type)
            else:
                self.get_logger().warn(
                    cl(
                        f"""
                        FormattedDataConverter::update_sensor_config -
                        Sensor data not available for type {input_sensor_type}:
                        NOT updating the sensor config.
                        Check your sensor data folder.
                        """
                    )
                )

    def update_module_config(self, input_data):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::update_module_config -
                Received request to update the module config with data:
                {input_data}
                """
            )
        )

        # Validate content of module config
        # And update if validated
        node_spin_frequency_hz = input_data.node_spin_frequency_hz
        sensor_data_folder = input_data.sensor_data_folder
        is_aiding_data_grouped = input_data.is_aiding_data_grouped

        # Verify spin frequency
        if node_spin_frequency_hz > 0:
            self.node_spin_frequency_hz = node_spin_frequency_hz
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::update_module_config -
                    Updated node spin frequency to:
                    {self.node_spin_frequency_hz} Hz
                    """
                )
            )
        else:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::update_module_config -
                    Node spin frequency received is NOT > 0:
                    {node_spin_frequency_hz} Hz
                    """
                )
            )
        
        # Define temporary paths for validation
        expanded_relative_path_sensor_data_folder = join(
            self.package_path, 
            sensor_data_folder,
        )
        full_path_sensor_data_folder = realpath(
            expandvars(sensor_data_folder)
        )
        if isdir(expanded_relative_path_sensor_data_folder):
            self.sensor_data_folder = expanded_relative_path_sensor_data_folder
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::update_module_config -
                    Updated sensor data folder to:
                    {self.sensor_data_folder}
                    """
                )
            )
        elif isdir(full_path_sensor_data_folder):
            self.sensor_data_folder = full_path_sensor_data_folder
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::update_module_config -
                    Updated sensor data folder to:
                    {self.sensor_data_folder}
                    """
                )
            )
        else:
            self.get_logger().error(
                cl(
                    f"""
                    FormattedDataConverter::update_module_config -
                    Sensor data folder (relative or absolute) path received cannot be found:
                    {sensor_data_folder}
                    Keeping sensor data folder path:
                    {self.sensor_data_folder}
                    """
                )
            )
        
        # Verify aiding data grouped stream
        if is_aiding_data_grouped is not None:
            self.is_aiding_data_grouped = is_aiding_data_grouped
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::update_module_config -
                    Updated aiding data grouped stream to:
                    {self.is_aiding_data_grouped}
                    """
                )
            )
        else:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::update_module_config -
                    Aiding data grouped stream is not a boolean:
                    {is_aiding_data_grouped}
                    """
                )
            )

    def check_sensor_data_availability(self, sensor_type):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::check_sensor_data_availability -
                Received request to check sensor data availability for:
                {sensor_type}
                """
            )
        )

        # Read through sensor data folder
        file_list = [
            f for f in listdir(self.sensor_data_folder) 
            if (isfile(join(self.sensor_data_folder, f)))
        ]

        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::check_sensor_data_availability -
                Found list of files in {self.sensor_data_folder}:
                {file_list}
                """
            )
        )

        # Check existence of file corresponding to sensor type
        try:
            sensor_type_dataset_details = self.sensor_dataset_config.get(sensor_type)
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::check_sensor_data_availability -
                    Found dataset details in {sensor_type} config:
                    {sensor_type_dataset_details}
                    """
                )
            )
            for source in sensor_type_dataset_details:
                sensor_type_source_dataset_details = sensor_type_dataset_details.get(source) 
                self.get_logger().debug(
                    cl(
                        f"""
                        FormattedDataConverter::check_sensor_data_availability -
                        Found source dataset details in {sensor_type} config:
                        {sensor_type_source_dataset_details}
                        """
                    )
                )
                dataset_filename = \
                    sensor_type_source_dataset_details.get('file_basename') \
                    + '.' \
                    + sensor_type_source_dataset_details.get('file_type') 
                self.get_logger().debug(
                    cl(
                        f"""
                        FormattedDataConverter::check_sensor_data_availability -
                        Found dataset filename in {sensor_type} config:
                        {dataset_filename}
                        """
                    )
                )
                
                if dataset_filename in file_list:
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::check_sensor_data_availability -
                            There is {sensor_type} dataset in {self.sensor_data_folder}
                            """
                        )
                    )
                    # Implementation improvement: 
                    # Check all datasets related to sensor type and not just one
                    return True

        except Exception as e:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::check_sensor_data_availability -
                    Exception {e} 
                    Caught when trying to check existence of dataset for {sensor_type}
                    """
                )
            )

        self.get_logger().warn(
            cl(
                f"""
                FormattedDataConverter::check_sensor_data_availability -
                There is NO {sensor_type} dataset in {self.sensor_data_folder}
                """
            )
        )
        
        return False

    def validate_sensor_covariance_format(self, covariance):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::validate_sensor_covariance_format -
                Received request to validate format of covariance:
                {covariance}
                """
            )
        )

        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::validate_sensor_covariance_format -
                Expected covariance format is a list of length:
                {len(self.default_sensor_covariance)}
                """
            )
        )
        if (isinstance(covariance, (list,))
                and len(covariance) == len(self.default_sensor_covariance)):
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::validate_sensor_covariance_format -
                    Sensor covariance format validated
                    """
                )
            )
            return True
        else:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::validate_sensor_covariance_format -
                    Sensor covariance format NOT validated
                    """
                )
            )
            return False

    def transmit_sensor_config_status(self):
        # self.get_logger().debug(
        #     cl(
        #         f"""
        #         FormattedDataConverter::transmit_sensor_config_status -
        #         Received request to transmit sensor config status
        #         """
        #     )
        # )
        
        sensor_status_stamped = SensorStatusStamped()
        sensor_status_stamped.header.stamp = self.get_clock().now().to_msg()

        # Read through the sensor config record
        for sensor_config in self.sensor_config_record:
            sensor_status_stamped.sensor_config_list.append(sensor_config)

        # Publish sensor status
        self.sensor_config_status_pub.publish(sensor_status_stamped)

    def transmit_sensor_data(self):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::transmit_sensor_data -
                Received request to transmit sensor data
                """
            )
        )
      
        sensor_type = None
        sensor_source = None

        try:

            # Read through the sensor config records
            # to identify which sensors are enabled
            sensor_enabled_list = [
                sensor_config 
                for sensor_config in self.sensor_config_record
                if sensor_config.is_sensor_enabled
            ]
            for sensor_config in sensor_enabled_list:
                self.get_logger().debug(
                    cl(
                        f"""
                        FormattedDataConverter::transmit_sensor_data -
                        Found sensor config (with sensor enabled) in the records:
                        {sensor_config}
                        """
                    )
                )
            
                sensor_type = sensor_config.sensor_type
                
                # Read through the sensor dataset config records
                # to identify which sensor sources have valid data
                sensor_enabled_dataset_list = [
                    sensor_dataset_config 
                    for sensor_dataset_config in self.sensor_dataset_record
                    if (sensor_dataset_config.get('sensor_type') == sensor_type 
                        and sensor_dataset_config.get('sensor_data') is not None) 
                ]

                for sensor_dataset_config in sensor_enabled_dataset_list:
                    sensor_source = sensor_dataset_config.get('sensor_source')
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::transmit_sensor_data -
                            Found sensor dataset config in the records for:
                            sensor_type = {sensor_type} and sensor_source = {sensor_source}
                            """
                        )
                    )

                    # Split data transmission between the different source types

                    current_ros_timestamp = self.get_clock().now().nanoseconds/(10**9)
                    past_epoch_timestamp = \
                        current_ros_timestamp - self.sensor_timestamp_equivalence_shift

                    ## Case POSITION
                    ## Looking for:
                    ## * Latitude in DD.DDDD
                    ## * Longitude in DD.DDDD
                    ## * Depth in M
                    ## * Covariance (optional)
                    if sensor_source == 'POSITION_SOURCE':
                        
                        lat_decimal_deg = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'latitude',
                            target_unit = 'DD.DDDD',
                        )
                        lon_decimal_deg = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'longitude',
                            target_unit = 'DD.DDDD',
                        )

                        depth_m = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'depth',
                            target_unit = 'M',
                        )
                        position_covariance = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'covariance',
                        )
                        
                        # Prepare values for message to publish
                        position_vector_x = lat_decimal_deg
                        position_vector_y = lon_decimal_deg
                        position_vector_z = depth_m
                        position_vector_covariance = position_covariance 
                        position_vector_stamped_header_stamp = self.get_clock().now().to_msg()
                        
                        # Catch any issue with incorrectly defined message types
                        # before publishing
                        if (position_vector_x is None
                                or position_vector_y is None
                                or position_vector_z is None
                                or position_vector_covariance is None
                                or position_vector_stamped_header_stamp is None):
                            self.get_logger().warn(
                                cl(
                                    f"""
                                    FormattedDataConverter::transmit_sensor_data -
                                    Message for {sensor_type}
                                    is incorrectly defined
                                    NOT publishing it
                                    """
                                )
                            )
                        
                        else:
                            # Sub-case aiding data is grouped
                            if self.is_aiding_data_grouped:

                                position_vector = Vector()
                                position_vector.x = position_vector_x
                                position_vector.y = position_vector_y
                                position_vector.z = position_vector_z
                                # Overwrite 3D considerations
                                # if default environment is 2D
                                if self.default_environment == '2D':
                                    position_vector.z = self.default_depth
                                position_vector.covariance = position_vector_covariance 
                                
                                position_vector_stamped = VectorStamped()
                                position_vector_stamped.header.stamp = position_vector_stamped_header_stamp
                                position_vector_stamped.vector = position_vector

                                self.get_logger().debug(
                                    cl(
                                        f"""
                                        FormattedDataConverter::transmit_sensor_data -
                                        Created position vector stamped:
                                        {position_vector_stamped}
                                        Publishing it onto relevant topic
                                        """
                                    )
                                )

                                self.aiding_position_source_pub.publish(position_vector_stamped)

                            # Sub-case aiding data is non-grouped
                            else:

                                # Generate aiding data publisher automatically 
                                # for current sensor type and current sensor source
                                # if not already generated
                                self.generate_automatic_aiding_data_publisher(
                                    sensor_type, 
                                    sensor_source
                                )

                                position_vector = Point()
                                position_vector.x = position_vector_x
                                position_vector.y = position_vector_y
                                position_vector.z = position_vector_z
                                # Overwrite 3D considerations
                                # if default environment is 2D
                                if self.default_environment == '2D':
                                    position_vector.z = self.default_depth
                                
                                pose_vector = Pose()
                                pose_vector.position = position_vector
                                
                                empty_list = [0.0, 0.0, 0.0]
                                pose_covariance = []
                                if not position_vector_covariance:
                                    position_vector_covariance = list(self.default_sensor_covariance)
                                if len(position_vector_covariance) == self.default_sensor_covariance_length:
                                    pose_covariance = position_vector_covariance[:3] + empty_list \
                                        + position_vector_covariance[3:6] + empty_list \
                                        + position_vector_covariance[6:] + empty_list \
                                        + empty_list*2*3
                                else:
                                    self.get_logger().warn(
                                        cl(
                                            f"""
                                            FormattedDataConverter::transmit_sensor_data -
                                            Sensor covariance length is not the length expected
                                            by the module: {self.default_sensor_covariance_length}
                                            Verify sensor covariance length in sensor config
                                            """
                                        )
                                    )
                                
                                pose_with_covariance = PoseWithCovariance()
                                pose_with_covariance.pose = pose_vector
                                pose_with_covariance.covariance = pose_covariance
                                    
                                pose_with_covariance_stamped = PoseWithCovarianceStamped()
                                pose_with_covariance_stamped.header.stamp = position_vector_stamped_header_stamp
                                pose_with_covariance_stamped.pose = pose_with_covariance
                                
                                aiding_data_publisher_list = [
                                    dict_i 
                                    for dict_i in self.aiding_data_publisher_record 
                                    if (dict_i.get('sensor_type', None) == sensor_type
                                        and dict_i.get('sensor_source', None) == sensor_source
                                    )
                                ]
                                if len(aiding_data_publisher_list) != 1:
                                    self.get_logger().warn(
                                        cl(
                                            f"""
                                            FormattedDataConverter::transmit_sensor_data -
                                            Sensor type {sensor_type} 
                                            related to sensor source {sensor_source}
                                            is not unique
                                            Verify your sensor config and sensor dataset config
                                            """
                                        )
                                    )
                                aiding_data_publisher_details = aiding_data_publisher_list[0]
                                publisher_topic = aiding_data_publisher_details.get(
                                    'publisher_topic', 
                                    None,
                                )
                                publisher_object = aiding_data_publisher_details.get(
                                    'publisher_object', 
                                    None,
                                )
                                self.get_logger().debug(
                                    cl(
                                        f"""
                                        FormattedDataConverter::transmit_sensor_data -
                                        Created pose with covariance stamped:
                                        {pose_with_covariance_stamped}
                                        Publishing it onto relevant topic:
                                        {publisher_topic}
                                        """
                                    )
                                )

                                publisher_object.publish(pose_with_covariance_stamped)

                    ## Case ATTITUDE
                    ## Looking for:
                    ## * Roll in DEG
                    ## * Pitch in DEG
                    ## * Yaw in DEG
                    ## * Covariance (optional)
                    ## All expressed in NED inertial frame
                    elif sensor_source == 'ATTITUDE_SOURCE':
                        
                        roll_deg = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'roll',
                            target_unit = 'DEG',
                        )
                        pitch_deg = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'pitch',
                            target_unit = 'DEG',
                        )
                        yaw_deg = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'heading',
                            target_unit = 'DEG',
                        )
                        attitude_covariance = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'covariance',
                        )

                        
                        # Prepare values for message to publish
                        attitude_vector_x = roll_deg
                        attitude_vector_y = pitch_deg
                        attitude_vector_z = yaw_deg
                        attitude_vector_covariance = attitude_covariance 
                        attitude_vector_stamped_header_stamp = self.get_clock().now().to_msg()
                        
                        # Catch any issue with incorrectly defined message types
                        # before publishing
                        if (attitude_vector_x is None
                                or attitude_vector_y is None
                                or attitude_vector_z is None
                                or attitude_vector_covariance is None
                                or attitude_vector_stamped_header_stamp is None):
                            self.get_logger().warn(
                                cl(
                                    f"""
                                    FormattedDataConverter::transmit_sensor_data -
                                    Message for {sensor_type}
                                    is incorrectly defined
                                    NOT publishing it
                                    """
                                )
                            )
                         
                        else:
                            attitude_vector = Vector()
                            attitude_vector.x = attitude_vector_x
                            attitude_vector.y = attitude_vector_y
                            attitude_vector.z = attitude_vector_z
                            attitude_vector.covariance = attitude_vector_covariance 
                            
                            attitude_vector_stamped = VectorStamped()
                            attitude_vector_stamped.header.stamp = attitude_vector_stamped_header_stamp
                            attitude_vector_stamped.vector = attitude_vector

                            self.get_logger().debug(
                                cl(
                                    f"""
                                    FormattedDataConverter::transmit_sensor_data -
                                    Created position vector stamped:
                                    {attitude_vector_stamped}
                                    Publishing it onto relevant topic
                                    """
                                )
                            )

                            self.attitude_source_pub.publish(attitude_vector_stamped)

                    ## Case VELOCITY
                    ## Looking for:
                    ## * Surge in M/S
                    ## * Sway in M/S
                    ## * Heave in M/S
                    ## * Covariance (optional)
                    ## All expressed in SNAME body-fixed frame
                    elif sensor_source == 'VELOCITY_SOURCE':
                        
                        surge_mps = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'surge',
                            target_unit = 'M/S',
                        )
                        sway_mps = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'sway',
                            target_unit = 'M/S',
                        )
                        heave_mps = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'heave',
                            target_unit = 'M/S',
                        )
                        velocity_covariance = self.get_sensor_data(
                            sensor_type, 
                            sensor_source,
                            target_epoch_timestamp = past_epoch_timestamp,
                            target_quantity = 'covariance',
                        )

                        # Prepare values for message to publish
                        velocity_vector_x = surge_mps
                        velocity_vector_y = sway_mps
                        velocity_vector_z = heave_mps
                        velocity_vector_covariance = velocity_covariance 
                        velocity_vector_stamped_header_stamp = self.get_clock().now().to_msg()

                        # Catch any issue with incorrectly defined message types
                        # before publishing
                        if (velocity_vector_x is None
                                or velocity_vector_y is None
                                or velocity_vector_z is None
                                or velocity_vector_covariance is None
                                or velocity_vector_stamped_header_stamp is None):
                            self.get_logger().warn(
                                cl(
                                    f"""
                                    FormattedDataConverter::transmit_sensor_data -
                                    Message for {sensor_type}
                                    is incorrectly defined
                                    NOT publishing it
                                    """
                                )
                            )
                         
                        else:
                            velocity_vector = Vector()
                            velocity_vector.x = surge_mps
                            velocity_vector.y = sway_mps
                            velocity_vector.z = heave_mps
                            velocity_vector.covariance = velocity_covariance 
                            
                            velocity_vector_stamped = VectorStamped()
                            velocity_vector_stamped.header.stamp = self.get_clock().now().to_msg()
                            velocity_vector_stamped.vector = velocity_vector

                            self.get_logger().debug(
                                cl(
                                    f"""
                                    FormattedDataConverter::transmit_sensor_data -
                                    Created position vector stamped:
                                    {velocity_vector_stamped}
                                    Publishing it onto relevant topic
                                    """
                                )
                            )

                            self.velocity_source_pub.publish(velocity_vector_stamped)
           
                    ## Case UNDEFINED
                    else:
                        raise Exception(f"No support for sensor source {sensor_source}")

        except Exception as e:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::transmit_sensor_data -
                    Cannot transmit sensor data
                    Exception: {e}
                    """
                )
            )

    def set_sensor_timestamp_equivalence(self):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::set_sensor_timestamp_equivalence -
                Received request to set sensor timestamp equivalence
                """
            )
        )
         
        # This action is performed when there is at least one sensor data record
        # and at least one of the sensors is enabled

        sensor_type = None
        sensor_source = None

        if len(self.sensor_config_record) > 0:
            
            try:

                # Go through the list of sensor_config_record
                # and determine whether one is enabled
                sensor_enabled_list = [
                    sensor_config 
                    for sensor_config in self.sensor_config_record
                    if sensor_config.is_sensor_enabled
                ]
                for sensor_config in sensor_enabled_list:
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::set_sensor_timestamp_equivalence -
                            Found sensor config (with sensor enabled) in the records:
                            {sensor_config}
                            """
                        )
                    )
                        
                    sensor_type = sensor_config.sensor_type

                    # Go through the list of sensor dataset records
                    # and determine whether one corresponding to
                    # sensor type has valid sensor data
                    sensor_enabled_dataset_list = [
                        sensor_dataset_config 
                        for sensor_dataset_config in self.sensor_dataset_record
                        if (sensor_dataset_config.get('sensor_type') == sensor_type 
                            and sensor_dataset_config.get('sensor_data') is not None) 
                    ]

                    for sensor_dataset_config in sensor_enabled_dataset_list:
                        sensor_source = sensor_dataset_config.get('sensor_source')
                        self.get_logger().debug(
                            cl(
                                f"""
                                FormattedDataConverter::set_sensor_timestamp_equivalence -
                                Found sensor dataset config in the records for:
                                sensor_type = {sensor_type} and sensor_source = {sensor_source}
                                """
                            )
                        )
                        # Pick the start time from enabled sensor's dataset first row
                        sensor_data_initial_epoch_timestamp = self.get_sensor_data(
                            sensor_type,
                            sensor_source,
                            target_row=0,
                            target_quantity='epoch_timestamp',
                        )
                        ## Tip: confirm that ros is returning utc timestamp
                        current_epoch_timestamp = time.time()
                        current_ros_timestamp = self.get_clock().now().nanoseconds/(10**9)
                        if (abs(current_epoch_timestamp - current_ros_timestamp) 
                                > self.timestamp_delta_acceptance):
                            raise Exception(f"Ros timestamp not aligned with UTC timestamp. \
                                Fix your ros timestamp by setting your machine time to UTC.")
                        self.sensor_timestamp_equivalence_shift = \
                            current_ros_timestamp - sensor_data_initial_epoch_timestamp
                        self.get_logger().debug(
                            cl(
                                f"""
                                FormattedDataConverter::set_sensor_timestamp_equivalence -
                                Set sensor timestamp equivalence to:
                                {self.sensor_timestamp_equivalence_shift}
                                This means that there are {self.sensor_timestamp_equivalence_shift} s
                                between current UTC time and start of sensor dataset time
                                """
                            )
                        )

                        return True
                    
                    self.get_logger().warn(
                        cl(
                            f"""
                            FormattedDataConverter::set_sensor_timestamp_equivalence -
                            Cannot find sensor dataset config in the records for:
                            {sensor_config}
                            """
                        )
                    )
                
            except Exception as e:
                self.get_logger().warn(
                    cl(
                        f"""
                        FormattedDataConverter::set_sensor_timestamp_equivalence -
                        Cannot set sensor timestamp equivalence.
                        Exception: {e}
                        """
                    )
                )
        
        else:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::set_sensor_timestamp_equivalence -
                    Cannot set sensor timestamp equivalence.
                    Review the conditions:
                    * Sensor config record: {self.sensor_config_record}
                    * Sensor timestamp equivalence shift: {self.sensor_timestamp_equivalence_shift}
                    """
                )
            )

    def get_sensor_data(
            self, 
            sensor_type, 
            sensor_source,
            target_row = None,
            target_epoch_timestamp = None,
            target_quantity = None,
            target_unit = None):
        
        # Tip 1: Select either target_row or target_epoch_timestamp
        # to get sensor data corresponding to row index
        # or closest epoch timestamp

        # This function uses a proportion of the log frequency of sensor data (p)
        # (which is expressed as p% of timestamp_epsilon_s)
        # to search for closest matching timestamp x to target_epoch_timestamp:
        # target_epoch_timestamp - p*timestamp_epsilon_s 
        #   <= x <= target_epoch_timestamp + p*timestamp_epsilon_s

        # Tip 2: Specify target_quantity and target_unit
        # to collect the exact quantity you need
        # (for instance: 'depth')

        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::get_sensor_data -
                Received request to get sensor data with:
                * Sensor type = {sensor_type}
                * Sensor source = {sensor_source}
                * Target row = {target_row}
                * Target epoch timestamp = {target_epoch_timestamp}
                * Target quantity = {target_quantity}
                * Target unit = {target_unit}
                """
            )
        )
       
        sensor_dataset_config = None
        sensor_dataset = None
        sensor_row_data = None
        sensor_singular_data = None
        
        try:
            
            # Pre-requisites:
            # Retrieve relevant sensor dataset config
            sensor_type_config = \
                self.sensor_dataset_config.get(sensor_type)
            sensor_type_source_config = \
                sensor_type_config.get(sensor_source)

            # 1) Identify sensor dataset config from request
            sensor_dataset_list = [
                sensor_dataset_config.get('sensor_data') 
                for sensor_dataset_config in self.sensor_dataset_record
                if (sensor_dataset_config.get('sensor_type') == sensor_type
                    and sensor_dataset_config.get('sensor_source') == sensor_source
                    and sensor_dataset_config.get('sensor_data') is not None)
            ]
            if len(sensor_dataset_list) == 0:
                raise Exception(
                    f"""
                    Cannot identify sensor dataset config from request
                    """
                )

            elif len(sensor_dataset_list) > 1:
                raise Exception(
                    f"""
                    Found multiple sensor dataset configs from request
                    Make sure that the datasets are unique for each:
                    sensor type = {sensor_type} and sensor source = {sensor_source}
                    """
                )

            else:
                self.get_logger().debug(
                    cl(
                        f"""
                        FormattedDataConverter::get_sensor_data -
                        Found unique sensor dataset for
                        sensor type = {sensor_type} and sensor source = {sensor_source}
                        """
                    )
                )
                sensor_dataset = sensor_dataset_list[0]

            # 2) Identify the type of data query for the dataset
            if target_row is not None:
                sensor_row_data = sensor_dataset.iloc[[target_row], :]
                self.get_logger().debug(
                    cl(
                        f"""
                        FormattedDataConverter::get_sensor_data -
                        Collected sensor row data for target row {target_row}:
                        {sensor_row_data}
                        """
                    )
                )

            elif target_epoch_timestamp is not None:
               
                # -----------------------------------------------
                # Note: the implementation right below enables the sensors
                # to publish at every node spin
                # even if they are not supposed to have collected data yet
                # The code uses the closest time-matching data
                # at each node spin
                # through the use of log_frequency_hz
                # which is the logging frequency of the sensor
                # -----------------------------------------------
                # timestamp_epsilon_hz = \
                #     sensor_type_source_config.get('log_frequency_hz', 'UNDEFINED')
                # if timestamp_epsilon_hz == 'UNDEFINED':
                #     raise Exception(
                #         f"""
                #         Cannot find log frequency hz for sensor
                #         type = {sensor_type} and source = {sensor_source}
                #         Verify your sensor dataset config.
                #         """
                #     )
                # timestamp_epsilon_s = 1/timestamp_epsilon_hz
                # -----------------------------------------------
                
                timestamp_epsilon_s = 1/self.node_spin_frequency_hz
                sensor_dataset_matching_epoch_timestamp = \
                    sensor_dataset[
                        abs(sensor_dataset['epoch_timestamp'] - target_epoch_timestamp)
                            <= (self.default_sensor_sampling_proportion * timestamp_epsilon_s)
                    ]
                if sensor_dataset_matching_epoch_timestamp.shape[0] > 0:
                    sensor_row_data = sensor_dataset_matching_epoch_timestamp.head(1)
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Found row of dataset 
                            with matching target epoch timestamp {target_epoch_timestamp}:
                            {sensor_row_data}
                            """
                        )
                    )
                else:
                    raise Exception(
                        f"""
                        Cannot find sub-dataset matching target epoch timestamp:
                        {target_epoch_timestamp}
                        """
                    )

            else:
                raise Exception(
                    f"""
                    Cannot identify the type of data query for the dataset
                    """
                )
            
            # 3) Identify target quantity specified
            if target_quantity is not None:
                quantity_details = None

                # a) Handle target quantity exceptions first
                
                # Define handy variables for this sub-step
                nmea_format_details = \
                    sensor_type_source_config.get('nmea_format', 'UNDEFINED')
                quantity_details = \
                    sensor_type_source_config.get(target_quantity, 'UNDEFINED')
                
                is_nmea_psimssb_requested = False
                nmea_psimssb_unit = None
                is_nmea_gpgga_requested = False
                nmea_gpgga_unit = None

                # If target quantity is epoch_timestamp
                # then it is an artificial quantity 
                # used for computing timestamps
                if target_quantity == 'epoch_timestamp':
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Target quantity is {target_quantity}
                            Therefore NOT checking sensor config for
                            matching column label/id
                            Because {target_quantity} 
                            is an artificial quantity
                            """
                        )
                    )
                    sensor_singular_data = \
                        sensor_row_data.at[0, target_quantity]

                # If target quantity is covariance
                elif target_quantity == 'covariance':
                    
                    covariance_details = quantity_details
                    if covariance_details == 'UNDEFINED':
                        raise Exception(
                            f"""
                            Cannot find target quantity {target_quantity}
                            in sensor dataset config
                            """
                        )
                    
                    covariance_col = \
                        covariance_details.get('col', 'UNDEFINED')

                    # Retrieve default covariance from sensor config records
                    # if matching col/id not specified in the sensor dataset config
                    if covariance_col == 'UNDEFINED': 
                        sensor_config_covariance_list = [
                            sensor_config.sensor_covariance.tolist()
                            for sensor_config in self.sensor_config_record
                            if sensor_config.sensor_type == sensor_type
                        ]

                        if len(sensor_config_covariance_list) == 0:
                            raise Exception(
                                f"""
                                Cannot identify default sensor covariance
                                for sensor type {sensor_type}
                                """
                            )

                        else:
                            default_covariance = sensor_config_covariance_list[0]
                            self.get_logger().warn(
                                cl(
                                    f"""
                                    FormattedDataConverter::get_sensor_data -
                                    Target quantity is {target_quantity}
                                    This quantity col is {covariance_col}
                                    Therefore, using a default covariance value
                                    for the sensor type {sensor_type} of:
                                    {default_covariance}
                                    """
                                )
                            )
                            sensor_singular_data = default_covariance
                    
                    else:
                        raise Exception(
                            f"""
                            Cannot retrieve covariance from dataset yet
                            because NOT IMPLEMENTED
                            """
                        )

                # If target quantity is latitude
                # and the NMEA format is specified too
                elif (target_quantity == 'latitude' 
                    and nmea_format_details != 'UNDEFINED'):
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Target quantity is {target_quantity}
                            and nmea format details are:
                            {nmea_format_details}
                            """
                        )
                    )
                    
                    nmea_format_col = \
                        nmea_format_details.get('col', 'UNDEFINED')
                    if nmea_format_col == 'UNDEFINED':
                        raise Exception(f"Cannot find column for NMEA format")
                    nmea_format = sensor_row_data.iat[0, nmea_format_col - 1]
                    nmea_format_trimmed = nmea_format.strip('$')
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Extracted NMEA format {nmea_format_trimmed}
                            from sensor row data
                            """
                        )
                    )
                    # If the NMEA format is GPGGA
                    if nmea_format_trimmed == 'GPGGA':
                        latitude_direction = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'latitude_direction'
                        )
                        latitude = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'latitude',
                            float,
                        )
                        if latitude_direction == 'N':
                            sensor_singular_data = latitude
                        elif latitude_direction == 'S':
                            sensor_singular_data = latitude
                        else:
                            raise Exception(f"Cannot recognise lat dir {latitude_direction}")
                        
                    # If the NMEA format is PSIMSSB
                    elif nmea_format_trimmed == 'PSIMSSB': 
                        is_nmea_psimssb_requested = True
                        coordinate_system = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'coordinate_system'
                        )
                        if coordinate_system == 'R':
                            nmea_psimssb_unit = 'RAD'
                        else:
                            raise Exception(f"Cannot recognise coord syst {coordinate_system}")
                        orientation = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'orientation'
                        )
                        if orientation == 'N':
                            latitude =  self.get_sensor_data_from_row_data(
                                sensor_row_data,
                                sensor_type_source_config,
                                'x_coordinate',
                                float,
                            )
                            sensor_singular_data = latitude
                        else:
                            raise Exception(f"Cannot recognise orient {orientation}")

                    else:
                        raise Exception(f"NMEA format {nmea_format_trimmed} NOT supported")
                
                # If target quantity is longitude
                # and the NMEA format is specified too
                elif (target_quantity == 'longitude' 
                    and nmea_format_details != 'UNDEFINED'):
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Target quantity is {target_quantity}
                            and nmea format details are:
                            {nmea_format_details}
                            """
                        )
                    )
                    
                    nmea_format_col = \
                        nmea_format_details.get('col', 'UNDEFINED')
                    if nmea_format_col == 'UNDEFINED':
                        raise Exception(f"Cannot find column for NMEA format")
                    nmea_format = sensor_row_data.iat[0, nmea_format_col - 1]
                    nmea_format_trimmed = nmea_format.strip('$')
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Extracted NMEA format {nmea_format_trimmed}
                            from sensor row data
                            """
                        )
                    )
                    # If the NMEA format is GPGGA
                    if nmea_format_trimmed == 'GPGGA':
                        longitude_direction = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'longitude_direction'
                        )
                        longitude = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'longitude',
                            float,
                        )
                        if longitude_direction == 'E':
                            sensor_singular_data = longitude
                        elif longitude_direction == 'W':
                            sensor_singular_data = -longitude
                        else:
                            raise Exception(f"Cannot recognise lon dir {longitude_direction}")
                        
                    # If the NMEA format is PSIMSSB
                    elif nmea_format_trimmed == 'PSIMSSB': 
                        is_nmea_psimssb_requested = True
                        coordinate_system = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'coordinate_system'
                        )
                        if coordinate_system == 'R':
                            nmea_psimssb_unit = 'RAD'
                        else:
                            raise Exception(f"Cannot recognise coord syst {coordinate_system}")
                        orientation = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'orientation'
                        )
                        if orientation == 'N':
                            longitude =  self.get_sensor_data_from_row_data(
                                sensor_row_data,
                                sensor_type_source_config,
                                'y_coordinate',
                                float,
                            )
                            sensor_singular_data = longitude
                        else:
                            raise Exception(f"Cannot recognise orient {orientation}")

                    else:
                        raise Exception(f"NMEA format {nmea_format_trimmed} NOT supported")
                
                # If target quantity is depth
                # and the NMEA format is specified too
                # and depth cannot be found
                # it might be specified under altitude
                elif (target_quantity == 'depth' 
                    and nmea_format_details != 'UNDEFINED'
                    and quantity_details == 'UNDEFINED'):
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Target quantity is {target_quantity}
                            and nmea format details are:
                            {nmea_format_details}
                            """
                        )
                    )
                    
                    nmea_format_col = \
                        nmea_format_details.get('col', 'UNDEFINED')
                    if nmea_format_col == 'UNDEFINED':
                        raise Exception(f"Cannot find column for NMEA format")
                    nmea_format = sensor_row_data.iat[0, nmea_format_col - 1]
                    nmea_format_trimmed = nmea_format.strip('$')
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Extracted NMEA format {nmea_format_trimmed}
                            from sensor row data
                            """
                        )
                    )
                    # If the NMEA format is GPGGA
                    if nmea_format_trimmed == 'GPGGA':
                        altitude = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'altitude',
                            float,
                        )
                        altitude_unit = self.get_sensor_data_from_row_data(
                            sensor_row_data,
                            sensor_type_source_config,
                            'altitude_unit'
                        )
                        is_nmea_gpgga_requested = True
                        sensor_singular_data = -altitude
                        nmea_gpgga_unit = altitude_unit
                    
                    else:
                        raise Exception(f"NMEA format {nmea_format_trimmed} NOT supported")

                # b) Find matching column labels/ids
                # for standard target quantities
                # from relevant sensor dataset config
                
                else: 
                    # In case the target quantity cannot be found
                    # in the sensor dataset config
                    if quantity_details == 'UNDEFINED':
                        raise Exception(
                            f"""
                            Cannot find {target_quantity}
                            in the sensor dataset config
                            Check your config.
                            """
                        )
                    
                    # In case the quantity requested is found
                    # in the sensor dataset config
                    else:
                        quantity_col = quantity_details.get('col', 'UNDEFINED')
                        
                        # In case the column for the target quantity
                        # is valid in the config
                        # (specified as an int)
                        if isinstance(quantity_col, int):

                            # DEV WARNING: columns in tables start from 1 from a user perspective / config
                            # but in python, they start from 0
                            sensor_singular_data = \
                                sensor_row_data.iat[0, quantity_col - 1]

                        # In case the column for the target quantity
                        # is not specified correctly in the config
                        else:
                            raise Exception(
                                f"""
                                Cannot find matching label/id
                                for {target_quantity} in sensor dataset config
                                """
                            )
                    
                # 4) Identify target unit specified
                if target_unit is not None:
                    
                    # a) Handle special cases
                    # for unit conversion request

                    # If target quantity is a special case
                    # related to NMEA string PSIMSSB
                    if is_nmea_psimssb_requested:
                        self.get_logger().debug(
                            cl(
                                f"""
                                FormattedDataConverter::get_sensor_data -
                                Identified request to convert quantity {target_quantity}
                                from {nmea_psimssb_unit} to {target_unit}
                                with special case PSIMSSB
                                """
                            )
                        )
                        converted_data = self.convert_value_unit(
                                sensor_singular_data,
                                nmea_psimssb_unit, 
                                target_unit,
                        )
                        sensor_singular_data = converted_data

                    # If target quantity is a special case
                    # related to NMEA string GPGGA
                    elif is_nmea_gpgga_requested:
                        self.get_logger().debug(
                            cl(
                                f"""
                                FormattedDataConverter::get_sensor_data -
                                Identified request to convert quantity {target_quantity}
                                from {nmea_gpgga_unit} to {target_unit}
                                with special case GPGGA
                                """
                            )
                        )
                        converted_data = self.convert_value_unit(
                                sensor_singular_data,
                                nmea_gpgga_unit, 
                                target_unit,
                        )
                        sensor_singular_data = converted_data

                    # b) Handle standard cases
                    # for unit conversion request
                    else:
                        quantity_unit = quantity_details.get('unit', 'UNDEFINED')
                        self.get_logger().debug(
                            cl(
                                f"""
                                FormattedDataConverter::get_sensor_data -
                                Identified request to convert quantity {target_quantity}
                                from {quantity_unit} to {target_unit}
                                """
                            )
                        )
                        # In case the unit for target quantity
                        # cannot be found
                        if quantity_unit == 'UNDEFINED':
                            raise Exception(f"Cannot find unit for {target_quantity}")

                        # In case the unit for target quantity
                        # is found
                        else:
                            converted_data = self.convert_value_unit(
                                    sensor_singular_data,
                                    quantity_unit, 
                                    target_unit,
                            )
                            sensor_singular_data = converted_data
                
                else:
                    self.get_logger().warn(
                        cl(
                            f"""
                            FormattedDataConverter::get_sensor_data -
                            Cannot identify target unit requested
                            Default unit set in the dataset config is returned
                            """
                        )
                    )
                    pass
                
                # 5) Return singular data
                self.get_logger().debug(
                    cl(
                        f"""
                        FormattedDataConverter::get_sensor_data -
                        Retrieved sensor singular data for {target_quantity}:
                        {sensor_singular_data}
                        """
                    )
                )
                    
                return sensor_singular_data

            else:
                raise Exception(
                    f"""
                    Cannot identify target singular quantity
                    to be retrieved in the dataset
                    Specify a target singular quantity
                    """
                )
     
        except Exception as e:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::get_sensor_data -
                    Cannot get sensor data
                    Exception: {e}
                    """
                )
            )
            return sensor_singular_data

    def get_sensor_data_from_row_data(
            self,
            sensor_row_data,
            sensor_type_source_config,
            target_quantity,
            target_type = None):
        # self.get_logger().debug(
        #     cl(
        #         f"""
        #         FormattedDataConverter::get_sensor_data_from_row_data -
        #         Received request to get sensor data with:
        #         * Sensor type source config = {sensor_type_source_config}
        #         * Target quantity = {target_quantity}
        #         """
        #     )
        # )
        quantity_details = \
            sensor_type_source_config.get(target_quantity, 'UNDEFINED')
        quantity_col = \
            quantity_details.get('col', 'UNDEFINED')
        quantity_value = sensor_row_data.iat[0, quantity_col - 1]
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::get_sensor_data_from_row_data -
                Retrieved quantity value for {target_quantity}:
                {quantity_value}
                """
            )
        )

        # Return with requested target type
        if target_type is None:
            return quantity_value
        else:
            return target_type(quantity_value)

    def load_sensor_data(self, sensor_type):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::load_sensor_data -
                Received request to load datasets for sensor type:
                {sensor_type}
                """
            )
        )
        
        # -----------------------------
        # The aim of dataset load into a record variable
        # is to make access to data quicker at runtime
        # while spending the time to read the dataset "offline"
        # and therefore only lightly impact 
        # the functions of the node

        # This efficient method of loading the datasets
        # for a sensor type and sensor source comes from the requirements:
        # * Mission datatest is relatively small per sensor type (approx. max 1MB)
        # * Number of mission datasets is limited (approx. max 100)
        # * RAM available is around 2GB min
        # This way, the RAM available will cover the memory needs (max 1GB)

        # For maximum efficiency, the data is loaded into pandas data frames
        # and stored in a dictionary containing metadata for easier access
        # Each set of data is then stored in a record list
        # -----------------------------

        # Check whether the dataset has already been loaded
        is_dataset_loaded = False
        for sensor_dataset in self.sensor_dataset_record:
            if sensor_dataset.get('sensor_type') == sensor_type:
                is_dataset_loaded = True
                break
        
        # If dataset has not been loaded
        if not is_dataset_loaded:
            try:
                # Read through the dataset config to identify
                # the datasets related to sensor type
                sensor_type_source_data = pd.DataFrame()
                sensor_type_dataset_details = self.sensor_dataset_config.get(sensor_type)
                for source in sensor_type_dataset_details:
                    sensor_type_source_dataset_details = sensor_type_dataset_details.get(source) 
                    
                    # Identify the extension
                    file_extension = sensor_type_source_dataset_details.get('file_type')
                    
                    # Identify the dataset file path
                    dataset_filepath = join(
                        self.sensor_data_folder,
                        sensor_type_source_dataset_details.get('file_basename') \
                        + '.' \
                        + file_extension
                    )
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::load_sensor_data -
                            Found dataset file path for {sensor_type} and {source}: 
                            {dataset_filepath}
                            """
                        )
                    )

                    # Load data from file into pandas dataframes
                    metadata_row_total = sensor_type_source_dataset_details.get('metadata_row_total', 'UNDEFINED')
                    if metadata_row_total == 'UNDEFINED':
                        metadata_row_total = 0
                    sensor_type_source_data = load_data_from_file(
                        dataset_file_path=dataset_filepath,
                        metadata_rows_to_skip=metadata_row_total,
                    )
                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::load_sensor_data -
                            Loaded dataset located at {dataset_filepath}.
                            The first 5 rows are:
                            {sensor_type_source_data.head()}
                            """
                        )
                    ) 

                    # Add epoch timestamp data for easier time-based data retrieval
                    updated_sensor_type_source_data = \
                            self.add_epoch_timestamp_data_to_df(
                                    sensor_type_source_data,
                                    sensor_type,
                                    source,
                    )

                    # Add metadata around loaded data for easier identification
                    sensor_type_source_data_curated = dict()
                    sensor_type_source_data_curated['sensor_type'] = sensor_type
                    sensor_type_source_data_curated['sensor_source'] = source
                    sensor_type_source_data_curated['sensor_data'] = updated_sensor_type_source_data

                    # Add loaded data to the data record
                    self.sensor_dataset_record.append(sensor_type_source_data_curated)

                    self.get_logger().debug(
                        cl(
                            f"""
                            FormattedDataConverter::load_sensor_data -
                            Stored dataset from {dataset_filepath}
                            """
                        )
                    )

            except Exception as e:
                self.get_logger().warn(
                    cl(
                        f"""
                        FormattedDataConverter::load_sensor_data -
                        Cannot load datasets for sensor type {sensor_type}
                        Exception: {e}
                        """
                    )
                )
        
        # If dataset has already been loaded
        else:
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::load_sensor_data -
                    Dataset for sensor type {sensor_type} already loaded
                    NOT loading the dataset again
                    """
                )
            )

        # --------------------------------------
        # Implementation improvement note:
        # Even if the sensor / source is disabled, the dataset is
        # not deleted to save time reloading it
        # In a future implementation, the dataset related to 
        # a sensor / source can be deleted to save space

        # Alternative implementation suggestion:
        # Access data from file whenever requested
        # rather than store all the data at once.
        # The aim is to reduce memory needed to access the data
        # for large datasets, while ensuring appropriate access
        # speed to the data
        # --------------------------------------

    def add_epoch_timestamp_data_to_df(self, df, sensor_type, sensor_source):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::add_epoch_timestamp_data_to_df -
                Received request to add epoch timestamp data to
                dataset related to:
                sensor type = {sensor_type} and sensor source = {sensor_source}
                """
            )
        )
        
        try:
            
            # Make sure that the epoch_timestamp column does not exist already
            if 'epoch_timestamp' in df.columns:
                raise Exception(f"Column `epoch_timestamp` already exists in df")

            # Retrieve relevant sensor config to perform df operation
            sensor_type_dataset_details = self.sensor_dataset_config.get(sensor_type)
            sensor_type_source_dataset_details = sensor_type_dataset_details.get(sensor_source)
            
            # DEV WARNING: columns in tables start from 1 from a user perspective / config
            # but in python, they start from 0
            
            # Collect timestamp details
            timestamp_details = sensor_type_source_dataset_details.get('timestamp', 'UNDEFINED')
            if timestamp_details == 'UNDEFINED':
                raise Exception(f"Cannot find `timestamp` reference in dataset config")
            timestamp_col_from_config = timestamp_details.get('col', 'UNDEFINED')
            if timestamp_col_from_config == 'UNDEFINED':
                raise Exception(f"Cannot find `col` for `timestamp` in dataset config")
            timestamp_col = timestamp_col_from_config - 1
            timestamp_unit = timestamp_details.get('unit', 'UNDEFINED')
            if timestamp_unit == 'UNDEFINED':
                raise Exception(f"Cannot find `unit` for `timestamp` in dataset config")
            timestamp_timezone = timestamp_details.get('time_standard', 'UNDEFINED')
            if timestamp_timezone == 'UNDEFINED':
                raise Exception(f"Cannot find `time_standard` for `timestamp` in dataset config")

            # Check whether the timestamp information is enough to determine the Unix UTC timestamp
            if timestamp_unit ==  'UNIX':
                if timestamp_timezone == 'UTC':
                    # Copy over the timestamp column since in the expected format 
                    df['epoch_timestamp'] = df.iloc[:, timestamp_col]

                else:
                    raise Exception(f"The `time_standard` {timestamp_timezone} for `timestamp` is NOT supported")
            
            # Otherwise perform necessary conversions which involve date details
            else:

                # Collect date details
                date_details = sensor_type_source_dataset_details.get('date', 'UNDEFINED')
                if date_details == 'UNDEFINED':
                    raise Exception(f"Cannot find `date` reference in dataset config")
                date_col_from_config = date_details.get('col', 'UNDEFINED')
                if date_col_from_config == 'UNDEFINED':
                    raise Exception(f"Cannot find `col` for `date` in dataset config")
                date_col = date_col_from_config - 1
                date_unit = date_details.get('unit', 'UNDEFINED')
                if date_unit == 'UNDEFINED':
                    raise Exception(f"Cannot find `unit` for `date` in dataset config")
                
                # Express date and timestamp into predictable format
                # for python package
                date_format = '%Y%m%d'
                timestamp_format = '%H%M%S.%f'
                timezone_format = timezone.utc
                date_timestamp_format = date_format + '-' + timestamp_format
               
                input_date_format_id = None
                ## Date cases
                ### Case (date ID = 1): YYYYMMDD
                if date_unit == 'YYYYMMDD':
                    input_date_format_id = 1
                else:
                    raise Exception(f"The `unit` {date_unit} for `date` is NOT supported")

                input_timestamp_format_id = None
                ## Timestamp cases
                ## Case (timestamp ID = 1): HHMMSS.SS
                if timestamp_unit == 'HHMMSS.SS':
                    input_timestamp_format_id = 1
                else:
                    raise Exception(f"The `unit` {timestamp_unit} for `timestamp` is NOT supported")

                input_timezone_format_id = None
                ## Timezone cases
                ### Case (timezone ID): UTC
                if timestamp_timezone == 'UTC':
                    input_timezone_format_id = 1
                else:
                    raise Exception(f"The `time_standard` {timestamp_timezone} for `timestamp` is NOT supported")

                # Compute epoch_timestamp column from date and timestamp retrieved 
                if (input_date_format_id == 1 
                        and input_timestamp_format_id == 1
                        and input_timezone_format_id == 1):
                    ## Tip 1: list comprehension is used to go through the dataframe faster
                    ## Tip 2: ensure that the inputs are not NaN (x == x)
                    df['epoch_timestamp'] = [
                        datetime.strptime(
                            str(round(x)) + '-' + str(round(float(y), 6)), 
                            date_timestamp_format
                        ).replace(tzinfo=timezone_format).timestamp()
                        if (x == x and y == y)
                        else float("NaN")
                        for (x,y) in zip(
                            df.iloc[:, date_col], 
                            df.iloc[:, timestamp_col],
                        )
                    ]
            
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::add_epoch_timestamp_data_to_df -
                    Successfully added epoch timestamp data to
                    the dataset related to
                    sensor type = {sensor_type} and sensor source = {sensor_source}
                    The first 5 rows are:
                    {df.head()}
                    """
                )
            )
 
        except Exception as e:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::add_epoch_timestamp_data_to_df -
                    Cannot add epoch timestamp data to df with:
                    sensor type = {sensor_type} and sensor source = {sensor_source}
                    Exception: {e}
                    """
                )
            )

        return df 

    def convert_value_unit(
            self,
            value_to_convert,
            from_unit, 
            to_unit):
        
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::convert_value_unit -
                Received request to convert {value_to_convert}
                from {from_unit} to {to_unit}
                """
            )
        )
        
        converted_value = value_to_convert

        try:

            # Case from_unit = to_unit
            if from_unit == to_unit:
                self.get_logger().debug(
                    cl(
                        f"""
                        FormattedDataConverter::convert_value_unit -
                        from and to units are similar
                        NOT converting the input value
                        """
                    )
                )

            # Case 'DDMM.MM'
            elif from_unit == 'DDMM.MM':
                # to 'DD.DDDD'
                if to_unit == 'DD.DDDD':
                    value_sign = sign(value_to_convert)
                    abs_value = abs(value_to_convert)
                    # Tip: conserve leading zeros
                    abs_value_str = f'{abs_value:0>14.9f}' 
                    dd = int(str(abs_value)[:2])
                    mm = float(str(abs_value)[2:])
                    converted_value = \
                        value_sign * (dd + mm/60)
            
            # Case 'DDDMM.MM'
            elif from_unit == 'DDDMM.MM':
                # to 'DD.DDDD'
                if to_unit == 'DD.DDDD':
                    value_sign = sign(value_to_convert)
                    abs_value = abs(value_to_convert)
                    # Tip: conserve leading zeros
                    abs_value_str = f'{abs_value:0>15.9f}' 
                    dd = int(abs_value_str[:3])
                    mm = float(str(abs_value)[3:])
                    converted_value = \
                        value_sign * (dd + mm/60)

            # Case 'RAD'
            elif from_unit == 'RAD':
                # to 'DEG' or 'DD.DDDD'
                if (to_unit == 'DEG'
                        or to_unit == 'DD.DDDD'):
                    converted_value = \
                        wrap_to_pi(value_to_convert) * 180/pi

            # Case UNDEFINED
            else:
                raise Exception(f"Conversion NOT supported")

            # Return converted value
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::convert_value_unit -
                    Converted {value_to_convert}
                    from {from_unit} to {to_unit}:
                    {converted_value}
                    """
                )
            )
            return converted_value

        except Exception as e:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::convert_value_unit -
                    Cannot convert {value_to_convert}
                    from {from_unit} to {to_unit}
                    Exception: {e}
                    """
                )
            )
            return converted_value 


    def get_dataset_startup_config(self):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::get_dataset_startup_config -
                Received request to get dataset startup config
                """
            )
        )

        try:
            # Read dataset config
            dataset_config = self.param_dict.get('sensor_datasets')
        
            self.get_logger().debug(
                cl(
                    f"""
                    FormattedDataConverter::get_dataset_startup_config -
                    Collected startup sensor dataset config:
                    {dataset_config}
                    """
                )
            )

            return dataset_config
        
        except Exception as e:
            self.get_logger().warn(
                cl(
                    f"""
                    FormattedDataConverter::get_dataset_startup_config -
                    Can't read startup sensor dataset config.
                    Verify your config.
                    Exception: {e}
                    """
                )
            )

        return None

    def generate_automatic_aiding_data_publisher(self, sensor_type, sensor_source):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::generate_automatic_aiding_data_publisher -
                Received request to generate aiding data publisher
                for sensor type {sensor_type}
                and sensor source {sensor_source}
                """
            )
        )

        # Search for aiding data publisher in the records
        # to ensure it does not exist already
        aiding_data_publisher_search_list = [
            dict_i 
            for dict_i in self.aiding_data_publisher_record
            if (dict_i.get('sensor_type', None) == sensor_type
                and dict_i.get('sensor_source', None) == sensor_source
            )
        ]

        # For the case where aiding data publisher
        # does not exist in the records
        if not aiding_data_publisher_search_list:
            
            # Create a specific publisher for the specified sensor type
            aiding_data_sensor_type_topic = \
                '/' + sensor_type.lower() + '_' + sensor_source.lower() + '_aiding_data'
            ## For the case where sensor source is POSITION
            if sensor_source == 'POSITION_SOURCE':
                aiding_data_sensor_type_publisher = self.create_publisher(
                    PoseWithCovarianceStamped, 
                    aiding_data_sensor_type_topic, 
                    self.default_node_pub_queue_size,
                )
            else:
                self.get_logger().warn(
                    cl(
                        f"""
                        FormattedDataConverter::generate_automatic_aiding_data_publisher -
                        Cannot generate automatic aiding data publisher
                        for sensor source {sensor_source}
                        because NOT implemented yet
                        """
                    )
                )

            # Add specific publisher config to the aiding data publisher record
            aiding_data_sensor_type_dict = {
                'sensor_type': sensor_type,
                'sensor_source': sensor_source,
                'publisher_topic': aiding_data_sensor_type_topic,
                'publisher_object': aiding_data_sensor_type_publisher,
            }
            self.aiding_data_publisher_record.append(aiding_data_sensor_type_dict)

    def on_sensor_config_update_received(self, msg):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::on_sensor_config_update_received -
                Received data to update the sensor config:
                {msg}
                """
            )
        )
        
        # Send request to udpate module config with received data
        self.update_sensor_config(msg.sensor_config)

    def on_module_config_update_received(self, msg):
        self.get_logger().debug(
            cl(
                f"""
                FormattedDataConverter::on_module_config_update_received -
                Received data to update the module config:
                {msg}
                """
            )
        )

        # Send request to udpate module config with received data
        self.update_module_config(msg.module_config)
 

def main():

    # Initialise rclpy library
    rclpy.init()
   
    # Define default module variables for practicality
    module_default_package_name = "task79_data_converter"
    module_default_node_name = "formatted_data_conversion_node"
    module_default_node_logging_level = rclpy.logging.LoggingSeverity.DEBUG
    module_default_node_pub_queue_size = 100
    module_default_node_sub_queue_size = 10
    module_default_package_path = get_package_share_directory(module_default_package_name)
    module_default_node_spin_frequency_hz = 1.0
    module_default_sensor_data_folder = getcwd()
    module_default_is_aiding_data_grouped = False
    module_default_sensor_covariance_length = 9
    module_default_sensor_covariance = [0.0 for x in range(module_default_sensor_covariance_length)]
    module_default_is_sensor_enabled = False
    module_default_sensor_sampling_proportion = 0.55
    module_default_environment = '2D'
    module_default_depth = 0.0
    module_default_timestamp_delta_acceptance = 1.0
  
    # Instantiate node
    node = FormattedDataConverter(
        module_default_node_name,
        module_default_node_logging_level,
        module_default_node_pub_queue_size,
        module_default_node_sub_queue_size,
        module_default_package_path,
        module_default_node_spin_frequency_hz,
        module_default_sensor_data_folder,
        module_default_is_aiding_data_grouped,
        module_default_sensor_covariance_length,
        module_default_sensor_covariance,
        module_default_sensor_sampling_proportion,
        module_default_environment,
        module_default_depth,
        module_default_timestamp_delta_acceptance,
    )
     
    node.get_logger().info(
        cl(
            f"""
            formatted_data_conversion_node::main -
            Defined a set of module variables for practicality:
            * Default package name = {module_default_package_name}
            * Default node name = {module_default_node_name}
            * Default node logging level = {module_default_node_logging_level}
            * Default node pub queue size = {module_default_node_pub_queue_size}
            * Default node sub queue size = {module_default_node_sub_queue_size}
            * Default package path = {module_default_package_path}
            * Default node spin frequency Hz = {module_default_node_spin_frequency_hz}
            * Default sensor data folder = {module_default_sensor_data_folder}
            * Default is aiding data grouped = {module_default_is_aiding_data_grouped}
            * Default sensor covariance length = {module_default_sensor_covariance_length}
            * Default sensor covariance = {module_default_sensor_covariance}
            * Default is sensor enabled = {module_default_is_sensor_enabled}
            * Default sensor sampling proportion = {module_default_sensor_sampling_proportion}
            * Default module environment = {module_default_environment}
            * Default module depth = {module_default_depth}
            * Default timestamp delta acceptance = {module_default_timestamp_delta_acceptance}
            """
        )
    )
    
    node.get_logger().info(
        cl(
            f"""
            formatted_data_conversion_node::main -
            Instantiated node with name: {module_default_node_name}
            """
        )
    )

    node.get_logger().info(
        cl(
            f"""
            formatted_data_conversion_node::main -
            Starting ros spin loop
            """
        )
    )

    # Run spin(node) in a thread
    # and make thread daemon so there is no need to join it to exit
    thread = threading.Thread(
        target=rclpy.spin, 
        args=(node, ), 
        daemon=True,
    )
    thread.start()

    # Set a node rate
    module_node_spin_frequency_hz = module_default_node_spin_frequency_hz
    rate = node.create_rate(module_node_spin_frequency_hz)

    # Give the node some time
    # to setup pubs and subs
    loop_times = 2
    for tick in range(loop_times):
        rate.sleep()

    while rclpy.ok():

        # Run the Formatted Data Converter
        node.run()

        # Update the node spin frequency at runtime
        input_node_spin_frequency_hz = \
            node.node_spin_frequency_hz
        if module_node_spin_frequency_hz != input_node_spin_frequency_hz:
            node.get_logger().info(
                cl(
                    f"""
                    formatted_data_conversion_node::main -
                    Updating ros spin rate frequency 
                    to: {input_node_spin_frequency_hz} Hz
                    """
                )
            )
            module_node_spin_frequency_hz = input_node_spin_frequency_hz
            rate = node.create_rate(module_node_spin_frequency_hz)
        
        rate.sleep()
    
    # Destroy the node gracefully
    node.destroy_node()
    rclpy.shutdown()

    
if __name__ == '__main__':
    main()

