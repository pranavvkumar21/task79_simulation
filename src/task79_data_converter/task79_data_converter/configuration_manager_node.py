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
from rclpy.qos import (
    QoSProfile,
    HistoryPolicy,
    QoSDurabilityPolicy,
)

import threading
from inspect import cleandoc as cl
from os import (
    getcwd,
)
from os.path import (
    join,
    realpath,
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

from task79_interfaces.msg import (
    SensorConfig,
    SensorConfigStamped, 
    ModuleConfig,
    ModuleConfigStamped,
)

from tools.ros2_management.ros2_extended_functionality_helper import (
    collect_ros_param,
)


class ConfigurationManager(Node):

    def __init__(
            self,
            default_node_name,
            default_node_logging_level,
            default_node_pub_queue_size,
            default_node_spin_frequency_hz,
            default_sensor_data_folder,
            default_is_aiding_data_grouped,
            default_sensor_type,
            default_sensor_covariance,
            default_is_sensor_enabled):
        super().__init__(
            default_node_name,
            allow_undeclared_parameters=True,
            automatically_declare_parameters_from_overrides=True,
        )
      
        # Define and initialise handy attributes
        self.default_node_logging_level = default_node_logging_level
        self.default_node_pub_queue_size = default_node_pub_queue_size
        self.default_node_spin_frequency_hz = default_node_spin_frequency_hz
        self.default_sensor_data_folder = default_sensor_data_folder
        self.default_is_aiding_data_grouped = default_is_aiding_data_grouped
        self.default_sensor_type = default_sensor_type
        self.default_sensor_covariance = default_sensor_covariance
        self.default_is_sensor_enabled = default_is_sensor_enabled

        # Set node logging level
        rclpy.logging.set_logger_level(
            self.get_logger().name,
            self.default_node_logging_level,
        )

        # Set up publishers
        ## Create a custome QoS profile
        ## to emulate latch behaviour
        latch_qos_profile = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
	        depth=self.default_node_pub_queue_size,
	        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.sensor_config_pub = self.create_publisher(
	        SensorConfigStamped, 
	        '/sensor_config_update', 
            qos_profile=latch_qos_profile,
	    )
        self.module_config_pub = self.create_publisher(
	        ModuleConfigStamped, 
	        '/module_config_update', 
            qos_profile=latch_qos_profile,
	    )
        
        self.get_logger().info(
            cl(
                f"""
                ConfigurationManager::__init__ -
                Created the publishers
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
        # Module and Sensor config
        self.param_dict = collect_ros_param(self)

    def run(self):
        self.get_logger().debug(
            cl(
                f"""
                ConfigurationManager::run -
                Received request to run the Configuration Manager
                """
            )
        )
 
        # Get module startup config
        module_startup_config = self.get_module_startup_config()

        # Transmit module config (to the FormattedDataConverter)
        self.transmit_module_config(module_startup_config)

        # Get sensor startup config
        sensor_startup_config = self.get_sensor_startup_config()

        # Transmit individual sensor config (to the FormattedDataConverter)
        for sensor_id in sensor_startup_config:
            sensor_id_config = sensor_startup_config.get(sensor_id)
            sensor_config = SensorConfig()
            sensor_config.sensor_type = \
                sensor_id_config.get('sensor_type', self.default_sensor_type)
            sensor_config.sensor_covariance = \
                sensor_id_config.get('sensor_covariance', self.default_sensor_covariance)
            sensor_config.is_sensor_enabled = \
                sensor_id_config.get('is_sensor_enabled', self.default_is_sensor_enabled)
            self.transmit_sensor_config(sensor_config)

    def get_module_startup_config(self):
        self.get_logger().debug(
            cl(
                f"""
                ConfigurationManager::get_module_startup_config -
                Received request to get module startup config
                """
            )
        )
        try:
            startup_module_config = self.param_dict.get(
                'module',
                None,
            )
            
            self.get_logger().debug(
                cl(
                    f"""
                    ConfigurationManager::get_module_startup_config -
                    Extracted module config from startup config:
                    {startup_module_config}
                    """
                )
            )
            
            # Read module config
            module_config = ModuleConfig()
            module_config.node_spin_frequency_hz = \
                startup_module_config.get(
                    'node_spin_frequency_hz', 
                    self.default_node_spin_frequency_hz,
                )
            module_config.sensor_data_folder = \
                startup_module_config.get(
                    'sensor_data_folder', 
                    self.default_sensor_data_folder,
                )
            module_config.is_aiding_data_grouped = \
                startup_module_config.get(
                    'is_aiding_data_grouped',
                    self.default_is_aiding_data_grouped,
                )
            
            return module_config
            
        except Exception as e:
            self.get_logger().warn(
                cl(
                    f"""
                    ConfigurationManager::get_module_startup_config -
                    Can't read startup module config.
                    Verify your config.
                    Exception: {e}
                    """
                )
            )
        
        return None
            
    def get_sensor_startup_config(self):
        self.get_logger().debug(
            cl(
                f"""
                ConfigurationManager::get_sensor_startup_config -
                Received request to get sensor startup config
                """
            )
        )
        try:
            startup_sensors_config = self.param_dict.get(
                'sensors',
                None,
            )
            
            self.get_logger().debug(
                cl(
                    f"""
                    ConfigurationManager::get_sensor_startup_config -
                    Extracted sensor config from startup config:
                    {startup_sensors_config}
                    """
                )
            )
             
            return startup_sensors_config

        except Exception as e:
            self.get_logger().warn(
                cl(
                    f"""
                    ConfigurationManager::get_sensor_startup_config -
                    Can't read startup sensor config.
                    Verify your config.
                    Exception: {e}
                    """            
                )
            )

        return None

    def transmit_module_config(self, input_config):
        self.get_logger().debug(
            cl(
                f"""
                ConfigurationManager::transmit_module_config -
                Received request to transmit module config:

                {input_config}
                """
            )
        )
        config_stamped = ModuleConfigStamped()
        config_stamped.header.stamp = self.get_clock().now().to_msg()
        config_stamped.module_config = input_config
        self.module_config_pub.publish(config_stamped)
    
    def transmit_sensor_config(self, input_config):
        self.get_logger().debug(
            cl(
                f"""
                ConfigurationManager::transmit_sensor_config -
                Received request to transmit sensor config:
                {input_config}
                """
            )
        )
        config_stamped = SensorConfigStamped()
        config_stamped.header.stamp = self.get_clock().now().to_msg()
        config_stamped.sensor_config = input_config
        self.sensor_config_pub.publish(config_stamped)


def main():

    # Initialise rclpy library
    rclpy.init()
   
    # Define default module variables for practicality
    module_default_package_name = "task79_data_converter"
    module_default_node_name = "configuration_manager_node"
    module_default_node_logging_level = rclpy.logging.LoggingSeverity.DEBUG
    module_default_node_pub_queue_size = 1
    module_default_node_spin_frequency_hz = 1.0
    module_default_sensor_data_folder = getcwd()
    module_default_is_aiding_data_grouped = False
    module_default_sensor_type = ""
    module_default_sensor_covariance_length = 9
    module_default_sensor_covariance = [0.0 for x in range(module_default_sensor_covariance_length)]
    module_default_is_sensor_enabled = False
   
    # Instantiate node
    node = ConfigurationManager(
        module_default_node_name,
        module_default_node_logging_level,
        module_default_node_pub_queue_size,
        module_default_node_spin_frequency_hz,
        module_default_sensor_data_folder,
        module_default_is_aiding_data_grouped,
        module_default_sensor_type,
        module_default_sensor_covariance,
        module_default_is_sensor_enabled,
    )
  
    node.get_logger().info(
        cl(
            f"""
            configuration_manager_node::main -
            Defined a set of default module variables for practicality:
            * Default package name = {module_default_package_name}
            * Default node name = {module_default_node_name}
            * Default node logging level = {module_default_node_logging_level}
            * Default node pub queue size = {module_default_node_pub_queue_size}
            * Default node spin frequency Hz = {module_default_node_spin_frequency_hz}
            * Default sensor data folder = {module_default_sensor_data_folder}
            * Default is aiding data grouped = {module_default_is_aiding_data_grouped}
            * Default sensor type = {module_default_sensor_type}
            * Default sensor covariance length = {module_default_sensor_covariance_length}
            * Default sensor covariance = {module_default_sensor_covariance}
            * Default is sensor enabled = {module_default_is_sensor_enabled}
            """
        )
    )

    node.get_logger().info(
        cl(
            f"""
            configuration_manager_node::main -
            Instantiated node with name: {module_default_node_name}
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

    # Run the Configuration Manager once
    node.run()

    node.get_logger().info(
        cl(
            f"""
            configuration_manager_node::main -
            Starting ros spin loop
            """
        )
    )

    while rclpy.ok():
        rate.sleep()
    
    # Destroy the node gracefully
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

