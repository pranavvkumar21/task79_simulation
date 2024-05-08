#!/usr/bin/env python3

# ------------------
# Author: Achille Martin - achille.martin@noc.ac.uk
# Date: 26/04/2024
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

from inspect import cleandoc as cl

import os
import sys

importer_folder = os.path.dirname(
	os.path.realpath(__file__)
)
parent_of_importer_folder = os.path.join(
	importer_folder, 
	os.path.pardir,
)
parent_of_parent_of_importer_folder = os.path.join(
	parent_of_importer_folder, 
	os.path.pardir,
)

sys.path.append(parent_of_importer_folder)
sys.path.append(parent_of_parent_of_importer_folder)

from tools.format_management.dict_format_converter import (
    convert_dotted_dict_to_std_dict,
)

def collect_ros_param(node):
    """
    Collect ROS params associated to a node into a dictionary 
  
    Parameters
    ----------
    node: rclpy.node.Node
        ROS2 node object
  
    Returns
    -------
    dict
        Dictionary of ROS params
    """
    
    node.get_logger().debug(
        cl(
            f"""
            {node.__class__.__name__}::collect_ros_param -
            Received request to collect ROS parameters
            """
        )
    )

    output_dict = dict()

    # Collect all ROS params associated to the node
    ros_param_dict = node._parameters
    # node.get_logger().debug(
    #     cl(
    #         f"""
    #         {node.__class__.__name__}::collect_ros_param -
    #         Collected raw ROS parameters:
    #         {ros_param_dict}
    #         """
    #     )
    # )

    # Extract values from ROS params
    # and link them to dotted keys
    dotted_param_dict = {
        key: ros_param_dict.get(key).value for key in ros_param_dict
    }

    # Convert the dotted keys
    # into standard keys
    std_param_dict = convert_dotted_dict_to_std_dict(
        dotted_param_dict
    )
    node.get_logger().debug(
        cl(
            f"""
            {node.__class__.__name__}::collect_ros_param -
            Gathered raw ROS params
            into a standard dictionary structure:
            {std_param_dict}
            """
        )
    )

    output_dict = std_param_dict

    return output_dict

