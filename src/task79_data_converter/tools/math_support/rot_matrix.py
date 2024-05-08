#!/usr/bin/env python3

# ------------------
# Author: Achille Martin - achille.martin@noc.ac.uk
# Date: 25/03/2024
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

import math
import numpy as np

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

from tools.unit_conversion.unit_converter import unit_converter


def rot_matrix(axis, angle_deg, rotation_type='3D'):
    """
    Get the rotation array 
    for a specified angle around a specified axis
  
    Parameters
    ----------
    axis: str
        The axis around which the rotation occurs
        'x', 'y' or 'z'
  
    angle_deg: float
        Angle of rotation around axis in degrees
        The sign of the angle depends on the right-hand rule rotation

    rotation_type: str
        The type of array returned
        '2D' is implemented
        '3D' is implemented

    Returns
    -------
    Numpy array
        Array describing the rotation desired
    """
    
    rot_array = None
    
    angle_rad = unit_converter(angle_deg, 'DEG', 'RAD') 
    
    if rotation_type == '2D':
        rot_array = np.array(
            [
                [math.cos(angle_rad), -math.sin(angle_rad)],
                [math.sin(angle_rad), math.cos(angle_rad)],
            ]
        )
    
    elif rotation_type == '3D':
        if axis == 'x':
            rot_array = np.array(
                [
                    [1, 0, 0],
                    [0, math.cos(angle_rad), -math.sin(angle_rad)],
                    [0, math.sin(angle_rad), math.cos(angle_rad)],
                ]
            )
        
        elif axis == 'y':
            rot_array = np.array(
                [
                    [math.cos(angle_rad), 0, math.sin(angle_rad)],
                    [0, 1, 0],
                    [-math.sin(angle_rad), 0, math.cos(angle_rad)],
                ]
            )

        elif axis == 'z':
            rot_array = np.array(
                [
                    [math.cos(angle_rad), -math.sin(angle_rad), 0],
                    [math.sin(angle_rad), math.cos(angle_rad), 0],
                    [0, 0, 1],
                ]
            )

        else:
            raise Exception(
                f"""
                Axis {axis} is NOT recognised
                Try with 'x', 'y' or 'z'
                """
            )

    else:
        raise Exception(
            f"""
            Array type {rotation_type} NOT handled yet
            """
        )

    return rot_array
