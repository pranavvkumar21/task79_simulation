#!/usr/bin/env python3

# ------------------
# Author: Achille Martin - achille.martin@noc.ac.uk
# Date: 22/03/2024
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

# This utility script enables the processing 
# of Syrinx raw data
# into body-levelled velocities (SNAME body-fixed frame)

import argparse
import logging as log_tool
import os
import sys
from datetime import datetime
import math
import numpy as np
import pandas as pd

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
from tools.file_management.load_data_from_file import load_data_from_file
from tools.math_support.mean import mean
from tools.math_support.standard_deviation import standard_deviation
from tools.math_support.range_min_max import range_min_max
from tools.math_support.rot_matrix import rot_matrix

# ====================== BEGINNING USER CONFIGURATION ======================

# Configure ocs dataset for processing (USER-defined)
ocs_dataset_dict = {
    'cols_to_extract': [
        'timestamp',
        'Syrinx_raw_water_track_velocity_x',
        'Syrinx_raw_water_track_velocity_y',
        'Syrinx_raw_water_track_velocity_z',
        'DVL_down_WT_valid',
        'Syrinx_raw_bottom_track_velocity_x',
        'Syrinx_raw_bottom_track_velocity_y',
        'Syrinx_raw_bottom_track_velocity_z',
        'DVL_down_BT_valid',
        'Syrinx_aiding_velocity_covariance_00',
        'Syrinx_aiding_velocity_covariance_10',
        'Syrinx_aiding_velocity_covariance_11',
        'Syrinx_aiding_velocity_covariance_20',
        'Syrinx_aiding_velocity_covariance_21',
        'Syrinx_aiding_velocity_covariance_22',
    ],
    'file_extension': 'csv',
}

# Configure additional sensor datasets for processing help (USER-defined)
## INS dataset configuration for attitude information (when sensor mode is 'standalone')
## Note that from a user perspective, columns are numbered from 1
ins_dataset_dict = {
    'file_extension': 'asc',
    'metadata_row_total': 5,
    'attitude_frame': 'NED',
    'roll': {
        'col': 3,
        'unit': 'DEG',
    },
    'pitch': {
        'col': 4,
        'unit': 'DEG',
    },
    'acceptance_stdev_deg': 1,
    'acceptance_max_range_deg': 7.5,
}

# Configure sensor device (USER-defined)
# The sensor device information can be obtained from the OCS dumps (`ocs_configuration.yaml`)
sensor_device_dict = {
    'mode': 'standalone',
    'mounting_position_m': [0, 0, 0],
    'mounting_orientation_deg': [0, 0, 45],
    'mounting_orientation_type': 'DOWN',
    'mounting_orientation_method': 'FORWARD',
    'heading_alignment_deg': 0,
    'mechanical_offset_deg': 0,
}

# Notes on sensor device configuration:
# * Sensor mode can be either 'standalone' (running on its own) 
#   or 'attitude-aided' (running with AHRS providing attitude)
# * Mounting position is the position of the device with respect to OCS CoG in SNAME body-fixed frame [m, m, m]
# * Mounting orientation is the orientation of the device with respect to SNAME body-fixed frame [deg, deg, deg]
# * Mounting orientation type describes the orientation purpose for the ADCP ('UP', 'DOWN', 'PAYLOAD') 
# * Mounting orientation method describes how the Syrinx is setup on its support:
#   - 'FORWARD' means that the FORWARD mark on the device matches the direction of travel
#   and it means that a +45deg Yaw rotation is required (according to Sonardyne's documentation)
# * Heading alignment is a correction provided around yaw axis to align DVL and compass
# * Mechanical offset is a physical correction provided around yaw to align DVL and support

# Set logger config
logger_name = 'process_syrinx_raw_data'
current_datetime = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
logger_output_file_name = logger_name + '_' + current_datetime + '.log'
logger_output_folder_path = os.environ.get(
    'ROS_LOG_DIR', 
    os.path.join(
        parent_of_parent_of_importer_folder,
        'log',
    ),
)
logger_output_prefix_format = "[%(asctime)s] [%(levelname)s] - %(message)s"
logger_logging_level = "DEBUG"

# Instantiate logger object
try:
    os.makedirs(logger_output_folder_path, exist_ok=True)
    logger = log_tool.getLogger(__name__)
    logger.setLevel(logger_logging_level)
    logger_output_file_path = os.path.join(logger_output_folder_path, logger_output_file_name)
    file_handler = log_tool.FileHandler(logger_output_file_path)
    formatter = log_tool.Formatter(logger_output_prefix_format)
    file_handler.setFormatter(formatter)
    logger.addHandler(file_handler)
except Exception as e:
    raise Exception(
        f"""
        Cannot instantiate logger {logger_name}
        EXCEPTION CAUGHT: {e}
        """
    )

# ====================== END USER CONFIGURATION ======================

def main():
    logger.info(f"__main__ - Script started")

    # 1) Get CLI inputs (required)

    parser = argparse.ArgumentParser(
        description="""
        Converts raw velocity data from Syrinx sensor
        Into (SNAME) body-levelled velocity data
        """
    )
    parser.add_argument(
        '-i',
        '--ocs_dataset_path',
        type=str,
        help="Path to the OCS (Syrinx) dataset file"
    )
    parser.add_argument(
        '-o',
        '--output_dataset_path',
        type=str,
        help="Path to the output dataset file (as .csv) generated by this script"
    )

    # Ensure that the sensor mode is handled
    if sensor_device_dict.get('mode') == 'standalone':
        parser.add_argument(
            '-e',
            '--ins_dataset_path',
            type=str,
            help="Path to the INS (ATT) dataset file"
        )
    else:
        raise Exception("Sensor mode other than 'standalone' is NOT handled yet.")

    args = parser.parse_args()

    logger.debug(
        f"""
        __main__ -
        Collected CLI inputs:
        * OCS dataset path = {args.ocs_dataset_path}
        * Output dataset path = {args.output_dataset_path}
        * INS dataset path = {args.ins_dataset_path}
        """
    )

    # Ensure required inputs are present
    if not args.ocs_dataset_path:
        parser.error("Please specify required input -i")
    if not args.output_dataset_path:
        parser.error("Please specify required input -o")
    if not args.ins_dataset_path:
        parser.error("Please specify required input -e")

    # Ensure that inputs are valid
    if not os.path.isfile(args.ocs_dataset_path):
        logger.error(
            f"""
            __main__ -
            {args.ocs_dataset_path} is not a file
            Review your inputs
            """
        )
    if not os.path.isfile(args.ins_dataset_path):
        logger.error(
            f"""
            __main__ -
            {args.ins_dataset_path} is not a file
            Review your inputs
            """
        )
    output_file_path, output_file_ext = \
        os.path.splitext(args.output_dataset_path)
    if not output_file_ext == '.csv':
        logger.error(
            f"""
            __main__ -
            Extension of file {args.output_dataset_path}
            must be `csv`
            Review your inputs
            """
        )

    # 2) Import data from files into pandas data frames for easier data management

    # Import OCS data
    ocs_data = load_data_from_file(
        dataset_file_path=args.ocs_dataset_path,
        target_header=0,
        target_cols=ocs_dataset_dict.get('cols_to_extract'),
    )
    logger.debug(
        f"""
        __main__ -
        Loaded dataset located at {args.ocs_dataset_path}.
        The first 10 rows are:
        {ocs_data.head(10)}
        """
    )

    # Import INS data if required
    ins_data = None
    if sensor_device_dict.get('mode') == 'standalone':
        ins_data = load_data_from_file(
            dataset_file_path=args.ins_dataset_path,
            metadata_rows_to_skip=ins_dataset_dict.get('metadata_row_total'),
            target_cols=[
                ins_dataset_dict['roll']['col'] - 1,
                ins_dataset_dict['pitch']['col'] - 1,
            ],
        )
        ins_data.columns = ['roll', 'pitch']
        logger.debug(
            f"""
            __main__ -
            Loaded dataset located at {args.ins_dataset_path}.
            The first 10 rows are:
            {ins_data.head(10)}
            """
        )
    else:
        raise Exception("Sensor mode other than 'standalone' is NOT handled yet.")

    # 3) Estimate attitude if standalone mode

    # Initialise handy variables with default values
    roll_mean = 0
    pitch_mean = 0
    roll_stdev = 0
    pitch_stdev = 0
    acceptance_stdev = ins_dataset_dict.get('acceptance_stdev_deg')  # deg
    roll_range_min_max = (0, 0)
    pitch_range_min_max = (0, 0)
    acceptance_max_range = ins_dataset_dict.get('acceptance_max_range_deg')  # deg


    if sensor_device_dict.get('mode') == 'standalone':

        roll_values = [i for i in ins_data['roll']]
        pitch_values = [i for i in ins_data['pitch']]

        # Calculate average values
        # (to use as the reference attitude quantity)
        roll_mean = mean(roll_values)
        pitch_mean = mean(pitch_values)

        logger.debug(
            f"""
            __main__ -
            Calculated:
            * Roll mean = {roll_mean}
            * Pitch mean = {pitch_mean}
            """
        )

        # Calculate standard deviation
        # (to make sure that there isn't too much spreading)
        roll_stdev = standard_deviation(roll_values)
        pitch_stdev = standard_deviation(pitch_values)

        logger.debug(
            f"""
            __main__ -
            Calculated:
            * Roll standard deviation = {roll_stdev}
            * Pitch standard deviation = {pitch_stdev}
            """
        )

        # Verify acceptance conditions on standard deviation
        if abs(roll_stdev) > acceptance_stdev:
            raise Exception(
                f"""
                Roll stdev {roll_stdev}
                is NOT within acceptance range {acceptance_stdev} deg
                """
            )
        if abs(pitch_stdev) > acceptance_stdev:
            raise Exception(
                f"""
                Pitch stdev {pitch_stdev}
                is NOT within acceptance range {acceptance_stdev} deg
                """
            )

        # Calculate range min max
        # (to spot outliers)
        roll_range_min_max = range_min_max(roll_values)
        pitch_range_min_max = range_min_max(pitch_values)

        logger.debug(
            f"""
            __main__ -
            Calculated:
            * Roll range min max = {roll_range_min_max}
            * Pitch range min max = {pitch_range_min_max}
            """
        )

        # Verify acceptance conditions on range min max
        roll_acceptance_range = roll_mean + acceptance_max_range
        if (abs(roll_range_min_max[0]) > roll_acceptance_range
                or abs(roll_range_min_max[1]) > roll_acceptance_range):
            raise Exception(
                f"""
                Roll range min max {roll_range_min_max}
                is NOT within absolute range {roll_acceptance_range} deg
                """
            )
        pitch_acceptance_range = pitch_mean + acceptance_max_range
        if (abs(pitch_range_min_max[0]) > pitch_acceptance_range
                or abs(roll_range_min_max[1]) > pitch_acceptance_range):
            raise Exception(
                f"""
                Pitch range min max {pitch_range_min_max}
                is NOT within absolute range {pitch_acceptance_range} deg
                """
            )

    else:
        raise Exception("Sensor mode other than 'standalone' is NOT handled yet.")

    # 4) Perform transformations on surge, sway, heave and add dependency on validity flag

    # Determine valid raw velocities depending on validity flags
    vel_update_management_dict = {
        1: {
            'raw_velocity_axis': 'raw_valid_velocity_x',
            'bt_velocity_axis': 'Syrinx_raw_bottom_track_velocity_x',
            'wt_velocity_axis': 'Syrinx_raw_water_track_velocity_x',
        },
        2: {
            'raw_velocity_axis': 'raw_valid_velocity_y',
            'bt_velocity_axis': 'Syrinx_raw_bottom_track_velocity_y',
            'wt_velocity_axis': 'Syrinx_raw_water_track_velocity_y',
        },
        3: {
            'raw_velocity_axis': 'raw_valid_velocity_z',
            'bt_velocity_axis': 'Syrinx_raw_bottom_track_velocity_z',
            'wt_velocity_axis': 'Syrinx_raw_water_track_velocity_z',
        },
    }

    for i in range(1,4):
        ocs_data[vel_update_management_dict[i]['raw_velocity_axis']] = [
            velocity_BT if float(validity_BT)==float(1)
            else velocity_WT if float(validity_WT)==float(1)
            else float("NaN")
            for (velocity_BT, velocity_WT, validity_BT, validity_WT) in zip(
                ocs_data[vel_update_management_dict[i]['bt_velocity_axis']],
                ocs_data[vel_update_management_dict[i]['wt_velocity_axis']],
                ocs_data['DVL_down_BT_valid'],
                ocs_data['DVL_down_WT_valid'],
            )
        ]

    logger.debug(
        f"""
        __main__ -
        Determined raw velocities
        according to validity flags for BT and WT.
        The first 10 rows of the updated OCS dataset are:
        {ocs_data.head(10)}
        """
    )

    # Apply necessary rotations to raw velocities

    roll_deg = roll_mean
    pitch_deg = pitch_mean
    yaw_deg = sensor_device_dict.get('mounting_orientation_deg', 0)[2]  # mounting orientation
    yaw_mounting_method_deg = 45 if (
        sensor_device_dict.get(
            'mounting_orientation_type',
            None,
        ) == 'DOWN'
        and sensor_device_dict.get(
            'mounting_orientation_method',
            None,
        ) == 'FORWARD'
    ) else 0

    # Note: the order of rotations, as indicated by the Syrinx documentation
    # is Yaw first, then roll, then pitch
    body_levelled_velocity_vectors = [
        np.linalg.multi_dot(
            [
                rot_matrix('y', pitch_deg),
                rot_matrix('x', roll_deg),
                rot_matrix('z', yaw_deg),
                rot_matrix('z', yaw_mounting_method_deg),
                np.array([x, y, z]).transpose(),
            ]
        )
        for (x, y, z)
        in zip(
            ocs_data['raw_valid_velocity_x'],
            ocs_data['raw_valid_velocity_y'],
            ocs_data['raw_valid_velocity_z'],
        )
    ]

    logger.debug(
        f"""
        __main__ -
        Calculated body-levelled velocities
        according to mounting method, mounting orientation and attitude
        The first 10 vectors are:
        {body_levelled_velocity_vectors[:10]}
        """
    )

    # 5) Generate csv with columns to copy over
    # and body-levelled surge, sway, heave

    # Prepare dataframe to write to csv
    dvl_data = ocs_data[
        [
            'timestamp',
            'Syrinx_aiding_velocity_covariance_00',
            'Syrinx_aiding_velocity_covariance_10',
            'Syrinx_aiding_velocity_covariance_11',
            'Syrinx_aiding_velocity_covariance_20',
            'Syrinx_aiding_velocity_covariance_21',
            'Syrinx_aiding_velocity_covariance_22',
        ]
    ]

    body_levelled_velocities_data = pd.DataFrame(
        body_levelled_velocity_vectors
    )
    body_levelled_velocities_data.columns = [
        'surge',
        'sway',
        'heave',
    ]

    dvl_data = pd.concat(
        [
            dvl_data,
            body_levelled_velocities_data,
        ],
        axis=1,
    )

    logger.debug(
        f"""
        __main__ -
        Generated dataframe with desired columns
        The first 10 rows are:
        {dvl_data.head(10)}
        """
    )

    # Create folder for output file (if it does not exist)
    full_output_dataset_path = \
        os.path.realpath(args.output_dataset_path)
    os.makedirs(
        os.path.dirname(full_output_dataset_path),
        exist_ok=True,
    )

    logger.debug(
        f"""
        __main__ -
        Created folder for output file:
        {full_output_dataset_path}
        """
    )

    # Write dataframe to output file
    dvl_data.to_csv(
        full_output_dataset_path,
        encoding='utf-8',
        index=False,
    )

    logger.debug(
        f"""
        __main__ -
        Created and populated output file:
        {full_output_dataset_path}
        """
    )

    logger.info(f"__main__ - Script terminated")


if __name__ == '__main__':
    main()
