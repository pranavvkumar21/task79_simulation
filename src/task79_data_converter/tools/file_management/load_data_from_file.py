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

import pandas as pd
import os

def load_data_from_file(
        dataset_file_path, 
        target_header=None, 
        target_cols=None, 
        metadata_rows_to_skip=0,
        separator=None,
        ):
    """ Method for loading data from file into a pandas dataframe
    """

    data_loaded = None
    
    try:
        # Extract full file path (to resolve symlinks and relative paths)
        # and extension from inputs
        dataset_file_full_path = os.path.realpath(
            dataset_file_path
        )
        _, dataset_file_extension = os.path.splitext(
            dataset_file_full_path
        )
        dataset_file_extension_no_dot = dataset_file_extension[1:]
        
        # Read the file depending on the specified extension
        if dataset_file_extension_no_dot == 'csv':
            if separator is None:
                separator = ',' 
            data_loaded = pd.read_csv(
                dataset_file_full_path,
                header=target_header,
                usecols=target_cols, 
                skiprows=metadata_rows_to_skip,
                sep=separator,
            )
        elif dataset_file_extension_no_dot == 'asc':
            if separator is None:
                separator = '\t' 
            data_loaded = pd.read_table(
                dataset_file_full_path,
                header=target_header,
                usecols=target_cols, 
                skiprows=metadata_rows_to_skip,
                sep=separator,
            )
        else:
            raise Exception(
                f"""
                file extension {dataset_file_extension_no_dot} NOT handled
                """
            )

        return data_loaded
    
    except Exception as e:
        raise Exception(
            f"""
            EXCEPTION CAUGHT: {e}
            Cannot load data from file: {dataset_file_path}
            Full file path is: {dataset_file_full_path}
            """
        )
