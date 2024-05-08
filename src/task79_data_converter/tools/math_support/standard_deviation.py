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

from tools.math_support.mean import mean

def standard_deviation(values, dataset_type='population'):
    """
    Get the standard deviation of a sorted list of values
  
    Parameters
    ----------
    values: iterable of float
        A list of numbers
    
    dataset_type: string
        Type of dataset
        'population' for whole population dataset
        'sample' for sample of population dataset

    Returns
    -------
    float
        Standard deviation of values
    """

    calculated_mean = mean(values)
    
    deviation_sum = sum([(i - calculated_mean)**2 for i in values])
    
    nb_values = len(values)
    denominator = None
    if dataset_type == 'population':
        denominator = nb_values
    elif dataset_type == 'sample':
        denominator = nb_values - 1
    else:
        raise Exception(f"Dataset type {dataset_type} NOT handled yet")
    
    standard_deviation = math.sqrt(deviation_sum/denominator)
    
    return standard_deviation
