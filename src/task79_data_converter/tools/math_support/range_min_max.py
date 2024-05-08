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

def range_min_max(values):
    """
    Get the range of a sorted list of values
  
    Parameters
    ----------
    values: iterable of float
        A list of numbers
  
    Returns
    -------
    (float, float)
        Min of values, Max of values
    """
    
    minimum_value = min(values)
    maximum_value = max(values)

    return (minimum_value, maximum_value)
