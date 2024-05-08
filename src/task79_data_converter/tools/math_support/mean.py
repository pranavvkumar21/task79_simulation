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

def mean(values):
    """
    Get the mean of a sorted list of values
  
    Parameters
    ----------
    values: iterable of float
        A list of numbers
  
    Returns
    -------
    float
        Mean of values
    """
    
    mean = sum(values) / len(values)
    
    return mean
