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

# Code heavily inspired from:
# https://stackoverflow.com/questions/75734160/convert-dictionary-keys-with-dotted-names-to-nested-dictionaries
# by Masklinn

def convert_dotted_dict_to_std_dict(input_dict):
    """
    Convert a dict with dotted keys (e.g. my_key.my_sub_key = '')
    into a standard dict (e.g. my_key: {my_sub_key: ''})
  
    Parameters
    ----------
    input_dict: dict
        Dictionary to convert
  
    Returns
    -------
    dict
        Dictionary converted
    """
    
    output_dict = {}
    for key, value in input_dict.items():
        current_dict = output_dict
        *keys, leaf = key.split('.')
        for k in keys:
            current_dict = current_dict.setdefault(k, {})
        current_dict[leaf] = value
    
    return output_dict

# Application example:
# Input dictionary
# >>> {'key_1.key_11': 'value_11', 'key_1.key_12': 'value_12', 'key_2': 'value_2'}
# Output dictionary
# >>> {'key_1': {'key_11': 'value_11', 'key_12': 'value_12'}, 'key_2': 'value_2'}
