#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
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
from tools.file_management.load_data_from_file import load_data_from_file


def main():
    # 1) Load gps data
    gps_dataset_file_path = os.path.join(
        parent_of_parent_of_importer_folder,
        'data',
        'mission_data',
        'test_12',
        'GPS.csv',
    )
    gps_data = load_data_from_file(gps_dataset_file_path, target_cols=[3, 5], metadata_rows_to_skip=2)
    print(gps_data.head())
    
    # Rename columns for practicality
    gps_data.columns = ['lat', 'lon']

    # Force expression of lat at N and lon as E
    gps_data['lon'] = [-elem for elem in gps_data['lon']]
    
    # Prepare plot overall GPS data
    fig_gps = plt.figure() 
    plt.title(f"GPS data from {gps_dataset_file_path}")
    plt.plot(pd.array(gps_data['lon']), pd.array(gps_data['lat']))
    plt.xlabel("Longitude (DM - East)")
    plt.ylabel("Latitude (DM - North)")
    plt.grid()

    plt.plot(gps_data['lon'][0], gps_data['lat'][0], 'ro', label="starting point")
    plt.legend(loc="upper right")

    # 2) Load dvl data
    dvl_dataset_file_path = os.path.join(
        parent_of_parent_of_importer_folder,
        'data',
        'mission_data',
        'test_12',
        'DVL.csv',
    )
    dvl_data = load_data_from_file(dvl_dataset_file_path, target_cols=[0, 7, 8, 9], target_header=0)
    print(dvl_data.head())
    
    # Prepare plot overall DVL data
    fig_dvl = plt.figure()
    plt.title(f"DVL data from {dvl_dataset_file_path}")
    
    ax1 = plt.subplot(3, 1, 1)
    plt.plot(pd.array(dvl_data['timestamp']), pd.array(dvl_data['surge']))
    plt.xlabel("Timestamp (UNIX - UTC)")
    plt.ylabel("Surge (m/s)")
    plt.grid()

    ax2 = plt.subplot(3, 1, 2, sharex=ax1)
    plt.plot(pd.array(dvl_data['timestamp']), pd.array(dvl_data['sway']))
    plt.ylabel("Sway (m/s)")
    plt.grid()
    
    ax3 = plt.subplot(3, 1, 3, sharex=ax1)
    plt.plot(pd.array(dvl_data['timestamp']), pd.array(dvl_data['heave']))
    plt.ylabel("Heave (m/s)")
    plt.grid()

    # Plot all
    plt.show()


if __name__ == '__main__':
    main() 
