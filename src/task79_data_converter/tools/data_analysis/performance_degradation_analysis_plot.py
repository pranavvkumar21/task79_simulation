#!/usr/bin/env python3

import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
from sklearn.metrics import r2_score
import pymap3d as pm
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


# Define variables modifiable by users

## Set waypoints
wpt_B_lat_dec = 50.36266389 
wpt_B_lon_dec = -4.16897778
wpt_D_lat_dec = 50.35586111
wpt_D_lon_dec = -4.17028056

## Set USBL beacon location
usbl_beacon_lat_dec = wpt_B_lat_dec
usbl_beacon_lon_dec = wpt_B_lon_dec

## Set LBL beacon location
## Where lld = lat_dec / lon_dec / depth_m
lbl_beacon_array_names = [
    "2303",
    "2307",
    "2309",
    "2807",
    "2809",
]
lbl_beacon_array_lld = np.array(
    [
        [50.3633559, -4.1698876, 26.546],
        [50.3637605, -4.1694870, 24.012],
        [50.3643833, -4.1700609, 24.558],
        [50.3641495, -4.1703711, 26.098],
        [50.3641993, -4.1695901, 23.091],
    ]
)
lbl_beacon_selected = "2303"
lbl_beacon_lat_dec = lbl_beacon_array_lld[
    lbl_beacon_array_names.index(lbl_beacon_selected), 
    0,
]
lbl_beacon_lon_dec = lbl_beacon_array_lld[
    lbl_beacon_array_names.index(lbl_beacon_selected), 
    1,
]
lbl_beacon_depth_m = lbl_beacon_array_lld[
    lbl_beacon_array_names.index(lbl_beacon_selected), 
    2,
]

## Define cut-off points
## for DVL bottom-lock influence research
ins_partially_dvl_aided_cut_off_lat_dec = 50.35727258199211
ins_partially_dvl_aided_cut_off_lon_dec = -4.169872421529599
ins_partially_dvl_aided_pick_up_lat_dec = 50.357268577090856
ins_partially_dvl_aided_pick_up_lon_dec = -4.1699073401656195
ins_initially_dvl_aided_cut_off_lat_dec = 50.358937948641966
ins_initially_dvl_aided_cut_off_lon_dec = -4.169692526979279
ins_initially_dvl_aided_pick_up_lat_dec = 50.364252417074916 
ins_initially_dvl_aided_pick_up_lon_dec = -4.1676379647516315

## Formatted mission data can be found
## in `task79_data_converter/data/formatted_mission_data`
## 
## The formatted mission data is generated
## through the `export_to_csv` function
## located in `formatted_data_conversion_node.py`
## and enabled via `self.is_export_to_csv_enabled`

## Set test data source
test_source = 'test_12_initially_dvl_aided'

## Set lat0 and lon0
## for Northings and Eastings reference
lat0_dec = wpt_B_lat_dec
lon0_dec = wpt_B_lon_dec
h0_m = 0
ellipsoid_shape = 'wgs84'
default_depth = 0

## Plot colours
gps_colour = "green" 
usbl_colour = "grey"
lbl_colour = "orange"
ins_free_inertial_colour = "paleturquoise"
ins_dvl_aided_colour = "blue"
ins_partially_dvl_aided_colour = "thistle"
ins_initially_dvl_aided_colour = "deeppink"

## Define dictionary to hold
## loaded datasets
## for easier data retrieval
loaded_dataset_dict = {
    'INS_FREE_INERTIAL': {
        'POSITION_SOURCE': pd.DataFrame(),
    },
    'INS_FULLY_AIDED': {
        'POSITION_SOURCE': pd.DataFrame(),
    },
    'INS_DVL_AIDED': {
        'POSITION_SOURCE': pd.DataFrame(),
    },
    'INS_PARTIALLY_DVL_AIDED': {
        'POSITION_SOURCE': pd.DataFrame(),
    },
    'INS_INITIALLY_DVL_AIDED': {
        'POSITION_SOURCE': pd.DataFrame(),
    },
    'GPS': {'POSITION_SOURCE': pd.DataFrame()},
    'USBL': {'POSITION_SOURCE': pd.DataFrame()},
    'LBL': {'POSITION_SOURCE': pd.DataFrame()},
    'DVL': {'VELOCITY_SOURCE': pd.DataFrame()},
}

def load_formatted_data():

    for sensor_type in loaded_dataset_dict:
        sensor_type_dict = loaded_dataset_dict.get(sensor_type, dict())
        for sensor_source in sensor_type_dict:
            sensor_source_df = sensor_type_dict.get(sensor_source, None)
            
            if sensor_source_df is not None:

                dataset_file_name = str(sensor_type) + '_' + str(sensor_source) + '.csv'

                dataset_file_path = os.path.realpath(
                    os.path.join(
                        parent_of_parent_of_importer_folder,
                        'data',
                        'formatted_mission_data',
                        test_source,
                        dataset_file_name,
                    )
                )
                
                data_frame = load_data_from_file(
                    dataset_file_path, 
                    target_header=0,
                )
                 
                # Remove duplicate timestamps
                # because might generate data conflicts
                data_frame = data_frame.drop_duplicates(subset=['timestamp_s'], keep='last')
              
                print(
                    f"""
                    Collected formatted data for {sensor_type} and {sensor_source}
                    into a dataframe: {data_frame.head()}
                    """
                )
                
                # Add Northings and Eastings if POSITION_SOURCE
                if sensor_source == 'POSITION_SOURCE':
                    (data_frame['northings_m'], data_frame['eastings_m'], _) = pm.geodetic2ned(
                        data_frame['lat_dec'], 
                        data_frame['lon_dec'], 
                        data_frame['depth_m'], 
                        lat0_dec, 
                        lon0_dec, 
                        h0_m,
                        ell=pm.utils.Ellipsoid(ellipsoid_shape)
                    )

                    print(
                        f"""
                        Added Northings and Eastings to GPS dataframe
                        with lat0 = {lat0_dec}
                        and lon0 = {lon0_dec}
                        and h0 = {h0_m}
                        {data_frame.head()}
                        """
                    )
            
                # Update loaded dataset dict
                loaded_dataset_dict[sensor_type][sensor_source] = data_frame

def reference_align_formatted_data(reference_df):

    for sensor_type in loaded_dataset_dict:
        sensor_type_dict = loaded_dataset_dict.get(sensor_type, dict())
        for sensor_source in sensor_type_dict:
            sensor_source_df = sensor_type_dict.get(sensor_source, None)
            
            if sensor_source_df is not None:
               
                # Merge sensor source df
                # with reference df
                # based on timestamp values
                merged_df = pd.merge(
                    reference_df['timestamp_s'], 
                    sensor_source_df, 
                    on='timestamp_s',
                    how='outer',
                    validate='one_to_one',
                    indicator=True,
                )
                
                print(
                    f"""
                    Merged df for {sensor_type} and {sensor_source}:
                    {merged_df}
                    """
                )
                
                # Remove timestamp values
                # only found in sensor source df
                right_only_mask = merged_df['_merge'] == 'right_only'
                merged_resized_df = merged_df[~right_only_mask] 

                print(
                    f"""
                    Resized df for {sensor_type} and {sensor_source}:
                    {merged_resized_df}
                    """
                )
                
                # Update loaded dataset dict
                loaded_dataset_dict[sensor_type][sensor_source] = merged_resized_df  

def main():
    
    # Load formatted data
    load_formatted_data()
    
    # Set reference df and add timestamps
    # The reference timestamp series will be taken from GPS dataset
    # because the GPS serves as ground truth
    reference_df = pd.DataFrame()
    reference_df['timestamp_s'] = loaded_dataset_dict['GPS']['POSITION_SOURCE']['timestamp_s']

    # Reference align formatted data
    # So that all datasets have at least 
    # one column in common
    reference_align_formatted_data(reference_df)

    # Perform useful calculations for future plots
    ## Obtain wpt northings and eastings
    (wpt_B_northings_m, wpt_B_eastings_m, _) = pm.geodetic2ned(
        wpt_B_lat_dec,
        wpt_B_lon_dec,
        default_depth,
        lat0_dec,
        lon0_dec,
        h0_m,
        ell=pm.utils.Ellipsoid(ellipsoid_shape)
    )
    (wpt_D_northings_m, wpt_D_eastings_m, _) = pm.geodetic2ned(
        wpt_D_lat_dec,
        wpt_D_lon_dec,
        default_depth,
        lat0_dec,
        lon0_dec,
        h0_m,
        ell=pm.utils.Ellipsoid(ellipsoid_shape)
    )
    (usbl_beacon_northings_m, usbl_beacon_eastings_m, _) = pm.geodetic2ned(
        usbl_beacon_lat_dec,
        usbl_beacon_lon_dec,
        default_depth,
        lat0_dec,
        lon0_dec,
        h0_m,
        ell=pm.utils.Ellipsoid(ellipsoid_shape)
    )
    (lbl_beacon_northings_m, lbl_beacon_eastings_m, _) = pm.geodetic2ned(
        lbl_beacon_lat_dec,
        lbl_beacon_lon_dec,
        default_depth,
        lat0_dec,
        lon0_dec,
        h0_m,
        ell=pm.utils.Ellipsoid(ellipsoid_shape)
    )
    (lbl_beacon_array_northings_m, lbl_beacon_array_eastings_m, _) = pm.geodetic2ned(
        lbl_beacon_array_lld[:, 0],
        lbl_beacon_array_lld[:, 1],
        default_depth,
        lat0_dec,
        lon0_dec,
        h0_m,
        ell=pm.utils.Ellipsoid(ellipsoid_shape)
    )
    (ins_partially_dvl_aided_cut_off_northings_m, 
            ins_partially_dvl_aided_cut_off_eastings_m, _) = pm.geodetic2ned(
        ins_partially_dvl_aided_cut_off_lat_dec,
        ins_partially_dvl_aided_cut_off_lon_dec,
        default_depth,
        lat0_dec,
        lon0_dec,
        h0_m,
        ell=pm.utils.Ellipsoid(ellipsoid_shape)
    )
    (ins_partially_dvl_aided_pick_up_northings_m, 
            ins_partially_dvl_aided_pick_up_eastings_m, _) = pm.geodetic2ned(
        ins_partially_dvl_aided_pick_up_lat_dec,
        ins_partially_dvl_aided_pick_up_lon_dec,
        default_depth,
        lat0_dec,
        lon0_dec,
        h0_m,
        ell=pm.utils.Ellipsoid(ellipsoid_shape)
    )
    (ins_initially_dvl_aided_cut_off_northings_m, 
            ins_initially_dvl_aided_cut_off_eastings_m, _) = pm.geodetic2ned(
        ins_initially_dvl_aided_cut_off_lat_dec,
        ins_initially_dvl_aided_cut_off_lon_dec,
        default_depth,
        lat0_dec,
        lon0_dec,
        h0_m,
        ell=pm.utils.Ellipsoid(ellipsoid_shape)
    )

    # ---------- Plot 2D trajectory ----------
    ## Setup window and plot
    fig_2D_traj, axs_2D_traj = plt.subplots()
    fig_2D_traj.canvas.set_window_title("2D Trajectory")
    axs_2D_traj.set_title(f"2D Trajectory \nfor {test_source}")
    ## Add GPS trajectory
    axs_2D_traj.plot(
        np.array(loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'].dropna()), 
        np.array(loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m'].dropna()),
        color=gps_colour,
        label="GPS",
        linewidth=3,
    )
    ## Add USBL trajectory
    axs_2D_traj.scatter(
        np.array(loaded_dataset_dict['USBL']['POSITION_SOURCE']['eastings_m'].dropna()), 
        np.array(loaded_dataset_dict['USBL']['POSITION_SOURCE']['northings_m'].dropna()), 
        color=usbl_colour,
        marker="o",
        label="USBL",
        s=5,
    )
    ## Add LBL trajectory
    axs_2D_traj.scatter(
        np.array(loaded_dataset_dict['LBL']['POSITION_SOURCE']['eastings_m'].dropna()), 
        np.array(loaded_dataset_dict['LBL']['POSITION_SOURCE']['northings_m'].dropna()), 
        color=lbl_colour,
        marker="o",
        label="LBL",
        s=5,
    )
    ## Add INS FREE INERTIAL trajectory
    axs_2D_traj.scatter(
        np.array(loaded_dataset_dict['INS_FREE_INERTIAL']['POSITION_SOURCE']['eastings_m'].dropna()), 
        np.array(loaded_dataset_dict['INS_FREE_INERTIAL']['POSITION_SOURCE']['northings_m'].dropna()), 
        color=ins_free_inertial_colour,
        marker="o",
        label="INS_FREE_INERTIAL",
        s=5,
    )
    ## Add INS FREE INERTIAL starting point for clarity
    axs_2D_traj.plot(
        np.array(loaded_dataset_dict['INS_FREE_INERTIAL']['POSITION_SOURCE']['eastings_m'].dropna()[0]), 
        np.array(loaded_dataset_dict['INS_FREE_INERTIAL']['POSITION_SOURCE']['northings_m'].dropna()[0]), 
        color="firebrick",
        marker="o",
        label="INS FREE INERTIAL starting point",
        markersize=4,
    )
    ## Add INS DVL AIDED trajectory
    axs_2D_traj.scatter(
        np.array(loaded_dataset_dict['INS_DVL_AIDED']['POSITION_SOURCE']['eastings_m'].dropna()), 
        np.array(loaded_dataset_dict['INS_DVL_AIDED']['POSITION_SOURCE']['northings_m'].dropna()), 
        color=ins_dvl_aided_colour,
        marker="o",
        label="INS_DVL_AIDED",
        s=5,
    )
    ## Add INS PARTIALLY DVL AIDED trajectory
    axs_2D_traj.scatter(
        np.array(loaded_dataset_dict['INS_PARTIALLY_DVL_AIDED']['POSITION_SOURCE']['eastings_m'].dropna()),
        np.array(loaded_dataset_dict['INS_PARTIALLY_DVL_AIDED']['POSITION_SOURCE']['northings_m'].dropna()),
        color=ins_partially_dvl_aided_colour,
        marker="o",
        label="INS_PARTIALLY_DVL_AIDED",
        s=5,
    )
    ## Add INS PARTIALLY DVL AIDED cut-off point for clarity
    axs_2D_traj.plot(
        np.array([ins_partially_dvl_aided_cut_off_eastings_m]), 
        np.array([ins_partially_dvl_aided_cut_off_northings_m]), 
        color="olive",
        marker="o",
        label="INS PARTIALLY DVL AIDED cut-off point",
        markersize=4,
    )
    ## Add INS PARTIALLY DVL AIDED pick-up point for clarity
    axs_2D_traj.plot(
        np.array([ins_partially_dvl_aided_pick_up_eastings_m]), 
        np.array([ins_partially_dvl_aided_pick_up_northings_m]), 
        color="peru",
        marker="o",
        label="INS PARTIALLY DVL AIDED pick-up point",
        markersize=4,
    )
    ## Add INS INITIALLY DVL AIDED trajectory
    axs_2D_traj.scatter(
        np.array(loaded_dataset_dict['INS_INITIALLY_DVL_AIDED']['POSITION_SOURCE']['eastings_m'].dropna()), 
        np.array(loaded_dataset_dict['INS_INITIALLY_DVL_AIDED']['POSITION_SOURCE']['northings_m'].dropna()), 
        color=ins_initially_dvl_aided_colour,
        marker="o",
        label="INS_INITIALLY_DVL_AIDED",
        s=5,
    )
    ## Add INS INITIALLY DVL AIDED cut-off point for clarity
    axs_2D_traj.plot(
        np.array([ins_initially_dvl_aided_cut_off_eastings_m]), 
        np.array([ins_initially_dvl_aided_cut_off_northings_m]), 
        color="yellow",
        marker="o",
        label="INS INITIALLY DVL AIDED cut-off point",
        markersize=5,
    )
    ## Add mission waypoints
    axs_2D_traj.plot(
        np.array([wpt_B_eastings_m, wpt_D_eastings_m]), 
        np.array([wpt_B_northings_m, wpt_D_northings_m]), 
        color="black",
        marker="o",
        linestyle="dashed",
        label="B-D",
        linewidth=1,
        markersize=3,
    )
    ## Add mission starting point
    valid_idx_eastings = loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'].first_valid_index()
    axs_2D_traj.plot(
        np.array(loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'][valid_idx_eastings]), 
        np.array(loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m'][valid_idx_eastings]),
        color="red",
        marker="o",
        label="GPS starting point",
        markersize=4,
    )
    ## Add LBL beacon locations
    x = np.array(lbl_beacon_array_eastings_m)
    y = np.array(lbl_beacon_array_northings_m) 
    axs_2D_traj.scatter(
        x,
        y,
        color="darkviolet",
        marker="o",
        label="LBL beacon array",
        s=20,
    )
    for idx, txt in enumerate(lbl_beacon_array_names):
        axs_2D_traj.annotate(txt, (x[idx], y[idx]))
    ## Add labels and legend
    axs_2D_traj.set_xlabel("Eastings (m)")
    axs_2D_traj.set_ylabel("Northings (m)")
    axs_2D_traj.legend(loc="upper right")
    axs_2D_traj.grid()

    print(
        f"""
        Cut-off point research for DVL loss of bottom-lock
        with INS_PARTIALLY_DVL_AIDED dataset is:
        lat dec = {ins_partially_dvl_aided_cut_off_lat_dec}
        lon dec = {ins_partially_dvl_aided_cut_off_lat_dec}
        """
    )
    print(
        f"""
        Cut-off point research for DVL loss of bottom-lock
        with INS_INITIALLY_DVL_AIDED dataset is:
        lat dec = {ins_initially_dvl_aided_cut_off_lat_dec}
        lon dec = {ins_initially_dvl_aided_cut_off_lon_dec}
        """
    )

    # ---------- Plot navigation performance comparison GPS vs USBL ----------
    ## Setup window and plot
    fig_gps_usbl_comp, axs_gps_usbl_comp = plt.subplots(
        nrows=4,
        ncols=1,
        sharex=True,
    )
    fig_gps_usbl_comp.canvas.set_window_title("Navigation performance comparison GPS vs USBL")
    fig_gps_usbl_comp.suptitle(f"Navigation performance comparison GPS vs USBL \nfor {test_source}")
    ## Subplot(0, 0)
    axs_gps_usbl_comp[0].set_title(f"Northings comparison")
    ### Add GPS
    axs_gps_usbl_comp[0].plot(
        np.array(reference_df['timestamp_s']),
        np.array(loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m']),
        color=gps_colour,
        label="GPS",
    )
    ### Add USBL
    axs_gps_usbl_comp[0].scatter(
        np.array(reference_df['timestamp_s']),
        np.array(loaded_dataset_dict['USBL']['POSITION_SOURCE']['northings_m']),
        color=usbl_colour,
        s=10,
        label="USBL",
    )
    ## Add labels and legend
    axs_gps_usbl_comp[0].set_xlabel("Timestamp (s)")
    axs_gps_usbl_comp[0].set_ylabel("Northings (m)")
    axs_gps_usbl_comp[0].legend(loc="upper right")
    axs_gps_usbl_comp[0].grid()
    ## Subplot(1, 0)
    axs_gps_usbl_comp[1].set_title(f"Eastings comparison")
    ### Add GPS
    axs_gps_usbl_comp[1].plot(
        np.array(reference_df['timestamp_s']),
        np.array(loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m']),
        color=gps_colour,
        label="GPS",
    )
    ### Add USBL
    axs_gps_usbl_comp[1].scatter(
        np.array(reference_df['timestamp_s']),
        np.array(loaded_dataset_dict['USBL']['POSITION_SOURCE']['eastings_m']),
        color=usbl_colour,
        s=10,
        label="USBL",
    )
    ## Add labels and legend
    axs_gps_usbl_comp[1].set_xlabel("Timestamp (s)")
    axs_gps_usbl_comp[1].set_ylabel("Eastings (m)")
    axs_gps_usbl_comp[1].legend(loc="upper right")
    axs_gps_usbl_comp[1].grid()
    ## Subplot(2, 0)
    axs_gps_usbl_comp[2].set_title(f"Horizontal distance error between GPS and USBL")
    ### Calculate horizontal distance between GPS and USBL
    usbl_df = pd.DataFrame()
    usbl_df['dist_err_gps_m'] = \
        np.sqrt(
            (loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m'] - loaded_dataset_dict['USBL']['POSITION_SOURCE']['northings_m'])**2
            + (loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'] - loaded_dataset_dict['USBL']['POSITION_SOURCE']['eastings_m'])**2
    )
    ### Add distance error
    axs_gps_usbl_comp[2].scatter(
        np.array(reference_df['timestamp_s']),
        np.array(usbl_df['dist_err_gps_m']),
        color="black",
        s=1,
    )
    ### Add labels and legend
    axs_gps_usbl_comp[2].set_xlabel("Timestamp (s)")
    axs_gps_usbl_comp[2].set_ylabel("Distance (m)")
    axs_gps_usbl_comp[2].grid()
    ## Subplot(3, 0)
    axs_gps_usbl_comp[3].set_title(f"Horizontal distance between GPS and USBL beacon")
    ### Calculate horizontal distance between GPS and USBL beacon
    usbl_beacon_northings_m_array = np.ones(
        (loaded_dataset_dict['GPS']['POSITION_SOURCE'].shape[0], 1)
    ) * usbl_beacon_northings_m
    usbl_beacon_eastings_m_array = np.ones(
        (loaded_dataset_dict['GPS']['POSITION_SOURCE'].shape[0], 1)
    ) * usbl_beacon_eastings_m
    usbl_beacon_df = pd.DataFrame(
        {
            'northings_m': usbl_beacon_northings_m_array[:, 0],
            'eastings_m': usbl_beacon_eastings_m_array[:, 0],
        }
    )
    usbl_beacon_df['dist_gps_m'] = \
        np.sqrt(
            (loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m'] - usbl_beacon_df['northings_m'])**2
            + (loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'] - usbl_beacon_df['eastings_m'])**2
    )
    ### Add distance
    axs_gps_usbl_comp[3].scatter(
        np.array(reference_df['timestamp_s']),
        np.array(usbl_beacon_df['dist_gps_m']),
        color="black",
        s=1,
    )
    ### Add labels and legend
    axs_gps_usbl_comp[3].set_xlabel("Timestamp (s)")
    axs_gps_usbl_comp[3].set_ylabel("Distance (m)")
    axs_gps_usbl_comp[3].grid()
    
    ## Maximise plot and adjust margins
    plt.subplots_adjust(hspace=0.35)

    # ---------- Plot navigation performance degradation theories for USBL ----------
    ## Setup window and plot
    fig_usbl_nav_perf_degr, axs_usbl_nav_perf_degr = plt.subplots()
    fig_usbl_nav_perf_degr.canvas.set_window_title("Navigation performance degradation theories for USBL")
    fig_usbl_nav_perf_degr.suptitle(f"Navigation performance degradation theories for USBL \nfor {test_source}")
   
    ## Add distance error vs distance to beacon
    axs_usbl_nav_perf_degr.scatter(
        np.array(usbl_beacon_df['dist_gps_m']), 
        np.array(usbl_df['dist_err_gps_m']), 
        color="black",
        marker="o",
        label="Sensor data",
    )
    ## Set graph limits to ignore outliers
    ymin = -10
    ymax = 50
    axs_usbl_nav_perf_degr.set_ylim([ymin, ymax])
    ## Compute polynomial fits and associated R2 scores
    print("-----------------------------")
    print("USBL poly fit model equations:")
    poly_deg_colour_dict = {
        1: "blue",
        2: "indigo",
        3: "violet",
        4: "hotpink",
    }
    # Collect inputs
    x_input = np.array(usbl_beacon_df['dist_gps_m'])
    y_input = np.array(usbl_df['dist_err_gps_m'])
    # Ensure that there are no NaN (only finite values)
    # otherwise the polyfit cannot be computed
    index_not_nan = np.isfinite(x_input) & np.isfinite(y_input)
    # Remove outliers for a better fit
    index_no_outlier = y_input < ymax
    idx_selected = index_not_nan & index_no_outlier
    x = x_input[idx_selected]
    y = y_input[idx_selected]
    for deg in poly_deg_colour_dict:
        # Create polynomial model
        poly_model = np.poly1d(
            np.polyfit(
                x, 
                y, 
                deg,
            )
        )
        # Calculate R2 score for the polynomial model
        r2_score_res = r2_score(
            y,
            poly_model(x),
        )
        # Add the polynomial plot
        axs_usbl_nav_perf_degr.plot(
            x, 
            poly_model(x), 
            color=poly_deg_colour_dict[deg],
            marker="o",
            markersize=2,
            linewidth=2,
            label=f"poly fit deg {deg}\nwith r2 score {r2_score_res}"
        )
        print(poly_model)
    ## Add labels and legend
    axs_usbl_nav_perf_degr.set_xlabel("Distance GPS to USBL beacon (m)")
    axs_usbl_nav_perf_degr.set_ylabel("Distance error GPS vs USBL (m)")
    axs_usbl_nav_perf_degr.legend(loc="upper right")
    axs_usbl_nav_perf_degr.grid()
     
    # ---------- Plot navigation performance comparison GPS vs LBL ----------
    ## Setup window and plot
    fig_gps_lbl_comp, axs_gps_lbl_comp = plt.subplots(
        nrows=4,
        ncols=1,
        sharex=True,
    )
    fig_gps_lbl_comp.canvas.set_window_title("Navigation performance comparison GPS vs LBL")
    fig_gps_lbl_comp.suptitle(f"Navigation performance comparison GPS vs LBL \nfor {test_source}")
    ## Subplot(0, 0)
    axs_gps_lbl_comp[0].set_title(f"Northings comparison")
    ### Add GPS
    axs_gps_lbl_comp[0].plot(
        np.array(reference_df['timestamp_s']),
        np.array(loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m']),
        color=gps_colour,
        label="GPS",
    )
    ### Add LBL
    axs_gps_lbl_comp[0].scatter(
        np.array(reference_df['timestamp_s']),
        np.array(loaded_dataset_dict['LBL']['POSITION_SOURCE']['northings_m']),
        color=lbl_colour,
        s=10,
        label="LBL",
    )
    ## Add labels and legend
    axs_gps_lbl_comp[0].set_xlabel("Timestamp (s)")
    axs_gps_lbl_comp[0].set_ylabel("Northings (m)")
    axs_gps_lbl_comp[0].legend(loc="upper right")
    axs_gps_lbl_comp[0].grid()
    ## Subplot(1, 0)
    axs_gps_lbl_comp[1].set_title(f"Eastings comparison")
    ### Add GPS
    axs_gps_lbl_comp[1].plot(
        np.array(reference_df['timestamp_s']),
        np.array(loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m']),
        color=gps_colour,
        label="GPS",
    )
    ### Add LBL
    axs_gps_lbl_comp[1].scatter(
        np.array(reference_df['timestamp_s']),
        np.array(loaded_dataset_dict['LBL']['POSITION_SOURCE']['eastings_m']),
        color=lbl_colour,
        s=10,
        label="LBL",
    )
    ## Add labels and legend
    axs_gps_lbl_comp[1].set_xlabel("Timestamp (s)")
    axs_gps_lbl_comp[1].set_ylabel("Eastings (m)")
    axs_gps_lbl_comp[1].legend(loc="upper right")
    axs_gps_lbl_comp[1].grid()
    ## Subplot(2, 0)
    axs_gps_lbl_comp[2].set_title(f"Horizontal distance error between GPS and LBL")
    ### Calculate horizontal distance between GPS and LBL
    lbl_df = pd.DataFrame()
    lbl_df['dist_err_gps_m'] = \
        np.sqrt(
            (loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m'] - loaded_dataset_dict['LBL']['POSITION_SOURCE']['northings_m'])**2
            + (loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'] - loaded_dataset_dict['LBL']['POSITION_SOURCE']['eastings_m'])**2
    )
    ### Add distance error
    axs_gps_lbl_comp[2].scatter(
        np.array(reference_df['timestamp_s']),
        np.array(lbl_df['dist_err_gps_m']),
        color="black",
        s=1,
    )
    ### Add labels and legend
    axs_gps_lbl_comp[2].set_xlabel("Timestamp (s)")
    axs_gps_lbl_comp[2].set_ylabel("Distance (m)")
    axs_gps_lbl_comp[2].grid()
    ## Subplot(3, 0)
    axs_gps_lbl_comp[3].set_title(f"Horizontal distance between GPS and LBL beacon")
    ### Calculate 3D distance between GPS and LBL beacon
    ### Note: this is a more accurate model
    ### than only 2D horizontal distance (which is linear too though)
    lbl_beacon_northings_m_array = np.ones(
        (loaded_dataset_dict['GPS']['POSITION_SOURCE'].shape[0], 1)
    ) * lbl_beacon_northings_m
    lbl_beacon_eastings_m_array = np.ones(
        (loaded_dataset_dict['GPS']['POSITION_SOURCE'].shape[0], 1)
    ) * lbl_beacon_eastings_m
    lbl_beacon_depth_m_array = np.ones(
        (loaded_dataset_dict['GPS']['POSITION_SOURCE'].shape[0], 1)
    ) * lbl_beacon_depth_m
    gps_position_source_depth_m_array = np.ones(
        (loaded_dataset_dict['GPS']['POSITION_SOURCE'].shape[0], 1)
    ) * default_depth
    lbl_beacon_df = pd.DataFrame(
        {
            'northings_m': lbl_beacon_northings_m_array[:, 0],
            'eastings_m': lbl_beacon_eastings_m_array[:, 0],
            'depth_m': lbl_beacon_depth_m_array[:, 0],
        }
    )
    lbl_beacon_df['dist_gps_m'] = \
        np.sqrt(
            (loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m'] - lbl_beacon_df['northings_m'])**2
            + (loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'] - lbl_beacon_df['eastings_m'])**2
            + (gps_position_source_depth_m_array[:, 0] - lbl_beacon_df['depth_m'])**2
    )
    ### Add distance
    axs_gps_lbl_comp[3].scatter(
        np.array(reference_df['timestamp_s']),
        np.array(lbl_beacon_df['dist_gps_m']),
        color="black",
        s=1,
    )
    ### Add labels and legend
    axs_gps_lbl_comp[3].set_xlabel("Timestamp (s)")
    axs_gps_lbl_comp[3].set_ylabel("Distance (m)")
    axs_gps_lbl_comp[3].grid()
    
    ## Maximise plot and adjust margins
    plt.subplots_adjust(hspace=0.35)
    
    # ---------- Plot navigation performance degradation theories for LBL ----------
    ## Setup window and plot
    fig_lbl_nav_perf_degr, axs_lbl_nav_perf_degr = plt.subplots()
    fig_lbl_nav_perf_degr.canvas.set_window_title("Navigation performance degradation theories for LBL")
    fig_lbl_nav_perf_degr.suptitle(f"Navigation performance degradation theories for LBL \nfor {test_source}")
   
    ## Add distance error vs distance to beacon
    axs_lbl_nav_perf_degr.scatter(
        np.array(lbl_beacon_df['dist_gps_m']), 
        np.array(lbl_df['dist_err_gps_m']), 
        color="black",
        marker="o",
        label="Sensor data",
    )
    ## Set graph limits to ignore outliers
    ymin = -10
    ymax = 60
    axs_lbl_nav_perf_degr.set_ylim([ymin, ymax])
    ## Compute polynomial fits and associated R2 scores
    print("-----------------------------")
    print("LBL poly fit model equations:")
    poly_deg_colour_dict = {
        1: "blue",
        2: "indigo",
        3: "violet",
        4: "hotpink",
    }
    # Collect inputs
    x_input = np.array(lbl_beacon_df['dist_gps_m'])
    y_input = np.array(lbl_df['dist_err_gps_m'])
    # Ensure that there are no NaN (only finite values)
    # otherwise the polyfit cannot be computed
    index_not_nan = np.isfinite(x_input) & np.isfinite(y_input)
    # Remove outliers for a better fit
    index_no_outlier = y_input < ymax
    idx_selected = index_not_nan & index_no_outlier
    x = x_input[idx_selected]
    y = y_input[idx_selected]
    for deg in poly_deg_colour_dict:
        # Create polynomial model
        poly_model = np.poly1d(
            np.polyfit(
                x, 
                y, 
                deg,
            )
        )
        # Calculate R2 score for the polynomial model
        r2_score_res = r2_score(
            y,
            poly_model(x),
        )
        # Add the polynomial plot
        axs_lbl_nav_perf_degr.plot(
            x, 
            poly_model(x), 
            color=poly_deg_colour_dict[deg],
            marker="o",
            markersize=2,
            linewidth=2,
            label=f"poly fit deg {deg}\nwith r2 score {r2_score_res}"
        )
        print(poly_model)
    ## Add labels and legend
    axs_lbl_nav_perf_degr.set_xlabel("Distance GPS to LBL beacon (m)")
    axs_lbl_nav_perf_degr.set_ylabel("Distance error GPS vs LBL (m)")
    axs_lbl_nav_perf_degr.legend(loc="upper right")
    axs_lbl_nav_perf_degr.grid()

    # ---------- Plot navigation performance degradation theories for DVL ----------
    ## Setup window and plot
    fig_dvl_nav_perf_degr, axs_dvl_nav_perf_degr = plt.subplots(
        nrows=2,
        ncols=1,
    )
    fig_dvl_nav_perf_degr.canvas.set_window_title("Navigation performance degradation models for DVL")
    fig_dvl_nav_perf_degr.suptitle(f"Navigation performance degradation models for DVL \nfor {test_source}")
    ## Subplot(0, 0)
    axs_dvl_nav_perf_degr[0].set_title(f"Model for PARTIAL DVL AIDING")
    ### Calculate horizontal distance between GPS and INS PARTIALLY DVL AIDED
    ins_partially_dvl_aided_df = pd.DataFrame()
    ins_partially_dvl_aided_df['dist_err_gps_m'] = \
        np.sqrt(
            (loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m'] - loaded_dataset_dict['INS_PARTIALLY_DVL_AIDED']['POSITION_SOURCE']['northings_m'])**2
            + (loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'] - loaded_dataset_dict['INS_PARTIALLY_DVL_AIDED']['POSITION_SOURCE']['eastings_m'])**2
    )
    ## Set graph limits to ignore outliers
    ymin = -1
    ymax = 10
    axs_dvl_nav_perf_degr[0].set_ylim([ymin, ymax])
    ## Compute polynomial fits and associated R2 scores
    print("-----------------------------")
    print("INS PARTIALLY DVL AIDED  poly fit model equations:")
    poly_deg_colour_dict = {
        1: "blue",
        2: "indigo",
        3: "violet",
        4: "hotpink",
    }
    # Collect inputs
    x_input = np.array(reference_df['timestamp_s'])
    y_input = np.array(ins_partially_dvl_aided_df['dist_err_gps_m'])
    # Find timestamp related to DVL cut-off point
    index_dvl_cut_off_timestamp = 0
    try:
        target_ins_df = loaded_dataset_dict['INS_PARTIALLY_DVL_AIDED']['POSITION_SOURCE']
        index_dvl_cut_off_lat_match = target_ins_df[
           target_ins_df['lat_dec']==ins_partially_dvl_aided_cut_off_lat_dec
        ].index.values
        if len(index_dvl_cut_off_lat_match) == 1:
            index_dvl_cut_off_timestamp = index_dvl_cut_off_lat_match
        else:
            raise Exception("Warning: found multiple or no matches for DVL cut off lat")
    except Exception as e:
        print("Caught error: {e}\nNot setting DVL cut-off point")
    timestamp_dvl_cut_off = x_input[index_dvl_cut_off_timestamp]
    # Find timestamp related to DVL pick-up point
    index_dvl_pick_up_timestamp = -1
    try:
        target_ins_df = loaded_dataset_dict['INS_PARTIALLY_DVL_AIDED']['POSITION_SOURCE']
        index_dvl_pick_up_lat_match = target_ins_df[
           target_ins_df['lat_dec']==ins_partially_dvl_aided_pick_up_lat_dec
        ].index.values
        if len(index_dvl_pick_up_lat_match) == 1:
            index_dvl_pick_up_timestamp = index_dvl_pick_up_lat_match
        else:
            raise Exception("Warning: found multiple or no matches for DVL cut off lat")
    except Exception as e:
        print("Caught error: {e}\nNot setting DVL cut-off point")
    timestamp_dvl_pick_up = x_input[index_dvl_pick_up_timestamp]
    # Ensure that there are no NaN (only finite values)
    # otherwise the polyfit cannot be computed
    index_not_nan = np.isfinite(x_input) & np.isfinite(y_input)
    # Remove outliers for a better fit
    index_no_outlier = y_input < ymax
    # Focus on range
    # between DVL aiding cut-off point / time
    # and DVL pick-up point / time
    index_no_dvl_post_cut_off = x_input > timestamp_dvl_cut_off 
    index_no_dvl_pre_pick_up = x_input < timestamp_dvl_pick_up
    # Group all conditions
    idx_selected = (index_not_nan 
        & index_no_outlier 
        & index_no_dvl_post_cut_off
        & index_no_dvl_pre_pick_up
    )
    x = x_input[idx_selected]
    y = y_input[idx_selected]
    # Set start time to 0
    # for model consistency
    time_offset = x[0]
    x -= time_offset
    for deg in poly_deg_colour_dict:
        # Create polynomial model
        poly_model = np.poly1d(
            np.polyfit(
                x, 
                y, 
                deg,
            )
        )
        # Calculate R2 score for the polynomial model
        r2_score_res = r2_score(
            y,
            poly_model(x),
        )
        # Add the polynomial plot
        axs_dvl_nav_perf_degr[0].plot(
            x, 
            poly_model(x), 
            color=poly_deg_colour_dict[deg],
            marker="o",
            markersize=2,
            linewidth=2,
            label=f"poly fit deg {deg}\nwith r2 score {r2_score_res}",
        )
        print(poly_model)
    ### Add curve fitting square model
    ### Since most promising from polynomial fit
    model_param, model_param_cov = curve_fit(dvl_influence_model, x, y)
    model_y = dvl_influence_model(x, model_param[0], model_param[1])
    r2_score_res = r2_score(
        y,
        model_y,
    )
    axs_dvl_nav_perf_degr[0].plot(
        x, 
        model_y, 
        color="lightgreen",
        marker="o",
        markersize=2,
        linewidth=2,
        label=f"Cubic fit\nwith r2 score {r2_score_res}"
    )
    print(f"\nCubic fit:y = {model_param[0]}*x^3 + {model_param[1]}")
    ### Add distance error
    axs_dvl_nav_perf_degr[0].scatter(
        x,
        y,
        color="grey",
        s=5,
        label="Sensor data",
        zorder=len(poly_deg_colour_dict) + 1,
    )
    ### Add labels and legend
    axs_dvl_nav_perf_degr[0].set_xlabel("Timestamp elapsed between DVL aiding cut-off and pick-up (s)")
    axs_dvl_nav_perf_degr[0].set_ylabel("Distance error GPS vs INS (m)")
    axs_dvl_nav_perf_degr[0].legend(loc="upper right")
    axs_dvl_nav_perf_degr[0].grid()
    ## Subplot(1, 0)
    axs_dvl_nav_perf_degr[1].set_title(f"Model for INITIAL DVL AIDING")
    ### Calculate horizontal distance between GPS and INS INITIALLY DVL AIDED
    ins_initially_dvl_aided_df = pd.DataFrame()
    ins_initially_dvl_aided_df['dist_err_gps_m'] = \
        np.sqrt(
            (loaded_dataset_dict['GPS']['POSITION_SOURCE']['northings_m'] - loaded_dataset_dict['INS_INITIALLY_DVL_AIDED']['POSITION_SOURCE']['northings_m'])**2
            + (loaded_dataset_dict['GPS']['POSITION_SOURCE']['eastings_m'] - loaded_dataset_dict['INS_INITIALLY_DVL_AIDED']['POSITION_SOURCE']['eastings_m'])**2
    )
    ## Set graph limits to ignore outliers
    ymin = -10
    ymax = 300
    axs_dvl_nav_perf_degr[1].set_ylim([ymin, ymax])
    ## Compute polynomial fits and associated R2 scores
    print("-----------------------------")
    print("INS INITIALLY DVL AIDED  poly fit model equations:")
    poly_deg_colour_dict = {
        1: "blue",
        2: "indigo",
        3: "violet",
        4: "hotpink",
    }
    # Collect inputs
    x_input = np.array(reference_df['timestamp_s'])
    y_input = np.array(ins_initially_dvl_aided_df['dist_err_gps_m'])
    # Find timestamp related to DVL cut-off point
    index_dvl_cut_off_timestamp = 0
    try:
        target_ins_df = loaded_dataset_dict['INS_INITIALLY_DVL_AIDED']['POSITION_SOURCE']
        index_dvl_cut_off_lat_match = target_ins_df[
           target_ins_df['lat_dec']==ins_initially_dvl_aided_cut_off_lat_dec
        ].index.values
        if len(index_dvl_cut_off_lat_match) == 1:
            index_dvl_cut_off_timestamp = index_dvl_cut_off_lat_match
        else:
            raise Exception("Warning: found multiple or no matches for DVL cut off lat")
    except Exception as e:
        print("Caught error: {e}\nNot setting DVL cut-off point")
    timestamp_dvl_cut_off = x_input[index_dvl_cut_off_timestamp]
    # Find timestamp related to DVL pick-up point
    index_dvl_pick_up_timestamp = -1
    try:
        target_ins_df = loaded_dataset_dict['INS_INITIALLY_DVL_AIDED']['POSITION_SOURCE']
        index_dvl_pick_up_lat_match = target_ins_df[
           target_ins_df['lat_dec']==ins_initially_dvl_aided_pick_up_lat_dec
        ].index.values
        if len(index_dvl_pick_up_lat_match) == 1:
            index_dvl_pick_up_timestamp = index_dvl_pick_up_lat_match
        else:
            raise Exception("Warning: found multiple or no matches for DVL cut off lat")
    except Exception as e:
        print("Caught error: {e}\nNot setting DVL cut-off point")
    timestamp_dvl_pick_up = x_input[index_dvl_pick_up_timestamp]
    # Ensure that there are no NaN (only finite values)
    # otherwise the polyfit cannot be computed
    index_not_nan = np.isfinite(x_input) & np.isfinite(y_input)
    # Remove outliers for a better fit
    index_no_outlier = y_input < ymax
    # Focus on range
    # between DVL aiding cut-off point / time
    # and DVL pick-up point / time
    index_no_dvl_post_cut_off = x_input > timestamp_dvl_cut_off 
    index_no_dvl_pre_pick_up = x_input < timestamp_dvl_pick_up
    # Group all conditions
    idx_selected = (index_not_nan 
        & index_no_outlier 
        & index_no_dvl_post_cut_off
        & index_no_dvl_pre_pick_up
    )
    x = x_input[idx_selected]
    y = y_input[idx_selected]
    # Set start time to 0
    # for model consistency
    time_offset = x[0]
    x -= time_offset
    for deg in poly_deg_colour_dict:
        # Create polynomial model
        poly_model = np.poly1d(
            np.polyfit(
                x, 
                y, 
                deg,
            )
        )
        # Calculate R2 score for the polynomial model
        r2_score_res = r2_score(
            y,
            poly_model(x),
        )
        # Add the polynomial plot
        axs_dvl_nav_perf_degr[1].plot(
            x, 
            poly_model(x), 
            color=poly_deg_colour_dict[deg],
            marker="o",
            markersize=2,
            linewidth=2,
            label=f"poly fit deg {deg}\nwith r2 score {r2_score_res}"
        )
        print(poly_model)
    ### Add curve fitting square model
    ### Since most promising from polynomial fit
    model_param, model_param_cov = curve_fit(dvl_influence_model, x, y)
    model_y = dvl_influence_model(x, model_param[0], model_param[1])
    r2_score_res = r2_score(
        y,
        model_y,
    )
    axs_dvl_nav_perf_degr[1].plot(
        x, 
        model_y, 
        color="lightgreen",
        marker="o",
        markersize=2,
        linewidth=2,
        label=f"Cubic fit\nwith r2 score {r2_score_res}"
    )
    print(f"\nCubic fit:y = {model_param[0]}*x^3 + {model_param[1]}")
    ### Add distance error
    axs_dvl_nav_perf_degr[1].scatter(
        x,
        y,
        color="grey",
        s=5,
        label="Sensor data",
        zorder=len(poly_deg_colour_dict) + 1,
    )
    ### Add labels and legend
    axs_dvl_nav_perf_degr[1].set_xlabel("Timestamp elasped between DVL aiding cut-off and pick-up (s)")
    axs_dvl_nav_perf_degr[1].set_ylabel("Distance error GPS vs INS (m)")
    axs_dvl_nav_perf_degr[1].legend(loc="upper right")
    axs_dvl_nav_perf_degr[1].grid()

    # Display plots
    plt.show()

def dvl_influence_model(x, a, b):
    return a*np.power(x, 3) + b

if __name__ == '__main__':
    main()

