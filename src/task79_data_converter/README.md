# Mission Data Conversion Module

## Overview

This project relates to Task #9 (validation of MPF navigation solution) of Progeny T79 project.

The strategy used for validation is to convert collected mission data to feed and test the MPF.

### Requirements

The requirements for this module are gathered [on Sharepoint](https://nocacuk.sharepoint.com/:w:/r/sites/NOCMARSTeam/Shared%20Documents/MARS%20On-board%20Control%20Software/NOCS%20Documentation/OCS%20Simulation/ROS%20NOCS%20Simulation/Simulation%20Work/Progeny%20Task%2079%20OCSv3%20%5BApril%202024%5D/Mission%20Data%20Conversion%20Module%20for%20MPF%20-%20Progeny.docx?d=wa4dc5a4567b34af88024cf7da7843de4&csf=1&web=1&e=3LGwPV).

### Design

The design is based on [PlantUML diagrams](https://nocacuk.sharepoint.com/:f:/r/sites/NOCMARSTeam/Shared%20Documents/MARS%20On-board%20Control%20Software/NOCS%20Documentation/OCS%20Simulation/ROS%20NOCS%20Simulation/Simulation%20Work/Progeny%20Task%2079%20OCSv3%20%5BApril%202024%5D/resources/diagrams?csf=1&web=1&e=d3JgkC) describing the interactions between involved entities.

### Implementation

The implemenation is contained in [this repo](https://github.com/pranavvkumar21/task79).

## Notations

Throughout this document, we will use:

* The workspace root environment variable: `WS_ROOT=<path_to_root_of_ros2_workspace>`

## Pre-requisites

* Make sure that you have a functional ROS2 workspace ([ROS2 Humble](https://docs.ros.org/en/humble/Installation.html) is highly recommended)

For instance, confirm that you can run:

```
ros2 --help
```

* Clone [this repo](https://github.com/pranavvkumar21/task79) onto your machine in your ROS2 workspace (for instance, directly under `src/`)

* Verify all the pre-requisites listed in root of the repo you downloaded: have a look at the [root README](https://github.com/pranavvkumar21/task79/blob/main/README.md)

* Install the extra required Python packages with:

```
colcon_cd task79_data_converter &&
pip3 install -r requirements.txt
```

* Make sure that you have all the desired datasets (usually located in `task79_data_converter/data/mission_data`)

For instance, the `DVL.csv` dataset might need to be generated from raw data coming from the Syrinx

You can use the utility script provided in `task79_data_converter/tools/data_processing`:

```
colcon_cd task79_data_converter &&
cd tools/data_processing &&
python3 process_syrinx_raw_data.py -i <input_ocs_dataset> -o <output_dvl_dataset> -e <optional_ins_dataset> 
```

For more information about the command, run: `python3 process_syrinx_raw_data.py -h`

_Note: you might need to inspect and review the user configuration located at the top of `task79_data_converter/tools/data_processing/process_syrinx_raw_data.py` when generating the DVL dataset._

* Make sure that your dataset files start roughly at the same timestamp (because the first timestamp of the first dataset read by the node is used as reference for the start time of the simulation)

## Usage

1) Set the startup sensor and module config in `task79_data_converter/config` folder

* The startup module config is `task79_data_converter/config/startup_module_config.yaml`.

This is where you can specify: 
- the node spin frequency (Hz)
- the sensor data folder (containing the data extracted from the sensors, and where the folder path can be an absolute or a relative path to the module)
- the aiding data grouped stream flag (if `True` alias grouped, then aiding data is sent as one group onto a specific topic; if `False` alias non-grouped, then aiding data is split up into individual sensor types and sources, and sent onto individual topics)

* The startup sensor config is `task79_data_converter/config/startup_sensor_config.yaml`.

This config file lists all sensors whose dataset you want to load on startup.

Then, for each sensor, you can specify:
- its type
- its default covariance value (for each measurement)
- its state (enabled or disabled, where `enabled` means that the module will output the data relevant to the enabled sensor)

2) Build and source your ROS2 workspace

```
cd $WS_ROOT &&
rosdep install -i --from-path src --rosdistro humble -y &&
colcon build --symlink-install --event-handlers console_direct+ --executor sequential
source install/local_setup.bash
```

3) Ensure that the node files are executable (for handiness)

```
colcon_cd task79_data_converter &&
chmod +x task79_data_converter/*.py &&
chmod +x tools/**/*.py
```

4) Set the directory to store ROS logs

By default, ROS logs are located in `$HOME/.ros/log`.

To change the location of the ROS logs for your nodes, set the environment variable in your `$HOME/.bashrc`:

```
mkdir -p $WS_ROOT/ros2_log
echo "export ROS_LOG_DIR=$WS_ROOT/ros2_log" >> ~/.bashrc && 
source ~/.bashrc
```

5) Launch the mission data conversion module with:

```
ros2 launch task79_data_converter task79_data_converter_launch.xml
```

6) Interact with the relevant rostopics

* Echo topic `/sensor_config_status` to get an overview of the sensor status config
* Publish on topic `/module_config_update` to update the module config at runtime (NOTE: only the node frequency and aiding data grouped stream flag can be changed at runtime)
* Publish on topic `/sensor_config_update` to update the sensor config at runtime (e.g. enable and disable sensors at runtime)

7) Observe data for navigation purposes

* Echo topic `/aiding_data` for grouped POSITION sources (directed towards Fuser for Progeny project)
* Echo topic `/<sensor_type_lowercase>_<sensor_source_lowercase>_aiding_data` for non-grouped POSITION sources (directed towards Perception for Progeny project)

_For instance: `/ins_free_inertial_position_source_aiding_data` would be used to publish POSITION_SOURCE of INS_FREE_INERTIAL sensor._

_All the sensor types and sensor sources are gathered in `task79_data_converter/config/startup_dataset_config.yaml`._

_If you have a doubt about the naming convention, use the command `ros2 topic list` at runtime to visualise the topics available._

* Echo topic `/attitude_estimator` for ATTITUDE sources (directed towards Dead-Reckoner for Progeny project)
* Echo topic `/velocity_estimator` for VELOCITY sources (directed towards Dead-Reckoner for Progeny project)

## Customisation

### How to add new sensors

* Define the dataset format in `task79_data_converter/config/startup_dataset_config.yaml`

This config file contains a list of sensor types associated to their SOURCE type. For instance, the `GPS` provides a position so it is associated to a `POSITION_SOURCE`.

Each sensor source is tied to a dataset. The filename and type are defined for each sensor source. Other quantities are defined as well, and often the units or format of the quantity is defined as well to help the data converter.

* Edit the python parsing function in `task79_data_converter/task79_data_converter/formatted_data_conversion_node.py` if integrating a new format (or handling a data processing exception)

The python script uses handy functions inside classes to handle singular and useful tasks. Make use of them.

