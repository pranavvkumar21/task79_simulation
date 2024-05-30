## Turtlebot3

### 1. ROS Packages

#### Turtlebot3_gazebo
modified TurtleBot3 package utilizes three distinct sensor modalities: an IMU, a laser scanner, and the RealSense D435 RGBD camera, integrated via the RealSense Gazebo plugin. This combination offers comprehensive perception capabilities, enabling the robot to gather data from different perspectives and modalities.

#### Turtlebot3_msgs
  
The turtlebot3_msgs package in ROS2 defines a set of message types used for communication between various nodes in the TurtleBot3 ecosystem. These messages encapsulate data related to sensor readings, robot state information, and control commands, enabling seamless interaction and interoperability within the ROS2 framework.
#### Turtlebot3_teleop
  
The turtlebot3_teleop package in ROS2 provides a convenient solution for teleoperating the TurtleBot3 robot using a keyboard or joystick. It offers intuitive control interfaces that allow users to command the robot's movements in real-time, facilitating manual navigation and exploration.

### 4.1 Installation

To install the turtlebot3 packages along with gazebo realsense plugin, 

**Create turtlebot worksace
```
mkdir -p ~/turtlebot3_ws/src
cd ~/turtlebot3_ws
git clone https://github.com/pranavvkumar21/task79_simulation.git
```

**Clone and build the Repository:**
``` git clone https://github.com/pranavvkumar21/task79_simulation.git
	cd ..
	colcon build
	source install/setup.bash
```

**add export turtlebot3 model**
```
export TURTLEBOT3_MODEL=waffle_pi >> ~/.bashrc
```

### 4.2 Usage

Launch turtlebot3 _gazebo

```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

In another shell run the teleop node 

```
ros2 run turtlebot3_teleop teleop_keyboard
```



## Task79 Data Converter

This ROS2 package contains the Mission Data Conversion Module aimed at converting data obtained from vehicle missions, into MPF-compliant inputs for testing purposes.

The package contains its own [README.md](src/task79_data_converter/README.md), as it can be used as a standalone module if desired.

_Note that `task79_data_converter` depends on `task79_interfaces` for its communication interface definitions._
