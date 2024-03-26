set(_AMENT_PACKAGE_NAME "realsense_gazebo_plugin")
set(realsense_gazebo_plugin_VERSION "1.1.0")
set(realsense_gazebo_plugin_MAINTAINER "Sergey Dorodnicov <sergey.dorodnicov@intel.com>, Doron Hirshberg <doron.hirshberg@intel.com>, Salah-Eddine Missri <missrisalaheddine@gmail.com>, Adria Roig <adria.roig@pal-robotics.com>, Sergio Moyano <sergio.moyano@pal-robotics.com>")
set(realsense_gazebo_plugin_BUILD_DEPENDS "gazebo_ros" "rclcpp" "sensor_msgs" "image_transport" "camera_info_manager")
set(realsense_gazebo_plugin_BUILDTOOL_DEPENDS "ament_cmake")
set(realsense_gazebo_plugin_BUILD_EXPORT_DEPENDS "gazebo_ros" "rclcpp" "sensor_msgs" "image_transport" "camera_info_manager")
set(realsense_gazebo_plugin_BUILDTOOL_EXPORT_DEPENDS )
set(realsense_gazebo_plugin_EXEC_DEPENDS "gazebo_ros" "rclcpp" "sensor_msgs" "image_transport" "camera_info_manager")
set(realsense_gazebo_plugin_TEST_DEPENDS "ament_lint_auto" "ament_lint_common")
set(realsense_gazebo_plugin_GROUP_DEPENDS )
set(realsense_gazebo_plugin_MEMBER_OF_GROUPS )
set(realsense_gazebo_plugin_DEPRECATED "")
set(realsense_gazebo_plugin_EXPORT_TAGS)
list(APPEND realsense_gazebo_plugin_EXPORT_TAGS "<gazebo_ros gazebo_media_path=\"${prefix}\" gazebo_model_path=\"models\" plugin_path=\"lib\"/>")
