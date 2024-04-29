ros2 launch rtabmap_launch rtabmap.launch.py rtabmap_args:="--delete_db_on_start" rgb_topic:=/camera/color/image_rect_color depth_topic:=/camera/depth/image_rect_raw camera_info_topic:=/camera/color/camera_info frame_id:=base_footprint approx_sync:=true wait_imu_to_init:=false imu_topic:=/imu qos:=1 rviz:=true use_sim_time:=true rtab_viz:=false
