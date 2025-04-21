To launch the particle filter, run the following

`rviz2 -d ~/ros2_ws/src/robot_localization/rviz/turtlebot_bag_files.rviz`

`ros2 launch robot_localization test_pf.py map_yaml:=src/robot_localization/maps/mac_1st_floor_9_23.yaml`

`ros2 bag play ros2_ws/src/robot_localization/bags/macfirst_floor_take_1 --clock`
