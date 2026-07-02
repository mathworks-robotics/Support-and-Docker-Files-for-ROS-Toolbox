#!/bin/bash

# Launch Gazebo with the UR5 robot and the ros2_control trajectory action interface
/bin/bash -c 'export ROS_DOMAIN_ID=25;export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/fastdds.xml; source /opt/ros/jazzy/setup.bash; source /home/user/ros2_ws/install/setup.bash;ros2 launch ur_simulation_gz ur_sim_action.launch.py ur_type:=ur5'
