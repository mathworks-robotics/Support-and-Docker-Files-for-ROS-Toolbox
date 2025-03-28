#!/bin/bash

# Launch Gazebo mini-maze world with TurtleBot3
/bin/bash -c 'export TURTLEBOT3_MODEL=waffle;export ROS_DOMAIN_ID=25;export FASTRTPS_DEFAULT_PROFILES_FILE=/home/user/fastdds.xml; source /opt/ros/jazzy/setup.bash; source /home/user/ros2_ws/install/setup.bash;ros2 launch turtlebot3_gazebo empty_world.launch.py'