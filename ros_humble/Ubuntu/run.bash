#!/bin/bash

# This script is a startup script for launching ROS 2 and Gazebo simulations with the TurtleBot3 robot.

# The following lines are commented out because they are not necessary for launching ROS 2 and Gazebo simulations:
# '/usr/sbin/sshd -D' starts the SSH daemon in the background, allowing remote access to the container over SSH.
# 'service ssh start' starts the SSH service for remote access.

# Set environment variables for Gazebo and ROS 2:
# 'export SVGA_VGPU10=0' sets the SVGA (virtual graphics card) environment variable to 0.
# 'export ROS_IP=$(hostname -I | tr -d [:blank:])' retrieves the IP address of the host machine and sets it as the ROS_IP environment variable.
# 'export ROS_MASTER_URI=http://$ROS_IP:11311' sets the ROS_MASTER_URI to the ROS master running on the host's IP address.
# 'export GAZEBO_MODEL_PATH' appends the path to Gazebo models (turtlebot3_gazebo package models) to the existing GAZEBO_MODEL_PATH environment variable.
# 'export TURTLEBOT3_MODEL=waffle_pi_mw' sets the TURTLEBOT3_MODEL environment variable to 'waffle_pi_mw'.

# Note: ROS_DOMAIN_ID and the sourcing of ROS 2 setup files have no inline comments, but they are used to configure ROS 2 environment.

export SVGA_VGPU10=0
export ROS_IP=$(hostname -I | tr -d [:blank:])
export ROS_MASTER_URI=http://$ROS_IP:11311
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/user/ros2_ws/src/turtlebot3_gazebo/models
export TURTLEBOT3_MODEL=waffle_pi_mw
export ROS_DOMAIN_ID=25 

# Source ROS 2 setup file:
# 'source /opt/ros/humble/setup.bash' sources the setup.bash file of the Humble version of ROS 2.
# It sets up the necessary environment variables for ROS 2.

source /opt/ros/humble/setup.bash

# Source ROS 2 setup file:
# 'source /home/user/ros2_ws/install/local_setup.bash' sources the local_setup.bash file of the ROS 2 workspace.
# It sets up the necessary environment variables for ROS 2.

source /home/user/ros2_ws/install/local_setup.bash

# Launch Gazebo and the TurtleBot3 simulation using ROS 2:
# '/usr/bin/supervisord' starts the Supervisor daemon to manage multiple processes.
# '|' pipes the output of 'supervisord' to 'ros2 launch turtlebot3_gazebo turtlebot3_mw.launch.py'.
# 'ros2 launch turtlebot3_gazebo turtlebot3_mw.launch.py' launches the TurtleBot3 simulation in Gazebo using ROS 2.

/usr/bin/supervisord | ros2 launch turtlebot3_gazebo turtlebot3_mw.launch.py
