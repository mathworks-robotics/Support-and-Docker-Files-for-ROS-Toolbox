#!/bin/bash

# This script sets up the ROS environment variables and sources the necessary setup files for the ROS workspace.

# Get the IP address of the host machine and store it in the ROS_IP environment variable.
# The `hostname -I` command returns the list of IP addresses assigned to the host machine, and `tr -d [:blank:]` removes any whitespace characters.
export ROS_IP=$(hostname -I | tr -d [:blank:])

# Set the ROS_MASTER_URI environment variable to point to the ROS master on the host machine.
# The ROS master is the central node in the ROS network, and it runs on port 11311.
# By setting ROS_MASTER_URI to http://$ROS_IP:11311, we tell ROS nodes to communicate with the master on the host's IP address.
export ROS_MASTER_URI=http://$ROS_IP:11311

# Source the ROS setup.bash file to add ROS-specific environment variables to the current shell session.
# The setup.bash file is provided by ROS and sets up environment variables required for ROS tools and libraries.
# It is essential to source this file to use ROS commands and tools.
source /opt/ros/noetic/setup.bash

# Source the setup.bash file of the specific catkin workspace.
# A catkin workspace is a directory where ROS packages are built and organized.
# The 'devel' folder inside the catkin workspace contains setup.bash that needs to be sourced to make the packages accessible.
source /home/user/catkin_ws/devel/setup.bash
