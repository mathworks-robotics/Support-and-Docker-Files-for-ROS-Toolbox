#!/bin/bash

# This script is a startup script for launching ROS 2 and Gazebo simulations with the TurtleBot3 robot.

# Start the SSH service to enable remote access to the container.
# 'service ssh start' starts the SSH service. This is helpful for remote login and management of the container.

# service ssh start

# Source the 'run.bash' script.
# 'source' is used to run the 'run.bash' script in the current shell session, setting up environment variables or custom configurations.

source /home/user/run.bash

# Launch Gazebo and the TurtleBot3 simulation using ROS 2:
# '/usr/bin/supervisord' starts the Supervisor daemon to manage multiple processes.
# '|' pipes the output of 'supervisord' to 'ros2 launch turtlebot3_gazebo turtlebot3_mw.launch.py'.
# 'ros2 launch turtlebot3_gazebo turtlebot3_mw.launch.py' launches the TurtleBot3 simulation in Gazebo using ROS 2.

/usr/bin/supervisord | ros2 launch turtlebot3_gazebo turtlebot3_mw.launch.py
