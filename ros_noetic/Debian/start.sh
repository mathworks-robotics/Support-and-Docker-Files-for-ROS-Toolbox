#!/bin/bash

# This script starts necessary services and launches multiple processes related to ROS and the 'mw_vision_example' package.

# Start the SSH service to enable remote access to the container.
# 'service ssh start' starts the SSH service. This is helpful for remote login and management of the container.

service ssh start

# Source the 'run.bash' script.
# 'source' is used to run the 'run.bash' script in the current shell session, setting up environment variables or custom configurations.

source /home/user/run.bash

# Start the Supervisor daemon to manage multiple processes and launch the 'mw_vision_example' package.
# '/usr/bin/supervisord' starts the Supervisor daemon, which manages multiple processes specified in the 'novnc.conf' file.
# It ensures that the 'noVNC' web-based VNC server, 'TurboVNC' server, and 'fluxbox' window manager are running.

# Launch the 'mw_vision_example' package using the 'roslaunch' command.
# 'roslaunch' is a ROS command used to launch a specific ROS package with its specified launch file.
# In this case, it launches the 'mw_vision_example_newstopsign.launch' file from the 'mw_vision_example' package.
# The 'roslaunch' command automatically sets up ROS environment variables and dependencies.

/usr/bin/supervisord | roslaunch mw_vision_example mw_vision_example_newstopsign.launch
