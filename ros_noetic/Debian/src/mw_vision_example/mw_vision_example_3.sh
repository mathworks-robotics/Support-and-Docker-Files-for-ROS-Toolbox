#!/bin/sh

export SVGA_VGPU10=0
export ROS_IP=$(hostname -I | tr -d [:blank:])
export ROS_MASTER_URI=http://$ROS_IP:11311

# Launch Gazebo world with TurtleBot3 Waffle Pi
gnome-terminal --title="Gazebo Sign Follower CoSim" -- /bin/bash -c 'source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash ; roslaunch mw_vision_example mw_vision_example_newstopsign_cosim.launch' 



