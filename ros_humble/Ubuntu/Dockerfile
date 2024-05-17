# Use the official OSRF ROS Humble Desktop image as the base image
FROM osrf/ros:humble-desktop

# Set the default shell to be used within the container
SHELL ["/bin/bash", "-c"]

# Set environment variables for ROS 2, Gazebo, and locale settings
RUN apt update &&  apt install locales
RUN locale-gen en_US en_US.UTF-8
RUN update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
RUN export LANG=en_US.UTF-8

# Verify locale settings
RUN locale

# Update the system and install necessary packages
RUN apt-get update; \
    apt-get -y upgrade; \
    apt-get install -y gnupg2 wget lsb-release

# Uncomment the following line if using a specific keyserver to fetch ROS 2 repository keys
#RUN apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-keys 871920D1991BC93C

# Install software-properties-common and add the Universe repository
RUN apt-get install -y software-properties-common
RUN add-apt-repository universe

# Update the system and install curl for further setup
RUN apt update && apt install -y curl
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Remove any existing ROS 2 source list files that might conflict
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list

# Add ROS 2 apt repository to the sources list
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update the system and install ROS Humble Desktop
RUN apt update
RUN apt upgrade
RUN apt install -y ros-humble-desktop

# Install additional ROS packages and dependencies
RUN apt update && apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools \
  python3-flake8-blind-except \
  python3-flake8-builtins \
  python3-flake8-class-newline \
  python3-flake8-comprehensions \
  python3-flake8-deprecated \
  python3-flake8-import-order \
  python3-flake8-quotes \
  python3-pytest-repeat \
  python3-pytest-rerunfailures \
  python3-colcon-common-extensions

# Install Gazebo and related ROS packages
RUN apt-get update && apt-get install -y libgazebo11
RUN apt install -y ros-humble-gazebo-ros-pkgs
RUN apt install -y ros-humble-turtlebot3
RUN apt install -y ros-humble-turtlebot3-gazebo
RUN apt install -y ros-humble-turtlebot3-simulations ros-humble-turtlebot3-gazebo ros-humble-turtlebot3-teleop

# Install ROS 2 navigation, cartographer, and other related packages
RUN apt-get update &&\
  apt-get install -y \
    ros-humble-ros2launch \
    ros-humble-turtlebot3-navigation2 \
    ros-humble-turtlebot3-cartographer \
    ros-humble-turtlebot3-msgs \
    ros-humble-turtlebot3-description \
    ros-humble-imu-sensor-broadcaster \
    ros-humble-diff-drive-controller \
    ros-humble-effort-controllers \
    ros-humble-joint-state-broadcaster \
    ros-humble-ros2controlcli \
    ros-humble-controller-manager \
    ros-humble-velocity-controllers \
    ros-humble-xacro \
    ros-humble-joint-trajectory-controller \
    ros-humble-rviz2 \
    ros-humble-tricycle-controller \
    ros-humble-gps-msgs

###############################
#### Set Up ROS Environment ####
###############################

ENV SVGA_VGPU10=0

# Create a directory for the ROS 2 workspace and copy source code
RUN mkdir -p /home/user/ros2_ws/src
COPY src /home/user/ros2_ws/src

# Build the ROS 2 workspace
RUN cd /home/user/ros2_ws && \
    source /opt/ros/humble/setup.bash && \
    colcon build --symlink-install

# Set environment for TurtleBot3 model
ENV TURTLEBOT3_MODEL=waffle_pi

# Set environment variables for display server and TurboVNC
ENV DISPLAY=:1
EXPOSE 8080

# Copy Supervisor configuration for noVNC
COPY novnc.conf /etc/supervisor/conf.d/novnc.conf

# Create a new user and set up SSH for remote access (Note: Some lines are commented out as they may cause permission issues)
# Replace 'password' with an actual password for the root user (not recommended for production)
RUN echo 'root:password' | chpasswd
# Uncomment the following line if you want to enable remote SSH login for root (not recommended for production)
#RUN sed -i 's/#\?PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# Install additional tools (Conky, Vim, ifconfig, ping, etc.)
RUN apt -y install conky-all
RUN apt -y install vim-gtk3
RUN apt -y install net-tools
RUN apt -y install iputils-ping

# Set environment variable for TurtleBot3 model
ENV TURTLEBOT3_MODEL=waffle_pi

### TurboVNC + VirtualGL + NoVNC ###

# Install required packages for GUI applications and X11 support

RUN apt-get update \
    # Update the package index to ensure that the package lists are up to date.
    && apt-get install -y --no-install-recommends \
       # Install the following packages with 'apt-get install':
       # - 'libglu1-mesa': Mesa OpenGL utility library (GLU).
       # - 'libgl1-mesa-glx': Mesa OpenGL extension (GLX) runtime libraries.
       # - 'libgl1-mesa-dri': Mesa Direct Rendering Infrastructure (DRI) runtime libraries.
       # - 'libxv1': X11 Video extension library.
       # - 'x11-utils': Miscellaneous X11 utilities.
       # - 'mesa-utils': Miscellaneous Mesa GL utilities.
       # - 'supervisor': Process control system for managing multiple processes.
       # - 'fluxbox': Lightweight and highly configurable window manager for X11.
       # - 'wget': Command-line utility for downloading files from the web.
       # - 'ca-certificates': Common CA certificates for enabling secure connections.
       # - 'python3': Python interpreter.
       # - 'python3-numpy': NumPy library for numerical computing with Python.
       # - 'x11vnc': VNC server for sharing the X display.
       # - 'xterm': X terminal emulator.
       # - 'xvfb': Virtual Framebuffer X server.
       libglu1-mesa \
       libgl1-mesa-glx \
       libgl1-mesa-dri \
       libxv1 \
       x11-utils \
       mesa-utils \
       supervisor \ 
       fluxbox \
       wget \
       ca-certificates \
       python3 \
       python3-numpy \
       x11vnc \
       xterm \
       xvfb \
    && rm -rf /var/lib/apt/lists/*
    # Remove the downloaded package lists from '/var/lib/apt/lists/'
    # to reduce the image size. The package lists are not needed after installation.

# Download and install noVNC, VirtualGL, and TurboVNC

# Create a directory named 'websockify' inside '/opt/noVNC/utils' to store noVNC utility files.
# Download the noVNC repository archive (version 1.2.0) from GitHub and extract its contents.
# The 'wget' command downloads the archive from the provided URL.
# The '-qO-' option tells 'wget' to be quiet during download and output the result to stdout.
# The output is then passed to 'tar' command through the pipe '|' symbol.
# The 'tar' command extracts the downloaded archive.
# The '--strip-components=1' flag is used to remove the first directory component (noVNC-version) during extraction.
# This ensures that the contents are placed directly in the specified directory '/opt/noVNC/' without an additional subdirectory.
# The '-zx' options of 'tar' specify that it should perform decompression (z) and extract (x) actions.
# The 'C' option is used to specify the target directory where the contents will be extracted, i.e., '/opt/noVNC/'.
# Create a symbolic link named 'index.html' inside '/opt/noVNC/' that points to 'vnc_lite.html'.
# This allows accessing the noVNC client by simply opening '/opt/noVNC/' in a web browser.
# Download the VirtualGL, TurboVNC, and libjpeg-turbo packages from SourceForge using 'wget'.
# The packages are required for enabling OpenGL graphics acceleration in virtual environments.
# The 'dpkg' command is used to install the downloaded '.deb' packages.
RUN mkdir -p /opt/noVNC/utils/websockify \
 && wget -qO- "https://github.com/novnc/noVNC/archive/refs/tags/v1.2.0.tar.gz" \
    | tar -zx --strip-components=1 -C /opt/noVNC \
 && ln -s /opt/noVNC/vnc_lite.html /opt/noVNC/index.html \
 && wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb \
 && wget https://sourceforge.net/projects/turbovnc/files/2.2.6/turbovnc_2.2.6_amd64.deb \
 && wget https://sourceforge.net/projects/libjpeg-turbo/files/2.0.6/libjpeg-turbo-official_2.0.6_amd64.deb \
 && dpkg -i virtualgl_*.deb \
 && dpkg -i turbovnc_*.deb \
 && dpkg -i libjpeg-turbo-official_*.deb

# Configure & run supervisor
# (Note: The 'CMD' for starting supervisord is commented out. Make sure to uncomment it if necessary)

RUN mkdir /var/run/sshd

# Copy custom scripts for execution
COPY run.bash /home/user/run.bash
COPY start.sh /home/user/start.sh
RUN chmod +x /home/user/run.bash
RUN chmod +x /home/user/start.sh
CMD ["/home/user/start.sh"]
