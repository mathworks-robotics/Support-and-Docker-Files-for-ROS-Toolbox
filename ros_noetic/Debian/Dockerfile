# Use the official ROS Noetic image based on Debian 10
FROM osrf/ros:noetic-desktop-full-buster

# Set the default shell for commands to bash
SHELL ["/bin/bash", "-c"]

# Install system dependencies and ROS packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    lsb-release \
    gnupg2 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-turtlebot3 \
    ros-noetic-turtlebot3-gazebo \
    ros-noetic-turtlebot3-description \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    python3-pip \
    libgazebo11-dev \
    openssh-server \
    conky-all \
    vim-gtk3 \
    net-tools \
    iputils-ping \
    && rm -rf /var/lib/apt/lists/*

# Set the default TurtleBot3 model to 'waffle_pi'
ENV TURTLEBOT3_MODEL=waffle_pi

# Create the catkin workspace and copy the 'src' folder into it
WORKDIR /home/user/catkin_ws
COPY src ./src

# Build the catkin workspace
RUN source /opt/ros/noetic/setup.bash \
    && catkin_make

# sets the SVGA (virtual graphics card) environment variable to 0 to improve Gazebo's performance inside the container.
ENV SVGA_VGPU10=0

### TurboVNC + VirtualGL + NoVNC ###

# Install required packages for GUI applications and X11 support
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
# - 'python': Python interpreter.
# - 'python-numpy': NumPy library for numerical computing with Python.
# - 'x11vnc': VNC server for sharing the X display.
# - 'xterm': X terminal emulator.
# - 'xvfb': Virtual Framebuffer X server.

RUN \
  apt-get update \
  && apt-get -y upgrade \
  && apt-get -y install \
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
  python \
  python-numpy \
  net-tools \
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

RUN \
 mkdir -p /opt/noVNC/utils/websockify \
 && wget -qO- "https://github.com/novnc/noVNC/archive/refs/tags/v1.2.0.tar.gz" \
    | tar -zx --strip-components=1 -C /opt/noVNC \
 && wget -qO- "https://github.com/kanaka/websockify/tarball/master" \
    | tar -zx --strip-components=1 -C /opt/noVNC/utils/websockify \
 && ln -s /opt/noVNC/vnc_lite.html /opt/noVNC/index.html \
 && wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb \
 && wget https://sourceforge.net/projects/turbovnc/files/2.2.6/turbovnc_2.2.6_amd64.deb \
 && wget https://sourceforge.net/projects/libjpeg-turbo/files/2.0.6/libjpeg-turbo-official_2.0.6_amd64.deb \
 && dpkg -i virtualgl_*.deb \
 && dpkg -i turbovnc_*.deb \
 && dpkg -i libjpeg-turbo-official_*.deb

# Set an environment variable for the display server
ENV DISPLAY=:1
# Expose port 8080 for accessing the VNC server through a web browser
EXPOSE 8080

# Configure & run supervisor to manage the VNC and noVNC servers
COPY novnc.conf /etc/supervisor/conf.d/novnc.conf
CMD ["/usr/bin/supervisord"]

# Create the directory for SSH server's PID file
RUN mkdir /var/run/sshd

# Set the root password for SSH access (Note: Replace 'password' with a secure password)
RUN echo 'root:password' | chpasswd

# Enable root login with password for SSH (Note: Not recommended for production)
RUN sed -i 's/#\?PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config

# SSH login fix. Otherwise, the user is kicked off after login
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

# Expose port 22 for SSH access
EXPOSE 22

# Copy the custom bash scripts 'run.bash' and 'start.sh' and make them executable
COPY run.bash /home/user/run.bash
COPY start.sh /home/user/start.sh
RUN chmod +x /home/user/run.bash
RUN chmod +x /home/user/start.sh

# Set the 'start.sh' script as the entry point for the container
CMD ["/home/user/start.sh"]
