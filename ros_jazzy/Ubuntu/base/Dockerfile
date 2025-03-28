# Use the official OSRF ROS Jazzy Desktop image as the base image
FROM osrf/ros:jazzy-desktop

# Set the default shell to be used within the container
SHELL ["/bin/bash", "-c"]

# ROS 2 Jazzy Installation
# Set environment variables for ROS 2, and locale settings
RUN apt-get update &&  apt install -y locales
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

# Update the system and install ROS Jazzy Desktop
RUN apt update
RUN apt upgrade
RUN apt install -y ros-jazzy-desktop

# Install additional ROS packages and dependencies
RUN apt-get update && apt-get install -y \
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


# Install additional DDS
RUN apt-get update && apt-get install -y \
      ros-jazzy-rmw-cyclonedds-cpp \
      ros-jazzy-rmw-fastrtps-dynamic-cpp
	
###############################
#### Set Up ROS Environment ####
###############################

ENV SVGA_VGPU10=0

# Create a directory for the ROS 2 workspace and copy source code
RUN mkdir -p /home/user/ros2_ws/src
COPY src /home/user/ros2_ws/src
COPY examples /home/user

# Build the ROS 2 workspace
RUN cd /home/user/ros2_ws && \
    source /opt/ros/jazzy/setup.bash && \
    colcon build --symlink-install

# Uncomment to set cyclonedds as rmw implementation
#ENV RMW_IMPLEMENTATION="rmw_cyclonedds_cpp"

# Root user
RUN apt-get update && apt-get install -y openssh-server nano net-tools
RUN mkdir /var/run/sshd
RUN echo 'root:password' | chpasswd
RUN sed -i 's/PermitRootLogin prohibit-password/PermitRootLogin yes/' /etc/ssh/sshd_config
RUN sed 's@session\s*required\s*pam_loginuid.so@session optional pam_loginuid.so@g' -i /etc/pam.d/sshd

# non-root user
# Create a new user 'user' and set password
RUN useradd -rm -d /home/user -s /bin/bash -G sudo -u 1001 user \
    && echo 'user:password' | chpasswd
# Configure sudo access for the new user
RUN echo 'user ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
# Set environment variable
ENV NOTVISIBLE="in users profile"
RUN echo "export VISIBLE=now" >> /etc/profile
RUN sudo chown -R user:sudo /home/user


# VNC Configuration
COPY novnc.conf /etc/supervisor/conf.d/novnc.conf

# Install applications for VNC.
RUN \
  apt-get update \
  && apt-get -y upgrade \
  && apt-get -y install \
  libglu1-mesa \
  libgl1-mesa-dri \
  libxv1 \
  x11-utils \
  mesa-utils \
  supervisor \ 
  fluxbox \
  wget \
  ca-certificates \
  net-tools \
  x11vnc \
  xterm \
  xvfb \
  && apt-get clean

# Install TurboVNC and noVNC
RUN \
  mkdir -p /opt/noVNC/utils/websockify \
  && wget -qO- "https://github.com/novnc/noVNC/archive/refs/tags/v1.2.0.tar.gz" \
     | tar -zx --strip-components=1 -C /opt/noVNC \
  && wget -qO- "https://github.com/kanaka/websockify/tarball/master" \
     | tar -zx --strip-components=1 -C /opt/noVNC/utils/websockify
RUN ln -s /opt/noVNC/vnc_lite.html /opt/noVNC/index.html 
RUN wget https://sourceforge.net/projects/virtualgl/files/2.6.5/virtualgl_2.6.5_amd64.deb 
RUN wget https://sourceforge.net/projects/turbovnc/files/2.2.6/turbovnc_2.2.6_amd64.deb 
RUN wget https://sourceforge.net/projects/libjpeg-turbo/files/2.0.6/libjpeg-turbo-official_2.0.6_amd64.deb 
RUN dpkg -i virtualgl_*.deb 
RUN dpkg -i turbovnc_*.deb 
RUN dpkg -i libjpeg-turbo-official_*.deb

ENV DISPLAY=:1
ENV DISPLAY_WIDTH 1080
ENV DISPLAY_HEIGHT 720
ENV HOME=/home/user 
ENV ROS_LOG_DIR=/tmp/.ros/log

WORKDIR /home/user

EXPOSE 22
EXPOSE 8080

CMD ["/bin/bash", "-c", "/usr/bin/supervisord"]