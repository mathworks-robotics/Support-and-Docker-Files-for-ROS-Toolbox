# Support-and-Docker-Files-for-ROS-Toolbox
This repository contains Dockerfiles, Gazebo world files and other supporting files to run the ROS Toolbox Examples which involves communication with ROS/ROS 2 Network.
The ROS Docker file consists of ROS Installation of Noetic and ROS 2 Installation of Humble along with Gazebo 11 packages.
[Docker&trade;](https://docs.docker.com/get-started/overview/) is an open platform for developing, shipping and running applications. 

## In 23b MATLAB Release, using this repo you can run the following two examples.

### Example 1
[Sign Following Robot with ROS in Simulink](https://www.mathworks.com/help/ros/ug/sign-following-robot-using-ros-simulink.html)

### Example 2
[Sign Following Robot with ROS 2 in Simulink](https://www.mathworks.com/help/ros/ug/sign-following-robot-using-ros2-simulink.html)

### MathWorks Products (https://www.mathworks.com)

Requires MATLAB&reg; release R2022b or higher
- [MATLAB&reg;](https://www.mathworks.com/products/matlab.html)
- [Simulink&reg;](https://www.mathworks.com/products/simulink.html)
- [ROS Toolbox](https://www.mathworks.com/products/ros.html)

## Clone the repository
Select the appropriate GitHub branch corresponding to your installed MATLAB version.

| MATLAB Release   | Build Status  | Branch    |
|:------------------|:---------:|----------:|
| R2025a            | :hourglass_flowing_sand:  | [R2025a](https://github.com/mathworks-robotics/Support-and-Docker-Files-for-ROS-Toolbox/tree/R2025a)    |
| R2024b or earlier | :white_check_mark:        | [R2023b-R2024b](https://github.com/mathworks-robotics/Support-and-Docker-Files-for-ROS-Toolbox/tree/R2023b-R2024b)    |


To clone the branch, run the command in terminal:
```
git clone -b <branch-name> https://github.com/mathworks-robotics/Support-and-Docker-Files-for-ROS-Toolbox.git
```

## Installation
### Installation Instructions for Docker on Linux
1. Download and Install Docker Engine: https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository.
2. To manage Docker as a non root user, follow these post installation steps for Docker Engine: https://docs.docker.com/engine/install/linux-postinstall/

### Installation Instructions for Docker on Windows
1. Install and setup WSL on your machine following this documentation: https://learn.microsoft.com/en-us/windows/wsl/install
2. Open WSL Terminal and install Docker Engine: https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository.
3. To manage Docker as a non root user, follow these post installation steps for Docker Engine: https://docs.docker.com/engine/install/linux-postinstall/

### Build the Docker Image
1. Clone this git repository to your local hard drive.
2. Open a Terminal (In linux) or WSL Terminal (In widndows) and navigate to the "Support-and-Docker-Files-for-ROS-Toolbox" folder.
3. Build the docker image: docker build -t <preferred_name> ros_noetic/Ubuntu (Ex: docker build -t my_noetic_docker_image .) 
4. The docker image is now created and is ready to use. You can see the image by running the command "docker images" in a Terminal.
5. Create a docker container of the docker image: "docker run -it --name <name_of_container> <name_of_image> Ex: docker run -it  --name ros_container my_noetic_docker_image bash
6. For docker command line options, please refer to this page: https://docs.docker.com/engine/reference/commandline/run/

### For Display Issues
If you are facing display issues, use the following commands to run the docker container
1. Allow connections from docker: xhost +local:docker
2. Run the docker container with the necessary environment variables and volume mounts to fix display issues: docker run -it --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --device /dev/snd --net=host <name_of_image>

## Examples
To learn how to communicate with Docker and ROS Toolbox, see [Sign Following Robot with ROS in Simulink](https://www.mathworks.com/help/ros/ug/sign-following-robot-using-ros-simulink.html). 
To see the docker commands see [Docker Commands](https://docs.docker.com/engine/reference/commandline/cli/)
<!--- Make sure you have a repo set up correctly if you are to follow this formatting --->

## License
<!--- Make sure you have a License.txt within your Repo --->
The license for this software is found in the license.txt file in this repository

## Community Support
You can add your questions at [MATLAB Answers](https://www.mathworks.com/matlabcentral/answers/index).

To report any issue, contact - [MathWorks Technical Support](https://www.mathworks.com/support/contact_us.html).

Copyright 2023-2025 The MathWorks, Inc.

<!--- Do not forget to the add the SECURITY.md to this repo --->
<!--- Add Topics #Topics to your Repo such as #MATLAB  --->

<!--- This is my comment --->

<!-- Include any Trademarks if this is the first time mentioning trademarked products (For Example:  MATLAB&reg; Simulink&reg; Trademark&trade; Simulink Test&#8482;) --> 

