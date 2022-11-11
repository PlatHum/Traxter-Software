# TIR-ANT's Software Dependencies
This document lists and explains the software setup of the Raspberry Pi from the flashing of the Ubuntu OS to the setup of the ROS2 workspace and finally the necessary packages for the TIR-ANT to run normally.

All of the information presented here for the Raspberry Pi can be mirrored for the user's workstation. The worksation is not relevant for computing. The TIR-ANT makes all of its calculations. However, to use the gazebo simulation, a computer with non-ARM chip architecture is required - normally the user's worksation - running TIR-ANT's software pipeline.

## Operating System
The [32GB microSD card ](https://www.amazon.es/-/pt/dp/B07RMXNLF4/ref%3Dsr_1_1?ascsubtag%3Dtomshardware-row-9175353303813734000-20%26geniuslink%3Dtrue%26keywords%3Dsilicon%2Bpower%2B32gb%2B3d%2Bnand%2Bhigh%2Bspeed%2Bmicrosd%2Bcard%2Bwith%2Badapter%26qid%3D1668184829%26qu%3DeyJxc2MiOiIxLjcyIiwicXNhIjoiMC45OSIsInFzcCI6IjAuMDAifQ%3D%3D%26sr%3D8-1) on the Raspberry Pi 4B 8GB was flashed with [Ubuntu 22.04.1 (Jammy Jellyfish) Desktop version](https://releases.ubuntu.com/22.04/). It can easily be the Server version instead and it would free up memory, but in the beginning of the project, for the creator's sanity, a GUI was needed for an easier debugging process.

Up until very recently, [Ubuntu MATE](https://ubuntu-mate.org/raspberry-pi/) was the optimised version of Ubuntu OS for Raspberry Pi boards. However, Canonical (makers of Ubuntu 22.04.1) promised that this newer vanilla version would perform better than ever on limited hardware. Furthermore, since the Jammy Jellyfish version of Ubuntu MATE would only be released some months after the release of the vanilla Ubuntu Jammy and urgency was of the essence, the robot's creator chose the aforementioned Ubuntu 22.04.1 Desktop version as the operating system.

For future work, perhaps a server version of the OS would improve performance, having the Raspberry Pi not worrying on GUI processes. Also, Ubuntu MATE might be a good alternative that would even improve overall performance.

## ROS2 Packages

Well, first, you should have [ROS2 Humble](https://docs.ros.org/en/humble/index.html) installed. The installation on the TIR-ANT was made from the Debian packages and not from source. The desktop version was installed.

The contents of this repository should be cloned into the ```src``` folder a ROS2 Humle workspace.

Now a list of the installed packages and correspondent commands to be run will be provided. Note that some of these packages might not be in use in the current state of the robot but at some point they were. This list is meant to be expanded as the robot keeps being worked on.

### Miscellaneous

- Colcon builder (```sudo apt install python3-colcon-common-extensions```)
- Navigation package (```sudo apt install ros-humble-navigation2``` and ```sudo apt install ros-humble-nav2-bringup```)
- Control package (```sudo apt install ros-humble-ros2-control``` and ```sudo apt install ros-humble-ros2-controllers```)
- Simulation manual control (```sudo apt install ros-humble-joint-state-publisher-gui```)

### SLAM related

- Google's Cartographer package (```sudo apt install ros-humble-cartographer-ros```)
- SLAM Toolbox package (```sudo apt install ros-humble-slam-toolbox```)
- RTABMap package (```sudo apt install ros-humble-octomap``` and ```sudo apt install ros-humble-octomap-ros``` and ```sudo apt install ros-humble-rtabmap``` and ```sudo apt install ros-humble-rtabmap-ros```) Some of these commands are redundant, however the successful installation of RTABMap was a arduous process and what finally worked was installing it from [source](https://github.com/introlab/rtabmap)

### Kalman Filter

- Robot Localisation package (```sudo apt install ros-humble-robot-localization```)

### PS3 controller

- Joy package (```sudo apt install ros-humble-joy```)

### Sensor Drivers

- For Hokuyo 2D Laser Rangefinder, in ```src``` of ROS2 workspace:
    1. ```git clone -b ros2-devel https://github.com/ros-perception/laser_proc.git```
    2. ```git clone -b ros2-devel https://github.com/ros-drivers/urg_c.git```
    3. ```git clone -b master https://github.com/ros-drivers/urg_node_msgs.git```
    4. ```git clone -b ros2-devel  https://github.com/ros-drivers/urg_node.git```
    5. The package from step 4 is not Humble compatible. The necessary changes are: 
        * changed the word "node_executable" in [urg_node/launch/urg_node_launch.py](https://github.com/ros-drivers/urg_node/blob/ros2-devel/launch/urg_node_launch.py) to "executable"
        * changed the previous executable to "urg_node_driver" instead of "urg_node"
- For Intel RealSense D435:
    1. Installed librealsense library with the help of [this tutorial](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md)
    2. Installed the ROS2 wrapper with the help of [this tutorial](https://github.com/IntelRealSense/realsense-ros)

### Low Level

For the ESP32 DevKit-C to be recognised by the system it was necessary to disable the brtltty service and add the user to the dialout and tty:
1. ```systemctl stop brltty-udev.service```
2. ```sudo systemctl mask brltty-udev.service```
3. ```systemctl stop brltty.service```
4. ```systemctl disable brltty.service```
5. ```sudo usermod -a -G dialout $USER```
6. ```sudo usermod -a -G tty $USER```

### microROS

Install the microROS agent from [here]( https://micro.ros.org/docs/tutorials/core/first_application_linux/). The only steps needed are the first one and the "Creating the micro-ROS agent" one.

### Notes

After installing all the packages stated above and cloning this repository, then do not forget to build the workspace. This means running:
```colcon build --symlink-install``` on the root directory of the workspace that you cloned into.

For communication with the robot, SSH communication was used using [Visual Studio Code](https://code.visualstudio.com/)'s client. Therefore, on the robot's Raspberry an SSH server was installed:
1. ```sudo apt-get install openssh-server```
2. ```sudo systemctl enable ssh --now```

Then the robot can be accessed remotely and its missions can be launched.

For programming the ESP32, [PlatformIO](https://platformio.org/) was used on VS Code. If you use PlatformIO, don't forget to update the [udev rules](https://docs.platformio.org/en/latest/core/installation/udev-rules.html). The PlatformIO project with the code for the LowLevel is available [here](traxter_scripts/platformio/New-Traxter-Low-Level.zip)