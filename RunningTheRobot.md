# The TIR-ANT
![views](TIR-ANT%20Views.png)

| **Identifier** 	| **Name** 	|
|:---:	|:---:	|
| 1 	| Raspberry Pi 4B 	|
| 2 	| USB hub 	|
| 3 	| Adafruit BNO055 	|
| 4 	| Reflective Marker 	|
| 5 	| Track 	|
| 6 	| Track Mount 	|
| 7 	| RealSense D435 	|
| 8 	| Hokuyo URG-04LX-UG01 	|
| 9 	| Low-level On/Off Switch 	|
| 10 	| Soaring LiPo Battery 11.1V/2200mAh 	|
| 11 	| 10A Fuse 	|
| 12 	| 25W Buck Converter 	|
| 13 	| ESP32 DevKit C 	|
| 14 	| MD25 	|
| 15 	| EMG30 	|

## Turning it ON/OFF

The low-level of the robot (EMG30s, MD25, ESP32 and BNO055) can be turned on or off using the low-level on/off switch at the backside of the robot. 

The Raspberry Pi 4B can be turned on by making sure the 25W buck converter is tunerd ON (there is a litle grey button on it) and connecting the USB-C from the buck converter to the power inlet of the raspberry. Turning off the low-level switch will not turn off the raspberry, the camera or the rangefinder.

From experience, to save battery, connect the sensors to the Raspberry Pi only when they are needed. Also, do not forget that the USB ports of the Raspberry Pi are not enough to power the Intel RealSense camera. Usually, both the rangefinder and the depth camera were connected/powered via the externally powered USB hub. An obvious improvement would be to find a USB hub capable of USB 3.0 instead of 2.0 for the best performance out of the depth camera.

Since everytime the low-level is turned on the BNO055 needs to be calibrated, which usually takes 3 to 4min, usually the low-level is left turned on with the Raspberry but the other sensors are turned off and on according with the mission needs to save battery.

Sometimes there is a glitch with the microROS agent and having it running and then connecting another sensor in another USB port leads to the agent crashing. This means that both the low-level and the other sensor need to be power-cycled and the microROS agent reset (which means recalibrating the BNO055).

If something feels weird, normally the best policy is to turn whatever it is off and on again. Hardware and software are weird sometimes.

## Software Pipeline

![Alt text](Software%20Pipeline.png)

Note: 
1. The ```.bashrc``` is a Bash shell script that Bash runs whenever it is started interactively. It initializes an interactive shell session. So everytime you open a new terminal, this file will be run. The Raspberry Pi of the TIR-ANT has two useful aliases. The command ```traxter``` will change directory into the ROS2 workspace of this repository and the command ```wssource``` will source a ROS2 workspace if you are on its root directory.

2. Before being able to run any of the commands explain on the following sections it is important to source the necessary contents: 
    - sourcing the ROS2 framework, one needs to run the following command: ```source /opt/ros/humble/setup.bash```  
    - source the microROS environment 
    - there is also the need to source the contents of the ROS2 workspace that contains the packages from this repository. Currently, the TIR-ANT's Raspberry Pi does this automatically everytime a terminal is opened (taking advantage of the ```.bashrc```).

Don't forget that if you make changes to the code, you need to build the workspace and re-source it on a previously opened terminal.

### Launching the Low-Level

Make sure the low-level switch is ON. You can make sure by glaning at the MD25 and determining if its lights are on. Connect the USB cable from the ESP32 to one of the Raspberry Pi's ports, this should turn the loghts of the ESP32 and the BNO055.   
On a properly sourced terminal run: ```ros2 launch traxter_bringup sensors.launch.xml sensorType:=lowlevel```  
This activates the microROS agent which will interpret the information coming out of the ESP32 directly into the ROS2 framework. This should make visible the following topics:
- ```traxter/imu/data/unprocessed``` with the information from the BNO055
- ```traxter/encoder/ticks``` with the tick count of each encoder
- ```traxter/motor/command``` topic which the ESP32 subscribes to for sending desired wheel speeds to the low-level

You should calibrate the BNO055 until all status fields from the ```traxter/imu/data/unprocessed``` message are at least at level 2. Also note that the magnetometer tends to lose calibration quality over time, so from time to time wave the robot around to re-calibrate the magnetometer. Also, note that for the best performance out of the EKF, you should change the value of the felt gravitational acceleration in its [configuration file](traxter_robot_localization/config/hardwareEKF.yaml) according to what the BNO055 is currently reading.

Don't forget that each time the microROS agent is shut-down, the USB cable connecting the low-level and the Raspberry Pi should also be turned disconnected - and for good practice the low-level switch should also be turned off.  
Note: there is some weird behaviour if the low-level switch is turned off while the USB cable is connected. Unless you are in an emergency, **first disconnect the USB cable and only after turn off the low-level switch**. The weird behaviour is related with the BNO055. If the procedure does not follow the aforementioned reccomended order, the BNO055 requires multiple power-cycles to stop it from entering an error state.

### Launching a Mission

The [```traxter_bringup```](traxter_bringup/) package is responsible for launching everything. It is advised that a careful analysis of the package's launchers be made before operating the robot. The main launcher file is the [```traxter_bringup.launch.py```](traxter_bringup/launch/traxter_bringup.launch.py). This launcher call all other more specific launchers of the same package that in turn launch all necessary nodes.  
This launcher accepts the following arguments:
- ```runType``` : which can be 
    - ```realHard``` meaning that it is running on real hardware, aka, the TIR-ANT;
    - ```fullSimul``` meaning that it is running on a worksation and the gazebo simulation will be launched;
    - ```inLoop``` meaning that it will launch everything except the exteroceptive sensors on real hardware. The simulation is launched and the depth camera and rangefinder data come from the simulation, the movement from the real robot.
- ```navType``` : which can be
    - ```manual``` meaning that the PS3 controller relevant nodes will be launched for the robot motion to be controlled manually;
    - ```auto``` meaning that some other node/package will publish on the relevant topics to make the robot move autonomously.
- ```sensorType``` which can be:
    - ```none``` meaning that no sensor wrappers need to be launched;
    - ```lowlevel``` meaning that the microROS agent will be launched to interpret the information coming from the ESP32;
    - ```hokuyo``` meaning that the Hokuyo rangefinder wrapper, the ```urg_node``` will be launched to take care of the data coming from the 2D rangefinder;
    - ```realsense``` meaning that the RealSense wrapper will be launched to take care of the data coming from the depth camera;
    - ```all``` meaning that all previous three options will be launched;
    - ```perception``` combining the ```hokuyo``` and ```realsense``` options.
- ```slamType``` which can be:
    - ```none``` meaning that no SLAM package needs to be launched;
    - ```tools``` meaning that the ```slam_toolbox``` needs to be launched;
    - ```rtab``` meaning that the ```rtabmap_ros``` SLAM package will be launched.
- ```recordFile``` which can be either ```none``` or any other name. If any other name is given as the value of the argument, the relevant topics will be recorded into a rosbag with the given name. For more information on which topics are recorded, please consult the [relevant launcher](traxter_bringup/launch/recordRosBag.launch.xml).
- there are other arguments that can be given to the launcher, namely the location of other non-default config files, or the location of the world file for simulation. Please consult the [launcher](traxter_bringup/launch/traxter_bringup.launch.py) for a look at the code.

So if the desired mission to be run on the TIR-ANT is one using the ```slam_toolbox``` package and assuming that the low-level as already been launched on another terminal and a rosbag recording of the mission is wanted with the name "example_command", the explicit command to be run is:  
```ros2 launch traxter_bringup traxter_bringup.launch.py runType:=realHard navType:=manual sensorType:=hokuyo recordFile:=example_command slamType:=tools```  
(most of these values are default and explicit commands for their value are not needed, please visit the launchers to see the default values)

Also note that the main configuration files that load values to the parameter server for the various nodes to use for the real hardware and for simulation are [here](traxter_bringup/config/defaultHardware.yaml) and [here](traxter_bringup/config/defaultSimulation.yaml), respectively. All packages with the ```config``` folder include configuration values for that package. For SLAM algorithm tuning or EKF tuning, please take a look at them.

### Notes
The RTAB-Map algorithm launch process seems to be very computationally heavy for the Raspberry Pi. Launching it normally with the process described before does not seem to work. The board runs out of memory during launch and weird behaviour happens. To get around the issue, the reccomended procedure to launch a mission using the RTAB-Map SLAM is the following:
1. Launch the low-level  
```ros2 launch traxter_bringup sensors.launch.xml sensorType:=lowlevel```   
Don't forget to have the ESP32 **and** the Hokuyo connected to the USB ports of the Raspberry or the USB-hub connected to the Raspberry. Don't forget to calibrate the BNO055 and to change the gravitational acceleration on the EKF configuration accordingly.
2. Connect the depth camera to the USB-hub and then launch the perception  
```ros2 launch traxter_bringup sensors.launch.xml sensorType:=perception```  
Wait until both the urg node and the realsense node are running.
3. Launch the pipeline without SLAM  
```ros2 launch traxter_bringup traxter_bringup.launch.py runType:=realHard navType:=manual sensorType:=none recordFile:=none slamType:=none```
4. Launch the RTAB-Map SLAM specifically  
```ros2 launch traxter_bringup slamType.launch.xml slamType:=rtab```

This should work. If not and there are a bunch of errors being printed on screen, repeat the process.

