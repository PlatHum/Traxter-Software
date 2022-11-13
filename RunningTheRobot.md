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