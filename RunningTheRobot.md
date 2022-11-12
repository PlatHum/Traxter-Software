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

Sometimes there is a glitch with the microROS agent and having it running and then connecting another sensor in another USB port leads to the agent crashing. This means that both the low-level and the other sensor need to be power-cycled and the microROS agent reset (which means recalibrating the BNO055 :) ).

If something feels weird, normally the best policy is to turn whatever it is off and on again. Hardware and software are weird sometimes.

## Software Pipeline

![Alt text](Software%20Pipeline.png)

_WORK IN PROGRESS_