#opens serial communication through the microROS agent with the ESP32
# aka 
#launches the robot's low-level

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='micro_ros_agent',
            executable='micro_ros_agent',
            name='micro_ros_agent',
            arguments=["serial", "--dev", "/dev/ttyUSB-ESP32", "-b", "115200"]
        )
    ])
# make sure the microROS package is sourced
# /dev/ttyUSB-ESP32 is a pre-programmed mask for the ESP32 on the Raspberry Pi. If you change board's, this will not apply