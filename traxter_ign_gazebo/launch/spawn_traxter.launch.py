#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random

import launch_ros
from launch import LaunchDescription


# this is the function launch  system will look for


def generate_launch_description():


    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.1]
    # [Roll, Pitch, Yaw]
    orientation = [0.0, 0.0, 0.0]

    # Spawn ROBOT Set IGN Gazebo
    ign_entity_spawner_node=launch_ros.actions.Node(
        package='ros_ign_gazebo',
        executable='create',
        name='create',
        arguments=[
            '-topic', "/robot_description", 
            '-x',str(position[0]),
            '-y', str(position[1]),
            '-z', str(position[2]),
            '-R', str(orientation[0]),
            '-P', str(orientation[1]),
            '-Y', str(orientation[2])
            ]
    )
    #print("Spawning Traxter in World.")

    

    # create and return launch description object
    return LaunchDescription(
        [
            ign_entity_spawner_node
        ]

    )