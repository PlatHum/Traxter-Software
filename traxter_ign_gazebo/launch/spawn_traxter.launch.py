#!/usr/bin/python3
# -*- coding: utf-8 -*-
import random
import os

import launch_ros
from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from ament_index_python.packages import get_package_prefix


# this is the function launch  system will look for


def generate_launch_description():

    pkg_ign_gazebo_ros = get_package_share_directory('ros_ign_gazebo')
    pkg_traxter_ign_gazebo = get_package_share_directory('traxter_ign_gazebo')

    # We get the whole install dir
    # We do this to avoid having to copy or softlink manually the packages so that gazebo can find them
    description_package_name = "traxter_description"
    install_dir = get_package_prefix(description_package_name)

    # Set the path to the WORLD model files. Is to find the models inside the models folder in traxter_ign_gazebo package
    gazebo_models_path = os.path.join(pkg_traxter_ign_gazebo, 'worlds')
    # os.environ["IGN_GAZEBO_RESOURCE_PATH"] = gazebo_models_path

    if 'IGN_GAZEBO_RESOURCE_PATH' in os.environ:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] =  os.environ['IGN_GAZEBO_RESOURCE_PATH'] + ':' + install_dir + '/share' + ':' + gazebo_models_path
    else:
        os.environ['IGN_GAZEBO_RESOURCE_PATH'] =  install_dir + "/share" + ':' + gazebo_models_path

    if 'IGN_GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['IGN_GAZEBO_PLUGIN_PATH'] = os.environ['IGN_GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['IGN_GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # Position and orientation
    # [X, Y, Z]
    position = [0.0, 0.0, 0.035]
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