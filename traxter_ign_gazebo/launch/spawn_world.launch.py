#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from launch.substitutions import PathJoinSubstitution

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

    

    #print("IGN_GAZEBO_RESOURCE_PATH=="+str(os.environ["IGN_GAZEBO_RESOURCE_PATH"]))
    #print("IGN_GAZEBO_PLUGIN_PATH=="+str(os.environ["IGN_GAZEBO_PLUGIN_PATH"]))

    world = LaunchConfiguration('world')

    # Gazebo launch
    ign_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ign_gazebo_ros, 'launch', 'ign_gazebo.launch.py'),
        ),
        launch_arguments={
            'ign_args': ['-r ', PathJoinSubstitution([gazebo_models_path, world])]
        }.items(),
    )

    # ign_gazebo = IncludeLaunchDescription(
    #    PythonLaunchDescriptionSource(
    #        os.path.join(pkg_ign_gazebo_ros, 'launch', 'ign_gazebo.launch.py'),
    #    ),
    #    launch_arguments={
    #        'ign_args': [PathJoinSubstitution([pkg_traxter_ign_gazebo, 'worlds', world])]
    #    }.items(),
    #)


    return LaunchDescription([
        DeclareLaunchArgument(
          'world',
          default_value='test_v2.sdf',
          description='World to spawn. Has to be a SDF world file inside /worlds folder'),
        ign_gazebo
    ])
    #'ign_args': ['-r -v 4 ', PathJoinSubstitution([pkg_traxter_ign_gazebo, 'worlds', world])]