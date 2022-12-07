#!/usr/bin/python3
# -*- coding: utf-8 -*-
import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_prefix
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
import launch_ros

def generate_launch_description():

    
    args=[
            #"/cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist",
            "/front_right_wheel/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double",
            "/front_left_wheel/cmd_vel@std_msgs/msg/Float64]ignition.msgs.Double",  
            "/lidar@sensor_msgs/msg/LaserScan[ignition.msgs.LaserScan",
            #"/lidar/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked",
            '/world/traxter_world/model/traxter/joint_state@sensor_msgs/msg/JointState[ignition.msgs.Model',
            '/odometry/true@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            #'/odometry/noisy@nav_msgs/msg/Odometry[ignition.msgs.Odometry',
            '/world/traxter_world/model/traxter/link/base_link/sensor/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/world/traxter_world/model/traxter/link/base_link/sensor/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/traxter_world/model/traxter/link/base_link/sensor/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/world/traxter_world/model/traxter/link/base_link/sensor/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
            '/imu@sensor_msgs/msg/Imu[ignition.msgs.IMU',
            '/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
            #'/model/traxter/tf@tf2_msgs/msg/TFMessage[ignition.msgs.Pose_V'# 
        ]
    remaps=[
                ('/lidar', '/scan'),#'/traxter/simulation/lidar/laser_scan'),
                #('/lidar/points', '/traxter/simulation/lidar/point_cloud'),
                ('/odometry/true','traxter/simulation/odometry/true'),
                #('/odometry/noisy','/odom'),#'traxter/simulation/odometry/noisy'),
                ('/front_right_wheel/cmd_vel','/traxter/simulation/command/right'),
                ('/front_left_wheel/cmd_vel','/traxter/simulation/command/left'),
                ('/world/traxter_world/model/traxter/joint_state','/joint_states'),
                ('/world/traxter_world/model/traxter/link/base_link/sensor/rgbd_camera/points','/cloud'),
                ('/world/traxter_world/model/traxter/link/base_link/sensor/rgbd_camera/image','rgb/image'),
                ('/world/traxter_world/model/traxter/link/base_link/sensor/rgbd_camera/depth_image','depth/image'),
                ('/world/traxter_world/model/traxter/link/base_link/sensor/rgbd_camera/camera_info','rgb/camera_info'),
                #('/model/traxter/tf','/tf')
                ('/imu','/traxter/simulation/imu')
            ]


    bridge_ign_ros=launch_ros.actions.Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='bridge',
        arguments=args,
        parameters=[{'use_sim_time': True}],
        remappings=remaps,
    )

    hokuyo_static_transform=launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'hokuyo', '--child-frame-id', 'traxter/base_link/hokuyo'],
            parameters=[{'use_sim_time': True}]
    )
    
    realsense_static_transform=launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '-1.570796327', '--pitch', '0', '--roll', '4.71238898', '--frame-id', 'd435_link', '--child-frame-id', 'traxter/base_link/rgbd_camera'],
            parameters=[{'use_sim_time': True}]
            
    )

    imu_static_transform=launch_ros.actions.Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            arguments = ['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'imu_link', '--child-frame-id', 'traxter/base_link/imu'],
            parameters=[{'use_sim_time': True}]
    )



    return LaunchDescription([
        bridge_ign_ros,
        RegisterEventHandler(
            OnProcessStart(
                target_action=bridge_ign_ros,
                on_start=[
                    LogInfo(msg='Bridge between Gazebo and ROS built.')
                ]
            )
        ),
        hokuyo_static_transform,
        realsense_static_transform,
        imu_static_transform        
    ])