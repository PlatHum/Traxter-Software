import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                LaunchConfiguration, LocalSubstitution,
                                PythonExpression, TextSubstitution)
from launch.conditions import LaunchConfigurationEquals, UnlessCondition

from launch_ros.substitutions import FindPackageShare

from launch.substitutions import PathJoinSubstitution

def generate_launch_description():

    world = LaunchConfiguration('world')
    runType = LaunchConfiguration('runType')
    navType = LaunchConfiguration('navType')
    odomType = LaunchConfiguration('odomType')
    configFile = LaunchConfiguration('configFile')
    runType_arg = DeclareLaunchArgument(
          'runType',
          default_value="realHard",
          description='Type of run. [fullSimul,inLoop,realHard]')

    world_arg = DeclareLaunchArgument(
          'world',
          default_value='test.sdf',
          description='World to spawn in simulation. Has to be a SDF world file inside /worlds folder')

    navType_arg = DeclareLaunchArgument(
          'navType',
          default_value='manual',
          description='Navigation type. [ manual , auto ]')

    odomType_arg = DeclareLaunchArgument(
          'odomType',
          default_value='default',
          description='Odometry calculation type. [ default , debug ]')

    configFile_arg = DeclareLaunchArgument(
          'configFile',
          default_value='default',
          description='What config parameter file to load from config directory of bringup.')

    #traxter_description_launch = IncludeLaunchDescription(
     # PythonLaunchDescriptionSource([os.path.join(
      #   get_package_share_directory('traxter_description'), 'launch',
       #  'traxter_description.launch.py')])
      #)

    runType_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_bringup'), 'launch','runType.launch.xml')],
            launch_arguments={
                'world': world,
                'runType': runType,
                'configFile':configFile
            }.items()
        )

    navType_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_bringup'), 'launch','navType.launch.xml')],
            launch_arguments={
                'navType': navType
            }.items()
        )

    odometry_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_odometry'), 'launch','traxter_odometry.launch.xml')],
            launch_arguments={
                'odomType': odomType,
                'configFile':configFile,
                'runType': runType
            }.items()
        )

    imu_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_imu_interpreter'), 'launch','imu_interpreter.launch.xml')],
            launch_arguments={
                'configFile':configFile,
                'runType': runType
            }.items()
        )

    kinematics_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_kinematics'), 'launch','traxter_kinematics.launch.xml')],
            launch_arguments={
                'configFile':configFile,
                'runType': runType
            }.items()
        )

    robot_localization_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_robot_localization'), 'launch','traxter_robot_localization.launch.xml')],
            launch_arguments={
                'runType': runType
            }.items()
        )


    return LaunchDescription([
        world_arg,
        runType_arg,
        navType_arg,
        odomType_arg,
        configFile_arg,
        runType_launch,
        navType_launch,
        odometry_launch,
        imu_launch,
        kinematics_launch,
        robot_localization_launch

    ])

    #'ign_args': ['-r -v 4 ', PathJoinSubstitution([pkg_traxter_ign_gazebo, 'worlds', world])]