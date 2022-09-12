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
    sensorType= LaunchConfiguration('sensorType')
    navType = LaunchConfiguration('navType')
    slamType = LaunchConfiguration('slamType')
    odomType = LaunchConfiguration('odomType')
    configFile = LaunchConfiguration('configFile')
    recordFile = LaunchConfiguration('recordFile')


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

    slamType_arg = DeclareLaunchArgument(
          'slamType',
          default_value='none',
          description='Navigation type. [ none, carto, tools, rtab, none ]')

    odomType_arg = DeclareLaunchArgument(
          'odomType',
          default_value='default',
          description='Odometry calculation type. [ default , debug ]')

    configFile_arg = DeclareLaunchArgument(
          'configFile',
          default_value='default',
          description='What config parameter file to load from config directory of bringup.')

    recordFile_arg = DeclareLaunchArgument(
          'recordFile',
          default_value='none',
          description='What to name the rosbag file. [none, #wanted_name#]')

    sensorType_arg = DeclareLaunchArgument(
          'sensorType',
          default_value='all',
          description='What sensors to launch. [all, perception, hokuyo, realsense, lowlevel]')

    traxter_description_launch = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('traxter_description'), 'launch',
         'traxter_description.launch.py')])
      ) 

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

    slamType_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_bringup'), 'launch','slamType.launch.xml')],
            launch_arguments={
                'runType': runType,
                'slamType': slamType
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

    sensor_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_bringup'), 'launch','sensors.launch.xml')],
            launch_arguments={
                'sensorType': sensorType
            }.items()
    )

    record_launch = IncludeLaunchDescription(
            [os.path.join(get_package_share_directory('traxter_bringup'), 'launch','recordRosBag.launch.xml')],
            launch_arguments={
                'recordFile': recordFile,
                'slamType': slamType
            }.items()
    )


    return LaunchDescription([
        world_arg,
        runType_arg,
        navType_arg,
        slamType_arg,
        odomType_arg,
        configFile_arg,
        recordFile_arg,
        sensorType_arg,
        sensor_launch,
        runType_launch,
        navType_launch,
        odometry_launch,
        imu_launch,
        robot_localization_launch,
        kinematics_launch,
        slamType_launch,
        record_launch,
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=sensor_launch,
                on_completion=[
                    LogInfo(msg='Launched Sensors.')
                ]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=runType_launch,
                on_completion=[
                    LogInfo(msg='Launched Robot Description.')
                ]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=navType_launch,
                on_completion=[
                    LogInfo(msg='Launched PS3 Controller Interpreter.')
                ]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=odometry_launch,
                on_completion=[
                    LogInfo(msg='Launched Odometry.')
                ]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=imu_launch,
                on_completion=[
                    LogInfo(msg='IMU interpretation.')
                ]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=kinematics_launch,
                on_completion=[
                    LogInfo(msg='Launched Speed-Command to Wheel-Speeds node.')
                ]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=robot_localization_launch,
                on_completion=[
                    LogInfo(msg='Launched EKF node.')
                ]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=robot_localization_launch,
                on_completion=[
                    LogInfo(msg='Launched SLAM.')
                ]
            )
        ),
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=record_launch,
                on_completion=[
                    LogInfo(msg='Started recording.')
                ]
            )
        )

    ])

    #'ign_args': ['-r -v 4 ', PathJoinSubstitution([pkg_traxter_ign_gazebo, 'worlds', world])]