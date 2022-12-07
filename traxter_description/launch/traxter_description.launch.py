import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution

# this is the function launch  system will look for
def generate_launch_description():

    ####### DATA INPUT ##########
    urdf_file = 'traxter.urdf.xacro'
    package_description = "traxter_description"
    simTime = LaunchConfiguration('simTime')

    ####### DATA INPUT END ##########
    #print("Fetching URDF")
    robot_desc_path = os.path.join(get_package_share_directory(package_description), "urdf", urdf_file)


    # Robot State Publisher

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher_node',
        emulate_tty=True,
        parameters=[{'use_sim_time': simTime, 'ignore_timestamp': True,'robot_description': Command(['xacro ', robot_desc_path])}],
        output="screen"
    )

    # create and return launch description object
    return LaunchDescription(
        [ 
            DeclareLaunchArgument(
            'simTime',
            default_value='False',
            description='Whether to se simulation time or not on robot description.'),           
            robot_state_publisher_node
        ]
    )