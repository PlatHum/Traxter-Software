
"""Launch the joy node with default configuration."""

import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():
    config_directory = os.path.join(
        ament_index_python.packages.get_package_share_directory('traxter_ps3_controller'),
        'config')
    joy_params = os.path.join(config_directory, 'joy-params.yaml')
    teleop_params = os.path.join(config_directory, 'teleop-params.yaml')
    joy_node = launch_ros.actions.Node(package='joy',
                                       executable='joy_node',
                                       output='both',
                                       parameters=[joy_params])

    teleop_node=launch_ros.actions.Node(
            package='teleop_twist_joy', executable='teleop_node',
            name='teleop_twist_joy_node', parameters=[teleop_params])

    return launch.LaunchDescription([joy_node, teleop_node,

                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=joy_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         )),
                                     launch.actions.RegisterEventHandler(
                                         event_handler=launch.event_handlers.OnProcessExit(
                                             target_action=teleop_node,
                                             on_exit=[launch.actions.EmitEvent(
                                                 event=launch.events.Shutdown())],
                                         ))
                                     ])