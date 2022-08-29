import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

   jsonpath=[os.path.join(
         get_package_share_directory('traxter_bringup'), 'config'),'/d435.json']
   camera_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense2_camera'), 'launch'),
         '/rs_launch.py']),
      launch_arguments={'device_type': 'd435',
      			'json_file_path':jsonpath,
      			'depth_module.profile':'640,480,15',
      			'rgb_camera.profile':'640,480,15',
      			'pointcloud.enable':'false',
                        'initial_reset': 'true',
                        'enable_pose':'false',
                        'enable_fisheye1':'false',
                        'enable_fisheye2':'false',
                        'align_depth.enable':'true',
                        'enable_sync':'true',
                        'allow_no_texture_points':'true',
                        'decimation_filter.enable':'true'}.items(),
      )

   return LaunchDescription([
    camera_node
   ])
        


