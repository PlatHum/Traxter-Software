import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

   camera_node = IncludeLaunchDescription(
      PythonLaunchDescriptionSource([os.path.join(
         get_package_share_directory('realsense2_camera'), 'launch'),
         '/rs_launch.py.launch.py']),
      launch_arguments={'device_type': 'd435', 
                        'enable_pointcloud': 'true',
                        'initial_reset': 'true',
                        'align_depth': 'true',
                        'base_frame_id': 'd435',
                        'odom_frame_id': 'odom',
                        'clip_distance': '3.0',
                        'publish_tf': 'false',
                        'filters': ['disparity','spatial','temporal','hole-filling','decimation'],
                        'color_fps': '15.0',
                        'color_height': '480',
                        'color_width': '640',
                        'color_qos': 'SENSOR_DATA',
                        'depth_fps': '15.0',
                        'depth_height': '480',
                        'depth_width': '640',
                        'depth_qos': 'SENSOR_DATA',
                        'pointcloud_texture_index': '0',
                        'pointcloud_texture_stream': 'RS2_STREAM_COLOR'}.items(),
      )

   return LaunchDescription([
    camera_node
   ])
        


