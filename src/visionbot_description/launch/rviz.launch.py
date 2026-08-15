import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  visionbot_desc = get_package_share_directory('visionbot_description')
  
  rviz_config_path = os.path.join(visionbot_desc, 'rviz', 'global_localization.rviz')

  return LaunchDescription([
    Node(
      package='rviz2',
      executable='rviz2',
      name='rviz2',
      arguments=['-d', rviz_config_path],
      output='screen'
    )
  ])