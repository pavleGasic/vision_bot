import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  perception_node_config_arg = DeclareLaunchArgument(
    'perception_node_config',
    default_value=os.path.join(
      get_package_share_directory('visionbot_perception'),
      'config',
      'yolo_params.yaml'
    )
  )

  perception_node_config = LaunchConfiguration('perception_node_config')

  perception_node = Node(
    package='visionbot_perception',
    executable='yolo_detector',
    name='yolo_detector',
    output='screen',
    parameters=[
      perception_node_config
    ]
  )

  return LaunchDescription([
    perception_node_config_arg,
    perception_node
  ])
