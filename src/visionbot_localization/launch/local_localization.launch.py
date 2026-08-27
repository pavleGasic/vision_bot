import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='true',
    description='Use simulation (Gazebo) clock if true'
  )

  ekf_config_arg = DeclareLaunchArgument(
    'ekf_config',
    default_value=os.path.join(
      get_package_share_directory('visionbot_localization'),
      'config',
      'ekf.yaml'
    )
  )

  use_sim_time = LaunchConfiguration('use_sim_time')
  ekf_config = LaunchConfiguration('ekf_config')

  local_localization_node = Node(
    package="robot_localization",
    executable="ekf_node",
    name="ekf_filter_node",
    output="screen",
    parameters=[
      ekf_config,
      {'use_sim_time': use_sim_time}
    ]
  )

  return LaunchDescription([
    use_sim_time_arg,
    ekf_config_arg,
    local_localization_node
  ])
