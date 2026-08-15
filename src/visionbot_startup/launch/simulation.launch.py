import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  use_slam = LaunchConfiguration('use_slam')

  use_slam_arg = DeclareLaunchArgument(
    'use_slam',
    default_value='false',
    description='Whether to use SLAM or not'
  )

  use_gz_gui_arg = DeclareLaunchArgument(
    'use_gui',
    default_value='false',
    description='Enable Gazebo GUI argument'
  )

  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('visionbot_description'),
        'launch',
        'gazebo.launch.py'
      )
    ),
    launch_arguments={
      'use_gui': LaunchConfiguration('use_gui')
    }.items()
  )

  control = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory('visionbot_control'),
      'launch',
      'control.launch.py'
    )
  )

  localization = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory('visionbot_navigation'),
      'launch',
      'global_localization.launch.py'
    ),
    condition=UnlessCondition(use_slam)
  )

  slam = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory('visionbot_navigation'),
      'launch',
      'slam.launch.py'
    ),
    condition=IfCondition(use_slam)
  )

  rviz = IncludeLaunchDescription(
    os.path.join(
      get_package_share_directory('visionbot_description'),
      'launch',
      'rviz.launch.py'
    )
  )

  return LaunchDescription([
    use_slam_arg,
    use_gz_gui_arg,
    gazebo,
    control,
    localization,
    slam,
    rviz
  ])
