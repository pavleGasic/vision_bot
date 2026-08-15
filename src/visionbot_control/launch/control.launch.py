import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
  visionbot_control = get_package_share_directory('visionbot_control')
    
  joint_state_broadcaster_spawner = Node(
    package='controller_manager',
    executable='spawner',
    arguments=[
      "joint_state_broadcaster",
      "--controller-manager",
      "/controller_manager"
    ]
  )
  
  diff_drive_controller_spawner = Node(
    package="controller_manager",
    executable="spawner",
    arguments=[
        "diff_drive_controller",
        "--controller-manager",
        "/controller_manager"
    ]
  )
  
  twist_mux_launch = IncludeLaunchDescription(
    os.path.join(get_package_share_directory('twist_mux'),
    'launch',
    'twist_mux_launch.py'
    ),
    launch_arguments={
      "cmd_vel_out": "/cmd_vel_mux_out",
      "config_topics": os.path.join(visionbot_control, 'config', 'twist_mux_topics.yaml'),
      "config_locks": os.path.join(visionbot_control, 'config', 'twist_mux_locks.yaml'),
      'use_sim_time': 'true'
    }.items()
  )
  
  heartbeat_node = Node(
    package='visionbot_control',
    executable='heartbeat',
    name='heartbeat_node',
    output="screen"
  )
  
  safety_stop = Node(
    package='visionbot_control',
    executable='safety_stop',
    name='safety_stop_node',
    output="screen"
  )
  
  return LaunchDescription([
    joint_state_broadcaster_spawner,
    diff_drive_controller_spawner,
    twist_mux_launch,
    heartbeat_node,
    safety_stop
  ])