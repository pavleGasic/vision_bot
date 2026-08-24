import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

  use_sim_time_arg = DeclareLaunchArgument(
    'use_sim_time',
    default_value='True'
  )

  use_sim_time = LaunchConfiguration('use_sim_time')

  lifecycle_nodes = ['controller_server', 'planner_server', 'smoother_server', 'bt_navigator']

  controller_server_arg = DeclareLaunchArgument(
    'controller_server',
    default_value=os.path.join(
      get_package_share_directory('visionbot_navigation'),
      'config',
      'controller_server.yaml'
    ),
    description='Path to the nav2 controller server configuration'
  )

  controller_server_config = LaunchConfiguration('controller_server')

  planner_server_arg = DeclareLaunchArgument(
    'planner_server',
    default_value=os.path.join(
      get_package_share_directory('visionbot_navigation'),
      'config',
      'planner_server.yaml'
    ),
    description='Path to the nav2 planner server configuration'
  )

  planner_server_config = LaunchConfiguration('planner_server')

  smoother_server_arg = DeclareLaunchArgument(
    'smoother_server',
    default_value=os.path.join(
      get_package_share_directory('visionbot_navigation'),
      'config',
      'smoother_server.yaml'
    ),
    description='Path to the nav2 smoother server configuration'
  )

  smoother_server_config = LaunchConfiguration('smoother_server')

  bt_navigator_arg = DeclareLaunchArgument(
      'bt_navigator',
      default_value=os.path.join(
        get_package_share_directory('visionbot_navigation'),
        'config',
        'bt_navigator.yaml'
      ),
      description='Path to the nav2 bt navigation configuration'
    )

  bt_navigator_config = LaunchConfiguration('bt_navigator')

  nav2_controller_server = Node(
    package='nav2_controller',
    executable='controller_server',
    name='controller_server',
    output='screen',
    parameters=[controller_server_config, {'use_sim_time': use_sim_time}]
  )

  nav2_planner_server = Node(
    package='nav2_planner',
    executable='planner_server',
    name='planner_server',
    output='screen',
    parameters=[planner_server_config, {'use_sim_time': use_sim_time}]
  )

  nav2_smoother_server = Node(
    package='nav2_smoother',
    executable='smoother_server',
    name='smoother_server',
    output='screen',
    parameters=[smoother_server_config, {'use_sim_time': use_sim_time}]
  )

  nav2_bt_navigator = Node(
    package='nav2_bt_navigator',
    executable='bt_navigator',
    name='bt_navigator',
    output='screen',
    parameters=[bt_navigator_config, {'use_sim_time': use_sim_time}]
  )

  nav2_lifecycle_manager = Node(
    package='nav2_lifecycle_manager',
    executable='lifecycle_manager',
    name='lifecycle_manager_navigation',
    output='screen',
    parameters=[
      {'node_names': lifecycle_nodes},
      {'use_sim_time': use_sim_time},
      {'autostart': True}
    ]
  )

  return LaunchDescription([
    use_sim_time_arg,
    controller_server_arg,
    planner_server_arg,
    smoother_server_arg,
    bt_navigator_arg,
    nav2_controller_server,
    nav2_planner_server,
    nav2_smoother_server,
    nav2_bt_navigator,
    nav2_lifecycle_manager
  ])
