import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
  model_name = LaunchConfiguration('model_name')
  model_path = LaunchConfiguration('model_path')
  world_name = LaunchConfiguration('world_name')
  map_name = LaunchConfiguration('map_name')

  model_name_arg = DeclareLaunchArgument(
    'model_name',
    default_value='yolov8n',
    description='Model name used for CSV filename (e.g. yolov8n, yolov8s)'
  )

  model_path_arg = DeclareLaunchArgument(
    'model_path',
    default_value=os.path.join(
      get_package_share_directory('visionbot_perception'),
      'models',
      'yolov8n.pt'
    ),
    description='YOLO model to benchmark (.pt or .onnx file)'
  )

  world_arg = DeclareLaunchArgument(
    'world_name',
    default_value='yolo',
    description='Gazebo world to benchmark (without .world or .sdf extension)'
  )

  map_arg = DeclareLaunchArgument(
    'map_name',
    default_value='benchmark',
    description='Nav2 map name to load from visionbot_localization/maps (without .yaml extension)'
  )

  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('visionbot_description'),
        'launch', 'gazebo.launch.py'
      )
    ),
    launch_arguments={
      'use_gui': 'false',
      'world_name': world_name
    }.items()
  )

  control = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('visionbot_control'),
        'launch', 'control.launch.py'
      )
    )
  )

  local_localization = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('visionbot_localization'),
        'launch', 'local_localization.launch.py'
      )
    )
  )

  global_localization = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('visionbot_localization'),
        'launch', 'global_localization.launch.py'
      )
    ),
    launch_arguments={
      'map_name': map_name
    }.items()
  )

  navigation = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('visionbot_navigation'),
        'launch', 'navigation.launch.py'
      )
    )
  )

  perception = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('visionbot_perception'),
        'launch', 'perception.launch.py'
      )
    ),
    launch_arguments={
      'model_path': model_path
    }.items()
  )

  rviz = Node(
    package='rviz2',
    executable='rviz2',
    arguments=['-d', os.path.join(
        get_package_share_directory('nav2_bringup'),
        'rviz',
        'nav2_default_view.rviz'
      )
    ],
    output='screen',
    parameters=[{'use_sim_time': True}]
  )

  benchmark_runner = Node(
    package='visionbot_benchmark',
    executable='benchmark',
    name='benchmark_runner',
    output='screen'
  )

  metrics_logger = Node(
    package='visionbot_benchmark',
    executable='metrics_logger',
    name='metrics_logger_node',
    output='screen',
    parameters=[{
      'model_name': model_name,
      'output_dir': '/tmp/visionbot_benchmark'
    }]
  )

  benchmark_nodes = TimerAction(
    period=30.0,
    actions=[
      benchmark_runner,
      metrics_logger
    ]
  )

  return LaunchDescription([
    model_name_arg,
    model_path_arg,
    world_arg,
    map_arg,
    gazebo,
    rviz,
    control,
    local_localization,
    global_localization,
    navigation,
    perception,
    benchmark_nodes
  ])
