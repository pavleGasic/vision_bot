import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression, IfElseSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource


def generate_launch_description():
  visionbot_description = get_package_share_directory('visionbot_description')

  model_arg = DeclareLaunchArgument(
    name='model',
    default_value=os.path.join(visionbot_description, 'description', 'robot.urdf.xacro'),
    description='Absolute path to robot URDF file'
  )

  use_gui_arg = DeclareLaunchArgument(
    name='use_gui',
    default_value='false',
    description='Run Gazebo server with no gui (headless)'
  )

  world_name_arg = DeclareLaunchArgument(
    name='world_name',
    default_value='small_house'
  )

  world_path = PathJoinSubstitution([
    visionbot_description,
    'worlds',
    PythonExpression(["'", LaunchConfiguration('world_name'), ".world'"])
  ])

  model_path = str(Path(visionbot_description).parent.resolve())
  model_path += os.pathsep + os.path.join(visionbot_description, 'models')

  gazebo_resource_path = SetEnvironmentVariable(
    'GZ_SIM_RESOURCE_PATH', model_path
  )

  robot_description = ParameterValue(
    Command([
      'xacro ',
      LaunchConfiguration('model')]),
    value_type=str
  )

  robot_state_publisher_node = Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    parameters=[
      {'robot_description': robot_description},
      {'use_sim_time': True}
    ]
  )

  gz_args = IfElseSubstitution(
    LaunchConfiguration('use_gui'),
    if_value=[world_path, " -v 4 -r"],
    else_value=[world_path, " -v 4 -r -s"]
  )

  gazebo = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
      os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
    launch_arguments={"gz_args": gz_args}.items(),
  )

  gazebo_spawner = Node(
      package="ros_gz_sim",
      executable="create",
      output="screen",
      arguments=[
          "-topic", "/robot_description",
          "-name", "visionbot"
      ]
  )

  gz_ros2_arguments = os.path.join(
    visionbot_description, 'config', 'gz_ros2_bridge.yaml')

  gz_ros2_bridge = Node(
    package='ros_gz_bridge',
    executable='parameter_bridge',
    arguments=['--ros-args', '-p', f'config_file:={gz_ros2_arguments}']
  )

  gz_image_bridge = Node(
    package='ros_gz_image',
    executable='image_bridge',
    arguments=['/camera/image_raw'],
    parameters=[{'use_sim_time': True}]
  )

  twist_stamper = Node(
    package='twist_stamper',
    executable='twist_stamper',
    parameters=[{'use_sim_time': True}],
    remappings=[('/cmd_vel_in', '/cmd_vel_mux_out'),
                ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')]
  )

  rosbridge_server = IncludeLaunchDescription(
    XMLLaunchDescriptionSource(
      os.path.join(
        get_package_share_directory('rosbridge_server'),
        'launch',
        'rosbridge_websocket_launch.xml'
      )
    )
  )

  return LaunchDescription([
    model_arg,
    use_gui_arg,
    world_name_arg,
    gazebo_resource_path,
    robot_state_publisher_node,
    gazebo,
    gazebo_spawner,
    gz_ros2_bridge,
    gz_image_bridge,
    twist_stamper,
    rosbridge_server
  ])
