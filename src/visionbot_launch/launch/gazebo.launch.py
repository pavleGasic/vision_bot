import os
from os import pathsep
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.actions import Node

from launch_xml.launch_description_sources import XMLLaunchDescriptionSource

def generate_launch_description():
    visionbot_description_dir = get_package_share_directory('visionbot_description')
    visionbot_launch_dir = get_package_share_directory('visionbot_launch')
    rosbridge_server='rosbridge_server'
    world_file=os.path.join(visionbot_description_dir, 'worlds', 'small_house.world')
    
    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(visionbot_description_dir, 'description', 'robot.urdf.xacro'),
        description="Absolute path to robot URDF file"
    )
    
    robot_description_config = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]), value_type=str
    )
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_config}]
    )
    
    nodes_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(visionbot_launch_dir, 'launch', 'nodes.launch.py'))
    )
    
    model_path = str(Path(visionbot_description_dir).parent.resolve())
    model_path += pathsep + os.path.join(visionbot_description_dir, 'models')
    
    gazebo_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', model_path
    )
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch"), "/gz_sim.launch.py"]),
        launch_arguments=[("gz_args", [" -v 4", " -r", f" {world_file}"])],
    )
    
    gazebo_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-topic", "/robot_description",
            "-name", "vision_bot"],
    )
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"])
    
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "diff_drive_controller",
            "--controller-manager",
            "/controller_manager"])
    
    twist_mux_launch = IncludeLaunchDescription(
      os.path.join(get_package_share_directory('twist_mux'),
      'launch',
      'twist_mux_launch.py'
      ),
      launch_arguments={
        "cmd_vel_out": "/diff_drive_controller/cmd_vel",
        "config_topics": os.path.join(visionbot_launch_dir, 'config', 'twist_mux_topics.yaml'),
        "config_locks": os.path.join(visionbot_launch_dir, 'config', 'twist_mux_locks.yaml'),
        "config_joy": os.path.join(visionbot_launch_dir, 'config', 'twist_mux_joy.yaml'),
        'use_sim_time': 'true'
      }.items()
    )
      
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_out', '/diff_drive_controller/cmd_vel')],
      )
    
    bridge_params = os.path.join(visionbot_launch_dir, 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_params}'])

    rosbridge_launch = IncludeLaunchDescription(
                XMLLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory(rosbridge_server),
                        'launch',
                        'rosbridge_websocket_launch.xml')))
    
    return LaunchDescription([
        model_arg,
        robot_state_publisher_node,
        nodes_launch,
        gazebo_resource_path,
        gazebo,
        gazebo_spawner,
        joint_state_broadcaster_spawner,
        diff_drive_controller_spawner,
        twist_mux_launch,
        twist_mux,
        ros_gz_bridge,
        rosbridge_launch
    ])