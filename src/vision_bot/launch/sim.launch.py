import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

def generate_launch_description():
    package_name='vision_bot'

    model = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory(package_name), 'launch', 'model.launch.py')]),
                launch_arguments={'use_sim_time': 'true'}.items()
    )

    twist_stamper = Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in', '/diff_drive_controller/cmd_vel_unstamped'),
                    ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')]),
                launch_arguments={'gz_args': '-r empty.sdf', 'on_exit_shutdown': 'true'}.items()
    )

    spawn_model = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description',
                   '-name', 'visionbot'],
        output='screen')
    
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{'use_sim_time': True}],
        output="screen")
    
    diff_drive_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
        parameters=[{'use_sim_time': True}],
        output="screen")

    bridge_params = os.path.join(get_package_share_directory(package_name), 'config', 'gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[bridge_params],
        output='screen')
    

    return LaunchDescription([
        model,
        twist_stamper,
        gazebo,
        spawn_model,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        ros_gz_bridge,
    ])