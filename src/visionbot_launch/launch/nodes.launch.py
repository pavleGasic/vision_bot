from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
  navigation_dir = 'visionbot_navigation'
  
  return LaunchDescription(
    [
      Node(
        package=navigation_dir,
        executable="heartbeat",
        name="heartbeat_pub_node"
      ),
      Node(
        package=navigation_dir,
        executable="safety_stop",
        name="safety_stop_node"
      ),
      Node(
        package='twist_stamper',
        executable='twist_stamper',
        parameters=[{'use_sim_time': True}],
        remappings=[('/cmd_vel_in', '/cmd_vel_mux_out'),
                    ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')]
      )
    ]
  )