from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_bot",
                executable="heartbeat",
                name="heartbeat_pub_node"
            )
        ]
    )