from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="vision_bot",
                executable="heartbeat",
                name="heartbeat_pub_node"
            ),
            Node(
                package="vision_bot",
                executable="yolo_detector_node.py",
                name="yolo_detector",
                parameters=[{
                    'model': 'yolov8n.pt',
                    'confidence': 0.5,
                    'device': 'cpu'
                }],
                output="screen"
            )
        ]
    )