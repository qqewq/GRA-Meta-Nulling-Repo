from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='robonull', executable='vision_node'),
        Node(package='robonull', executable='grasp_node'),
        Node(package='robonull', executable='nav_node'),
        Node(package='robonull', executable='meta_null_node'),
    ])
