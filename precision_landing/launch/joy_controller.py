from launch import LaunchDescription
from launch_ros.actions import Node
import os 

def generate_launch_description():
    joy_node = Node(
    package='joy',
    executable='joy_node')

    return LaunchDescription([
        joy_node
    ])


    