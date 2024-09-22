from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vroom',
            executable='fsm',
            name='fsm_node'
        ),
        Node(
            package='vroom',
            executable='motordrive',
            name='motordrive_node'
        )])