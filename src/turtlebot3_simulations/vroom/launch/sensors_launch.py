"""
Launch file for the sensors for turtlebot maze solving.

Written: Daniel Monteiro
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(
        package='vroom',
        executable='lidar',
        name='lidar_node'
    ),
    Node(
        package='vroom',
        executable='odom',
        name='odom_node'
    ),
    Node(
        package='vroom',
        executable='camerareader',
        name='camerareader_node'
    )])