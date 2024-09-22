from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    # Select wall side to follow
    dict_of_options = {"Left": "Left", "Right": "Right"}
    wall_side = input(f"What side do you want to follow? (Options: {', '.join(list(dict_of_options.keys()))}): ")
    if wall_side in list(dict_of_options.keys()):
        wall_choice = dict_of_options[wall_side]
    else:
        print(f"{wall_side} not an option!. Sorry.")
        exit()

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
        ),
        Node(
            package='vroom',
            executable='fsm',
            name='fsm_node',
            arguments=[wall_choice]
        ),
        Node(
            package='vroom',
            executable='motordrive',
            name='motordrive_node'
        )])