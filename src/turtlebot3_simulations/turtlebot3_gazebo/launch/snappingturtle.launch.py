"""
This is the launch file for Project 1 for the Master Oogway team. This 
file starts the world, creates the maze, positions the turtlebot, etc.


"""

# all of the imports 
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess



def generate_launch_description():
    # create an empty world
    world_file_name = 'empty_world.world'
    world = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                         'worlds', world_file_name)

    # Path to your turtlebot model SDF file
    turtlebot_model_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                                         'models', 'turtlebot3_burger', 'model.sdf')
    
    # make it easy to try different maps
    dict_of_options = {"1":"TestMaze1"}
    maze_num = input(f"What Practice Maze number do you want to try? (Options: {', '.join(list(dict_of_options.keys()))}): ")
    if maze_num in list(dict_of_options.keys()):
        maze_name = dict_of_options[maze_num]
    else:
        print(f"{maze_num} not an option!. Sorry.")
        exit()
        
    # Path to your walls model SDF file
    maze_model_file = os.path.join(get_package_share_directory('turtlebot3_gazebo'),
                                    'worlds', maze_name, 'model.sdf')


    # get the position of the maze
    maze_postions = {"1": ["-8.2", "-7.0", "0.0"]}
    curr_maze_pos = maze_postions[maze_num]

    # get the position of the robot
    robot_postions = {"1": ["0.0", "0.0", "0.01"]}
    curr_robot_postions = robot_postions[maze_num]

    # get the rotation of the robot
    robot_rotation = {"1":"-1.5708"}
    curr_robot_rotation = robot_rotation[maze_num]

    return LaunchDescription([
        # Start Gazebo with the correct ROS 2 plugins
        ExecuteProcess(
            cmd=['gazebo', '--verbose', world, '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),
        # Node to spawn the turtlebot
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_turtlebot',
            arguments=[
                '-entity', 'turtlebot',
                '-file', turtlebot_model_file,
                '-x', curr_robot_postions[0], '-y', curr_robot_postions[1], '-z', curr_robot_postions[2],
                '-Y', curr_robot_rotation,
            ],
            output='screen'
        ),
        # Node to spawn the walls
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='spawn_walls',
            arguments=[
                '-entity', 'walls',
                '-file', maze_model_file,
                '-x', curr_maze_pos[0], '-y', curr_maze_pos[1], '-z', curr_maze_pos[2]
            ],
            output='screen'
        )
    ])