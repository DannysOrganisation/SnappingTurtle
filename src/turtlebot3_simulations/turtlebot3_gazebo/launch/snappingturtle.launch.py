"""
This is the launch file for Project 1 for the Master Oogway team. This 
file starts the world, creates the maze, positions the turtlebot, etc.


"""

# all of the imports 
import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

def generate_launch_description():
    # Directory to the Gazebo launch files
    launch_file_dir = os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Use simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Select maze number
    dict_of_options = {"Enclosed-Maze": "Closed-Maze", "Floating":"Floating"}
    maze_num = input(f"What Practice Maze number do you want to try? (Options: {', '.join(list(dict_of_options.keys()))}): ")
    if maze_num in list(dict_of_options.keys()):
        maze_name = dict_of_options[maze_num]
    else:
        print(f"{maze_num} not an option!. Sorry.")
        exit()

    # Define the world file
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    # Get the maze model file
    maze_model_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds', maze_name, 'model.sdf'
    )

    # Gazebo server command
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world}.items()
    )

    # Gazebo client command
    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Robot state publisher command
    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # Get the maze position
    maze_positions = {"Enclosed-Maze": ["4", "1", "0.0"], "Floating":["3.9", "1.5", "0.0"]}
    curr_maze_pos = maze_positions[maze_num]

    # Get the robot position
    robot_positions = {"Enclosed-Maze": ["0.0", "0.0", "0.01"], "Floating":["0.0", "0.0", "0.01"]}
    curr_robot_positions = robot_positions[maze_num]

    # Get the robot rotation
    robot_rotation = {"Enclosed-Maze": "-1.5708", "Floating":"-3.141"}
    rotation = robot_rotation[maze_num]

    # Launch configuration for robot spawn
    x_pose = LaunchConfiguration('x_pose', default=curr_robot_positions[0])
    y_pose = LaunchConfiguration('y_pose', default=curr_robot_positions[1])
    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_file_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={
            'x_pose': x_pose,
            'y_pose': y_pose,
            'Y': rotation,
        }.items()
    )

    # Spawn the maze walls
    spawn_walls_cmd = Node(
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

    # spawn some of the sensors
    lidar_sensor = Node(
        package='vroom',
        executable='lidar',
        name='lidar_node'
    )

    odom_sensor = Node(
        package='vroom',
        executable='odom',
        name='odom_node'
    )

    camera_sensor = Node(
        package='vroom',
        executable='camerareader',
        name='camerareader_node'
    )
    # Create launch description
    ld = LaunchDescription()


    # Add actions
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(spawn_walls_cmd)
    ld.add_action(lidar_sensor)
    ld.add_action(odom_sensor)
    ld.add_action(camera_sensor)
    return ld
