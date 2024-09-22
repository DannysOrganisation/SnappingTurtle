# SnappingTurtles 
This repository holds the code for MTRX3760 Project 1 and is created by the Master 
Oogway group. It holds code that allows a turtlebot simulation to solve a maze.

## Setting up the world
Must have a launch file which calls the world to launch. The worlds
are held in the `src/turtlebot3_simulations/turtlebot3_gazebo/worlds` folder.
To check them out enter the folder from the terminal and execute `gazebo <wold_name>.world`

To run the world file, run
```bash
colcon build --symlink-install && source install/setup.bash && . /usr/share/gazebo/setup.sh && ros2 launch turtlebot3_gazebo snappingturtle.launch.py
```


## To run the program
Go into the SnappingTurtle base directory and make sure the following command has been run.
```bash
colcon build --symlink-install
```
Colcon is short for collective construct, and ensures that all packages have been built correctly.
Then type the following command 
```bash
source install/setup.bash
ros2 launch turtlebot3_gazebo <launchfile_name>.launch.py
```

The actually drivin code can be spawned through the following code in a new terminal.

```bash
colcon build --symlink-install && source install/setup.bash && . /usr/share/gazebo/setup.sh && ros2 launch vroom snapping_turtle_launch.py
```

# Solve the maze
Once the world is created and the robot initiated, to run the main do the following:
```
colcon build --symlink-install && source install/setup.bash && ros2 run 
```

## Visualise the robot
Rviz is a seperate program that we can use that interacts with the ROS nodes which helps us visualise 
the robots perception of the world around it.
Simply type the following command to bring it up.
```
rviz2
```
We can then `Add` a specific topic that is getting published and view its output.
You can create a config for the rviz that saves the specific visualisation settings for a 
specific use case. This is saved into a `.rviz2` file. Specific configs can be deployed using 
```
rviz2 -d <config_file>.rfiz2
```
This can also be an argument in the launch file.
<br /><br />
The turtlebot developers have a way to see what they think is the best view of the simulation. In order to show this, the following command can be used from the base directory.
```
ros2 launch turtlebot3_bringup rviz2.launch.py
```

Another tool for visualising is PlotJuggler - can support other sources of data such as CSV files.
```
ros2 run plotjuggler plotjuggler
```




## Cool random things
- I noticed something that Khit does in his lectures is double tab middway through a command to see what the
possible options are from there.