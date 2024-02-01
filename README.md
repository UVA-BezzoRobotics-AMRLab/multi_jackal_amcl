# Multi-Jackal Simulator using Gazebo ROS in Ubuntu 20/ROS Noetic

This package is a port of Nick Sullivan's multi-jackal simulator for Ubuntu 16. The ROS documentation can be found [here](http://wiki.ros.org/multi_jackal_tutorials).

# Installation Instructions

_This package is designed for Ubuntu 20/ROS Noetic. Make sure your system is running Ubuntu 20 and that ROS is installed: http://wiki.ros.org/noetic/Installation/Ubuntu_ (install the full desktop version: ros-noetic-desktop-full)

- Create a catkin workspace for your packages
```
mkdir -p ~/jackal_ws/src
cd ~/jackal_ws/
catkin_make
```
- Install and compile all the required base jackal noetic packages from clearpath:
```
cd src
git clone https://github.com/jackal/jackal.git
git clone https://github.com/jackal/jackal_simulator.git
git clone https://github.com/jackal/jackal_desktop.git
git clone https://github.com/ros-visualization/interactive_marker_twist_server.git
cd ~/jackal_ws
rosdep install --from-paths . --ignore-src --rosdistro=noetic
catkin_make
```
- Install and compile this simulator in jackal_ws/src:
```
cd src
git clone https://github.com/laurenbramblett/multi-jackal-apf
cd ~/jackal_ws
rosdep install --from-paths . --ignore-src
catkin_make
```
You are ready to run!

If you'd like to use move_base (optional), install the following ROS packages:
```
sudo apt-get install ros-noetic-geometry2
sudo apt-get install ros-noetic-navigation
```

# Overview
These packages make use of the robotic simulator Gazebo, along with the Jackal 
robot description. Multiple Jackals are spawned and are able to be moved 
independently. The difference between this package and the [Jackal package](https://github.com/jackal/jackal), 
is that multiple Jackals are able to be spawned without causing overlap. 
Significant amounts of code are from Clearpaths Jackal package, this just has 
minor changes.

If you only want to simulate one, then follow the 
[guide](https://www.clearpathrobotics.com/assets/guides/jackal/simulation.html). 
The problem is that it isn't scalable. They use the same transformation tree and 
some message names. You can see the problem yourself if you spawn two and have a 
look at the topics and TF tree.

# Files
## multi_jackal_tutorials
The starting point for simulating the robots. Contains launch, config, and world files.
Starts up a Gazebo session and launches robots using `multi_jackal_base`.
Example: `roslaunch multi_jackal_tutorials one_jackal.launch`.

You can also move your jackals with the following python script after running a launch file such as the one above:
`python3 ~/<multi-jackal-ws>/src/multi_jackal_tutorials/scripts/multi-jackal-apf.py`

### FOR CAPSTONE:
Edit the following lines to change the goal or adjust initial parameters for "multi-jackal-apf.py"
- real_robot = True (sets the pose to a relative odometry frame)
- goal = [List of goals] (sets the goals for each robot. Length of list is determined by number of robots in the experiment. This goal will either be relative to the robots odometry or global frame depending on value of real_robot)
- sub_names = [List of subscriber names] (defines the rostopic that the script will listen to for relative pose)
- pub_names = [list of publisher names] (defines the rostopic that the script will publish velocity commands to)
- fields = [Potential field initialization] (defines the constants that will tune how the robot moves)
  - kappa_attr: the coefficient for how much the robot is attracted to a goal
  - kappa_rep_obs: the coefficient for how much the robot is repulsed by an obstacle
  - kappa_rep_veh: the coefficient for how much the robot is repulsed by another vehicle
  - d0: the distance threshold for when the robot should move away from an obstacle
  - d1: the distance threshold for when the robot should move away from another vehicle 

#### ASTAR Instructions
In five separate terminals, run the following - you can use a launch file to combine all the following but it is left separate for interpretation
```
roscore
roslaunch multi_jackal_tutorials two_jackal.launch
gzclient
python3 ~/<multi-jackal-ws>/src/multi_jackal_tutorials/scripts/paths_run_astar.py
python3 ~/<multi-jackal-ws>/src/multi_jackal_tutorials/scripts/multi-jackal-apf-simple.py
```


#### Mapping Instructions (for simulator):
In three terminals, run the following:
```
roslaunch jackal_gazebo jackal_world.launch config:=front_laser
roslaunch jackal_navigation gmapping_demo.launch 
roslaunch jackal_viz view_robot.launch config:=gmapping
```
TODO: Change the gmapping configuration in the gmapping launch file to use the gazebo state to map. There is an example of the world pose being published in `$(find multi_jackal_tutorials)/scripts/publish_global_pose.py` and in the gmapping launch file change in `$(find multi_jackal_tutorials)/multi_gmapping.launch`. You do not need the namespaces though and the front scan topic can remain the default as in the single robot workspace (just the jackal_ws) `$(find jackal_navigation)/launch/include/gmapping.launch` 

Note: The $(find ...) points to a ros package. This should be the folder name inside either jackal_ws or multi_jackal_ws.


- The first roslaunch command launches gazebo and loads the `jackal_world` racetrack. Make sure the configuration is `front_laser` or you will have no tool to map with (i.e. in this case, LiDAR). 
- The second roslaunch command runs the gmapping_demo, which stores your LiDAR scans into an occupancy map. You can adjust things like the maximum range and the rate at which the map updates in the `<jackal-ws>/src/jackal_navigation/launch/include/gmapping.launch` file. If it is not already, change the `param name="map_update_interval"` to something less than 1 (recommend 0.5).
- The third roslaunch command allows you to view the resulting map. When you are satisfied run the following command in a separate terminal which will save your map as mymap (change the directory of either your terminal or the mymap path to change the location it saves):
```
rosrun map_server map_saver -f mymap
```

#### Mapping Instructions (for turtlebot):
Please follow the similar instructions on the real turtlebot [here](https://learn.turtlebot.com/2015/02/01/11/)


#### ASTAR Instructions for path planning given pgm --> from saved map
In three terminals run the following:
```
roslaunch multi_jackal_tutorials two_jackal_navstack.launch
roslaunch astar astar_capstone.launch
rosrun multi_jackal_tutorials multi-apf-jackal-navstack.py
```
You can now use move_base to move the jackals to coordinate goals. I have tested the solution with the current implementation. Both robots now map using gmapping. Gmapping calls a published pose from the gazebo states. All launch files also publish a series of transforms or run python files in order to publish the needed topics. I double checked and the astar is also path planning from the initial locations -- I also made sure that the robot is truly arriving at the right location -- I think the previous error was potentially user error or a pgm mapping issue. It should be solved in this iteration. 

NOTE: The move base can be finicky so rerun if it gets stuck

TODO: Make identifying location of cities/tasks easier by storing 2D Nav goals in Rviz rather than putting them in via array in dictionary 

You can play with the Astar package by running
```
roslaunch astar astar.launch
```
An rviz screen will pop up and you can select a pose estimate in the top toolbar by clicking the tool and then selecting a location on the map. Then select a 2D Nav Goal similarly and it should show you the resultant astar path.

## multi_jackal_base
Contains a single launch file that calls all other jackal components.

## multi_jackal_control
Launches the velocity controller plugin and robot controls.

## multi_jackal_description
Creates a plugin for publishing robot states and transformations. Loads a 
parameter that describes the robot for use in Gazebo.

## multi_jackal_nav
Creates the localization and move_base nodes.

# Running
Make sure the file `multi_jackal_description/scripts/env_run` is executable.

Example launch files can be found in `multi_jackal_tutorials/launch`. Gazebo can be viewed with `gzclient` in another terminal.

NOTE: rviz folder and config are included but Noetic's TF handling has been completely reworked, one robot can be visualized but multiple is not yet implemented
TODO: fix TF tree and rviz
# multi_jackal_amcl
