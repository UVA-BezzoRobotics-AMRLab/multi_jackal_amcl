## Evolutionary-based Coordination of Multi-Robot Systems with Dynamic Constraints in Ubuntu 20/ROS Noetic

In this repository, we provide the code for our results in our paper titled ``Evolutionary-based Coordination of Multi-Robot Systems with Dynamic Constraints."  Our solution incorporates: a digital twin of a monitored environment, A* and artificial potential field methods for path planning and navigation, and human-in-the-loop prioritization. A copy of the paper is provided in this repository [here](https://github.com/UVA-BezzoRobotics-AMRLab/multi_jackal_amcl/blob/main/Evolutionary-based_Coordination_of_Multi-Robot_Systems_with_Dynamic_Constraints.pdf).


If using this repository, please cite our work:
```
@inproceedings{shah2024coord,
  title={Evolutionary-based Coordination of Multi-Robot Systems with Dynamic Constraints},
  author={Shah, Vihar and Heeter, Matthew and Vallarino, Jose and Sherman, Patrick and Bramblett, Lauren and Bezzo, Nicola},
  booktitle={2024 Systems and Information Engineering Design Symposium (SIEDS)},
  year={2024},
  organization={IEEE}
}
```

Credit: This package is a port of Nick Sullivan's multi-jackal simulator for Ubuntu 16. The ROS documentation can be found [here](http://wiki.ros.org/multi_jackal_tutorials).

#### Installation Instructions

_This package is designed for Ubuntu 20/ROS Noetic. Make sure your system is running Ubuntu 20 and that ROS is installed: http://wiki.ros.org/noetic/Installation/Ubuntu_ (install the full desktop version: ros-noetic-desktop-full)

- Create a catkin workspace for your packages
```
mkdir -p ~/jackal_ws/src
cd ~/jackal_ws/
catkin build
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
catkin build
```
- Install and compile this simulator in jackal_ws/src:
```
cd src
git clone https://github.com/UVA-BezzoRobotics-AMRLab/multi_jackal_amcl
cd ~/jackal_ws
rosdep install --from-paths . --ignore-src
catkin build
```
You are ready to run!

If you'd like to use move_base (optional), install the following ROS packages:
```
sudo apt-get install ros-noetic-geometry2
sudo apt-get install ros-noetic-navigation
```

#### Overview
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

#### Using AMCL
Here we provide an example and scalable output for multiple jackals using amcl to localize each robot. 
```
roslaunch multi_jackal_tutorials two_jackal_amcl.launch map_file:=your/map/file
```
After doing this, an RVIZ will pop up with the estimated pose of both robots (initialized at [0,0,0] if using no transforms). Change the topic of the 2D pose estimate using the tool in the top left (i.e. `/jackal0/initialpose` for jackal0) then estimate the pose of the Jackal. 

##### Additional use for AMCL -- multiple traveling salesman solver
The previous launch file also launched an Astar node. Running the following python script will initialize your mTSP and show similar results as provided in our paper. 
```
rosrun multi_jackal_tutorials multi-apf-jackal-astar.py
```
You can change the configuration file in `$(find multi_jackal_tutorials)/configs/example.yaml` to change city locations, velocities, number of agents, and the genetic algorithm parameters.

###### Manipulating robot control:
Edit the following lines to change the goal or adjust initial parameters for "multi-jackal-apf-astar.py"
- goal = [List of goals] (sets the goals for each robot. Length of list is determined by number of robots in the experiment. This goal will either be relative to the robots odometry or global frame depending on value of real_robot)
- sub_names = [List of subscriber names] (defines the rostopic that the script will listen to for relative pose)
- pub_names = [list of publisher names] (defines the rostopic that the script will publish velocity commands to)
- fields = [Potential field initialization] (defines the constants that will tune how the robot moves)
  - kappa_attr: the coefficient for how much the robot is attracted to a goal
  - kappa_rep_obs: the coefficient for how much the robot is repulsed by an obstacle
  - kappa_rep_veh: the coefficient for how much the robot is repulsed by another vehicle
  - d0: the distance threshold for when the robot should move away from an obstacle
  - d1: the distance threshold for when the robot should move away from another vehicle 

