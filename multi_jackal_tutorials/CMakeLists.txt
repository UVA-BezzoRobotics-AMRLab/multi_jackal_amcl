cmake_minimum_required(VERSION 3.0.2)
project(multi_jackal_tutorials)
add_definitions(-std=c++17)

find_package(catkin REQUIRED COMPONENTS 
  roslaunch  
  roscpp
  rospy
  sensor_msgs
  std_msgs  
  nav_msgs
  tf2_ros
  tf2_geometry_msgs
)

catkin_package()



roslaunch_add_file_check(launch/one_jackal.launch)
roslaunch_add_file_check(launch/two_jackal.launch)
roslaunch_add_file_check(launch/three_jackal.launch)

include_directories(
  ${PROJECT_SOURCE_DIR}
  ${catkin_INCLUDE_DIRS}
)

add_executable(robot_explorer src/explore_robot.cpp)
add_executable(odom_transformer src/odom_transformer.cpp)
target_link_libraries(odom_transformer ${catkin_LIBRARIES})

install(
	DIRECTORY launch rviz worlds
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
