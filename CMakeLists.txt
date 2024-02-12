cmake_minimum_required(VERSION 3.0.2)
project(ros_turtlebot3_control)

find_package(catkin REQUIRED COMPONENTS
  move_base_msgs
  turtlebot3_gazebo
  turtlebot3_navigation
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch maps param rviz
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

