cmake_minimum_required(VERSION 3.0.2)
project(scara_robot)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_plugins
  gazebo_ros
  gazebo_ros_control
  geometry_msgs
  roscpp
  sensor_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(scara_control src/scara_control.cpp)
# This will link executable to the appropriate libraries
target_link_libraries(scara_control ${catkin_LIBRARIES})
