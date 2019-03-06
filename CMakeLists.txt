cmake_minimum_required(VERSION 2.8.3)
project(waypoint_server)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
)
