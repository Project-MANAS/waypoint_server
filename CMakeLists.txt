cmake_minimum_required(VERSION 2.8.3)
project(waypoint_server)

find_package(catkin REQUIRED COMPONENTS
  geographic_msgs
  geometry_msgs
  nav_msgs
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  tf2_geometry_msgs
  message_generation
)

add_service_files(
  FILES
  SetPoseWaypoint.srv
  SetGeoWaypoint.srv
  QueryTargetWaypoint.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS
  tf2_ros
  tf2_geometry_msgs
  message_runtime
)

include_directories(
 ${catkin_INCLUDE_DIRS}
)
