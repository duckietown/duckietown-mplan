cmake_minimum_required(VERSION 2.8.3)
project(obst_avoid_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  std_msgs
  geometry_msgs
  sensor_msgs
)

add_message_files(
  FILES
  Actor.msg
  MovingObject.msg
  Obstacles.msg
  TimedPath.msg
  Twist2DStamped.msg
  Pose2DStamped.msg
  Vector2D.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
)


catkin_package(
   CATKIN_DEPENDS
   geometry_msgs
   roscpp rospy
   std_msgs
   message_runtime
   visualization_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
)
