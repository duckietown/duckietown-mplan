cmake_minimum_required(VERSION 2.8.3)
project(obst_avoid)

find_package(catkin REQUIRED COMPONENTS
  rospy
  obst_avoid_msgs
  std_msgs
  geometry_msgs
  obst_avoid_viz
)

catkin_python_setup()
catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)


if (CATKIN_ENABLE_TESTING)
    find_package(rostest REQUIRED)
    #add_rostest(tests/obst_avoid_tester_node.test)
endif()
