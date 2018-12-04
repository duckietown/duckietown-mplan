cmake_minimum_required(VERSION 2.8.3)
project(obst_avoid_viz)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
  roscpp
  visualization_msgs
  flock_simulator
  tf2
  duckietown_visualization
)

catkin_python_setup()
catkin_package()



###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
  CATKIN_DEPENDS
  DEPENDS
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(duck_viz src/libraries/obst_avoid_viz/duck_viz.cpp)
target_link_libraries(duck_viz ${catkin_LIBRARIES})

add_executable(duck_viz_node src/duck_viz_node.cpp)
target_link_libraries(duck_viz_node duck_viz ${catkin_LIBRARIES})
