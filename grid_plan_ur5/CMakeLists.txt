cmake_minimum_required(VERSION 2.8.3)
project(grid_plan_ur5)

add_definitions(-g -std=c++0x -pthread -O1)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  #ascent_mk2
  dmp
  #grid_experiments
  #grid_tasks

  kdl_parser
  urdf

  # TF
  tf
  tf_conversions

  # MoveIt stuff
  moveit_core
  moveit_msgs
  moveit_ros_perception
  moveit_ros_planning_interface
  moveit_ros_planning

  # messages
  sensor_msgs
  trajectory_msgs
  oro_barrett_msgs

  # ros stuff
  rosbag
)

find_package(orocos_kdl REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS python thread)
find_package(PythonLibs 2.7 REQUIRED)

find_package(PkgConfig REQUIRED)

find_package( PkgConfig )
pkg_check_modules( EIGEN3 REQUIRED eigen3 )

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)
#find_package(GCOP REQUIRED)

## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
#catkin_python_setup()

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES grid_planner
  CATKIN_DEPENDS dmp tf tf_conversions moveit_core oro_barrett_msgs grid_plan_core
  #DEPENDS GCOP EIGEN3
  DEPENDS EIGEN3 orocos_kdl
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_PATH} ${EIGEN3_INCLUDE_DIRS} )

#message(STATUS ${GCOP_INCLUDE_DIRS})
add_executable(ur5_task_test src/ur5/task_test_node.cpp)

SET(ALL_LIBRARIES 
  ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES}
)

target_link_libraries(ur5_task_test ${ALL_LIBRARIES})
