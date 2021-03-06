cmake_minimum_required(VERSION 2.8.3)
project(grid_plan)

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
  grid_plan_msgs

  # ros stuff
  rosbag

  # action
  actionlib
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
catkin_python_setup()

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
  CATKIN_DEPENDS dmp tf tf_conversions moveit_core oro_barrett_msgs 
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

set(PY_SOURCES
  src/grid_planner.cpp
  src/collision_map.cpp
)

set(SOURCES 
  src/grid_planner.cpp
  src/collision_map.cpp
  src/features.cpp
  src/test_features.cpp
  src/training_features.cpp
  src/skill.cpp
  src/visualize.cpp
  src/wam_training_features.cpp
  src/robot_kinematics.cpp
  src/task_model.cpp
  src/instantiated_skill.cpp
  src/instantiated_skill_init.cpp

  src/traj/trajectory_distribution.cpp
  src/traj/dmp_trajectory_distribution.cpp

  src/utils/commander.cpp
  src/utils/params.cpp

  src/wam/input.cpp

  src/dist/utils.cc
)

set(PY_HEADERS
  include/grid/grid_planner.h
  include/grid/collision_map.h
)

set(HEADERS
  include/grid/grid_planner.h
  include/grid/collision_map.h
  include/grid/test_features.h
  include/grid/skill.h
  include/grid/training_features.h
  include/grid/features.h
  include/grid/visualize.h
  include/grid/trajectory_distribution.h
  include/grid/dmp_trajectory_distribution.h
  include/grid/wam_training_features.h
  include/grid/robot_kinematics.h
  include/grid/task_model.h
  include/grid/instantiated_skill.h

  include/grid/utils/commander.h
  include/grid/utils/params.h

  include/grid/wam/input.h

  include/grid/dist/gmm.h
  include/grid/dist/normal.h
  include/grid/dist/utils.h
)

## Declare a cpp library
add_library(grid_planner ${SOURCES} ${HEADERS})
add_library(pygrid_planner ${PY_SOURCES} ${PY_HEADERS})

SET(PY_PROJECT_NAME grid)
SET(PY_PACKAGE_DIR python/grid)

set_target_properties(pygrid_planner
  PROPERTIES COMPILE_FLAGS -DGEN_PYTHON_BINDINGS
  PREFIX ""
  LIBRARY_OUTPUT_DIRECTORY ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_PYTHON_DESTINATION})
#add_python_export_library(${PY_PROJECT_NAME} ${PY_PACKAGE_DIR}
#  src/grid_planner.cpp
#  include/grid/grid_planner.h
#)

## Declare a cpp executable
add_executable(grid_plan_node src/test/grid_plan_node.cpp)
add_executable(features_test src/test/feature_computation_test.cpp)
add_executable(trajectory_test src/test/trajectory_generation_test.cpp)
add_executable(dmp_test src/test/dmp_generation_test.cpp)
add_executable(training_test src/test/training_test_node.cpp)
add_executable(execution_test src/test/execution_test_node.cpp)
add_executable(dmp_execution_test src/test/dmp_execution_test_node.cpp)
add_executable(moveit_robot_model_test src/test/moveit_robot_test.cpp)

add_executable(task_test src/task_test_node.cpp)
add_executable(simple_test src/simple_test_node.cpp)
add_executable(align_test src/align_test_node.cpp)

## Add cmake target dependencies of the executable/libname,rary
## as an example, message headers may need to be generated before nodes
add_dependencies(grid_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(grid_plan_node grid_planner)
add_dependencies(features_test grid_planner)

## Specify libraries to link a library or executable target against
target_link_libraries(grid_planner
  ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${GCOP_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES}
)
target_link_libraries(pygrid_planner
  ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${GCOP_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES}
  #${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}
)

SET(ALL_LIBRARIES 
  grid_planner ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${PYTHON_LIBRARIES}
)

target_link_libraries(grid_plan_node ${ALL_LIBRARIES})
target_link_libraries(features_test ${ALL_LIBRARIES})
target_link_libraries(trajectory_test ${ALL_LIBRARIES})
target_link_libraries(dmp_test ${ALL_LIBRARIES})
target_link_libraries(training_test ${ALL_LIBRARIES})
target_link_libraries(execution_test ${ALL_LIBRARIES})
target_link_libraries(dmp_execution_test ${ALL_LIBRARIES})
target_link_libraries(task_test ${ALL_LIBRARIES})
target_link_libraries(simple_test ${ALL_LIBRARIES})
target_link_libraries(align_test ${ALL_LIBRARIES})
target_link_libraries(moveit_robot_model_test ${ALL_LIBRARIES})
