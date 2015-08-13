cmake_minimum_required(VERSION 2.8.3)
project(grid_plan)

add_definitions(-g -std=c++0x -pthread)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  ascent_mk2
  dmp
  grid_experiments
  grid_tasks
  tf
  tf_conversions
)

## System dependencies are found with CMake's conventions
find_package(Boost REQUIRED COMPONENTS python)
find_package(PythonLibs REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules)
find_package(GCOP REQUIRED)

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
  CATKIN_DEPENDS ascent_mk2 dmp grid_experiments grid_tasks tf tf_conversions
  DEPENDS GCOP
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories( include ${catkin_INCLUDE_DIRS} ${GCOP_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ${PYTHON_INCLUDE_PATH} )

## Declare a cpp library
add_library(grid_planner
  src/grid_planner.cpp
)

## Declare a cpp executable
add_executable(grid_plan_node src/grid_plan_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
add_dependencies(grid_plan_node grid_planner)

## Specify libraries to link a library or executable target against
target_link_libraries(grid_planner
  ${catkin_LIBRARIES} ${GCOP_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}
)
target_link_libraries(grid_plan_node
  grid_planner ${catkin_LIBRARIES} ${GCOP_LIBRARIES} ${Boost_LIBRARIES} ${PYTHON_LIBRARIES}
)

