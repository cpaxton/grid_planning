#include <ros/ros.h>
#include <grid/grid_planner.h>

using namespace grid;

int main(int argc, char **argv) {
  GridPlanner gp("robot_description","/gazebo/barrett_manager/wam/joint_states");
  return 0;
}
