#include <grid/grid_planner.h>
#include <dmp/dmp.h>

using namespace dmp;

namespace grid {

  const std::string GridPlanner::TIME("time");
  const std::string GridPlanner::GRIPPER("gripper");

  GridPlanner::GridPlanner(std::string robot_description_) : nh() {
    // no contents yet
    // needs to set up the Robot objects and listeners
  }

  bool GridPlanner::Plan(std::string action1,
                         std::string action2,
                         std::unordered_map<std::string, std::string> object_mapping)
  {
    return false;
  }

}
