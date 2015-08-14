#include <grid/grid_planner.h>

#include <exception>
#include <iostream>

using namespace dmp;

using planning_scene::PlanningScene;
using robot_model_loader::RobotModelLoader;
using robot_model::RobotModelPtr;
using robot_state::RobotState;
using collision_detection::CollisionRobot;

namespace grid {

  const std::string GridPlanner::TIME("time");
  const std::string GridPlanner::GRIPPER("gripper");

  GridPlanner::GridPlanner(std::string robot_description_) : nh() {

    // needs to set up the Robot objects and listeners
    try {

      robot_model_loader::RobotModelLoader robot_model_loader(robot_description_);
      ROS_INFO("Loaded model from \"%s\"!",robot_description_.c_str());

      model = robot_model_loader.getModel();

    } catch (std::exception ex) {
      std::cerr << ex.what() << std::endl;
    }

  }

  bool GridPlanner::Plan(std::string action1,
                         std::string action2,
                         std::unordered_map<std::string, std::string> object_mapping)
  {
    return false;
  }

}
