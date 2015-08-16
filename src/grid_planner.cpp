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

  /* add an object to the action here */
  bool GridPlanner::AddObject(const std::string &object_name) {
    ROS_WARN("\"GridPlanner::AddObject\" not yet implemented!");
    return false;
  }

  /* add an action */
  bool GridPlanner::AddAction(const std::string &action_name) {
    ROS_WARN("\"GridPlanner::AddAction\" not yet implemented!");
    return false;
  }

  /* instantiate a planning request with the given objects */
  bool GridPlanner::Plan(const std::string &action1,
                         const std::string &action2,
                         const std::unordered_map<std::string, std::string> &object_mapping)
  {
    return false;
  }

  std::list<std::list<double> > GridPlanner::pyTryPrimitives(const std::list<double> &primitives) {

  }

}
using namespace boost::python;

BOOST_PYTHON_MODULE(pygrid_planner) {
  class_<grid::GridPlanner>("GridPlanner",init<std::string>())
    .def("Plan", &grid::GridPlanner::Plan)
    .def("AddAction", &grid::GridPlanner::AddAction)
    .def("AddObject", &grid::GridPlanner::AddObject)
    .def("TryPrimitives", &grid::GridPlanner::pyTryPrimitives)
    ;
}


