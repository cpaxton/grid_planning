#ifndef GRID_PLANNER
#define GRID_PLANNER

/* Grounded Robot Instruction from Demonstrations
 * ---
 *  This particular file contains the "fast" version of the C++ GRID planner.
 *  It depends on a modified DMP library, so that we can quickly search for DMP trajectories that will work.
 */

// General ROS dependencies
#include <ros/ros.h>

// STL
#include <string>
#include <unordered_map>

// Boost
#include <boost/python.hpp>

// MoveIt!
#include <moveit/collision_detection/collision_robot.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>

// joint states
#include <sensor_msgs/JointState.h>

// primitives for motion planning
#include <dmp/dmp.h>

namespace grid {

  /**
   * GridPlanner
   * This class defines a single planner.
   * It is associated with a robot model and listens to a planning scene.
   * To plan, we associate an object key ("link", for example) with a particular object in the world via TF frame.
   *
   * So, a query would go:
   *  GridPlanner gp();
   *  ...
   *  world = std::unordered_map<std::string,std::string>();
   *  world["link"] = "gbeam_link_1/gbeam_link";
   *  gp.plan("approach","grasp",world);
   *
   *  We instantiate a GridPlanner by providing a set of labeled demonstrations for each "state".
   *  We plan from state A at time 0 to state B at time 0.
   *
   *  Each state is associated with a single (FOR NOW) Gaussian representing features.
   */
  class GridPlanner {

  public:

    /* constructor */
    GridPlanner(std::string RobotDescription = "robot_desciption");

    static const std::string TIME;
    static const std::string GRIPPER; // fixed to BHand for now!

    bool Plan(std::string action1, std::string action2, std::unordered_map<std::string, std::string> object_mapping);

  protected:
    std::unordered_map<std::string, std::string> object_lookup;
    robot_model::RobotModelPtr model;

  private:
    ros::NodeHandle nh;
  };

}

using namespace boost::python;

BOOST_PYTHON_MODULE(grid) {
  class_<grid::GridPlanner>("GridPlanner",init<std::string>())
    .def("Plan", &grid::GridPlanner::Plan);
}

#endif
