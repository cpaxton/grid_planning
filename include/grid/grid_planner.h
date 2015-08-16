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
#include <list>
#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

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
   *
   *  --- ON PLANNING ---
   *  The key step for planning is just repeatedly sampling new primitives from some distribution.
   *  For the sake of saving implementation time, the way we do this is to sample from our distibution in PYTHON and call this code.
   *  There is a python-targeted version of the TryPrimitives helper function we can use.
   *  Then the responsibilities of this code are to:
   *  - maintain an accurate PlanningScene
   *  - maintain a robot model (with joint states, etc.)
   *  - call DMP code to generate trajectories
   */
  class GridPlanner {

  public:

    /* constructor */
    GridPlanner(std::string RobotDescription = std::string("robot_desciption"),
                std::string JointStateTopic = std::string("joint_states"),
                double padding=0.0);

    static const std::string TIME;
    static const std::string GRIPPER; // fixed to BHand for now!

    /* instantiate a planning request with the given objects */
    bool Plan(const std::string &action1,
              const std::string &action2,
              const std::unordered_map<std::string, std::string> &object_mapping);

    /* add an object to the action here */
    bool AddObject(const std::string &object_name);

    /* add an action */
    bool AddAction(const std::string &action_name);

    /* try a set of motion primitives; see if they work.
     * this is aimed at the python version of the code. */
    std::list<std::list<double> > pyTryPrimitives(const std::list<double> &primitives);

  protected:
    std::unordered_map<std::string, std::string> object_lookup;
    robot_model::RobotModelPtr model;

  private:
    ros::NodeHandle nh;
    std::shared_ptr<robot_state::RobotState> state;
    std::shared_ptr<planning_scene::PlanningScene> scene;
    ros::Subscriber js_sub;

    /* keep robot joints up to date */
    void JointStateCallback(const sensor_msgs::JointState::ConstPtr &msg);
  };

}

#endif
