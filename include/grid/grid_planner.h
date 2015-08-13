#ifndef GRID_PLANNER
#define GRID_PLANNER

/* Grounded Robot Instruction from Demonstrations
 * ---
 *  This particular file contains the "fast" version of the C++ GRID planner.
 *  It depends on a modified DMP library, so that we can quickly search for DMP trajectories that will work.
 */

#include <dmp/dmp.h>
#include <boost/python.hpp>

#include <string>

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

    /* constructor */
    GridPlanner(std::string RobotDescription = "robot_desciption");

  private:
    ros::NodeHandle nh;
  };

}

#endif
