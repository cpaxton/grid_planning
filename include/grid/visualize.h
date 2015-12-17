#ifndef _GRID_DEBUG
#define _GRID_DEBUG

#include <grid/features.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>

namespace grid {

  /*  create a pose array message from a KDL trajectory */
  geometry_msgs::PoseArray toPoseArray(Trajectory *traj, double dt, const std::string &frame);

}

#endif
