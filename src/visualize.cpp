#include <grid/visualize.h>

#include <tf_conversions/tf_kdl.h>
#include <tf/transform_datatypes.h>

namespace grid {

  /*  create a pose array message from a KDL trajectory */
  geometry_msgs::PoseArray toPoseArray(Trajectory *traj, double dt, const std::string &frame, unsigned int ntrajs) {
    geometry_msgs::PoseArray msg;
    msg.header.frame_id = frame;

    for (unsigned int i = 0; i < ntrajs; ++i) {
      for (double t = 0; t < traj[i].Duration(); t += dt) {
        geometry_msgs::Pose p;
        tf::Pose tfp;
        tf::poseKDLToTF(traj[i].Pos(t),tfp);
        tf::poseTFToMsg(tfp,p);

        msg.poses.push_back(p);
      }
    }

    return msg;
  }

}
