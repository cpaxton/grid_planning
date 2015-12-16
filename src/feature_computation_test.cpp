#include <grid/test_features.h>
#include <ros/ros.h>

using namespace grid;
using namespace Eigen;

int main (int argc, char **argv) {
  ros::init(argc,argv,"grid_features_test_node");


  TestFeatures test;
  test.addFeature("node",grid::POSE_FEATURE);
  test.addFeature("link",grid::POSE_FEATURE);
  test.setAgentFrame("wam/wrist_palm_link");
  test.setWorldFrame("world");
  test.setFrame("gbeam_node_1/gbeam_node","node");
  test.setFrame("gbeam_link_1/gbeam_link","link");

  ROS_INFO("Done setting up. Sleeping...");
  ros::Duration(1.0).sleep();
  ROS_INFO("Transforms:");
  test.lookup("node");
  test.lookup("link");
  test.lookup("agent");


  ros::Rate rate(30);
  try {
    while (ros::ok()) {

      test.updateWorldfromTF();

      Trajectory traj;

      std::vector<FeatureVector> features;

      // construct a trajectory
      for (double z = 0.2; z < 1.4; z += 0.3) {
        Quaterniond r1 = Quaterniond(0,0,0,1);
        Affine3d t1 = Translation3d(0,0,z) * r1;
        traj.push_back(t1);
      }      

      features = test.getFeaturesForTrajectory("link",traj);

      rate.sleep();
    }
  } catch (ros::Exception ex) {
    ROS_ERROR("%s",ex.what());
  }
}
