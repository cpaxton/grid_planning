#include <grid/test_features.h>
#include <ros/ros.h>
#include <ctime>

using namespace grid;
using namespace KDL;

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
      for (double z = 0.2; z < 2.0; z += 0.2) {
        Rotation r1 = Rotation::RPY(0,0,0);
        Vector v1 = Vector(0,0,z);
        Frame t1 = Frame(r1,v1);
        traj.push_back(t1);
      }      

      // look at the time it takes to compute features
      {
        using namespace std;

        clock_t begin = clock();
        features = test.getFeaturesForTrajectory("link",traj);
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Computing features for " << features.size() << " positions took " << elapsed_secs << "seconds." << std::endl;
      }

      for (int idx = 0; idx < features.size(); ++idx) {
        std::cout << "[" << idx << " ] Feature values: ";
        for (int i = 0; i < features[idx].size(); ++i) {
          std::cout << features[idx][i] << " ";
        }
        std::cout << std::endl;
      }

      rate.sleep();
    }
  } catch (ros::Exception ex) {
    ROS_ERROR("%s",ex.what());
  }
}
