#include <grid/test_features.h>
#include <ros/ros.h>

using namespace grid;

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
  ros::Duration(3.0).sleep();
  ROS_INFO("Transforms:");
  test.lookup("node");
  test.lookup("link");
  test.lookup("agent");

}
