
#include <grid/trajectory_distribution.h>
#include <grid/test_features.h>
#include <grid/visualize.h>

using namespace grid;
using namespace KDL;

int main(int argc, char **argv) {
  ros::init(argc,argv,"grid_trajectory_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory",1000);

  TestFeatures test;
  test.addFeature("node",grid::POSE_FEATURE);
  test.addFeature("link",grid::POSE_FEATURE);
  test.setAgentFrame("wam/wrist_palm_link");
  test.setWorldFrame("world");
  test.setFrame("gbeam_node_1/gbeam_node","node");
  test.setFrame("gbeam_link_1/gbeam_link","link");


  TrajectoryDistribution dist(3,1);
  dist.initialize(test,Skill::DefaultSkill("node"));

  ros::Rate rate(30);
  unsigned int ntrajs = 10;
  try {
    while (ros::ok()) {

      Trajectory *trajs = dist.sample(ntrajs);
      pub.publish(toPoseArray(trajs,0.05,"wam/wrist_palm_link",ntrajs));
      delete trajs;
    }
  } catch (ros::Exception ex) {
    ROS_ERROR("%s",ex.what());
  }

}
