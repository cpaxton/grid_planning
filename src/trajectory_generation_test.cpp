
#include <grid/trajectory_distribution.h>
#include <grid/test_features.h>

using namespace grid;
using namespace KDL;

int main(int argc, char **argv) {
  ros::init_node(argc,argv,"grid_trajectory_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory",1000);

  TestFeatures test;
  test.addFeature("node",grid::POSE_FEATURE);
  test.addFeature("link",grid::POSE_FEATURE);
  test.setAgentFrame("wam/wrist_palm_link");
  test.setWorldFrame("world");
  test.setFrame("gbeam_node_1/gbeam_node","node");
  test.setFrame("gbeam_link_1/gbeam_link","link");


  TrajectoryDistribution dist(test,Skill::DefaultSkill("node"));

  ros::Rate rate(30);
  unsigned int ntrajs = 10;
  try {
    while (ros::ok()) {

      Trajectory *trajs = dist.sample(ntrajs);
      pub.publish(toPoseArray(traj,0.05,"wam/wrist_palm_link",ntrajs));
      delete trajs;
    }
  }

}
