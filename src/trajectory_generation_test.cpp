
#include <grid/trajectory_distribution.h>
#include <grid/test_features.h>

using namespace grid;
using namespace KDL;

int main(int argc, char **argv) {
  ros::init_node(argc,argv,"grid_trajectory_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory",1000);


}
