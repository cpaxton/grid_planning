#include <ros/ros.h>
#include <grid/training_features.h>
#include <grid/wam_training_features.h>

using namespace grid;

int main(int argc, char **argv) {
  ros::init(argc,argv,"training_test_node");

  std::vector<std::string> objects;
  objects.push_back("link");
  objects.push_back("node");

  RobotKinematics rk("robot_description","wam/base_link","wam/wrist_palm_link");
  WamTrainingFeatures wtf(objects);

  wtf.read("test.bag");

  //wtf.printTrainingFeaturesInfo();
}
