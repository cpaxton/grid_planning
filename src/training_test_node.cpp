#include <ros/ros.h>
#include <grid/training_features.h>
#include <grid/wam_training_features.h>
#include <grid/visualize.h>

using namespace grid;

int main(int argc, char **argv) {
  ros::init(argc,argv,"training_test_node");

  std::vector<std::string> objects;
  objects.push_back("link");
  objects.push_back("node");

  RobotKinematics *rk = new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link");
  RobotKinematicsPointer rk_ptr = RobotKinematicsPointer(rk);
  WamTrainingFeatures wtf(objects);

  wtf.setRobotKinematics(rk_ptr);
  wtf.read("test.bag");

  wtf.printTrainingFeaturesInfo();

  wtf.printExtractedFeatures();

  std::vector<std::vector <double> > data = wtf.getAllFeatureValues();

  for (std::vector<double> &vec: data) {
    for (double &val: vec) {
      std::cout << val << " ";
    }
    std::cout << std::endl;
  }

  std::vector<Pose> poses = wtf.getPose("link");


  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory",1000);

  ros::Rate rate = ros::Rate(10);
  while (ros::ok()) {

    geometry_msgs::PoseArray msg;
    msg.header.frame_id = "gbeam_link_1/gbeam_link"; //"wam/wrist_palm_link";

    for (Pose &pose: poses) {
      tf::Pose tfp;
      geometry_msgs::Pose p;
      tf::poseKDLToTF(pose,tfp);
      tf::poseTFToMsg(tfp,p);
      msg.poses.push_back(p);
    }

    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }
}
