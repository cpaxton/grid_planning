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

  std::vector<std::shared_ptr<WamTrainingFeatures> > wtf(3);

  std::string filenames[] = {"data/sim/app1.bag", "data/sim/app2.bag", "data/sim/app3.bag"};

  for (unsigned int i = 0; i < 3; ++i) {
    std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
    wtf_ex->setRobotKinematics(rk_ptr);
    wtf_ex->read(filenames[i]);
    wtf[i] = wtf_ex;
  }

  wtf[0]->printTrainingFeaturesInfo();
  wtf[0]->printExtractedFeatures();

  std::vector<FeatureVector> data;
  for (unsigned int i = 0; i < 3; ++i) {
    std::vector<FeatureVector> ex_data = wtf[i]->getAllFeatureValues();
    std::cout << "... preparing example " << (i+1) << " with " << ex_data.size() << " observations." << std::endl;
    data.insert(data.end(),ex_data.begin(),ex_data.end());
  }

  std::cout << "Total observations: " << data.size() << std::endl;
  std::vector<std::pair<FeatureVector,double> > training_data;
  for (FeatureVector &vec: data) {
    std::pair<FeatureVector,double> obs(vec,1.0);
    training_data.push_back(obs);
  }
  std::cout << "... converted into training data with " << data.size() << " weighted observations." << std::endl;
  Gmm<> gmm;
  gmm.Fit(training_data);
  std::cout << "Successfully fit GMM!" << std::end;


  // try learning a GMM model

  // publish trajectories
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory",1000);

  ros::Rate rate = ros::Rate(10);
  while (ros::ok()) {

    geometry_msgs::PoseArray msg;
    msg.header.frame_id = "gbeam_link_1/gbeam_link"; //"wam/wrist_palm_link";
    for (unsigned int i = 0; i < 3; ++i) {

      std::vector<Pose> poses = wtf[i]->getPose("link");
      for (Pose &pose: poses) {
        tf::Pose tfp;
        geometry_msgs::Pose p;
        tf::poseKDLToTF(pose,tfp);
        tf::poseTFToMsg(tfp,p);
        msg.poses.push_back(p);
      }
    }

    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

}
