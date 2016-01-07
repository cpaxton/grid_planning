
#include <grid/trajectory_distribution.h>
#include <grid/test_features.h>
#include <grid/wam_training_features.h>
#include <grid/visualize.h>

#include <trajectory_msgs/JointTrajectory.h>

using namespace grid;
using namespace KDL;

int main(int argc, char **argv) {
  ros::init(argc,argv,"grid_trajectory_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples",1000);
  ros::Publisher jpub = nh.advertise<trajectory_msgs::JointTrajectory>("trajectory",1000);

  RobotKinematicsPointer rk_ptr = RobotKinematicsPointer(new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link"));

  TestFeatures test;
  test.addFeature("node",grid::POSE_FEATURE);
  test.addFeature("link",grid::POSE_FEATURE);
  test.addFeature("time",grid::TIME_FEATURE);
  test.setAgentFrame("wam/wrist_palm_link");
  test.setWorldFrame("world");
  test.setFrame("gbeam_node_1/gbeam_node","node");
  test.setFrame("gbeam_link_1/gbeam_link","link");

  Skill approach("approach",1);
  approach.appendFeature("link").appendFeature("time");
  approach.setInitializationFeature("link"); // must be a pose so we can find out where to start looking

  /* LOAD TRAINING DATA */
  {

    std::vector<std::string> objects;
    objects.push_back("link");
    objects.push_back("node");

    std::string filenames[] = {"data/sim/app1.bag", "data/sim/app2.bag", "data/sim/app3.bag"};

    std::vector<std::shared_ptr<WamTrainingFeatures> > wtf(3);
    for (unsigned int i = 0; i < 3; ++i) {
      std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
      wtf_ex->addFeature("time",TIME_FEATURE);
      wtf_ex->setRobotKinematics(rk_ptr);
      wtf_ex->read(filenames[i]);
      wtf[i] = wtf_ex;
    }

    // add each skill
    for (unsigned int i = 0; i < 3; ++i) {
      approach.addTrainingData(*wtf[i]);
    }
    approach.trainSkillModel();
    approach.printGmm();

    for (unsigned int i = 0; i < 3; ++i) {
      std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
      wtf_ex->addFeature("time",TIME_FEATURE);
      wtf_ex->setRobotKinematics(rk_ptr);
      wtf_ex->read(filenames[i]);
      std::vector<FeatureVector> data = wtf_ex->getFeatureValues(approach.getFeatures());
      std::cout << data.size() << " features." << std::endl;
      for (FeatureVector &vec: data) {
        std::pair<FeatureVector,double> obs(vec,1.0);
        for (unsigned int i = 0; i < vec.size(); ++i) {
          std::cout << vec(i) << " ";
        }
        std::cout << std::endl;
      }
      FeatureVector v = approach.logL(data);
      double p = v.sum() / v.size();
      std::cout << "training example " << i << ": p = " << p << std::endl;
    }
  }
  ROS_INFO("Done setting up. Sleeping...");
  ros::Duration(1.0).sleep();

  ros::Rate rate(1);
  unsigned int ntrajs = 10;

  ros::spinOnce();

  ROS_INFO("Updating world...");
  test.updateWorldfromTF();

  ROS_INFO("Initializing trajectory distribution...");
  TrajectoryDistribution dist(3,1);
  dist.initialize(test,approach);

  std::vector<Trajectory *> trajs(ntrajs);
  std::vector<double> ps(ntrajs);

  for (unsigned int i = 0; i < 20; ++i) {
    ros::spinOnce();

    // sample trajectories
    trajs = dist.sample(ntrajs);
      std::cout << "[" << i << "] Publishing trajectories..." << std::endl;
      pub.publish(toPoseArray(trajs,0.05,"world"));
      std::cout << "   ... done." << std::endl;

    // compute probabilities
    for (unsigned int j = 0; j < trajs.size(); ++j) {
          std::vector<FeatureVector> features = test.getFeaturesForTrajectory(approach.getFeatures(),trajs[j]);
          FeatureVector v = approach.logL(features);
          double d = v.array().exp().sum(); // would add other terms first
          std::cout << "   - " << j << ": " << d << std::endl;
    }

    // update distribution

  }

}
