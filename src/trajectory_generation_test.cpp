
#include <grid/trajectory_distribution.h>
#include <grid/test_features.h>
#include <grid/wam_training_features.h>
#include <grid/visualize.h>

using namespace grid;
using namespace KDL;

int main(int argc, char **argv) {
  ros::init(argc,argv,"grid_trajectory_test_node");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples",1000);

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


  /* LOOP */

  ROS_INFO("Done setting up. Sleeping...");
  ros::Duration(1.0).sleep();

  ros::Rate rate(1);
  unsigned int ntrajs = 1;
  try {
    while (ros::ok()) {

      ROS_INFO("Updating world...");
      test.updateWorldfromTF();

      ROS_INFO("Initializing trajectory distribution...");
      TrajectoryDistribution dist(3,1);
      dist.initialize(test,approach);

      ROS_INFO("Generating trajectories...");
      std::vector<Trajectory *> trajs;

      // look at the time it takes to compute features
      {
        using namespace std;

        clock_t begin = clock();
        trajs = dist.sample(ntrajs);
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Sampling " << ntrajs << " trajectories took " << elapsed_secs << " seconds." << std::endl;
      }

      // generate the features
      // see how long that takes
      {
        using namespace std;

        clock_t begin = clock();
        for (unsigned int i = 0; i < trajs.size(); ++i) {
          std::vector<FeatureVector> features = test.getFeaturesForTrajectory(approach.getFeatures(),trajs[i]);
          FeatureVector v = approach.logL(features);
          double p = v.sum() / v.size();
          std::cout << " - traj " << i << ": avg p = " << p << std::endl;
        }
        clock_t end = clock();
        double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
        std::cout << "Generating features for " << ntrajs << " trajectories took " << elapsed_secs << " seconds." << std::endl;
      }

      std::cout << "Publishing trajectories..." << std::endl;
      pub.publish(toPoseArray(trajs,0.05,"world"));
      std::cout << "Done." << std::endl;

      for(unsigned int i = 0; i < trajs.size(); ++i) {
        delete trajs[i];
      }

      rate.sleep();
    }
  } catch (ros::Exception ex) {
    ROS_ERROR("%s",ex.what());
  }

}
