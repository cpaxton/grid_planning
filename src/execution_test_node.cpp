
#include <grid/trajectory_distribution.h>
#include <grid/test_features.h>
#include <grid/wam_training_features.h>
#include <grid/visualize.h>

#include <trajectory_msgs/JointTrajectory.h>

using namespace grid;
using namespace KDL;



void load_and_train_skill(Skill &skill, RobotKinematicsPointer &rk_ptr, const std::string filenames[]) {
  /* LOAD TRAINING DATA FOR GRASP*/

    std::vector<std::string> objects;
    objects.push_back("link");
    objects.push_back("node");

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
      skill.addTrainingData(*wtf[i]);
    }
    skill.trainSkillModel();
    skill.printGmm();

    for (unsigned int i = 0; i < 3; ++i) {
      std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
      wtf_ex->addFeature("time",TIME_FEATURE);
      wtf_ex->setRobotKinematics(rk_ptr);
      wtf_ex->read(filenames[i]);
      std::vector<FeatureVector> data = wtf_ex->getFeatureValues(skill.getFeatures());

#if 0
      for (FeatureVector &vec: data) {
        std::pair<FeatureVector,double> obs(vec,1.0);
        for (unsigned int i = 0; i < vec.size(); ++i) {
          std::cout << vec(i) << " ";
        }
        std::cout << std::endl;
      }
#endif

      skill.normalizeData(data);
      FeatureVector v = skill.logL(data);
      double p = v.array().exp().sum() / v.size();
      std::cout << "training example " << i << " with " << data.size() << " examples: p = " << p << std::endl;
    }
}


int main(int argc, char **argv) {
  ros::init(argc,argv,"grid_execution_test_node");
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

  double step_size;
  double noise;
  int ntrajs = 50;
  int iter = 10;
  ros::NodeHandle nh_tilde("~");
  if (not nh_tilde.getParam("step_size",step_size)) {
    step_size = 0.80;
  }
  if (not nh_tilde.getParam("noise",noise)) {
    noise = 1e-5;
  }
  if (not nh_tilde.getParam("ntrajs",ntrajs)) {
    ntrajs = 50;
  }
  if (not nh_tilde.getParam("iter",iter)) {
    iter = 10;
  }

  Skill approach("approach",1);
  approach.appendFeature("link").appendFeature("time");
  approach.setInitializationFeature("link"); // must be a pose so we can find out where to start looking

  Skill grasp("grasp",1);
  grasp.appendFeature("link").appendFeature("time");
  grasp.setInitializationFeature("link");

  /* LOAD TRAINING DATA FOR APPROACH */
  {
    std::string filenames[] = {"data/sim/app1.bag", "data/sim/app2.bag", "data/sim/app3.bag"};
    load_and_train_skill(approach, rk_ptr, filenames);
  }
  /* LOAD TRAINING DATA FOR GRASP */
  {
    std::string filenames[] = {"data/sim/grasp1.bag", "data/sim/grasp2.bag", "data/sim/grasp3.bag"};
    load_and_train_skill(grasp, rk_ptr, filenames);
  }


  ROS_INFO("Done setting up. Sleeping...");
  ros::Duration(1.0).sleep();

  ros::Rate rate(1);

  ros::spinOnce();

  ROS_INFO("Updating world...");
  test.updateWorldfromTF();

  ROS_INFO("Initializing trajectory distribution...");
  TrajectoryDistribution dist(3,1);
  dist.initialize(test,approach);

  std::vector<Trajectory *> trajs(ntrajs);
  std::vector<EigenVectornd> params(ntrajs);
  std::vector<double> ps(ntrajs);

  for (int i = 0; i < iter; ++i) {
    ros::Duration(0.25).sleep();
    ros::spinOnce();

    // sample trajectories
    
    dist.sample(params,trajs);
    pub.publish(toPoseArray(trajs,0.05,"world"));

    double sum = 0;

    // compute probabilities
    for (unsigned int j = 0; j < trajs.size(); ++j) {
      std::vector<FeatureVector> features = test.getFeaturesForTrajectory(approach.getFeatures(),trajs[j]);
      approach.normalizeData(features);
      FeatureVector v = approach.logL(features);
      ps[j] = v.array().exp().sum() / v.size(); // would add other terms first
      sum += ps[j];
    }

    // update distribution
    dist.update(params,ps,noise,step_size);

    for (unsigned int j = 0; j < trajs.size(); ++j) {
      delete trajs[j];
      trajs[j] = 0;
    }


    std::cout << "[" << i << "] >>>> AVG P = " << (sum / ntrajs) << std::endl;

  }
}
