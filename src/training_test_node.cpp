#include <ros/ros.h>
#include <grid/training_features.h>
#include <grid/wam_training_features.h>
#include <grid/visualize.h>
#include <grid/skill.h>

#include <fstream>

using namespace grid;

int main(int argc, char **argv) {
  ros::init(argc,argv,"training_test_node");

  std::vector<std::string> objects;
  objects.push_back("link");
  objects.push_back("node");

  RobotKinematics *rk = new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link");
  RobotKinematicsPtr rk_ptr = RobotKinematicsPtr(rk);


  unsigned int ntraining = 3u; //9u;
  std::vector<std::shared_ptr<WamTrainingFeatures> > wtf(ntraining);
  //std::string filenames[] = {"data/sim/app1.bag", "data/sim/app2.bag", "data/sim/app3.bag"};
  //std::string filenames[] = {"data/sim/align1.bag", "data/sim/align2.bag", "data/sim/align3.bag"};
  //std::string filenames[] = {"data/sim/release1b.bag", "data/sim/release2b.bag", "data/sim/release3b.bag"};
  std::string filenames[] = {"data/sim/release1.bag", "data/sim/release2.bag", "data/sim/release3.bag"};
  //std::string filenames[] = {"data/sim/release1.bag", "data/sim/release2.bag", "data/sim/release3.bag",
  //  "data/sim/release1b.bag", "data/sim/release2b.bag", "data/sim/release3b.bag","data/sim/release1c.bag", "data/sim/release2c.bag", "data/sim/release3c.bag"};
  //std::string filenames[] = {"data/sim/grasp1.bag", "data/sim/grasp2.bag", "data/sim/grasp3.bag"};

  for (unsigned int i = 0; i < ntraining; ++i) {
    std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
    wtf_ex->addFeature("time",TIME_FEATURE);
    wtf_ex->setRobotKinematics(rk_ptr);
    wtf_ex->read(filenames[i],10);
    wtf_ex->attachObjectFrame("link");
    wtf[i] = wtf_ex;
  }

  wtf[0]->printTrainingFeaturesInfo();
  wtf[0]->printExtractedFeatures();

  std::vector<FeatureVector> data;
  std::cout << "Getting features..." << std::endl;
  {

    std::vector<std::string> features;
    //features.push_back("link");
    features.push_back("node");
    features.push_back("time");

    clock_t begin = clock();
    for (unsigned int i = 0; i < ntraining; ++i) {
      std::vector<FeatureVector> ex_data = wtf[i]->getFeatureValues(features);
      //std::cout << "... preparing example " << (i+1) << " with " << ex_data.size() << " observations." << std::endl;
      data.insert(data.end(),ex_data.begin(),ex_data.end());
    }
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Converting into features took " << elapsed_secs << "seconds." << std::endl;
  }

  std::cout << "Total observations: " << data.size() << std::endl;
  std::vector<std::pair<FeatureVector,double> > training_data;
  unsigned int size = data[0].size();
  for (FeatureVector &vec: data) {
    std::pair<FeatureVector,double> obs(vec,1.0/data.size());
    //for (unsigned int i = 0; i < vec.size(); ++i) {
    //  std::cout << vec(i) << " ";
    //}
    //std::cout << std::endl;
    if (size != vec.size()) {
      std::cout << "ERROR: " << size << " != " << vec.size() << "!" << std::endl;
      break;
    }
    training_data.push_back(obs);
  }

  // try learning a GMM model
  std::cout << "... converted into training data with " << data.size() << " weighted observations." << std::endl;
  Gmm gmm(size,1);
  gmm.Init(*data.begin(),*data.rbegin());
  //gmm.Print(std::cout);

  {
    clock_t begin = clock();
    gmm.Fit(training_data);
    clock_t end = clock();
    double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;
    std::cout << "Fitting GMM took " << elapsed_secs << "seconds." << std::endl;
  }


  std::cout << "Successfully fit GMM!" << std::endl;
  gmm.Print(std::cout);

  std::cout << "Running skill test:" << std::endl;

  Skill test("align",1);
  //test.appendFeature("link").appendFeature("time");
  //test.attachObject("link");
  test.appendFeature("node").appendFeature("time");
  test.attachObject("link");
  for (unsigned int i = 0; i < ntraining; ++i) {
    test.addTrainingData(*wtf[i]);
  }
  test.trainSkillModel();
  std::cout << "Skill trained!" << std::endl;
  test.printGmm();

  // publish trajectories
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples",1000);

  // get ready
  geometry_msgs::PoseArray msg;
  msg.header.frame_id = "gbeam_node_1/gbeam_node";//"gbeam_link_1/gbeam_link"; //"wam/wrist_palm_link";
  //msg.header.frame_id = "wam/wrist_palm_link";
  for (unsigned int i = 0; i < ntraining; ++i) {

    //std::vector<Pose> poses = wtf[i]->getPose("link");
    std::vector<Pose> poses = wtf[i]->getPose("node");

    std::vector<FeatureVector> v = wtf[i]->getFeatureValues(test.getFeatures());

    //for (Pose &pose: poses) {
    for (FeatureVector &vec: v) {
      Pose pose = wtf[i]->getPoseFrom("node",vec);
      for (unsigned int i = 0; i < vec.size(); ++ i) {
        std::cout << vec(i) << " ";
      }
      std::cout << "\n";
      tf::Pose tfp;
      geometry_msgs::Pose p;
      tf::poseKDLToTF(pose,tfp);
      tf::poseTFToMsg(tfp,p);
      msg.poses.push_back(p);
    }



    test.normalizeData(v);
    FeatureVector p = test.logL(v);
    std::cout << "[" << i << "] avg = " << p.sum() / p.size() << std::endl;

  }

  ros::Rate rate = ros::Rate(10);
  while (ros::ok()) {

    pub.publish(msg);
    ros::spinOnce();
    rate.sleep();
  }

}
