
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

  ROS_INFO("Done setting up. Sleeping...");
  ros::Duration(1.0).sleep();

  ros::Rate rate(1);
  unsigned int ntrajs = 100;
  try {
    while (ros::ok()) {

      ROS_INFO("Updating world...");
      test.updateWorldfromTF();

      ROS_INFO("Initializing trajectory distribution...");
      TrajectoryDistribution dist(3,1);
      dist.initialize(test,Skill::DefaultSkill("node"));

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
          std::vector<FeatureVector> features = test.getFeaturesForTrajectory("link",trajs[i]);
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
