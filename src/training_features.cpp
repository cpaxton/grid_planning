#include <grid/training_features.h>

// read in poses from topics
#include <geometry_msgs/Pose.h>
#include <iostream>
#include <sstream>

namespace grid {

  /**
   * initialize training features with the necessary world objects to find
   */
  TrainingFeatures::TrainingFeatures(const std::vector<std::string> &objects_) : objects(objects_) {
    topics.push_back("joint_states");
    topics.push_back("base_tform");

    for (std::string &obj: objects) {
      std::stringstream ss;
      ss << "world/" << obj;
      topics.push_back(ss.str().c_str());
    }
  }

  /**
   * print basic info for debugging
   */
  void TrainingFeatures::printTrainingFeaturesInfo() {
    std::cout << "Topics: " << std::endl;
    for (std::string &topic: topics) {
      std::cout << " - " << topic << std::endl;
    }
  }

  /* getPose
   * This function needs to be implemented by inheriting classes.
   * Time field helps determine when the query should occur.
   * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
   */
  std::vector<Pose> TrainingFeatures::getPose(const std::string &name,
                                              unsigned long int mintime,
                                              unsigned long int maxtime) {
    std::vector<Pose> poses;


    return poses;
  }


  /* getFeatureValues
   * Returns a list of features converted into a format we can use.
   */
  std::vector<std::vector<double> > TrainingFeatures::getFeatureValues(const std::string &name,
                                                                       unsigned long int mintime,
                                                                       unsigned long int maxtime) {
    std::vector<std::vector<double> > values;



    return values;
  }

  void TrainingFeatures::open(const std::string &bagfile) {
    bag.open(bagfile, rosbag::bagmode::Read);


  }

#if 0
  /*
   * return the gripper features from a rosbag
   * default: returns nothing
   */
  virtual std::vector<double> TrainingFeatures::getGripperFeatures(rosbag::MessageInstance const &m) {
    return std::vector<double>();
  }
#endif

}
