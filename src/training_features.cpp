#include <grid/training_features.h>

// read in poses from topics
#include <geometry_msgs/Pose.h>

namespace grid {

    std::vector<Pose> TrainingFeatures::getPose(const std::string &name,
                                unsigned long int mintime,
                                unsigned long int maxtime) {
      std::vector<Pose> poses;


      return poses;
    }

    void TrainingFeatures::open(const std::string &bagfile) {
      bag.open(bagfile, rosbag::bagmode::Read);
    }


}
