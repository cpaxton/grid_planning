#ifndef _GRID_WAM_TRAINING_FEATURES
#define _GRID_WAM_TRAINING_FEATURES

#include <grid/features.h>
#include <grid/training_features.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// training features reads the feature data from ROS topics
#include <ros/ros.h>

namespace grid {

  class WamTrainingFeatures: public TrainingFeatures {
  public:

    /**
     * initialize training features with the necessary world objects to find
     */
    WamTrainingFeatures(const std::vector<std::string> &objects);

  protected:

    /*
     * return the gripper features from a rosbag
     */
    virtual std::vector<double> getGripperFeatures(rosbag::MessageInstance const &m);

  };

}

#endif
