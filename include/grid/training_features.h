#ifndef _GRID_TRAINING_FEATURES
#define _GRID_TRAINING_FEATURES

#include <grid/features.h>

// training features reads the feature data from ROS topics
#include <ros/ros.h>

namespace grid {
  
  class TrainingFeatures: public Features {

    public:

    /* getPose
     * This function needs to be implemented by inheriting classes.
     * Time field helps determine when the query should occur.
     * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
     */
    std::vector<pose_t> getPose(const std::string &name,
                                unsigned long int mintime = 0,
                                unsigned long int maxtime = 0);
  };

}

#endif
