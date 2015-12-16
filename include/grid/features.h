#ifndef _GRID_FEATURES
#define _GRID_FEATURES

#include <vector>
#include <unordered_map>

#include <gcop/pose.h>
//#include <tf_conversions/tf_kdl.h>

/**
 * Features
 * Parent abstract class for a set of robot features.
 * Abstract method produces appropriate poses given queries.
 */

namespace grid {

  // use GCOP's pose class for now
  typedef gcop_urdf::Pose Pose;

  std::vector<double> poseToArray(const Pose &pose);

  /* FeatureType
   * Save each feature as its own thing.
   */
  typedef enum FeatureType { POSE, FLOAT } FeatureType;

  class Features {

    /* getPose
     * This function needs to be implemented by inheriting classes.
     * Time field helps determine when the query should occur.
     * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
     */
    virtual std::vector<Pose> getPose(const std::string &name,
                                      unsigned long int mintime = 0,
                                      unsigned long int maxtime = 0) = 0;

    /* getFeatureValues
     * Returns a list of features converted into a format we can use.
     */
    virtual std::vector<std::vector<double> > getFeatureValues(const std::string &name,
                                                               unsigned long int mintime = 0,
                                                               unsigned long int maxtime = 0) = 0;

    /*
     * Processing code.
     * Needs to, among other things, get all the features and turn them into normalized data.
     */
    void initialize();

    /**
     * Get a structure containing all the appropriate features
     */
    std::vector< std::vector <double> > getFeatures(std::vector<std::string> &names);

    /**
     * add a feature
     */
    void addFeature(const std::string &name, const FeatureType type);

  protected:
    std::unordered_map<std::string,FeatureType> feature_types;
  };
}

#endif
