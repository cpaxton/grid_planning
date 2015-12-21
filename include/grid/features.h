#ifndef _GRID_FEATURES
#define _GRID_FEATURES

#include <vector>
#include <unordered_map>

//#include <gcop/pose.h>

//#include <Eigen/Geometry>
#include <kdl/frames.hpp>
#include <kdl/trajectory.hpp>
#include <tf_conversions/tf_kdl.h>

// used for other type definitions
#include <grid/dist/gmm.h>

/**
 * Features
 * Parent abstract class for a set of robot features.
 * Abstract method produces appropriate poses given queries.
 */

namespace grid {

  // grid (for robots) is going to be based on KDL
  typedef KDL::Frame Pose;

  // feature distribution defined here as a GCOP gmm for now
  typedef gcop::Gmm<> GMM;

  // use array of poses for now (not using velocity/commands)
  typedef std::vector<Pose> TrajectoryFrames;

  // trajectories are represented as KDL trajectories
  typedef KDL::Trajectory Trajectory;

  typedef std::vector<double> FeatureVector;

  std::vector<double> poseToArray(const Pose &pose);

  /* FeatureType
   * Save each feature as its own thing.
   */
  typedef enum FeatureType { POSE_FEATURE, FLOAT_FEATURE } FeatureType;

  class Features {
  public:

    static const std::string AGENT;
    static const unsigned int POSE_FEATURES_SIZE;
    static const unsigned int POSE_FEATURE_X;
    static const unsigned int POSE_FEATURE_Y;
    static const unsigned int POSE_FEATURE_Z;
    static const unsigned int POSE_FEATURE_ROLL;
    static const unsigned int POSE_FEATURE_PITCH;
    static const unsigned int POSE_FEATURE_YAW;

    /* getPose
     * This function needs to be implemented by inheriting classes.
     * Time field helps determine when the query should occur.
     * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
     */
    virtual TrajectoryFrames getPose(const std::string &name,
                                      unsigned long int mintime = 0,
                                      unsigned long int maxtime = 0) = 0;

    /* getFeatureValues
     * Returns a list of features converted into a format we can use.
     */
    virtual std::vector< FeatureVector > getFeatureValues(const std::string &name,
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
    std::vector< FeatureVector > getFeatures(std::vector<std::string> &names);

    /**
     * add a feature
     */
    void addFeature(const std::string &name, const FeatureType type);

  protected:
    std::unordered_map<std::string,FeatureType> feature_types;
  };
}

#endif
