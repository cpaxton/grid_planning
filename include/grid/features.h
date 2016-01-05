#ifndef _GRID_FEATURES
#define _GRID_FEATURES

#include <vector>
#include <unordered_map>
#include <memory>

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
  typedef gcop::Gmm<> Gmm;
  typedef std::shared_ptr<gcop::Gmm<> > GmmPtr;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> EigenVectornd;

  // use array of poses for now (not using velocity/commands)
  typedef std::vector<Pose> TrajectoryFrames;

  // trajectories are represented as KDL trajectories
  typedef KDL::Trajectory Trajectory;

  typedef std::vector<double> FeatureVector;
  //typedef EigenVectornd FeatureVector;

  std::vector<double> poseToArray(const Pose &pose);

  /* FeatureType
   * Save each feature as its own thing.
   */
  typedef enum FeatureType { POSE_FEATURE, FLOAT_FEATURE } FeatureType;

  static const std::string AGENT("agent");
  static const unsigned int POSE_FEATURES_SIZE(6);
  static const unsigned int POSE_FEATURE_X(0);
  static const unsigned int POSE_FEATURE_Y(1);
  static const unsigned int POSE_FEATURE_Z(2);
  static const unsigned int POSE_FEATURE_ROLL(3);
  static const unsigned int POSE_FEATURE_PITCH(4);
  static const unsigned int POSE_FEATURE_YAW(5);
  static const unsigned int FLOAT_FEATURES_SIZE(1);

  static const unsigned int SPLINE_POS1(6);
  static const unsigned int SPLINE_VEL1(7);
  static const unsigned int SPLINE_ACC1(8);
  static const unsigned int SPLINE_POS2(9);
  static const unsigned int SPLINE_VEL2(10);
  static const unsigned int SPLINE_ACC2(11);
  static const unsigned int SEGMENT_DURATION(12);
  static const unsigned int SPLINE_DIM(13);

  static const std::string JOINT_STATES_TOPIC("joint_states");
  static const std::string GRIPPER_MSG_TOPIC("gripper_msg");
  static const std::string BASE_TFORM_TOPIC("base_tform");
  static const std::string GRIPPER_CMD("gripper_cmd");

  class Features {
  public:

    /* getPose
     * This function needs to be implemented by inheriting classes.
     * Time field helps determine when the query should occur.
     * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
     */
    virtual TrajectoryFrames getPose(const std::string &name,
                                     double mintime = 0,
                                     double maxtime = 0) = 0;

    /* getFeatureValues
     * Returns a list of features converted into a format we can use.
     */
    virtual std::vector< FeatureVector > getFeatureValues(const std::string &name,
                                                          double mintime = 0,
                                                          double maxtime = 0) = 0;

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
     * also updates expected array size
     */
    void addFeature(const std::string &name, const FeatureType type, unsigned int size = 0);

    /**
     * get the type of a feature
     */
    FeatureType getFeatureType(const std::string &name) const;

    /**
     * getFeaturesSize
     * Compute number of features we expect to see
     */
    unsigned int getFeaturesSize() const;

    /**
     * getPoseFeatures
     * Load pose data at index
     */
    static void getPoseFeatures(const Pose &pose, FeatureVector &f, unsigned int idx);

    /**
     * updateFeaturesSize
     * Compute number of features we expect to see
     */
    void updateFeaturesSize();

  protected:
    unsigned int features_size;
    std::unordered_map<std::string,FeatureType> feature_types;
    std::unordered_map<std::string,unsigned int> feature_sizes;
  };
}

#endif
