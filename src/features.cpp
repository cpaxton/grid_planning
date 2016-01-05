#include<grid/features.h>

namespace grid {

  std::vector< FeatureVector > Features::getFeatures(std::vector<std::string> &names) {

    std::vector< std::vector <double> > f;

    for (const std::string &name: names) {

    }
  }

  /**
   * add a feature
   * also updates expected array size
   */
  void Features::addFeature(const std::string &name, const FeatureType type, unsigned int size) {
    feature_types[name] = type;
    if (type == POSE_FEATURE) {
      feature_sizes[name] = POSE_FEATURES_SIZE;
    } else {
      feature_sizes[name] = size;
    }
    updateFeaturesSize();
  }

  /**
   * get the type of a feature
   */
  FeatureType Features::getFeatureType(const std::string &name) const {
    return feature_types.at(name);
  }

  /**
   * updateFeaturesSize
   * Compute number of features we expect to see
   */
  void Features::updateFeaturesSize() {
    features_size = 0;
    for (std::pair<const std::string,unsigned int> &pair: feature_sizes) {
      features_size += pair.second;
    }
  }
  /**
   * getFeaturesSize
   * Compute number of features we expect to see
   */
  unsigned int Features::getFeaturesSize() const {
    return features_size;
  }

  /** 
   * setRobotKinematics
   * sets the thing that will actually compute forward and inverse kinematics for our robot
   */
  void Features::setRobotKinematics(std::shared_ptr<RobotKinematics> rk) {
    robot = rk;
    n_dof = robot->getDegreesOfFreedom();
  }

  /**
   * getPoseFeatures
   * Load pose data at index
   */
  void Features::getPoseFeatures(const Pose &pose, FeatureVector &f, unsigned int idx) {

#if 0
    f[idx+POSE_FEATURE_X] = pose.p.x();
    f[idx+POSE_FEATURE_Y] = pose.p.y();
    f[idx+POSE_FEATURE_Z] = pose.p.z();
    pose.M.GetRPY(f[idx+POSE_FEATURE_ROLL], f[idx+POSE_FEATURE_PITCH], f[idx+POSE_FEATURE_YAW]);
#endif

    f(idx+POSE_FEATURE_X) = pose.p.x();
    f(idx+POSE_FEATURE_Y) = pose.p.y();
    f(idx+POSE_FEATURE_Z) = pose.p.z();
    pose.M.GetRPY(f(idx+POSE_FEATURE_ROLL), f(idx+POSE_FEATURE_PITCH), f(idx+POSE_FEATURE_YAW));
  }
}
