#include<grid/features.h>

namespace grid {

  const std::string Features::AGENT("agent");
  const unsigned int Features::POSE_FEATURES_SIZE(6);
  const unsigned int Features::POSE_FEATURE_X(0);
  const unsigned int Features::POSE_FEATURE_Y(1);
  const unsigned int Features::POSE_FEATURE_Z(2);
  const unsigned int Features::POSE_FEATURE_ROLL(3);
  const unsigned int Features::POSE_FEATURE_PITCH(4);
  const unsigned int Features::POSE_FEATURE_YAW(5);

  std::vector< FeatureVector > Features::getFeatures(std::vector<std::string> &names) {

    std::vector< std::vector <double> > f;

    for (const std::string &name: names) {

    }
  }

  void Features::addFeature(const std::string &name, const FeatureType type) {
    feature_types[name] = type;
  }

}
