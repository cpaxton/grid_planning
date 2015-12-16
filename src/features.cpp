#include<grid/features.h>

namespace grid {

  const std::string Features::AGENT("agent");
  const unsigned int Features::POSE_FEATURES_SIZE(6);

    std::vector< std::vector <double> > Features::getFeatures(std::vector<std::string> &names) {

      std::vector< std::vector <double> > f;

      for (const std::string &name: names) {

      }
    }

    void Features::addFeature(const std::string &name, const FeatureType type) {
      feature_types[name] = type;
    }

}
