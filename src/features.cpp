#include<grid/features.h>

namespace grid {

  std::vector< FeatureVector > Features::getFeatures(std::vector<std::string> &names) {

    std::vector< std::vector <double> > f;

    for (const std::string &name: names) {

    }
  }

  void Features::addFeature(const std::string &name, const FeatureType type) {
    feature_types[name] = type;
  }

  /**
   * get the type of a feature
   */
  FeatureType Features::getFeatureType(const std::string &name) const {
    return feature_types.at(name);
  }


}
