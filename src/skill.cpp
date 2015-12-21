#include <grid/skill.h>

namespace grid {

    /**
     * concatenate
     * Stick two actions together to generate a combined set of constraints.
     * This is an "OR" operation, not an "AND".
     */
    Skill Skill::concatenate(const Skill &a) const {

    }


    /**
     * default skill;
     * one object feature; identity covariance; k=1 gmm
     */
    Skill Skill::DefaultSkill(const std::string &object) {

      Skill skill(1,POSE_FEATURES_SIZE);
      skill.feature_names.push_back(object);
      skill.best_feature_name = object;

      return skill;
    }


    /**
     * return the name of the best feature and only that feature
     */
    const std::string &Skill::getBestFeature() const {
      return best_feature_name;
    }

    /**
     * create a skill based on a set of features and a number of clusters
     */
    Skill::Skill(int k, std::vector<std::string> &feature_names_, Features &features) :
      feature_names(feature_names_)
    {
      int dim = 0;
      for (std::string &key: feature_names) {
        switch(features.getFeatureType(key)) {
          case POSE_FEATURE:
            dim += POSE_FEATURES_SIZE; break;
          case FLOAT_FEATURE:
            dim += FLOAT_FEATURES_SIZE; break;
          default:
            break;
        }
        exec_model = GmmPtr(new Gmm(dim,k));
      }
    }

    /**
     * create a skill based on k and d
     */
    Skill::Skill(int k, int d) : exec_model(new Gmm(d,k)) {

    }
}
