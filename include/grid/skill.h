#ifndef _GRID_ACTION
#define _GRID_ACTION

#include <vector>
#include <string>

#include <grid/features.h>

/**
 * Skill
 * Probabilistic representation of the soft set of constraints on a single skill.
 * */

namespace grid {

  /**
   * Skill
   * Represents an action as a set of one or more multivariate Gaussians over a set of features.
   * Skills also are associated with TYPES of parameters.
   *
   */
  class Skill {
  protected:

    /** stores feature expectations for the skill */
    GMM exec_model;

  public:

    /**
     * Stores the list of feature names we will be querying.
     */
    std::vector<std::string> feature_names;

    /**
     * concatenate
     * Stick two actions together to generate a combined set of constraints.
     * This is an "OR" operation, not an "AND".
     */
    Skill concatenate(const Skill &a) const;

    /**
     * default skill;
     * one object feature; identity covariance; k=1 gmm
     */
    static Skill DefaultSkill(const std::string &object);
  };
}

#endif
