#ifndef _GRID_SKILL
#define _GRID_SKILL

#include <vector>
#include <string>

#include <grid/features.h>
#include <grid/training_features.h>

// used for other type definitions
#include <grid/dist/gmm.h>


/**
 * Skill
 * Probabilistic representation of the soft set of constraints on a single skill.
 * */

namespace grid {

  typedef gcop::Gmm<> Gmm;
  typedef std::shared_ptr<gcop::Gmm<> > GmmPtr;


  /**
   * Skill
   * Represents an action as a set of one or more multivariate Gaussians over a set of features.
   * Skills also are associated with TYPES of parameters.
   *
   */
  class Skill {

  public:
      typedef Eigen::Matrix<double, Eigen::Dynamic, 1> Vectornd;
      typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> Matrixnd;

    /**
     * return a pose associated with object frame for a given feature
     */
    Pose getInitializationFinalPose() const;

    /**
     * return a pose associated with object frame for a given feature
     */
    Pose &getInitializationFinalPose();

    /**
     * return the exact list of features used for training this
     */
    const std::vector<std::string> &getFeatures() const;

    /**
     * which feature should we use to initialize things?
     */
    Skill &setInitializationFeature(const std::string &feature);

    /**
     * add a feature to this skill
     */
    Skill &appendFeature(const std::string &feature);

    /**
     * set up the name
     */
    Skill &setName(const std::string &name);

    /**
     * return name of this skill
     */
    const std::string &getName() const;

    /**
     * create a skill based on a set of features and a number of clusters
     */
    Skill(int k, std::vector<std::string> &feature_names, Features &feature_lookup);

    /**
     * create a skill based on k and d
     */
    Skill(const std::string &name = "", int k  = 1);

    /**
     * Adding training data
     * What data do we want to use? add as a Features object
     * Store this as a vector
     */
    void addTrainingData(TrainingFeatures &data);

    /**
     * trainSkillModel
     * train the model for expected features while learning this action
     */
    void trainSkillModel();

    /**
     * probabilities
     * what is the probability associated with each feature vector in the data?
     * returns as the log likelihood
     */
    FeatureVector logL(std::vector<FeatureVector> &data);

    /**
     * return the name of the best feature and only that feature
     */
    const std::string &getInitializationFeature() const;

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
    static Skill DefaultSkill(const std::string &name, const std::string &object);

    /**
     * print gmm
     */
    void printGmm();

    /**
     * normalize a bunch of data before sending it through the GMM
     */
    void normalizeData(std::vector<FeatureVector> &data);

  protected:

    /** number of clusters in gmm */
    int k;

    /** stores feature expectations for the skill */
    GmmPtr model;

    /**
     * Stores the list of feature names we will be querying.
     */
    std::vector<std::string> feature_names;

    /**
     * Stored as a reference to compute initial starting trajectory guess
     * This is the "most important" feature
     */
    std::string best_feature_name;

    /**
     * set up the training data
     */
    std::vector<std::pair<FeatureVector, double> > training_data;

    /**
     * set up the training data
     * these are the normalized versions of all that data
     */
    std::vector<std::pair<FeatureVector, double> > normalized_training_data;

    /**
     * skill name
     */
    std::string name;

    Pose init_final; // final pose relative to best_feature_name

    std::vector<Matrixnd> P; // true P as per initialization
    FeatureVector mean; // normalizer mean
    FeatureVector std; // normalizer std dev
  };
}

#endif
