#include <grid/skill.h>

namespace grid {

  /**
   * concatenate
   * Stick two actions together to generate a combined set of constraints.
   * This is an "OR" operation, not an "AND".
   */
  Skill Skill::concatenate(const Skill &a) const {
    std::cerr << __FILE__ << ":" << __LINE__ << ": Concatenate not implemented!" << std::endl;
  }


  /**
   * default skill;
   * one object feature; identity covariance; k=1 gmm
   */
  Skill Skill::DefaultSkill(const std::string &name, const std::string &object) {

    //Skill skill(1,POSE_FEATURES_SIZE);
    Skill skill(name,1);
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
    unsigned int dim = features.getFeaturesSize();
    model = GmmPtr(new Gmm(dim,k));
  }

  /**
   * create a skill based on k and d
   */
  Skill::Skill(const std::string &name_, int k_) : name(name_), k(k_) {
    // do nothing for now
  }

  /**
   * Adding training data
   * What data do we want to use? add as a Features object
   * Store this as a vector
   */
  void Skill::addTrainingData(TrainingFeatures &data) {

    std::vector<FeatureVector> ex_data = data.getAllFeatureValues();

    for (FeatureVector &ex: ex_data) {
      std::pair<FeatureVector,double> obs(ex,1.0);
      training_data.push_back(obs);
    }

  }

  /**
   * trainSkillModel
   * train the model for expected features while learning this action
   */
  void Skill::trainSkillModel() {
    if (training_data.size() > 0) {

      unsigned int dim = training_data[0].first.size();
      model = GmmPtr(new Gmm(dim,k));

      model->Init(training_data.begin()->first,training_data.rbegin()->first);
      model->Fit(training_data);

    } else {
      std::cerr << __FILE__ << ":" << __LINE__ << ": Tried to train empty Skill!" << std::endl;
    }
  }

  /**
   * probabilities
   * what is the probability associated with each feature vector in the data?
   */
  FeatureVector Skill::p(std::vector<FeatureVector> data) {
    FeatureVector vec(data.size());


    return vec;
  }

  /**
   * set up the name
   */
  void Skill::setName(const std::string &name_) {
    name = name_;
  }

  /**
   * return name of this skill
   */
  const std::string &Skill::getName() const {
    return name;
  }
}
