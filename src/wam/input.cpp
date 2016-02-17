#include <grid/wam/input.h>

#include <grid/wam_training_features.h>

namespace grid {


  void load_and_train_skill(Skill &skill, RobotKinematicsPtr &rk_ptr, const std::string filenames[], unsigned int len, unsigned int k) {
    /* LOAD TRAINING DATA FOR GRASP*/

    if (len == 0) {
      len = 3;
    }

    std::vector<std::string> objects;
    objects.push_back("link");
    objects.push_back("node");

    std::vector<std::shared_ptr<WamTrainingFeatures> > wtf(len);
    for (unsigned int i = 0; i < len; ++i) {
      std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
      wtf_ex->setUseDiff(not skill.isStatic());
      wtf_ex->addFeature("time",TIME_FEATURE);
      wtf_ex->setRobotKinematics(rk_ptr);
      wtf_ex->read(filenames[i],10);
      if (skill.hasAttachedObject()) {
        wtf_ex->attachObjectFrame(skill.getAttachedObject());
      }
      wtf[i] = wtf_ex;
    }

    // add each skill
    for (unsigned int i = 0; i < len; ++i) {
      skill.addTrainingData(*wtf[i]);
    }
    skill.trainSkillModel();

    for (unsigned int i = 0; i < len; ++i) {
      std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
      wtf_ex->setUseDiff(not skill.isStatic());
      wtf_ex->addFeature("time",TIME_FEATURE);
      wtf_ex->setRobotKinematics(rk_ptr);
      wtf_ex->read(filenames[i],10);
      if (skill.hasAttachedObject()) {
        wtf_ex->attachObjectFrame(skill.getAttachedObject());
      }
      std::vector<FeatureVector> data = wtf_ex->getFeatureValues(skill.getFeatures());

#if 0
      for (FeatureVector &vec: data) {
        std::pair<FeatureVector,double> obs(vec,1.0);
        for (unsigned int i = 0; i < vec.size(); ++i) {
          std::cout << vec(i) << " ";
        }
        std::cout << std::endl;
      }
#endif

      skill.normalizeData(data);
      FeatureVector v = skill.logL(data);
      double p = v.array().exp().sum() / v.size();
      std::cout << "[" << skill.getName() << "] training example " << i << " with " << data.size() << " examples: p = " << p << std::endl;
    }

    //skill.printGmm();
  }



}
