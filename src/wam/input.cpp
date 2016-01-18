#include <grid/wam/input.h>

#include <grid/wam_training_features.h>

namespace grid {


  void load_and_train_skill(Skill &skill, RobotKinematicsPointer &rk_ptr, const std::string filenames[]) {
    /* LOAD TRAINING DATA FOR GRASP*/

    std::vector<std::string> objects;
    objects.push_back("link");
    objects.push_back("node");

    std::vector<std::shared_ptr<WamTrainingFeatures> > wtf(3);
    for (unsigned int i = 0; i < 3; ++i) {
      std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
      wtf_ex->addFeature("time",TIME_FEATURE);
      wtf_ex->setRobotKinematics(rk_ptr);
      wtf_ex->read(filenames[i]);
      if (skill.hasAttachedObject()) {
        //std::cout << "attaching object \"" << skill.attachedObjectFrame() << "\"\n";
        wtf_ex->attachObjectFrame(skill.attachedObjectFrame());
      }
      wtf[i] = wtf_ex;
    }

    // add each skill
    for (unsigned int i = 0; i < 3; ++i) {
      skill.addTrainingData(*wtf[i]);
    }
    skill.trainSkillModel();

    for (unsigned int i = 0; i < 3; ++i) {
      std::shared_ptr<WamTrainingFeatures> wtf_ex(new WamTrainingFeatures(objects));
      wtf_ex->addFeature("time",TIME_FEATURE);
      wtf_ex->setRobotKinematics(rk_ptr);
      wtf_ex->read(filenames[i]);
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
  }



}
