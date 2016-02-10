#include <grid/instantiated_skill.h>

namespace grid {


  /**
   * create a new skill with dmps
   */
  InstantiatedSkillPtr InstantiatedSkill::DmpInstance(SkillPtr skill,
                                                      TestFeaturesPtr features,
                                                      RobotKinematicsPtr robot,
                                                      unsigned int nbasis,
                                                      GridPlanner *checker)
  {

    Params p = readRosParams();
    InstantiatedSkillPtr is(new InstantiatedSkill(p));
    is->skill = skill;
    is->features = features;
    is->robot = robot;
    is->dmp_dist = DmpTrajectoryDistributionPtr(
        new DmpTrajectoryDistribution(robot->getDegreesOfFreedom(),
                                      nbasis,
                                      robot));
    is->dmp_dist->attachObjectFrame(skill->getDefaultAttachedObjectPose());
    is->dmp_dist->initialize(*features,*skill);

    if(checker) {
      is->dmp_dist->setCollisionChecker(checker);
    }

    return is;
  }

  /**
   * create a new skill with dmps
   */
  InstantiatedSkillPtr InstantiatedSkill::DmpInstance(SkillPtr skill,
                                                      SkillPtr grasp,
                                                      TestFeaturesPtr features,
                                                      RobotKinematicsPtr robot,
                                                      unsigned int nbasis,
                                                      GridPlanner *checker)
  {

    Params p = readRosParams();
    InstantiatedSkillPtr is(new InstantiatedSkill(p));
    is->skill = skill;
    is->features = features;
    is->robot = robot;
    is->dmp_dist = DmpTrajectoryDistributionPtr(
        new DmpTrajectoryDistribution(robot->getDegreesOfFreedom(),
                                      nbasis,
                                      robot));
    is->dmp_dist->attachObjectFromSkill(*grasp);
    is->dmp_dist->initialize(*features,*skill);

    if(checker) {
      is->dmp_dist->setCollisionChecker(checker);
    }

    return is;
  }

  /**
   * create a new skill with spline and segments
   */
  InstantiatedSkillPtr InstantiatedSkill::SplineInstance(SkillPtr skill,
                                                         TestFeaturesPtr features,
                                                         RobotKinematicsPtr robot,
                                                         unsigned int nseg)
  {

    InstantiatedSkillPtr is(new InstantiatedSkill());
    is->skill = skill;
    is->features = features;
    is->robot = robot;
    is->spline_dist = TrajectoryDistributionPtr(new TrajectoryDistribution(nseg));
    is->spline_dist->initialize(*features,*skill);

    return is;
  }

  /**
   * create an empty root node
   */
  InstantiatedSkillPtr InstantiatedSkill::Root() {
    Params p = readRosParams();
    return InstantiatedSkillPtr (new InstantiatedSkill(p));
  }

  /**
   * define a possible child
   */
  InstantiatedSkill &InstantiatedSkill::addNext(InstantiatedSkillPtr skill) {
    next.push_back(skill);
    T.push_back(1);
    last_T.push_back(1);

    updateTransitions();
  }


}
