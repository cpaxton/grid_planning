#include <grid/instantiated_skill.h>

namespace grid {

  unsigned int InstantiatedSkill::next_id(0);

  /** 
   * default constructor
   */
  InstantiatedSkill::InstantiatedSkill()
    : id(next_id++), done(false), spline_dist(0), dmp_dist(0), skill(0)
  {
  }

  /**
   * set all child skills to not done
   * assumes children of an unfinished node are not finished
   */
  InstantiatedSkill &InstantiatedSkill::refresh() {
    if (done) {
      for (InstantiatedSkillPointer ptr: next) {
        ptr->refresh(); 
      }
      done = false;
    }
    return *this;
  }


    /**
     * create a new skill with dmps
     */
    InstantiatedSkillPointer InstantiatedSkill::DmpInstance(SkillPointer skill,
                                                            TestFeaturesPointer features,
                                                            RobotKinematicsPointer robot,
                                                            unsigned int nbasis)
    {

      InstantiatedSkillPointer is(new InstantiatedSkill());
      is->skill = skill;
      is->features = features;
      is->dmp_dist = DmpTrajectoryDistributionPointer(
          new DmpTrajectoryDistribution(robot->getDegreesOfFreedom(),
                                        nbasis,
                                        robot));

      return is;
    }

    /**
     * create a new skill with spline and segments
     */
    InstantiatedSkillPointer InstantiatedSkill::SplineInstance(SkillPointer skill,
                                                               TestFeaturesPointer features,
                                                               unsigned int nseg)
    {

      InstantiatedSkillPointer is(new InstantiatedSkill());
      is->skill = skill;
      is->features = features;
      is->spline_dist = TrajectoryDistributionPointer(new TrajectoryDistribution(nseg));

      return is;
    }

    /**
     * create an empty root node
     */
    InstantiatedSkillPointer InstantiatedSkill::Root() {
      return InstantiatedSkillPointer(new InstantiatedSkill());
    }

    /**
     * define a possible child
     */
    InstantiatedSkill &InstantiatedSkill::addNext(InstantiatedSkillPointer skill) {
      next.push_back(skill);
      T.push_back(1);
    }


    /**
     * run a single iteration of the loop. return a set of trajectories.
     * this is very similar to code in the demo
     */
    void InstantiatedSkill::step() {


    }

}
