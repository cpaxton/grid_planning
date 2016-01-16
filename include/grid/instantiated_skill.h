#ifndef _GRID_INSTANTIATED_SKILL
#define _GRID_INSTANTIATED_SKILL

#include <memory>

#include <grid/test_features.h>
#include <grid/dmp_trajectory_distribution.h>
#include <grid/trajectory_distribution.h>
#include <grid/robot_kinematics.h>
#include <grid/skill.h>

namespace grid {

  typedef std::shared_ptr<TestFeatures> TestFeaturesPointer;

  /**
   * creating predicates
   * What is going to change after we do this
   */
  struct PredicateEffect {
    std::string predicate;
    bool value;
    SkillPointer skill; // this is where we actually need to learn the effects model
  };

  class InstantiatedSkill;
  typedef std::shared_ptr<InstantiatedSkill> InstantiatedSkillPointer;

  /**
   * Defines a particular instance of a skill
   */
  class InstantiatedSkill {

  protected:

    static unsigned int next_id;

    unsigned int id; // unique id for this skill
    bool done; // set to true if we don't need to keep evaluating this
    bool touched; // has anyone done anything with this skill yet

    std::unordered_map<std::string,std::string> assignment;
    SkillPointer skill; // the skill itself
    TrajectoryDistributionPointer spline_dist; // the path we end up taking for this skill
    DmpTrajectoryDistributionPointer dmp_dist; // the path we end up taking for this skill
    TestFeaturesPointer features;

    std::vector<double> T; // probability of going to each of the possible next actions
    std::vector<InstantiatedSkillPointer> next;

    std::vector<PredicateEffect> effects;

  public:

    /** 
     * default constructor
     */
    InstantiatedSkill();

    /**
     * set all child skills to not done
     */
    InstantiatedSkill &refresh();

    /**
     * create a new skill with dmps
     */
    static InstantiatedSkillPointer DmpInstance(SkillPointer skill,
                                                TestFeaturesPointer features,
                                                RobotKinematicsPointer robot,
                                                unsigned int nbasis);

    /**
     * create a new skill with spline and segments
     */
    static InstantiatedSkillPointer SplineInstance(SkillPointer skill, TestFeaturesPointer features, unsigned int nseg);

    /**
     * create an empty root node
     */
    static InstantiatedSkillPointer Root();

    /**
     * define a possible child
     */
    InstantiatedSkill &addNext(InstantiatedSkillPointer skill);

    /**
     * run a single iteration of the loop. return a set of trajectories.
     * this is very similar to code in the demo
     */
    void step();

  };


}

#endif
