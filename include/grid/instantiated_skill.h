#ifndef _GRID_INSTANTIATED_SKILL
#define _GRID_INSTANTIATED_SKILL

#include <memory>

#include <grid/dmp_trajectory_distribution.h>
#include <grid/trajectory_distribution.h>
#include <grid/skill.h>

namespace grid {

  /**
   * creating predicates
   * What is going to change after we do this
   */
  struct PredicateEffect {
    std::string predicate;
    bool value;
    SkillPointer skill; // this is where we actually need to learn the effects model
  };

  struct InstantiatedSkill;
  typedef std::shared_ptr<InstantiatedSkill> InstantiatedSkillPointer;

  /**
   * Defines a particular instance of a skill
   */
  struct InstantiatedSkill {

    static unsigned int next_id;

    unsigned int id; // unique id for this skill
    bool done; // set to true if we don't need to keep evaluating this

    std::unordered_map<std::string,std::string> assignment;
    SkillPointer skill; // the skill itself
    TrajectoryDistributionPointer spline_dist; // the path we end up taking for this skill
    DmpTrajectoryDistributionPointer dmp_dist; // the path we end up taking for this skill

    std::vector<double> T; // probability of going to each of the possible next actions
    std::vector<InstantiatedSkillPointer> next;

    std::vector<PredicateEffect> effects;


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
    InstantiatedSkillPointer DmpInstance(Skill &skill, unsigned int nbasis);

    /**
     * create a new skill with spline and segments
     */
    InstantiatedSkillPointer SplineInstance(Skill &skill, unsigned int nseg);

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
