#ifndef _GRID_INSTANTIATED_SKILL
#define _GRID_INSTANTIATED_SKILL

#include <memory>

#include <grid/trajectory_distribution.h>
#include <grid/skill.h>

namespace grid {


  /**
   * Defines a particular instance of a skill
   */
  struct InstantiatedSkill {

    unsigned int id; // unique id for this skill

    std::unordered_map<std::string,std::string> assignment;
    SkillPointer skill; // the skill itself
    TrajectoryDistribution dist; // the path we end up taking for this skill

    std::vector<double> T; // probability of going to each of the possible next actions
    std::vector<InstantiatedSkillPointer> next;

    std::vector<PredicateEffect> effects;
  };


  typedef std::shared_ptr<InstantiatedSkill> InstantiatedSkillPointer;

}

#endif
