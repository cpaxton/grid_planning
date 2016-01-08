#ifndef _GRID_TASK_MODEL
#define _GRID_TASK_MODEL

#include <unordered_map>
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

    std::vector<double> transitions; // probability of going to each of the possible next actions
    
  };

  class TaskModel {

  public:

    /**
     * create an empty task model
     */
    TaskModel();

  protected:

    std::vector<double> initial; // initial skill distribution
    std::vector<InstantiatedSkill> skills; // stores all possible actions we could take in this world

  };

}

#endif
