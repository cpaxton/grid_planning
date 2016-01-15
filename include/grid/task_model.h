#ifndef _GRID_TASK_MODEL
#define _GRID_TASK_MODEL

#include <unordered_map>

#include <grid/skill.h>
#include <grid/instantiated_skill.h>
#include <grid/trajectory_distribution.h>

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

  /**
   * Defines the whole task model.
   * We add a set of "raw" skills, appropriately configured.
   * We use these to set up transitions.
   * Each skill is instantiated based on the possible assignments of objects to it.
   */
  class TaskModel {

  public:

    /**
     * create an empty task model
     */
    TaskModel();

    /**
     * add an object with a particular class
     */
    TaskModel &addObject(const std::string &obj, const std::string &obj_class);

    /**
     * Enable transitions between different skills
     * This add is only in one direction from skill1 to skill2
     */
    TaskModel &setTransition(const Skill &skill1, const Skill &skill2, bool allowed = true);

    /**
     * add an uninstantiated skill
     * appends it to skills vector
     * adds a reference in skills to id mapping
     */
    TaskModel &addSkill(const Skill &skill);

  protected:

    std::vector<std::string> available_objects; // unique object ids for the current world
    std::unordered_map<std::string,std::string> object_classes; // mapping of objects to classes

    std::vector<double> initial; // initial skill distribution
    std::vector<InstantiatedSkill> inst_skills; // stores all possible actions we could take in this world

    std::vector<Skill> skills; // all possible skills
    std::unordered_map<std::string,unsigned int> skill_to_id; // map skills to unique ids
    std::vector<std::vector<bool> > transitions; // allowed transition matrix

  };

}

#endif
