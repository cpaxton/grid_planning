#include <grid/instantiated_skill.h>

namespace grid {

  unsigned int InstantiatedSkill::next_id(0);

  /** 
   * default constructor
   */
  InstantiatedSkill::InstantiatedSkill()
    : id(next_id++), done(false)
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

}
