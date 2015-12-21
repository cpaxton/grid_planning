#include <grid/skill.h>

namespace grid {

    /**
     * concatenate
     * Stick two actions together to generate a combined set of constraints.
     * This is an "OR" operation, not an "AND".
     */
    Skill Skill::concatenate(const Skill &a) const {

    }


    /**
     * default skill;
     * one object feature; identity covariance; k=1 gmm
     */
    Skill Skill::DefaultSkill(const std::string &object) {

    }

}
