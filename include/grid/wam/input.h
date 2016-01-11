#ifndef _GRID_INPUT
#define _GRID_INPUT

#include <grid/skill.h>
#include <grid/robot_kinematics.h>

namespace grid {

  void load_and_train_skill(Skill &skill,
                            RobotKinematicsPointer &rk_ptr,
                            const std::string filenames[]);
}

#endif
