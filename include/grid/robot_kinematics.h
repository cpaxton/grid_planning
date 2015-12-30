#ifndef _GRID_ROBOT_KINEMATICS
#define _GRID_ROBOT_KINEMATICS

namespace grid {

  /** 
   * Robot kinematics object.
   * For now it's an Orocos thing.
   * Pass one of these to any particular entity and it will be able to provide the inverse kinematics solving
   * /other stuff necessary to run things behind the scenes.
   */
  class RobotKinematics {

    RobotKinematics(const std::string &robot_description_param, const std::string &root_link, const std::string &ee_link);

  }
}

#endif
