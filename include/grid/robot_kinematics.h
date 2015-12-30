#ifndef _GRID_ROBOT_KINEMATICS
#define _GRID_ROBOT_KINEMATICS

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

namespace grid {

  /** 
   * Robot kinematics object.
   * For now it's an Orocos thing.
   * Pass one of these to any particular entity and it will be able to provide the inverse kinematics solving
   * /other stuff necessary to run things behind the scenes.
   */
  class RobotKinematics {
  public:

    RobotKinematics(const std::string &robot_description_param, const std::string &root_link, const std::string &ee_link);

    KDL::Tree &tree();
    KDL::Chain &chain();

  protected:
    std::string robot_description_param;
    std::string robot_description;
    std::string ee_link;
    std::string root_link;
    KDL::Chain kdl_chain;
    KDL::Tree kdl_tree;
    unsigned int n_dof;

  };
}

#endif
