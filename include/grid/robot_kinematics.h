#ifndef _GRID_ROBOT_KINEMATICS
#define _GRID_ROBOT_KINEMATICS

//#include <grid/features.h>
#include <kdl/chain.hpp>
#include <kdl/tree.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>

#include <memory>

namespace grid {

  // grid (for robots) is going to be based on KDL
  typedef KDL::Frame Pose;

  /** 
   * Robot kinematics object.
   * For now it's an Orocos thing.
   * Pass one of these to any particular entity and it will be able to provide the inverse kinematics solving
   * /other stuff necessary to run things behind the scenes.
   *
   * TODO: make this OrocosRobotKinematics, have it inherit from RobotKinematics
   * This way we can have other things like VREP support
   */
  class RobotKinematics {
  public:

    RobotKinematics(const std::string &robot_description_param, const std::string &root_link, const std::string &ee_link);

    KDL::Tree &tree();
    KDL::Chain &chain();

    /**
     * FkPos
     * Compute position forward kinematics
     */
    Pose FkPos(std::vector<double> pos);

    /**
     * get number of degrees of freedom
     */
    unsigned int getDegreesOfFreedom() const;

  protected:
    std::string robot_description_param;
    std::string robot_description;
    std::string ee_link;
    std::string root_link;
    KDL::Chain kdl_chain;
    KDL::Tree kdl_tree;
    std::shared_ptr<KDL::ChainFkSolverPos> kdl_fk_solver_pos;
    unsigned int n_dof;
    int verbose;

  };

  typedef std::shared_ptr<RobotKinematics> RobotKinematicsPointer;
}

#endif
