#include <grid/robot_kinematics.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>

namespace grid {


  RobotKinematics::RobotKinematics(const std::string &robot_description_param_, 
                                   const std::string &root_link_,
                                   const std::string &ee_link_)
    : robot_description_param(robot_description_param_),
    root_link(root_link_),
    ee_link(ee_link_),
    verbose(0)

  {
    ros::NodeHandle nh;
    nh.getParam(robot_description_param, robot_description);

    urdf::Model urdf_model;
    urdf_model.initString(robot_description);
    if (verbose) {
      std::cout << robot_description << std::endl;
    }
    if (!(kdl_parser::treeFromUrdfModel(urdf_model, kdl_tree))) {
      ROS_ERROR("Could not load tree!");
    }

    if (!kdl_tree.getChain(root_link, ee_link, kdl_chain)) {
      ROS_ERROR("Could not get chain from %s to %s!",root_link.c_str(),ee_link.c_str());
    } else {
      kdl_fk_solver_pos.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
    }
    n_dof = getDegreesOfFreedom();
  }

  /**
   * FkPos
   * Compute position forward kinematics
   */
  Pose RobotKinematics::FkPos(std::vector<double> pos) {
    using KDL::JntArray;

    Pose p;
    JntArray q(n_dof);

    //std::cout << "dof = " << n_dof << std::endl;

    for (unsigned int i = 0; i < n_dof; ++i) {
      q(i) = pos[i];
    }

    kdl_fk_solver_pos->JntToCart(q,p);

    return p;
  }

  /**
   * get number of degrees of freedom
   */
  unsigned int RobotKinematics::getDegreesOfFreedom() const {
    return kdl_chain.getNrOfJoints();
  }

  KDL::Tree &RobotKinematics::tree() {
    return kdl_tree;
  }

  KDL::Chain &RobotKinematics::chain() {
    return kdl_chain;
  }
}
