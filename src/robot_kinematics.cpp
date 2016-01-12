#include <grid/robot_kinematics.h>
#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <ros/ros.h>

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/chainiksolverpos_nr_jl.hpp>

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

      n_dof = getDegreesOfFreedom();

      joint_limits_min.resize(n_dof);
      joint_limits_max.resize(n_dof);

      // load joint limits
      // based off some of Jon's code in lcsr_controllers
      unsigned int i = 0;
      for (auto &link: kdl_chain.segments) {
        if (link.getJoint().getType() != KDL::Joint::None) {
          joint_limits_min(i) = urdf_model.joints_[link.getJoint().getName()]->limits->lower;
          joint_limits_max(i) = urdf_model.joints_[link.getJoint().getName()]->limits->upper;
          ++i;
        }
      }

      unsigned int max_iter_ik_vel = 150u;
      double tol_ik_vel = 1e-6;
      unsigned int max_iter_ik_pos = 10u;
      double tol_ik_pos = 1e-6;

      kdl_fk_solver_pos.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
      kdl_ik_solver_vel.reset(
          new KDL::ChainIkSolverVel_wdls(
              kdl_chain,
              tol_ik_vel,
              max_iter_ik_vel
              )
          );
      kdl_ik_solver_pos.reset(
          new KDL::ChainIkSolverPos_NR_JL(
              kdl_chain,
              joint_limits_min,
              joint_limits_max,
              *kdl_fk_solver_pos,
              *kdl_ik_solver_vel,
              max_iter_ik_pos,
              tol_ik_pos
              )
          );

    }
  }

  /**
   * FkPos
   * Compute position forward kinematics
   */
  Pose RobotKinematics::FkPos(std::vector<double> pos) {
    using KDL::JntArray;

    Pose p;
    JntArray q(n_dof);

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
