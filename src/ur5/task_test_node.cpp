

/**
 * TASK MODEL TEST
 * This test creates the default WAM task model, plus TestFeatures object to retrieve data.
 * It creates a bunch of different versions of each of the skills for various different objects in the world.
 */


#include <grid/skill.h>
#include <grid/task_model.h>
#include <grid/robot_kinematics.h>
#include <grid/grid_planner.h>
#include <grid/visualize.h>
#include <grid/utils/params.h>
#include <grid/wam/input.h>

#include "load_ur5_skills.hpp"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <grid_plan/CommandAction.h>

using namespace grid;

void update_features(std::unordered_map<std::string, TestFeaturesPtr> &features) {
  for (auto &nf: features) {
    nf.second->updateWorldfromTF();
  }
}

void load_to_one_array(std::vector<InstantiatedSkillPtr> &is, std::vector<JointTrajectory> &trajs) {
  trajs.resize(0);
  for (auto &ptr: is) {
    if (ptr->last_samples > 0) { // and ptr->last_probability > 1e-199) {
      for (auto &traj: ptr->trajs) {
        trajs.push_back(traj);
      }
    }
  }
}

int main(int argc, char **argv) {

  ros::init(argc,argv,"task_model_test_node");
  ros::NodeHandle nh;

  actionlib::SimpleActionClient<grid_plan::CommandAction> ac("command", true);

  Params p = readRosParams();
  std::string ee("ee_link");
  RobotKinematicsPtr robot = RobotKinematicsPtr(new RobotKinematics("robot_description","base_link",ee));
  GridPlanner gp("robot_description","joint_states","planning_scene",0.05);
  gp.SetDof(robot->getDegreesOfFreedom());
  gp.SetCollisions("gbeam_soup.gbeam_link_1",true);
  gp.SetCollisions("gbeam_soup.gbeam_link_2",true);
  gp.SetCollisions("gbeam_soup.gbeam_node_1",false);
  gp.SetCollisions("gbeam_soup.gbeam_node_2",false);

  // disable a bunch of collisions
  //gp.SetDefaultCollisions("wam/shoulder_yaw_link",true);


  gp.SetVerbose(p.collisions_verbose);

  if (p.collisions_verbose) {
    gp.PrintInfo();
  }

  GridPlanner *checker = 0;
  GridPlanner *checker2 = 0;
  if (p.detect_collisions) {
    checker = &gp;
    checker2 = &gp;
  }

  std::unordered_map<std::string, SkillPtr> skills;
  if (p.test == 0) {
    skills = loadSkills(ee);
  } else if (p.test == 1) {
    //skills = loadWamSkillsAuto();
    exit(-1);
  }
  std::unordered_map<std::string, TestFeaturesPtr> features = setupTestFeaturesForTrials(ee);

  InstantiatedSkillPtr root = InstantiatedSkill::Root();

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples",1000);
  ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_2",1000);
  ros::Publisher pub3 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_3",1000);
  ros::Publisher attached_pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_attached",1000);

  ros::spinOnce();
  robot->updateHint(gp.currentPos());
  robot->updateVelocityHint(gp.currentVel());

  std::cout << "sleeping...\n";

  ros::Duration(1.0).sleep();

  ros::spinOnce();
  robot->updateHint(gp.currentPos());
  robot->updateVelocityHint(gp.currentVel());

  update_features(features);

  /*************************************************************************/

  unsigned int nbasis = 5;
  InstantiatedSkillPtr app1 = InstantiatedSkill::DmpInstance(skills.at("approach_link"), features.at("node1,link1"), robot, nbasis, checker2);
  InstantiatedSkillPtr app2 = InstantiatedSkill::DmpInstance(skills["approach_node"], features["node1,link1"], robot, nbasis, checker2);

  std::cout << "Initializing grasps..." << std::endl;
  InstantiatedSkillPtr grasp1 = InstantiatedSkill::DmpInstance(skills["grasp_link"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr grasp2 = InstantiatedSkill::DmpInstance(skills["grasp_node"], features["node1,link1"], robot, nbasis, checker);

  std::cout << "Initializing aligns..." << std::endl;
  InstantiatedSkillPtr align11 = InstantiatedSkill::DmpInstance(skills["align_link"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr align21 = InstantiatedSkill::DmpInstance(skills["align_node"], features["node1,link1"], robot, nbasis, checker);

  std::cout << "Initializing places..." << std::endl;
  InstantiatedSkillPtr place11 = InstantiatedSkill::DmpInstance(skills["place_link"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr place21 = InstantiatedSkill::DmpInstance(skills["place_node"], features["node1,link1"], robot, nbasis, checker);

  std::cout << "Initializing releases..." << std::endl;
  InstantiatedSkillPtr release11 = InstantiatedSkill::DmpInstance(skills["release_link"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr release21 = InstantiatedSkill::DmpInstance(skills["release_node"], features["node1,link1"], robot, nbasis, checker);

  root->addNext(app1); 
  //root->addNext(app2);

  app1->addNext(grasp1); app1->pub = &pub;
  app2->addNext(grasp2); app2->pub = &pub;

  grasp1->addNext(align11); 
  grasp2->addNext(align21);

  align11->addNext(place11); align11->pub = &pub2;
  align21->addNext(place21); align21->pub = &pub2;

  place11->addNext(release11); place11->pub = &pub3;
  place21->addNext(release21); place21->pub = &pub3;

  std::vector<InstantiatedSkillPtr> approaches;
  approaches.push_back(app1);
  approaches.push_back(app2);
  std::vector<InstantiatedSkillPtr> aligns;
  aligns.push_back(align11);
  aligns.push_back(align21);
  std::vector<InstantiatedSkillPtr> releases;
  releases.push_back(release11);
  releases.push_back(release21);
  std::vector<InstantiatedSkillPtr> places;
  places.push_back(place11);
  places.push_back(place21);
  std::vector<InstantiatedSkillPtr> grasps;
  grasps.push_back(grasp1);
  grasps.push_back(grasp2);

  /*************************************************************************/

  std::vector<trajectory_msgs::JointTrajectory> approach_trajs;
  std::vector<trajectory_msgs::JointTrajectory> align_trajs;
  std::vector<trajectory_msgs::JointTrajectory> place_trajs;
  std::vector<trajectory_msgs::JointTrajectory> grasp_trajs;
  std::vector<trajectory_msgs::JointTrajectory> release_trajs;

  //std::vector<double> ps(1.0,p.ntrajs);
  //std::vector<trajectory_msgs::JointTrajectoryPoint> starts(p.ntrajs);
  std::vector<double> ps(1.0,1);
  std::vector<double> ps_out(1.0,1);
  std::vector<trajectory_msgs::JointTrajectoryPoint> starts(1);

  for (auto &pt: starts) {
    pt.positions = gp.currentPos();
    pt.velocities = gp.currentVel();
  }
  ps[0] = 1.;

  std::vector<double> iter_p(p.iter);

  int horizon = p.starting_horizon;
  double prob = 0;
  for (unsigned int i = 0; i < p.iter; ++i) { 
    assert(ros::ok());
    ros::spinOnce();
    robot->updateHint(gp.currentPos());
    robot->updateVelocityHint(gp.currentVel());


    // this is where the magic happens
    //ps_out[0] = 0.;
    //ps[0] = 1.; // set prior
    ps_out[0] = 0.;
    ps[0] = 0.; // set prior
    root->step(ps,starts,ps_out,prob,1,horizon,p.ntrajs);

    /* PUT EVERYTHING INTO SOME MESSAGES */
    {
      load_to_one_array(approaches,approach_trajs);
      load_to_one_array(aligns,align_trajs);
      load_to_one_array(places,place_trajs);
      std::cout << "pub len = " << approach_trajs.size() << "\n";
      pub.publish(toPoseArray(approach_trajs,app1->features->getWorldFrame(),robot));
      pub2.publish(toPoseArray(align_trajs,app1->features->getWorldFrame(),robot));
      pub3.publish(toPoseArray(place_trajs,app1->features->getWorldFrame(),robot));
      attached_pub.publish(toPoseArray(place_trajs,app1->features->getWorldFrame(),robot,align11->features->getAttachedObjectFrame()));
    }

    iter_p[i] = exp(ps_out[0]);
    std::cout << "\n\n===================================\n";
    std::cout << "ITER " << i; // << std::endl;
    std::cout << ": " << iter_p[i] << " ... ";
    if (i > 1) {

      std::cout << fabs(iter_p[i] - iter_p[i-1]) << " < " << (p.update_horizon * iter_p[i]);
      std::cout << std::endl;

      if (fabs(iter_p[i] - iter_p[i-1]) < (p.update_horizon * iter_p[i])) {
        ++horizon;
        std::cout << "horizon = " << horizon << "\n";
        if (horizon > p.max_horizon) {
          std::cout << std::endl;
          --horizon; // don't execute that last node
          break;
        } else {
          root->refresh(horizon-1);
        }
        //root->refresh(horizon);
      }
    } else {
      std::cout << std::endl;
    }

    ros::Duration(p.wait).sleep();
  }

  // execute here
  root->execute(gp,ac,p.execute_depth,false,p.replan_depth);

}
