

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
  std::string ee("ee_fixed_link");
  RobotKinematicsPtr robot = RobotKinematicsPtr(new RobotKinematics("robot_description","base_link",ee));
  GridPlanner gp("robot_description","joint_states","planning_scene",0.01);
  gp.SetDof(robot->getDegreesOfFreedom());
  gp.SetVerbose(p.collisions_verbose);
  gp.SetCollisions("Obj::link_uniform::1",true);
  gp.SetCollisions("Obj::link_uniform::2",true);
  gp.SetCollisions("Obj::node_uniform::1",true);
  gp.SetCollisions("forearm_link",true);
  gp.SetCollisions("wrist_2_link",true);

  GridPlanner gp2("robot_description","joint_states","planning_scene",0.01);
  gp2.SetDof(robot->getDegreesOfFreedom());
  gp2.SetVerbose(p.collisions_verbose);
  gp2.SetCollisions("forearm_link",true);
  gp2.SetCollisions("wrist_2_link",true);

  // disable a bunch of collisions
  //gp.SetDefaultCollisions("wam/shoulder_yaw_link",true);

  if (p.collisions_verbose) {
    gp.PrintInfo();
  }

  GridPlanner *checker = 0;
  GridPlanner *checker2 = 0;
  if (p.detect_collisions) {
    checker = &gp;
    checker2 = &gp2;
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
  ros::Publisher pub1 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_1",1000);
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

  std::cout << "[link 1] Initializing lineups..." << std::endl;
  InstantiatedSkillPtr lineup1 = InstantiatedSkill::DmpInstance(skills.at("pre_approach_link"), features.at("node1,link1"), robot, nbasis, checker2);
  InstantiatedSkillPtr lineup1b = InstantiatedSkill::DmpInstance(skills.at("pre_approach_link"), features.at("node1,link1b"), robot, nbasis, checker2);
  InstantiatedSkillPtr lineup2 = InstantiatedSkill::DmpInstance(skills.at("pre_approach_link"), features.at("node1,link2"), robot, nbasis, checker2);
  InstantiatedSkillPtr lineup2b = InstantiatedSkill::DmpInstance(skills.at("pre_approach_link"), features.at("node1,link2b"), robot, nbasis, checker2);
  InstantiatedSkillPtr lineup_node1 = InstantiatedSkill::DmpInstance(skills.at("pre_approach_node"), features.at("node1,link1"), robot, nbasis, checker2);

  std::cout << "Initializing approaches..." << std::endl;
  InstantiatedSkillPtr app1 = InstantiatedSkill::DmpInstance(skills.at("approach_link"), features.at("node1,link1"), robot, nbasis, checker2);
  InstantiatedSkillPtr app1b = InstantiatedSkill::DmpInstance(skills.at("approach_link"), features.at("node1,link1b"), robot, nbasis, checker2);
  InstantiatedSkillPtr app_node1 = InstantiatedSkill::DmpInstance(skills["approach_node"], features["node1,link1"], robot, nbasis, checker2);
  InstantiatedSkillPtr app2 = InstantiatedSkill::DmpInstance(skills.at("approach_link"), features.at("node1,link2"), robot, nbasis, checker2);
  InstantiatedSkillPtr app2b = InstantiatedSkill::DmpInstance(skills.at("approach_link"), features.at("node1,link2b"), robot, nbasis, checker2);

  std::cout << "Initializing grasps..." << std::endl;
  InstantiatedSkillPtr grasp1 = InstantiatedSkill::DmpInstance(skills["grasp_link"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr grasp1b = InstantiatedSkill::DmpInstance(skills["grasp_link"], features["node1,link1b"], robot, nbasis, checker);
  InstantiatedSkillPtr grasp2 = InstantiatedSkill::DmpInstance(skills["grasp_link"], features["node1,link2"], robot, nbasis, checker);
  InstantiatedSkillPtr grasp2b = InstantiatedSkill::DmpInstance(skills["grasp_link"], features["node1,link2b"], robot, nbasis, checker);
  InstantiatedSkillPtr grasp_node1 = InstantiatedSkill::DmpInstance(skills["grasp_node"], features["node1,link1"], robot, nbasis, checker);

  std::cout << "Initializing aligns..." << std::endl;
  InstantiatedSkillPtr align11 = InstantiatedSkill::DmpInstance(skills["align_link"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr align11b = InstantiatedSkill::DmpInstance(skills["align_link"], features["node1,link1b"], robot, nbasis, checker);
  InstantiatedSkillPtr align12 = InstantiatedSkill::DmpInstance(skills["align_link"], features["node1,link2"], robot, nbasis, checker);
  InstantiatedSkillPtr align12b = InstantiatedSkill::DmpInstance(skills["align_link"], features["node1,link2b"], robot, nbasis, checker);
  InstantiatedSkillPtr align_node11 = InstantiatedSkill::DmpInstance(skills["align_node"], features["node1,link1"], robot, nbasis, checker);

  std::cout << "Initializing places..." << std::endl;
  InstantiatedSkillPtr place11 = InstantiatedSkill::DmpInstance(skills["place_link"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr place11b = InstantiatedSkill::DmpInstance(skills["place_link"], features["node1,link1b"], robot, nbasis, checker);
  InstantiatedSkillPtr place_node11 = InstantiatedSkill::DmpInstance(skills["place_node"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr place_node11b = InstantiatedSkill::DmpInstance(skills["place_node"], features["node1,link1b"], robot, nbasis, checker);
  InstantiatedSkillPtr place12 = InstantiatedSkill::DmpInstance(skills["place_link"], features["node1,link2"], robot, nbasis, checker);
  InstantiatedSkillPtr place12b = InstantiatedSkill::DmpInstance(skills["place_link"], features["node1,link2b"], robot, nbasis, checker);
  InstantiatedSkillPtr place_node12 = InstantiatedSkill::DmpInstance(skills["place_node"], features["node1,link2"], robot, nbasis, checker);
  InstantiatedSkillPtr place_node12b = InstantiatedSkill::DmpInstance(skills["place_node"], features["node1,link2b"], robot, nbasis, checker);

#if 0
  std::cout << "Initializing releases..." << std::endl;
  InstantiatedSkillPtr release11 = InstantiatedSkill::DmpInstance(skills["release_link"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr release11b = InstantiatedSkill::DmpInstance(skills["release_link"], features["node1,link1b"], robot, nbasis, checker);
  InstantiatedSkillPtr release_node11 = InstantiatedSkill::DmpInstance(skills["release_node"], features["node1,link1"], robot, nbasis, checker);
  InstantiatedSkillPtr release_node11b = InstantiatedSkill::DmpInstance(skills["release_node"], features["node1,link1b"], robot, nbasis, checker);
  InstantiatedSkillPtr release12 = InstantiatedSkill::DmpInstance(skills["release_link"], features["node1,link2"], robot, nbasis, checker);
  InstantiatedSkillPtr release12b = InstantiatedSkill::DmpInstance(skills["release_link"], features["node1,link2b"], robot, nbasis, checker);
  InstantiatedSkillPtr release_node12 = InstantiatedSkill::DmpInstance(skills["release_node"], features["node1,link2"], robot, nbasis, checker);
  InstantiatedSkillPtr release_node12b = InstantiatedSkill::DmpInstance(skills["release_node"], features["node1,link2b"], robot, nbasis, checker);
#endif

  root->addNext(lineup1); 
  root->addNext(lineup1b); 
  root->addNext(lineup2); 
  root->addNext(lineup2b); 
  root->addNext(lineup_node1);

  lineup1->addNext(app1);  lineup1->pub = &pub;
  lineup1b->addNext(app1b);  lineup1b->pub = &pub; 
  lineup2->addNext(app2);  lineup2->pub = &pub;
  lineup2b->addNext(app2b);  lineup2b->pub = &pub; 
  lineup_node1->addNext(app_node1);  lineup_node1->pub = &pub;

  app1->addNext(grasp1); app1->pub = &pub1;
  app1b->addNext(grasp1b); app1->pub = &pub1;
  app2->addNext(grasp2); app2->pub = &pub2;
  app2b->addNext(grasp2b); app2->pub = &pub2;
  app_node1->addNext(grasp_node1); app_node1->pub = &pub1;

  grasp1->addNext(align11); 
  grasp1b->addNext(align11b); 
  grasp2->addNext(align12); 
  grasp2b->addNext(align12b); 
  grasp_node1->addNext(align_node11);

  align11->addNext(place11); align11->pub = &pub2;
  align11b->addNext(place11b); align11b->pub = &pub2;
  align12->addNext(place12); align12->pub = &pub2;
  align12b->addNext(place12b); align12b->pub = &pub2;
  align_node11->addNext(place_node11); align_node11->pub = &pub2;
  align_node11->addNext(place_node11b);
  align_node11->addNext(place_node12);
  align_node11->addNext(place_node12b);

  place11->pub = &pub3;
  place11b->pub = &pub3;
  place12->pub = &pub3;
  place12b->pub = &pub3;

  place_node11->pub = &pub3;
  place_node11b->pub = &pub3;
  place_node12->pub = &pub3;
  place_node12b->pub = &pub3;

  std::vector<InstantiatedSkillPtr> lineups;
  lineups.push_back(lineup1);
  lineups.push_back(lineup1b);
  lineups.push_back(lineup2);
  lineups.push_back(lineup2b);
  lineups.push_back(lineup_node1);

  std::vector<InstantiatedSkillPtr> approaches;
  approaches.push_back(app1);
  approaches.push_back(app1b);
  approaches.push_back(app2);
  approaches.push_back(app2b);
  approaches.push_back(app_node1);
  std::vector<InstantiatedSkillPtr> aligns;
  aligns.push_back(align11);
  aligns.push_back(align11b);
  aligns.push_back(align12);
  aligns.push_back(align12b);
  aligns.push_back(align_node11);
  std::vector<InstantiatedSkillPtr> places;
  places.push_back(place11);
  places.push_back(place11b);
  places.push_back(place_node11);
  places.push_back(place_node11b);
  places.push_back(place12);
  places.push_back(place12b);
  places.push_back(place_node12);
  places.push_back(place_node12b);


  /*************************************************************************/

  std::vector<trajectory_msgs::JointTrajectory> lineup_trajs;
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
      load_to_one_array(lineups,lineup_trajs);
      load_to_one_array(approaches,approach_trajs);
      load_to_one_array(aligns,align_trajs);
      load_to_one_array(places,place_trajs);
      std::cout << "pub len = " << approach_trajs.size() << "\n";
      pub.publish(toPoseArray(lineup_trajs,lineup1->features->getWorldFrame(),robot));
      pub1.publish(toPoseArray(approach_trajs,app1->features->getWorldFrame(),robot));
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

#if 1
  grid_plan::CommandGoal cmd;
  cmd.name = "release";
  std::cout << "waiting for server... (" << horizon << ")\n";
  ac.waitForServer();
  std::cout << "sending command...\n";
  ac.sendGoal(cmd);
  std::cout << "waiting for result\n";
  ac.waitForResult();
#endif

}
