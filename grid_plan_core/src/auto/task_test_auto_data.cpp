

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

#include "load_wam_skills_auto_data.hpp"

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <grid_plan_msgs/CommandAction.h>

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

  actionlib::SimpleActionClient<grid_plan_msgs::CommandAction> ac("command", true);

  Params p = readRosParams();
  RobotKinematicsPtr robot = RobotKinematicsPtr(new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link"));
  GridPlanner gp("robot_description","/gazebo/barrett_manager/wam/joint_states","/gazebo/raw_planning_scene",0.05);
  gp.SetDof(robot->getDegreesOfFreedom());
  gp.SetCollisions("gbeam_soup",true);
  gp.SetCollisions("gbeam_soup.gbeam_link_1",true);
  gp.SetCollisions("gbeam_soup.gbeam_link_2",true);
  gp.SetCollisions("gbeam_soup.gbeam_node_1",false);
  gp.SetCollisions("gbeam_soup.gbeam_node_2",false);
  gp.SetVerbose(p.collisions_verbose);

  GridPlanner *checker = 0;
  if (p.detect_collisions) {
    checker = &gp;
  }

  ros::ServiceClient client = nh.serviceClient<std_srvs::Empty>("/gazebo/publish_planning_scene");
  std_srvs::Empty empty;
  client.call(empty);

  std::unordered_map<std::string, SkillPtr> skills = loadWamSkills();
  std::unordered_map<std::string, TestFeaturesPtr> features = setupTestFeaturesForTrials();

  InstantiatedSkillPtr root = InstantiatedSkill::Root();

  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples",1000);
  ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_2",1000);
  ros::Publisher pub3 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_3",1000);
  ros::Publisher pub4 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_4",1000);
  ros::Publisher pub5 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_5",1000);
  ros::Publisher pub6 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_6",1000);
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

  InstantiatedSkillPtr app1 = InstantiatedSkill::DmpInstance(
      skills.at("approach"),
      features.at("node1,link1"),
      robot,
      5);

  InstantiatedSkillPtr app2 = InstantiatedSkill::DmpInstance(
      skills["approach"],
      features["node2,link2"],
      robot,
      5);

  std::cout << "Initializing grasps..." << std::endl;
  InstantiatedSkillPtr grasp1 = InstantiatedSkill::DmpInstance(skills["grasp"], features["node1,link1"], robot, 5, checker);
  InstantiatedSkillPtr grasp2 = InstantiatedSkill::DmpInstance(skills["grasp"], features["node2,link2"], robot, 5, checker);

  std::cout << "Initializing aligns..." << std::endl;
  InstantiatedSkillPtr align11 = InstantiatedSkill::DmpInstance(skills["align"], features["node1,link1"], robot, 5, checker);
  InstantiatedSkillPtr align12 = InstantiatedSkill::DmpInstance(skills["align"], features["node2,link1"], robot, 5, checker);
  InstantiatedSkillPtr align21 = InstantiatedSkill::DmpInstance(skills["align"], features["node1,link2"], robot, 5, checker);
  InstantiatedSkillPtr align22 = InstantiatedSkill::DmpInstance(skills["align"], features["node2,link2"], robot, 5, checker);

  std::cout << "Initializing places..." << std::endl;
  InstantiatedSkillPtr place11 = InstantiatedSkill::DmpInstance(skills["place"], features["node1,link1"], robot, 5, checker);
  InstantiatedSkillPtr place12 = InstantiatedSkill::DmpInstance(skills["place"], features["node2,link1"], robot, 5, checker);
  InstantiatedSkillPtr place21 = InstantiatedSkill::DmpInstance(skills["place"], features["node1,link2"], robot, 5, checker);
  InstantiatedSkillPtr place22 = InstantiatedSkill::DmpInstance(skills["place"], features["node2,link2"], robot, 5, checker);

  std::cout << "Initializing releases..." << std::endl;
  InstantiatedSkillPtr release11 = InstantiatedSkill::DmpInstance(skills["release"], features["node1,link1"], robot, 5, checker);
  InstantiatedSkillPtr release12 = InstantiatedSkill::DmpInstance(skills["release"], features["node2,link1"], robot, 5, checker);
  InstantiatedSkillPtr release21 = InstantiatedSkill::DmpInstance(skills["release"], features["node1,link2"], robot, 5, checker);
  InstantiatedSkillPtr release22 = InstantiatedSkill::DmpInstance(skills["release"], features["node2,link2"], robot, 5, checker);

  std::cout << "Initializing disengages..." << std::endl;
  InstantiatedSkillPtr disengage11 = InstantiatedSkill::DmpInstance(skills["disengage"], features["node1,link1"], robot, 5, checker);
  InstantiatedSkillPtr disengage12 = InstantiatedSkill::DmpInstance(skills["disengage"], features["node2,link1"], robot, 5, checker);
  InstantiatedSkillPtr disengage21 = InstantiatedSkill::DmpInstance(skills["disengage"], features["node1,link2"], robot, 5, checker);
  InstantiatedSkillPtr disengage22 = InstantiatedSkill::DmpInstance(skills["disengage"], features["node2,link2"], robot, 5, checker);

  root->addNext(app1); 
  root->addNext(app2);

#if 0
  app1->addNext(disengage1);
  app2->addNext(disengage2);
#endif

#if 1
  app1->addNext(grasp1); app1->pub = &pub;
  app2->addNext(grasp2); app2->pub = &pub;

  grasp1->addNext(align11); 
  grasp1->addNext(align12);
  grasp2->addNext(align21);
  grasp2->addNext(align22);
#else
  app1->addNext(align11);
  app1->addNext(align12);
  app2->addNext(align21);
  app2->addNext(align22);
#endif

  align11->addNext(place11); align11->pub = &pub3;
  align12->addNext(place12); align12->pub = &pub3;
  align21->addNext(place21); align21->pub = &pub3;
  align22->addNext(place22); align22->pub = &pub3;

  place11->addNext(release11); place11->pub = &pub4;
  place12->addNext(release12); place12->pub = &pub4;
  place21->addNext(release21); place21->pub = &pub4;
  place22->addNext(release22); place22->pub = &pub4;

  release11->addNext(disengage11);
  release12->addNext(disengage12);
  release21->addNext(disengage21);
  release22->addNext(disengage22);

  std::vector<InstantiatedSkillPtr> approaches;
  approaches.push_back(app1);
  approaches.push_back(app2);
  std::vector<InstantiatedSkillPtr> aligns;
  aligns.push_back(align11);
  aligns.push_back(align21);
  aligns.push_back(align12);
  aligns.push_back(align22);
  std::vector<InstantiatedSkillPtr> releases;
  releases.push_back(release11);
  releases.push_back(release21);
  releases.push_back(release12);
  releases.push_back(release22);
  std::vector<InstantiatedSkillPtr> places;
  places.push_back(place11);
  places.push_back(place21);
  places.push_back(place12);
  places.push_back(place22);
  std::vector<InstantiatedSkillPtr> grasps;
  grasps.push_back(grasp1);
  grasps.push_back(grasp2);
  std::vector<InstantiatedSkillPtr> disengages;
  disengages.push_back(disengage11);
  disengages.push_back(disengage21);
  disengages.push_back(disengage12);
  disengages.push_back(disengage22);

  /*************************************************************************/

  std::vector<trajectory_msgs::JointTrajectory> approach_trajs;
  std::vector<trajectory_msgs::JointTrajectory> disengage_trajs;
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

#if 0
    align22->useCurrentFeatures = true;
    align22->updateCurrentAttachedObjectFrame();
    place22->useCurrentFeatures = true;
    place22->updateCurrentAttachedObjectFrame();
    release22->useCurrentFeatures = true;
    release22->updateCurrentAttachedObjectFrame();
    //place22->step(ps,starts,ps_out,prob,1,horizon,p.ntrajs);
    align22->step(ps,starts,ps_out,prob,1,horizon,p.ntrajs);
#endif

    /* PUT EVERYTHING INTO SOME MESSAGES */
    {
      load_to_one_array(approaches,approach_trajs);
      load_to_one_array(aligns,align_trajs);
      load_to_one_array(places,place_trajs);
      load_to_one_array(grasps,grasp_trajs);
      load_to_one_array(disengages,disengage_trajs);
      load_to_one_array(releases,release_trajs);
      std::cout << "pub len = " << approach_trajs.size() << "\n";
      pub.publish(toPoseArray(approach_trajs,app1->features->getWorldFrame(),robot));
      pub2.publish(toPoseArray(disengage_trajs,app1->features->getWorldFrame(),robot));
      pub5.publish(toPoseArray(grasp_trajs,grasp1->features->getWorldFrame(),robot));
      pub6.publish(toPoseArray(release_trajs,app1->features->getWorldFrame(),robot));
      pub3.publish(toPoseArray(align_trajs,app1->features->getWorldFrame(),robot));
      attached_pub.publish(toPoseArray(place_trajs,app1->features->getWorldFrame(),robot,align11->features->getAttachedObjectFrame()));
      //attached_pub.publish(toPoseArray(place_trajs,app1->features->getWorldFrame(),robot,place22->getAttachedObjectFrame()));
      pub4.publish(toPoseArray(place_trajs,app1->features->getWorldFrame(),robot));
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
  root->execute(gp,ac,p.max_horizon,false);
  //align22->execute(gp,ac,horizon-1,true);

}
