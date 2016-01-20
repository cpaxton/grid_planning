

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

#include "wam/load_wam_skills.hpp"

using namespace grid;

void update_features(std::unordered_map<std::string, TestFeaturesPointer> &features) {
  for (auto &nf: features) {
    nf.second->updateWorldfromTF();
  }
}

int main(int argc, char **argv) {

  ros::init(argc,argv,"task_model_test_node");
  Params p = readRosParams();
  RobotKinematicsPointer robot = RobotKinematicsPointer(new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link"));
  GridPlanner gp("robot_description","/gazebo/barrett_manager/wam/joint_states","/gazebo/planning_scene");

  std::unordered_map<std::string, SkillPointer> skills = loadWamSkills();
  std::unordered_map<std::string, TestFeaturesPointer> features = setupTestFeaturesForTrials();

  InstantiatedSkillPointer root = InstantiatedSkill::Root();

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples",1000);
  ros::Publisher pub2 = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples_2",1000);

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

  InstantiatedSkillPointer app1 = InstantiatedSkill::DmpInstance(
      skills.at("approach"),
      features.at("node1,link1"),
      robot,
      5);

  InstantiatedSkillPointer app2 = InstantiatedSkill::DmpInstance(
      skills["approach"],
      features["node2,link2"],
      robot,
      5);

#if 0
  InstantiatedSkillPointer prep1 = InstantiatedSkill::DmpInstance(
      SkillPointer(0),
      features["node1,link1"],
      robot,
      5);

  InstantiatedSkillPointer prep2 = InstantiatedSkill::DmpInstance(
      SkillPointer(0),
      features["node2,link2"],
      robot,
      5);
#endif

  InstantiatedSkillPointer disengage1 = InstantiatedSkill::DmpInstance(
      skills["disengage"],
      features["node1,link1"],
      robot,
      5);

  InstantiatedSkillPointer disengage2 = InstantiatedSkill::DmpInstance(
      skills["disengage"],
      features["node2,link2"],
      robot,
      5);
  
  std::cout << "Initializing grasps..." << std::endl;
  InstantiatedSkillPointer grasp1 = InstantiatedSkill::DmpInstance(skills["grasp"], features["node1,link1"], robot, 5);
  InstantiatedSkillPointer grasp2 = InstantiatedSkill::DmpInstance(skills["grasp"], features["node2,link2"], robot, 5);

  std::cout << "Initializing aligns..." << std::endl;
  InstantiatedSkillPointer align11 = InstantiatedSkill::DmpInstance(skills["align"], features["node1,link1"], robot, 5);
  InstantiatedSkillPointer align12 = InstantiatedSkill::DmpInstance(skills["align"], features["node1,link2"], robot, 5);
  InstantiatedSkillPointer align21 = InstantiatedSkill::DmpInstance(skills["align"], features["node2,link1"], robot, 5);
  InstantiatedSkillPointer align22 = InstantiatedSkill::DmpInstance(skills["align"], features["node2,link2"], robot, 5);

  std::cout << "Initializing places..." << std::endl;
  InstantiatedSkillPointer place11 = InstantiatedSkill::DmpInstance(skills["place"], features["node1,link1"], robot, 5);
  InstantiatedSkillPointer place12 = InstantiatedSkill::DmpInstance(skills["place"], features["node1,link2"], robot, 5);
  InstantiatedSkillPointer place21 = InstantiatedSkill::DmpInstance(skills["place"], features["node2,link1"], robot, 5);
  InstantiatedSkillPointer place22 = InstantiatedSkill::DmpInstance(skills["place"], features["node2,link2"], robot, 5);

  root->addNext(app1);
  root->addNext(app2);
#if 0
  root->addNext(prep1);
  root->addNext(prep2);
#endif

#if 0
  app1->addNext(disengage1);
  app2->addNext(disengage2);
#endif

  app1->addNext(align11);
  app1->addNext(align12);
  app2->addNext(align21);
  app2->addNext(align22);

  align11->addNext(place11);
  align12->addNext(place12);
  align21->addNext(place21);
  align22->addNext(place22);

  /*************************************************************************/

  std::vector<trajectory_msgs::JointTrajectory> approach_trajs;
  std::vector<trajectory_msgs::JointTrajectory> disengage_trajs;

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

  std::cout << starts.size() << std::endl;
  for (unsigned int i = 0; i < starts.size(); ++i) {
    std::cout << starts[i].positions.size() << "\n";
  }

  int horizon = 2;
  double prob = 0;
  for (unsigned int i = 0; i < p.iter; ++i) {
    ros::spinOnce();
    robot->updateHint(gp.currentPos());
    robot->updateVelocityHint(gp.currentVel());

    std::cout << "ITER " << i << std::endl;

    // this is where the magic happens
    ps[0] = 1.; // set prior
    root->step(ps,starts,ps_out,prob,1,horizon,p.ntrajs);

    approach_trajs.resize(0);
    for (auto &traj: app1->trajs) {
      approach_trajs.push_back(traj);
    }

    /* PUT EVERYTHING INTO SOME MESSAGES */
    {
      for (auto &traj: app2->trajs) {
        approach_trajs.push_back(traj);
      }
      disengage_trajs.resize(0);
      for (auto &traj: disengage1->trajs) {
        disengage_trajs.push_back(traj);
      }
      for (auto &traj: disengage2->trajs) {
        disengage_trajs.push_back(traj);
      }
      pub.publish(toPoseArray(approach_trajs,app1->features->getWorldFrame(),robot));
      pub2.publish(toPoseArray(disengage_trajs,disengage1->features->getWorldFrame(),robot));
    }

    ros::Duration(p.wait).sleep();
  }

}
