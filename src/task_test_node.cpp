

/**
 * TASK MODEL TEST
 * This test creates the default WAM task model, plus TestFeatures object to retrieve data.
 * It creates a bunch of different versions of each of the skills for various different objects in the world.
 */


#include <grid/skill.h>
#include <grid/task_model.h>
#include <grid/robot_kinematics.h>
#include <grid/grid_planner.h>

#include <grid/utils/params.hpp>

#include <grid/wam/input.h>

#include "utils/load_wam_skills.hpp"

using namespace grid;

int main(int argc, char **argv) {

  ros::init(argc,argv,"task_model_test_node");
  Params p = readRosParams();
  RobotKinematicsPointer robot = RobotKinematicsPointer(new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link"));
  GridPlanner gp("robot_description","/gazebo/barrett_manager/wam/joint_states","/gazebo/planning_scene");

  std::unordered_map<std::string, SkillPointer> skills = loadWamSkills();
  std::unordered_map<std::string, TestFeaturesPointer> features = setupTestFeaturesForTrials();

  InstantiatedSkillPointer root = InstantiatedSkill::Root();

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

  InstantiatedSkillPointer grasp1 = InstantiatedSkill::DmpInstance(
      skills["grasp"],
      features["node1,link1"],
      robot,
      5);

  InstantiatedSkillPointer grasp2 = InstantiatedSkill::DmpInstance(
      skills["grasp"],
      features["node2,link2"],
      robot,
      5);

  root->addNext(app1);
  root->addNext(app2);
  root->addNext(prep1);
  root->addNext(prep2);

  app1->addNext(grasp1);
  app1->addNext(app1);

  app2->addNext(grasp2);
  app2->addNext(app2);

}
