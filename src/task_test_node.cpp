

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

  TestFeatures *test = new TestFeatures();
  test->addFeature("node",grid::POSE_FEATURE);
  test->addFeature("link",grid::POSE_FEATURE);
  test->addFeature("time",grid::TIME_FEATURE);
  test->setAgentFrame("wam/wrist_palm_link");
  //test.setBaseFrame("wam/base_link");
  //test.setWorldFrame("world");
  test->setWorldFrame("wam/base_link");
  test->setFrame("gbeam_node_1/gbeam_node","node");
  test->setFrame("gbeam_link_1/gbeam_link","link");
  
  TestFeaturesPointer test_ptr(test);

  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<geometry_msgs::PoseArray>("trajectory_examples",1000);

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
#if 0
  root->addNext(prep1);
  root->addNext(prep2);
#endif

  app1->addNext(grasp1);
  app1->addNext(app1);

  app2->addNext(grasp2);
  app2->addNext(app2);

  /*************************************************************************/

  for (unsigned int i = 0; i < p.iter; ++i) {
    ros::spinOnce();
    robot->updateHint(gp.currentPos());
    robot->updateVelocityHint(gp.currentVel());

    std::cout << "ITER " << i << std::endl;

    app1->step();
    pub.publish(toPoseArray(app1->trajs,app1->features->getWorldFrame(),robot));

    ros::Duration(p.wait).sleep();
  }

}
