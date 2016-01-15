

/**
 * TASK MODEL TEST
 * This test creates the default WAM task model, plus TestFeatures object to retrieve data.
 * It creates a bunch of different versions of each of the skills for various different objects in the world.
 */


#include <grid/skill.h>
#include <grid/task_model.h>
#include <grid/robot_kinematics.h>

#include <grid/utils/params.hpp>

#include <grid/wam/input.h>

#include "utils/load_wam_skills.hpp"

using namespace grid;

int main(int argc, char **argv) {

  ros::init(argc,argv,"task_model_test_node");

  Params p = readRosParams();

  std::unordered_map<std::string, SkillPointer> skills = loadWamSkills();
  std::unordered_map<std::string, TestFeaturesPointer> features = setupTestFeaturesForTrials();

}
