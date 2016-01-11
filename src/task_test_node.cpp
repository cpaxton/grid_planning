

/**
 * TASK MODEL TEST
 * This test creates the default WAM task model, plus TestFeatures object to retrieve data.
 * It creates a bunch of different versions of each of the skills for various different objects in the world.
 */


#include <grid/skill.h>
#include <grid/task_model.h>
#include <grid/robot_kinematics.h>

#include <grid/wam/input.h>

using namespace grid;

int main(int argc, char **argv) {

  ros::init(argc,argv,"task_model_test_node");

  double step_size;
  double noise;
  int ntrajs = 50;
  int iter = 10;
  ros::NodeHandle nh_tilde("~");
  if (not nh_tilde.getParam("step_size",step_size)) {
    step_size = 0.80;
  }
  if (not nh_tilde.getParam("noise",noise)) {
    noise = 1e-10;
  }
  if (not nh_tilde.getParam("ntrajs",ntrajs)) {
    ntrajs = 50;
  }
  if (not nh_tilde.getParam("iter",iter)) {
    iter = 10;
  }

  Skill approach("approach");
  Skill grasp("grasp");
  Skill align("align");
  Skill place("place");
  Skill release("release");
  Skill disengage("disengage");

  approach.appendFeature("link").appendFeature("time").setInitializationFeature("link");
  grasp.appendFeature("link").appendFeature("time").setInitializationFeature("link");
  align.appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link");
  place.appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link");
  release.appendFeature("link").appendFeature("time").setInitializationFeature("link");
  disengage.appendFeature("link").appendFeature("time").setInitializationFeature("link");

  /* SET UP THE ROBOT KINEMATICS */
  RobotKinematicsPointer rk_ptr = RobotKinematicsPointer(new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link"));

  /* LOAD TRAINING DATA FOR APPROACH */
  {
    std::string filenames[] = {"data/sim/app1.bag", "data/sim/app2.bag", "data/sim/app3.bag"};
    load_and_train_skill(approach, rk_ptr, filenames);
  }
  /* LOAD TRAINING DATA FOR GRASP */
  {
    std::string filenames[] = {"data/sim/grasp1.bag", "data/sim/grasp2.bag", "data/sim/grasp3.bag"};
    load_and_train_skill(grasp, rk_ptr, filenames);
  }
  /* LOAD TRAINING DATA FOR ALIGN */
  {
    std::string filenames[] = {"data/sim/align1.bag", "data/sim/align2.bag", "data/sim/align3.bag"};
    load_and_train_skill(align, rk_ptr, filenames);
  }
  /* LOAD TRAINING DATA FOR PLACE */
  {
    std::string filenames[] = {"data/sim/place1.bag", "data/sim/place2.bag", "data/sim/place3.bag"};
    load_and_train_skill(place, rk_ptr, filenames);
  }
  /* LOAD TRAINING DATA FOR RELEASE */
  {
    std::string filenames[] = {"data/sim/release1.bag", "data/sim/release2.bag", "data/sim/release3.bag"};
    load_and_train_skill(release, rk_ptr, filenames);
  }
  /* LOAD TRAINING DATA FOR DISENGAGE */
  {
    std::string filenames[] = {"data/sim/disengage1.bag", "data/sim/disengage2.bag", "data/sim/disengage3.bag"};
    load_and_train_skill(disengage, rk_ptr, filenames);
  }

}
