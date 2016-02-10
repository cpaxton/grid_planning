#ifndef _GRID_LOAD_WAM_SKILLS
#define _GRID_LOAD_WAM_SKILLS

#include <grid/skill.h>

namespace grid {

  std::unordered_map<std::string, TestFeaturesPtr> setupTestFeaturesForTrials() {


    /* Initialize Base Set of Features */
    TestFeaturesPtr tf11(new TestFeatures());
    tf11->setAgentFrame("wam/wrist_palm_link")
      .setWorldFrame("wam/base_link")
      .addFeature("node",POSE_FEATURE)
      .addFeature("link",POSE_FEATURE)
      .addFeature("time",TIME_FEATURE);

    TestFeaturesPtr tf12(new TestFeatures());
    tf12->setAgentFrame("wam/wrist_palm_link")
      .setWorldFrame("wam/base_link")
      .addFeature("node",POSE_FEATURE)
      .addFeature("link",POSE_FEATURE)
      .addFeature("time",TIME_FEATURE);

    TestFeaturesPtr tf21(new TestFeatures());
    tf21->setAgentFrame("wam/wrist_palm_link")
      .setWorldFrame("wam/base_link")
      .addFeature("node",POSE_FEATURE)
      .addFeature("link",POSE_FEATURE)
      .addFeature("time",TIME_FEATURE);

    TestFeaturesPtr tf22(new TestFeatures());
    tf22->setAgentFrame("wam/wrist_palm_link")
      .setWorldFrame("wam/base_link")
      .addFeature("node",POSE_FEATURE)
      .addFeature("link",POSE_FEATURE)
      .addFeature("time",TIME_FEATURE);

    tf11->setFrame("gbeam_node_1/gbeam_node","node").setFrame("gbeam_link_1/gbeam_link","link");
    tf12->setFrame("gbeam_node_1/gbeam_node","node").setFrame("gbeam_link_2/gbeam_link","link");
    tf21->setFrame("gbeam_node_2/gbeam_node","node").setFrame("gbeam_link_1/gbeam_link","link");
    tf22->setFrame("gbeam_node_2/gbeam_node","node").setFrame("gbeam_link_2/gbeam_link","link");

    std::unordered_map<std::string, TestFeaturesPtr> features;

    features["node1,link1"] = tf11;
    features["node1,link2"] = tf12;
    features["node2,link1"] = tf21;
    features["node2,link2"] = tf22;

    return features;
  }


  std::unordered_map<std::string, SkillPtr> loadWamSkills() {

    std::unordered_map<std::string, SkillPtr> skills;

    SkillPtr approach(new Skill("approach",3));
    SkillPtr grasp(new Skill("grasp",1));
    SkillPtr align(new Skill("align",1));
    SkillPtr place(new Skill("place",1));
    SkillPtr release(new Skill("release",1));
    SkillPtr disengage(new Skill("disengage",1));

    /* SET UP THE SKILLS */
    approach->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false);
    grasp->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(true);
    align->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(false);
    place->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(false);
    release->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(true);
    disengage->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false);

    /* SET UP THE ROBOT KINEMATICS */
    RobotKinematicsPtr rk_ptr = RobotKinematicsPtr(new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link"));

    /* LOAD TRAINING DATA FOR APPROACH */
    {
      std::string filenames[] = {"data/sim/app1.bag", "data/sim/app2.bag", "data/sim/app3.bag"};
      load_and_train_skill(*approach, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR GRASP */
    {
      std::string filenames[] = {"data/sim/grasp1.bag", "data/sim/grasp2.bag", "data/sim/grasp3.bag"};
      load_and_train_skill(*grasp, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR ALIGN */
    {
      std::string filenames[] = {"data/sim/align1.bag", "data/sim/align2.bag", "data/sim/align3.bag"};
      load_and_train_skill(*align, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR PLACE */
    {
      std::string filenames[] = {"data/sim/place1.bag", "data/sim/place3.bag", "data/sim/place3.bag"};
      load_and_train_skill(*place, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR RELEASE */
    {
      //std::string filenames[] = {"data/sim/release1.bag", "data/sim/release2.bag", "data/sim/release3.bag"};
      std::string filenames[] = {"data/sim/release1b.bag", "data/sim/release2b.bag", "data/sim/release3b.bag"};
      load_and_train_skill(*release, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR DISENGAGE */
    {
      std::string filenames[] = {"data/sim/disengage1.bag", "data/sim/disengage2.bag", "data/sim/disengage3.bag"};
      load_and_train_skill(*disengage, rk_ptr, filenames, 3);
    }

    skills["approach"] = approach;
    skills["grasp"] = grasp;
    skills["align"] = align;
    skills["place"] = place;
    skills["release"] = release;
    skills["disengage"] = disengage;

    return skills;

  }
}

#endif
