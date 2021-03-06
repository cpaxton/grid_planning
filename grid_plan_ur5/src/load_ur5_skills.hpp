#ifndef _GRID_LOAD_WAM_SKILLS
#define _GRID_LOAD_WAM_SKILLS

#include <grid/skill.h>

namespace grid {

  std::unordered_map<std::string, TestFeaturesPtr> setupTestFeaturesForTrials(const std::string &ee) {

    const std::string &base_link = "base_link";

    /* Initialize Base Set of Features */
    TestFeaturesPtr tf11(new TestFeatures());
    tf11->setAgentFrame(ee)
      .setWorldFrame("base_link")
      .addFeature("node",POSE_FEATURE)
      .addFeature("link",POSE_FEATURE)
      .addFeature("time",TIME_FEATURE);

    TestFeaturesPtr tf21(new TestFeatures());
    tf21->setAgentFrame(ee)
      .setWorldFrame("base_link")
      .addFeature("node",POSE_FEATURE)
      .addFeature("link",POSE_FEATURE)
      .addFeature("time",TIME_FEATURE);

    tf11->setFrame("Obj::node_uniform::1","node").setFrame("Obj::link_uniform::1","link");
    tf21->setFrame("Obj::node_uniform::2","node").setFrame("Obj::link_uniform::1","link");

    std::unordered_map<std::string, TestFeaturesPtr> features;

    features["node1,link1"] = tf11;
    features["node2,link1"] = tf21;

    return features;
  }


  std::unordered_map<std::string, SkillPtr> loadSkills(const std::string &ee) {

    std::unordered_map<std::string, SkillPtr> skills;

    SkillPtr approach_link(new Skill("approach_link",3));
    SkillPtr approach_node(new Skill("approach_node",3));
    SkillPtr grasp_link(new Skill("grasp_link",1));
    SkillPtr grasp_node(new Skill("grasp_node",1));
    SkillPtr align_link(new Skill("align_link",3));
    SkillPtr align_node(new Skill("align_node",3));
    SkillPtr place_link(new Skill("place_link",3));
    SkillPtr place_node(new Skill("place_node",3));
    SkillPtr release_link(new Skill("release_link",1));
    SkillPtr release_node(new Skill("release_node",1));

    /* SET UP THE SKILLS */
    approach_link->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false).setPrior(3.0 / 6.0);
    approach_node->appendFeature("node").appendFeature("time").setInitializationFeature("node").setStatic(false).setPrior(1.0 / 6.0);
    grasp_link->appendFeature("link").setInitializationFeature("link").setStatic(true);
    grasp_node->appendFeature("link").setInitializationFeature("link").setStatic(true);
    align_link->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(false);
    align_node->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(false);
    place_link->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(false);
    place_node->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(false);
    release_link->appendFeature("node").setInitializationFeature("node").attachObject("link").setStatic(true);
    release_node->appendFeature("node").setInitializationFeature("node").attachObject("link").setStatic(true);

    /* SET UP THE ROBOT KINEMATICS */
    RobotKinematicsPtr rk_ptr = RobotKinematicsPtr(new RobotKinematics("robot_description","base_link",ee));

    /* LOAD TRAINING DATA FOR APPROACH */
    {
      std::string filenames[] = {
        "data/ur5_assembly_v3/approach_link_x_31.bag",
        "data/ur5_assembly_v3/approach_link_x_32.bag",
        //"data/ur5_assembly_v3/approach_link_x_30.bag",
      };
      load_and_train_skill(*approach_link, rk_ptr, filenames, 2);
    }
    /* LOAD TRAINING DATA FOR APPROACH RIGHT */
    {
      std::string filenames[] = {"data/sim/approach_right01.bag", "data/sim/approach_right02.bag", "data/sim/approach_right03.bag"};
      //load_and_train_skill(*approach_node, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR GRASP */
    {
      std::string filenames[] = {
        "data/ur5_assembly_v3/grasp_link_x_31.bag", 
        "data/ur5_assembly_v3/grasp_link_x_32.bag", 
        //"data/ur5_assembly_v3/grasp_link_x_30.bag",
      };
      load_and_train_skill(*grasp_link, rk_ptr, filenames, 2);
    }
    /* LOAD TRAINING DATA FOR GRASP */
    {
      std::string filenames[] = {"data/sim/grasp01.bag", "data/sim/grasp02.bag", "data/sim/grasp03.bag"};
      //load_and_train_skill(*grasp_node, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR ALIGN */
    {
      std::string filenames[] = {"data/sim/align1.bag", "data/sim/align2.bag", "data/sim/align3.bag"};
      //load_and_train_skill(*align_link, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR ALIGN */
    {
      std::string filenames[] = {"data/sim/align1.bag", "data/sim/align2.bag", "data/sim/align3.bag"};
      //load_and_train_skill(*align_node, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR PLACE */
    {
      std::string filenames[] = {"data/sim/place1.bag", "data/sim/place3.bag"};
      //load_and_train_skill(*place_link, rk_ptr, filenames, 2);
    }
    /* LOAD TRAINING DATA FOR PLACE */
    {
      std::string filenames[] = {"data/sim/place1.bag", "data/sim/place3.bag"};
      //load_and_train_skill(*place_node, rk_ptr, filenames, 2);
    }
    /* LOAD TRAINING DATA FOR RELEASE */
    {
      std::string filenames[] = {"data/sim/release1.bag", "data/sim/release2.bag", "data/sim/release3.bag"};
      //load_and_train_skill(*release_link, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR RELEASE */
    {
      std::string filenames[] = {"data/sim/release1.bag", "data/sim/release2.bag", "data/sim/release3.bag"};
      //load_and_train_skill(*release_node, rk_ptr, filenames, 3);
    }

    skills["approach_link"] = approach_link;
    skills["approach_node"] = approach_node;
    skills["grasp_link"] = grasp_link;
    skills["grasp_node"] = grasp_node;
    skills["align_link"] = align_link;
    skills["align_node"] = align_node;
    skills["place_link"] = place_link;
    skills["place_node"] = place_node;
    skills["release_link"] = release_link;
    skills["release_node"] = release_node;

    return skills;

  }
}

#endif
