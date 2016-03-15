#ifndef _GRID_LOAD_WAM_SKILLS
#define _GRID_LOAD_WAM_SKILLS

#include <grid/skill.h>

namespace grid {

  std::unordered_map<std::string, TestFeaturesPtr> setupTestFeaturesForTrials(const std::string &ee) {

    const std::string &base_link = "base_link";

    /* Initialize Base Set of Features */
    TestFeaturesPtr tf11(new TestFeatures());
    TestFeaturesPtr tf11b(new TestFeatures());
    TestFeaturesPtr tf12(new TestFeatures());
    TestFeaturesPtr tf12b(new TestFeatures());
    tf11->setAgentFrame(ee).setWorldFrame("base_link").addFeature("node",POSE_FEATURE).addFeature("link",POSE_FEATURE).addFeature("time",TIME_FEATURE);
    tf11b->setAgentFrame(ee).setWorldFrame("base_link").addFeature("node",POSE_FEATURE).addFeature("link",POSE_FEATURE).addFeature("time",TIME_FEATURE);
    tf12->setAgentFrame(ee).setWorldFrame("base_link").addFeature("node",POSE_FEATURE).addFeature("link",POSE_FEATURE).addFeature("time",TIME_FEATURE);
    tf12b->setAgentFrame(ee).setWorldFrame("base_link").addFeature("node",POSE_FEATURE).addFeature("link",POSE_FEATURE).addFeature("time",TIME_FEATURE);

    tf11->setFrame("Obj::node_uniform::1","node").setFrame("Obj::link_uniform::1","link");
    tf11b->setFrame("Obj::node_uniform::1","node").setFrame("Obj::link_uniform::1b","link");
    tf12->setFrame("Obj::node_uniform::1","node").setFrame("Obj::link_uniform::2","link");
    tf12b->setFrame("Obj::node_uniform::1","node").setFrame("Obj::link_uniform::2b","link");
    //tf21->setFrame("Obj::node_uniform::2","node").setFrame("Obj::link_uniform::1","link");

    std::unordered_map<std::string, TestFeaturesPtr> features;

    features["node1,link1"] = tf11;
    features["node1,link1b"] = tf11b;
    features["node1,link2"] = tf12;
    features["node1,link2b"] = tf12b;
    //features["node2,link1"] = tf21;

    return features;
  }


  std::unordered_map<std::string, SkillPtr> loadSkills(const std::string &ee) {

    std::unordered_map<std::string, SkillPtr> skills;

    SkillPtr pre_link(new Skill("pre_link",2));
    SkillPtr pre_node(new Skill("pre_node",2));
    SkillPtr approach_link(new Skill("approach_link",3));
    SkillPtr approach_node(new Skill("approach_node",3));
    SkillPtr grasp_link(new Skill("grasp_link",1));
    SkillPtr grasp_node(new Skill("grasp_node",1));
    SkillPtr align_link(new Skill("align_link",1));
    SkillPtr align_node(new Skill("align_node",1));
    SkillPtr place_link(new Skill("place_link",2));
    SkillPtr place_node(new Skill("place_node",3));

    /* SET UP THE SKILLS */
    pre_link->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false);//.setPrior(1e-200);
    pre_node->appendFeature("node").appendFeature("time").setInitializationFeature("node").setStatic(false);//.setPrior(1);//.setPrior(1.0 / 6.0);
    approach_link->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false);//.setPrior(3.0 / 6.0);
    approach_node->appendFeature("node").appendFeature("time").setInitializationFeature("node").setStatic(false);//.setPrior(1.0 / 6.0);
    grasp_link->appendFeature("link").setInitializationFeature("link").setStatic(true);
    grasp_node->appendFeature("node").setInitializationFeature("node").setStatic(true);
    align_link->appendFeature("link").appendFeature("time").setInitializationFeature("link");//.attachObject("link").setStatic(false);
    align_node->appendFeature("node").appendFeature("time").setInitializationFeature("node");//.attachObject("link").setStatic(false);
    place_link->appendFeature("node").appendFeature("time").setInitializationFeature("node");//.attachObject("link").setStatic(false);
    place_node->appendFeature("link").appendFeature("time").setInitializationFeature("link");//.attachObject("link").setStatic(false);


#if 0
    SkillPtr release_link(new Skill("release_link",1));
    SkillPtr release_node(new Skill("release_node",1));
    release_link->appendFeature("node").setInitializationFeature("node").setStatic(true);
    release_node->appendFeature("link").setInitializationFeature("link").setStatic(true);
#endif

    /* SET UP THE ROBOT KINEMATICS */
    RobotKinematicsPtr rk_ptr = RobotKinematicsPtr(new RobotKinematics("robot_description","base_link",ee));

    /* LOAD TRAINING DATA FOR PRE APPROACH LINK */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/lineup_link_x_0.bag",
        //"data/ur5_assembly_v5_wristUp/lineup_link_x_1.bag",
        //"data/ur5_assembly_v5_wristUp/lineup_link_x_2.bag",
        //"data/ur5_assembly_v5_wristUp/lineup_link_x_3.bag",
        "data/ur5_assembly_v6_wristUp/lineup_link_x_10.bag",

      };
      load_and_train_skill(*pre_link, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR PRE APPROACH NODE */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/lineup_node_x_0.bag",
        //"data/ur5_assembly_v5_wristUp/lineup_node_x_1.bag",
        //"data/ur5_assembly_v5_wristUp/lineup_node_x_2.bag",
        "data/ur5_assembly_v6_wristUp/lineup_node_z_10.bag",
      };
      load_and_train_skill(*pre_node, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR APPROACH */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/approach_link_x_0.bag",
        //"data/ur5_assembly_v5_wristUp/approach_link_x_1.bag",
        //"data/ur5_assembly_v5_wristUp/approach_link_x_2.bag",
        //"data/ur5_assembly_v5_wristUp/approach_link_x_3.bag",
        "data/ur5_assembly_v6_wristUp/approach_link_x_10.bag",

      };
      load_and_train_skill(*approach_link, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR APPROACH RIGHT */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/approach_node_x_0.bag",
        //"data/ur5_assembly_v5_wristUp/approach_node_x_1.bag",
        //"data/ur5_assembly_v5_wristUp/approach_node_x_2.bag",
        "data/ur5_assembly_v6_wristUp/approach_node_z_10.bag",
        //"data/ur5_assembly_v5_wristUp/approach_node_x_10.bag",
      };
      load_and_train_skill(*approach_node, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR GRASP */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/grasp_link_x_0.bag", 
        //"data/ur5_assembly_v5_wristUp/grasp_link_x_1.bag",
        //"data/ur5_assembly_v5_wristUp/grasp_link_x_2.bag",
        "data/ur5_assembly_v6_wristUp/grasp_link_x_10.bag",
      };
      load_and_train_skill(*grasp_link, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR GRASP */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/grasp_node_x_0.bag",
        //"data/ur5_assembly_v5_wristUp/grasp_node_x_1.bag",
        //"data/ur5_assembly_v5_wristUp/grasp_node_x_2.bag",
        "data/ur5_assembly_v6_wristUp/grasp_node_z_10.bag",
      };
      load_and_train_skill(*grasp_node, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR ALIGN */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/preassemble_link_x_0.bag", 
        //"data/ur5_assembly_v5_wristUp/preassemble_link_x_1.bag", 
        //"data/ur5_assembly_v5_wristUp/preassemble_link_x_2.bag", 
        //"data/ur5_assembly_v5_wristUp/preassemble_link_x_3.bag", 
        "data/ur5_assembly_v6_wristUp/preassemble_link_x_10.bag", 
      };
      load_and_train_skill(*align_link, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR ALIGN */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/preassemble_node_x_0.bag",
        //"data/ur5_assembly_v5_wristUp/preassemble_node_x_1.bag",
        //"data/ur5_assembly_v5_wristUp/preassemble_node_x_2.bag",
        "data/ur5_assembly_v6_wristUp/preassemble_node_z_10.bag",
      };
      load_and_train_skill(*align_node, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR PLACE */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/assemble_link_x_0.bag", 
        //"data/ur5_assembly_v5_wristUp/assemble_link_x_1.bag", 
        //"data/ur5_assembly_v5_wristUp/assemble_link_x_2.bag", 
        //"data/ur5_assembly_v5_wristUp/assemble_link_x_3.bag", 
        "data/ur5_assembly_v6_wristUp/assemble_link_x_10.bag", 
      };
      load_and_train_skill(*place_link, rk_ptr, filenames, 1);
    }
    /* LOAD TRAINING DATA FOR PLACE */
    {
      std::string filenames[] = {
        //"data/ur5_assembly_v5_wristUp/assemble_node_x_0.bag", 
        //"data/ur5_assembly_v5_wristUp/assemble_node_x_1.bag", 
        //"data/ur5_assembly_v5_wristUp/assemble_node_x_2.bag",
        "data/ur5_assembly_v6_wristUp/assemble_node_z_10.bag", 
      };
      load_and_train_skill(*place_node, rk_ptr, filenames, 1);
    }

#if 0
    /* LOAD TRAINING DATA FOR RELEASE */
    {
      std::string filenames[] = {
        "data/ur5_assembly_v5_wristUp/release_link_x_70.bag", 
        "data/ur5_assembly_v5_wristUp/release_link_x_71.bag", 
        "data/ur5_assembly_v5_wristUp/release_link_x_72.bag", 
      };
      load_and_train_skill(*release_link, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR RELEASE */
    {
      std::string filenames[] = {
        "data/ur5_assembly_v5_wristUp/release_node_z_70.bag", 
        "data/ur5_assembly_v5_wristUp/release_node_z_71.bag", 
        "data/ur5_assembly_v5_wristUp/release_node_z_72.bag", 
      };
      load_and_train_skill(*release_node, rk_ptr, filenames, 3);
    }
    skills["release_link"] = release_link;
    skills["release_node"] = release_node;
#endif

    skills["pre_approach_link"] = pre_link;
    skills["pre_approach_node"] = pre_node;
    skills["approach_link"] = approach_link;
    skills["approach_node"] = approach_node;
    skills["grasp_link"] = grasp_link;
    skills["grasp_node"] = grasp_node;
    skills["align_link"] = align_link;
    skills["align_node"] = align_node;
    skills["place_link"] = place_link;
    skills["place_node"] = place_node;

    return skills;

  }
}

#endif
