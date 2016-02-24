#ifndef _GRID_LOAD_WAM_SKILLS_AUTO
#define _GRID_LOAD_WAM_SKILLS_AUTO

#include <grid/skill.h>

namespace grid {

  std::unordered_map<std::string, SkillPtr> loadWamSkillsAuto() {

    SkillPtr approach(new Skill("approach",2));
    SkillPtr approach_right(new Skill("approach_right",2));
    SkillPtr approach_left(new Skill("approach_left",2));
    SkillPtr grasp(new Skill("grasp",1));
    SkillPtr align(new Skill("align",2));
    SkillPtr place(new Skill("place",2));
    SkillPtr release(new Skill("release",1));
    SkillPtr disengage(new Skill("disengage",2));

    /* SET UP THE SKILLS */
    approach->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false).setPrior(3.0 / 6.0);
    approach_left->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false).setPrior(1.0 / 6.0);
    approach_right->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false).setPrior(2.0 / 6.0);
    grasp->appendFeature("link").setInitializationFeature("link").setStatic(true);
    align->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(false);
    place->appendFeature("node").appendFeature("time").setInitializationFeature("node").attachObject("link").setStatic(false);
    release->appendFeature("node").setInitializationFeature("node").attachObject("link").setStatic(true);
    disengage->appendFeature("link").appendFeature("time").setInitializationFeature("link").setStatic(false);


    /* SET UP THE ROBOT KINEMATICS */
    RobotKinematicsPtr rk_ptr = RobotKinematicsPtr(new RobotKinematics("robot_description","wam/base_link","wam/wrist_palm_link"));

    /* LOAD TRAINING DATA FOR APPROACH */
    {
      int downsampling[] = {10,10,10,0,0,0};
      std::string filenames[] = {"data/sim/approach01.bag",
        "data/sim/approach02.bag",
        "data/sim/approach03.bag",
        "data/sim_auto/approach1.bag",
        "data/sim_auto/approach2.bag",
        "data/sim_auto/approach3.bag"
      };
      load_and_train_skill(*approach, rk_ptr, filenames, 6, &downsampling[0]);
    }
    /* LOAD TRAINING DATA FOR APPROACH RIGHT */
    {
      std::string filenames[] = {"data/sim/approach_right01.bag",
        "data/sim/approach_right02.bag",
        "data/sim/approach_right03.bag"};
      load_and_train_skill(*approach_right, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR APPROACH LEFT */
    {
      std::string filenames[] = {"data/sim/approach_left01.bag", "data/sim/approach_left02.bag", "data/sim/approach_left03.bag"};
      load_and_train_skill(*approach_left, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR GRASP */
    {
      std::string filenames[] = {"data/sim/grasp1.bag", "data/sim/grasp2.bag", "data/sim/grasp3.bag"};
      load_and_train_skill(*grasp, rk_ptr, filenames, 3);
    }
    /* LOAD TRAINING DATA FOR ALIGN */
    {
      int downsampling[] = {10,10,10,0,0,0};
      std::string filenames[] = {"data/sim/align1.bag",
        "data/sim/align2.bag",
        "data/sim/align3.bag",
        "data/sim_auto/align1.bag",
        "data/sim_auto/align2.bag",
        "data/sim_auto/align3.bag"
      };
      load_and_train_skill(*align, rk_ptr, filenames, 6, &downsampling[0]);
    }
    /* LOAD TRAINING DATA FOR PLACE */
    {
      int downsampling[] = {10,10,0,0,0};
      std::string filenames[] = {"data/sim/place1.bag",
        "data/sim/place3.bag",
        "data/sim_auto/place3.bag",
        "data/sim_auto/place4.bag",
        "data/sim_auto/place5.bag"
      };
      load_and_train_skill(*place, rk_ptr, filenames, 5, &downsampling[0]);
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

    std::unordered_map<std::string, SkillPtr> skills;
    skills["approach"] = approach;
    skills["approach_right"] = approach_right;
    skills["approach_left"] = approach_left;
    skills["grasp"] = grasp;
    skills["align"] = align;
    skills["place"] = place;
    skills["release"] = release;
    skills["disengage"] = disengage;

    return skills;

  }
}

#endif
