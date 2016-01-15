#ifndef _GRID_PARAMS
#define _GRID_PARAMS

#include <ros/ros.h>

namespace grid {


  struct Params {
    double base_model_norm;
    double model_norm_step;
    double base_sampling_noise;
    double sampling_noise_step;
    double step_size;
    double noise;
    double wait;
    int ntrajs = 50;
    int iter = 10;
    std::string skill_name;
    std::string goal_name;
  };

  Params readRosParams() {

    Params p;
    ros::NodeHandle nh_tilde("~");
    if (not nh_tilde.getParam("step_size",p.step_size)) {
      p.step_size = 0.80;
    }
    if (not nh_tilde.getParam("noise",p.noise)) {
      p.noise = 1e-10;
    }
    if (not nh_tilde.getParam("ntrajs",p.ntrajs)) {
      p.ntrajs = 50;
    }
    if (not nh_tilde.getParam("iter",p.iter)) {
      p.iter = 10;
    }
    if (not nh_tilde.getParam("skill",p.skill_name)) {
      p.skill_name = "approach";
    }
    if (not nh_tilde.getParam("goal",p.goal_name)) {
      p.goal_name = "grasp";
    }
    if (not nh_tilde.getParam("base_model_norm",p.base_model_norm)) {
      p.base_model_norm = 0.01;
    }
    if (not nh_tilde.getParam("model_norm_step",p.model_norm_step)) {
      p.model_norm_step = 0.1;
    }
    if (not nh_tilde.getParam("base_sampling_noise",p.base_sampling_noise)) {
      p.base_sampling_noise = 0.01;
    }
    if (not nh_tilde.getParam("sampling_noise_step",p.sampling_noise_step)) {
      p.sampling_noise_step = 0.1;
    }
    if (not nh_tilde.getParam("wait",p.wait)) {
      p.wait = 0.25;
    }

  return p;

  }


}

#endif
