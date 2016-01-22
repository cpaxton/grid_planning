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
    int starting_horizon;
    int verbosity;
    bool detect_collisions;
    std::string skill_name;
    std::string goal_name;
  };

  Params readRosParams();


}

#endif
