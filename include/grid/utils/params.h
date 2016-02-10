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
    int ntrajs;
    int iter;
    int starting_horizon;
    int max_horizon;
    int verbosity;
    bool detect_collisions;
    std::string skill_name;
    std::string goal_name;
    double update_horizon;
    bool collisions_verbose;
  };

  Params readRosParams();


}

#endif
