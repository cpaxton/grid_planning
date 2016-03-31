#ifndef _GRID_SERVER_PARAMS
#define _GRID_SERVER_PARAMS

#include <ros/ros.h>
#include <string>

namespace grid {

  /** struct to store parameters for setting up planning programmatically
   *
   **/
  struct ServerParams {
    std::string ns;
    std::string base_link;
    std::string end_link;
    std::string robot_description_param;
    std::string move_group;
    double goal_noise;
    double trajectory_noise;
  };

  /**
   * read in server parameters from ros
   */
  void readServerParams(ServerParams &params, std::string ns = "/grid");

}

#endif
