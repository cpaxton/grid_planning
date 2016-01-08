#ifndef _GRID_TRAJ
#define _GRID_TRAJ

#include <grid/trajectory.h>

// primitives for motion planning
#include <dmp/dmp.h>

namespace grid {

  class DmpTrajectory {
  public:

  protected:
    unsigned int dof;
    unsigned int num_basis;
    double k_gain;
    double d_gain;
    double tau;
    double threshold;

  };

}

#endif
