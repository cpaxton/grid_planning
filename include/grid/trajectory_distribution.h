#ifndef _GRID_TRAJECTORY_DISTRIBUTION
#define _GRID_TRAJECTORY_DISTRIBUTION

#include <grid/features.h>
#include <grid/test_features.h>
#include <grid/skill.h>
#include <grid/dist/gmm.h>

namespace grid {

  class TrajectoryDistribution {
    protected:
      gcop::Gmm<> dist; // stores distributions
      unsigned int nseg; // number of segments
      unsigned int dim; // dimensionality of the trajectory space

    public:

      /**
       * Initialize a trajectory distribution with velocity profile, etc.
       */
      TrajectoryDistribution(int nseg, int k = 1);

      /**
       * initialize
       * set up the distribution based on a skill and an environment
       */
      void initialize(const Features &features, const Skill &skill);

      /**
       * sample
       * Pull a random trajectory from the gmm
       * Convert it into a KDL trajectory
       */
      Trajectory *sample() const;
  };

}

#endif