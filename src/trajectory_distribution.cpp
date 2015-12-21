#include <grid/trajectory_distribution.h>

namespace grid {

  /**
   * Initialize a trajectory distribution with velocity profile, etc.
   */
  TrajectoryDistribution::TrajectoryDistribution(int nseg_, int k_) : nseg(nseg_), dist(nseg_*13) {
    dim = 13 * nseg; // velocity, acceleration, position setpoints for each segment
  }

  /**
   * initialize
   * set up the distribution based on a skill and an environment
   */
  void TrajectoryDistribution::initialize(const Features &features, const Skill &skill) {

  }

  /**
   * sample
   * Pull a random trajectory from the gmm
   * Convert it into a KDL trajectory
   */
  Trajectory *TrajectoryDistribution::sample() const {
    Trajectory *traj;

    return traj;
  }

}