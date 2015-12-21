#include <grid/trajectory_distribution.h>

/* KDL includes */
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_roundedcomposite.hpp>

using namespace KDL;

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
  void TrajectoryDistribution::initialize(TestFeatures &features, const Skill &skill) {
    Pose p0 = features.lookup(AGENT);
    Pose p1 = features.lookup(skill.getBestFeature());
    double eqradius = 1;

    RotationalInterpolation_SingleAxis *ri = new RotationalInterpolation_SingleAxis();
    Path_Line *path = new Path_Line(p0,p1,ri,eqradius);

    std::cout << "Path length: " << path->PathLength();
    for (int i = 0; i < nseg; ++i) {
      Pose p = path->Pos(((double)(i+1)/(double)nseg) * path->PathLength());
    }

    delete path;
  }

  /**
   * sample
   * Pull a random trajectory from the gmm
   * Convert it into a KDL trajectory
   */
  Trajectory *TrajectoryDistribution::sample(unsigned int nsamples) const {
    Trajectory *traj = new Trajectory_Composite[nsamples];

    double prc1 = 0.1;
    double prc2 = 0.2;

    for (int sample = 0; sample < nsamples; ++sample) {
      for (int i = 0; i < nseg; ++i) {

        RotationalInterpolation_SingleAxis *ri = new RotationalInterpolation_SingleAxis();
        Path_RoundedComposite *path = new Path_RoundedComposite(prc1,prc2,ri);

        // generate a random set point

        // generate random parameters

        // add to the trajectory
      }
    }

    return traj;
  }

}
