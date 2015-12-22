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
  TrajectoryDistribution::TrajectoryDistribution(int nseg_, int k_) : nseg(nseg_), dist(nseg_*6,k_) {
    dim = 6 * nseg; // velocity, acceleration, position setpoints for each segment
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

    std::cout << "Path length: " << path->PathLength() << std::endl;
    for (int i = 0; i < nseg; ++i) {
      double s = ((double)(i+1)/(double)nseg) * path->PathLength();
      std::cout << "(" << i+1 << "/" << nseg << ") position = " << s << std::endl;
      Pose p = path->Pos(s);

      int idx = 6 * i;

      // set up x, y, z
      dist.ns[0].mu[idx+POSE_FEATURE_X] = p.p.x();
      dist.ns[0].mu[idx+POSE_FEATURE_Y] = p.p.y();
      dist.ns[0].mu[idx+POSE_FEATURE_Z] = p.p.z();

      // set up roll, pitch, yaw
      {
        double roll, pitch, yaw;
        p.M.GetRPY(roll,pitch,yaw);
        dist.ns[0].mu[idx+POSE_FEATURE_YAW] = yaw;
        dist.ns[0].mu[idx+POSE_FEATURE_PITCH] = pitch;
        dist.ns[0].mu[idx+POSE_FEATURE_ROLL] = roll;
      }
    }

    for (int j = 0; j < dim; ++j) {
      dist.ns[0].P(j,j) = 1;
    }
    dist.Update();

    initial = p0;
    delete path;
  }

  /**
   * sample
   * Pull a random trajectory from the gmm
   * Convert it into a KDL trajectory
   */
  std::vector<Trajectory *> TrajectoryDistribution::sample(unsigned int nsamples) {
    //Trajectory_Composite *traj = new Trajectory_Composite[nsamples];
    std::vector<Trajectory *> traj = std::vector<Trajectory *>(nsamples);

    for (int sample = 0; sample < nsamples; ++sample) {
      const Frame *prev = &initial;

      Trajectory_Composite *ctraj = new Trajectory_Composite();

      EigenVectornd vec;
      vec.resize(dim);
      dist.Sample(vec);

      std::cout << "Sampled: ";
      for (int j = 0; j < dim; ++j) {
        std::cout << vec[j] << " ";
      }
      std::cout << std::endl;

      for (int i = 0; i < nseg; ++i) {

        double prc1 = 0.1;
        double prc2 = 0.2;
        RotationalInterpolation_SingleAxis *ri = new RotationalInterpolation_SingleAxis();
        Path_RoundedComposite *path = new Path_RoundedComposite(prc1,prc2,ri);

        // generate a random set point and add it to the path
        {

          int idx = 6*i;
          Rotation r1 = Rotation::RPY(vec[idx+POSE_FEATURE_ROLL],vec[idx+POSE_FEATURE_PITCH],vec[idx+POSE_FEATURE_YAW]);
          Vector v1 = Vector(vec[idx+POSE_FEATURE_X],vec[idx+POSE_FEATURE_Y],vec[idx+POSE_FEATURE_Z]);

          Frame t1 = Frame(r1,v1);
          path->Add(Frame(*prev));
          path->Add(t1);
          path->Finish();
        }

        // generate random parameters for velocity profile
        //VelocityProfile_Spline *velprof = new VelocityProfile_Spline();
        VelocityProfile *velprof = new VelocityProfile_Trap(0.5,0.1);
        //std::cout << "Path length: " << path->PathLength() << std::endl;
        velprof->SetProfile(0,path->PathLength());
        //velprof->SetProfileDuration(0.0, 0.5, 3.0);

        // add to the trajectory
        Trajectory_Segment *seg = new Trajectory_Segment(path, velprof);
        ctraj->Add(seg);
      }

      traj[sample] = ctraj;
    }

    std::cout << "sizeof = " << sizeof(traj) << std::endl;
    for (int i =0; i < nsamples; ++i) {
      std::cout << i << ": " << traj[i]->Duration() << std::endl;
    }

    return traj;
  }

}
