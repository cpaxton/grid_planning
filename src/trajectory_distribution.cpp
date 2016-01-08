#include <grid/trajectory_distribution.h>

/* KDL includes */
#include <kdl/trajectory_composite.hpp>
#include <kdl/trajectory_segment.hpp>
#include <kdl/velocityprofile_spline.hpp>
#include <kdl/velocityprofile_trap.hpp>
#include <kdl/rotational_interpolation_sa.hpp>
#include <kdl/path_line.hpp>
#include <kdl/path_roundedcomposite.hpp>

#define SHOW_SAMPLED_VALUES 0
#define DEFAULT_SIGMA 0.001

using namespace KDL;

namespace grid {

  /**
   * Initialize a trajectory distribution with velocity profile, etc.
   */
  TrajectoryDistribution::TrajectoryDistribution(int nseg_, int k_)
    : nseg(nseg_),
    dist(nseg_*(POSE_FEATURES_SIZE + SPLINE_DIM),k_),
    verbose(false)
  {
    dim = (POSE_FEATURES_SIZE + SPLINE_DIM) * nseg; // velocity, acceleration, position setpoints for each segment
  }

  /**
   * initialize
   * set up the distribution based on a skill and an environment
   */
  void TrajectoryDistribution::initialize(TestFeatures &features, const Skill &skill, std::vector<double> sigma) {
    Pose p0 = features.lookup(AGENT);
    Pose p1 = features.lookup(skill.getInitializationFeature()) * skill.getInitializationFinalPose();
    double eqradius = 1;

    RotationalInterpolation_SingleAxis *ri = new RotationalInterpolation_SingleAxis();
    Path_Line *path = new Path_Line(p0,p1,ri,eqradius);

    std::cout << "Path length: " << path->PathLength() << std::endl;
    for (int i = 0; i < nseg; ++i) {
      double s = ((double)(i+1)/(double)nseg) * path->PathLength();
      std::cout << "(" << i+1 << "/" << nseg << ") position = " << s << std::endl;
      Pose p = path->Pos(s);

      int idx = SPLINE_DIM * i;

      // set up x, y, z
      dist.ns[0].mu[idx+POSE_FEATURE_X] = p.p.x();
      dist.ns[0].mu[idx+POSE_FEATURE_Y] = p.p.y();
      dist.ns[0].mu[idx+POSE_FEATURE_Z] = p.p.z();

      // set up roll, pitch, yaw
      {
#ifdef USE_ROTATION_RPY
        double roll, pitch, yaw;
        p.M.GetRPY(roll,pitch,yaw);
        dist.ns[0].mu[idx+POSE_FEATURE_YAW] = yaw;
        dist.ns[0].mu[idx+POSE_FEATURE_PITCH] = pitch;
        dist.ns[0].mu[idx+POSE_FEATURE_ROLL] = roll;
#else
        double x,y,z,w;
        p.M.GetQuaternion(x,y,z,w);
        dist.ns[0].mu[idx+POSE_FEATURE_WX] = x;
        dist.ns[0].mu[idx+POSE_FEATURE_WY] = y;
        dist.ns[0].mu[idx+POSE_FEATURE_WZ] = z;
        dist.ns[0].mu[idx+POSE_FEATURE_WW] = w;
#endif

      }

      idx += POSE_FEATURES_SIZE;
      dist.ns[0].mu[idx+SPLINE_POS1] = 0.0;
      dist.ns[0].mu[idx+SPLINE_VEL1] = 0.01;
      dist.ns[0].mu[idx+SPLINE_ACC1] = 0.01;
      dist.ns[0].mu[idx+SPLINE_POS2] = 1;
      dist.ns[0].mu[idx+SPLINE_VEL2] = 0.01;
      dist.ns[0].mu[idx+SPLINE_ACC2] = -0.01;
      dist.ns[0].mu[idx+SEGMENT_DURATION] = 1.0;
    }

    if (sigma.size() < dim) {
      if (verbose) {
        std::cerr << "[GRID/TRAJECTORY DISTRIBUTION] Noise argument for trajectory search initialization was the wrong size!" << std::endl;
        std::cerr << "[GRID/TRAJECTORY DISTRIBUTION] Should be: " << dim << std::endl;
        std::cerr << "[GRID/TRAJECTORY DISTRIBUTION] Was: " << sigma.size() << std::endl;
      }
      for (int j = 0; j < dim; ++j) {
        dist.ns[0].P(j,j) = DEFAULT_SIGMA;
      }

    } else {
      for (int j = 0; j < dim; ++j) {
        dist.ns[0].P(j,j) = sigma[j];
      }
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
      Frame prev = Frame(initial);

      Trajectory_Composite *ctraj = new Trajectory_Composite();

      EigenVectornd vec;
      vec.resize(dim);
      dist.Sample(vec);

#if SHOW_SAMPLED_VALUES
      std::cout << "Sampled: ";
      for (int j = 0; j < dim; ++j) {
        std::cout << vec[j] << " ";
      }
      std::cout << std::endl;
#endif

      int idx = 0;
      for (int i = 0; i < nseg; ++i) {

        idx = (POSE_FEATURES_SIZE+SPLINE_DIM)*i;

        double prc1 = 0.1;
        double prc2 = 0.2;
        RotationalInterpolation_SingleAxis *ri = new RotationalInterpolation_SingleAxis();
        Path_RoundedComposite *path = new Path_RoundedComposite(prc1,prc2,ri);


        // generate a random set point and add it to the path
        {

#ifdef USE_ROTATION_RPY
          Rotation r1 = Rotation::RPY(vec[idx+POSE_FEATURE_ROLL],vec[idx+POSE_FEATURE_PITCH],vec[idx+POSE_FEATURE_YAW]);
#else
          double x,y,z,w;
          x = vec[idx+POSE_FEATURE_WX];
          y = vec[idx+POSE_FEATURE_WY];
          z = vec[idx+POSE_FEATURE_WZ];
          w = vec[idx+POSE_FEATURE_WW];
          double norm = 1/sqrt((x*x) + (y*y) + (z*z) + (w*w));
          Rotation r1 = Rotation::Quaternion(norm*vec[idx+POSE_FEATURE_WX],norm*vec[idx+POSE_FEATURE_WY],norm*vec[idx+POSE_FEATURE_WZ],norm*vec[idx+POSE_FEATURE_WW]);
#endif
          Vector v1 = Vector(vec[idx+POSE_FEATURE_X],vec[idx+POSE_FEATURE_Y],vec[idx+POSE_FEATURE_Z]);

          Frame t1 = Frame(r1,v1);
          path->Add(prev);
          path->Add(t1);
          path->Finish();
          prev = t1;
        }

        // generate random parameters for velocity profile
        //VelocityProfile *velprof = new VelocityProfile_Trap(0.5,0.1);
        VelocityProfile_Spline *velprof = new VelocityProfile_Spline();
        {
          idx += POSE_FEATURES_SIZE;

          double pos1,vel1,acc1,pos2,acc2,vel2,duration;
          pos1 = 0; //vec[idx+SPLINE_POS1];
          pos2 = path->PathLength(); //vec[idx+SPLINE_POS2]*path->PathLength();
          vel1 = vec[idx+SPLINE_VEL1];
          vel2 = vec[idx+SPLINE_VEL2];
          acc1 = vec[idx+SPLINE_ACC1];
          acc2 = vec[idx+SPLINE_ACC2];
          duration = fabs(vec[idx+SEGMENT_DURATION]);

          //std::cout << "Path length: " << path->PathLength() << std::endl;
          velprof->SetProfile(0,path->PathLength());
          velprof->SetProfileDuration(pos1,vel1,acc1,pos2,vel2,acc2,duration);
          //velprof->SetProfileDuration(pos1,vel1,pos2,vel2,duration);
          //velprof->SetProfileDuration(pos1,pos2,duration);
          //velprof->Write(std::cout);
        }

        // add to the trajectory
        Trajectory_Segment *seg = new Trajectory_Segment(path, velprof);
        ctraj->Add(seg);
      }

      traj[sample] = ctraj;
    }

    return traj;
  }

}
