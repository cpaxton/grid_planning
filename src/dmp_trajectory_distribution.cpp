#include <grid/dmp_trajectory_distribution.h>


#include <Eigen/Dense>
#include <trajectory_msgs/JointTrajectoryPoint.h>

using trajectory_msgs::JointTrajectoryPoint;
using namespace Eigen;

namespace grid {

  /**
   * Initialize a trajectory distribution with given params
   */
  DmpTrajectoryDistribution::DmpTrajectoryDistribution(unsigned int dim_, unsigned int nbasis_, RobotKinematicsPointer robot_)
    : dim(dim_),
    nbasis(nbasis_),
    robot(robot_),
    dist((dim*nbasis) + POSE_FEATURES_SIZE,1),
    verbose(false)
  {
    // set up anything else?
  }

  /**
   * initialize
   * set up the distribution based on a skill and an environment
   */
  void DmpTrajectoryDistribution::initialize(TestFeatures &features, const Skill &skill, std::vector<double> sigma) {

  }

  /**
   * sample
   * Pull a random trajectory from the gmm
   * Convert it into a KDL trajectory
   * NON-CONST becuse Gmm::sample is likewise non-const
   */
  void DmpTrajectoryDistribution::sample(std::vector<EigenVectornd> &params,std::vector<JointTrajectory> &trajs) {

  }



  /* ==================================================================== */
  /* ==================================================================== */
  /*             BELOW THIS: SAME AS TRAJECTORY DISTRIBUTION              */
  /* ==================================================================== */
  /* ==================================================================== */


  /**
   * update
   * take a set of trajectories and samples
   * use the trajectories to reweight the distribution
   */
  void DmpTrajectoryDistribution::update(
      std::vector<EigenVectornd> &params,
      std::vector<double> &ps,
      double diagonal_noise)
  {
    update(params,ps,diagonal_noise,def_step_size);
  }

  /**
   * update
   * take a set of trajectories and samples
   * use the trajectories to reweight the distribution
   */
  void DmpTrajectoryDistribution::update(
      std::vector<EigenVectornd> &params,
      std::vector<double> &ps)
  {
    update(params,ps,diagonal_sigma,def_step_size);
  }


  /**
   * update
   * take a set of trajectories and samples
   * use the trajectories to reweight the distribution
   */
  void DmpTrajectoryDistribution::update(
      std::vector<EigenVectornd> &params,
      std::vector<double> &ps,
      double diagonal_noise,
      double step_size)
  {

    double psum = 0;
    for (double &d: ps) {
      psum += d;
    }

    if (dist.k == 1) {

      // one cluster only
      // compute mean

      dist.ns[0].mu *= (1 - step_size); //setZero();
      dist.ns[0].P *= (1 - step_size); //setZero();

      for (unsigned int i = 0; i < params.size(); ++i) {
        //std::cout << "mu rows = " << dist.ns[0].mu.rows() << ", vec rows = " << vec.rows() << std::endl;
        //std::cout << "mu cols = " << dist.ns[0].mu.cols() << ", vec cols = " << vec.cols() << std::endl;
        double wt = step_size * ps[i] / psum;
        dist.ns[0].mu += params[i] * wt;
      }

      for (unsigned int i = 0; i < params.size(); ++i) {
        double wt = step_size * ps[i] / psum;
        //std::cout << wt << ", " << ps[i] << ", " << psum << std::endl;
        dist.ns[0].P += wt * (params[i] - dist.ns[0].mu) * (params[i] - dist.ns[0].mu).transpose();
      }


    } else {

      // set up weighted data
      // and then fit GMM again

      std::vector<std::pair<EigenVectornd,double> > data(ps.size());
      for (unsigned int i = 0; i < ps.size(); ++i) {
        data[0].first = params[i];
        data[0].second = ps[i] / psum;
      }

      dist.Fit(data);

    }

    for (unsigned int i = 0; i < dist.k; ++i) {
      dist.ns[0].P += diagonal_noise * Matrix<double,Dynamic,Dynamic>::Identity(dim,dim);
    }

    dist.Update();
  }
};
