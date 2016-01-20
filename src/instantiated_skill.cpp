#include <grid/instantiated_skill.h>
//#include <grid/utils/params.hpp>


using trajectory_msgs::JointTrajectory;
using trajectory_msgs::JointTrajectoryPoint;

namespace grid {

  unsigned int InstantiatedSkill::next_id(0);

  /** 
   * default constructor
   */
  InstantiatedSkill::InstantiatedSkill()
    : id(next_id++), done(false), current(false),
    touched(false), spline_dist(0), dmp_dist(0), skill(0),
    trajs(), effects(), cur_iter(0)
  {
  }

  /**
   * set up with parameters
   */
  InstantiatedSkill::InstantiatedSkill(Params &p_) :
    p(p_),
    id(next_id++), done(false), touched(false), spline_dist(0), dmp_dist(0), skill(0),
    effects(),
    ps(p_.ntrajs), iter_lls(p_.iter),
    current(false),
    trajs(p_.ntrajs), next_ps(p_.ntrajs),
    params(p_.ntrajs), cur_iter(0)
  {
    reset();
  }


  /**
   * set all variables back to original values
   */
  void InstantiatedSkill::reset() {
    done = false;
    touched = false;
    model_norm = p.base_model_norm;
    best_p = 0;
    cur_iter = 0;
    best_idx = 0;
    for (double &d: iter_lls) {
      d = 0;
    }
  }

  /**
   * set all child skills to not done
   * assumes children of an unfinished node are not finished
   */
  InstantiatedSkill &InstantiatedSkill::refresh() {
    if (touched) {
      for (InstantiatedSkillPointer ptr: next) {
        ptr->refresh();
      }
      reset();
    }
    return *this;
  }


  /**
   * create a new skill with dmps
   */
  InstantiatedSkillPointer InstantiatedSkill::DmpInstance(SkillPointer skill,
                                                          TestFeaturesPointer features,
                                                          RobotKinematicsPointer robot,
                                                          unsigned int nbasis)
  {

    Params p = readRosParams();
    InstantiatedSkillPointer is(new InstantiatedSkill(p));
    is->skill = skill;
    is->features = features;
    is->robot = robot;
    is->dmp_dist = DmpTrajectoryDistributionPointer(
        new DmpTrajectoryDistribution(robot->getDegreesOfFreedom(),
                                      nbasis,
                                      robot));
    is->dmp_dist->initialize(*features,*skill);

    return is;
  }

  /**
   * create a new skill with spline and segments
   */
  InstantiatedSkillPointer InstantiatedSkill::SplineInstance(SkillPointer skill,
                                                             TestFeaturesPointer features,
                                                             RobotKinematicsPointer robot,
                                                             unsigned int nseg)
  {

    InstantiatedSkillPointer is(new InstantiatedSkill());
    is->skill = skill;
    is->features = features;
    is->robot = robot;
    is->spline_dist = TrajectoryDistributionPointer(new TrajectoryDistribution(nseg));
    is->spline_dist->initialize(*features,*skill);

    return is;
  }

  /**
   * create an empty root node
   */
  InstantiatedSkillPointer InstantiatedSkill::Root() {
    Params p = readRosParams();
    return InstantiatedSkillPointer (new InstantiatedSkill(p));
  }

  /**
   * define a possible child
   */
  InstantiatedSkill &InstantiatedSkill::addNext(InstantiatedSkillPointer skill) {
    next.push_back(skill);
    T.push_back(1);

    updateTransitions();
  }

  /**
   * normalize the transition probabilities
   */
  void InstantiatedSkill::updateTransitions() {

    double tsum = 0;
    for (double &d: T) {
      tsum += d;
    }
    for (double &d: T) {
      d /= tsum;
    }
  }

  /**
   * run a single iteration of the loop. return a set of trajectories.
   * this is very similar to code in the demo
   */
  void InstantiatedSkill::step(std::vector<double> &ps,
                               std::vector<JointTrajectoryPoint> &start_pts,
                               int horizon,
                               unsigned int nsamples,
                               unsigned int start_idx)
  {

    touched = true;

    if (horizon < 0) {
      return;
    } else if (nsamples == 0) { 
      nsamples = p.ntrajs;
    }


    double sum = 0;

    if (skill) {

      // sample trajectories
      dmp_dist->sample(params,trajs);

      // compute probabilities
      for (unsigned int j = 0; j < trajs.size(); ++j) {

        // TODO: speed this up
        std::vector<Pose> poses = robot->FkPos(trajs[j]);

        if (skill) {
          skill->resetModel();
          skill->addModelNormalization(model_norm);

          // TODO: speed this up
          std::vector<FeatureVector> obs = features->getFeaturesForTrajectory(skill->getFeatures(),poses);
          skill->normalizeData(obs);
          FeatureVector v = skill->logL(obs);
          ps[j] = (v.array().exp().sum() / v.size()); // would add other terms first
        } else {
          ps[j] = 0;
        }

        sum += ps[j];

        if (ps[j] > best_p) {
          best_p = ps[j];
          best_idx = j;
        }

      }
    }

    // update end points

    if (horizon > 0) {
      unsigned int next_idx = 0;
      unsigned int next_skill_idx = 0;
      for (auto &ns: next) {
        unsigned int next_nsamples = ceil(T[next_skill_idx]*nsamples);
        ns->step(next_ps, end_pts, horizon-1, next_nsamples, next_idx);
        next_idx += next_nsamples;
        ++next_skill_idx;
      }
    }

    if(skill) {
      dmp_dist->update(params,ps,p.noise,p.step_size);
    }

    // compute ll for this iteration
    iter_lls[cur_iter] = sum / p.ntrajs;

    // decrease normalization
    if (cur_iter > 0 && iter_lls[cur_iter] > iter_lls[cur_iter-1]) {
      model_norm *= p.model_norm_step;
    }

    if (skill) {
      std::cout << "[" << id << "] " << skill->getName() << " >>>> AVG P = " << (sum / p.ntrajs) << std::endl;
    } else {
      std::cout << "[" << id << "] [no skill] >>>> AVG P = " << (sum / p.ntrajs) << std::endl;
    }

    ++cur_iter;

    updateTransitions();
  }
}
