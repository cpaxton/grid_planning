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
    trajs(p_.ntrajs),
    next_ps(p_.ntrajs),
    params(p_.ntrajs), cur_iter(0),
    next_skill(p_.ntrajs),
    prev_idx(p_.ntrajs),
    end_pts(p_.ntrajs),
    start_pts(p_.ntrajs),
    start_ps(p_.ntrajs),
    prev_p_sums(p_.ntrajs),
    prev_counts(p_.ntrajs),
    transitions_step(p_.step_size),
    acc(p_.ntrajs)
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
    last_T.push_back(1);

    updateTransitions();
  }

  /**
   * normalize the transition probabilities
   */
  void InstantiatedSkill::updateTransitions() {

    double last_tsum = 0;
    for (double &d: last_T) {
      last_tsum += d;
    }
    double tsum = 0;
    for (double &d: T) {
      tsum += d;
    }
    std::cout << "Transitions: ";
    for(unsigned int i = 0; i < T.size(); ++i) {
      T[i] = ((1 - p.step_size)*(last_T[i]/last_tsum))
        + (p.step_size * (T[i] / tsum));
      std::cout << T[i] << " ";
      last_T[i] = T[i];
    }
    std::cout << std::endl;
  }


  // randomly sample an index from the probabilities
  unsigned int InstantiatedSkill::sampleIndex(unsigned int nsamples) const {
    // sample a random index from the skill

    assert(fabs(acc.at(nsamples-1) - 1) < 1e-5);

    double r = (double)rand() / RAND_MAX;
    for (unsigned int i = 0; i < nsamples; ++i) {
      if (r < acc.at(i)) return i;
    }
    return 0;
  }

  /**
   * run a single iteration of the loop. return a set of trajectories.
   * this is very similar to code in the demo
   */
  void InstantiatedSkill::step(const std::vector<double> &prev_ps,
                               const std::vector<JointTrajectoryPoint> &prev_end_pts,
                               std::vector<double> &ps_out,
                               double &probability,
                               unsigned int len,
                               int horizon,
                               unsigned int nsamples)
  {


    probability = 0;

    unsigned int next_len = nsamples;

    //std::cout << "in: " << len << std::endl;
    //for (unsigned int i = 0; i < len; ++i) {
    //  std::cout << prev_end_pts[i].positions.size() << "\n";
    //}

    /************* ACCUMULATE PROBABILITIES *************/
    for (unsigned int i = 0; i < nsamples; ++i) {
      next_ps[i] = 0;
      if (i > 0) {
        acc[i] = prev_ps[i] + acc[i-1];
      } else {
        acc[i] = prev_ps[i];
      }

    }
    for (double &d: acc) {
      d /= acc[nsamples-1];
    }

    if (p.verbosity > 3) {
      std::cout << "accumulating for " << id << ": ";
      for (double &d: acc) {
        std::cout << d << " ";
      }
      std::cout << "\n";
    }

    touched = true;

    /************* SAMPLE TRAJECTORIES IF NECESSARY *************/
    if (horizon < 0) {
      return;
    } else if (nsamples == 0) { 
      nsamples = p.ntrajs;
    }

    double sum = 0;

    if (skill) { // or goal?

      // sample start points
      for (unsigned int i = 0; i < nsamples; ++i) {

        unsigned int idx = sampleIndex(nsamples);

        // sample an index
        start_pts[i].positions = prev_end_pts[idx].positions;
        start_pts[i].velocities = prev_end_pts[idx].velocities;
        start_ps[i] = prev_ps[idx];
        prev_idx[i] = idx;
        prev_p_sums[idx] = 0;

        // initialize
        next_ps[i] = 0;
      }

      // sample trajectories
      dmp_dist->sample(start_pts,params,trajs,nsamples);

      // compute probabilities
      for (unsigned int j = 0; j < nsamples; ++j) {


        if (trajs[j].points.size() == 0) {
          ps[j] = 0;
          continue;
        }

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
          ps[j] = 1;
        }
        if (p.verbosity > 1) {
          std::cout << "[" << id << "] " << skill->getName() << ": "<<ps[j]<<" * "<<start_ps[j]<<"\n";
        }
        ps[j] *= start_ps[j];

        sum += ps[j];

        if (ps[j] > best_p) {
          best_p = ps[j];
          best_idx = j;
        }

        // set up all the end points!
        end_pts[j].positions = trajs[j].points.rbegin()->positions;
        end_pts[j].velocities = trajs[j].points.rbegin()->velocities;
      }
    } else {
      sum = 0;
      for (unsigned int i = 0; i < len; ++ i) {
        ps[i] = prev_ps.at(i);
        sum += ps[i];
        end_pts[i].positions = prev_end_pts.at(i).positions;
        end_pts[i].velocities = prev_end_pts.at(i).velocities;
      }
      next_len = len;
    }

    // do we want to continue?
    // if so descend through the tree
    // descent through the tree
    if (horizon > 0) {

      unsigned int next_idx = 0;
      unsigned int next_skill_idx = 0;
      for (auto &ns: next) {
        unsigned int next_nsamples = ceil(T[next_skill_idx]*nsamples);
        ns->step(ps, end_pts,
                 next_ps, T[next_idx], // outputs
                 next_len, horizon-1, next_nsamples); // params
        next_idx += next_nsamples;

        if (p.verbosity > 0) {
          std::cout << " >>> probability of " << ns->skill->getName()
            << " " << ns->id << " = " << T[next_idx]
            << std::endl;
        }

        ++next_skill_idx;
      }

      for (unsigned int i = 0; i < nsamples; ++i) {
        if (p.verbosity > 1) {
          if (skill) {
            std::cout << "[" << id << "] " << skill->getName()
              << ": "<<ps[i]<<" * "<<next_ps[i]<<"\n";
          } else {
            std::cout << "[" << id << "] [no skill]"
              << ": "<<ps[i]<<" * "<<next_ps[i]<<"\n";
          }
        }
        ps[i] *= next_ps[i];
      }
    }

    updateTransitions();

    // update probabilities for all
    {
      // now loop over all the stuff!
      // go over probabilities and make sure they work
      // use the start_idx field to match start_idx to 
      double prev_psum_sum = 0;
      for (unsigned int i = 0; i < nsamples; ++i) {
        if (p.verbosity > 2) {
          std::cout << " - propogating p(" << i << ") = " << ps[i] << " back to " << prev_idx[i] << " ... " << probability << "\n";
        }
        prev_p_sums[prev_idx[i]] += ps[i];
        ++prev_counts[prev_idx[i]];
        probability += ps[i];
      }
      probability /= nsamples;

      // update transitions based on these results
      for (unsigned int i = 0; i < len; ++i) {
        if (prev_counts[i] > 0) {
          ps_out[i] += prev_p_sums[i] / prev_counts[i];
        } else {
          ps_out[i] += 0;
        }
        if (p.verbosity > 0) {
          std::cout << " - future sum for " << i << " = " << prev_ps[i]
            << "(" << prev_counts[i] << " chosen, sum = " << prev_p_sums[i] << ")"
            << "\n";
        }
      }
    }

    // normalize here
    for (double &d: ps) {
      d /= sum;
    }
    if(skill) {
      dmp_dist->update(params,ps,nsamples,p.noise,p.step_size);
    }

    // compute ll for this iteration
    iter_lls[cur_iter] = sum / p.ntrajs;

    // decrease normalization
    if (cur_iter > 0 && iter_lls[cur_iter] > iter_lls[cur_iter-1]) {
      model_norm *= p.model_norm_step;
    }

    if (p.verbosity >= 0) {
      if (skill) {
        std::cout << "[" << id << "] " << skill->getName() << " >>>> AVG P = " << (sum / nsamples) << std::endl;
      } else {
        std::cout << "[" << id << "] [no skill] >>>> AVG P = " << (sum / len) << std::endl;
      }
    }

    ++cur_iter;
  }
}
