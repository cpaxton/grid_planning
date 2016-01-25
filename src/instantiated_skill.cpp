#include <grid/instantiated_skill.h>
//#include <grid/utils/params.hpp>

//#define LOW_PROBABILITY 0
//#define MAX_PROBABILITY 1
#define LOW_PROBABILITY -999999
#define MAX_PROBABILITY 0

using namespace grid_plan;
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
    params(p_.ntrajs), cur_iter(0), good_iter(0),
    next_skill(p_.ntrajs),
    prev_idx(p_.ntrajs),
    end_pts(p_.ntrajs),
    start_pts(p_.ntrajs),
    start_ps(p_.ntrajs),
    my_ps(p_.ntrajs),
    prev_p_sums(p_.ntrajs),
    prev_counts(p_.ntrajs),
    transitions_step(p_.step_size),
    acc(p_.ntrajs)
  {
    reset();
  }


  /**
   * set all variables back to original values
   * set all children to not done
   */
  void InstantiatedSkill::reset() {
    done = false;
    touched = false;
    model_norm = p.base_model_norm;
    best_p = LOW_PROBABILITY;
    cur_iter = 0;
    best_idx = 0;
    for (double &d: iter_lls) {
      d = 0;
    }
    for (InstantiatedSkillPointer ptr: next) {
      ptr->reset();
    }
  }


  /**
   * create a new skill with dmps
   */
  InstantiatedSkillPointer InstantiatedSkill::DmpInstance(SkillPointer skill,
                                                          TestFeaturesPointer features,
                                                          RobotKinematicsPointer robot,
                                                          unsigned int nbasis,
                                                          GridPlanner *checker)
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

    if(checker) {
      is->dmp_dist->setCollisionChecker(checker);
    }

    return is;
  }

  /**
   * create a new skill with dmps
   */
  InstantiatedSkillPointer InstantiatedSkill::DmpInstance(SkillPointer skill,
                                                          SkillPointer grasp,
                                                          TestFeaturesPointer features,
                                                          RobotKinematicsPointer robot,
                                                          unsigned int nbasis,
                                                          GridPlanner *checker)
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
    is->dmp_dist->attachObjectFromSkill(*grasp);
    is->dmp_dist->initialize(*features,*skill);

    if(checker) {
      is->dmp_dist->setCollisionChecker(checker);
    }

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
    std::cout << "new transitions: ";
    for (double &d: T) {
      std::cout << d << " ";
      tsum += d;
    }
    std::cout << "\n";

    if (tsum < 1e-200) {
      for (unsigned int i = 0; i < T.size(); ++i) {
        T[i] = last_T[i];
      }
      return;
    } else {
      for(unsigned int i = 0; i < T.size(); ++i) {
        T[i] = ((1 - p.step_size)*(last_T[i]/last_tsum))
          + (p.step_size * (T[i] / tsum));
        last_T[i] = T[i];
      }
    }

    if (p.verbosity > 0) {
      std::cout << "Transitions: ";
      for(unsigned int i = 0; i < T.size(); ++i) {
        std::cout << T[i] << " ";
      }
      std::cout << std::endl;
    }
  }


  // randomly sample an index from the probabilities
  unsigned int InstantiatedSkill::sampleIndex(unsigned int nsamples) const {
    // sample a random index from the skill

    assert(fabs(acc.at(nsamples-1) - 1) < 1e-5);

    double r = (double)rand() / RAND_MAX;
    //double r = unif_rand(re);
    for (unsigned int i = 0; i < nsamples; ++i) {
      if (r < acc.at(i)) return i;
    }
    return 0;
  }

  // initialize next probabilities
  void InstantiatedSkill::initializePs(std::vector<double> &ps_, double val) {
    for (unsigned int i = 0; i < ps_.size(); ++i) {
      ps_[i] = val;
    }
  }

  void InstantiatedSkill::accumulateProbs(const std::vector<double> &prev_ps, unsigned int len) {
    /************* ACCUMULATE PROBABILITIES *************/
    for (unsigned int i = 0; i < len; ++i) {
      if (i > 0) {
        acc[i] = exp(prev_ps[i]) + acc[i-1];
        //acc[i] = exp(prev_ps[i]) + acc[i-1];
      } else {
        acc[i] = exp(prev_ps[i]);
        //acc[i] = exp(prev_ps[i]);
      }

    }
    for (double &d: acc) {
      d /= acc[len-1];
      assert (not isnan(d));
    }

    if (p.verbosity > 3) {
      std::cout << "accumulating for " << id << ": ";
      for (double &d: acc) {
        std::cout << d << " ";
      }
      std::cout << "\n";
    }
  }

  void InstantiatedSkill::copyEndPoints(const std::vector<JointTrajectoryPoint> &prev_end_pts,
                                        const std::vector<double> &prev_ps,
                                        unsigned int len)
  {
    for (unsigned int i = 0; i < len; ++ i) {
      start_ps[i] = prev_ps.at(i);
      my_ps[i] = 0;
      end_pts[i].positions = prev_end_pts.at(i).positions;
      end_pts[i].velocities = prev_end_pts.at(i).velocities;
    }
  }

  /**
   * add some noise and refresh norm terms
   */
  void InstantiatedSkill::refresh(int horizon) {
    std::cout << "refreshing\n";
    //model_norm = p.base_model_norm;
    if (horizon > 0) {
      for (auto &child: next) {
        child->refresh(horizon-1);
      }
    }
  }

  void InstantiatedSkill::initializeCounts(std::vector<unsigned int> &ps, unsigned int val) {
    for (auto &u: ps) {
      u = val;
    }
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

    unsigned int next_len = nsamples;
    if (len == 0 || horizon < 0 || nsamples == 0) {
      std::cout << "SKIPPING\n";
      probability = 1e-200;
      return;
    } else if (horizon == 0 || next.size() == 0) {
      initializePs(next_ps,0);
    } else {
      initializePs(next_ps,LOW_PROBABILITY);
    }
    touched = true;
    initializePs(prev_p_sums,0);
    initializeCounts(prev_counts,0u);
    accumulateProbs(prev_ps,len);

    /************* SAMPLE TRAJECTORIES IF NECESSARY *************/
    if (skill) {

      // sample start points
      for (unsigned int i = 0; i < nsamples; ++i) {

        unsigned int idx = sampleIndex(len);
        assert (idx < len);

        // sample an index
        start_pts[i].positions = prev_end_pts[idx].positions;
        start_pts[i].velocities = prev_end_pts[idx].velocities;
        start_ps[i] = prev_ps[idx];
        prev_idx[i] = idx;
        ps_out[idx] = 0;
      }

      if (!skill->isStatic()) {
        // sample trajectories
        dmp_dist->sample(start_pts,params,trajs,nsamples);
      } else {
        // just stay put
        for (unsigned int i = 0; i < nsamples; ++i) {
          trajs[i].points.resize(1);
          trajs[i].points[0].positions = start_pts[i].positions;
          trajs[i].points[0].velocities = start_pts[i].velocities;
        }
      }

      // compute probabilities
      for (unsigned int j = 0; j < nsamples; ++j) {

        if (trajs[j].points.size() == 0) {
          my_ps[j] = LOW_PROBABILITY;
          continue;
        }

        // TODO: speed this up
        std::vector<Pose> poses = robot->FkPos(trajs[j]);

        if (skill) { // and not done) {
          skill->resetModel();
          skill->addModelNormalization(model_norm);

          // TODO: speed this up
          std::vector<FeatureVector> obs = features->getFeaturesForTrajectory(
              skill->getFeatures(),
              poses,
              dmp_dist->hasAttachedObject(),
              dmp_dist->getAttachedObjectFrame()
              );
          skill->normalizeData(obs);
          FeatureVector v = skill->logL(obs);
          my_ps[j] = log(v.array().exp().sum() / v.size()); // would add other terms first
        }
        if (p.verbosity > 1) {
          std::cout << "[" << id << "] " << j << ": " << skill->getName() << ": "<< my_ps[j]<<" + "<< start_ps[j]<<"\n";
        }

        // set up all the end points!
        end_pts[j].positions = trajs[j].points.rbegin()->positions;
        end_pts[j].velocities = trajs[j].points.rbegin()->velocities;
        }
      } else {
        copyEndPoints(prev_end_pts, prev_ps, len);
        next_len = len;
      }

      // check to make sure this is a valid path to explore
      double sum_so_far = 0;
      for (unsigned int j = 0; j < nsamples; ++j) {
        //std::cout << start_ps[j] << " " << my_ps[j] << "\n";
        ps[j] = my_ps[j] + start_ps[j];
        sum_so_far += exp(my_ps[j]);
      }

      // do we want to continue?
      // if so descend through the tree
      // descent through the tree
      if (horizon > 0 && log(sum_so_far) > LOW_PROBABILITY) {

        unsigned int next_skill_idx = 0;
        for (auto &ns: next) {
          unsigned int next_nsamples = floor((T[next_skill_idx]*nsamples) + 0.5);
          //std::cout << "samples: " << next_nsamples << ", " << T[next_skill_idx] << "\n";
          ns->step(my_ps, end_pts,
                   next_ps, T[next_skill_idx], // outputs
                   next_len, horizon-1, next_nsamples); // params

          if (p.verbosity > 0) {
            std::cout << " >>> probability of " << ns->skill->getName()
              << " " << ns->id << " = " << T[next_skill_idx]
              << std::endl;
          }

          ++next_skill_idx;
        }

        for (unsigned int i = 0; i < nsamples; ++i) {
          if (p.verbosity > 1) {
            if (skill) {
              std::cout << "[" << id << "] " << skill->getName()
                << ": "<<my_ps[i]<<" + "<< next_ps[i]<<"\n";
            } else {
              std::cout << "[" << id << "] [no skill]"
                << ": "<<my_ps[i]<<" + "<< next_ps[i] <<"\n";
            }
          }
          ps[i] = start_ps[i] + my_ps[i] + next_ps[i];
          if (ps[i] > best_p) {
            best_p = my_ps[i];
            best_idx = i;
          }
        }
      }

      updateTransitions();

      // update probabilities for all
      {
        // now loop over all the stuff!
        // go over probabilities and make sure they work
        // use the start_idx field to match start_idx to 
        double prev_psum_sum = 0;
        probability = 0;
        for (unsigned int i = 0; i < nsamples; ++i) {
          if (p.verbosity > 2) {
            std::cout << " - propogating p(" << i << ") = " << my_ps[i] + next_ps[i] << " back to " << prev_idx[i] << " ... " << log(probability) << "\n";
          }
          prev_p_sums[prev_idx[i]] += exp(my_ps[i]+next_ps[i]);
          ++prev_counts[prev_idx[i]];
          probability += exp(my_ps[i]+next_ps[i]);
        }
        //std::cout << "P =" << probability << " nsamples = " << nsamples <<"\n";
        probability /= nsamples;
        //std::cout << "P =" << probability <<"\n";
        //probability = log(probability);

        // update transitions based on these results
        for (unsigned int i = 0; i < len; ++i) {
          if (prev_counts[i] > 0) {
            ps_out[i] += log(prev_p_sums[i] / prev_counts[i]);
          } else {
            ps_out[i] += 0;
          }
          if (p.verbosity > 0) {
            std::cout << " - future sum for " << i << " = " << ps_out[i]
              << " (" << prev_counts[i] << " chosen, sum = " << log(prev_p_sums[i]) << ")"
              << "\n";
          }
        }
      }

      double sum = 0;
      // normalize everything
      {
        for (unsigned int i = 0; i < nsamples; ++i) {
          sum += exp(ps[i]);
        }
        // normalize here
        for (unsigned int i = 0; i < nsamples; ++i) {
          ps[i] = exp(ps[i]) / sum;
          //assert(not isnan(ps[i]));
        }
      }

      bool skip = false;
      if (log(sum) < LOW_PROBABILITY) {
        skip = true;
        //std::cout << "nothing here \n";
      }

      if(skill and not skill->isStatic() and not skip) {

        if (p.verbosity > 4) {
          std::cout << skill->getName() << " probabilities: ";
          for (double &p: ps) {
            std::cout << p << " ";
          }
          std::cout << std::endl;
        }

        if (nsamples > 1) {
          dmp_dist->update(params,ps,nsamples,p.noise,p.step_size);
          dmp_dist->addNoise(pow(0.01,cur_iter+2));
        }
      }

      // compute ll for this iteration
      iter_lls[cur_iter] = sum / p.ntrajs;

      if (cur_iter > 0 and fabs(iter_lls[cur_iter]-iter_lls[cur_iter]) < 1e-2*iter_lls[cur_iter]) {
        done = true;
      }

      // decrease normalization
      if (cur_iter > 0 && iter_lls[cur_iter] > iter_lls[cur_iter-1]) {
        model_norm *= p.model_norm_step;
        ++good_iter;
      }

      if (p.verbosity >= 0) {
        if (skill) {
          std::cout << "[" << id << "] " << skill->getName() << " >>>> AVG P = " << (sum / len) << std::endl;
        } else {
          std::cout << "[" << id << "] [no skill] >>>> AVG P = " << (sum / len) << std::endl;
        }
      }

      ++cur_iter;
    }

    /**
     * descend through the tree
     * execute as we reach nodes that require it
     * use gripper tool to send messages
     */
    void InstantiatedSkill::execute(actionlib::SimpleActionClient<grid_plan::CommandAction> &ac,
                                    int horizon)
    {

      // trigger action server
      CommandGoal cmd;
      if (skill) {
        cmd.name = skill->getName();
        cmd.traj = trajs[best_idx];
      }
      std::cout << "waiting for server...\n";
      ac.waitForServer();
      std::cout << "sending command...\n";
      ac.sendGoal(cmd);

      // continue execution
      if (horizon > 0 && next.size() > 0) {
        if (skill && skill->isStatic()) {
          // replan from here
        }

        ac.waitForResult();

        double best_t_p = 0;
        unsigned int idx = 0;
        for (unsigned int i = 0; i < T.size(); ++i) {
          if (T[i] > best_t_p) {
            idx = i;
            best_t_p = T[i];
          }
        }

        next[idx]->execute(ac,horizon-1);
      } else {


      }
    }

  }
