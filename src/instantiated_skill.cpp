#include <grid/instantiated_skill.h>
//#include <grid/utils/params.hpp>

namespace grid {

  unsigned int InstantiatedSkill::next_id(0);

  /** 
   * default constructor
   */
  InstantiatedSkill::InstantiatedSkill()
    : id(next_id++), done(false), touched(false), spline_dist(0), dmp_dist(0), skill(0), trajs(), effects(), cur_iter(0)
  {
  }

  /**
   * set up with parameters
   */
  InstantiatedSkill::InstantiatedSkill(Params &p_) :
    p(p_),
    id(next_id++), done(false), touched(false), spline_dist(0), dmp_dist(0), skill(0),
    effects(), iter_lls(p_.iter), trajs(p_.ntrajs), params(p_.ntrajs), cur_iter(0)
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

    InstantiatedSkillPointer is(new InstantiatedSkill());
    is->skill = skill;
    is->features = features;
    is->robot = robot;
    is->dmp_dist = DmpTrajectoryDistributionPointer(
        new DmpTrajectoryDistribution(robot->getDegreesOfFreedom(),
                                      nbasis,
                                      robot));

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

    return is;
  }

  /**
   * create an empty root node
   */
  InstantiatedSkillPointer InstantiatedSkill::Root() {
    return InstantiatedSkillPointer(new InstantiatedSkill());
  }

  /**
   * define a possible child
   */
  InstantiatedSkill &InstantiatedSkill::addNext(InstantiatedSkillPointer skill) {
    next.push_back(skill);
    T.push_back(1);
  }


  /**
   * run a single iteration of the loop. return a set of trajectories.
   * this is very similar to code in the demo
   */
  void InstantiatedSkill::step() {

    touched = true;

    // sample trajectories
    dmp_dist->sample(params,trajs);

    double sum = 0;

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

    //if (sum > 1e-50) { 
    // update distribution
    dmp_dist->update(params,ps,p.noise,p.step_size);
    //} else {
    //i--;
    //   continue;
    //}

    iter_lls[cur_iter] = sum / p.ntrajs;


    if (cur_iter > 0 && iter_lls[cur_iter] > iter_lls[cur_iter-1]) {
      //std::cout<<"decreasing normalization\n";
      model_norm *= p.model_norm_step;
    }

    ++cur_iter;
  }

}
