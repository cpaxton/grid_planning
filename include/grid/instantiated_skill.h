#ifndef _GRID_INSTANTIATED_SKILL
#define _GRID_INSTANTIATED_SKILL

#include <memory>

#include <grid/test_features.h>
#include <grid/dmp_trajectory_distribution.h>
#include <grid/trajectory_distribution.h>
#include <grid/robot_kinematics.h>
#include <grid/skill.h>
#include <grid/utils/params.h>

#include <random>

namespace grid {

  typedef std::shared_ptr<TestFeatures> TestFeaturesPointer;

  /**
   * creating predicates
   * What is going to change after we do this
   */
  struct PredicateEffect {
    std::string predicate;
    bool value;
    SkillPointer skill; // this is where we actually need to learn the effects model
  };

  class InstantiatedSkill;
  typedef std::shared_ptr<InstantiatedSkill> InstantiatedSkillPointer;

  /**
   * Defines a particular instance of a skill
   */
  class InstantiatedSkill {

  protected:

    static unsigned int next_id;

    const unsigned int id; // unique id for this skill
    bool done; // set to true if we don't need to keep evaluating this
    bool touched; // has anyone done anything with this skill yet

    std::unordered_map<std::string,std::string> assignment;
    SkillPointer skill; // the skill itself
    TrajectoryDistributionPointer spline_dist; // the path we end up taking for this skill
    DmpTrajectoryDistributionPointer dmp_dist; // the path we end up taking for this skill

    std::vector<double> T; // probability of going to each of the possible next actions
    std::vector<double> last_T; // probability of going to each of the possible next actions
    std::vector<InstantiatedSkillPointer> next;

    // selected endpoints for this trajectory
    std::vector<JointTrajectoryPoint> end_pts;

    // store start points for this trajectory
    std::vector<JointTrajectoryPoint> start_pts;
    std::vector<double> start_ps;
    std::vector<unsigned int> prev_idx;
    std::vector<unsigned int> prev_counts;
    std::vector<double> prev_p_sums;
    std::vector<double> acc;


    std::vector<PredicateEffect> effects;

    RobotKinematicsPointer robot;

    Params p;

    // parameters
    double model_norm;
    double best_p;
    double transitions_step;
    unsigned int best_idx;
    unsigned int cur_iter;

    //static std::uniform_real_distribution<double> unif_rand(0.,1.);
    //static std::default_random_engine re;

  public:

    TestFeaturesPointer features;

    // data
    std::vector<FeatureVector> params;
    std::vector<JointTrajectory> trajs;
    std::vector<double> ps;
    std::vector<double> next_ps;
    std::vector<double> iter_lls;
    std::vector<unsigned int> next_skill;

    bool current;

    /**
     * normalize the transition probabilities
     */
    void updateTransitions();

    /** 
     * default constructor
     */
    InstantiatedSkill();

    /**
     * set up with parameters
     */
    InstantiatedSkill(Params &p_);

    /**
     * set all child skills to not done
     */
    InstantiatedSkill &refresh();

    /**
     * set all variables back to original values
     */
    void reset();

    /**
     * create a new skill with dmps
     */
    static InstantiatedSkillPointer DmpInstance(SkillPointer skill,
                                                TestFeaturesPointer features,
                                                RobotKinematicsPointer robot,
                                                unsigned int nbasis);

    /**
     * create a new skill with dmps
     */
    static InstantiatedSkillPointer DmpInstance(SkillPointer skill,
                                                SkillPointer grasp,
                                                TestFeaturesPointer features,
                                                RobotKinematicsPointer robot,
                                                unsigned int nbasis);


    /**
     * create a new skill with spline and segments
     */
    static InstantiatedSkillPointer SplineInstance(SkillPointer skill,
                                                   TestFeaturesPointer features,
                                                   RobotKinematicsPointer robot,
                                                   unsigned int nseg);

    /**
     * create an empty root node
     */
    static InstantiatedSkillPointer Root();

    /**
     * define a possible child
     */
    InstantiatedSkill &addNext(InstantiatedSkillPointer skill);

    /**
     * run a single iteration of the loop. return a set of trajectories.
     * this is very similar to code in the demo
     * PROBABILITY is p(next_skill)
     * PS_OUT is the adjusted probability of each trajectory (normalized)
     */
    void step(const std::vector<double> &ps,
              const std::vector<trajectory_msgs::JointTrajectoryPoint> &start_pts,
              std::vector<double> &ps_out,
              double &probability,
              unsigned int len, // number of input samples provided (AKA prev samples)
              int horizon,
              unsigned int samples);

    /**
     * descend through the tree
     * execute as we reach nodes that require it
     * use gripper tool to send messages
     */
    void execute();


    // randomly sample an index from the probabilities
    unsigned int sampleIndex(unsigned int nsamples) const;

    void initializePs(std::vector<double> &ps);
    void accumulateProbs(const std::vector<double> &prev_ps, unsigned int len);
    void copyEndPoints(const std::vector<JointTrajectoryPoint> &prev_end_pts,
                       const std::vector<double> &prev_ps, unsigned int len);

  };


}

#endif
