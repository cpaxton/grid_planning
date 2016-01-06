#ifndef _GRID_TEST_FEATURES
#define _GRID_TEST_FEATURES

#include <grid/features.h>
#include <tf/transform_listener.h>

namespace grid {

  class TestFeatures : public Features {

  public:

    /* getPose
     * This function needs to be implemented by inheriting classes.
     * Time field helps determine when the query should occur.
     * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
     */
    std::vector<Pose> getPose(const std::string &name,
                              double mintime = 0,
                              double maxtime = 0);
    /* getFeatureValues
     * Returns a list of features converted into a format we can use.
     */
    std::vector<FeatureVector> getFeatureValues(const std::string &name,
                                                double mintime = 0,
                                                double maxtime = 1);

    /* getFeaturesForTrajectory
     * Get information for a single feature over the whole trajectory given in traj.
     * Traj is KDL::Trajectory
     */
    std::vector<FeatureVector> getFeaturesForTrajectory(const std::vector<std::string> &names, Trajectory *traj, double dt = 0.05);

    /* getFeaturesForTrajectory
     * Get information for a single feature over the whole trajectory given in traj.
     * Traj is a set of frames
     */
  std::vector<FeatureVector> getFeaturesForTrajectory(const std::vector<std::string> &name, TrajectoryFrames traj);

    /* addFrame
     * Adds a frame of reference as a feature
     */
    void setFrame(const std::string &frame, const std::string &objectClass);

    /* addAgent:
     * configure agent's manipulation frame
     */
    void setAgentFrame(const std::string &agentFrame);

    /* configure world frame for this TestFeatures object
    */
    void setWorldFrame(const std::string &worldFrame);

    /* lookup tf frame for key
    */
    Pose lookup(const std::string &key);

    /*
     * run lookup for all objects
     * store results for poses from tf
     */
    void updateWorldfromTF();

  private:
    std::unordered_map<std::string, std::string> objectClassToID;
    std::unordered_map<std::string, Pose> currentPose; // used for fast lookup

    // name of the frame representing the end effector, i.e. what are we planning from?
    std::string agentFrame;

    // name of the root/world frame
    std::string worldFrame;

    // tf
    tf::TransformListener listener;
  };

}

#endif
