#include <grid/test_features.h>

#define DEBUG_PRINT_TF_POSE 1

namespace grid {

  /* getPose
   * This function needs to be implemented by inheriting classes.
   * Time field helps determine when the query should occur.
   * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
   */
  TrajectoryFrames TestFeatures::getPose(const std::string &name,
                                          double mintime,
                                          double maxtime) {
    TrajectoryFrames poses;


    return poses;
  }

  /* getFeatureValues
   * Returns a list of features converted into a format we can use.
   */
  std::vector< FeatureVector > TestFeatures::getFeatureValues(const std::string &name,
                                                                   double mintime,
                                                                   double maxtime) {
    std::vector< FeatureVector > values;

    return values;
  }



  /* setFrame
   * Adds a frame of reference as a feature
   */
  void TestFeatures::setFrame(const std::string &frame, const std::string &objectClass) {
    objectClassToID[objectClass] = frame;
  }

  /* addAgent:
   * configure agent's manipulation frame
   */
  void TestFeatures::setAgentFrame(const std::string &agentFrame_) {
    agentFrame = agentFrame_;
    objectClassToID[AGENT] = agentFrame_;
    addFeature(AGENT,POSE_FEATURE);
  }

  /* configure world frame for this TestFeatures object
  */
  void TestFeatures::setWorldFrame(const std::string &worldFrame_) {
    worldFrame = worldFrame_;
  }

  /* lookup tf frame for key
  */
  Pose TestFeatures::lookup(const std::string &key) {
    tf::StampedTransform transform;
    Pose p;

    try{
      std::cout << "looking up " << objectClassToID[key] << " for " << key << std::endl;
      listener.lookupTransform(worldFrame, objectClassToID[key],
                               ros::Time(0), transform);
      tf::transformTFToKDL(transform, p);
#if DEBUG_PRINT_TF_POSE
      std::cout << "[" << key << "] x = " << transform.getOrigin().getX() << std::endl;
      std::cout << "[" << key << "] y = " << transform.getOrigin().getY() << std::endl;
      std::cout << "[" << key << "] z = " << transform.getOrigin().getZ() << std::endl;
#endif
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }

    return p;
  }

  /*
   * run lookup for all objects
   * store results for poses from tf
   */
  void TestFeatures::updateWorldfromTF() {
    for (const std::pair<std::string,FeatureType> &feature: feature_types) {
      //std::cout << feature.first << ", " << feature.second << std::endl;
      if(feature.second == POSE_FEATURE) {
        currentPose[feature.first] = lookup(feature.first);
      }
    }
  }

  /* getFeaturesForTrajectory
   * Get information for a single feature over the whole trajectory given in traj.
   * Traj is ???
   */
  std::vector<FeatureVector> TestFeatures::getFeaturesForTrajectory(const std::string &name, TrajectoryFrames traj) {
    std::vector<FeatureVector> features(traj.size());
    unsigned int next_idx = 0;
    for (const Pose &p: traj) {
      Pose offset = currentPose[name].Inverse() * currentPose[AGENT] * p;

      FeatureVector f(POSE_FEATURES_SIZE);
      getPoseFeatures(offset,f,0);
      
      features[next_idx++] = f;
    }
    return features;
  }

  /* getFeaturesForTrajectory
   * Get information for a single feature over the whole trajectory given in traj.
   * Traj is KDL::Trajectory
   */
  std::vector<FeatureVector> TestFeatures::getFeaturesForTrajectory(const std::string &name, Trajectory *traj, double dt) {
    std::vector<FeatureVector> features((unsigned int)ceil(traj->Duration() / dt));
    unsigned int next_idx = 0;
    for (double t = 0; t < traj->Duration(); t += dt) {
      Pose offset = currentPose[name].Inverse() * currentPose[AGENT] * traj->Pos(t);

      FeatureVector f(POSE_FEATURES_SIZE);
      f[POSE_FEATURE_X] = offset.p.x();
      f[POSE_FEATURE_Y] = offset.p.y();
      f[POSE_FEATURE_Z] = offset.p.z();
      offset.M.GetRPY(f[POSE_FEATURE_ROLL], f[POSE_FEATURE_PITCH], f[POSE_FEATURE_YAW]);
      
      features[next_idx++] = f;
      //features.push_back(f);
    }
    return features;
  }
}
