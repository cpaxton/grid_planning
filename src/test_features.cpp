#include <grid/test_features.h>

//#define DEBUG_PRINT_TF_POSE

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
#ifdef DEBUG_PRINT_TF_POSE
      std::cout << "looking up " << objectClassToID[key] << " for " << key << std::endl;
#endif
      listener.lookupTransform(worldFrame, objectClassToID[key],
                               ros::Time(0), transform);
      tf::transformTFToKDL(transform, p);
#ifdef DEBUG_PRINT_TF_POSE
      std::cout << "[" << key << "] x = " << transform.getOrigin().getX() << std::endl;
      std::cout << "[" << key << "] y = " << transform.getOrigin().getY() << std::endl;
      std::cout << "[" << key << "] z = " << transform.getOrigin().getZ() << std::endl;
#endif
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s with key \"%s\"",ex.what(),key.c_str());
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
        //std::cout << feature.first << std::endl;
        currentPose[feature.first] = lookup(feature.first);
      }
    }
  }

  /* getFeaturesForTrajectory
   * Get information for a single feature over the whole trajectory given in traj.
   * Traj is KDL::Trajectory
   */
  std::vector<FeatureVector> TestFeatures::getFeaturesForTrajectory(const std::vector<std::string> &names, Trajectory *traj, double dt) {
    std::vector<FeatureVector> features((unsigned int)ceil(traj->Duration() / dt));
    unsigned int next_idx = 0;
    unsigned int dim = getFeaturesSize(names);
    for (double t = 0; t < traj->Duration(); t += dt) {
      unsigned int idx = 0;
      FeatureVector f(dim);
      for (const std::string &name: names) {
        if (feature_types[name] == POSE_FEATURE) {
          Pose offset = currentPose[name].Inverse() * currentPose[AGENT] * traj->Pos(t);

          //std::cout << __LINE__ << ": " << dim << ", " << idx << std::endl;
          getPoseFeatures(offset,f,idx);
          idx+= POSE_FEATURES_SIZE;

        } else if (feature_types[name] == TIME_FEATURE) {
          f(idx) = t / traj->Duration();
          idx += TIME_FEATURES_SIZE;
        }
      }
      //std::cout << next_idx << "/" << features.size() << std::endl;
      //std::cout << t << "/" << traj->Duration() << std::endl;
      features[next_idx++] = f;
      //features.push_back(f);
    }
    return features;
  }

  /* getFeaturesForTrajectory
   * Get information for a single feature over the whole trajectory given in traj.
   * Traj is a set of frames
   */
  std::vector<FeatureVector> TestFeatures::getFeaturesForTrajectory(const std::vector<std::string> &names, TrajectoryFrames traj) {

    std::vector<FeatureVector> features(traj.size());

    unsigned int next_idx = 0;
    unsigned int dim = getFeaturesSize(names);

    for (const Pose &p: traj) {
      unsigned int idx = 0;
      FeatureVector f(dim);

      for (const std::string &name: names) {

        if (feature_types[name] == POSE_FEATURE) {
          Pose offset = currentPose[name].Inverse() * currentPose[AGENT] * p;

          getPoseFeatures(offset,f,idx);
          idx+= POSE_FEATURES_SIZE;

        } else if (feature_types[name] == TIME_FEATURE) {
          f(idx) = (double)next_idx / (double)traj.size();
          idx += TIME_FEATURES_SIZE;
        }

      }
      features[next_idx++] = f;

      return features;
    }
  }
}
