#include <grid/test_features.h>

namespace grid {

  const std::string TestFeatures::AGENT("agent");

  /* getPose
   * This function needs to be implemented by inheriting classes.
   * Time field helps determine when the query should occur.
   * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
   */
  std::vector<Pose> TestFeatures::getPose(const std::string &name,
                                          unsigned long int mintime,
                                          unsigned long int maxtime) {
    std::vector<Pose> poses;


    return poses;
  }

  /* getFeatureValues
   * Returns a list of features converted into a format we can use.
   */
  std::vector<std::vector<double> > TestFeatures::getFeatureValues(const std::string &name,
                                                                   unsigned long int mintime,
                                                                   unsigned long int maxtime) {
    std::vector<std::vector<double> > values;



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
    try{
      listener.lookupTransform(objectClassToID[key], worldFrame,  
                               ros::Time(0), transform);
      std::cout << "[" << key << "] x = " << transform.getOrigin().getX() << std::endl;
      std::cout << "[" << key << "] y = " << transform.getOrigin().getY() << std::endl;
      std::cout << "[" << key << "] z = " << transform.getOrigin().getZ() << std::endl;
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
    }
  }
}
