#include <grid/features.h>
#include <tf/transform_listener.h>

namespace grid {

  class TestFeatures : public Features {

  public:

    static const std::string AGENT;

    /* getPose
     * This function needs to be implemented by inheriting classes.
     * Time field helps determine when the query should occur.
     * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
     */
    std::vector<Pose> getPose(const std::string &name,
                              unsigned long int mintime = 0,
                              unsigned long int maxtime = 0);
    /* getFeatureValues
     * Returns a list of features converted into a format we can use.
     */
    std::vector<std::vector<double> > getFeatureValues(const std::string &name,
                                                       unsigned long int mintime = 0,
                                                       unsigned long int maxtime = 1);

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

  private:
    std::unordered_map<std::string, std::string> objectClassToID;

    // name of the frame representing the end effector, i.e. what are we planning from?
    std::string agentFrame;

    // name of the root/world frame
    std::string worldFrame;

    // tf
    tf::TransformListener listener;
  };

}
