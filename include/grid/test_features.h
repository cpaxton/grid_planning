#include <grid/features.h>

namespace grid {

  class TestFeatures : public Features {

  public:

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
                                                       unsigned long int maxtime = 0);
  };

}
