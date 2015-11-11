#ifndef _GRID_FEATURES
#define _GRID_FEATURES

/**
 * Features
 * Parent abstract class for a set of robot features.
 * Abstract method produces appropriate poses given queries.
 */

namespace grid {

  class Features {

    /* getPose
     * This function needs to be implemented by inheriting classes.
     * Time field helps determine when the query should occur.
     * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
     */
    virtual std::vector<pose_t> getPose(const std::string &name,
        unsigned long int mintime = 0,
        unsigned long int maxtime = 0) = 0;

    /*
     * Processing code.
     * Needs to, among other things, get all the features and turn them into normalized data.
     */
    void initialize();

    /**
     *
     */
    std::vector< std::vector <double> > getFeatures(std::vector<std::string> &names);

  };
}

#endif
