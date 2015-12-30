#ifndef _GRID_TRAINING_FEATURES
#define _GRID_TRAINING_FEATURES

#include <grid/features.h>

#include <unordered_map>

// training features reads the feature data from ROS topics
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>

#include <kdl/chain.hpp>
#include <kdl/tree.hpp>

namespace grid {

  struct WorldConfiguration {
    ros::Time t; 
    std::unordered_map<std::string,Pose> object_poses;
    Pose base_tform;
    sensor_msgs::JointState joint_states;
    Pose ee_tform;
    std::vector<double> gripper_cmd;
  };

  class TrainingFeatures: public Features {

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
    /**
     * read
     * Open a rosbag containing the demonstrations.
     * We make some assumptions as to how these are stored.
     * This function will read in the poses and other information associated with the robot.
     * This information all gets stored and can be used to compute features or retrieve world configurations.
     */
    void read(const std::string &bagfile);

    /**
     * initialize training features with the necessary world objects to find
     */
    TrainingFeatures(const std::vector<std::string> &objects, const std::string &robot_description_param = "robot_description");

    /**
     * print basic info for debugging
     */
    void printTrainingFeaturesInfo();

  protected:

    /*
     * return the gripper features from a rosbag
     * must be implemented for the specific gripper being used
     */
    virtual std::vector<double> getGripperFeatures(rosbag::MessageInstance const &m) = 0;


    std::vector<std::string> objects; // objects we need
    std::vector<std::string> topics; // topics to pull from rosbag
    rosbag::Bag bag; // current bag holding all demonstration examples
    std::vector<WorldConfiguration> data; //all loaded data
    std::unordered_map<std::string,std::string> topic_to_object; //maps topics back to objects
    std::string robot_description_param;
    std::string robot_description;
    std::string tip_link;
    std::string root_link;
    unsigned int n_dof;
    KDL::Chain kdl_chain;
    KDL::Tree kdl_tree;

    /**
     * return the joint states data we care about
     */
    Pose getJointStatesData(rosbag::MessageInstance const &m);

    /**
     * get the object poses we care about
     */
    Pose getObjectData(rosbag::MessageInstance const &m);

  };

}

#endif
