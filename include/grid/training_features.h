#ifndef _GRID_TRAINING_FEATURES
#define _GRID_TRAINING_FEATURES

#include <grid/features.h>

#include <unordered_map>

// training features reads the feature data from ROS topics
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sensor_msgs/JointState.h>

#include <grid/robot_kinematics.h>

#include <memory>

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
                              double mintime = 0,
                              double maxtime = 0);

    /* getFeatureValues
     * Returns a list of features converted into a format we can use.
     */
    std::vector<std::vector<double> > getFeatureValues(const std::string &name,
                                                       double mintime = 0,
                                                       double maxtime = 0);


    /**
     * get all available features
     * for testing, at least for now
     */
    std::vector<std::vector<double> > getAllFeatureValues();

    /**
     * helper
     * convert a world into a set of features
     */
    std::vector<double> worldToFeatures(const WorldConfiguration &w) const;

    /**
     * read
     * Open a rosbag containing the demonstrations.
     * We make some assumptions as to how these are stored.
     * This function will read in the poses and other information associated with the robot.
     * This information all gets stored and can be used to compute features or retrieve world configurations.
     */
    void read(const std::string &bagfile);

    /** 
     * setRobotKinematics
     * sets the thing that will actually compute forward and inverse kinematics for our robot
     */
    void setRobotKinematics(std::shared_ptr<RobotKinematics> rk);

    /**
     * initialize training features with the necessary world objects to find
     */
    TrainingFeatures(const std::vector<std::string> &objects);

    /**
     * print basic info for debugging
     */
    void printTrainingFeaturesInfo();

    /**
     * print all extractable features for the different objects
     */
    void printExtractedFeatures();

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
    RobotKinematicsPointer robot; // stores the robot itself
    unsigned int n_dof;

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
