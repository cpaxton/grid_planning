#include <grid/training_features.h>

// read in poses from topics
#include <geometry_msgs/Pose.h>
#include <kdl_conversions/kdl_msg.h>
#include <iostream>
#include <sstream>

namespace grid {

  /**
   * initialize training features with the necessary world objects to find
   */
  TrainingFeatures::TrainingFeatures(const std::vector<std::string> &objects_) : objects(objects_) {
    topics.push_back(JOINT_STATES_TOPIC);
    topics.push_back(BASE_TFORM_TOPIC);
    topics.push_back(GRIPPER_MSG_TOPIC);

    for (std::string &obj: objects) {
      std::stringstream ss;
      ss << "world/" << obj;
      topics.push_back(ss.str().c_str());
      topic_to_object[ss.str().c_str()]=obj;
    }
  }

  /**
   * print basic info for debugging
   */
  void TrainingFeatures::printTrainingFeaturesInfo() {
    std::cout << "Topics: " << std::endl;
    for (std::string &topic: topics) {
      std::cout << " - " << topic << std::endl;
    }

    std::cout << "Loaded " << data.size() << " data points." << std::endl;

    for (WorldConfiguration &conf: data) {
      std::cout << "Data point at t=" << conf.t << std::endl;
      std::cout << "\tBase at x=" << conf.base_tform.p.x()
        <<", y=" << conf.base_tform.p.y()
        <<", z=" << conf.base_tform.p.y()
        <<std::endl;
      std::cout <<"\tJoints = ";
      for (double &q: conf.joint_states.position) {
        std::cout << q << ", ";
      }
      std::cout << std::endl;

      for(std::pair<const std::string,Pose> &obj: conf.object_poses) {
        std::cout <<"\tObject \"" << obj.first << "\" at x=" << obj.second.p.x()
          <<", y=" << obj.second.p.y()
          <<", z=" << obj.second.p.z()
          <<std::endl;
      }

    }
  }

  /* getPose
   * This function needs to be implemented by inheriting classes.
   * Time field helps determine when the query should occur.
   * A feature query gets the set of all featutes for different points in time, normalizes them, and returns.
   */
  std::vector<Pose> TrainingFeatures::getPose(const std::string &name,
                                              unsigned long int mintime,
                                              unsigned long int maxtime) {
    std::vector<Pose> poses;


    return poses;
  }


  /* getFeatureValues
   * Returns a list of features converted into a format we can use.
   */
  std::vector<std::vector<double> > TrainingFeatures::getFeatureValues(const std::string &name,
                                                                       unsigned long int mintime,
                                                                       unsigned long int maxtime) {
    std::vector<std::vector<double> > values;



    return values;
  }

  /**
   * read
   * Open a rosbag containing the demonstrations.
   * We make some assumptions as to how these are stored.
   * This function will read in the poses and other information associated with the robot.
   * This information all gets stored and can be used to compute features or retrieve world configurations.
   */
  void TrainingFeatures::read(const std::string &bagfile) {
    using rosbag::View;
    using rosbag::MessageInstance;
    typedef geometry_msgs::Pose PoseMsg;

    bag.open(bagfile, rosbag::bagmode::Read);

    View view(bag, rosbag::TopicQuery(topics));

    WorldConfiguration conf;
    conf.t = ros::Time(0);

    // loop over all messages
    for(const MessageInstance &m: view) {

      std::cout << "[READING IN] " << m.getTopic() << std::endl;

      if (conf.t == ros::Time(0)) {
        conf.t = m.getTime();
      } else if (conf.t != m.getTime()) {
        data.push_back(conf);
        conf = WorldConfiguration();
        conf.t = m.getTime();
      }

      if (m.getTopic() == GRIPPER_MSG_TOPIC) {
        conf.gripper_cmd = getGripperFeatures(m);
      } else if (m.getTopic() == JOINT_STATES_TOPIC) {

        // save joints
        using sensor_msgs::JointState;
        conf.joint_states = *m.instantiate<JointState>();

        // compute forward kinematics

      } else if (m.getTopic() == BASE_TFORM_TOPIC) {

        // convert from message to KDL pose
        PoseMsg msg = *m.instantiate<PoseMsg>();
        Pose p;
        tf::poseMsgToKDL(msg,p);
        conf.base_tform = p;

      } else if (topic_to_object.find(m.getTopic()) != topic_to_object.end()) {

        // convert from message to KDL pose
        PoseMsg msg = *m.instantiate<PoseMsg>();
        Pose p;
        tf::poseMsgToKDL(msg,p);

        // assign data
        conf.object_poses[topic_to_object[m.getTopic()]] = p;
      }
    }

    bag.close();

  }

  /**
   * return the joint states data we care about
   */
  Pose TrainingFeatures::getJointStatesData(rosbag::MessageInstance const &m) {
    Pose p;

    return p;
  }

  /**
   * get the object poses we care about
   */
  Pose TrainingFeatures::getObjectData(rosbag::MessageInstance const &m) {
    Pose p;

    return p;
  }

#if 0
  /*
   * return the gripper features from a rosbag
   * default: returns nothing
   */
  virtual std::vector<double> TrainingFeatures::getGripperFeatures(rosbag::MessageInstance const &m) {
    return std::vector<double>();
  }
#endif

}
