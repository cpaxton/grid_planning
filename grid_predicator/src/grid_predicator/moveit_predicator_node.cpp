// ROS
#include <ros/ros.h>

// for debugging
#include <iostream>

// predicator stuff
#include <grid_predicator/predicator.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "moveit_predicator_node");

  grid_predicator::PredicateContext pc(true);

  // define spin rate
  ros::Rate rate(30);

  // start main loop
  while(ros::ok()) {
    ros::spinOnce();
    pc.tick();
    rate.sleep();
  }

  pc.cleanup();
}
