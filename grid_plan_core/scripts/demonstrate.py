#!/usr/bin/env python

" IO "
import sys
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

" GRID "
import grid

" ROS "
import rospy
import tf

" Message types "
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from oro_barrett_msgs.msg import BHandCmd

" Conversions "
import tf_conversions.posemath as pm
import PyKDL as kdl

'''
Demonstration
Listen for information from the WAM arm to collect a demonstration
Save the whole joint state trajectory as well as the whole feature space trajectory.

Feature space:
    - stored as a set of KDL transforms between bhand_wrist and objects

Joint space:
    - stored as arrays of joint states
    - also has a variable for the closure of the gripper
'''


if __name__ == '__main__':

    filename = 'demo.yml'
    link = 'link'
    if len(sys.argv) > 1:
        filename = sys.argv[1]
    if len(sys.argv) > 2:
        link = sys.argv[2]

    rospy.init_node('demonstration_observer')
    demo = grid.RobotFeatures()

    # set up parameters
    world='/world'
    frame='/wam/hand/bhand_palm_link'
    obj1='/gbeam_link_1/gbeam_' + link
    obj2='/gbeam_node_1/gbeam_node'

    # set up ros things
    rate = rospy.Rate(10)

    demo.AddObject("link",obj1);
    demo.AddObject("node",obj2);

    rospy.sleep(rospy.Duration(0.5))

    demo.StartRecording()

    try:
        tl = tf.TransformListener()
        while not rospy.is_shutdown():
            #try:
            #    (trans,rot) = tl.lookupTransform('world',obj1,rospy.Time(0))
            #    print (trans,rot)
            #except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            #    print e
            #print rospy.Time.now()
            rate.sleep()

    except rospy.ROSInterruptException:
        demo.save(filename)
