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
#from oro_barrett_msgs.msg import BHandCmd
from robotiq_c_model_control.msg import CModel_gripper_command

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
    if len(sys.argv) > 1:
        filename = sys.argv[1]

    rospy.init_node('demonstration_observer')
    demo = grid.RobotFeatures(preset='ur5')

    # set up parameters
    world='/world'
    frame='/ee_link'
    #print demo.base_link
    #print demo.end_link

    # fix world parameters
    #obj1='/gbeam_link_1/gbeam_link'
    #obj2='/gbeam_node_1/gbeam_node'
    obj1='filtered/camera_2/ar_marker_1'
    obj2='filtered/camera_2/ar_marker_2'

    # set up ros things
    rate = rospy.Rate(1)

    demo.AddObject("tool",obj1);
    demo.AddObject("vise",obj2);

    rospy.sleep(rospy.Duration(0.5))

    demo.StartRecording()

    try:
        #tl = tf.TransformListener()
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
