#!/usr/bin/env python

import grid

" IO "
import sys
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

import copy

" ROS "
import rospy
import sensor_msgs
from geometry_msgs.msg import PoseArray

" math "
import numpy as np
import tf_conversions.posemath as pm

" machine learning "
from sklearn.mixture import GMM
#from gmm import GMM # conditional GMM

" Fitting DMPs "
from dmp.srv import *
from dmp.msg import *
from grid import *

" control "
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

" kdl utilities "
import PyKDL
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

" visualizations "
from visualization_msgs.msg import MarkerArray
from visualization_msgs.msg import Marker

'''
check_features
script computes features and sees if they make sense
'''

pos = None
vel = None
robot = None

def js_cb(msg):
    global pos
    global vel
    global robot
    pos = msg.position
    vel = msg.velocity

    if not robot == None:

        world = None
        while world == None or not robot.TfUpdateWorld():
            world = robot.TfCreateWorld()

        print robot.GetFeatures(pos,1,world,["link"])

rospy.init_node('check_features_node')

global robot
robot = grid.RobotFeatures()
obj1='/gbeam_link_1/gbeam_link'
obj2='/gbeam_node_1/gbeam_node'
robot.AddObject("link",obj1);
robot.AddObject("node",obj2);

sub = rospy.Subscriber('/gazebo/barrett_manager/wam/joint_states',sensor_msgs.msg.JointState,js_cb)

rate = rospy.Rate(10)

try:
    while not rospy.is_shutdown():
        rate.sleep()
except rospy.ROSInterruptException, e:
    pass
