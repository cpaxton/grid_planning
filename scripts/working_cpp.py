#!/usr/bin/env python

import grid_plan

from moveit_ros_planning_interface._moveit_roscpp_initializer import roscpp_init

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

filenames = ["app1.yml","app2.yml","app3.yml"]

roscpp_init('ipython', [])
rospy.init_node('ipython2')

""" ========================================================================= """
gp = grid_plan.GridPlanner('robot_description',
        '/gazebo/barrett_manager/wam/joint_states',
        '/gazebo/planning_scene',0.0)
gp.SetDof(7);
gp.SetNumBasisFunctions(5);
gp.SetK(100);
gp.SetD(20);
gp.SetTau(1.0);
gp.SetGoalThreshold(0.1);

""" ========================================================================= """
robot = RobotFeatures()
obj1='/gbeam_link_1/gbeam_link'
obj2='/gbeam_node_1/gbeam_node'
robot.AddObject("link",obj1);
robot.AddObject("node",obj2);
base_link = 'wam/base_link'
end_link = 'wam/wrist_palm_link'

data,params,num_weights,goals = LoadDataDMP(filenames,['link'])

print "Fitting GMM to trajectory parameters..."
Z = GMM(covariance_type="full")
Z.n_components = 1
Z = Z.fit(params)

print "Fitting GMM to expert action features..."
training_data = np.array(data[0][1])
for i in range(1,len(data)):
    training_data = np.concatenate((training_data,data[i][1]))
traj_model = GMM(n_components=1,covariance_type="full")
traj_model.fit(training_data)

print "Fitting GMM to expert goal features..."
expert = GMM(n_components=1,covariance_type="full")
expert = expert.fit(goals)

