
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

rospy.init_node('ipython')
data,params,num_weights,goals = LoadDataDMP(filenames)

data = grid.LoadRobotFeatures('app3.yml')
fx,goal = data.GetTrainingFeatures(['link'])


