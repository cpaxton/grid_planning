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

if __name__ == '__main__':
    rospy.init_node('sample_trajectories')

    if len(sys.argv) > 1:
        filenames = sys.argv[1:]
    else:
        filenames = ["app1.yml","app2.yml"]


    data,params,num_weights = LoadDataDMP(filenames)

    print "Fitting GMM to trajectory parameters..."
    #Z = GMM(dim=len(params[0]),ncomps=2,data=np.array(params),method="kmeans")
    Z = GMM()
    Z = Z.fit(params)

    print "Fitting GMM to expert features..."
    end_training_data = [data[0][1][-1]]
    for i in range(1,len(data)):
        #training_data = np.concatenate((training_data,data[i][1]))
        end_training_data += [data[i][1][-1]]
