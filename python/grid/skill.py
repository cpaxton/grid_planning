
# ROS stuff
import rospy
from urdf_parser_py.urdf import URDF
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
import copy

import numpy as np

# KDL utilities
import PyKDL
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

# machine learning utils (python)
from sklearn.mixture import GMM


# tf stuff
import tf
import tf_conversions.posemath as pm

# input message types 
import sensor_msgs
import oro_barrett_msgs
import trajectory_msgs
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from oro_barrett_msgs.msg import BHandCmd

# output message types
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray


'''
RobotSkill
Defines a skill with a goal and a set of differential constraints
Goals are represented as the distribution of features that must be true for an action to be considered successful
'''
class RobotSkill:
    
    '''
    set up the robot skill
    skills contain a model of expected features as they change over time
    they also contain a description for our own purposes
    oh, and which objects are involved
    '''
    def __init__(self,data,params,goals,action_k=4,goal_k=4,objs={},name=""):
        self.name = name

        self.action_model = GMM(n_components=action_k,covariance_type="full")
        self.goal_model = GMM(n_components=goal_k,covariance_type="full")
        self.trajectory_model = GMM(n_components=1,covariance_type="full")
        
        # learn action, goal, and trajectory models
        self.goal_model.fit(goals)
        self.trajectory_model.fit(params)

    '''
    load the robot skill from a file
    '''
    def __init__(self, filename):
        stream = file(filename,'r')
        data = yaml.load(stream,Loader=Loader)

        self.name = data['name']
        self.action_model = data['action_model']
        self.goal_model = data['goal_model']
        self.trajectory_model = data['trajectory_model']

    '''
    save the robot skill to a file
    '''
    def save(self,filename):
        stream = file(filename,'w')

        out = {}
        out['name'] = self.name
        out['action_model'] = self.action_model
        out['goal_model'] = self.goal_model
        out['trajectory_model'] = self.trajectory_model

        yaml.dump(out,stream)

