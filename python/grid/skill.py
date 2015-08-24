
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
    def __init__(self,data=[],params=[],goals=[],action_k=4,goal_k=4,objs=[],name="",filename=None,num_gripper_vars=3):
        self.name = name

        self.num_gripper_vars = num_gripper_vars
        self.action_model = GMM(n_components=action_k,covariance_type="full")
        self.goal_model = GMM(n_components=goal_k,covariance_type="full")
        self.trajectory_model = GMM(n_components=1,covariance_type="full")
        self.gripper_model = GMM(n_components=action_k,covariance_type="full")
        self.objs = objs

        if filename == None and len(data) > 0:
            # learn action, goal, and trajectory models
            self.goal_model.fit(goals)
            self.action_model.fit(data)
            self.trajectory_model.fit(params)
            self.t_factor = 0.1

            if 'gripper' in objs:
                # remove last few indices from
                self.gripper_model = self.action_model
                self.action_model = copy.deepcopy(self.gripper_model)

                # marginalizing out vars in gaussians is easy
                self.action_model.means_ = self.gripper_model.means_[:,:-num_gripper_vars]
                self.action_model.covars_ = self.gripper_model.covars_[:,:-num_gripper_vars,:-num_gripper_vars]
                self.goal_model.covars_ = self.goal_model.covars_[:,:-num_gripper_vars,:-num_gripper_vars]
                self.goal_model.means_ = self.goal_model.means_[:,:-num_gripper_vars]

        elif not filename == None:
            stream = file(filename,'r')
            data = yaml.load(stream,Loader=Loader)

            self.name = data['name']
            self.action_model = data['action_model']
            self.goal_model = data['goal_model']
            self.gripper_model = data['gripper_model']
            self.trajectory_model = data['trajectory_model']
            self.objs = data['objs']
            self.num_gripper_vars = data['num_gripper_vars']
            self.t_factor = 0.1

    '''
    save the robot skill to a file
    '''
    def save(self,filename):
        stream = file(filename,'w')

        out = {}
        out['name'] = self.name
        out['action_model'] = self.action_model
        out['goal_model'] = self.goal_model
        out['gripper_model'] = self.gripper_model
        out['trajectory_model'] = self.trajectory_model
        out['objs'] = self.objs
        out['num_gripper_vars'] = self.num_gripper_vars

        yaml.dump(out,stream)

    '''
    execution loop update for trajectory skills
    '''
    def update(self, trajs, p_z, t, p_obs=1):
        wts = np.zeros(len(trajs));
        for i in range(len(trajs)):
            p_exp = self.trajectory_model.score(trajs[i][:-1])
            p_exp_f = self.goal_model.score(trajs[i][-1])

            p_exp = np.concatenate((p_exp,p_exp_f))

            wts[i] = weight(p_exp, 1, p_z[i], t[i], self.t_lambda)


'''
This function determines the weight on a given example point
p_expert: different for each point
p_obs: same (actually fixed at 1)
p_z: same for each trajectory
t: different for each point
'''
def weight(p_expert,p_obs,p_z,t,t_lambda=0.1):
    return (p_expert * t_lambda**(1-t)) / (p_obs * p_z)

