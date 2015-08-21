
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
from std_srvs.srv import Empty

" math "
import numpy as np
import tf_conversions.posemath as pm

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

NUM_VALID = 50
NUM_SAMPLES = 2500

class PyPlanner:

    def __init__(self,preset="wam"):

        if preset == "wam":
            self.gp = grid_plan.GridPlanner('robot_description',
                    '/gazebo/barrett_manager/wam/joint_states',
                    '/gazebo/planning_scene',0.0)
            self.gp.SetDof(7);
            self.gp.SetNumBasisFunctions(5);
            self.gp.SetK(100);
            self.gp.SetD(20);
            self.gp.SetTau(2.0);
            self.gp.SetGoalThreshold(0.1);
            self.gp.SetVerbose(False);


            self.robot = grid.RobotFeatures()
            #self.obj1='/gbeam_link_1/gbeam_link'
            #self.obj2='/gbeam_node_1/gbeam_node'
            #self.robot.AddObject("link",obj1);
            #self.robot.AddObject("node",obj2);
            self.base_link = 'wam/base_link'
            self.end_link = 'wam/wrist_palm_link'

    '''
    take a skill and associated objects
    return a trajectory
    - objs is a mapping between tf frames and names
    - skill is a RobotSkill
    '''
    def plan(self,skill,objs,num_iter=20):
        for (obj,frame) in objs:
            self.robot.AddObject(obj,frame)


