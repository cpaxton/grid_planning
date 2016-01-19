#!/usr/bin/env python

import grid_plan
from grid_plan import PyPlanner
from grid_plan import GripperRegressor
from grid_plan import TrajectoryCommander

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

rospy.init_node('run_sim_commander_node')

""" ========================================================================= """
for i in range(2,len(sys.argv)):
	skill_filename = 'skills/sim/%s_skill.yml'%(sys.argv[1])
	skills.append(grid.RobotSkill(filename=skill_filename))
	print "Loaded skill '%s'"%(skills[-1].name)

#rospy.wait_for_service('/gazebo/publish_planning_scene')
#pps = rospy.ServiceProxy('/gazebo/publish_planning_scene',Empty)
#pps()
#rospy.sleep(rospy.Duration(0.1))
#pps()

config = [('link','/gbeam_link_1/gbeam_link'),('node','/gbeam_node_1/gbeam_node')]

reg = GripperRegressor(gp.robot,gp.gripper_topic,gp.skill_topic,"/progress")
for skill in skills:
	reg.addSkill(skill)
	reg.configure(config)

tc = TrajectoryCommander(gp.robot,"/trajectory","/progress","/gazebo/traj_rml/
reg.start()

try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
except rospy.ROSInterruptException, ex:
    pass

