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

filenames = ["app1.yml","app2.yml","app3.yml"]

rospy.init_node('ipython2')

""" ========================================================================= """
gp = PyPlanner()

if sys.argv[1] == "approach":
    skill = grid.RobotSkill(filename='skills/approach_skill.yml')
elif sys.argv[1] == "transport":
    skill = grid.RobotSkill(filename='skills/transport_skill.yml')
elif sys.argv[1] == "grasp":
    skill = grid.RobotSkill(filename='skills/grasp_skill.yml')
elif sys.argv[1] == "disengage":
    skill = grid.RobotSkill(filename='skills/disengage_skill.yml')

print "Loaded skill '%s'"%(skill.name)

""" ========================================================================= """

print "Starting search:"

rospy.wait_for_service('/gazebo/publish_planning_scene')
pps = rospy.ServiceProxy('/gazebo/publish_planning_scene',Empty)
pps()
rospy.sleep(rospy.Duration(0.1))
pps()

reg = GripperRegressor(gp.robot,gp.gripper_topic,gp.skill_topic,"/progress")
reg.addSkill(skill)

tc = TrajectoryCommander(gp.robot,"/trajectory","/progress","/gazebo/traj_rml/action")

cmd,msg,traj,Z = gp.plan(
        skill,
        [('link','/gbeam_link_1/gbeam_link'),('node','/gbeam_node_1/gbeam_node')],
        num_iter=30,
        tol=0.001,
        num_valid=50,
        num_samples=2500)

pub = rospy.Publisher("/trajectory",JointTrajectory)
pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)

rospy.sleep(rospy.Duration(0.5))
pub.publish(cmd)
pa_ee_pub.publish(msg)

try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pa_ee_pub.publish(msg)
        rate.sleep()
except rospy.ROSInterruptException, ex:
    pass

