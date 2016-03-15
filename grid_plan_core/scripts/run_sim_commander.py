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
skills = []
for i in range(2,len(sys.argv)):
	skill_filename = 'skills/sim/%s_skill.yml'%(sys.argv[1])
	skills.append(grid.RobotSkill(filename=skill_filename))
	print "Loaded skill '%s'"%(skills[-1].name)

#rospy.wait_for_service('/gazebo/publish_planning_scene')
#pps = rospy.ServiceProxy('/gazebo/publish_planning_scene',Empty)
#pps()
#rospy.sleep(rospy.Duration(0.1))
#pps()

skill_topic = "current_skill"
config = [('link','/gbeam_link_1/gbeam_link'),('node','/gbeam_node_1/gbeam_node')]
joint_states_topic="/gazebo/barrett_manager/wam/joint_states"
planning_scene_topic="/gazebo/planning_scene"
gripper_topic='/gazebo/barrett_manager/hand/cmd'

preset = "wam7_sim"
if preset == "wam7_sim":
    base_link = 'wam/base_link'
    end_link = 'wam/wrist_palm_link'
    robot_description="robot_description"
    joint_states_topic="/gazebo/barrett_manager/wam/joint_states"
    planning_scene_topic="/gazebo/planning_scene"
    gripper_topic="/gazebo/barrett_manager/hand/cmd"
    command_topic="/gazebo/traj_rml/joint_traj_cmd"
    dof = 7

elif preset == "ur5":
    base_link = '/base_link'
    end_link = '/ee_link'
    robot_description="/robot_description"
    joint_states_topic="/joint_states"
    planning_scene_topic="/planning_scene"
    command_topic='/arm_controller/command'
    dof = 6

robot = grid.RobotFeatures(
        base_link=base_link,
        end_link=end_link,
        js_topic=joint_states_topic,
        gripper_topic=gripper_topic,
	dof=dof,
	preset=preset)

reg = GripperRegressor(robot,gripper_topic,skill_topic,"/progress")
for skill in skills:
	reg.addSkill(skill)

print "Configuring regressor..."
reg.configure(config)

tc = TrajectoryCommander(robot,"/trajectory","/progress","/gazebo/traj_rml/action")
#reg.start()

try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rate.sleep()
except rospy.ROSInterruptException, ex:
    pass

