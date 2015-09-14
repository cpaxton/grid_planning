#!/usr/bin/env python

import grid_plan
from grid_plan import PyPlanner
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

rospy.init_node('grid_run_ur5')

""" ========================================================================= """
gp = PyPlanner(preset='ur5')

skill_filename = 'ur5_skills/%s_skill.yml'%(sys.argv[1])
skill = grid.RobotSkill(filename=skill_filename)
print "Loaded skill '%s'"%(skill.name)

gp.robot.SetActionNormalizer(skill)
if len(sys.argv) > 2:
    goal_filename = 'ur5_skills/%s_skill.yml'%(sys.argv[2])
    goal = grid.RobotSkill(filename=goal_filename)
    print "Loaded next skill '%s'"%(goal.name)
    skill.goal_model = goal.GetGoalModel(skill.objs,preset="ur5")
    gp.robot.SetGoalNormalizer(goal)
else:
    skill.goal_model = None

gp.SetTrajectory(skill.trajectory_model)
gp.robot.UpdateManipObj(skill.manip_objs)
#if skill.name in ['transport','align','place','release']:
#    print "Disabling collisions with gbeam objects."
#    gp.gp.SetCollisions('gbeam_soup',True)

""" ========================================================================= """

print "Starting search:"

config = [('tool','filtered/camera_2/ar_marker_1'),('vise','filtered/camera_2/ar_marker_2')]

#reg = GripperRegressor(gp.robot,gp.gripper_topic,gp.skill_topic,"/progress")
#reg.addSkill(skill)
#reg.configure(config)

if not skill.name=='close':
    tc = TrajectoryCommander(gp.robot,"/trajectory","/progress","/gazebo/traj_rml/action")

    rospy.sleep(rospy.Duration(0.1))

    skill_guesses = {'take':[-0.1,-0.05,0],'close':[-0.1,-0.05,0],'lift':[-0.3,0.5,0.1]}

    print "Calling planner..."
    cmd,msg,traj,Z = gp.plan(
            skill,
            config,
            num_iter=30,
            tol=1e-20,
            num_valid=50,
            num_samples=500,
            step_size=0.55,
            npts=4,
            skip_bad=False,
            guess_goal_x=skill_guesses[skill.name])

    print "Saving trajectory result."

    stream = file('traj.yml','w')
    stream.write(yaml.dump(traj,Dumper=Dumper))
    stream.close()

    print "Publishing."

    #reg.start()

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

else:

    print "Publishing."
    gp.notify(skill.name)
    rospy.sleep(rospy.Duration(0.1))
    gp.notify(skill.name)
    rospy.sleep(rospy.Duration(0.1))
    #reg.start()

    print "=== TODO: SEND GRIPPER COMMAND HERE ===="

    tc = TrajectoryCommander(gp.robot,None,"/progress",None,step=0.01)
    tc.play(0.05)

    print "Done."
    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException, ex:
        pass
