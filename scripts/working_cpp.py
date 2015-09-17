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

skill_filename = 'skills/%s_skill.yml'%(sys.argv[1])
skill = grid.RobotSkill(filename=skill_filename)
print "Loaded skill '%s'"%(skill.name)

gp.robot.SetActionNormalizer(skill)
if len(sys.argv) > 2:
    goal_filename = 'skills/%s_skill.yml'%(sys.argv[2])
    goal = grid.RobotSkill(filename=goal_filename)
    print "Loaded next skill '%s'"%(goal.name)
    skill.goal_model = goal.GetGoalModel(skill.objs)
    gp.robot.SetGoalNormalizer(goal)
else:
    skill.goal_model = None
    #gp.robot.SetGoalNormalizer(skill)

gp.SetTrajectory(skill.trajectory_model)
gp.robot.UpdateManipObj(skill.manip_objs)
if skill.name in ['transport','align','place','release']:
    print "Disabling collisions with gbeam objects."
    gp.gp.SetCollisions('gbeam_soup',True)

""" ========================================================================= """

print "Starting search:"

rospy.wait_for_service('/gazebo/publish_planning_scene')
pps = rospy.ServiceProxy('/gazebo/publish_planning_scene',Empty)
pps()
rospy.sleep(rospy.Duration(0.1))
pps()

config = [('link','/gbeam_link_1/gbeam_link'),('node','/gbeam_node_1/gbeam_node')]

reg = GripperRegressor(gp.robot,gp.gripper_topic,gp.skill_topic,"/progress")
reg.addSkill(skill)
reg.configure(config)

if not (skill.name=='grasp' or skill.name == 'release'):
    tc = TrajectoryCommander(gp.robot,"/trajectory","/progress","/gazebo/traj_rml/action")

    rospy.sleep(rospy.Duration(0.1))

    skill_guesses = {'approach':None,'grasp':[0,0,0],'release':[0,0,0],'transport':None,'align':[0.4,0,0],'place':None,'disengage':[0,0,-0.5]}

    npts = 4
    if skill.name == 'place':
        npts = 2
    cmd,msg,traj,Z = gp.plan(
            skill,
            config,
            num_iter=30,
            tol=1e-20,
            num_valid=20,
            num_samples=250,
            step_size=0.55,
            npts=npts, skip_bad=False,
            guess_goal_x=skill_guesses[skill.name])

    print "Saving trajectory result."

    stream = file('traj.yml','w')
    stream.write(yaml.dump(traj,Dumper=Dumper))
    stream.close()

    print "Publishing."

    reg.start()

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
    reg.start()

    tc = TrajectoryCommander(gp.robot,None,"/progress",None,step=0.01)
    tc.play(0.05)

    print "Done."
    try:
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
    except rospy.ROSInterruptException, ex:
        pass
