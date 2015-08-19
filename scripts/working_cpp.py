#!/usr/bin/env python

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

filenames = ["app1.yml","app2.yml","app3.yml"]

roscpp_init('ipython', [])
rospy.init_node('ipython2')

""" ========================================================================= """
gp = grid_plan.GridPlanner('robot_description',
        '/gazebo/barrett_manager/wam/joint_states',
        '/gazebo/planning_scene',0.0)
gp.SetDof(7);
gp.SetNumBasisFunctions(5);
gp.SetK(100);
gp.SetD(20);
gp.SetTau(2.0);
gp.SetGoalThreshold(0.1);
gp.SetVerbose(False);

""" ========================================================================= """
robot = grid.RobotFeatures()
obj1='/gbeam_link_1/gbeam_link'
obj2='/gbeam_node_1/gbeam_node'
robot.AddObject("link",obj1);
robot.AddObject("node",obj2);
base_link = 'wam/base_link'
end_link = 'wam/wrist_palm_link'

approach = grid.RobotSkill(filename='skills/approach_skill.yml')
transport = grid.RobotSkill(filename='skills/transport_skill.yml')
grasp = grid.RobotSkill(filename='skills/grasp_skill.yml')

""" ========================================================================= """

rospy.wait_for_service('/gazebo/publish_planning_scene')
pps = rospy.ServiceProxy('/gazebo/publish_planning_scene',Empty)
pps()
rospy.sleep(rospy.Duration(0.1))
pps()

Z = copy.deepcopy(approach.trajectory_model)
for i in range(Z.n_components):
    Z.covars_[i,:,:] += 0.00001 * np.eye(Z.covars_.shape[1])
    for j in range(7):
        Z.covars_[i,j,j] += 0.2

traj_params = Z.sample(2500)
traj = []
valid = []
count = 0
j = 0
#for z in traj_params:
while len(valid) < 10 and j < 2500:
    traj_ = gp.TryPrimitives(list(traj_params[j]))

    if not len(traj_) == 0:
        count += 1
        traj = traj_
        valid.append(traj_params[j])

    j += 1

for i in range(1,5):
    print "Iteration %d... (based on %d valid samples)"%(i,count)
    Z = Z.fit(valid)
    Z.covars_[0,:,:] += 0.000001 * np.eye(Z.covars_.shape[1])
    traj_params = Z.sample(2500)
    valid = []
    count = 0
    j = 0
    #for z in traj_params:
    while len(valid) < 10 and j < 2500:
        traj_ = gp.TryPrimitives(list(traj_params[j]))

        if not len(traj_) == 0:
            count += 1
            traj = traj_
            valid.append(traj_params[j])

        j += 1

print "Found %d total valid trajectories."%(count)

cmd = JointTrajectory()
msg = PoseArray()
msg.header.frame_id = base_link
pts = []
vels = []
for (pt,vel) in traj:
    cmd_pt = JointTrajectoryPoint()
    cmd_pt.positions = pt
    cmd_pt.velocities = np.array(vel)*0.05
    pts.append(pt)
    vels.append(vel)
    cmd.points.append(cmd_pt)
    f = robot.GetForward(pt[:7])
    msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))\

if len(cmd.points) > 0:
    cmd.points[-1].velocities = [0]*7

pub = rospy.Publisher('/gazebo/traj_rml/joint_traj_cmd',JointTrajectory)
pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)

if False:
    from matplotlib import pyplot as plt
    plt.figure(1)
    plt.plot(pts)
    plt.figure(2)
    plt.plot(vels)
    plt.show()

# plot desired position and velocity

rospy.sleep(rospy.Duration(0.25))
pub.publish(cmd)
pa_ee_pub.publish(msg)

try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pa_ee_pub.publish(msg)
        rate.sleep()
except rospy.ROSInterruptException, ex:
    pass

