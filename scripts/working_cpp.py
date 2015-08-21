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

NUM_VALID = 50
NUM_SAMPLES = 2500

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

robot.goal_model = approach.goal_model
robot.traj_model = approach.action_model

print " - getting TF information for generating trajectories..."

world = None
while world == None or not robot.TfUpdateWorld():
    world = robot.TfCreateWorld()

print world

""" ========================================================================= """

print "Starting search:"

rospy.wait_for_service('/gazebo/publish_planning_scene')
pps = rospy.ServiceProxy('/gazebo/publish_planning_scene',Empty)
pps()
rospy.sleep(rospy.Duration(0.1))
pps()

gp.PrintInfo();

Z = copy.deepcopy(approach.trajectory_model)
for i in range(Z.n_components):
    Z.covars_[i,:,:] += 0.00001 * np.eye(Z.covars_.shape[1])
    for j in range(7):
        Z.covars_[i,j,j] += 0.2

traj_params = Z.sample(NUM_SAMPLES)
valid = []
elite = []
lls = np.zeros(NUM_VALID)
count = 0
j = 0
#for z in traj_params:
while len(valid) < NUM_VALID and j < NUM_SAMPLES:
    traj_ = gp.TryPrimitives(list(traj_params[j]))

    if not len(traj_) == 0:
        count += 1
        valid.append(traj_params[j])
        pts = [p for p,v in traj_]
        ll = robot.GetTrajectoryLikelihood(pts,world,objs=['link'])
        lls[len(valid)-1] = ll

    j+=1

ll_threshold = np.percentile(lls,80)
for (ll,z) in zip(lls,valid):
    if ll >= ll_threshold:
        elite.append(z)

#elite = valid
for i in range(1,20):
    print "Iteration %d... (based on %d valid samples)"%(i,count)
    Z = Z.fit(elite)
    Z.covars_[0,:,:] += 0.00001 * np.eye(Z.covars_.shape[1])
    traj_params = Z.sample(NUM_SAMPLES)
    valid = []
    elite = []
    trajs = []
    lls = np.zeros(NUM_VALID)
    count = 0
    j = 0
    #for z in traj_params:
    while len(valid) < NUM_VALID and j < NUM_SAMPLES:
        traj_ = gp.TryPrimitives(list(traj_params[j]))

        if not len(traj_) == 0:
            count += 1
            valid.append(traj_params[j])
            pts = [p for p,v in traj_]
            ll = robot.GetTrajectoryLikelihood(pts,world,objs=['link'])
            lls[len(valid)-1] = ll
            trajs.append(traj_)

        j += 1

    ll_threshold = np.percentile(lls,97)
    for (ll,z) in zip(lls,valid):
        if ll >= ll_threshold:
            elite.append(z)

    print "... avg ll = %f, percentile = %f"%(np.mean(lls),ll_threshold)

print lls

traj = trajs[lls.tolist().index(np.max(lls))]

print "Found %d total valid trajectories."%(count)

cmd = JointTrajectory()
msg = PoseArray()
msg.header.frame_id = base_link
pts = []
vels = []
for (pt,vel) in traj:
    cmd_pt = JointTrajectoryPoint()
    cmd_pt.positions = pt
    cmd_pt.velocities = np.array(vel)*0.02
    pts.append(pt)
    vels.append(vel)
    cmd.points.append(cmd_pt)
    f = robot.GetForward(pt[:7])
    msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))

if len(cmd.points) > 0:
    cmd.points[-1].velocities = [0]*7

pub = rospy.Publisher('/gazebo/traj_rml/joint_traj_cmd',JointTrajectory)
pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)

#print traj
print traj[-1][0]

rospy.sleep(rospy.Duration(0.5))
pub.publish(cmd)
pa_ee_pub.publish(msg)

# plot desired position and velocity
if False:
    from matplotlib import pyplot as plt
    plt.figure(1)
    plt.plot(pts)
    plt.figure(2)
    plt.plot(vels)
    plt.show()

try:
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        pa_ee_pub.publish(msg)
        rate.sleep()
except rospy.ROSInterruptException, ex:
    pass

