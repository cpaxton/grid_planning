#!/usr/bin/env python

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

" math "
import numpy as np
import tf_conversions.posemath as pm

" Fitting DMPs "
from dmp.srv import *
from dmp.msg import *
from grid import *

" control "
from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint

" kdl utilities "
from urdf_parser_py.urdf import URDF
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

'''
fit_dmps
This function fits a set of dynamic movement primitives and saves them to a file.
Based off of the DMP code from the ROS wiki.

DMPs are saved for each individual motion, so this computes DMPs and features for each DMP point.
'''

pos = None
vel = None

def js_cb(msg):
    global pos
    global vel
    pos = msg.position
    vel = msg.velocity
    

if __name__ == '__main__':
    rospy.init_node('demonstration_fitting')

    if len(sys.argv) > 1:
        filename = sys.argv[1]
    else:
        filename = "app1.yml"

    demo = grid.LoadRobotFeatures(filename)

    sub = rospy.Subscriber('/gazebo/barrett_manager/wam/joint_states',sensor_msgs.msg.JointState,js_cb)

    rate = rospy.Rate(10)

    print "Loaded data, computing features..."
    #fx,x,u,t = demo.get_features([('ee','link'),('ee','node'),('link','node')])
    fx = demo.GetTrainingFeatures()
    x = demo.GetJointPositions()

    # DMP parameters
    dims = len(x[0])
    dt = 0.1
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 4

    resp = RequestDMP(x,0.1,K,D,5)
    dmp = resp.dmp_list
    print dmp
    
    RequestActiveDMP(dmp)

    print "waiting for joint states"
    while pos == None:
        rate.sleep()

    #x0 = x[0] #pos
    #xdot0 = [0]*dims #vel
    x0 = pos
    xdot0 = vel
    t0 = 0
    xf = x[-1]
    threshold = [0.1] * dims
    tau = resp.tau
    int_iter = 5
    seg_length = -1
    dt = 0.1

    plan = PlanDMP(x0,xdot0,t0,xf,threshold,seg_length,tau,dt,int_iter)
    print plan

    cmd = JointTrajectory()
    cmd.header.seq = 0
    for pt,t in zip(plan.plan.points,plan.plan.times):
        cmd_pt = JointTrajectoryPoint()
        cmd_pt.positions = pt.positions[0:7]
        cmd_pt.velocities = pt.velocities[0:7]
        #cmd_pt.time_from_start = rospy.Duration(t*5)
        cmd_pt.time_from_start = rospy.Duration(0)

        cmd.points.append(cmd_pt)

    base_link = 'wam/base_link'
    end_link = 'wam/wrist_palm_link'
    robot = URDF.from_parameter_server()
    tree = kdl_tree_from_urdf_model(robot)
    chain = tree.getChain(base_link, end_link)
    kdl_kin = KDLKinematics(robot, base_link, end_link)

    msg = PoseArray()
    for pt in cmd.points:
    #for pt in x:
        mat = kdl_kin.forward(pt.positions)
        #mat = kdl_kin.forward(pt)
        f = pm.fromMatrix(mat)
        msg.poses.append(pm.toMsg(f))

    msg2 = PoseArray()
    for pt in x:
        mat = kdl_kin.forward(pt[0:7])
        f = pm.fromMatrix(mat)
        msg2.poses.append(pm.toMsg(f))

    for i in range(15):
        dmp2 = copy.deepcopy(dmp)
        for primitive in dmp2[:7]:
            wts = list(primitive.weights)
            for j in range(len(wts)):
                wts[j] += np.random.normal(0,0.25)
            primitive.weights = tuple(wts)
        RequestActiveDMP(dmp2)
        plan = PlanDMP(x0,xdot0,t0,xf,threshold,seg_length,tau,dt,int_iter)
        for pt in plan.plan.points:
            mat = kdl_kin.forward(pt.positions[:7])
            f = pm.fromMatrix(mat)
            msg.poses.append(pm.toMsg(f))

    msg.header.frame_id = base_link
    pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)

    msg2.header.frame_id = base_link
    pa_ee2_pub = rospy.Publisher('/dbg_link',PoseArray)

    pa_ee_pub.publish(msg)
    pa_ee2_pub.publish(msg2)

    pub = rospy.Publisher('/gazebo/traj_rml/joint_traj_cmd',JointTrajectory)
    pub.publish(cmd)

    try:
        while not rospy.is_shutdown():
            pa_ee_pub.publish(msg)
            pa_ee2_pub.publish(msg2)
            rate.sleep()
    except rospy.ROSInterruptException, ex:
        pass

