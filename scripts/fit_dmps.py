#!/usr/bin/env python

import grid

" IO "
import sys
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper

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

    #stream = file(filename,'r');
    #demo = yaml.load(stream,Loader=Loader);
    demo = grid.LoadYaml(filename)

    sub = rospy.Subscriber('/gazebo/barrett_manager/wam/joint_states',sensor_msgs.msg.JointState,js_cb)

    rate = rospy.Rate(10)

    print "Loaded data, computing features..."
    #fx,x,u,t = demo.get_features([('ee','link'),('ee','node'),('link','node')])
    fx,x,u,t = demo.get_features([('ee','link')])

    # DMP parameters
    dims = len(u[0])
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

    x0 = x[0] #pos
    xdot0 = [0]*dims #vel
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
        cmd_pt.positions = pt.positions
        cmd_pt.velocities = pt.velocities
        #cmd_pt.time_from_start = rospy.Duration(t)
        cmd_pt.time_from_start = rospy.Duration(0)

        cmd.points.append(cmd_pt)

    base_link = 'wam/base_link'
    end_link = 'wam/wrist_palm_link'
    robot = URDF.from_parameter_server()
    tree = kdl_tree_from_urdf_model(robot)
    chain = tree.getChain(base_link, end_link)
    kdl_kin = KDLKinematics(robot, base_link, end_link)

    msg = PoseArray()
    for pt in plan.plan.points:
    #for pt in x:
        mat = kdl_kin.forward(pt.positions)
        #mat = kdl_kin.forward(pt)
        f = pm.fromMatrix(mat)
        msg.poses.append(pm.toMsg(f))

    msg2 = PoseArray()
    for pt in x:
        mat = kdl_kin.forward(pt)
        f = pm.fromMatrix(mat)
        msg2.poses.append(pm.toMsg(f))

    msg.header.frame_id = base_link
    pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)
    pa_ee_pub.publish(msg)

    msg2.header.frame_id = base_link
    pa_ee2_pub = rospy.Publisher('/dbg_link',PoseArray)
    pa_ee2_pub.publish(msg2)

    pub = rospy.Publisher('/gazebo/traj_rml/joint_traj_cmd',JointTrajectory)
    pub.publish(cmd)

    while not rospy.is_shutdown():
        pa_ee_pub.publish(msg)
        pa_ee_pub.publish(msg2)
    
