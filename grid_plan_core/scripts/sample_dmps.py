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

" machine learning "
from sklearn.mixture import GMM
#from gmm import GMM # conditional GMM

" Fitting DMPs "
from dmp.srv import *
from dmp.msg import *
from grid import *

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
    rospy.init_node('sample_trajectories')

    if len(sys.argv) > 1:
        filenames = sys.argv[1:]
    else:
        filenames = ["app1.yml","app2.yml"]


    data,params,num_weights,goals = LoadDataDMP(filenames,['link'])

    print "Fitting GMM to trajectory parameters..."
    Z = GMM(covariance_type="full")
    Z.n_components = 1
    Z = Z.fit(params)

    print "Fitting GMM to expert action features..."
    training_data = np.array(data[0][1])
    for i in range(1,len(data)):
        training_data = np.concatenate((training_data,data[i][1]))
    traj_model = GMM(n_components=1,covariance_type="full")
    traj_model.fit(training_data)

    print "Fitting GMM to expert goal features..."
    expert = GMM(n_components=1,covariance_type="full")
    expert = expert.fit(goals)

    print "Initializing RobotFeatures listener:"
    sub = rospy.Subscriber('/gazebo/barrett_manager/wam/joint_states',sensor_msgs.msg.JointState,js_cb)
    rate = rospy.Rate(10)
    RequestActiveDMP(data[0][2].dmp_list)

    print " - waiting for joint states"
    while pos == None:
        rate.sleep()

    x = data[0][0].GetTrajectory()
    dims = data[0][0].Dims()
    x0 = pos
    xdot0 = vel
    t0 = 0
    xf = x[-1]
    threshold = [0.1] * dims
    tau = data[0][2].tau
    int_iter = 5
    seg_length = -1
    dt = 0.1

    print " - instantiating a new robot..."

    robot = RobotFeatures()
    obj1='/gbeam_link_1/gbeam_link'
    obj2='/gbeam_node_1/gbeam_node'
    robot.AddObject("link",obj1);
    robot.AddObject("node",obj2);
    base_link = 'wam/base_link'
    end_link = 'wam/wrist_palm_link'

    print " - getting TF information for generating trajectories..."

    world = None
    while world == None or not robot.TfUpdateWorld():
        world = robot.TfCreateWorld()

    world.pop('node')
    print world

    print " - setting feature models..."
    robot.goal_model = expert
    robot.traj_model = traj_model

    print "Generating new trajectories..."

    for i in range(Z.n_components):
        Z.covars_[i,:,:] += 1 * np.eye(Z.covars_.shape[1])
    (lls,search_lls,search_trajs,search_params,all_trajs) = grid.SearchDMP(
            Z,robot,world, # traj distribution, robot, and world
            x0,xdot0,t0,threshold,seg_length,tau,dt,int_iter, # DMP setup
            dmp=data[0][2].dmp_list, # dmp initialization
            ll_percentile=98,
            num_weights=num_weights,
            num_samples=500)

    for i in range(10):
        Z = Z.fit(search_params)
        Z.covars_[0,:,:] += 0.000001 * np.eye(Z.covars_.shape[1])
        #for i in range(Z.n_components):
        #    Z.covars_[i,:,:] += 1e-2 * np.eye(Z.covars_.shape[1])
        (lls,search_lls,search_trajs,search_params,all_trajs) = grid.SearchDMP(
                Z,robot,world, # traj distribution, robot, and world
                x0,xdot0,t0,threshold,seg_length,tau,dt,int_iter, # DMP setup
                dmp=data[0][2].dmp_list, # dmp initialization
                ll_percentile=98,
                num_weights=num_weights,
                num_samples=500)

    Z = Z.fit(search_params)

    print "sending best trajectory command..."

    best_ll = np.max(search_lls)
    best_idx = search_lls.index(best_ll)
    traj = search_trajs[best_idx]

    cmd = JointTrajectory()
    for pt in traj:
        cmd_pt = JointTrajectoryPoint()
        cmd_pt.positions = pt
        cmd.points.append(cmd_pt)

    cmd_pt = JointTrajectoryPoint()
    cmd_pt.positions = traj[-1]

    pub = rospy.Publisher('/gazebo/traj_rml/joint_traj_cmd',JointTrajectory)
    pubpt = rospy.Publisher('/gazebo/traj_rml/joint_traj_point_cmd',JointTrajectoryPoint)
    rospy.sleep(rospy.Duration(0.25))

    print "publishing point!"
    pub.publish(cmd)

    print "creating trajectory messages..."

    search = MarkerArray()
    count = 1
    for (ll,traj) in zip(lls,search_trajs):
        wt = np.exp(ll)
        for pt in traj:
            count += 1
            #f = robot.GetForward(pt[:7])
            #msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))

            if count % 10 == 0:
                marker = GetMarkerMsg(robot,pt[:7],wt,len(search.markers))
                search.markers.append(marker)
        """
        marker = GetMarkerMsg(robot,traj[0][:7],wt,len(search.markers))
        search.markers.append(marker)
        marker = GetMarkerMsg(robot,traj[-1][:7],wt,len(search.markers))
        search.markers.append(marker)
        """

    msg = PoseArray()
    for traj in all_trajs:
        """
        for pt in traj:
            f = robot.GetForward(pt[:7])
            msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))
        """
        f = robot.GetForward(traj[0][:7])
        msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))
        f = robot.GetForward(traj[-1][:7])
        msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))

    print "Showing trajectories now."

    msg.header.frame_id = base_link
    pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)
    pa_pub = rospy.Publisher('/dbg_ee_link',PoseArray)

    pub2 = rospy.Publisher('/search_trajectory',MarkerArray)

    try:
        while not rospy.is_shutdown():
            pa_ee_pub.publish(msg)
            #pa_pub.publish(dbg_ee_poses)
            pub2.publish(search)
            rate.sleep()
    except rospy.ROSInterruptException, ex:
        pass

