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


    data,params,num_weights = LoadDataDMP(filenames)

    print "Fitting GMM to trajectory parameters..."
    #Z = GMM(dim=len(params[0]),ncomps=2,data=np.array(params),method="kmeans")
    Z = GMM()
    Z = Z.fit(params)

    print "Fitting GMM to expert features..."
    training_data = [data[0][1][-1]]
    for i in range(1,len(data)):
        #training_data = np.concatenate((training_data,data[i][1]))
        training_data += [data[i][1][-1]]

    expert = GMM(n_components=1,covariance_type="full")
    expert = expert.fit(training_data)
    print expert.covars_
    print expert.covars_.shape
    #expert = GMM(dim=training_data.shape[1],ncomps=5,data=training_data,method="kmeans")

    sub = rospy.Subscriber('/gazebo/barrett_manager/wam/joint_states',sensor_msgs.msg.JointState,js_cb)

    rate = rospy.Rate(10)

    RequestActiveDMP(data[0][2].dmp_list)

    print "waiting for joint states"
    while pos == None:
        rate.sleep()

    x = data[0][0].GetTrajectory()
    dims = data[0][0].Dims()
    #x0 = x[0] #pos
    #xdot0 = [0]*dims #vel
    x0 = pos
    xdot0 = vel
    t0 = 0
    xf = x[-1]
    threshold = [0.1] * dims
    tau = data[0][2].tau
    int_iter = 5
    seg_length = -1
    dt = 0.1

    plan = PlanDMP(x0,xdot0,t0,xf,threshold,seg_length,tau,dt,int_iter)
    #print plan

    cmd = JointTrajectory()
    cmd.header.seq = 0
    for pt,t in zip(plan.plan.points,plan.plan.times):
        cmd_pt = JointTrajectoryPoint()
        cmd_pt.positions = pt.positions[0:7]
        cmd_pt.velocities = pt.velocities[0:7]
        #cmd_pt.time_from_start = rospy.Duration(t*5)
        cmd_pt.time_from_start = rospy.Duration(0)

        cmd.points.append(cmd_pt)

    print "instantiating a new robot..."

    robot = RobotFeatures()
    obj1='/gbeam_link_1/gbeam_link'
    obj2='/gbeam_node_1/gbeam_node'
    robot.AddObject("link",obj1);
    robot.AddObject("node",obj2);
    base_link = 'wam/base_link'
    end_link = 'wam/wrist_palm_link'

    msg = PoseArray()
    for pt in cmd.points:
        f = robot.GetForward(pt.positions)
        msg.poses.append(pm.toMsg(f))

    msg2 = PoseArray()
    for pt in x:
        f = robot.GetForward(pt[0:7])
        msg2.poses.append(pm.toMsg(f))

    print "getting TF information for generating trajectories..."

    world = None
    while world == None:
        world = robot.TfCreateWorld()
    robot.TfUpdateWorld()

    print world

    print "setting feature model..."

    #robot.SetFeatureModel(expert)
    #robot.feature_model = expert
    robot.goal_model = expert

    print "generating new trajectories..."

    (lls,search_trajs,search_params) = grid.SearchDMP(Z,robot,world, # traj distribution
            x0,xdot0,t0,threshold,seg_length,tau,dt,int_iter, # DMP setup
            dmp=data[0][2].dmp_list, # dmp initialization
            ll_percentile=90,
            num_weights=num_weights,
            num_samples=100)

    for i in range(3):
        Z = Z.fit(search_params)
        (lls,search_trajs,search_params) = grid.SearchDMP(Z,robot,world, # traj distribution
                x0,xdot0,t0,threshold,seg_length,tau,dt,int_iter, # DMP setup
                dmp=data[0][2].dmp_list, # dmp initialization
                ll_percentile=90,
                num_weights=num_weights,
                num_samples=100)

    Z.fit(search_params)

    '''
    if len(search_params) > 0:
        # relearn Z
        Z.fit(search_params)
        print "Refitting and resampling..."
        dmps = Z.sample(100)

        search = MarkerArray()
        search_trajs = []
        search_params = []
        lls = []
        count = 1
        for i in range(50):
            dmp2 = copy.deepcopy(dmp)
            (goal,dmp2) = grid.ParamToDMP(dmps[i],dmp,num_weights=num_weights)

            RequestActiveDMP(dmp2)

            plan = PlanDMP(x0,xdot0,t0,goal,threshold,seg_length,tau,dt,int_iter)

            #ll = robot.GetTrajectoryLikelihood(plan.plan.points,world)
            #print ll

            traj = [pt.positions[:7] for pt in plan.plan.points]
            ll = robot.GetTrajectoryLikelihood(traj,world,(3,17))

            wt = np.exp(ll)

            lls.append(ll)
            if wt > 1e-50:
                search_trajs.append(traj)
                search_params.append(grid.ParamFromDMP(goal,dmp2))

                for pt in plan.plan.points:
                    count += 1
                    f = robot.GetForward(pt.positions[:7])
                    msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))

                    if count % 5 == 0 and count < 500:
                        marker = GetMarkerMsg(robot,pt.positions[:7],wt,len(search.markers))
                        search.markers.append(marker)

        print "... done with DMPs round 2."

        print "Average goal probability: %f"%(np.mean(lls))
        print "Found %d with p>%f."%(len(search_trajs),1e-50)
    '''

    search = MarkerArray()
    count = 1
    for (ll,traj) in zip(lls,search_trajs):
        wt = np.exp(ll)
        for pt in traj:
            count += 1
            f = robot.GetForward(pt[:7])
            msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))

            if count % 5 == 0 and count <= 1000:
                marker = GetMarkerMsg(robot,pt[:7],wt,len(search.markers))
                search.markers.append(marker)


    print "Showing trajectories now."

    msg.header.frame_id = base_link
    pa_ee_pub = rospy.Publisher('/dbg_ee',PoseArray)

    msg2.header.frame_id = base_link
    pa_ee2_pub = rospy.Publisher('/dbg_link',PoseArray)

    pa_ee_pub.publish(msg)
    pa_ee2_pub.publish(msg2)

    pub = rospy.Publisher('/gazebo/traj_rml/joint_traj_cmd',JointTrajectory)
    pub.publish(cmd)

    pub2 = rospy.Publisher('/search_trajectory',MarkerArray)

    try:
        while not rospy.is_shutdown():
            pa_ee_pub.publish(msg)
            pa_ee2_pub.publish(msg2)
            pub2.publish(search)
            rate.sleep()
    except rospy.ROSInterruptException, ex:
        pass

