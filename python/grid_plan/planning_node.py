
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
import std_msgs

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

NUM_VALID = 50
NUM_SAMPLES = 2500

SKILL_TOPIC = "current_skill"

roscpp_set = False

class PyPlanner:

    def __init__(self,
            robot_description="robot_description",
            joint_states_topic="/gazebo/barrett_manager/wam/joint_states",
            planning_scene_topic="/gazebo/planning_scene",
            preset="wam"):

        global roscpp_set
        if not roscpp_set:
            roscpp_init('grid_planning_node_cpp',[])

        self.gp = grid_plan.GridPlanner(
                robot_description,
                joint_states_topic,
                planning_scene_topic,
                0.0)
        self.gp.SetDof(7);
        self.gp.SetNumBasisFunctions(5);
        self.gp.SetK(100);
        self.gp.SetD(20);
        self.gp.SetTau(2.0);
        self.gp.SetGoalThreshold(0.1);
        self.gp.SetVerbose(False);
        if preset == "wam":
            self.robot = grid.RobotFeatures()
            self.base_link = 'wam/base_link'
            self.end_link = 'wam/wrist_palm_link'

        self.skill_pub = rospy.Publisher(SKILL_TOPIC,std_msgs.msg.String)

    '''
    take a skill and associated objects
    return a trajectory
    - objs is a mapping between tf frames and names
    - skill is a RobotSkill
    '''
    def plan(self,skill,objs,num_iter=20):

        print "Planning skill '%s'..."%(skill.name)
        self.skill_pub.publish(skill.name)

        print " - adding objects: "

        for (obj,frame) in objs:
            print "    - %s = %s"%(obj,frame)
            self.robot.AddObject(obj,frame)

        self.robot.traj_model = skill.action_model
        self.robot.goal_model = skill.goal_model

        print " - getting TF information for generating trajectories..."

        world = None
        while world == None or not self.robot.TfUpdateWorld():
            world = self.robot.TfCreateWorld()

        print world

        
        Z = copy.deepcopy(skill.trajectory_model)
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

        while len(valid) < NUM_VALID and j < NUM_SAMPLES:
            traj_ = self.gp.TryPrimitives(list(traj_params[j]))

            if not len(traj_) == 0:
                count += 1
                valid.append(traj_params[j])
                pts = [p for p,v in traj_]
                #p_z = np.exp(Z.score(traj_params[j]))
                p_z = Z.score(traj_params[j])[0]
                #ll = np.mean(self.robot.GetTrajectoryWeight(pts,world,skill.objs,p_z))
                wts = self.robot.GetTrajectoryWeight(pts,world,skill.objs,p_z)
                print wts
                ll = np.mean(wts)
                #ll = self.robot.GetTrajectoryLikelihood(pts,world,objs=skill.objs)
                lls[len(valid)-1] = ll

            j+=1

        ll_threshold = np.percentile(lls,80)
        for (ll,z) in zip(lls,valid):
            if ll >= ll_threshold:
                elite.append(z)

        for i in range(1,20):
            print "Iteration %d... (based on %d valid samples)"%(i,count)
            Z = Z.fit(elite)
            Z.covars_[0,:,:] += 0.00001 * np.eye(Z.covars_.shape[1])
            traj_params = Z.sample(NUM_SAMPLES)
            valid = []
            elite = []
            trajs = []
            wts = []
            lls = np.zeros(NUM_VALID)
            count = 0
            j = 0
            #for z in traj_params:
            while len(valid) < NUM_VALID and j < NUM_SAMPLES:
                traj_ = self.gp.TryPrimitives(list(traj_params[j]))

                if not len(traj_) == 0:
                    count += 1
                    valid.append(traj_params[j])
                    pts = [p for p,v in traj_]
                    #p_z = np.exp(Z.score(traj_params[j]))
                    p_z = Z.score_samples(traj_params[j])[0]
                    wts = self.robot.GetTrajectoryWeight(pts,world,skill.objs,p_z)
                    ll = np.mean(wts)
                    #ll = self.robot.GetTrajectoryLikelihood(pts,world,objs=skill.objs)
                    lls[len(valid)-1] = ll
                    trajs.append(traj_)

                j += 1

            ll_threshold = np.percentile(lls,97)
            for (ll,z) in zip(lls,valid):
                if ll >= ll_threshold:
                    elite.append(z)

            print "... avg ll = %f, percentile = %f"%(np.mean(lls),ll_threshold)

        traj = trajs[lls.tolist().index(np.max(lls))]

        print "Found %d total valid trajectories."%(count)

        cmd = JointTrajectory()
        msg = PoseArray()
        msg.header.frame_id = self.base_link
        pts = []
        vels = []
        for (pt,vel) in traj:
            cmd_pt = JointTrajectoryPoint()
            cmd_pt.positions = pt
            cmd_pt.velocities = np.array(vel)*0.02
            pts.append(pt)
            vels.append(vel)
            cmd.points.append(cmd_pt)
            f = self.robot.GetForward(pt[:7])
            msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))

        if len(cmd.points) > 0:
            cmd.points[-1].velocities = [0]*7

        return cmd,msg,traj,Z
