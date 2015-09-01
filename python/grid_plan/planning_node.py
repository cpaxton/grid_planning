
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

SKILL_TOPIC = "current_skill"

roscpp_set = False

def Sample(gmm):
    idx = int(np.floor((np.random.rand() * gmm.n_components)))
    A = np.linalg.cholesky(gmm.covars_[idx,:,:])
    ndim = gmm.means_.shape[1]

    n = np.random.normal(0,1,ndim)
    
    # cholesky returns the transpose of the MATLAB function chol()
    return gmm.means_[idx,:] + A.dot(n)

def Update(Z,wts,params):
    print "Updating Z..."
    sum_wts = sum(wts)
    Z.means_ = np.zeros(Z.means_.shape)
    Z.covars_ = np.zeros(Z.covars_.shape)
    for wt,param in zip(wts,params):
        Z.means_[0,:] += np.array(param) * wt / sum_wts
    for wt,param in zip(wts,params):
        ar = np.array([param])
        diff = Z.means_[0,:] - ar
        #print "---"
        #print diff.T.dot(diff)
        Z.covars_[0,:,:] += wt / sum_wts * diff.T.dot(diff)

    Z.covars_[0,:,:] += 0.00001 * np.eye(Z.covars_.shape[1])

    print Z.means_
    print Z.covars_

    return Z

class PyPlanner:

    def __init__(self,
            robot_description="robot_description",
            joint_states_topic="/gazebo/barrett_manager/wam/joint_states",
            planning_scene_topic="/gazebo/planning_scene",
            gripper_topic='/gazebo/barrett_manager/hand/cmd',
            skill_topic=SKILL_TOPIC,
            preset="wam_sim"):

        global roscpp_set
        if not roscpp_set:
            roscpp_init('grid_planning_node_cpp',[])


        if preset == "wam_sim":
            self.base_link = 'wam/base_link'
            self.end_link = 'wam/wrist_palm_link'
            robot_description="robot_description"
            joint_states_topic="/gazebo/barrett_manager/wam/joint_states"
            planning_scene_topic="/gazebo/planning_scene"
            gripper_topic="/gazebo/barrett_manager/hand/cmd"
            self.command_topic="/gazebo/traj_rml/joint_traj_cmd"
            dof = 7

        elif preset == "ur5_sim":
            self.base_link = '/base_link'
            self.end_link = '/ee_link'
            robot_description="robot_description"
            joint_states_topic="/joint_states"
            planning_scene_topic="/gazebo/planning_scene"
            self.command_topic='/arm_controller/command'
            dof = 6

        self.gripper_topic = gripper_topic
        self.skill_topic = skill_topic
        self.planning_scene_topic = planning_scene_topic
        self.robot = grid.RobotFeatures(
                base_link=self.base_link,
                end_link=self.end_link,
                js_topic=joint_states_topic,
                gripper_topic=gripper_topic)
        self.gp = grid_plan.GridPlanner(
                robot_description,
                joint_states_topic,
                planning_scene_topic,
                0.0)
        self.gp.SetDof(dof);
        self.gp.SetNumBasisFunctions(5);
        self.gp.SetK(100);
        self.gp.SetD(20);
        self.gp.SetTau(2.0);
        self.gp.SetGoalThreshold(0.1);
        self.gp.SetVerbose(False);
        self.skill_pub = rospy.Publisher(SKILL_TOPIC,std_msgs.msg.String)

    '''
    take a skill and associated objects
    return a trajectory
    - objs is a mapping between tf frames and names
    - skill is a RobotSkill
    '''
    def plan(self,skill,objs,num_iter=20,tol=0.0001,num_valid=30,num_samples=2500,give_up=10):

        print "Planning skill '%s'..."%(skill.name)
        self.skill_pub.publish(skill.name)

        print " - adding objects: "

        self.robot.ResetIndices()
        for (obj,frame) in objs:
            print "    - %s = %s"%(obj,frame)
            self.robot.AddObject(obj,frame)

        obj_keys = copy.copy(skill.objs)
        if 'gripper' in obj_keys:
            obj_keys.remove('gripper')

        nvars = skill.action_model.covars_.shape[1]
        for i in range(skill.action_model.n_components):
            skill.action_model.covars_[i,:,:] += 0.00001*np.eye(nvars)
        if not skill.goal_model is None:
            nvars = skill.goal_model.covars_.shape[1]
            for i in range(skill.goal_model.n_components):
                skill.goal_model.covars_[i,:,:] += 0.00001*np.eye(nvars)
        #self.robot.traj_model = skill.action_model
        #self.robot.goal_model = skill.goal_model
        self.robot.ConfigureSkill(skill.action_model,skill.goal_model)

        print " - getting TF information for generating trajectories..."

        world = None
        while world == None or not self.robot.TfUpdateWorld():
            world = self.robot.TfCreateWorld()

        print world
        
        Z = copy.deepcopy(skill.trajectory_model)
        for i in range(Z.n_components):
            Z.covars_[i,:,:] += 0.000001 * np.eye(Z.covars_.shape[1])
            for j in range(7):
                Z.covars_[i,j,j] += 0.1

        params = [0]*num_valid #Z.sample(num_samples)
        valid = []
        elite = []
        lls = np.zeros(num_valid)
        count = 0
        j = 0

        while len(valid) < num_valid and j < num_samples:
            traj_params = Sample(Z)
            #print traj_params[j]
            traj_ = self.gp.TryPrimitives(list(traj_params))

            if not len(traj_) == 0:
                valid.append(traj_params)
                pts = [p for p,v in traj_]

                p_z = Z.score(traj_params)[0]
                ll,wts = self.robot.GetTrajectoryWeight(pts,world,obj_keys,p_z)
                #ll = self.robot.GetTrajectoryLikelihood(pts,world,objs=skill.objs)
                lls[count] = ll
                params[count] = traj_params
                count += 1

            j+=1

        last_avg = np.mean(lls)
        ll_threshold = np.percentile(lls,80)
        for (ll,z) in zip(lls,valid):
            if ll >= ll_threshold:
                elite.append(z)

        #print wts

        skipped = 0
        for i in range(1,num_iter):
            print "Iteration %d... (based on %d valid samples)"%(i,count)
            Z = Z.fit(elite)
            #Z = Update(Z,lls,params)
            valid = []
            trajs = []
            lls = np.zeros(num_valid)
            count = 0
            j = 0
            #for z in traj_params:
            while len(valid) < num_valid and j < num_samples:
                traj_params = Sample(Z)
                traj_ = self.gp.TryPrimitives(list(traj_params))

                if not len(traj_) == 0:
                    valid.append(traj_params)
                    pts = [p for p,v in traj_]

                    p_z = 0 #Z.score(traj_params)[0]
                    ll,wts = self.robot.GetTrajectoryWeight(pts,world,obj_keys,p_z)
                    #ll = self.robot.GetTrajectoryLikelihood(pts,world,objs=skill.objs)
                    lls[count] = ll
                    params[count] = traj_params
                    trajs.append(traj_)
                    count += 1

                j += 1

            #print wts

            cur_avg = np.mean(lls)

            if cur_avg < last_avg:
                print "skipping: %g < %g"%(cur_avg, last_avg)
                skipped += 1
                if skipped >= give_up:
                    print "Stuck after %d skipped; let's just give up."%(skipped)
                    break
                else:
                    continue
            elif np.abs(cur_avg - last_avg) <= tol*last_avg and i > 5:
                print "Done! %g - %g = %g"%(cur_avg, last_avg, np.abs(cur_avg - last_avg))
                break

            skipped = 0
            last_avg = cur_avg
            elite = []
            ll_threshold = np.percentile(lls,92)
            for (ll,z) in zip(lls,valid):
                if ll >= ll_threshold:
                    elite.append(z)


            print "... avg ll = %f, percentile = %f"%(cur_avg,ll_threshold)

        traj = trajs[lls.tolist().index(np.max(lls))]

        print "Found %d total valid trajectories."%(count)

        cmd = JointTrajectory()
        msg = PoseArray()
        msg.header.frame_id = self.base_link
        for (pt,vel) in traj:
            cmd_pt = JointTrajectoryPoint()
            cmd_pt.positions = pt
            cmd_pt.velocities = np.array(vel)*0.1
            #cmd_pt.time_from_start = rospy.Duration.from_sec(t)
            cmd.points.append(cmd_pt)
            f = self.robot.GetForward(pt[:7])
            msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))

        if len(cmd.points) > 0:
            cmd.points[-1].velocities = [0]*7

        return cmd,msg,traj,Z
