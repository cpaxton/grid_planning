
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
import time

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
MSG_TOPIC = "search_trajectories"

roscpp_set = False

def Sample(gmm):
    idx = int(np.floor((np.random.rand() * gmm.n_components)))
    A = np.linalg.cholesky(gmm.covars_[idx,:,:])
    ndim = gmm.means_.shape[1]

    n = np.random.normal(0,1,ndim)
    
    # cholesky returns the transpose of the MATLAB function chol()
    return gmm.means_[idx,:] + A.dot(n)

def Update(Z,wts,params,step_size):
    print "Updating Z..."
    if np.sum(wts) == 0:
        wts = np.ones(len(wts)) / len(wts)
        print "ERR! No plausible locations to go!"
        return Z.means_, Z.covars_
    else:
        wts /= np.sum(wts)
    n_means_ = np.zeros(Z.means_.shape)
    n_covars_ = np.zeros(Z.covars_.shape)
    for wt,param in zip(wts,params):
        n_means_[0,:] += np.array(param) * wt
    for wt,param in zip(wts,params):
        ar = np.array([param])
        diff = Z.means_[0,:] - ar
        n_covars_[0,:,:] += wt * diff.T.dot(diff)

    print Z.means_
    print step_size
    print n_means_

    means_ = ((1 - step_size)*Z.means_) + (step_size * n_means_)
    covars_ = ((1 - step_size)*Z.covars_) + (step_size * n_covars_)

    #Z.means_ = means_
    #Z.covars_ = covars_

    #Z.covars_[0,:,:] += 0.00001 * np.eye(Z.covars_.shape[1])

    return means_,covars_

class PyPlanner:

    def __init__(self,
            robot_description="robot_description",
            joint_states_topic="/gazebo/barrett_manager/wam/joint_states",
            planning_scene_topic="/gazebo/planning_scene",
            gripper_topic='/gazebo/barrett_manager/hand/cmd',
            skill_topic=SKILL_TOPIC,
            preset="wam7_sim"):

        global roscpp_set
        if not roscpp_set:
            roscpp_init('grid_planning_node_cpp',[])


        if preset == "wam7_sim":
            self.base_link = 'wam/base_link'
            self.end_link = 'wam/wrist_palm_link'
            robot_description="robot_description"
            joint_states_topic="/gazebo/barrett_manager/wam/joint_states"
            planning_scene_topic="/gazebo/planning_scene"
            gripper_topic="/gazebo/barrett_manager/hand/cmd"
            self.command_topic="/gazebo/traj_rml/joint_traj_cmd"
            dof = 7

        elif preset == "ur5":
            self.base_link = '/base_link'
            self.end_link = '/ee_link'
            robot_description="/robot_description"
            joint_states_topic="/joint_states"
            planning_scene_topic="/planning_scene"
            self.command_topic='/arm_controller/command'
            dof = 6

        self.traj_model = None
        self.gripper_topic = gripper_topic
        self.skill_topic = skill_topic
        self.planning_scene_topic = planning_scene_topic
        self.robot = grid.RobotFeatures(
                base_link=self.base_link,
                end_link=self.end_link,
                js_topic=joint_states_topic,
                gripper_topic=gripper_topic,
		dof=dof)
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
        self.msg_pub = rospy.Publisher(MSG_TOPIC,PoseArray)
        self.sample_pub = rospy.Publisher('samples',PoseArray)

    '''
    take a skill and associated objects
    return a trajectory
    - objs is a mapping between tf frames and names
    - skill is a RobotSkill
    '''
    def plan(self,skill,objs,num_iter=20,tol=0.0001,num_valid=30,num_samples=2500,give_up=10,step_size=0.5, guess_goal_x=[0,0,0],npts=5):

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

        print ' - configuring skill...'
        nvars = skill.action_model.covars_.shape[1]
        for i in range(skill.action_model.n_components):
            skill.action_model.covars_[i,:,:] += 0.00001*np.eye(nvars)
        #skill.action_model.covars_ *= 1
        if not skill.goal_model is None:
            nvars = skill.goal_model.covars_.shape[1]
            for i in range(skill.goal_model.n_components):
                skill.goal_model.covars_[i,:,:] += 0.00001*np.eye(nvars)
            #skill.goal_model.covars_ *= 1
        self.robot.ConfigureSkill(skill.action_model,skill.goal_model)

        print " - getting TF information for generating trajectories..."
        world = None
        while world == None or not self.robot.TfUpdateWorld():
            world = self.robot.TfCreateWorld()
        
        print " - setting up initial search distribution..."
        q = self.gp.GetJointPositions()
        ee = self.robot.GetForward(q)
        important_objs = [obj for (obj,frame) in objs if obj in skill.objs]
        print important_objs
        if guess_goal_x == None and len(important_objs) > 0:
            print "   ... found world object named %s"%(important_objs[0])
            fobj = (self.robot.base_tform * ee).Inverse() * world[important_objs[0]]
            guess_goal_x = [fobj.p.x(),fobj.p.y(),fobj.p.z()]
            #Z.means_[0,:self.robot.dof] = [fobj.p.x(),fobj.p.y(),fobj.p.z()] + [0,0,0,0];
        elif guess_goal_x == None:
            #Z.means_[0,:self.robot.dof] = [0,0,0,0,0,0,0];
            guess_goal_x = [0,0,0]
        #else:
        #    Z.means_[0,:self.robot.dof] = guess_goal_x + [0,0,0,0];

        #print Z.means_[0,:self.robot.dof]
        Z = grid_plan.InitSearch(npts,np.array(guess_goal_x))

        #Z.covars_[0] = 0.1*np.eye(Z.covars_.shape[1])
        #Z.covars_[0,:self.robot.dof,:self.robot.dof] = 0.1*np.eye(self.robot.dof)
        #self.gp.PrintInfo()

        params = [0]*num_valid #Z.sample(num_samples)
        lls = np.zeros(num_valid)
        wts = np.zeros(num_valid)
        count = 0
        last_avg = 0

        skipped = 0
        for i in range(0,num_iter):
            print "Iteration %d... (based on %d valid samples)"%(i,count)

            valid = []
            trajs = []
            search_pts = []

            count = 0
            j = 0
            #for z in traj_params:
            start = time.clock()
            '''
            smsg = PoseArray()
            smsg.header.frame_id = self.base_link
            '''
            while len(valid) < num_valid and j < num_samples:
                #traj_params,traj = Sample(Z)
                traj_params,traj = grid_plan.SamplePrimitives(ee,Z,self.robot.kdl_kin,q)
                #print traj

                #traj_ = self.gp.TryPrimitives(list(cpy_traj_params))
                traj_valid = not any([pt is None for pt in traj]) and self.gp.TryTrajectory(traj)

                '''
                for pt in traj:
                    if not pt is None:
                        smsg.poses.append(pm.toMsg(self.robot.GetForward(pt)))
                '''

                search_pts += traj
                
                #if not len(traj_) == 0:
                if traj_valid:
                    valid.append(traj_params)

                    p_z = Z.score(traj_params)[0]

                    #wt,p,_ = self.robot.GetTrajectoryWeight([p for p,v in traj],world,obj_keys,p_z)
                    wt,p = self.robot.GetTrajectoryWeight(traj,world,obj_keys,p_z)
                    lls[count] = p
                    wts[count] = wt
                    trajs.append(traj)
                    count += 1

                j += 1
            print "[SAMPLE] elapsed: %f"%(time.clock() - start)

            cur_avg = np.mean(lls)

            if cur_avg < last_avg:
                print "skipping: %g < %g"%(cur_avg, last_avg)
                skipped += 1
                if skipped >= give_up:
                    print "Stuck after %d skipped; let's just give up."%(skipped)
                    break
            elif np.abs(cur_avg - last_avg) <= tol and i > 5:
                print "Done! %g - %g = %g"%(cur_avg, last_avg, np.abs(cur_avg - last_avg))
                break
            else:

                skipped = 0
                last_avg = cur_avg
                elite = []
                elite_wts = []

                '''
                ll_threshold = np.percentile(lls,92)
                for (ll,z) in zip(lls,valid):
                    if ll >= ll_threshold:
                        elite.append(z)
                        elite_wts.append(ll)
                Z = Z.fit(elite)
                '''
                ll_threshold = np.percentile(wts,0) # was 92
                for (ll,z) in zip(lls,valid):
                    if ll >= ll_threshold:
                        elite.append(z)
                        elite_wts.append(ll)
                mu,sig = Update(Z,elite_wts,elite,step_size)
                Z.means_ = mu
                Z.covars_ = sig

                print mu

                print "... avg ll = %g, percentile = %g"%(cur_avg,ll_threshold)

            # send message
            msg = PoseArray()
            msg.header.frame_id = self.base_link
            #for (pt,vel) in search_pts:
            '''
            self.sample_pub.publish(smsg)
            '''
            for pt in search_pts:
                if not pt == None:
                    f = self.robot.GetForward(pt[:7])
                    msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))
            self.msg_pub.publish(msg)

            # PAUSE
            #print "\n\t\tpress [ENTER] to continue...\n"
            #raw_input();

        traj = trajs[lls.tolist().index(np.max(lls))]

        print "Found %d total valid trajectories."%(count)

        cmd = JointTrajectory()
        msg = PoseArray()
        msg.header.frame_id = self.base_link
        #for (pt,vel) in traj:
        for pt in traj:
            cmd_pt = JointTrajectoryPoint()
            cmd_pt.positions = pt
            #cmd_pt.velocities = np.array(vel)*0.1
            #cmd_pt.time_from_start = rospy.Duration.from_sec(t)
            cmd.points.append(cmd_pt)
            f = self.robot.GetForward(pt[:7])
            msg.poses.append(pm.toMsg(f * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2))))

        if len(cmd.points) > 0:
            cmd.points[-1].velocities = [0]*7

        return cmd,msg,traj,Z

    def SetTrajectory(self, traj_model):
        self.traj_model = traj_model
