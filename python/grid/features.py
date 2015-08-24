
# ROS stuff
import rospy
from urdf_parser_py.urdf import URDF
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper
import copy

import numpy as np

# KDL utilities
import PyKDL
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

# tf stuff
import tf
import tf_conversions.posemath as pm

# input message types 
import sensor_msgs
import oro_barrett_msgs
import trajectory_msgs
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from oro_barrett_msgs.msg import BHandCmd

# output message types
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseArray

TIME = 'time'
GRIPPER = 'gripper'
JOINT = 'joint' # features indicating total joint velocity/effort
NUM_OBJ_VARS = 8
NUM_GRIPPER_VARS = 3
NUM_TIME_VARS = 1

class RobotFeatures:

    '''
    create a robot
    loads robot description and kinematics from parameter server if available
    configured as a kinematic chain; uses KDLKinematics for robot forward kinematics
    '''
    def __init__(self,
            base_link='wam/base_link',
            end_link='wam/wrist_palm_link',
            world_frame='/world',
            js_topic='/gazebo/barrett_manager/wam/joint_states',
            gripper_topic='/gazebo/barrett_manager/hand/cmd',
            objects={}, indices={},
            robot_description_param='robot_description',
            dof=7,
            filename=None
            ):

        self.dof = dof;
        self.world_frame = world_frame
        self.base_link = base_link
        self.end_link = end_link
        self.robot_description_param=robot_description_param

        if not robot_description_param == 'robot_description':
            pass
        else:
            self.robot = URDF.from_parameter_server()
            self.tree = kdl_tree_from_urdf_model(self.robot)
            self.chain = self.tree.getChain(base_link, end_link)
            self.kdl_kin = KDLKinematics(self.robot, base_link, end_link)

        # create transform listener to get object information
        self.tfl = tf.TransformListener()

        # empty list of objects
        self.objects = objects
        self.world = {}

        self.last_gripper_msg = rospy.Time(0)
        self.gripper_t_threshold = 0.05

        self.times = []
        self.joint_states = []
        self.gripper_cmds = []
        self.world_states = []

        self.indices = indices
        self.max_index = 0

        self.feature_model = None
        self.sub_model = None

        self.js_topic = js_topic
        self.gripper_topic = gripper_topic

        self.recorded = False
        self.quiet = True # by default hide TF error messages

        if not filename == None:
            stream = file(filename,'r')
            data = yaml.load(stream,Loader=Loader)
            self.joint_states = data['joint_states']
            self.world_states = data['world_states']
            self.times = data['times']
            self.base_tform = data['base_tform']

            if data.has_key('indices'):
                self.indices = data['indices']
                self.max_index = data['max_index']
            else: # initialize the indices
                for obj in self.world_states[0].keys():
                    self.AddObject(obj)

            self.recorded = True
            self.gripper_cmds = data['gripper_cmds']

    def StartRecording(self):
        if self.recorded:
            return

        self.recorded = True
        self.js_sub = rospy.Subscriber(self.js_topic,sensor_msgs.msg.JointState,self.js_cb)
        self.gripper_sub = rospy.Subscriber(self.gripper_topic,oro_barrett_msgs.msg.BHandCmd,self.gripper_cb)

    def save(self,filename):

        if self.recorded:
            stream = file(filename,'w')

            data = {}
            data['times'] = self.times
            data['world_frame'] = self.world_frame
            data['gripper_cmds'] = self.gripper_cmds
            data['joint_states'] = self.joint_states
            data['world_states'] = self.world_states
            data['base_link'] = self.base_link
            data['end_link'] = self.end_link
            data['robot_description_param'] = self.robot_description_param
            data['base_tform'] = self.base_tform
            data['objects'] = self.objects
            data['indices'] = self.indices
            data['max_index'] = self.max_index

            yaml.dump(data,stream)

        else:
            rospy.logerr("Could not save; no recording done!")

    def js_cb(self,msg):

        if self.TfUpdateWorld() and (rospy.Time.now() - self.last_gripper_msg).to_sec() < self.gripper_t_threshold:
            # record joints
            self.times.append(rospy.Time.now())
            self.joint_states.append(msg)
            self.gripper_cmds.append(self.gripper_cmd)
            self.world_states.append(copy.deepcopy(self.world))

    def gripper_cb(self,msg):

        self.gripper_cmd = msg
        self.last_gripper_msg = rospy.Time.now()

    '''
    Add an object we can use as a reference
    for now number of gripper, object variables are all hard coded
    '''
    def AddObject(self,obj,frame=""):
        if obj == TIME:
            nvars = NUM_TIME_VARS
        elif obj == GRIPPER:
            nvars = NUM_GRIPPER_VARS
        else:
            nvars = NUM_OBJ_VARS
            self.objects[obj] = frame

        self.indices[obj] = (self.max_index,self.max_index+nvars)
        self.max_index += nvars

    '''
    GetForward
    Returns the position of the gripper from a given set of joint positions
    Also gets relative positions to objects at different frames of reference
    '''
    def GetForward(self,q):

        mat = self.kdl_kin.forward(q)
        f = pm.fromMatrix(mat)

        return f

    '''
    SetWorld
    Sets locations of different objects at the beginning of the action.
    '''
    def SetWorld(self,frames):
        for obj in self.objects.keys():
            self.world[obj] = frames[obj]

    '''
    TfUpdateWorld
    '''
    def TfUpdateWorld(self):
        for (obj,frame) in self.objects.items():
            try:
                (trans,rot) = self.tfl.lookupTransform(self.world_frame,frame,rospy.Time(0))
                self.world[obj] = pm.fromTf((trans,rot))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                if not self.quiet:
                    print "ERR: %s"%(e)
                return False

        try:
            (trans,rot) = self.tfl.lookupTransform(self.world_frame,self.base_link,rospy.Time(0))
            self.base_tform = pm.fromTf((trans,rot))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            if not self.quiet:
                print "ERR: %s"%(e)
            return False


        return True

    '''
    Create a world and return it; just calls TfUpdateWorld() to do this (the updated world is the local copy)
    '''
    def TfCreateWorld(self):
        self.TfUpdateWorld()
        return self.world

    '''
    Get an actual trajectory: the things we are trying to learn how to reproduce
    '''
    def GetTrajectory(self):
        traj = []
        for i in range(len(self.times)):
            pt = [j for j in self.joint_states[i].position[:self.dof]] + [k for k in self.gripper_cmds[i].cmd[:NUM_GRIPPER_VARS]]
            traj.append(pt)
        return traj

    def GetWorldPoseMsg(self,frame):

        msg = PoseArray()
        msg.header.frame_id = self.world_frame

        for i in range(len(self.world_states)): 
            pmsg = pm.toMsg(self.world_states[i][frame] * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2)))
            msg.poses.append(pmsg)

        return msg

    '''
    GetFwdPoseMsg
    Create a pose array from forward kinematics
    '''
    def GetFwdPoseMsg(self):

        msg = PoseArray()
        msg.header.frame_id = self.world_frame

        for i in range(len(self.world_states)): 
            mat = self.kdl_kin.forward(self.joint_states[i].position[:7])
            ee_frame = self.base_tform * pm.fromMatrix(mat)
            pmsg = pm.toMsg(ee_frame * PyKDL.Frame(PyKDL.Rotation.RotY(-1*np.pi/2)))
            msg.poses.append(pmsg)

        return msg

    '''
    GetTrajectoryWeight()
    - z is the trajectory params
    - Z is the trajectory distribution
    - p_obs is the probability of these feature observations (fixed at one)
    '''
    def GetTrajectoryWeight(self,traj,world,objs,p_z,p_obs=1,t_lambda=0.5):

        weights = [0.0]*len(traj)

        features,goal_features = self.GetFeaturesForTrajectory(traj,world,objs)

        N = len(features)
        scores,_ = self.traj_model.score_samples(features)
        denom = 1 / (p_obs * p_z )

        #print scores

        for i in range(N):
            weights[i] = np.exp(scores[i]) * t_lambda**(N-i) * denom

        # lambda**(N-i) [where i=N] == lambda**0 == 1
        weights[-1] = np.exp(self.goal_model.score_samples(goal_features)[0]) * denom


        return weights

    '''
    GetTrajectoryLikelihood
    slow computation of trajectory likelihood...
    Computes the same features as before
    Will then score them as per usual
    '''
    def GetTrajectoryLikelihood(self,traj,world,objs,step=1.,sigma=0.000):

        features,goal_features = self.GetFeaturesForTrajectory(traj,world,objs)
        isum = np.sum(range(len(features)))

        avg = 0
        for i in range(len(features)):
            avg += (float(i) / float(isum)) * self.traj_model.score(features[i])

        return self.goal_model.score(goal_features) + avg

    '''
    GetFeatures
    '''
    def GetFeaturesForTrajectory(self,traj,world,objs):

        features = [[]]*(len(traj)-1)

        i = 0
        for i in range(len(traj)-1):
            t = float(i) / len(traj)

            #features[i] = self.GetFeatures(traj[i],t,world,objs)
            #diffs[i] = self.GetDiffFeatures(traj[i-1][:self.dof],traj[i][:self.dof])
            features[i] = self.GetFeatures(traj[i],t,world,objs) + self.GetDiffFeatures(traj[i-1][:self.dof],traj[i][:self.dof])

        goal_features = self.GetFeatures(traj[-1],1,world,objs)

        return features,goal_features

    '''
    GetFeatures
    Gets the features for a particular combination of world, time, and point.
    '''
    def GetFeatures(self,pt,t,world,objs):

        features = []

        # compute forward transform
        q = pt[:self.dof]
        ee_frame = self.GetForward(q)

        for obj in objs:

            if obj == TIME:
                features += [t]
            elif obj == GRIPPER:
                features += pt[self.dof:]
            else:

                # we care about this world object...
                obj_frame = world[obj]

                # ... so get object offset to end effector ...
                offset = obj_frame.Inverse() * (self.base_tform * ee_frame)

                # ... use position offset and distance ...
                features += offset.p
                features += [offset.p.Norm()]

                # ... and use axis/angle representation
                (theta,w) = offset.M.GetRotAngle()
                #features += [theta] + [ww for ww in w]
                #features += [theta*ww for ww in w] # rotation vector
                rv = PyKDL.Vector(theta*w[0],theta*w[1],theta*w[2])
                features += list(rv) + [rv.Norm()]

        return features

    '''
    GetTrainingFeatures
    Takes a joint-space trajectory (with times) and produces an output vector of (expected) features based on known object positions
    '''
    def GetTrainingFeatures(self,objs=None):
        
        if objs == None:
            objs = self.indices.keys()
        
        ftraj = [] # feature-space trajectory
        traj = self.GetTrajectory()

        start_t = self.times[0].to_sec()
        end_t = self.times[-1].to_sec()
        for i in range(1,len(traj)):

            features = []
            diff = []

            # compute features for difference between current and next end effector frame
            #f0 = self.GetForward(traj[i-1][:7])
            #f1 = self.GetForward(traj[i][:7])
            #df = f1.Inverse() * f0
            #diff = [df.p.Norm(), df.M.GetRotAngle()[0]]
            diff = self.GetDiffFeatures(traj[i-1][:self.dof],traj[i][:self.dof])

            # loop over objects/world at this time step
            t = (self.times[i-1].to_sec() - start_t) / (end_t - start_t)
            ftraj += [self.GetFeatures(traj[i-1],t,self.world_states[i-1],objs) + diff]
        
        goal = self.GetFeatures(traj[-1],1.0,self.world_states[-1],objs)

        return ftraj,goal

    '''
    Compute velocity features
    How much did the end effector move over the course of the trajectory?
    '''
    def GetVelocityFeatures(self):
        ftraj = []

        for i in range(2,len(traj)):

            ee = GetForward(self.joint_states[i].positions)
            ee0 = GetForward(self.joint_states[i-1].positions)

            dee = ee0.Inverse() * ee

            ftraj += [dee.p + dee.M.GetRPY() + dee.p.Norm()]

    '''
    GetFeatureLabels()
    Gets a set of string labels, one for each feature.
    This is to make debugging/visualizing results a little bit easier.
    '''
    def GetFeatureLabels(self,objs=None):
        
        if objs==None:
            objs = self.indices.keys()

        labels = []

        for obj in objs: #self.world_states[0].keys():

            if obj == TIME:
                labels += ["time"]
            elif obj == GRIPPER:
                labels += ["gripper1", "gripper2", "gripper3"]
            else:
                labels += ["%s_ee_x"%obj]
                labels += ["%s_ee_y"%obj]
                labels += ["%s_ee_z"%obj]
                labels += ["%s_ee_dist"%obj]
                labels += ["%s_ee_theta"%obj]
                labels += ["%s_ee_wx"%obj]
                labels += ["%s_ee_wy"%obj]
                labels += ["%s_ee_wz"%obj]

        return labels

    '''
    Get the diff features we are using
    These are the translation, distance, and axis-angle rotation between frames
    '''
    def GetDiffFeatures(self,q0,q1):
            f0 = self.GetForward(q0)
            f1 = self.GetForward(q1)
            df = f1.Inverse() * f0
            theta,w = df.M.GetRotAngle()
            rv = PyKDL.Vector(theta*w[0], theta*w[1], theta*w[2])
            #diff = [x for x in df.p] + [df.p.Norm(), theta] + [ww for ww in w]
            #diff = [x for x in df.p] + [df.p.Norm()] + list(rv) + [rv.Norm()]
            diff = [df.p.Norm()] + [rv.Norm()]
            return diff
    '''
    GetJointPositions()
    Just get the positions of each joint
    '''
    def GetJointPositions(self):
        return [pt.position for pt in self.joint_states]

    def Dims(self):
        return len(self.joint_states[0].position) + 3

def LoadRobotFeatures(filename):

    stream = file(filename,'r')
    data = yaml.load(stream,Loader=Loader)

    r = RobotFeatures(base_link=data['base_link'],
            end_link=data['end_link']
            ,world_frame=data['world_frame'],
            robot_description_param=data['robot_description_param'])

    r.gripper_cmds = data['gripper_cmds']
    r.joint_states = data['joint_states']
    r.world_states = data['world_states']
    r.times = data['times']
    r.base_tform = data['base_tform']

    if data.has_key('indices'):
        r.indices = data['indices']
        r.max_index = data['max_index']
    else: # initialize the indices
        for obj in r.world_states[0].keys():
            r.AddObject(obj)

    r.recorded = True

    return r

