
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
            robot_description_param='robot_description'):

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
        self.max_index = 10

        self.feature_model = None
        self.sub_model = None
        self.idx = None

        self.js_topic = js_topic
        self.gripper_topic = gripper_topic

        self.recorded = False

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
    '''
    def AddObject(self,obj,frame):
        self.objects[obj] = frame
        if len(self.indices) == 0:
            self.indices[obj] = np.r_[3:10]
            self.max_index = 10
        else:
            self.indices[obj] = np.r_[self.max_index:self.max_index+7]
            self.max_index = self.max_index + 7

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
                print "ERR: %s"%(e)
                return False

        try:
            (trans,rot) = self.tfl.lookupTransform(self.world_frame,self.base_link,rospy.Time(0))
            self.base_tform = pm.fromTf((trans,rot))

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
            print "ERR: %s"%(e)
            return False


        return True

    '''
    Create a world; does not store the results or anything like that
    '''
    def TfCreateWorld(self):
        world = {}
        for (obj,frame) in self.objects.items():
            try:
                (trans,rot) = self.tfl.lookupTransform(self.world_frame,frame,rospy.Time(0))
                world[obj] = pm.fromTf((trans,rot))
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                print "ERR: %s"%(e)
                return None

        return world

    '''
    Get an actual trajectory: the things we are trying to learn how to reproduce
    '''
    def GetTrajectory(self):
        traj = []
        for i in range(len(self.times)):
            pt = [j for j in self.joint_states[i].position[:7]] + [k for k in self.gripper_cmds[i].cmd[:3]]
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

    def GetTrajectoryLikelihood(self,traj,world,idx,step=1.,sigma=0.000):

        #for i in range(len(traj)):
        #    t = float(i) / len(traj)
        #    f = self.GetFeatures(traj[i],t,world)
        f = self.GetFeatures(traj[-1],1,world,objs=world.keys())

        if self.idx == None or not self.idx == idx:
            self.idx = idx
            self.sub_model = copy.deepcopy(self.goal_model)
            if self.goal_model.covariance_type == "full":
                self.sub_model.means_ = self.goal_model.means_[:,self.idx[0]:self.idx[1]]
                self.sub_model.covars_ = step*self.goal_model.covars_[:,self.idx[0]:self.idx[1],self.idx[0]:self.idx[1]] + sigma*np.eye(self.idx[1]-self.idx[0])
            elif self.goal_model.covariance_type == "diag":
                self.sub_model.means_ = self.goal_model.means_[:,self.idx[0]:self.idx[1]]
                self.sub_model.covars_ = step*self.goal_model.covars_[:,self.idx[0]:self.idx[1]]

        return self.sub_model.score(f)

    def GetLikelihood(self,pt,t,world,idx,step=10,sigma=0.00001):
        if self.idx == None or not self.idx == idx:
            self.idx = idx
            self.sub_model = copy.deepcopy(self.feature_model)
            self.sub_model.means_ = self.feature_model.means_[:,self.idx]
            self.sub_model.covars_ = step*self.feature_model.covars_[:,self.idx] + sigma*np.eye(self.idx[1]-self.idx[0])
        
        x = self.GetFeatures(pt,t,world)

        return self.sub_model.score(x)


    '''
    GetFeatures
    Gets the features for a particular combination of world, time, and point.
    '''
    def GetFeatures(self,pt,t,world,objs=None):
        features = []

        q = pt[:7]
        gripper_cmd = pt[7:]

        features += gripper_cmd # include gripper closure as a feature

        # compute forward transform
        ee_frame = self.GetForward(q)

        for obj in objs:

            obj_frame = world[obj]

            #print (obj, obj_frame)
            offset = obj_frame.Inverse() * (self.base_tform * ee_frame)
            #offset = ((self.base_tform * ee_frame).Inverse() * obj_frame).Inverse()

            features += offset.p
            features += offset.M.GetRPY()
            features += [offset.p.Norm()]

        return features

    '''
    GetFeaturesDiff
    Compute forward kinematics
    Look up changes in frames over time
    '''
    def GetFeaturesDiff(self):
        ftraj = []

        print "ERR: not implemented yet!"

    '''
    GetFeatures
    '''
    def GetFeaturesForTrajectory(self,traj,world):
        ftraj = []

        print "ERR: not implemented yet!"

    '''
    GetTrainingFeatures
    Takes a joint-space trajectory (with times) and produces an output vector of (expected) features based on known object positions
    '''
    def GetTrainingFeatures(self,objs=None):
        
        ftraj = [] # feature-space trajectory
        traj = self.GetTrajectory()

        if objs == None or len(objs)==0:
            objs = self.world_states[0].keys()

        for i in range(1,len(traj)):

            features = []
            diff = []

            # compute features for difference between current and next end effector frame
            f0 = self.GetForward(traj[i-1][:7])
            f1 = self.GetForward(traj[i][:7])
            df = f1.Inverse() * f0
            diff = [df.p.Norm(), PyKDL.Vector(*df.M.GetRPY()).Norm()]

            # loop over objects/world at this time step
            ftraj += [self.GetFeatures(traj[i-1],self.times[i-1],self.world_states[i-1],objs) + diff]
        
        goal = self.GetFeatures(traj[-1],self.times[-1],self.world_states[-1],objs)

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
    def GetFeatureLabels(self):

        labels = ["gripper1", "gripper2", "gripper3"]

        for obj in self.world_states[0].keys():
            labels += ["%s_ee_x"%obj]
            labels += ["%s_ee_y"%obj]
            labels += ["%s_ee_z"%obj]
            labels += ["%s_ee_roll"%obj]
            labels += ["%s_ee_pitch"%obj]
            labels += ["%s_ee_yaw"%obj]
            labels += ["%s_ee_dist"%obj]
        return labels

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
    if data.has_key('max_index'):
        r.max_index = data['max_index']

    r.recorded = True

    return r

'''
import rospy
import grid
rospy.init_node('testing')
r = grid.RobotFeatures()
r.AddObject("platform","/platform_link")
r.TfUpdateWorld()
rospy.sleep(rospy.Duration(0.5))
r.TfUpdateWorld()
r.GetForward([0,0,0,0,0,0,0])
#r.StartRecording()

world='/world'
frame='/wam/hand/bhand_palm_link'
obj1='/gbeam_link_1/gbeam_link'
obj2='/gbeam_node_1/gbeam_node'
r.AddObject("link",obj1);
r.AddObject("node",obj2);

r.save("test.yml")
r2 = LoadRobotFeatures("test.yml")
'''
