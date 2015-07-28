
# ROS stuff
import rospy
from urdf_parser_py.urdf import URDF
import yaml
try:
    from yaml import CLoader as Loader, CDumper as Dumper
except ImportError:
    from yaml import Loader, Dumper


# KDL utilities
import PyKDL
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

# tf stuff
import tf
import tf_conversions.posemath as pm

# Message types 
import sensor_msgs
import oro_barrett_msgs
import trajectory_msgs
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState
from oro_barrett_msgs.msg import BHandCmd

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
            objects={},
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

            yaml.dump(data,stream)

        else:
            rospy.logerr("Could not save; no recording done!")

    def js_cb(self,msg):
        #print (rospy.Time.now() - self.last_gripper_msg).to_sec() < self.gripper_t_threshold
        if self.TfUpdateWorld() and (rospy.Time.now() - self.last_gripper_msg).to_sec() < self.gripper_t_threshold:
            # record joints
            self.times.append(rospy.Time.now())
            self.joint_states.append(msg)
            self.gripper_cmds.append(self.gripper_cmd)
            self.world_states.append(self.world)

    def gripper_cb(self,msg):
        #print msg
        self.gripper_cmd = msg # [i for i in msg.cmd]
        self.last_gripper_msg = rospy.Time.now()

    '''
    Add an object we can use as a reference
    '''
    def AddObject(self,obj,frame):
        self.objects[obj] = frame

    '''
    GetForward
    Returns the position of the gripper from a given set of joint positions
    Also gets relative positions to objects at different frames of reference
    '''
    def GetForward(self,q,objs=None):
        mat = self.kdl_kin.forward(q)
        f = pm.fromMatrix(mat)

        frames = []
        if objs==None:
            for frame in self.world.values():
                frames.append(frame.Inverse() * f)
        else:
            for obj in objs:
                frames.append(world[obj].Inverse() * f)

        return (f, frames)

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
        return True


    '''
    Get an actual trajectory: the things we are trying to learn how to reproduce
    '''
    def GetTrajectory(self):
        traj = []
        for i in range(len(self.times)):
            pt = [j for j in self.joint_states[i].position[:7]] + [k for k in self.gripper_cmds[i].cmd[:3]]
            traj.append(pt)
        return traj

    '''
    GetFeatures
    Takes a joint-space trajectory (with times) and produces an output vector of (expected) features based on known object positions
    '''
    def GetFeatures(self,traj=self.GetTrajectory(),objs=None):
        
        ftraj = []

        for pt in traj:

            features = []

            q = pt[:7]
            gripper_cmd = pt[7:]
            features += gripper_cmd # include gripper closure as a feature

            mat = self.kdl_kin.forward(q)
            f = pm.fromMatrix(mat)

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
