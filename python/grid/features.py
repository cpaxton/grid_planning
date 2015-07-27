
# ROS stuff
import rospy
from urdf_parser_py.urdf import URDF

# KDL utilities
import PyKDL
from pykdl_utils.kdl_kinematics import KDLKinematics
from pykdl_utils.kdl_parser import kdl_tree_from_urdf_model

# tf stuff
import tf
import tf_conversions.posemath as pm

# Message types 
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
            robot_description_param=None):
        if not robot_description_param == None:
            pass
        else:

            self.world_frame = world_frame
        self.base_link = base_link
        self.end_link = end_link
        self.robot = URDF.from_parameter_server()
        self.tree = kdl_tree_from_urdf_model(robot)
        self.chain = tree.getChain(base_link, end_link)
        self.kdl_kin = KDLKinematics(robot, base_link, end_link)

        # create transform listener to get object information
        self.tf = tf.TransformListener()

        # empty list of objects
        self.objects = objects
        self.world = {}

        self.last_gripper_msg = rospy.Time(0)
        self.gripper_t_threshold = 0.1

        self.times = []
        self.joint_states = []
        self.gripper_cmds = []
        self.world_states = []

        self.js_topic = js_topic
        self.gripper_topic = gripper_topic

    def StartRecording(self):
        self.js_sub = rospy.Subscriber(self.js_topic,sensor_msgs.msg.JointState)
        self.gripper_sub = rospy.Subscriber(self.js_topic,oro_barrett_msgs.msg.BHandCmd)

    def save(self,filename):
        grid.SaveYaml(filename,self)

    def js_cb(self,msg):
        if self.TfUpdateWorld() and (rospy.Time.now() - self.last_gripper_msg).to_sec() < self.gripper_t_threshold:
            # record joints
            self.times.append(rospy.Time.now())
            self.joint_states.append(msg)
            self.gripper_cmds.append(self.gripper_cmd)
            self.world_states.append(self.world)

    def gripper_cb(self,msg):
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
                (trans,rot) = self.tf.lookupTransform(self.world_frame,frame,rospy.Time(0))
                self.world[obj] = pm.fromTf((trans,rot))

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException), e:
                print e
                return False
        return True

    '''
    GetFeatures
    Takes a joint-space trajectory (with times) and produces an output vector of (expected) features based on known object positions
    '''
    def GetFeatures(self,traj,objs=None):
        for pt in traj:
            q = pt[:7]
            gripper_cmd = pt[7:]

            mat = self.kdl_kin.forward(q)
            f = pm.fromMatrix(mat)



r = RobotFeatures()
r.AddObject("platform","/platform_link")
r.TfUpdateWorld()
r.GetForward([0,0,0,0,0,0,0])
