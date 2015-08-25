import rospy
import grid
import oro_barrett_msgs
from oro_barrett_msgs.msg import BHandCmd
import std_msgs
import copy
import sensor_msgs
import numpy as np
import pypr.clustering.gmm as gmm
import thread

SKILL_TOPIC = "current_skill"

class GripperRegressor:

    '''
    reads in a std_msgs/string object
    '''
    def skill_cb(self,msg):

        self.active_skill = msg.data
        self.skill_is_active = True

        if self.active_skill in self.skills:
            skill = self.skills[self.active_skill]
            self.robot.indices = {}
            [self.robot.AddObject(obj) for obj in skill.objs]
            self.means = [i for i in skill.gripper_model.means_]
            self.covars = [i for i in skill.gripper_model.covars_]
            self.weights = skill.gripper_model.weights_

        self.ndims = self.robot.max_index

    '''
    keeps progrss up to date
    '''
    def progress_cb(self,msg):
        self.progress = msg.data
        self.tick()

    '''
    keep joints up to date
    '''
    def js_cb(self,msg):
        self.js = msg

    '''
    sets up the node based on a GMM and the gripper topic
    '''
    def __init__(self,features,cmd_topic,skill_topic,progress_topic):
        #self.gmm = gmm
        #self.features = features
        self.gmms = {}
        self.skills = {}
        self.active_skill = None
        self.means = []
        self.covars = []
        self.weights = []
        self.objs = {}
        self.js = None
        self.ndims = 0

        self.progress = 0
        self.skill_is_active = False
        self.skill_sub = rospy.Subscriber(skill_topic,std_msgs.msg.String,self.skill_cb)
        self.progress_sub = rospy.Subscriber(progress_topic,std_msgs.msg.Float64,self.progress_cb)
        self.js_sub = rospy.Subscriber(features.js_topic,sensor_msgs.msg.JointState,self.js_cb)
        self.cmd_pub = rospy.Publisher(cmd_topic,oro_barrett_msgs.msg.BHandCmd)
        self.robot = grid.RobotFeatures(
                base_link=features.base_link,
                end_link=features.end_link,
                world_frame=features.world_frame,
                gripper_topic=features.gripper_topic,
                objects=features.objects,
                indices=features.indices,
                robot_description_param=features.robot_description_param,
                dof=features.dof
                )
        self.configured = False

    '''
    tick()
    update robot and world state appropriately
    compute features
    publish a bhandcmd message
    '''
    def tick(self):

        if not self.configured:
            rospy.logerr('Regressor world not configured!')
            return

        world = None
        print self.robot.objects
        while world is None:
            world = self.robot.TfCreateWorld()

        if self.js is None:
            rospy.logerr('Regressor missing information!')
            return

        if self.skill_is_active and self.active_skill in self.gmms:

            # get features for current time step
            objs = self.skills[self.active_skill].objs

            f = self.robot.GetFeatures(self.js.position,self.progress,world,objs)

            # create new data with gripper indices missing
            data = np.zeros((1,self.ndims)) * np.nan
            for obj,idx in self.robot.indices.items():
                if obj == "gripper":
                    continue

                f = self.robot.GetFeatures(self.js.position,self.progress,world,[obj])
                #print (np.r_[idx[0]:idx[1]], self.ndims, f)
                #print (data[0,np.ix_(np.r_[idx[0]:idx[1]])],f)
                data[0,np.ix_(np.r_[idx[0]:idx[1]])] = f


            # use pypr to fill in missing data
            gmm.predict(data,self.means,self.covars,self.weights)

            # create message to send
            msg = BHandCmd()
            idx = self.robot.indices['gripper']
            msg.cmd = [0,0,0,0]
            #print msg.cmd[0:(idx[1]-idx[0])]
            msg.cmd[0:(idx[1]-idx[0])] = data[0,np.ix_(np.r_[idx[0]:idx[1]])].tolist()[0]
            msg.mode = [4]*4
            self.cmd_pub.publish(msg)

            # clean up
            if self.progress >= 1.0:
                print "Done with skill!"
                self.skill_is_active = False
    '''
    configure()
    sets up the objects associated with this regressor
    '''
    def configure(self,objs):
        for obj,frame in objs:
            self.robot.AddObject(obj,frame)
        self.configured = True

        print self.robot.objects

    '''
    start a loop
    '''
    def start(self):
        try:
            thread.start_new_thread(self.loop,())
        except Exception, ex:
            print ex

    def loop(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            try:
                self.tick()
            except Exception, ex:
                print ex
            rate.sleep()

    def addSkill(self,skill):

        for obj in skill.objs:
            self.robot.AddObject(obj)

        self.skills[skill.name] = skill
        self.gmms[skill.name] = skill.gripper_model

