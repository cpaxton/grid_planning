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
        
        rospy.logwarn('GripperRegressor not set up to handle diff variables yet!')

        self.active_skill = msg.data
    
        if self.active_skill in self.skills:
            skill = self.skills[self.active_skill]
            self.robot.indices = {}
            [self.robot.AddObject(obj) for obj in skill.objs]
            [self.robot.AddObject(obj,frame) for obj,frame in self.config if obj in skill.objs]
            self.means = [i for i in skill.gripper_model.means_]
            self.covars = [i for i in skill.gripper_model.covars_]
            self.weights = skill.gripper_model.weights_
            self.objs = skill.objs
        self.robot.AddObject('gripper')

        self.ndims = self.robot.max_index

        print self.robot.objects
        while self.world is None:
            self.world = self.robot.TfCreateWorld()

        self.skill_is_active = True

    '''
    keeps progrss up to date
    '''
    def progress_cb(self,msg):
        self.progress = msg.data
        #self.tick()

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
        self.config = []
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
                objects=copy.deepcopy(features.objects),
                indices=copy.deepcopy(features.indices),
                diff_indices=copy.deepcopy(features.diff_indices),
                robot_description_param=features.robot_description_param,
                dof=features.dof
                )
        self.configured = False

        self.world = None

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

        if self.js is None:
            rospy.logerr('Regressor missing information!')
            return
        
        if self.skill_is_active and self.active_skill in self.gmms:

            # get features for current time step
            objs = self.skills[self.active_skill].objs

            f = self.robot.GetFeatures(self.js.position,self.progress,self.world,objs)

            # create new data with gripper indices missing
            data = np.zeros((1,self.ndims)) * np.nan
            for obj,idx in self.robot.diff_indices.items():
                if obj == "gripper":
                    continue

            obj_req = copy.copy(objs)
            if 'gripper' in obj_req:
                obj_req.remove('gripper')

            idx = self.robot.GetIndices(obj_req)

            #f = self.robot.GetFeatures(self.js.position,self.progress,self.world,[obj])
            #print (np.r_[idx[0]:idx[1]], self.ndims, f)
            #print (data[0,np.ix_(np.r_[idx[0]:idx[1]])],f)
            data[0,np.ix_(idx)] = f

            # use pypr to fill in missing data
            gmm.predict(data,self.means,self.covars,self.weights)

            # create message to send
            msg = BHandCmd()
            idx = self.robot.indices['gripper']
            #idx = self.robot.GetIndices(['gripper'])
            msg.cmd = [0,0,0,0]
            #print msg.cmd[0:(idx[1]-idx[0])]
            msg.cmd[0:(idx[1]-idx[0])] = data[0,np.ix_(np.r_[idx[0]:idx[1]])].tolist()[0]
            #msg.cmd[0:(max(idx)+1-min(idx))] = data[0,np.ix(idx)].tolist()[0]
            if any(np.isnan(msg.cmd)):
                print "ERR: Expected gripper command not defined!"
                msg.cmd = [0,0,0,0]
            msg.mode = [4]*4
            self.cmd_pub.publish(msg)

            print (msg.cmd, self.progress)

        # clean up and stop gripper
        if self.progress >= 1.0 and self.skill_is_active:
            print "Done with skill!"
            self.skill_is_active = False

            msg = BHandCmd()
            idx = self.robot.indices['gripper']
            msg.cmd = [0,0,0,0]
            msg.mode = [4]*4
            self.cmd_pub.publish(msg)
            return True

        return False
    '''
    configure()
    sets up the objects associated with this regressor
    '''
    def configure(self,objs):
        for obj,frame in objs:
            self.robot.AddObject(obj,frame)
        self.config = objs
        self.configured = True

    '''
    start a loop
    '''
    def start(self):
        #print "Getting world..."
        #while self.world is None:
        #    print self.robot.objects
        #    self.world = self.robot.TfCreateWorld()
        #print "...done."
        try:
            thread.start_new_thread(self.loop,())
        except Exception, ex:
            print ex

    def loop(self):
        rate = rospy.Rate(10)
        done = False
        while not rospy.is_shutdown():
            #try:
            done = self.tick()
            #except Exception, ex:
            #    print ex
            rate.sleep()

    def addSkill(self,skill):

        for obj in skill.objs:
            self.robot.AddObject(obj)

        self.skills[skill.name] = skill
        self.gmms[skill.name] = skill.gripper_model

